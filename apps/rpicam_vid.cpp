/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_vid.cpp - libcamera video record app.
 */

#include <chrono>
#include <libyuv.h>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include "core/drawtext_renderer.hpp"
#include "core/rpicam_encoder.hpp"
#include "output/output.hpp"

using namespace std::placeholders;

// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
}

static int get_key_or_signal(VideoOptions const *options, pollfd p[1])
{
	int key = 0;
	if (signal_received == SIGINT)
		return 'x';
	if (options->keypress)
	{
		poll(p, 1, 0);
		if (p[0].revents & POLLIN)
		{
			char *user_string = nullptr;
			size_t len;
			[[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
			key = user_string[0];
		}
	}
	if (options->signal)
	{
		if (signal_received == SIGUSR1)
			key = '\n';
		else if ((signal_received == SIGUSR2) || (signal_received == SIGPIPE))
			key = 'x';
		signal_received = 0;
	}
	return key;
}

static int get_colourspace_flags(std::string const &codec)
{
	if (codec == "mjpeg" || codec == "yuv420")
		return RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE;
	else
		return RPiCamEncoder::FLAG_VIDEO_NONE;
}

static void apply_overlay_to_frame(const VideoOptions *options, RPiCamEncoder &app, CompletedRequestPtr &completed_request)
{
	if (options->drawtext_elements.empty())
		return;

	libcamera::Stream *stream = app.VideoStream();
	libcamera::FrameBuffer *buffer = completed_request->buffers[stream];

	// Use the public mapped buffer access
	const std::vector<libcamera::Span<uint8_t>> &mem = completed_request->mapped_buffers[stream];
	if (mem.size() < 3)
		return;

	// Get image dimensions from stream config
	libcamera::Size size = stream->configuration().size;
	int width = size.width;
	int height = size.height;
	int stride = width * 4;

	std::vector<uint8_t> rgb_frame(stride * height);

	// Convert YUV to ARGB
	libyuv::I420ToARGB(
		mem[0].data(), width,
		mem[1].data(), width / 2,
		mem[2].data(), width / 2,
		rgb_frame.data(), stride,
		width, height
	);

	// Render overlay text into ARGB buffer
	render_drawtext_elements(rgb_frame.data(), width, height, stride, options->drawtext_elements);

	// Convert ARGB back to YUV
	std::vector<uint8_t> yuv_frame(mem[0].size() + mem[1].size() + mem[2].size());

	libyuv::ARGBToI420(
		rgb_frame.data(), stride,
		yuv_frame.data(), width,
		yuv_frame.data() + width * height, width / 2,
		yuv_frame.data() + width * height + (width / 2) * (height / 2), width / 2,
		width, height
	);

	// Copy back into the mapped spans
	std::memcpy(mem[0].data(), yuv_frame.data(), mem[0].size());
	std::memcpy(mem[1].data(), yuv_frame.data() + mem[0].size(), mem[1].size());
	std::memcpy(mem[2].data(), yuv_frame.data() + mem[0].size() + mem[1].size(), mem[2].size());
}

// The main even loop for the application.

static void event_loop(RPiCamEncoder &app)
{
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();
	app.ConfigureVideo(get_colourspace_flags(options->codec));
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	// Monitoring for keypresses and signals.
	signal(SIGUSR1, default_signal_handler);
	signal(SIGUSR2, default_signal_handler);
	signal(SIGINT, default_signal_handler);
	// SIGPIPE gets raised when trying to write to an already closed socket. This can happen, when
	// you're using TCP to stream to VLC and the user presses the stop button in VLC. Catching the
	// signal to be able to react on it, otherwise the app terminates.
	signal(SIGPIPE, default_signal_handler);
	pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

	for (unsigned int count = 0; ; count++)
	{
		RPiCamEncoder::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamEncoder::MsgType::Quit)
			return;
		else if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		int key = get_key_or_signal(options, p);
		if (key == '\n')
			output->Signal();

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = !options->frames && options->timeout &&
					   ((now - start_time) > options->timeout.value);
		bool frameout = options->frames && count >= options->frames;
		if (timeout || frameout || key == 'x' || key == 'X')
		{
			if (timeout)
				LOG(1, "Halting: reached timeout of " << options->timeout.get<std::chrono::milliseconds>()
													  << " milliseconds.");
			app.StopCamera(); // stop complains if encoder very slow to close
			app.StopEncoder();
			return;
		}
		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		apply_overlay_to_frame(options, app, completed_request);
		if (!app.EncodeBuffer(completed_request, app.VideoStream()))
		{
			// Keep advancing our "start time" if we're still waiting to start recording (e.g.
			// waiting for synchronisation with another camera).
			start_time = now;
			count = 0; // reset the "frames encoded" counter too
		}
		app.ShowPreview(completed_request, app.VideoStream());
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamEncoder app;
		VideoOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
