/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * annotate_cv_stage.cpp - add text annotation to image
 */

// The text string can include the % directives supported by FrameInfo.

#include <time.h>
#include <vector>
#include <chrono>
#include <regex>
#include <optional>

#include <libcamera/stream.h>

#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;

using Stream = libcamera::Stream;

struct TextAnnotation {
	std::string text;
	int x;
	int y;
	int fg;
	std::optional<int> bg;  // Optional background color
	std::optional<int> border_color;  // Optional border color
	int border_width;  // Border width in pixels
	double scale;
	int thickness;
	double alpha;
	std::string template_format;
	std::chrono::system_clock::time_point last_update;
	std::string cached_text;
	Mat cached_image;  // Store the last drawn text region
	Rect text_region;  // Store the region where text was drawn
	bool is_static;    // Whether the text is static (no dynamic content)
};

class AnnotateCvStage : public PostProcessingStage
{
public:
	AnnotateCvStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	StreamInfo info_;
	std::vector<TextAnnotation> annotations_;
	double adjusted_scale_;
	int adjusted_thickness_;
	std::chrono::milliseconds update_interval_;

	void drawAnnotation(Mat &im, TextAnnotation &annotation, const FrameInfo &info);
	void updateDynamicText(TextAnnotation &annotation);
	std::string processTemplate(const std::string &template_str);
	void createTextCache(TextAnnotation &annotation, const std::string &text);
};

#define NAME "annotate_cv"

char const *AnnotateCvStage::Name() const
{
	return NAME;
}

void AnnotateCvStage::Read(boost::property_tree::ptree const &params)
{
	// Clear existing annotations
	annotations_.clear();

	// Read update interval from config, default to 1 second
	update_interval_ = std::chrono::milliseconds(
		params.get<int>("update_interval_ms", 1000));

	// Read each annotation from the params
	for (const auto &annotation_node : params)
	{
		if (annotation_node.first == "annotation")
		{
			TextAnnotation annotation;
			annotation.text = annotation_node.second.get<std::string>("text", "");
			annotation.x = annotation_node.second.get<int>("x", 0);
			annotation.y = annotation_node.second.get<int>("y", 0);
			annotation.fg = annotation_node.second.get<int>("fg", 255);
			
			// Make background optional
			if (annotation_node.second.count("bg"))
				annotation.bg = annotation_node.second.get<int>("bg");
			
			// Make border optional
			if (annotation_node.second.count("border_color"))
				annotation.border_color = annotation_node.second.get<int>("border_color");
			annotation.border_width = annotation_node.second.get<int>("border_width", 0);
			
			annotation.scale = annotation_node.second.get<double>("scale", 1.0);
			annotation.thickness = annotation_node.second.get<int>("thickness", 2);
			annotation.alpha = annotation_node.second.get<double>("alpha", 0.5);
			annotation.template_format = annotation_node.second.get<std::string>("template", "");
			annotation.last_update = std::chrono::system_clock::now();
			annotation.cached_text = "";
			annotation.is_static = annotation.template_format.empty();

			annotations_.push_back(annotation);
		}
	}
}

void AnnotateCvStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("AnnotateCvStage: only YUV420 format supported");
	info_ = app_->GetStreamInfo(stream_);

	// Adjust the scale and thickness according to the image size
	adjusted_scale_ = info_.width / 1200.0;
	adjusted_thickness_ = std::max(static_cast<int>(info_.width / 700), 1);

	// Create initial cache for static texts
	for (auto &annotation : annotations_)
	{
		if (annotation.is_static)
		{
			// For static text, we can use the text directly without FrameInfo
			createTextCache(annotation, annotation.text);
		}
	}
}

void AnnotateCvStage::createTextCache(TextAnnotation &annotation, const std::string &text)
{
	int font = FONT_HERSHEY_SIMPLEX;
	int baseline = 0;
	double scale = annotation.scale * adjusted_scale_;
	int thickness = std::max(static_cast<int>(annotation.thickness * adjusted_thickness_), 1);
	int border_width = annotation.border_width;

	// Calculate text size including border
	Size size = getTextSize(text, font, scale, thickness, &baseline);
	int border_padding = border_width * 2;  // Border on both sides

	// Calculate text region including border
	annotation.text_region = Rect(
		annotation.x - border_width,
		annotation.y - border_width,
		size.width + border_padding,
		size.height + baseline + border_padding
	);

	// Create a new cached image for this text
	annotation.cached_image = Mat::zeros(annotation.text_region.height, annotation.text_region.width, CV_8U);

	// Draw background rectangle with alpha only if bg is specified
	if (annotation.bg)
	{
		annotation.cached_image.setTo(*annotation.bg);
	}

	// Draw border if specified
	if (annotation.border_color && border_width > 0)
	{
		// Create a temporary image for the border
		Mat border_image = Mat::zeros(annotation.text_region.height, annotation.text_region.width, CV_8U);
		
		// Draw text in border color
		putText(border_image, text, 
				Point(border_width, size.height + border_width),
				font, scale, *annotation.border_color, thickness, 0);

		// Create a kernel for dilation
		Mat kernel = getStructuringElement(MORPH_RECT, 
			Size(2 * border_width + 1, 2 * border_width + 1));

		// Dilate the text to create the border
		dilate(border_image, border_image, kernel);

		// Add the border to the cached image
		add(annotation.cached_image, border_image, annotation.cached_image);
	}

	// Draw main text
	putText(annotation.cached_image, text, 
			Point(border_width, size.height + border_width),
			font, scale, annotation.fg, thickness, 0);
}

std::string AnnotateCvStage::processTemplate(const std::string &template_str)
{
	std::string result = template_str;
	std::regex expr_pattern("%\\{([^}]+)\\}");
	std::smatch matches;

	while (std::regex_search(result, matches, expr_pattern))
	{
		std::string expr = matches[1].str();
		std::string replacement;

		if (expr.find("pts:") == 0 || expr.find("localtime:") == 0 || expr.find("gmtime:") == 0)
		{
			// Parse the time expression: pts:gmtime:timestamp:format
			std::regex time_pattern("(pts|localtime|gmtime):([^:]+):(.+)");
			std::smatch time_matches;
			
			if (std::regex_match(expr, time_matches, time_pattern))
			{
				std::string time_type = time_matches[1].str();
				std::string timestamp_str = time_matches[2].str();
				std::string format = time_matches[3].str();
				
				// Replace escaped colons in format
				std::regex colon_pattern("\\\\:");
				format = std::regex_replace(format, colon_pattern, ":");

				time_t timestamp;
				if (time_type == "pts")
				{
					// Use the current frame's timestamp
					auto now = std::chrono::system_clock::now();
					timestamp = std::chrono::system_clock::to_time_t(now);
				}
				else
				{
					try {
						timestamp = std::stoll(timestamp_str);
					} catch (...) {
						timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
					}
				}

				tm *tm_ptr = (time_type == "gmtime") ? gmtime(&timestamp) : localtime(&timestamp);
				char buffer[256];
				strftime(buffer, sizeof(buffer), format.c_str(), tm_ptr);
				replacement = buffer;
			}
		}
		else if (expr == "pts")
		{
			// Return current timestamp in seconds
			auto now = std::chrono::system_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
			replacement = std::to_string(duration.count());
		}
		else if (expr == "eof")
		{
			// Return 0 for EOF (not applicable in our case)
			replacement = "0";
		}
		else if (expr == "n")
		{
			// Return frame number
			replacement = "0";  // We don't have access to frame number in this context
		}
		else if (expr == "t")
		{
			// Return current timestamp in seconds
			auto now = std::chrono::system_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
			replacement = std::to_string(duration.count());
		}
		else if (expr == "metadata")
		{
			// Return empty string for metadata (not implemented)
			replacement = "";
		}
		else
		{
			// Unknown expression, keep it as is
			replacement = "%{" + expr + "}";
		}

		result.replace(matches.position(), matches.length(), replacement);
	}

	return result;
}

void AnnotateCvStage::updateDynamicText(TextAnnotation &annotation)
{
	auto now = std::chrono::system_clock::now();
	if (!annotation.is_static && 
		(now - annotation.last_update) >= update_interval_)
	{
		annotation.cached_text = processTemplate(annotation.template_format);
		createTextCache(annotation, annotation.cached_text);
		annotation.last_update = now;
	}
}

void AnnotateCvStage::drawAnnotation(Mat &im, TextAnnotation &annotation, const FrameInfo &info)
{
	// Update dynamic text if needed
	if (!annotation.is_static)
	{
		updateDynamicText(annotation);
		// For dynamic text, we need to process the template with FrameInfo
		std::string text = info.ToString(annotation.cached_text);
		createTextCache(annotation, text);
	}

	// Copy the cached image to the output frame
	if (!annotation.cached_image.empty())
	{
		// Ensure the region is within image bounds
		Rect roi = annotation.text_region & Rect(0, 0, im.cols, im.rows);
		if (roi.area() > 0)
		{
			Mat src_roi = annotation.cached_image(Rect(0, 0, roi.width, roi.height));
			Mat dst_roi = im(roi);
			
			if (annotation.bg)
			{
				// Blend with alpha
				addWeighted(src_roi, annotation.alpha, dst_roi, 1 - annotation.alpha, 0, dst_roi);
			}
			else
			{
				// Just copy the text
				src_roi.copyTo(dst_roi);
			}
		}
	}
}

bool AnnotateCvStage::Process(CompletedRequestPtr &completed_request)
{
	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	FrameInfo info(completed_request);

	uint8_t *ptr = (uint8_t *)buffer.data();
	Mat im(info_.height, info_.width, CV_8U, ptr, info_.stride);

	// Draw each annotation
	for (auto &annotation : annotations_)
	{
		drawAnnotation(im, annotation, info);
	}

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new AnnotateCvStage(app);
}

static RegisterStage reg(NAME, &Create);
