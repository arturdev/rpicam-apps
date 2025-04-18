/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * annotate_cv_stage.cpp - add text annotation to image (grayscale optimized)
 */

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
    std::optional<int> bg;
    std::optional<int> border_color;
    int border_width;
    double scale;
    int thickness;
    double alpha;
    std::string template_format;
    std::chrono::system_clock::time_point last_update;
    std::string cached_text;
    Mat cached_image;
    Rect text_region;
    bool is_static;
};

class AnnotateCvStage : public PostProcessingStage {
public:
    AnnotateCvStage(RPiCamApp *app) : PostProcessingStage(app) {}

    char const *Name() const override { return "annotate_cv"; }
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

void AnnotateCvStage::Read(boost::property_tree::ptree const &params) {
    auto config = params.get_child("annotate_cv");
    annotations_.clear();
    update_interval_ = std::chrono::milliseconds(config.get<int>("update_interval_ms", 1000));

    for (const auto &annotation_node : config.get_child("texts")) {
        const auto &value = annotation_node.second;
        TextAnnotation annotation;
        annotation.text = value.get<std::string>("text", "");
        annotation.x = value.get<int>("x", 0);
        annotation.y = value.get<int>("y", 0);
        annotation.fg = value.get<int>("fg", 255);

        if (value.count("bg"))
            annotation.bg = value.get<int>("bg");

        if (value.count("border_color"))
            annotation.border_color = value.get<int>("border_color");

        annotation.border_width = value.get<int>("border_width", 0);
        annotation.scale = value.get<double>("scale", 1.0);
        annotation.thickness = value.get<int>("thickness", 2);
        annotation.alpha = value.get<double>("alpha", 0.5);
        annotation.template_format = value.get<std::string>("template", "");
        annotation.last_update = std::chrono::system_clock::now();
        annotation.cached_text = "";
        annotation.is_static = annotation.template_format.empty();
        annotations_.push_back(annotation);
    }
}


void AnnotateCvStage::Configure() {
    stream_ = app_->GetMainStream();
    if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
        throw std::runtime_error("AnnotateCvStage: only YUV420 format supported");
    info_ = app_->GetStreamInfo(stream_);
    adjusted_scale_ = std::max(0.5, std::min(info_.width / 1200.0, 2.0));
    adjusted_thickness_ = std::max(static_cast<int>(info_.width / 700), 1);

    for (auto &annotation : annotations_) {
        if (annotation.is_static) {
            createTextCache(annotation, annotation.text);
        }
    }
}

void AnnotateCvStage::createTextCache(TextAnnotation &annotation, const std::string &text) {
    int font = FONT_HERSHEY_SIMPLEX;
    int baseline = 0;
    double scale = annotation.scale * adjusted_scale_;
    int thickness = std::max(static_cast<int>(annotation.thickness * adjusted_thickness_), 1);
    int border_width = annotation.border_width;

    Size size = getTextSize(text, font, scale, thickness, &baseline);
    int border_padding = border_width * 2;
    annotation.text_region = Rect(annotation.x - border_width, annotation.y - border_width,
                                  size.width + border_padding, size.height + baseline + border_padding);

    annotation.cached_image = Mat::zeros(annotation.text_region.height, annotation.text_region.width, CV_8UC1);

    if (annotation.bg)
        annotation.cached_image.setTo(*annotation.bg);

    if (annotation.border_color && border_width > 0) {
        Mat border_image = Mat::zeros(annotation.text_region.height, annotation.text_region.width, CV_8UC1);
        putText(border_image, text, Point(border_width, size.height + border_width), font, scale,
                *annotation.border_color, thickness, 0);
        Mat kernel = getStructuringElement(MORPH_RECT, Size(2 * border_width + 1, 2 * border_width + 1));
        dilate(border_image, border_image, kernel);
        add(annotation.cached_image, border_image, annotation.cached_image);
    }

    putText(annotation.cached_image, text, Point(border_width, size.height + border_width), font, scale,
            annotation.fg, thickness, 0);
}

std::string AnnotateCvStage::processTemplate(const std::string &template_str) {
    return template_str; // Skip dynamic template handling for now (optional)
}

void AnnotateCvStage::updateDynamicText(TextAnnotation &annotation) {
    auto now = std::chrono::system_clock::now();
    if (!annotation.is_static && (now - annotation.last_update) >= update_interval_) {
        annotation.cached_text = processTemplate(annotation.template_format);
        createTextCache(annotation, annotation.cached_text);
        annotation.last_update = now;
    }
}

void AnnotateCvStage::drawAnnotation(Mat &im, TextAnnotation &annotation, const FrameInfo &info) {
    if (!annotation.is_static) {
        updateDynamicText(annotation);
        std::string text = info.ToString(annotation.cached_text);
        createTextCache(annotation, text);
    }

    if (!annotation.cached_image.empty()) {
        Rect roi = annotation.text_region & Rect(0, 0, im.cols, im.rows);
        if (roi.area() > 0) {
            Mat src_roi = annotation.cached_image(Rect(0, 0, roi.width, roi.height));
            Mat dst_roi = im(roi);
            src_roi.copyTo(dst_roi);
        }
    }
}

bool AnnotateCvStage::Process(CompletedRequestPtr &completed_request) {
    BufferWriteSync w(app_, completed_request->buffers[stream_]);
    libcamera::Span<uint8_t> buffer = w.Get()[0];
    FrameInfo info(completed_request);
    uint8_t *ptr = (uint8_t *)buffer.data();
    Mat im(info_.height, info_.width, CV_8UC1, ptr, info_.stride);

    for (auto &annotation : annotations_)
        drawAnnotation(im, annotation, info);

    return false;
}

static PostProcessingStage *Create(RPiCamApp *app) { return new AnnotateCvStage(app); }
static RegisterStage reg("annotate_cv", &Create);
