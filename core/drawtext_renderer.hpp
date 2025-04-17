#pragma once
#include "video_options.hpp"
#include "drawtext_utils.hpp"

#include <cairo/cairo.h>
#include <cairo/cairo-ft.h>

inline void render_drawtext_elements(
	uint8_t *rgb_data, int width, int height, int stride,
	const std::vector<DrawTextElement> &elements
) {
	cairo_surface_t *surface = cairo_image_surface_create_for_data(
		rgb_data, CAIRO_FORMAT_RGB24, width, height, stride);
	cairo_t *cr = cairo_create(surface);

	for (const auto &e : elements) {
		std::string expanded = expand_text_template(e.text_template);

		// Border
		if (e.borderw > 0) {
			cairo_set_source_rgb(cr, 0, 0, 0); // black border
			cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
			cairo_set_font_size(cr, e.fontsize + e.borderw);
			cairo_move_to(cr, e.x, e.y);
			cairo_show_text(cr, expanded.c_str());
		}

		// Main text
		cairo_set_source_rgb(cr, 1, 1, 1); // white
		cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
		cairo_set_font_size(cr, e.fontsize);
		cairo_move_to(cr, e.x, e.y);
		cairo_show_text(cr, expanded.c_str());
	}

	cairo_destroy(cr);
	cairo_surface_destroy(surface);
}
