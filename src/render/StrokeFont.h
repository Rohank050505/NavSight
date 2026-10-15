#pragma once

#include <string_view>

#include "render/Canvas.h"

// Minimal vector stroke font (A-Z, 0-9 and a few symbols).
//
// XPLMDrawString only offers fixed-size, unrotatable bitmap fonts and does not
// exist in an avionics/3-D cockpit context, whereas instrument labels must
// scale with the window and rotate with the compass card. Glyphs are line
// strokes on a 4x6 grid, rendered through Canvas so they work on any backend.
namespace navsight::render {

enum class TextAlign { Left, Center, Right };

struct TextStyle {
    float height = 12.0f;         // cap height in canvas units
    float weight = 0.12f;         // stroke width as a fraction of height
    float angleRad = 0.0f;        // clockwise rotation about the anchor
    TextAlign align = TextAlign::Center;
    Color color{};
    float aspect = 1.0f;          // horizontal scale; < 1 = condensed
};

// Condensed width used for panel/header text (reference design is condensed).
constexpr float kCondensed = 0.78f;

// Width of `text` in canvas units at the given height (before rotation).
float measureText(std::string_view text, float height, float aspect = 1.0f);

// Draw `text`. The anchor is vertically centred on the cap height; horizontally
// it is the left edge, centre, or right edge depending on style.align.
void drawText(Canvas& canvas, std::string_view text, Vec2 anchor, const TextStyle& style);

}  // namespace navsight::render
