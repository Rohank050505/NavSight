#pragma once

#include <cmath>
#include <string_view>
#include <vector>

#include "render/Canvas.h"
#include "render/StrokeFont.h"

// Shared geometry and drawing helpers for the instrument layers.
namespace navsight::render {

struct Rect {
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    float width() const { return x1 - x0; }
    float height() const { return y1 - y0; }
};

// Layout frame: unit coordinates where 1.0 = content height, origin bottom-left.
struct Frame {
    float ox = 0, oy = 0, unit = 1;
    Vec2 at(float u, float v) const { return {ox + u * unit, oy + v * unit}; }
    float len(float k) const { return k * unit; }
    Rect rect(float u0, float v0, float u1, float v1) const {
        return {ox + u0 * unit, oy + v0 * unit, ox + u1 * unit, oy + v1 * unit};
    }
};

// A frame rotated by theta (degrees clockwise from up) around an origin:
// +x is "right" of the axis, +y is "along" it.
struct RotFrame {
    Vec2 origin;
    Vec2 right, along;  // already scaled to canvas units
    Vec2 operator()(float x, float y) const { return origin + right * x + along * y; }
};

// Dial coordinates: units where the dial face radius is ~0.415 (historical
// unit-square layout), centred on the dial, scaled into the frame.
struct Dial {
    Vec2 center;
    float scale = 1;

    float len(float k) const { return k * scale; }
    Vec2 pt(float x, float y) const { return {center.x + x * scale, center.y + y * scale}; }
    Vec2 polar(float r, double thetaDeg) const {
        const double t = thetaDeg * 3.14159265358979323846 / 180.0;
        return {center.x + r * scale * static_cast<float>(std::sin(t)),
                center.y + r * scale * static_cast<float>(std::cos(t))};
    }
    RotFrame rotated(double thetaDeg) const {
        const double t = thetaDeg * 3.14159265358979323846 / 180.0;
        const float s = static_cast<float>(std::sin(t)), c = static_cast<float>(std::cos(t));
        return {center, Vec2{c, -s} * scale, Vec2{s, c} * scale};
    }
};

inline Color withAlpha(Color c, float a) { return {c.r, c.g, c.b, a}; }

void dashedLine(Canvas& c, Vec2 a, Vec2 b, float width, float dash, float gap, Color color);
void strokeClosed(Canvas& c, const Vec2* pts, std::size_t n, float width, Color color);
void diamond(Canvas& c, Vec2 center, float halfW, float halfH, Color color);
void diamondOutline(Canvas& c, Vec2 center, float halfW, float halfH, float width, Color color);

// Chevron ("^" shape) whose tip is at `tip`, pointing along unit vector `dir`.
void chevron(Canvas& c, Vec2 tip, Vec2 dir, float size, float thickness, Color color);
// Block arrow centred at `center`, pointing along unit vector `dir`.
void blockArrow(Canvas& c, Vec2 center, Vec2 dir, float length, float thickness, Color color);

// Text helpers that shrink the height so the text fits in maxWidth.
float fitHeight(std::string_view text, float height, float maxWidth, float aspect = 1.0f);
void drawTextFit(Canvas& c, std::string_view text, Vec2 anchor, TextStyle style, float maxWidth);

void fillRoundRect(Canvas& c, const Rect& r, float radius, Color color);
void strokeRoundRect(Canvas& c, const Rect& r, float radius, float width, Color color);

}  // namespace navsight::render
