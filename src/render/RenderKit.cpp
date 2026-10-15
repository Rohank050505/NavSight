#include "render/RenderKit.h"

#include <algorithm>

namespace navsight::render {

void dashedLine(Canvas& c, Vec2 a, Vec2 b, float width, float dash, float gap, Color color) {
    const Vec2 d = b - a;
    const float total = std::sqrt(d.x * d.x + d.y * d.y);
    if (total <= 1e-4f || dash <= 0.0f) return;
    const Vec2 u = d * (1.0f / total);
    for (float s = 0.0f; s < total; s += dash + gap) {
        const float e = std::min(s + dash, total);
        c.line(a + u * s, a + u * e, width, color);
    }
}

void strokeClosed(Canvas& c, const Vec2* pts, std::size_t n, float width, Color color) {
    for (std::size_t i = 0; i < n; ++i) c.line(pts[i], pts[(i + 1) % n], width, color);
    for (std::size_t i = 0; i < n; ++i) c.fillCircle(pts[i], width * 0.5f, color, 8);
}

void diamond(Canvas& c, Vec2 m, float hw, float hh, Color color) {
    const Vec2 pts[] = {{m.x, m.y + hh}, {m.x + hw, m.y}, {m.x, m.y - hh}, {m.x - hw, m.y}};
    c.fillConvex(pts, 4, color);
}

void diamondOutline(Canvas& c, Vec2 m, float hw, float hh, float width, Color color) {
    const Vec2 pts[] = {{m.x, m.y + hh}, {m.x + hw, m.y}, {m.x, m.y - hh}, {m.x - hw, m.y}};
    strokeClosed(c, pts, 4, width, color);
}

void chevron(Canvas& c, Vec2 tip, Vec2 dir, float size, float thickness, Color color) {
    const Vec2 perp{-dir.y, dir.x};
    const Vec2 back = tip - dir * (size * 0.75f);
    const Vec2 pts[] = {back + perp * size, tip, back - perp * size};
    c.polyline(pts, 3, thickness, color);
}

void blockArrow(Canvas& c, Vec2 center, Vec2 dir, float length, float thickness, Color color) {
    const Vec2 perp{-dir.y, dir.x};
    const Vec2 tip = center + dir * (length * 0.5f);
    const Vec2 tail = center - dir * (length * 0.5f);
    const float headLen = length * 0.45f;
    const Vec2 headBase = tip - dir * headLen;
    c.line(tail, headBase + dir * (thickness * 0.3f), thickness, color);
    c.fillCircle(tail, thickness * 0.5f, color, 10);
    const Vec2 head[] = {tip, headBase + perp * (headLen * 0.85f), headBase - perp * (headLen * 0.85f)};
    c.fillConvex(head, 3, color);
}

float fitHeight(std::string_view text, float height, float maxWidth, float aspect) {
    const float w = measureText(text, height, aspect);
    return (w > maxWidth && w > 0.0f) ? height * (maxWidth / w) : height;
}

void drawTextFit(Canvas& c, std::string_view text, Vec2 anchor, TextStyle style, float maxWidth) {
    style.height = fitHeight(text, style.height, maxWidth, style.aspect);
    drawText(c, text, anchor, style);
}

namespace {
// Rounded rectangle outline as a convex polygon (counter-clockwise).
std::vector<Vec2> roundRectPath(const Rect& r, float radius) {
    radius = std::max(0.0f, std::min(radius, std::min(r.width(), r.height()) * 0.5f));
    constexpr int kSeg = 5;
    const Vec2 centers[] = {{r.x1 - radius, r.y0 + radius}, {r.x1 - radius, r.y1 - radius},
                            {r.x0 + radius, r.y1 - radius}, {r.x0 + radius, r.y0 + radius}};
    std::vector<Vec2> pts;
    for (int corner = 0; corner < 4; ++corner) {
        const float a0 = -1.5707963f + corner * 1.5707963f;  // start angle of this corner
        for (int i = 0; i <= kSeg; ++i) {
            const float a = a0 + 1.5707963f * static_cast<float>(i) / kSeg;
            pts.push_back({centers[corner].x + radius * std::cos(a), centers[corner].y + radius * std::sin(a)});
        }
    }
    return pts;
}
}  // namespace

void fillRoundRect(Canvas& c, const Rect& r, float radius, Color color) {
    const auto pts = roundRectPath(r, radius);
    c.fillConvex(pts.data(), pts.size(), color);
}

void strokeRoundRect(Canvas& c, const Rect& r, float radius, float width, Color color) {
    const float h = width * 0.5f;
    const auto pts = roundRectPath({r.x0 + h, r.y0 + h, r.x1 - h, r.y1 - h}, std::max(0.0f, radius - h));
    for (std::size_t i = 0; i < pts.size(); ++i) c.line(pts[i], pts[(i + 1) % pts.size()], width, color);
}

}  // namespace navsight::render
