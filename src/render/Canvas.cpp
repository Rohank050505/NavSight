#include "render/Canvas.h"

#include <algorithm>
#include <cmath>

namespace navsight::render {
namespace {

constexpr float kTwoPi = 6.28318530718f;

int autoSegments(float radius) {
    // ~1 segment per 1.6 units of circumference, bounded.
    return std::clamp(static_cast<int>(radius * 4.0f), 8, 180);
}

Color transparent(Color c) { return {c.r, c.g, c.b, 0.0f}; }

}  // namespace

void Canvas::triangle(Vec2 a, Vec2 b, Vec2 c, Color color) {
    batch_.push_back({a, color});
    batch_.push_back({b, color});
    batch_.push_back({c, color});
}

void Canvas::triangle(const Vertex& a, const Vertex& b, const Vertex& c) {
    batch_.push_back(a);
    batch_.push_back(b);
    batch_.push_back(c);
}

// Quad from an inner edge (opaque) to an outer edge (transparent): the
// anti-aliasing fringe.
void Canvas::fringe(Vec2 in0, Vec2 in1, Vec2 out0, Vec2 out1, Color color) {
    const Color clear = transparent(color);
    triangle({in0, color}, {in1, color}, {out1, clear});
    triangle({in0, color}, {out1, clear}, {out0, clear});
}

void Canvas::fillRect(float x0, float y0, float x1, float y1, Color color) {
    triangle({x0, y0}, {x1, y0}, {x1, y1}, color);
    triangle({x0, y0}, {x1, y1}, {x0, y1}, color);
}

void Canvas::strokeRect(float x0, float y0, float x1, float y1, float w, Color color) {
    fillRect(x0, y0, x1, y0 + w, color);
    fillRect(x0, y1 - w, x1, y1, color);
    fillRect(x0, y0 + w, x0 + w, y1 - w, color);
    fillRect(x1 - w, y0 + w, x1, y1 - w, color);
}

void Canvas::fillConvex(const Vec2* pts, std::size_t n, Color color) {
    if (n < 3) return;
    for (std::size_t i = 1; i + 1 < n; ++i) triangle(pts[0], pts[i], pts[i + 1], color);
    if (feather_ <= 0.0f) return;

    // Fringe along every edge, extruded along per-vertex outward normals
    // (average of the adjacent edge normals) so the fringe is continuous.
    float area2 = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = pts[i], b = pts[(i + 1) % n];
        area2 += a.x * b.y - b.x * a.y;
    }
    const float outward = area2 >= 0.0f ? 1.0f : -1.0f;  // CCW: right-hand normal points out
    auto edgeNormal = [&](std::size_t i) {
        const Vec2 a = pts[i], b = pts[(i + 1) % n];
        const Vec2 d = b - a;
        const float len = std::sqrt(d.x * d.x + d.y * d.y);
        return len <= 1e-6f ? Vec2{} : Vec2{d.y / len, -d.x / len} * outward;
    };
    std::vector<Vec2> outer(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 n0 = edgeNormal((i + n - 1) % n), n1 = edgeNormal(i);
        Vec2 v = n0 + n1;
        const float len = std::sqrt(v.x * v.x + v.y * v.y);
        v = len <= 1e-6f ? n1 : v * (1.0f / len);
        outer[i] = pts[i] + v * feather_;
    }
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        fringe(pts[i], pts[j], outer[i], outer[j], color);
    }
}

void Canvas::line(Vec2 a, Vec2 b, float width, Color color) {
    const Vec2 d = b - a;
    const float len = std::sqrt(d.x * d.x + d.y * d.y);
    if (len <= 1e-6f) return;
    const Vec2 u{-d.y / len, d.x / len};
    // With feathering, pull the opaque core in by half the fringe so the
    // stroke keeps the same apparent weight (thin strokes keep >= 70%).
    const float half = feather_ > 0.0f ? std::max(width * 0.35f, width * 0.5f - feather_ * 0.5f) : width * 0.5f;
    const Vec2 n = u * half;
    triangle(a + n, b + n, b - n, color);
    triangle(a + n, b - n, a - n, color);
    if (feather_ > 0.0f) {
        const Vec2 f = u * feather_;
        fringe(a + n, b + n, a + n + f, b + n + f, color);
        fringe(a - n, b - n, a - n - f, b - n - f, color);
    }
}

void Canvas::polyline(const Vec2* pts, std::size_t n, float width, Color color, bool roundJoins) {
    for (std::size_t i = 0; i + 1 < n; ++i) line(pts[i], pts[i + 1], width, color);
    if (roundJoins) {
        const int segs = std::max(6, autoSegments(width * 0.5f) / 2);
        for (std::size_t i = 0; i < n; ++i) fillCircle(pts[i], width * 0.5f, color, segs);
    }
}

void Canvas::fillCircle(Vec2 c, float r, Color color, int segments) {
    if (segments <= 0) segments = autoSegments(r);
    if (feather_ > 0.0f) r = std::max(r * 0.7f, r - feather_ * 0.5f);  // same apparent size (see line())
    Vec2 prev{c.x + r, c.y};
    for (int i = 1; i <= segments; ++i) {
        const float a = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        const Vec2 cur{c.x + r * std::cos(a), c.y + r * std::sin(a)};
        triangle(c, prev, cur, color);
        if (feather_ > 0.0f) {
            const float k = (r + feather_) / r;
            fringe(prev, cur, c + (prev - c) * k, c + (cur - c) * k, color);
        }
        prev = cur;
    }
}

void Canvas::ring(Vec2 c, float r0, float r1, Color color, int segments) {
    if (segments <= 0) segments = autoSegments(r1);
    for (int i = 0; i < segments; ++i) {
        const float a0 = kTwoPi * static_cast<float>(i) / static_cast<float>(segments);
        const float a1 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(segments);
        const Vec2 u0{std::cos(a0), std::sin(a0)}, u1{std::cos(a1), std::sin(a1)};
        const Vec2 i0 = c + u0 * r0, o0 = c + u0 * r1, i1 = c + u1 * r0, o1 = c + u1 * r1;
        triangle(i0, o0, o1, color);
        triangle(i0, o1, i1, color);
        if (feather_ > 0.0f) {
            fringe(o0, o1, c + u0 * (r1 + feather_), c + u1 * (r1 + feather_), color);
            const float rin = std::max(0.0f, r0 - feather_);
            fringe(i0, i1, c + u0 * rin, c + u1 * rin, color);
        }
    }
}

}  // namespace navsight::render
