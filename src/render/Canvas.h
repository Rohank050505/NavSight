#pragma once

#include <cstddef>
#include <vector>

// Backend-neutral 2-D drawing surface.
//
// Every shape is reduced to coloured triangles and batched; a backend only has
// to implement flush(). Today the backend is legacy OpenGL (GlCanvas), which
// is what X-Plane 12.4 supports in window and avionics draw callbacks. If the
// announced "Panel Graphics" SDK API ships, or the instrument moves into a 3-D
// cockpit device, only a new backend is needed - the instrument renderer is
// written purely against this interface.
//
// Coordinates: x right, y up, in whatever units the host passes (boxels for
// windows, texels for avionics devices).
namespace navsight::render {

struct Vec2 {
    float x = 0.0f, y = 0.0f;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }

struct Color {
    float r = 1.0f, g = 1.0f, b = 1.0f, a = 1.0f;
};

class Canvas {
public:
    virtual ~Canvas() = default;

    struct Vertex {
        Vec2 pos;
        Color color;
    };

    // Primitive: append one triangle (uniform colour / per-vertex colour).
    void triangle(Vec2 a, Vec2 b, Vec2 c, Color color);
    void triangle(const Vertex& a, const Vertex& b, const Vertex& c);

    // Anti-aliasing: width (canvas units, ~1 pixel) of a fringe that fades to
    // transparent around lines, circles, rings and convex shapes. 0 = off.
    void setFeather(float width) { feather_ = width; }
    float feather() const { return feather_; }

    // Composite helpers (all reduce to triangle()).
    void fillRect(float x0, float y0, float x1, float y1, Color color);
    void strokeRect(float x0, float y0, float x1, float y1, float width, Color color);
    void fillConvex(const Vec2* pts, std::size_t n, Color color);
    void line(Vec2 a, Vec2 b, float width, Color color);
    void polyline(const Vec2* pts, std::size_t n, float width, Color color, bool roundJoins = true);
    void fillCircle(Vec2 center, float radius, Color color, int segments = 0);
    void ring(Vec2 center, float innerRadius, float outerRadius, Color color, int segments = 0);

    // Submit everything batched so far to the backend, in painter's order.
    virtual void flush() = 0;

    std::size_t pendingVertexCount() const { return batch_.size(); }

protected:
    void fringe(Vec2 in0, Vec2 in1, Vec2 out0, Vec2 out1, Color color);

    std::vector<Vertex> batch_;
    float feather_ = 0.0f;
};

}  // namespace navsight::render
