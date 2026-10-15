#pragma once

#include <cstddef>

#include "render/Canvas.h"

namespace navsight::render {

// Canvas backend for X-Plane window/avionics draw callbacks using the legacy
// OpenGL state X-Plane sets up for plugins. Must only be flushed from inside
// an X-Plane draw callback.
//
// Draws with GL 1.1 client vertex arrays (one glDrawArrays per frame) when it
// is safe to, otherwise immediate mode. The fast path can be disabled with
// config `fast_rendering 0` if a graphics driver misbehaves.
class GlCanvas final : public Canvas {
public:
    void flush() override;

    static void setFastPath(bool on) { s_fastPath = on; }
    std::size_t lastTriangles() const { return lastTriangles_; }
    bool lastUsedFastPath() const { return lastUsedFastPath_; }

private:
    static bool s_fastPath;
    std::size_t lastTriangles_ = 0;
    bool lastUsedFastPath_ = false;
};

}  // namespace navsight::render
