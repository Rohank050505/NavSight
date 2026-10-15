#pragma once

#include <string>

// Persistent user settings. Deliberately free of any X-Plane dependency so it
// can be unit-tested and, later, shared with the companion application.
//
// File format: one "key value" pair per line, '#' starts a comment. Unknown
// keys are ignored (forward compatible); malformed values keep the default.
namespace navsight {

struct WindowGeometry {
    int left = 0, top = 0, right = 0, bottom = 0;
    bool isSet() const { return right > left && top > bottom; }
    int width() const { return right - left; }
    int height() const { return top - bottom; }
};

struct Config {
    // Bumped when the default window shape changes; older files get their
    // saved size replaced (position kept). 2 = wide 1.4:1, 3 = square 600x600.
    static constexpr int kCurrentLayoutVersion = 3;
    int layoutVersion = kCurrentLayoutVersion;

    // Window
    bool windowVisible = true;
    bool showPanels = true;  // LATERAL / VERTICAL panels shown (false = collapsed to edge tabs)
    int brightness = 0;      // 0 day, 1 dim, 2 night (file: day / dim / night)
    bool fastRendering = true;  // GL vertex arrays; 0 = immediate mode (driver workaround)
    WindowGeometry window;  // global desktop boxels; unset => default placement

    // Instrument behaviour
    float headingSmoothingSec = 0.05f;  // display low-pass time constant; 0 = raw
    float lateralClosureSec = 20.0f;    // guidance: ideal time to centre the course
    float verticalClosureSec = 10.0f;   // guidance: ideal time to centre the glidepath
    bool invertLateral = false;         // flip if an aircraft reports deflection inverted
    bool invertVertical = false;

    // Diagnostics
    bool debugLogging = false;

    bool load(const std::string& path);        // false if missing/unreadable
    bool save(const std::string& path) const;  // false on write failure
};

}  // namespace navsight
