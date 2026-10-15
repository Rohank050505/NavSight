#pragma once

#include "render/Canvas.h"

// Colours sampled from the NavSight reference design, with semantic names, as
// a runtime-selectable theme: Day (reference colours), Dim and Night (scaled
// down for dark cockpits; whites dimmed further so the magenta/cyan/amber
// symbology stays distinguishable). Layers read colours through pal(); the
// renderer selects the theme at the start of each draw from ViewOptions.
namespace navsight::render {

enum class Brightness { Day, Dim, Night };
const char* toString(Brightness b);
Brightness nextBrightness(Brightness b);

// X-macro: name, r, g, b, a (Day values), luminance class.
//   W = white/neutral symbol (dimmed extra at night), S = coloured symbol,
//   F = background / fill.
#define NAVSIGHT_THEME_COLORS(X)                                  \
    X(kBezel, 0.075f, 0.082f, 0.094f, 1.0f, F)                   \
    X(kBezelRing, 0.17f, 0.18f, 0.20f, 1.0f, F)                  \
    X(kDialFace, 0.035f, 0.043f, 0.055f, 1.0f, F)                \
    X(kDialRim, 0.26f, 0.28f, 0.31f, 1.0f, F)                    \
    X(kScale, 0.93f, 0.94f, 0.95f, 1.0f, W)       /* ticks, numerals */ \
    X(kAircraft, 0.96f, 0.96f, 0.97f, 1.0f, W)                   \
    X(kReadoutBg, 0.0f, 0.0f, 0.0f, 0.85f, F)                    \
    X(kReadoutBorder, 0.35f, 0.38f, 0.42f, 1.0f, F)              \
    X(kCyan, 0.25f, 0.82f, 1.0f, 1.0f, S)         /* guidance, active readouts */ \
    X(kMagenta, 0.88f, 0.31f, 1.0f, 1.0f, S)      /* measured deviation; GPS needle */ \
    X(kNavGreen, 0.40f, 1.0f, 0.18f, 1.0f, S)     /* ground-based (NAV) needle: yellow-green, distinct from the strip */ \
    X(kGreen, 0.25f, 0.88f, 0.48f, 1.0f, S)       /* runway, converging */ \
    X(kAmber, 1.0f, 0.72f, 0.18f, 1.0f, S)        /* caution */ \
    X(kRed, 0.86f, 0.15f, 0.15f, 1.0f, S)         /* failure flags */ \
    X(kMuted, 0.55f, 0.58f, 0.62f, 1.0f, W)       /* secondary labels */ \
    X(kText, 0.88f, 0.90f, 0.92f, 1.0f, W)        /* panel labels */ \
    X(kStripFill, 0.16f, 0.22f, 0.19f, 0.55f, F)  /* runway strip */ \
    X(kCourseRefFill, 0.55f, 0.58f, 0.62f, 0.07f, F)             \
    X(kAircraftOutline, 0.0f, 0.0f, 0.0f, 0.85f, F)              \
    X(kAircraftShade, 0.74f, 0.76f, 0.79f, 1.0f, W)              \
    X(kPanelBg, 0.045f, 0.055f, 0.07f, 0.94f, F)                 \
    X(kPanelBorder, 0.22f, 0.24f, 0.27f, 1.0f, F)                \
    X(kPipOff, 0.24f, 0.26f, 0.29f, 1.0f, F)                     \
    X(kMeterOff, 0.30f, 0.32f, 0.35f, 1.0f, F)

struct Theme {
#define NAVSIGHT_THEME_MEMBER(name, r, g, b, a, cls) Color name{r, g, b, a};
    NAVSIGHT_THEME_COLORS(NAVSIGHT_THEME_MEMBER)
#undef NAVSIGHT_THEME_MEMBER
};

Theme makeTheme(Brightness b);

// Active theme (single-threaded: set by the renderer before drawing).
void setBrightness(Brightness b);
Brightness brightness();
const Theme& theme();
inline const Theme& pal() { return theme(); }

}  // namespace navsight::render
