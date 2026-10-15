#include "render/Palette.h"

namespace navsight::render {
namespace {

// Brightness factors per luminance class: {W white/neutral, S coloured symbol, F fill}.
struct Factors {
    float w, s, f;
};
constexpr Factors kDay{1.0f, 1.0f, 1.0f};
constexpr Factors kDim{0.72f, 0.80f, 0.80f};
constexpr Factors kNight{0.45f, 0.58f, 0.60f};

enum Cls { W, S, F };

Color scaled(Color c, float k) { return {c.r * k, c.g * k, c.b * k, c.a}; }

Theme g_theme = makeTheme(Brightness::Day);
Brightness g_brightness = Brightness::Day;

}  // namespace

const char* toString(Brightness b) {
    switch (b) {
        case Brightness::Day: return "DAY";
        case Brightness::Dim: return "DIM";
        case Brightness::Night: return "NIGHT";
    }
    return "?";
}

Brightness nextBrightness(Brightness b) {
    return b == Brightness::Day ? Brightness::Dim : b == Brightness::Dim ? Brightness::Night : Brightness::Day;
}

Theme makeTheme(Brightness b) {
    const Factors k = b == Brightness::Day ? kDay : b == Brightness::Dim ? kDim : kNight;
    Theme t;
#define NAVSIGHT_THEME_SCALE(name, r, g, bl, a, cls) \
    t.name = scaled(Color{r, g, bl, a}, cls == W ? k.w : cls == S ? k.s : k.f);
    NAVSIGHT_THEME_COLORS(NAVSIGHT_THEME_SCALE)
#undef NAVSIGHT_THEME_SCALE
    return t;
}

void setBrightness(Brightness b) {
    if (b == g_brightness) return;
    g_brightness = b;
    g_theme = makeTheme(b);
}

Brightness brightness() { return g_brightness; }
const Theme& theme() { return g_theme; }

}  // namespace navsight::render
