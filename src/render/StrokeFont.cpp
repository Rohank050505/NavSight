#include "render/StrokeFont.h"

#include <array>
#include <cctype>
#include <cmath>
#include <vector>

namespace navsight::render {
namespace {

// Each glyph: polylines separated by '|', points as two digits "xy" on a
// 0..4 (x) by 0..6 (y) grid, origin bottom-left.
struct GlyphDef {
    char ch;
    float width;  // advance width in grid units, excluding inter-glyph gap
    const char* strokes;
};

constexpr GlyphDef kGlyphDefs[] = {
    {'A', 4, "00 04 26 44 40|02 42"},  // low crossbar: stays an "A" at small sizes
    {'B', 4, "00 06 36 45 44 33 03|33 42 41 30 00"},
    {'C', 4, "45 36 16 05 01 10 30 41"},
    {'D', 4, "00 06 26 44 42 20 00"},
    {'E', 4, "46 06 00 40|03 33"},
    {'F', 4, "46 06 00|03 33"},
    {'G', 4, "45 36 16 05 01 10 30 41 43 23"},
    {'H', 4, "00 06|40 46|03 43"},
    {'I', 2, "06 26|16 10|00 20"},
    {'J', 4, "46 41 30 10 01"},
    {'K', 4, "00 06|46 02|13 40"},
    {'L', 4, "06 00 40"},
    {'M', 4, "00 06 23 46 40"},
    {'N', 4, "00 06 40 46"},
    {'O', 4, "10 01 05 16 36 45 41 30 10"},
    {'P', 4, "00 06 36 45 44 33 03"},
    {'Q', 4, "10 01 05 16 36 45 41 30 10|22 40"},
    {'R', 4, "00 06 36 45 44 33 03|23 40"},
    {'S', 4, "45 36 16 05 04 13 33 42 41 30 10 01"},
    {'T', 4, "06 46|26 20"},
    {'U', 4, "06 01 10 30 41 46"},
    {'V', 4, "06 20 46"},
    {'W', 4, "06 10 23 30 46"},
    {'X', 4, "06 40|00 46"},
    {'Y', 4, "06 23 46|23 20"},
    {'Z', 4, "06 46 00 40"},
    {'0', 4, "10 01 05 16 36 45 41 30 10"},
    {'1', 4, "15 26 20|10 30"},
    {'2', 4, "05 16 36 45 44 00 40"},
    {'3', 4, "05 16 36 45 44 33 13|33 42 41 30 10 01"},
    {'4', 4, "30 36 02 42"},
    {'5', 4, "46 06 04 34 43 41 30 10 01"},
    {'6', 4, "45 36 16 05 01 10 30 41 42 33 13 02"},
    {'7', 4, "06 46 20"},
    {'8', 4, "13 04 05 16 36 45 44 33 13 02 01 10 30 41 42 33"},
    {'9', 4, "01 10 30 41 45 36 16 05 04 13 43"},
    {'-', 4, "03 43"},
    {'+', 4, "03 43|25 21"},
    {'/', 4, "00 46"},
    {'>', 3, "05 33 01"},
    {'.', 0, "00 01"},
    {':', 0, "01 02|04 05"},
    {'*', 2, "05 16 25 14 05"},  // '*' renders as a degree sign (small diamond)
    {' ', 2, ""},
};

constexpr float kGap = 1.6f;      // inter-glyph spacing, grid units
constexpr float kGridH = 6.0f;    // cap height, grid units

struct Glyph {
    float width = 4.0f;
    std::vector<std::vector<Vec2>> strokes;
};

const std::array<Glyph, 128>& glyphTable() {
    static const std::array<Glyph, 128> table = [] {
        std::array<Glyph, 128> t{};
        for (const GlyphDef& def : kGlyphDefs) {
            Glyph& g = t[static_cast<unsigned char>(def.ch)];
            g.width = def.width;
            std::vector<Vec2> current;
            for (const char* p = def.strokes; *p; ++p) {
                if (*p == '|') {
                    g.strokes.push_back(std::move(current));
                    current.clear();
                } else if (std::isdigit(static_cast<unsigned char>(p[0])) &&
                           std::isdigit(static_cast<unsigned char>(p[1]))) {
                    current.push_back({static_cast<float>(p[0] - '0'), static_cast<float>(p[1] - '0')});
                    ++p;
                }
            }
            if (!current.empty()) g.strokes.push_back(std::move(current));
        }
        return t;
    }();
    return table;
}

const Glyph* lookup(char c) {
    // The degree sign (U+00B0) isn't ASCII; callers pass '*' for it. Unknown
    // characters render as blank space rather than failing.
    const unsigned char uc = static_cast<unsigned char>(std::toupper(static_cast<unsigned char>(c)));
    if (uc >= 128) return &glyphTable()[' '];
    const Glyph& g = glyphTable()[uc];
    if (g.strokes.empty() && uc != ' ') return &glyphTable()[' '];
    return &g;
}

float widthUnits(std::string_view text) {
    float w = 0.0f;
    for (std::size_t i = 0; i < text.size(); ++i) {
        w += lookup(text[i])->width;
        if (i + 1 < text.size()) w += kGap;
    }
    return w;
}

}  // namespace

float measureText(std::string_view text, float height, float aspect) {
    return widthUnits(text) * (height / kGridH) * aspect;
}

void drawText(Canvas& canvas, std::string_view text, Vec2 anchor, const TextStyle& style) {
    const float scale = style.height / kGridH;
    const float xScale = scale * style.aspect;
    const float totalUnits = widthUnits(text);
    float startUnits = 0.0f;
    if (style.align == TextAlign::Center) startUnits = -totalUnits * 0.5f;
    else if (style.align == TextAlign::Right) startUnits = -totalUnits;

    // Clockwise rotation: local +x maps to `right`, local +y maps to `up`.
    const float s = std::sin(style.angleRad), c = std::cos(style.angleRad);
    const Vec2 right{c, -s};
    const Vec2 up{s, c};
    auto toCanvas = [&](float ux, float uy) {
        const float lx = ux * xScale, ly = (uy - kGridH * 0.5f) * scale;
        return anchor + right * lx + up * ly;
    };

    const float strokeWidth = style.height * style.weight;
    std::vector<Vec2> pts;
    float penUnits = startUnits;
    for (char ch : text) {
        const Glyph* g = lookup(ch);
        for (const auto& stroke : g->strokes) {
            pts.clear();
            for (const Vec2& p : stroke) pts.push_back(toCanvas(penUnits + p.x, p.y));
            if (pts.size() == 1) pts.push_back(pts[0]);
            canvas.polyline(pts.data(), pts.size(), strokeWidth, style.color);
        }
        penUnits += g->width + kGap;
    }
}

}  // namespace navsight::render
