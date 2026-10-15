// Side panels: LATERAL (left) and VERTICAL (right) guidance read-outs.
// Laid out in fractions of the panel height so both panels scale identically
// with the window.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "render/Layers.h"
#include "render/Palette.h"

namespace navsight::render::layers {

namespace {

void glyph(Canvas& c, float u, Vec2 at, const AxisText& t) {
    switch (t.glyph) {
        case Glyph::Arrow:
            blockArrow(c, at, t.arrowDir, u * 0.13f, u * 0.03f, t.actionColor);
            break;
        case Glyph::Hold:
            for (float dy : {-0.019f, 0.019f}) {
                c.line({at.x - u * 0.045f, at.y + u * dy}, {at.x + u * 0.045f, at.y + u * dy}, u * 0.016f,
                       t.actionColor);
            }
            break;
        case Glyph::OnTarget:
            c.ring(at, u * 0.036f, u * 0.05f, t.actionColor);
            c.fillCircle(at, u * 0.016f, t.actionColor);
            break;
        case Glyph::None:
            break;
    }
}

}  // namespace

void sidePanel(Canvas& c, const Frame& f, const Rect& r, const InstrumentState& s, Axis axis) {
    const GuidanceAxis& g = axis == Axis::Lateral ? s.lateral : s.vertical;
    const AxisText t = describeAxis(g, axis, &s.trajectory);
    const bool valid = g.out.valid;
    const float u = r.height();
    const float top = r.y1;

    fillRoundRect(c, r, f.len(0.014f), pal().kPanelBg);
    strokeRoundRect(c, r, f.len(0.014f), f.len(0.0025f), pal().kPanelBorder);

    // Deviation meter beside the title/value/cue block: magenta share =
    // |measured deviation| / full scale.
    {
        const float x0 = r.x0 + u * 0.024f, x1 = x0 + u * 0.013f;
        const float yTop = top - u * 0.045f, yBot = top - u * 0.335f;
        c.fillRect(x0, yBot, x1, yTop, pal().kMeterOff);
        if (valid) {
            const float frac = static_cast<float>(
                std::clamp(std::fabs(g.out.positionDots) / g.fullScaleDots, 0.04, 1.0));
            c.fillRect(x0, yTop - (yTop - yBot) * frac, x1, yTop, pal().kMagenta);
        }
    }

    const float tx = r.x0 + u * 0.062f;              // text column beside the meter
    const float tmax = r.x1 - u * 0.025f - tx;
    const float fx = r.x0 + u * 0.03f;               // full-width text below the meter
    const float fmax = r.x1 - u * 0.025f - fx;

    TextStyle st;
    st.align = TextAlign::Left;
    st.aspect = kCondensed;

    st.height = u * 0.044f;
    st.color = pal().kText;
    drawTextFit(c, axis == Axis::Lateral ? "LATERAL" : "VERTICAL", {tx, top - u * 0.07f}, st, tmax - u * 0.06f);

    // Collapse affordance (the whole panel is clickable): chevron pointing to
    // the edge the panel folds into.
    {
        const Vec2 dir = axis == Axis::Lateral ? Vec2{-1.0f, 0.0f} : Vec2{1.0f, 0.0f};
        const Vec2 at{r.x1 - u * 0.04f + dir.x * u * 0.012f, top - u * 0.07f};
        chevron(c, at, dir, u * 0.018f, u * 0.007f, pal().kMuted);
    }

    // Measured deviation: "0.4 DOT R" - big number + smaller unit, bottom-aligned.
    {
        float bigH = u * 0.085f, smallH = u * 0.048f;
        float gap = u * 0.018f;
        const float total = measureText(t.value, bigH, kCondensed) +
                            (t.unit.empty() ? 0.0f : gap + measureText(t.unit, smallH, kCondensed));
        if (total > tmax) {
            const float k = tmax / total;
            bigH *= k;
            smallH *= k;
            gap *= k;
        }
        const float y = top - u * 0.165f;
        st.color = valid ? pal().kMagenta : pal().kMuted;
        st.height = bigH;
        st.weight = 0.15f;
        drawText(c, t.value, {tx, y}, st);
        if (!t.unit.empty()) {
            st.height = smallH;
            st.weight = 0.13f;
            drawText(c, t.unit, {tx + measureText(t.value, bigH, kCondensed) + gap, y - (bigH - smallH) * 0.5f}, st);
        }
        st.weight = 0.12f;
    }

    // Directional cue (cyan = calculated guidance, amber = ease off).
    glyph(c, u, {tx + tmax * 0.45f, top - u * 0.27f}, t);

    st.height = u * 0.056f;
    st.color = t.actionColor;
    st.weight = 0.14f;
    drawTextFit(c, t.action, {fx, top - u * 0.395f}, st, fmax);
    st.weight = 0.12f;
    if (!t.qualifier.empty()) {
        st.height = u * 0.04f;
        st.color = pal().kText;
        drawTextFit(c, t.qualifier, {fx, top - u * 0.46f}, st, fmax);
    }

    c.line({r.x0 + u * 0.022f, top - u * 0.515f}, {r.x1 - u * 0.022f, top - u * 0.515f}, u * 0.004f,
           pal().kPanelBorder);

    st.height = u * 0.036f;
    st.color = pal().kText;
    drawText(c, "TREND", {fx, top - u * 0.565f}, st);

    if (g.predictive) {  // trend is calculated, not measured
        const float k = 0.7071f;
        const Vec2 dir = t.trendArrow < 0 ? Vec2{k, -k} : t.trendArrow > 0 ? Vec2{k, k} : Vec2{1, 0};
        blockArrow(c, {fx + u * 0.045f, top - u * 0.645f}, dir, u * 0.085f, u * 0.02f, t.trendColor);
    }

    st.height = u * 0.05f;
    st.color = t.trendColor;
    st.weight = 0.13f;
    drawTextFit(c, t.trend, {fx, top - u * 0.735f}, st, fmax);
    st.weight = 0.12f;

    if (!t.prediction.empty()) {
        st.height = u * 0.034f;
        st.color = t.predictionColor;  // cyan estimate / amber overshoot / grey low confidence
        drawTextFit(c, t.prediction, {fx, top - u * 0.805f}, st, fmax);
    }

    // Severity pips: each lit pip = up to 0.5 dot of deviation.
    const float pipR = u * 0.019f, pitch = std::min(u * 0.062f, fmax / std::max(1, g.pipCount));
    for (int i = 0; i < g.pipCount; ++i) {
        const Vec2 p{fx + pipR + i * pitch, top - u * 0.9f};
        const bool lit = valid && i < t.litPips;
        c.fillCircle(p, pipR, lit ? pal().kCyan : pal().kPipOff);
    }
}

namespace {
bool sameColor(const Color& a, const Color& b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
std::string formatDotsShort(double v) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%.1f", std::min(std::fabs(v), 9.9));
    return buf;
}
}  // namespace

void collapsedTab(Canvas& c, const Frame& f, const Rect& r, const InstrumentState& s, Axis axis) {
    const GuidanceAxis& g = axis == Axis::Lateral ? s.lateral : s.vertical;
    const AxisText t = describeAxis(g, axis, &s.trajectory);
    // Keep cautions visible while collapsed: amber border.
    const bool caution = sameColor(t.actionColor, pal().kAmber) || sameColor(t.predictionColor, pal().kAmber) ||
                         sameColor(t.actionColor, pal().kRed);
    const float w = r.width();
    const float cx = (r.x0 + r.x1) * 0.5f;

    fillRoundRect(c, r, f.len(0.012f), pal().kPanelBg);
    strokeRoundRect(c, r, f.len(0.012f), f.len(caution ? 0.004f : 0.0025f), caution ? pal().kAmber : pal().kPanelBorder);

    // Expand chevron pointing back toward the dial.
    const Vec2 dir = axis == Axis::Lateral ? Vec2{1.0f, 0.0f} : Vec2{-1.0f, 0.0f};
    chevron(c, {cx + dir.x * w * 0.14f, r.y1 - w * 0.42f}, dir, w * 0.2f, w * 0.075f, pal().kMuted);

    TextStyle st;
    st.aspect = kCondensed;
    st.height = w * 0.32f;
    st.color = pal().kMuted;
    drawTextFit(c, axis == Axis::Lateral ? "LAT" : "VRT", {cx, r.y1 - w * 0.95f}, st, w * 0.8f);

    // Measured deviation stays visible: value + side letter.
    st.height = w * 0.42f;
    st.weight = 0.14f;
    st.color = g.out.valid ? pal().kMagenta : pal().kMuted;
    drawTextFit(c, g.out.valid ? formatDotsShort(g.out.positionDots) : "--", {cx, (r.y0 + r.y1) * 0.5f - w * 0.1f}, st,
                w * 0.86f);
    if (g.out.valid && std::fabs(g.out.positionDots) >= 0.05) {
        st.height = w * 0.3f;
        st.weight = 0.12f;
        const char* side = axis == Axis::Lateral ? (g.out.positionDots > 0 ? "R" : "L")
                                                 : (g.out.positionDots > 0 ? "HI" : "LO");
        drawTextFit(c, side, {cx, (r.y0 + r.y1) * 0.5f - w * 0.62f}, st, w * 0.8f);
    }

    // Directional cue (calculated) as a small arrow at the bottom.
    if (t.glyph == Glyph::Arrow) blockArrow(c, {cx, r.y0 + w * 0.45f}, t.arrowDir, w * 0.55f, w * 0.13f, t.actionColor);
}

}  // namespace navsight::render::layers
