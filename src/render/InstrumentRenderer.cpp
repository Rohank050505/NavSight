#include "render/InstrumentRenderer.h"

#include <algorithm>
#include <cstdio>
#include <string>

#include "render/Layers.h"
#include "render/Palette.h"

namespace navsight::render {

namespace {

void drawDial(Canvas& c, const Dial& d, const InstrumentState& s, const DisplayFx* fx) {
    using namespace layers;
    dialBackground(c, d);
    runwayOverlay(c, d, s);
    compassRose(c, d, s);
    bearingPointer(c, d, s);
    courseDeviation(c, d, s);
    projectedTrack(c, d, s, fx);
    glideslope(c, d, s);
    alignmentBox(c, d, s, fx);
    guidanceChevrons(c, d, s, fx);
    fixedMarkers(c, d);
    aircraftSymbol(c, d);
    headingBox(c, d, s);
    failureFlags(c, d, s);
}

std::string degText(bool valid, double deg) {
    if (!valid) return "---*";
    int v = static_cast<int>(deg + 0.5) % 360;
    if (v == 0) v = 360;
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%03d*", v);
    return buf;
}

// Line of up to three tones: "APPROACH:" (text) + value (colour) + optional
// tag (e.g. amber "MAN"), scaled together to fit.
void labelValue(Canvas& c, Vec2 at, const std::string& label, const std::string& value, Color valueColor,
                float height, float maxWidth, const std::string& tag = {}, Color tagColor = {}) {
    const std::string gap = "  ";
    const std::string all = label + gap + value + (tag.empty() ? "" : " " + tag);
    const float w = measureText(all, height, kCondensed);
    if (w > maxWidth) height *= maxWidth / w;
    TextStyle st;
    st.align = TextAlign::Left;
    st.height = height;
    st.aspect = kCondensed;
    st.color = pal().kText;
    drawText(c, label, at, st);
    const float vx = at.x + measureText(label + gap, height, kCondensed);
    st.color = valueColor;
    drawText(c, value, {vx, at.y}, st);
    if (!tag.empty()) {
        st.color = tagColor;
        st.weight = 0.16f;
        drawText(c, tag, {vx + measureText(value + " ", height, kCondensed), at.y}, st);
    }
}

void boxPanel(Canvas& c, const Frame& f, const Rect& r) {
    fillRoundRect(c, r, f.len(0.014f), pal().kPanelBg);
    strokeRoundRect(c, r, f.len(0.014f), f.len(0.0025f), pal().kPanelBorder);
}

// Geometry shared by draw() and hitTest().
Frame squareFrame(const InstrumentRenderer::Viewport& vp) {
    const float side = std::min(vp.width, vp.height);
    return {vp.x + (vp.width - side) * 0.5f, vp.y + (vp.height - side) * 0.5f, side};
}
Rect panelRect(const Frame& f, Axis a) {
    return a == Axis::Lateral ? f.rect(0.012f, 0.165f, 0.197f, 0.735f) : f.rect(0.803f, 0.165f, 0.988f, 0.735f);
}
// Collapsed: slim tabs on the outer edges, outside the dial's tick ring.
Rect tabRect(const Frame& f, Axis a) {
    return a == Axis::Lateral ? f.rect(0.008f, 0.33f, 0.062f, 0.57f) : f.rect(0.938f, 0.33f, 0.992f, 0.57f);
}
// The APPROACH / SOURCE box (opens the runway picker).
Rect approachRect(const Frame& f, bool standard) {
    return standard ? f.rect(0.615f, 0.868f, 0.985f, 0.985f) : f.rect(0.50f, 0.92f, 1.0f, 1.0f);
}
bool contains(const Rect& r, float x, float y) { return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1; }

struct PickerLayout {
    Rect panel;
    Rect cand[3];
    Rect side[4];
    Rect autoBtn, apply;
    Vec2 title, entry, active;
};

PickerLayout pickerLayout(const Frame& f, bool standard) {
    PickerLayout L;
    L.panel = standard ? f.rect(0.20f, 0.27f, 0.80f, 0.73f) : f.rect(0.08f, 0.20f, 0.92f, 0.80f);
    const Rect& p = L.panel;
    const float W = p.width(), H = p.height();
    const float m = 0.05f * W, gap = 0.03f * W;
    auto row = [&](Rect* out, int n, float yTop, float yBot) {
        const float bw = (W - 2 * m - (n - 1) * gap) / n;
        for (int i = 0; i < n; ++i) {
            const float x0 = p.x0 + m + i * (bw + gap);
            out[i] = {x0, p.y1 - yBot * H, x0 + bw, p.y1 - yTop * H};
        }
    };
    row(L.cand, 3, 0.20f, 0.37f);
    row(L.side, 4, 0.42f, 0.57f);
    Rect bottom[2];
    row(bottom, 2, 0.82f, 0.95f);
    L.autoBtn = bottom[0];
    L.apply = bottom[1];
    L.title = {p.x0 + m, p.y1 - 0.10f * H};
    L.entry = {p.x0 + m, p.y1 - 0.655f * H};
    L.active = {p.x0 + m, p.y1 - 0.745f * H};
    return L;
}

void button(Canvas& c, const Frame& f, const Rect& r, const std::string& text, bool selected, bool enabled) {
    fillRoundRect(c, r, f.len(0.008f), selected ? withAlpha(pal().kCyan, 0.18f) : pal().kReadoutBg);
    strokeRoundRect(c, r, f.len(0.008f), f.len(selected ? 0.004f : 0.0025f),
                    selected ? pal().kCyan : pal().kPanelBorder);
    TextStyle st;
    st.height = r.height() * 0.5f;
    st.aspect = kCondensed;
    st.weight = 0.14f;
    st.color = !enabled ? pal().kMuted : selected ? pal().kCyan : pal().kText;
    drawTextFit(c, text, {(r.x0 + r.x1) * 0.5f, (r.y0 + r.y1) * 0.5f}, st, r.width() * 0.85f);
}

void runwayPicker(Canvas& c, const Frame& f, bool standard, const RunwayPicker& pk) {
    // Dim everything behind the picker.
    c.fillRect(f.ox, f.oy, f.ox + f.unit, f.oy + f.unit, withAlpha(pal().kDialFace, 0.55f));
    const PickerLayout L = pickerLayout(f, standard);
    fillRoundRect(c, L.panel, f.len(0.016f), pal().kPanelBg);
    strokeRoundRect(c, L.panel, f.len(0.016f), f.len(0.004f), pal().kCyan);

    TextStyle st;
    st.align = TextAlign::Left;
    st.aspect = kCondensed;
    st.height = L.panel.height() * 0.07f;
    st.weight = 0.15f;
    st.color = pal().kCyan;
    char title[48];
    if (pk.courseValid)
        std::snprintf(title, sizeof(title), "RUNWAY OVERRIDE  CRS %03d*", static_cast<int>(pk.courseDegMag + 0.5) % 360);
    else
        std::snprintf(title, sizeof(title), "RUNWAY OVERRIDE");
    drawTextFit(c, title, L.title, st, L.panel.width() * 0.9f);

    for (int i = 0; i < 3; ++i) {
        const bool have = i < static_cast<int>(pk.candidates.size());
        button(c, f, L.cand[i], have ? pk.candidates[i] : "--", have && pk.typed.empty() && pk.selected == i, have);
    }
    static constexpr const char* kSides[] = {"-", "L", "C", "R"};
    static constexpr char kSideChars[] = {0, 'L', 'C', 'R'};
    for (int i = 0; i < 4; ++i) button(c, f, L.side[i], kSides[i], pk.side == kSideChars[i], true);

    st.weight = 0.13f;
    st.height = L.panel.height() * 0.06f;
    st.color = pal().kText;
    const std::string entry = "ENTRY  " + pk.entryText() + "   (TYPE 0-9 L C R / ENTER / ESC)";
    drawTextFit(c, entry, L.entry, st, L.panel.width() * 0.9f);
    st.color = pk.active.empty() ? pal().kMuted : pal().kAmber;
    drawTextFit(c, pk.active.empty() ? "ACTIVE  AUTO (NAV DATA)" : "ACTIVE  RWY " + pk.active + " MAN", L.active, st,
                L.panel.width() * 0.9f);

    button(c, f, L.autoBtn, "AUTO", pk.active.empty(), true);
    button(c, f, L.apply, "APPLY", false, !pk.designator().empty());
}

// Title box (top left) and approach/source box (top right), as in the reference.
void header(Canvas& c, const Frame& f, const InstrumentState& s) {
    const Rect title = f.rect(0.015f, 0.868f, 0.575f, 0.985f);
    boxPanel(c, f, title);
    TextStyle st;
    st.align = TextAlign::Left;
    st.height = f.len(0.046f);
    st.weight = 0.16f;
    st.aspect = 0.9f;
    st.color = pal().kCyan;
    drawText(c, "NAVSIGHT", f.at(0.035f, 0.944f), st);
    st.height = f.len(0.0225f);
    st.weight = 0.13f;
    st.aspect = kCondensed;
    st.color = pal().kText;
    if (s.sourceCheck.caution()) {  // wrong source / procedure setup outranks everything
        st.color = pal().kAmber;
        st.weight = 0.15f;
        drawTextFit(c, s.sourceCheck.banner, f.at(0.035f, 0.893f), st, f.len(0.52f));
    } else if (s.runwayCourseMismatch) {
        st.color = pal().kAmber;
        drawTextFit(c, "CHECK RUNWAY - DOES NOT MATCH COURSE", f.at(0.035f, 0.893f), st, f.len(0.52f));
    } else {
        // An approach whose runway can't be verified is shown as a course reference.
        const InstrumentMode shown = (s.mode == InstrumentMode::RunwayAlignment && s.runway.empty())
                                         ? InstrumentMode::Navigation
                                         : s.mode;
        drawTextFit(c, modeTitle(shown), f.at(0.035f, 0.893f), st, f.len(0.52f));
    }

    const Rect info = approachRect(f, true);
    boxPanel(c, f, info);
    const float maxW = f.len(0.34f), h = f.len(0.025f);

    std::string value;
    Color valueColor = pal().kCyan;
    const char* label = "SOURCE:";
    if (s.mode == InstrumentMode::RunwayAlignment) {
        label = "APPROACH:";
        value = s.approachLabel + (s.approachMode.empty() ? "" : " " + s.approachMode) +
                (s.airport.empty() ? "" : " " + s.airport);
    } else if (s.mode == InstrumentMode::Navigation) {
        value = s.sourceName + (s.approachLabel.empty() ? "" : " " + s.approachLabel);
    } else {
        value = s.sourceName + " NO SIGNAL";
        valueColor = pal().kMuted;
    }
    labelValue(c, f.at(0.632f, 0.944f), label, value, valueColor, h, maxW, s.runwayManual ? "MAN" : "",
               pal().kAmber);

    const bool nav = s.mode != InstrumentMode::Heading;
    // Bearing (to the station/waypoint) and track (over the ground) are
    // different quantities; both are shown, labelled.
    std::string line = "CRS: " + degText(nav && s.courseValid, s.courseDegMag) +
                       "  BRG: " + degText(nav && s.bearingValid, s.bearingDegMag);
    if (s.trackValid) line += "  TRK: " + degText(true, s.trackDegMag);
    st.height = fitHeight(line, h, maxW, kCondensed);
    st.color = pal().kText;
    drawText(c, line, f.at(0.632f, 0.893f), st);
}

// Nav source check status: green SRC OK, amber caution, grey when not applicable.
Rect sourceChipRect(const Frame& f) { return f.rect(0.385f, 0.013f, 0.615f, 0.057f); }

void sourceChip(Canvas& c, const Frame& f, const nav::SourceCheckResult& r) {
    const Color color = r.status == nav::SourceCheckStatus::Caution ? pal().kAmber
                        : r.status == nav::SourceCheckStatus::Ok    ? pal().kGreen
                                                                    : pal().kMuted;
    const Rect rect = sourceChipRect(f);
    fillRoundRect(c, rect, f.len(0.010f), r.caution() ? withAlpha(pal().kAmber, 0.16f) : pal().kReadoutBg);
    strokeRoundRect(c, rect, f.len(0.010f), f.len(r.caution() ? 0.004f : 0.0025f), color);
    TextStyle st;
    st.height = f.len(0.020f);
    st.aspect = kCondensed;
    st.weight = 0.15f;
    st.color = color;
    drawTextFit(c, r.chip, {(rect.x0 + rect.x1) * 0.5f, (rect.y0 + rect.y1) * 0.5f}, st, rect.width() * 0.88f);
}

void footer(Canvas& c, const Frame& f, const InstrumentState& s) {
    sourceChip(c, f, s.sourceCheck);
    TextStyle st;
    st.height = f.len(0.022f);
    st.aspect = kCondensed;
    st.align = TextAlign::Left;
    if (s.paused) {
        st.color = pal().kAmber;
        drawText(c, "PAUSED", f.at(0.025f, 0.075f), st);
    }
    if (s.mode == InstrumentMode::Heading) return;
    st.color = pal().kMuted;
    drawText(c, s.sourceName, f.at(0.025f, 0.035f), st);
    const std::string dist = distanceText(s);
    if (!dist.empty()) {
        st.align = TextAlign::Right;
        drawTextFit(c, dist, f.at(0.975f, 0.035f), st, f.len(0.34f));  // clear of the source chip
    }
}

void compactHeader(Canvas& c, const Frame& f, const InstrumentState& s) {
    TextStyle st;
    st.align = TextAlign::Left;
    st.height = f.len(0.042f);
    st.weight = 0.15f;
    st.aspect = 0.9f;
    st.color = pal().kCyan;
    drawText(c, "NAVSIGHT", f.at(0.04f, 0.955f), st);

    st.align = TextAlign::Right;
    st.height = f.len(0.028f);
    st.weight = 0.12f;
    st.aspect = kCondensed;
    if (s.paused) {
        st.color = pal().kAmber;
        drawText(c, "PAUSED", f.at(0.96f, 0.955f), st);
    } else if (s.sourceCheck.caution()) {
        st.color = pal().kAmber;
        drawTextFit(c, s.sourceCheck.chip, f.at(0.96f, 0.955f), st, f.len(0.45f));
    } else {
        std::string text = s.mode == InstrumentMode::Heading ? modeShort(s.mode) : s.approachLabel;
        if (text.empty()) text = modeShort(s.mode);
        if (s.runwayManual) text += " MAN";
        st.color = s.runwayManual || s.runwayCourseMismatch ? pal().kAmber : pal().kMuted;
        drawTextFit(c, text, f.at(0.96f, 0.955f), st, f.len(0.45f));
    }
}

}  // namespace

InstrumentRenderer::Layout InstrumentRenderer::chooseLayout(float width, float height) {
    return std::min(width, height) >= kStandardMinSide ? Layout::Standard : Layout::Compact;
}

InstrumentRenderer::Hit InstrumentRenderer::hitTest(const Viewport& vp, float x, float y, const ViewOptions& view) {
    if (vp.width < 8.0f || vp.height < 8.0f) return Hit::None;
    const Frame f = squareFrame(vp);
    const bool standard = chooseLayout(vp.width, vp.height) == Layout::Standard;

    if (view.picker.open) {  // the picker captures every click
        const PickerLayout L = pickerLayout(f, standard);
        for (int i = 0; i < 3; ++i)
            if (contains(L.cand[i], x, y)) return static_cast<Hit>(static_cast<int>(Hit::PickerCandidate0) + i);
        for (int i = 0; i < 4; ++i)
            if (contains(L.side[i], x, y)) return static_cast<Hit>(static_cast<int>(Hit::PickerSideNone) + i);
        if (contains(L.autoBtn, x, y)) return Hit::PickerAuto;
        if (contains(L.apply, x, y)) return Hit::PickerApply;
        return contains(L.panel, x, y) ? Hit::PickerInside : Hit::PickerOutside;
    }

    if (contains(approachRect(f, standard), x, y)) return Hit::OpenRunwayPicker;
    if (!standard) return Hit::None;
    for (Axis a : {Axis::Lateral, Axis::Vertical}) {
        if (contains(view.showPanels ? panelRect(f, a) : tabRect(f, a), x, y)) return Hit::TogglePanels;
    }
    return Hit::None;
}

void InstrumentRenderer::draw(Canvas& c, const InstrumentState& s, const Viewport& vp, const ViewOptions& view) const {
    if (vp.width < 8.0f || vp.height < 8.0f) return;  // nothing legible to draw
    setBrightness(view.brightness);
    c.fillRect(vp.x, vp.y, vp.x + vp.width, vp.y + vp.height, pal().kBezel);

    const Frame f = squareFrame(vp);
    const bool standard = chooseLayout(vp.width, vp.height) == Layout::Standard;

    if (standard) {
        // Dial scaled so the bezel radius is 0.35 of the side; the heading box
        // above the lubber line stays clear of the header boxes.
        const Dial d{f.at(0.5f, 0.445f), f.len(0.35f / layers::kBezelRadius)};
        drawDial(c, d, s, view.fx);
        header(c, f, s);
        footer(c, f, s);
        for (Axis a : {Axis::Lateral, Axis::Vertical}) {
            // Panels overlap the bezel (not the face), like the reference design.
            if (view.showPanels) layers::sidePanel(c, f, panelRect(f, a), s, a);
            else layers::collapsedTab(c, f, tabRect(f, a), s, a);
        }
    } else {
        const Dial d{f.at(0.5f, 0.44f), f.len(0.93f)};
        compactHeader(c, f, s);
        drawDial(c, d, s, view.fx);
    }
    if (view.picker.open) runwayPicker(c, f, standard, view.picker);
}

}  // namespace navsight::render
