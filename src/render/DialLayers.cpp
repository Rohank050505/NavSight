// Dial layers: everything drawn inside / on the round instrument.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

#include "render/Layers.h"
#include "render/Palette.h"

namespace navsight::render::layers {


using nav::Command;

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

bool navMode(const InstrumentState& s) { return s.mode != InstrumentMode::Heading; }

// Course axis angle on screen (heading-up card), degrees clockwise from up.
double courseTheta(const InstrumentState& s) { return s.courseDegMag - s.displayHeadingDeg; }

// Lateral position of the course line relative to the aircraft, dial units.
float courseOffset(double aircraftPosDots) {
    const double needle = std::clamp(-aircraftPosDots, -double(kMaxNeedleDots), double(kMaxNeedleDots));
    return static_cast<float>(needle) * kDotSpacing;
}

void flag(Canvas& c, const Dial& d, Vec2 at, const char* text) {
    const Vec2 m = d.pt(at.x, at.y);
    const float hw = d.len(0.05f), hh = d.len(0.026f);
    c.fillRect(m.x - hw, m.y - hh, m.x + hw, m.y + hh, pal().kRed);
    TextStyle st;
    st.height = d.len(0.03f);
    st.color = pal().kScale;
    drawText(c, text, m, st);
}

}  // namespace

void dialBackground(Canvas& c, const Dial& d) {
    c.fillCircle(d.center, d.len(kBezelRadius), pal().kBezelRing);
    c.fillCircle(d.center, d.len(kFaceRadius + kRimWidth), pal().kDialRim);
    c.fillCircle(d.center, d.len(kFaceRadius), pal().kDialFace);
}

// 2-D course-alignment strip (not synthetic vision): a constant-width band
// laid along the selected course, rotated with the card (course - heading) and
// displaced by the measured lateral deviation on the same dot scale as the CDI
// needle. The aircraft symbol on the dashed centreline = on course; strip
// vertical = tracking parallel to the course. The band is clipped to the dial
// face, so it never suggests distance to, or position over, a runway.
//
// Two variants:
//  - Runway strip: approach with a runway designator verified from the nav
//    database. Green boundaries + designator near the bottom.
//  - Course reference: any other valid course (VOR, GPS leg, or an approach
//    whose runway couldn't be verified). Narrow neutral band, labelled with the
//    course instead of inventing a runway.
void runwayOverlay(Canvas& c, const Dial& d, const InstrumentState& s) {
    if (s.mode == InstrumentMode::Heading || !s.courseValid || !s.lateral.out.valid) return;

    const bool runway = s.mode == InstrumentMode::RunwayAlignment && !s.runway.empty();
    const double theta = courseTheta(s);
    const RotFrame rf = d.rotated(theta);
    const float xc = courseOffset(s.lateral.out.positionDots);
    const float hw = runway ? kStripHalfWidth : kCourseRefHalfWidth;
    constexpr float R = kStripClipRadius;

    const float xl = xc - hw, xr = xc + hw;
    if (std::fabs(xl) >= R || std::fabs(xr) >= R) return;
    const float yl = std::sqrt(R * R - xl * xl), yr = std::sqrt(R * R - xr * xr);

    // Band ∩ disc is convex: right edge up, arc over the top, left edge down,
    // arc under the bottom.
    std::vector<Vec2> poly;
    auto arc = [&](float a0, float a1) {
        while (a1 < a0) a1 += 6.2831853f;
        const int n = std::max(2, static_cast<int>((a1 - a0) * 24.0f));
        for (int i = 0; i <= n; ++i) {
            const float a = a0 + (a1 - a0) * static_cast<float>(i) / n;
            poly.push_back(rf(R * std::cos(a), R * std::sin(a)));
        }
    };
    poly.push_back(rf(xr, -yr));
    arc(std::atan2(yr, xr), std::atan2(yl, xl));
    arc(std::atan2(-yl, xl), std::atan2(-yr, xr));
    c.fillConvex(poly.data(), poly.size(), runway ? pal().kStripFill : pal().kCourseRefFill);

    // Boundaries
    if (runway) {
        const Color edge = s.aligned ? pal().kGreen : withAlpha(pal().kGreen, 0.8f);
        for (const auto& [x, y] : {std::pair{xl, yl}, std::pair{xr, yr}}) {
            c.line(rf(x, -y), rf(x, y), d.len(0.016f), withAlpha(pal().kGreen, 0.14f));  // soft glow
            c.line(rf(x, -y), rf(x, y), d.len(0.005f), edge);
        }
        // Subtle dashed edge stripes just inside the boundaries.
        for (const float x : {xl + 0.016f, xr - 0.016f}) {
            const float y = std::sqrt(std::max(0.0f, R * R - x * x)) - 0.02f;
            dashedLine(c, rf(x, -y), rf(x, y), d.len(0.0035f), d.len(0.028f), d.len(0.022f),
                       withAlpha(pal().kScale, 0.35f));
        }
    } else {
        for (const auto& [x, y] : {std::pair{xl, yl}, std::pair{xr, yr}}) {
            dashedLine(c, rf(x, -y), rf(x, y), d.len(0.004f), d.len(0.022f), d.len(0.016f),
                       withAlpha(pal().kMuted, 0.7f));
        }
    }

    // Label near the bottom of the band: runway designator, or the course.
    TextStyle st;
    st.angleRad = static_cast<float>(theta * kDegToRad);  // upright when tracking the course
    constexpr float yLabel = -0.235f;  // inside the rose's label ring (inner edge ~0.29)
    if (runway) {
        st.height = d.len(0.062f);
        st.weight = 0.17f;
        st.color = withAlpha(pal().kScale, 0.95f);
        drawTextFit(c, s.runway, rf(xc, yLabel), st, d.len(hw * 1.7f));
        if (s.runwayManual) {
            // Pilot-declared runway: tag it so it never reads as verified data.
            st.height = d.len(0.024f);
            st.weight = 0.16f;
            st.color = pal().kAmber;
            drawText(c, "MAN", rf(xc, yLabel - 0.052f), st);
        }
    } else {
        char buf[12];
        const int crs = static_cast<int>(std::lround(s.courseDegMag)) % 360;
        std::snprintf(buf, sizeof(buf), "%03d*", crs == 0 ? 360 : crs);
        st.height = d.len(0.034f);
        st.color = pal().kMuted;
        st.aspect = kCondensed;
        drawTextFit(c, buf, rf(xc, yLabel), st, d.len(hw * 1.8f));
    }

    // Dashed centreline, from above the label to the top of the band.
    const float yTop = std::sqrt(std::max(0.0f, R * R - xc * xc)) - 0.025f;
    dashedLine(c, rf(xc, yLabel + 0.055f), rf(xc, yTop), d.len(0.006f), d.len(0.032f), d.len(0.022f),
               withAlpha(pal().kScale, runway ? 0.85f : 0.6f));
}

void compassRose(Canvas& c, const Dial& d, const InstrumentState& s) {
    // Heading-up card: compass bearing B is drawn at screen angle (B - heading).
    const double hdg = s.displayHeadingDeg;

    for (int deg = 0; deg < 360; deg += 5) {
        const double theta = deg - hdg;
        const bool major = deg % 30 == 0;
        const bool tens = deg % 10 == 0;
        const float length = major ? 0.045f : tens ? 0.034f : 0.02f;
        const float width = major ? 0.0075f : tens ? 0.0055f : 0.004f;
        c.line(d.polar(kTickOuter, theta), d.polar(kTickOuter - length, theta), d.len(width), pal().kScale);
    }

    // HSI convention: cardinals as letters, other 30-degree marks in tens.
    static constexpr const char* kCardinals[] = {"N", "E", "S", "W"};
    for (int deg = 0; deg < 360; deg += 30) {
        const double theta = deg - hdg;
        const bool cardinal = deg % 90 == 0;
        char label[4];
        if (cardinal) std::snprintf(label, sizeof(label), "%s", kCardinals[deg / 90]);
        else std::snprintf(label, sizeof(label), "%d", deg / 10);

        TextStyle st;
        st.height = d.len(cardinal ? 0.060f : 0.048f);
        st.weight = cardinal ? 0.16f : 0.13f;
        st.angleRad = static_cast<float>(theta * kDegToRad);  // tops point outward
        st.color = pal().kScale;
        drawText(c, label, d.polar(kLabelRadius, theta), st);
    }
}

void bearingPointer(Canvas& c, const Dial& d, const InstrumentState& s) {
    if (!navMode(s) || !s.bearingValid) return;
    const double theta = s.bearingDegMag - s.displayHeadingDeg;
    const Vec2 tip = d.polar(kTickOuter + 0.002f, theta);
    const Vec2 b0 = d.polar(kFaceRadius + kRimWidth + 0.004f, theta - 2.6);
    const Vec2 b1 = d.polar(kFaceRadius + kRimWidth + 0.004f, theta + 2.6);
    c.triangle(tip, b0, b1, pal().kCyan);
}

// Classic HSI course pointer + deviation bar, in magenta.
void courseDeviation(Canvas& c, const Dial& d, const InstrumentState& s) {
    if (!navMode(s) || !s.courseValid) return;
    const RotFrame rf = d.rotated(courseTheta(s));
    const float w = d.len(0.012f);

    // Deviation scale dots, perpendicular to the course.
    for (int i : {-2, -1, 1, 2}) {
        c.ring(rf(i * kDotSpacing, 0.0f), d.len(0.009f), d.len(0.0135f), withAlpha(pal().kScale, 0.9f), 16);
    }

    // Course pointer head and tail (fixed on the course axis).
    c.line(rf(0, 0.20f), rf(0, 0.315f), w, navColor(s));
    const Vec2 head[] = {rf(0, 0.385f), rf(0.03f, 0.31f), rf(-0.03f, 0.31f)};
    c.fillConvex(head, 3, navColor(s));
    // Tail: leave a gap around the runway designator so it stays legible.
    const bool designator = s.mode == InstrumentMode::RunwayAlignment && !s.runway.empty();
    const float xs = courseOffset(s.lateral.out.valid ? s.lateral.out.positionDots : 0.0);
    if (designator && std::fabs(xs) < 0.07f) {
        c.line(rf(0, -0.282f), rf(0, -0.36f), w, navColor(s));  // below the designator
    } else {
        c.line(rf(0, -0.20f), rf(0, -0.36f), w, navColor(s));
    }

    // Deviation bar: displaced toward the course.
    const auto& lat = s.lateral.out;
    if (!lat.valid) return;
    const float xn = courseOffset(lat.positionDots);
    c.line(rf(xn, -0.165f), rf(xn, 0.165f), d.len(0.022f), withAlpha(navColor(s), 0.25f));  // glow
    c.line(rf(xn, -0.165f), rf(xn, 0.165f), w, navColor(s));
    for (float y : {-0.10f, 0.10f}) {
        const Vec2 dm[] = {rf(xn, y + 0.03f), rf(xn + 0.017f, y), rf(xn, y - 0.03f), rf(xn - 0.017f, y)};
        c.fillConvex(dm, 4, navColor(s));
    }

    // Predicted bar position at the prediction horizon (hollow, cyan), level
    // with the bar's lower diamond: "this diamond will be here in 10 s".
    const float xp = courseOffset(lat.predictedDots);
    // (Superseded by the projected track when a trajectory estimate exists.)
    if (s.lateral.predictive && !drawsProjectedTrack(s) && std::fabs(xp - xn) > 0.3f * kDotSpacing) {
        constexpr float yp = -0.10f;
        const Vec2 p[] = {rf(xp, yp + 0.03f), rf(xp + 0.018f, yp), rf(xp, yp - 0.03f), rf(xp - 0.018f, yp)};
        strokeClosed(c, p, 4, d.len(0.005f), pal().kCyan);
    }
}

// Glideslope / glidepath scale on the right of the aircraft.
void glideslope(Canvas& c, const Dial& d, const InstrumentState& s) {
    const auto& v = s.vertical.out;
    const bool expected = s.mode == InstrumentMode::RunwayAlignment && s.verticalExpected;
    if (!v.valid && !expected) return;

    // Inside the label ring (labels' inner edge ~0.29) and outside the chevrons.
    constexpr float x = 0.235f;
    for (int i : {-2, -1, 1, 2}) {
        c.ring(d.pt(x, i * kGsDotSpacing), d.len(0.009f), d.len(0.0135f), withAlpha(pal().kScale, 0.9f), 16);
    }
    c.line(d.pt(x - 0.022f, 0), d.pt(x + 0.022f, 0), d.len(0.005f), pal().kScale);

    if (!v.valid) {
        flag(c, d, {x, 0.0f}, "GS");
        return;
    }
    auto needleY = [](double posDots) {
        return static_cast<float>(std::clamp(-posDots, -double(kMaxNeedleDots), double(kMaxNeedleDots))) *
               kGsDotSpacing;
    };
    const float y = needleY(v.positionDots);
    // Short bar either side of the diamond; starts clear of the lateral chevrons.
    c.line(d.pt(x - 0.04f, y), d.pt(x + 0.04f, y), d.len(0.006f), withAlpha(navColor(s), 0.8f));
    diamond(c, d.pt(x, y), d.len(0.03f), d.len(0.019f), navColor(s));

    const float yp = needleY(v.predictedDots);
    if (s.vertical.predictive && std::fabs(yp - y) > 0.3f * kGsDotSpacing) {
        diamondOutline(c, d.pt(x, yp), d.len(0.03f), d.len(0.019f), d.len(0.005f), pal().kCyan);
    }
}

bool drawsProjectedTrack(const InstrumentState& s) {
    const auto& L = s.trajectory.lateral;
    return navMode(s) && s.lateral.predictive && s.courseValid && L.valid && L.confidence != nav::Confidence::None;
}

// Projected track: where the aircraft will be over the next 20 s if it keeps
// its groundspeed and turn rate. Drawn in the course-aligned frame with the
// SAME lateral scale as the CDI dots and the runway strip, so the point where
// it meets the strip centreline is the predicted course crossing. Along-track
// uses the same nm-per-dial-unit as cross-track (isotropic), so the path
// leaves the nose in the true track direction. Coarse scales (en route) give a
// short path; fine scales (short final) are clipped at the dial face.
void projectedTrack(Canvas& c, const Dial& d, const InstrumentState& s, const DisplayFx* fx) {
    if (!drawsProjectedTrack(s)) return;
    const auto& L = s.trajectory.lateral;
    const bool trusted = L.confidence == nav::Confidence::Calibrated || L.confidence == nav::Confidence::Measured;
    // A prior-scale estimate is drawn dimmed; fades in when it first appears.
    const float alpha = (trusted ? 0.95f : 0.45f) * (fx ? fx->trackAlpha : 1.0f);
    if (alpha <= 0.01f) return;

    const RotFrame rf = d.rotated(courseTheta(s));
    const float pos0 = static_cast<float>(s.lateral.out.positionDots);
    const float alongScale = kDotSpacing / static_cast<float>(std::max(1e-3, L.nmPerDot));  // dial units per nm
    const float sign0 = pos0 > 0.0f ? 1.0f : -1.0f;
    auto local = [&](const nav::PathPoint& p) {
        return Vec2{(p.crossDots - pos0) * kDotSpacing, p.alongNm * alongScale};
    };
    auto toDial = [&](const nav::PathPoint& p) {
        const Vec2 l = local(p);
        return rf(l.x, l.y);
    };
    auto inside = [&](const nav::PathPoint& p) {
        const Vec2 l = local(p);
        return std::fabs(p.crossDots) <= kMaxNeedleDots + 0.1f && l.x * l.x + l.y * l.y <= kStripClipRadius * kStripClipRadius;
    };

    Vec2 prev = toDial(L.path[0]);
    for (std::size_t i = 1; i < L.path.size(); ++i) {
        const nav::PathPoint& p = L.path[i];
        if (!inside(p)) break;  // off the scale / outside the face
        const Vec2 cur = toDial(p);
        const bool beyond = L.crossValid && p.tSec >= L.crossSec && -p.crossDots * sign0 > 0.5f;
        const Color col = withAlpha(beyond ? pal().kAmber : pal().kCyan, alpha);
        if (i % 2 == 1) c.line(prev, cur, d.len(0.007f), col);  // dotted: every other 0.5 s step
        prev = cur;
        if (i == 20 || i == 40) c.fillCircle(cur, d.len(0.011f), withAlpha(pal().kCyan, alpha), 10);  // 10 s / 20 s
    }

    // Predicted overshoot peak.
    if (trusted && L.overshootDots > 0.5) {
        float best = -1.0f;
        std::size_t at = 0;
        for (std::size_t i = 0; i < L.path.size(); ++i) {
            const float beyondBy = -L.path[i].crossDots * sign0;
            if (!inside(L.path[i])) break;
            if (beyondBy > best) {
                best = beyondBy;
                at = i;
            }
        }
        c.ring(toDial(L.path[at]), d.len(0.012f), d.len(0.019f), withAlpha(pal().kAmber, alpha), 16);
    }
}

void alignmentBox(Canvas& c, const Dial& d, const InstrumentState& s, const DisplayFx* fx) {
    const float alpha = fx ? fx->alignedAlpha : (s.aligned ? 1.0f : 0.0f);  // fades in/out
    if (alpha <= 0.01f) return;
    constexpr float h = 0.095f;
    const Vec2 corners[] = {d.pt(-h, -h), d.pt(h, -h), d.pt(h, h), d.pt(-h, h)};
    for (int i = 0; i < 4; ++i) {
        dashedLine(c, corners[i], corners[(i + 1) % 4], d.len(0.005f), d.len(0.02f), d.len(0.013f),
                   withAlpha(pal().kGreen, alpha));
    }
}

// Predictive guidance chevrons around the aircraft: direction to fly, count =
// strength of the correction. Amber when easing off (overshoot predicted).
// With display fx, cues fade in, and the last cue fades out when the advice
// becomes HOLD / ON COURSE (an invalid axis drops it immediately).
void guidanceChevrons(Canvas& c, const Dial& d, const InstrumentState& s, const DisplayFx* fx) {
    if (!navMode(s)) return;
    static constexpr float kFade[] = {1.0f, 0.65f, 0.4f};

    auto drawCue = [&](int direction, nav::Strength strength, bool ease, float alpha, bool lateral) {
        if (alpha <= 0.01f || direction == 0) return;
        const Color base = ease ? pal().kAmber : pal().kCyan;
        const int n = static_cast<int>(strength);
        const float dir = static_cast<float>(direction);
        for (int i = 0; i < n; ++i) {
            // Tips at 0.115-0.185: clear of the aircraft and of the GS scale (0.235).
            const float off = 0.115f + i * 0.035f;
            const Vec2 tip = lateral ? d.pt(dir * off, 0) : d.pt(0, dir * off);
            const Vec2 v = lateral ? Vec2{dir, 0} : Vec2{0, dir};
            chevron(c, tip, v, d.len(0.026f), d.len(0.010f), withAlpha(base, kFade[i] * alpha));
        }
    };
    auto drawAxis = [&](const GuidanceAxis& axis, const DisplayFx::Cue* cue, bool lateral) {
        if (cue) {
            if (cue->present) drawCue(cue->direction, cue->strength, cue->ease, cue->alpha, lateral);
            return;
        }
        const nav::AxisOutput& o = axis.out;
        if (!axis.predictive || (o.command != Command::Correct && o.command != Command::Ease)) return;
        drawCue(o.direction, o.strength, o.command == Command::Ease, 1.0f, lateral);
    };
    drawAxis(s.lateral, fx ? &fx->lateralChevrons : nullptr, true);
    drawAxis(s.vertical, fx ? &fx->verticalChevrons : nullptr, false);
}

void fixedMarkers(Canvas& c, const Dial& d) {
    // Lubber index at 12 o'clock (pointing down onto the card).
    {
        const Vec2 tip = d.polar(kTickOuter + 0.003f, 0.0);
        const Vec2 bl = d.polar(0.445f, -3.2), br = d.polar(0.445f, 3.2);
        c.triangle(tip, br, bl, pal().kScale);
    }
    // Smaller fixed references at 3, 6 and 9 o'clock, outside the face.
    for (double theta : {90.0, 180.0, 270.0}) {
        const Vec2 tip = d.polar(kFaceRadius + kRimWidth + 0.002f, theta);
        const Vec2 b0 = d.polar(kFaceRadius + kRimWidth + 0.026f, theta - 2.4);
        const Vec2 b1 = d.polar(kFaceRadius + kRimWidth + 0.026f, theta + 2.4);
        c.triangle(tip, b0, b1, pal().kScale);
    }
    for (double theta : {45.0, 135.0, 225.0, 315.0}) {
        c.line(d.polar(kFaceRadius + kRimWidth + 0.004f, theta), d.polar(kFaceRadius + kRimWidth + 0.024f, theta),
               d.len(0.006f), pal().kScale);
    }
}

void aircraftSymbol(Canvas& c, const Dial& d) {
    // Top-down airliner silhouette in symbol units (nose at +1); convex parts.
    const float s = d.len(0.074f);
    auto P = [&](float x, float y) { return Vec2{d.center.x + x * s, d.center.y + y * s}; };

    const Vec2 fuselage[] = {P(0.0f, 1.0f),    P(0.09f, 0.86f),  P(0.11f, 0.55f),
                             P(0.11f, -0.75f), P(0.05f, -1.0f),  P(-0.05f, -1.0f),
                             P(-0.11f, -0.75f), P(-0.11f, 0.55f), P(-0.09f, 0.86f)};
    const Vec2 wingL[] = {P(-0.1f, 0.36f), P(-0.1f, 0.02f), P(-1.0f, -0.20f), P(-1.0f, -0.07f)};
    const Vec2 wingR[] = {P(0.1f, 0.36f), P(1.0f, -0.07f), P(1.0f, -0.20f), P(0.1f, 0.02f)};
    const Vec2 tailL[] = {P(-0.1f, -0.62f), P(-0.1f, -0.86f), P(-0.42f, -0.98f), P(-0.42f, -0.88f)};
    const Vec2 tailR[] = {P(0.1f, -0.62f), P(0.42f, -0.88f), P(0.42f, -0.98f), P(0.1f, -0.86f)};

    // Dark outline first, so the white symbol stays distinct over the strip,
    // needle and chevrons.
    const float outline = s * 0.16f;
    for (const auto* part : {wingL, wingR, tailL, tailR}) strokeClosed(c, part, 4, outline, pal().kAircraftOutline);
    strokeClosed(c, fuselage, 9, outline, pal().kAircraftOutline);

    for (const auto* part : {wingL, wingR, tailL, tailR}) c.fillConvex(part, 4, pal().kAircraft);
    c.fillConvex(fuselage, 9, pal().kAircraft);

    // Subtle shading: trailing half of each wing, and a cockpit window.
    const Vec2 shadeL[] = {P(-0.1f, 0.18f), P(-0.1f, 0.02f), P(-1.0f, -0.20f), P(-1.0f, -0.135f)};
    const Vec2 shadeR[] = {P(0.1f, 0.18f), P(1.0f, -0.135f), P(1.0f, -0.20f), P(0.1f, 0.02f)};
    c.fillConvex(shadeL, 4, pal().kAircraftShade);
    c.fillConvex(shadeR, 4, pal().kAircraftShade);
    const Vec2 window[] = {P(0.0f, 0.88f), P(0.06f, 0.78f), P(0.0f, 0.74f), P(-0.06f, 0.78f)};
    c.fillConvex(window, 4, pal().kAircraftOutline);
}

void headingBox(Canvas& c, const Dial& d, const InstrumentState& s) {
    const Vec2 mid = d.polar(kHeadingBoxCenterR, 0.0);
    const float hw = d.len(0.085f), hh = d.len(kHeadingBoxHalfH);
    c.fillRect(mid.x - hw, mid.y - hh, mid.x + hw, mid.y + hh, pal().kReadoutBg);
    c.strokeRect(mid.x - hw, mid.y - hh, mid.x + hw, mid.y + hh, d.len(0.004f), pal().kReadoutBorder);

    char text[8];
    TextStyle st;
    st.height = d.len(0.042f);
    st.weight = 0.15f;
    if (s.headingValid) {
        std::snprintf(text, sizeof(text), "%03d*", s.headingReadoutDeg);  // '*' = degree sign
        st.color = pal().kCyan;
    } else {
        std::snprintf(text, sizeof(text), "---*");
        st.color = pal().kAmber;
    }
    drawText(c, text, mid, st);
}

void failureFlags(Canvas& c, const Dial& d, const InstrumentState& s) {
    if (!s.headingValid) flag(c, d, {-0.17f, 0.08f}, "HDG");
    if (s.lateralFlagged) {
        flag(c, d, {-0.17f, -0.08f}, "NAV");
    } else if (s.lateralLost) {
        const bool gps = s.sourceName.rfind("GPS", 0) == 0;
        flag(c, d, {-0.17f, -0.08f}, gps ? "GPS" : "LOC");
    }
}

}  // namespace navsight::render::layers
