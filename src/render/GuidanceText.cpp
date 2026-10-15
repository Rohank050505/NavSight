#include "render/GuidanceText.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "render/Palette.h"

namespace navsight::render {


using nav::Command;
using nav::Strength;
using nav::Trend;

namespace {

constexpr double kCentredDots = 0.05;

std::string formatDots(double v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.1f", std::min(std::fabs(v), 9.9));
    return buf;
}

std::string unitText(double pos, Axis axis) {
    if (std::fabs(pos) < kCentredDots) return "DOT";
    if (axis == Axis::Lateral) return pos > 0 ? "DOT R" : "DOT L";
    return pos > 0 ? "DOT HIGH" : "DOT LOW";
}

const char* strengthWord(Strength s) {
    switch (s) {
        case Strength::Slight: return "SLIGHTLY";
        case Strength::Moderate: return "MODERATELY";
        case Strength::Firm: return "FIRMLY";
    }
    return "";
}

const char* directionWord(int dir, Axis axis) {
    if (axis == Axis::Lateral) return dir < 0 ? "TURN LEFT" : "TURN RIGHT";
    return dir > 0 ? "CLIMB" : "DESCEND";
}

}  // namespace

namespace {

// Trajectory-based prediction line. Returns false if there is no estimate.
bool trajectoryPrediction(const nav::TrajectoryOutput& tr, Axis which, AxisText& t) {
    char buf[40];
    if (which == Axis::Lateral) {
        const nav::LateralPrediction& L = tr.lateral;
        if (!L.valid) return false;
        const bool trusted = L.confidence == nav::Confidence::Calibrated || L.confidence == nav::Confidence::Measured;
        const char* est = trusted ? "" : "EST ";
        if (L.turnNow && L.rolloutOvershootDots >= 0.1) {
            std::snprintf(buf, sizeof(buf), "%sOVERSHOOT %.1f DOT", est, L.rolloutOvershootDots);
            t.predictionColor = trusted ? pal().kAmber : pal().kMuted;
        } else if (L.turnInValid && L.turnInSec <= 15.0) {
            std::snprintf(buf, sizeof(buf), "%sTURN IN %dS", est, static_cast<int>(std::lround(L.turnInSec)));
            t.predictionColor = trusted ? pal().kCyan : pal().kMuted;
        } else if (L.crossValid) {
            std::snprintf(buf, sizeof(buf), "%sCROSS %dS", est, static_cast<int>(std::lround(L.crossSec)));
            const bool overshoot = L.outcome == nav::LateralOutcome::CrossingOvershoot;
            t.predictionColor = !trusted ? pal().kMuted : overshoot ? pal().kAmber : pal().kCyan;
        } else {
            return false;
        }
        t.prediction = buf;
        return true;
    }
    const nav::VerticalPrediction& V = tr.vertical;
    if (!V.valid) return false;
    if (V.overshoot) {
        t.prediction = "PATH OVERSHOOT";
        t.predictionColor = pal().kAmber;
    } else {
        std::snprintf(buf, sizeof(buf), "REQ VS %d", static_cast<int>(std::lround(V.requiredVsFpm / 10.0) * 10));
        t.prediction = buf;
        t.predictionColor = V.pathAngleAssumed ? pal().kMuted : pal().kCyan;  // grey: 3.0 deg assumed
    }
    return true;
}

}  // namespace

AxisText describeAxis(const GuidanceAxis& axis, Axis which, const nav::TrajectoryOutput* traj) {
    AxisText t;
    const nav::AxisOutput& o = axis.out;
    t.trend = "--";
    t.trendColor = pal().kMuted;

    if (axis.status == nav::SignalStatus::Flagged) {
        t.value = "---";
        t.action = "NAV FLAG";
        t.qualifier = "RECEIVER";
        t.actionColor = pal().kRed;  // genuine receiver failure indication
        return t;
    }
    if (!o.valid) {
        t.value = "---";
        t.action = which == Axis::Lateral ? "NO COURSE" : "NO GLIDEPATH";
        t.actionColor = pal().kMuted;
        return t;
    }

    // Measured deviation is shown whenever the receiver provides it...
    const bool fullScale = axis.status == nav::SignalStatus::FullScale;
    t.value = formatDots(o.positionDots) + (fullScale ? "+" : "");
    t.unit = unitText(o.positionDots, which);
    t.litPips = std::clamp(static_cast<int>(std::ceil(std::fabs(o.positionDots) / 0.5 - 1e-9)), 1, axis.pipCount);

    // ...but calculated cues only when validation allows them.
    if (!axis.predictive) {
        t.actionColor = pal().kMuted;
        switch (axis.status) {
            case nav::SignalStatus::Acquiring:
                t.action = "ACQUIRING";
                t.qualifier = "SIGNAL";
                break;
            case nav::SignalStatus::FullScale:
                t.action = "FULL SCALE";
                t.qualifier = which == Axis::Lateral ? "OFF COURSE" : "OFF PATH";
                t.actionColor = pal().kAmber;
                break;
            case nav::SignalStatus::NotEstablished:
                t.action = "NOT ESTABLISHED";
                t.qualifier = "LATERALLY";
                break;
            default:  // valid but gated, e.g. low groundspeed
                t.action = "STANDBY";
                t.qualifier = "LOW SPEED";
                break;
        }
        return t;
    }

    switch (o.command) {
        case Command::None:
        case Command::OnTarget:
            t.action = which == Axis::Lateral ? "ON COURSE" : "ON GLIDEPATH";
            t.qualifier = "STABLE";
            t.actionColor = pal().kGreen;
            t.glyph = Glyph::OnTarget;
            break;
        case Command::Hold:
            t.action = "HOLD";
            t.qualifier = which == Axis::Lateral ? "HEADING" : "RATE";
            t.actionColor = pal().kCyan;
            t.glyph = Glyph::Hold;
            break;
        case Command::Correct:
        case Command::Ease:
            t.action = directionWord(o.direction, which);
            // On a descending path, "CLIMB" would be misread: the correction is
            // a change of descent rate. Use that wording when VS data exists.
            if (which == Axis::Vertical && traj && traj->vertical.commandValid && traj->vertical.targetVsFpm < -100.0)
                t.action = o.direction > 0 ? "LESS DESCENT" : "MORE DESCENT";
            t.qualifier = o.command == Command::Ease ? "EASE OFF" : strengthWord(o.strength);
            if (which == Axis::Lateral && traj && traj->lateral.commandValid && traj->lateral.turnNow)
                t.qualifier = "NOW";  // inside the standard-rate roll-out distance
            t.actionColor = o.command == Command::Ease ? pal().kAmber : pal().kCyan;
            t.glyph = Glyph::Arrow;
            t.arrowDir = which == Axis::Lateral ? Vec2{static_cast<float>(o.direction), 0.0f}
                                                : Vec2{0.0f, static_cast<float>(o.direction)};
            break;
    }

    switch (o.trend) {
        case Trend::Converging:
            t.trend = "CONVERGING";
            t.trendColor = pal().kGreen;
            t.trendArrow = -1;
            break;
        case Trend::Diverging:
            t.trend = "DIVERGING";
            t.trendColor = pal().kAmber;
            t.trendArrow = +1;
            break;
        case Trend::Steady: {
            const bool centred = std::fabs(o.positionDots) < 0.1;
            t.trend = centred ? "CENTERED" : "STEADY";
            t.trendColor = centred ? pal().kGreen : pal().kMuted;
            t.trendArrow = 0;
            break;
        }
    }

    // Prediction line: trajectory-based estimate if available, else rate-based.
    t.predictionColor = pal().kMuted;
    char buf[32];
    if (traj && trajectoryPrediction(*traj, which, t)) {
        // set by trajectoryPrediction
    } else if (o.interceptValid && o.trend == Trend::Converging) {
        std::snprintf(buf, sizeof(buf), "INTCPT %dS", static_cast<int>(std::lround(o.interceptSec)));
        t.prediction = buf;
    } else if (o.trend == Trend::Diverging) {
        std::snprintf(buf, sizeof(buf), "10S: %s %s", formatDots(o.predictedDots).c_str(),
                      unitText(o.predictedDots, which).c_str());
        t.prediction = buf;
    }

    t.litPips = std::clamp(static_cast<int>(std::ceil(std::fabs(o.positionDots) / 0.5 - 1e-9)), 0, axis.pipCount);
    if (t.litPips == 0) t.litPips = 1;  // first pip = "signal present, within half a dot"
    return t;
}

std::string distanceText(const InstrumentState& s) {
    char buf[48];
    switch (s.distanceKind) {
        case InstrumentState::DistanceKind::Dme:
            std::snprintf(buf, sizeof(buf), "DME %.1f NM", s.distanceNm);
            return buf;
        case InstrumentState::DistanceKind::Gps:
            if (s.activeFixId.empty()) std::snprintf(buf, sizeof(buf), "DIST %.1f NM", s.distanceNm);
            else std::snprintf(buf, sizeof(buf), "DIST %.1f NM TO %s", s.distanceNm, s.activeFixId.c_str());
            return buf;
        case InstrumentState::DistanceKind::None:
            break;
    }
    return {};
}

const char* modeTitle(InstrumentMode mode) {
    switch (mode) {
        case InstrumentMode::Heading: return "HSI OVERLAY - HEADING MODE";
        case InstrumentMode::Navigation: return "HSI OVERLAY - COURSE REFERENCE MODE";
        case InstrumentMode::RunwayAlignment: return "HSI OVERLAY - RUNWAY ALIGNMENT MODE";
    }
    return "";
}

const char* modeShort(InstrumentMode mode) {
    switch (mode) {
        case InstrumentMode::Heading: return "HDG MODE";
        case InstrumentMode::Navigation: return "CRS REF";
        case InstrumentMode::RunwayAlignment: return "RWY ALIGN";
    }
    return "";
}

}  // namespace navsight::render
