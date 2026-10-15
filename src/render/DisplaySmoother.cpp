#include "render/DisplaySmoother.h"

#include <algorithm>
#include <cmath>

#include "nav/Angles.h"

namespace navsight::render {

double DisplaySmoother::AngleFilt::step(double deg, double alpha) {
    // Follow the shortest way round so 359 -> 1 doesn't sweep the dial.
    unwrapped += nav::shortestDelta(nav::wrap360(unwrapped), deg);
    return nav::wrap360(f.step(unwrapped, alpha));
}

void DisplaySmoother::fadeCue(DisplayFx::Cue& cue, const GuidanceAxis& axis, double dt) const {
    const nav::AxisOutput& o = axis.out;
    const bool want = axis.predictive && (o.command == nav::Command::Correct || o.command == nav::Command::Ease);
    const float step = static_cast<float>(dt / p_.fadeSec);
    if (!axis.predictive) {  // invalid: gone immediately
        cue = DisplayFx::Cue{};
        return;
    }
    if (want) {
        cue.present = true;
        cue.direction = o.direction;
        cue.strength = o.strength;
        cue.ease = o.command == nav::Command::Ease;
        cue.alpha = std::min(1.0f, cue.alpha + step);
    } else if (cue.present) {
        cue.alpha = std::max(0.0f, cue.alpha - step);  // fade out the last cue
        if (cue.alpha <= 0.0f) cue.present = false;
    }
}

InstrumentState DisplaySmoother::apply(const InstrumentState& raw, double dt, DisplayFx& fx) {
    InstrumentState s = raw;
    dt = std::clamp(dt, 0.0, 1.0);
    const std::string identity = raw.sourceName + "|" + raw.approachLabel;
    const bool snapAll = !have_ || dt > 0.5 || identity != identity_;
    const double alpha = 1.0 - std::exp(-dt / (0.5 * p_.motionTauSec));  // per stage
    if (snapAll) {  // first frame / new source / long gap: no easing, no fades carried over
        fx_ = DisplayFx{};
        fx_.trackAlpha = 0.0f;
        trackShown_ = false;
    }

    auto smooth = [&](Filt& f, double& value, bool valid) {
        if (!valid || snapAll || std::fabs(value - f.b) > p_.snapDots) f.snap(value);
        else value = f.step(value, alpha);
    };
    smooth(latPos_, s.lateral.out.positionDots, raw.lateral.out.valid);
    smooth(latPred_, s.lateral.out.predictedDots, raw.lateral.out.valid);
    smooth(vertPos_, s.vertical.out.positionDots, raw.vertical.out.valid);
    smooth(vertPred_, s.vertical.out.predictedDots, raw.vertical.out.valid);

    auto smoothAngle = [&](AngleFilt& f, double& value, bool valid) {
        if (!valid || snapAll) f.snap(value);
        else value = f.step(value, alpha);
    };
    smoothAngle(course_, s.courseDegMag, raw.courseValid);
    smoothAngle(bearing_, s.bearingDegMag, raw.bearingValid);

    // Projected track: ease each point; fade in when it first appears.
    auto& path = s.trajectory.lateral.path;
    const bool trackNow = raw.trajectory.lateral.valid && raw.lateral.predictive;
    for (std::size_t i = 0; i < path.size(); ++i) {
        double c = path[i].crossDots, a = path[i].alongNm;
        const bool snapPt = snapAll || !trackShown_ || !trackNow;
        smooth(pathCross_[i], c, !snapPt);
        smooth(pathAlong_[i], a, !snapPt);
        path[i].crossDots = static_cast<float>(c);
        path[i].alongNm = static_cast<float>(a);
    }
    if (!trackNow) fx_.trackAlpha = 0.0f;
    else fx_.trackAlpha = std::min(1.0f, (trackShown_ ? fx_.trackAlpha : 0.0f) + static_cast<float>(dt / p_.fadeSec));
    trackShown_ = trackNow;

    // Cue fades.
    fadeCue(fx_.lateralChevrons, raw.lateral, dt);
    fadeCue(fx_.verticalChevrons, raw.vertical, dt);
    const float step = static_cast<float>(dt / p_.fadeSec);
    fx_.alignedAlpha = raw.aligned ? std::min(1.0f, fx_.alignedAlpha + step) : std::max(0.0f, fx_.alignedAlpha - step);
    if (raw.mode != InstrumentMode::RunwayAlignment) fx_.alignedAlpha = 0.0f;  // left the approach: gone

    have_ = true;
    identity_ = identity;
    fx = fx_;
    return s;
}

}  // namespace navsight::render
