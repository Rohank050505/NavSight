#include "nav/AxisGuidance.h"

#include <cmath>

namespace navsight::nav {
namespace {

constexpr double kMaxGapSec = 1.0;  // longer gaps restart the filters

double signOf(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }

double lowPassAlpha(double dt, double tau) { return tau <= 0.0 ? 1.0 : 1.0 - std::exp(-dt / tau); }

}  // namespace

void AxisGuidance::reset() {
    out_ = AxisOutput{};
    haveHistory_ = false;
    filteredError_ = 0.0;
}

Strength AxisGuidance::gradeStrength(double e) const {
    const double h = p_.hysteresisDots;
    switch (out_.strength) {
        case Strength::Slight:
            if (e > p_.firmDots + h) return Strength::Firm;
            if (e > p_.moderateDots + h) return Strength::Moderate;
            return Strength::Slight;
        case Strength::Moderate:
            if (e > p_.firmDots + h) return Strength::Firm;
            if (e < p_.moderateDots - h) return Strength::Slight;
            return Strength::Moderate;
        case Strength::Firm:
            if (e < p_.moderateDots - h) return Strength::Slight;
            if (e < p_.firmDots - h) return Strength::Moderate;
            return Strength::Firm;
    }
    return Strength::Slight;
}

const AxisOutput& AxisGuidance::update(bool valid, double pos, double t) {
    if (!valid || !std::isfinite(pos)) {
        reset();
        return out_;
    }

    const double dt = haveHistory_ ? t - lastTime_ : 0.0;
    if (!haveHistory_ || dt > kMaxGapSec || dt < 0.0) {
        // (Re)acquired: no rate information yet.
        out_.rateDotsPerSec = 0.0;
        filteredError_ = 0.0;
        out_.trend = Trend::Steady;
        out_.strength = Strength::Slight;
        out_.command = Command::None;
    } else if (dt > 1e-4) {
        const double rawRate = (pos - lastPos_) / dt;
        out_.rateDotsPerSec += lowPassAlpha(dt, p_.rateFilterSec) * (rawRate - out_.rateDotsPerSec);
    }  // dt == 0 (paused / same frame): keep previous rate

    const double rate = out_.rateDotsPerSec;
    const double absPos = std::fabs(pos);
    out_.valid = true;
    out_.positionDots = pos;
    out_.predictedDots = pos + rate * p_.predictionHorizonSec;

    // Closure: positive when moving toward the centre.
    const double closing = -signOf(pos) * rate;

    // Trend, with hysteresis so it doesn't chatter around the threshold.
    const double enter = p_.steadyRateDotsPerSec, leave = p_.steadyRateDotsPerSec * 0.5;
    if (absPos < p_.onTargetDots) {
        out_.trend = Trend::Steady;
    } else if (out_.trend == Trend::Converging) {
        if (closing < leave) out_.trend = closing < -enter ? Trend::Diverging : Trend::Steady;
    } else if (out_.trend == Trend::Diverging) {
        if (closing > -leave) out_.trend = closing > enter ? Trend::Converging : Trend::Steady;
    } else {
        if (closing > enter) out_.trend = Trend::Converging;
        else if (closing < -enter) out_.trend = Trend::Diverging;
    }

    out_.interceptValid = false;
    if (absPos >= p_.onTargetDots && closing > 1e-3) {
        const double tti = absPos / closing;
        if (tti <= p_.maxInterceptSec) {
            out_.interceptValid = true;
            out_.interceptSec = tti;
        }
    }

    // Command law (see header).
    const double tau = p_.closureTimeSec;
    const double error = (-pos / tau - rate) * tau;
    if (dt > 1e-4 && haveHistory_) filteredError_ += lowPassAlpha(dt, p_.commandFilterSec) * (error - filteredError_);
    else if (!haveHistory_) filteredError_ = error;
    const double e = filteredError_;
    const double absE = std::fabs(e);

    const bool wasHolding = out_.command == Command::Hold || out_.command == Command::OnTarget;
    const double holdLimit = p_.holdBandDots + (wasHolding ? p_.hysteresisDots : -p_.hysteresisDots);

    if (absPos < p_.onTargetDots && std::fabs(rate) < p_.steadyRateDotsPerSec * 3.0) {
        out_.command = Command::OnTarget;
        out_.direction = 0;
        out_.strength = Strength::Slight;
    } else if (absE < holdLimit) {
        out_.command = Command::Hold;
        out_.direction = 0;
        out_.strength = Strength::Slight;
    } else {
        out_.direction = e > 0.0 ? +1 : -1;
        const bool towardPath = absPos < p_.onTargetDots || out_.direction == -static_cast<int>(signOf(pos));
        out_.command = towardPath ? Command::Correct : Command::Ease;
        out_.strength = gradeStrength(absE);
    }

    haveHistory_ = true;
    lastPos_ = pos;
    lastTime_ = t;
    return out_;
}

}  // namespace navsight::nav
