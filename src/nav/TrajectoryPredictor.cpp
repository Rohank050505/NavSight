#include "nav/TrajectoryPredictor.h"

#include <algorithm>
#include <cmath>

#include "nav/Angles.h"

namespace navsight::nav {
namespace {

constexpr double kFpmPerKt = 101.2686;  // 1 kt = 101.27 ft/min
constexpr double kOnTargetDots = 0.1;

double signOf(double v) { return v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0); }
double lowPassAlpha(double dt, double tau) { return tau <= 0.0 ? 1.0 : 1.0 - std::exp(-dt / tau); }

// Same hysteresis scheme as AxisGuidance::gradeStrength, with explicit thresholds.
Strength grade(double e, Strength prev, double moderate, double firm, double h) {
    switch (prev) {
        case Strength::Slight:
            if (e > firm + h) return Strength::Firm;
            if (e > moderate + h) return Strength::Moderate;
            return Strength::Slight;
        case Strength::Moderate:
            if (e > firm + h) return Strength::Firm;
            if (e < moderate - h) return Strength::Slight;
            return Strength::Moderate;
        case Strength::Firm:
            if (e < moderate - h) return Strength::Slight;
            if (e < firm - h) return Strength::Moderate;
            return Strength::Firm;
    }
    return Strength::Slight;
}

}  // namespace

const char* toString(Confidence c) {
    switch (c) {
        case Confidence::None: return "NONE";
        case Confidence::Prior: return "PRIOR";
        case Confidence::Calibrated: return "CALIBRATED";
        case Confidence::Measured: return "MEASURED";
    }
    return "?";
}

const char* toString(LateralOutcome o) {
    switch (o) {
        case LateralOutcome::Unknown: return "UNKNOWN";
        case LateralOutcome::Stabilizing: return "STABILIZING";
        case LateralOutcome::Parallel: return "PARALLEL";
        case LateralOutcome::Converging: return "CONVERGING";
        case LateralOutcome::Diverging: return "DIVERGING";
        case LateralOutcome::CrossingOvershoot: return "CROSSING+OVERSHOOT";
    }
    return "?";
}

double TrajectoryPredictor::requiredVerticalSpeedFpm(double groundspeedKt, double pathAngleDeg) {
    return -groundspeedKt * kFpmPerKt * std::tan(degToRad(pathAngleDeg));
}

double TrajectoryPredictor::standardRateRadiusNm(double groundspeedKt, double turnRateDegSec) {
    return (groundspeedKt / 3600.0) / degToRad(std::max(0.1, turnRateDegSec));
}

void TrajectoryPredictor::reset() {
    out_ = TrajectoryOutput{};
    haveTime_ = false;
    haveTrack_ = false;
    turnRate_ = 0.0;
    calHave_ = false;
    calGoodSec_ = 0.0;
    vCalHave_ = false;
    vCalGoodSec_ = 0.0;
}

const TrajectoryOutput& TrajectoryPredictor::update(const TrajectoryInput& in) {
    if (in.discontinuity) reset();
    const double dt = haveTime_ ? std::clamp(in.simTimeSec - lastTime_, 0.0, 1.0) : 0.0;
    haveTime_ = true;
    lastTime_ = in.simTimeSec;

    updateTurnRate(in, dt);
    predictLateral(in);
    predictVertical(in, dt);
    // Calibrate after predicting, using this frame's track error; the result
    // is used from the next frame on.
    updateScale(in, dt, out_.lateral.trackErrorDeg);
    return out_;
}

// Turn rate = filtered rate of change of ground track (what the path actually
// does), falling back to yaw rate when track is unusable (slow / unavailable).
void TrajectoryPredictor::updateTurnRate(const TrajectoryInput& in, double dt) {
    const bool trackUsable = in.trackValid && in.groundspeedValid && in.groundspeedKt >= p_.minGroundspeedKt;
    if (trackUsable) {
        if (haveTrack_ && dt > 1e-4) {
            const double raw = shortestDelta(lastTrack_, in.trackDegMag) / dt;
            turnRate_ += lowPassAlpha(dt, p_.turnRateFilterSec) * (raw - turnRate_);
        }
        haveTrack_ = true;
        lastTrack_ = in.trackDegMag;
    } else {
        haveTrack_ = false;
        turnRate_ = in.yawRateValid ? in.yawRateDegSec : 0.0;
    }
    turnRate_ = std::clamp(turnRate_, -p_.maxTurnRateDegSec, p_.maxTurnRateDegSec);
}

// Lateral scale. GPS reports nm/dot. Angular receivers (LOC/VOR) scale with
// distance: nm/dot = distance * tan(deg/dot); deg/dot starts at a nominal prior
// and is calibrated from straight-flight samples where
//   cross-track rate (nm/s) = GS * sin(track error)   [geometry, measured inputs]
//   deviation rate (dots/s)                          [measured, filtered]
// so nm/dot = ratio, and deg/dot = atan(nm/dot / distance).
void TrajectoryPredictor::updateScale(const TrajectoryInput& in, double dt, double trackErrorDeg) {
    if (!in.lateralPredictive || !in.trackValid || !in.groundspeedValid || in.distanceNm < 0.0 ||
        in.priorDegPerDot <= 0.0 || in.gpsNmPerDot > 0.0 || dt <= 0.0)
        return;

    // Angular rate of the aircraft as seen from the antenna, from geometry:
    //   theta = atan(x / d),  d/dt theta = (x' d - x d') / (d^2 + x^2)
    // where x' = GS sin(track error) and d' = -/+ GS cos(track error)
    // (approaching / receding). x itself comes from the current estimate.
    const double dist = std::max(0.2, in.distanceNm);
    const double gsNmS = in.groundspeedKt / 3600.0;
    const double degPerDotNow = calHave_ ? std::exp(calLogDegPerDot_) : in.priorDegPerDot;
    const double x = dist * std::tan(degToRad(degPerDotNow * in.lateralDots));
    const double xDot = gsNmS * std::sin(degToRad(trackErrorDeg));
    const double dDot = (in.stationAhead ? -1.0 : 1.0) * gsNmS * std::cos(degToRad(trackErrorDeg));
    const double thetaDotDeg = radToDeg((xDot * dist - x * dDot) / (dist * dist + x * x));
    const double rate = in.lateralRateDotsSec;
    const bool good = std::fabs(trackErrorDeg) >= p_.calMinTrackErrorDeg &&
                      std::fabs(turnRate_) <= p_.calMaxTurnRateDegSec && std::fabs(rate) >= p_.calMinRateDotsSec &&
                      thetaDotDeg * rate > 0.0 && in.groundspeedKt >= p_.minGroundspeedKt;
    if (!good) return;

    const double degInst = thetaDotDeg / rate;
    if (degInst < in.priorDegPerDot / 4.0 || degInst > in.priorDegPerDot * 4.0) return;  // implausible sample
    const double logv = std::log(degInst);
    if (!calHave_) {
        calLogDegPerDot_ = logv;
        calHave_ = true;
    } else {
        calLogDegPerDot_ += lowPassAlpha(dt, p_.calFilterSec) * (logv - calLogDegPerDot_);
    }
    calGoodSec_ += dt;
}

void TrajectoryPredictor::predictLateral(const TrajectoryInput& in) {
    const LateralPrediction prev = out_.lateral;
    LateralPrediction& L = out_.lateral;
    L = LateralPrediction{};

    if (!in.courseValid || !in.trackValid || !in.groundspeedValid || in.groundspeedKt < p_.minGroundspeedKt) return;
    const double tke = wrap180(in.trackDegMag - in.courseDegMag);
    L.trackErrorDeg = tke;
    L.turnRateDegSec = turnRate_;
    if (!in.lateralPredictive) return;

    // Scale and confidence.
    if (in.gpsNmPerDot > 0.0) {
        L.nmPerDot = in.gpsNmPerDot;
        L.confidence = Confidence::Measured;
    } else if (in.distanceNm >= 0.0 && in.priorDegPerDot > 0.0) {
        const bool calibrated = calHave_ && calGoodSec_ >= p_.calRequiredSec;
        L.degPerDot = calibrated ? std::exp(calLogDegPerDot_) : in.priorDegPerDot;
        L.nmPerDot = std::max(0.2, in.distanceNm) * std::tan(degToRad(L.degPerDot));
        L.confidence = calibrated ? Confidence::Calibrated : Confidence::Prior;
    } else {
        return;  // no way to relate dots to distance
    }
    if (L.nmPerDot <= 1e-4) return;
    L.valid = true;

    const double k = L.nmPerDot;  // local scale (at the current distance for angular sources)
    const double gsNmS = in.groundspeedKt / 3600.0;
    const double dots0 = in.lateralDots;
    const bool angular = L.confidence == Confidence::Prior || L.confidence == Confidence::Calibrated;
    const double dist0 = std::max(0.2, in.distanceNm);
    // Cross-track distance <-> dots. Angular receivers: theta = atan(x / d),
    // with d shrinking as the aircraft approaches the antenna.
    auto toDots = [&](double xNm, double alongNm) {
        if (!angular) return xNm / k;
        const double d = std::max(0.2, dist0 + (in.stationAhead ? -alongNm : alongNm));
        return radToDeg(std::atan(xNm / d)) / L.degPerDot;
    };
    const double x0 = angular ? dist0 * std::tan(degToRad(L.degPerDot * dots0)) : dots0 * k;
    const double sign0 = std::fabs(dots0) < 0.05 ? 0.0 : signOf(dots0);

    // Projection: constant groundspeed and turn rate (midpoint integration).
    double x = x0, along = 0.0, peak = 0.0;
    bool crossed = false;
    L.path[0] = {0.0f, 0.0f, static_cast<float>(dots0)};
    for (int i = 1; i <= LateralPrediction::kSteps; ++i) {
        const double t = i * LateralPrediction::kStepSec;
        const double a = degToRad(tke + turnRate_ * (t - 0.5 * LateralPrediction::kStepSec));
        x += gsNmS * std::sin(a) * LateralPrediction::kStepSec;
        along += gsNmS * std::cos(a) * LateralPrediction::kStepSec;
        const double dots = toDots(x, along);
        L.path[i] = {static_cast<float>(t), static_cast<float>(along), static_cast<float>(dots)};
        if (sign0 != 0.0 && !crossed && dots * sign0 <= 0.0) {
            crossed = true;
            const double prevDots = L.path[i - 1].crossDots;
            const double frac = prevDots / (prevDots - dots);
            L.crossValid = true;
            L.crossSec = (i - 1 + frac) * LateralPrediction::kStepSec;
        }
        if (crossed) peak = std::max(peak, -dots * sign0);
    }
    L.predicted10Dots = L.path[20].crossDots;
    L.predicted20Dots = L.path[LateralPrediction::kSteps].crossDots;
    L.overshootDots = crossed ? peak : 0.0;

    // Outcome.
    const double closingNmS = -signOf(x0) * gsNmS * std::sin(degToRad(tke));
    const double closeThresh = gsNmS * std::sin(degToRad(2.0));
    if (std::fabs(dots0) < 0.2 && std::fabs(tke) < 3.0) L.outcome = LateralOutcome::Stabilizing;
    else if (crossed && L.overshootDots > 0.5) L.outcome = LateralOutcome::CrossingOvershoot;
    else if (closingNmS > closeThresh) L.outcome = LateralOutcome::Converging;
    else if (closingNmS < -closeThresh) L.outcome = LateralOutcome::Diverging;
    else L.outcome = LateralOutcome::Parallel;

    // Standard-rate roll-out: lateral distance consumed turning onto the course.
    const double absTke = std::fabs(tke);
    if (sign0 != 0.0 && closingNmS > 0.0 && absTke >= 10.0) {
        const double radius = standardRateRadiusNm(in.groundspeedKt, std::max(p_.standardRateDegSec, std::fabs(turnRate_)));
        L.leadNm = radius * (1.0 - std::cos(degToRad(absTke)));
        const bool rollingOut = turnRate_ * signOf(tke) <= -2.0;  // already turning toward the course heading
        if (std::fabs(x0) <= L.leadNm && !rollingOut) {
            L.turnNow = true;
            L.rolloutOvershootDots = (L.leadNm - std::fabs(x0)) / k;
        } else if (std::fabs(x0) > L.leadNm) {
            L.turnInValid = true;
            L.turnInSec = (std::fabs(x0) - L.leadNm) / closingNmS;
        }
    }

    // Track-based correction - only with a trustworthy scale.
    if (L.confidence != Confidence::Calibrated && L.confidence != Confidence::Measured) return;
    L.commandValid = true;
    L.desiredTrackErrorDeg = -signOf(x0) * std::min(p_.maxInterceptDeg, p_.interceptGainDegPerNm * std::fabs(x0));
    if (L.turnNow) {
        L.command = Command::Ease;  // turn away from the intercept, toward the course heading
        L.direction = tke > 0.0 ? -1 : +1;
        L.strength = Strength::Firm;
        return;
    }
    if (std::fabs(dots0) < kOnTargetDots && absTke < 3.0) {
        L.command = Command::OnTarget;
        return;
    }
    const double err = tke - L.desiredTrackErrorDeg;
    const bool wasHolding = prev.command == Command::Hold || prev.command == Command::OnTarget;
    const double holdLimit = p_.holdBandDeg + (wasHolding ? p_.angleHysteresisDeg : -p_.angleHysteresisDeg);
    // An intercept toward the course that is at least as steep as desired (up
    // to max(desired + 15, 45) deg) is fine: when to roll out is the job of the
    // standard-rate lead geometry above (TURN IN / TURN NOW), not of this law.
    const bool toward = sign0 != 0.0 && signOf(tke) == -sign0;
    const double absDesired = std::fabs(L.desiredTrackErrorDeg);
    const double steepLimit = std::max(absDesired + 15.0, 45.0);
    if (std::fabs(err) < holdLimit ||
        (toward && absTke >= absDesired - holdLimit && absTke <= steepLimit + (wasHolding ? p_.angleHysteresisDeg : 0.0))) {
        L.command = Command::Hold;
        return;
    }
    L.direction = err > 0.0 ? -1 : +1;
    const bool turnTowardCourse = std::fabs(dots0) < kOnTargetDots || L.direction == -static_cast<int>(signOf(x0));
    L.command = turnTowardCourse ? Command::Correct : Command::Ease;
    L.strength = grade(std::fabs(err), prev.commandValid ? prev.strength : Strength::Slight, p_.moderateDeg, p_.firmDeg,
                       p_.angleHysteresisDeg);
}

void TrajectoryPredictor::predictVertical(const TrajectoryInput& in, double dt) {
    const VerticalPrediction prev = out_.vertical;
    VerticalPrediction& V = out_.vertical;
    V = VerticalPrediction{};
    if (!in.verticalPredictive || !in.verticalSpeedValid || !in.groundspeedValid ||
        in.groundspeedKt < p_.minGroundspeedKt)
        return;

    V.valid = true;
    V.pathAngleDeg = std::clamp(in.pathAngleDeg, 1.0, 8.0);
    V.pathAngleAssumed = in.pathAngleAssumed;
    V.requiredVsFpm = requiredVerticalSpeedFpm(in.groundspeedKt, V.pathAngleDeg);
    V.vsErrorFpm = in.verticalSpeedFpm - V.requiredVsFpm;
    const double pos = in.verticalDots;

    // Learn how fast the diamond moves per fpm of VS error (scale changes with
    // distance, so this is a slowly-updated local gain).
    const double rate = in.verticalRateDotsSec;
    if (dt > 0.0 && std::fabs(V.vsErrorFpm) >= 150.0 && std::fabs(rate) >= 0.005 && V.vsErrorFpm * rate > 0.0) {
        const double inst = rate / V.vsErrorFpm;
        if (!vCalHave_) {
            vCalGain_ = inst;
            vCalHave_ = true;
        } else {
            vCalGain_ += lowPassAlpha(dt, p_.calFilterSec) * (inst - vCalGain_);
        }
        vCalGoodSec_ += dt;
    }
    V.rateCalibrated = vCalHave_ && vCalGoodSec_ >= 3.0;

    const double predRate = V.rateCalibrated ? vCalGain_ * V.vsErrorFpm : rate;
    V.predicted10Dots = pos + predRate * 10.0;
    const double closing = -signOf(pos) * predRate;
    if (std::fabs(pos) >= kOnTargetDots && closing > 1e-3) {
        const double tc = std::fabs(pos) / closing;
        if (tc <= 20.0) {
            V.crossValid = true;
            V.crossSec = tc;
            // Constant-VS assumption: the VS error at the crossing is today's.
            V.overshoot = std::fabs(V.vsErrorFpm) >= p_.vsOvershootFpm && -signOf(pos) * V.vsErrorFpm > 0.0;
        }
    }

    // Command: target VS = required VS plus a bounded correction toward the path.
    V.commandValid = true;
    V.targetVsFpm = V.requiredVsFpm - signOf(pos) * std::min(p_.verticalMaxCorrectionFpm,
                                                               p_.verticalGainFpmPerDot * std::fabs(pos));
    if (std::fabs(pos) < kOnTargetDots && std::fabs(V.vsErrorFpm) < 150.0) {
        V.command = Command::OnTarget;
        return;
    }
    const double err = in.verticalSpeedFpm - V.targetVsFpm;
    const bool wasHolding = prev.command == Command::Hold || prev.command == Command::OnTarget;
    const double holdLimit = p_.vsHoldBandFpm + (wasHolding ? p_.vsHysteresisFpm : -p_.vsHysteresisFpm);
    if (std::fabs(err) < holdLimit) {
        V.command = Command::Hold;
        return;
    }
    V.direction = err > 0.0 ? -1 : +1;
    const bool toward = std::fabs(pos) < kOnTargetDots || V.direction == -static_cast<int>(signOf(pos));
    V.command = toward ? Command::Correct : Command::Ease;
    V.strength = grade(std::fabs(err), prev.commandValid ? prev.strength : Strength::Slight, p_.vsModerateFpm,
                       p_.vsFirmFpm, p_.vsHysteresisFpm);
}

}  // namespace navsight::nav
