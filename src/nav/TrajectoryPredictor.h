#pragma once

#include <array>

#include "nav/AxisGuidance.h"

// Trajectory-based prediction: where will the aircraft be relative to the
// procedure if it keeps doing what it is doing?
//
// Unlike AxisGuidance (which extrapolates the *deviation's* rate of change),
// this works from the aircraft's motion: ground track vs selected course,
// groundspeed, turn rate and vertical speed. That lets NavSight see a
// convergence, crossing or overshoot before the needle moves.
//
// Everything here is ESTIMATED. Each output carries a confidence level, and
// callers must not present low-confidence estimates as instructions.
//
// Assumptions (documented in README "Prediction model"):
//  - groundspeed and turn rate stay constant over the 20 s horizon;
//  - a roll-out onto the course would be flown at standard rate (3 deg/s);
//  - lateral scale: GPS reports nm/dot directly; for LOC/VOR the angular
//    width per dot starts from a prior and is self-calibrated in flight from
//    (groundspeed * sin(track error)) / (measured dots per second);
//  - vertical: constant vertical speed; the required descent rate follows
//    from groundspeed and the glidepath angle.
//
// Pure C++ (no X-Plane calls).
namespace navsight::nav {

enum class Confidence {
    None,        // inputs missing: no trajectory estimate
    Prior,       // scale from a nominal receiver width only: draw dimmed, no commands
    Calibrated,  // scale self-calibrated in flight
    Measured     // scale reported by the receiver (GPS nm/dot)
};
const char* toString(Confidence c);

enum class LateralOutcome { Unknown, Stabilizing, Parallel, Converging, Diverging, CrossingOvershoot };
const char* toString(LateralOutcome o);

struct PathPoint {
    float tSec = 0.0f;
    float alongNm = 0.0f;    // along-course distance from now
    float crossDots = 0.0f;  // aircraft position relative to the course (+ right)
};

struct LateralPrediction {
    bool valid = false;
    Confidence confidence = Confidence::None;
    double trackErrorDeg = 0.0;    // track - course, + = tracking right of the course
    double turnRateDegSec = 0.0;   // + = turning right
    double nmPerDot = 0.0;
    double degPerDot = 0.0;        // angular sources only

    static constexpr int kSteps = 40;   // 20 s at 0.5 s
    static constexpr double kStepSec = 0.5;
    std::array<PathPoint, kSteps + 1> path{};
    double predicted10Dots = 0.0;
    double predicted20Dots = 0.0;

    bool crossValid = false;       // projected path crosses the course within the horizon
    double crossSec = 0.0;
    double overshootDots = 0.0;    // peak excursion beyond the course after crossing

    // Standard-rate roll-out geometry.
    double leadNm = 0.0;           // lateral distance a roll-out onto the course needs
    bool turnInValid = false;
    double turnInSec = 0.0;        // time until the roll-out must begin
    bool turnNow = false;          // already inside the lead distance: overshoot unless turning now
    double rolloutOvershootDots = 0.0;

    LateralOutcome outcome = LateralOutcome::Unknown;

    // Track-based correction (only when confidence >= Calibrated).
    bool commandValid = false;
    Command command = Command::None;
    int direction = 0;             // -1 turn left, +1 turn right
    Strength strength = Strength::Slight;
    double desiredTrackErrorDeg = 0.0;
};

struct VerticalPrediction {
    bool valid = false;
    double pathAngleDeg = 0.0;
    bool pathAngleAssumed = true;  // true = nominal 3.0 deg, not from data
    double requiredVsFpm = 0.0;    // descent rate that tracks the path at this groundspeed
    double vsErrorFpm = 0.0;       // actual - required (+ = shallower/climbing vs path)
    bool rateCalibrated = false;   // dots-per-second per fpm learned in flight

    double predicted10Dots = 0.0;
    bool crossValid = false;
    double crossSec = 0.0;
    bool overshoot = false;        // will cross the path with a large VS error

    bool commandValid = false;
    Command command = Command::None;
    int direction = 0;             // -1 descend more, +1 descend less / climb
    Strength strength = Strength::Slight;
    double targetVsFpm = 0.0;
};

struct TrajectoryOutput {
    LateralPrediction lateral;
    VerticalPrediction vertical;
};

struct TrajectoryInput {
    double simTimeSec = 0.0;
    bool discontinuity = false;    // from NavValidator: drop all history

    // Lateral (validated measurement + filtered rate from AxisGuidance)
    bool lateralPredictive = false;
    double lateralDots = 0.0;      // + = right of course
    double lateralRateDotsSec = 0.0;
    bool courseValid = false;
    double courseDegMag = 0.0;
    bool trackValid = false;
    double trackDegMag = 0.0;
    bool groundspeedValid = false;
    double groundspeedKt = 0.0;
    bool yawRateValid = false;
    double yawRateDegSec = 0.0;

    // Lateral scale sources (any may be absent)
    double gpsNmPerDot = 0.0;      // > 0: GPS-reported sensitivity
    double distanceNm = -1.0;      // >= 0: distance to the LOC/VOR antenna
    bool stationAhead = true;      // flying toward the antenna (TO); false = FROM
    double priorDegPerDot = 0.0;   // > 0: nominal angular width per dot

    // Vertical
    bool verticalPredictive = false;
    double verticalDots = 0.0;     // + = above path
    double verticalRateDotsSec = 0.0;
    bool verticalSpeedValid = false;
    double verticalSpeedFpm = 0.0;
    double pathAngleDeg = 3.0;
    bool pathAngleAssumed = true;
};

class TrajectoryPredictor {
public:
    struct Params {
        double standardRateDegSec = 3.0;
        double maxTurnRateDegSec = 6.0;
        double turnRateFilterSec = 1.0;
        double minGroundspeedKt = 40.0;
        // Lateral command law: desired intercept = -sign(x) * min(maxIntercept, gain * |x_nm|)
        double maxInterceptDeg = 30.0;
        double interceptGainDegPerNm = 30.0;
        double holdBandDeg = 4.0;
        double moderateDeg = 10.0;
        double firmDeg = 20.0;
        double angleHysteresisDeg = 1.5;
        double turnInWarnSec = 15.0;
        // Self-calibration
        double calMinTrackErrorDeg = 5.0;
        double calMaxTurnRateDegSec = 1.0;
        double calMinRateDotsSec = 0.01;
        double calFilterSec = 4.0;
        double calRequiredSec = 4.0;
        // Vertical
        double verticalGainFpmPerDot = 150.0;
        double verticalMaxCorrectionFpm = 400.0;
        double vsHoldBandFpm = 120.0;
        double vsModerateFpm = 250.0;
        double vsFirmFpm = 500.0;
        double vsHysteresisFpm = 30.0;
        double vsOvershootFpm = 250.0;
    };

    TrajectoryPredictor() = default;
    explicit TrajectoryPredictor(Params p) : p_(p) {}

    const TrajectoryOutput& update(const TrajectoryInput& in);
    const TrajectoryOutput& output() const { return out_; }
    void reset();

    // Exposed for tests / documentation.
    static double requiredVerticalSpeedFpm(double groundspeedKt, double pathAngleDeg);
    static double standardRateRadiusNm(double groundspeedKt, double turnRateDegSec);

private:
    void updateTurnRate(const TrajectoryInput& in, double dt);
    void updateScale(const TrajectoryInput& in, double dt, double trackErrorDeg);
    void predictLateral(const TrajectoryInput& in);
    void predictVertical(const TrajectoryInput& in, double dt);

    Params p_;
    TrajectoryOutput out_;
    bool haveTime_ = false;
    double lastTime_ = 0.0;

    bool haveTrack_ = false;
    double lastTrack_ = 0.0;
    double turnRate_ = 0.0;

    // Angular-scale calibration (deg per dot), log-space filtered.
    bool calHave_ = false;
    double calLogDegPerDot_ = 0.0;
    double calGoodSec_ = 0.0;

    // Vertical: dots/s per fpm of VS error.
    bool vCalHave_ = false;
    double vCalGain_ = 0.0;
    double vCalGoodSec_ = 0.0;
};

}  // namespace navsight::nav
