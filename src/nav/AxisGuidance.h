#pragma once

// Predictive guidance for one deviation axis (lateral or vertical).
//
// Input: the aircraft's position relative to the target path, in dots, over
// time. Output: a filtered rate of change, a trend, a predicted position, a
// time-to-intercept, and a correction command.
//
// The command compares the actual closure rate with an "ideal" closure that
// would remove the deviation exponentially over `closureTimeSec`:
//
//     desiredRate = -position / closureTimeSec
//     error       = (desiredRate - rate) * closureTimeSec     (in dots)
//
// |error| small     -> HOLD  (current correction is right; leave it)
// error toward path -> CORRECT (turn/climb toward the path), graded by size
// error away        -> EASE    (closing too fast; overshoot predicted)
//
// The error is low-pass filtered and graded with hysteresis so the advice
// doesn't flicker on noisy data.
namespace navsight::nav {

enum class Trend { Steady, Converging, Diverging };

enum class Command {
    None,      // axis invalid
    OnTarget,  // centred and stable
    Hold,      // keep the current correction
    Correct,   // move toward the path
    Ease       // reduce the correction (closing too fast)
};

enum class Strength { Slight = 1, Moderate = 2, Firm = 3 };

struct AxisOutput {
    bool valid = false;
    double positionDots = 0.0;      // + = right of course / above glidepath
    double rateDotsPerSec = 0.0;    // filtered d(position)/dt
    double predictedDots = 0.0;     // position after the prediction horizon
    Trend trend = Trend::Steady;
    bool interceptValid = false;
    double interceptSec = 0.0;      // time until centred at current closure
    Command command = Command::None;
    int direction = 0;              // -1 = left/down, +1 = right/up (where to move)
    Strength strength = Strength::Slight;
};

class AxisGuidance {
public:
    struct Params {
        double closureTimeSec = 20.0;     // ideal time constant to centre
        double rateFilterSec = 1.0;       // derivative low-pass
        double commandFilterSec = 0.6;    // error low-pass
        double predictionHorizonSec = 10.0;
        double onTargetDots = 0.1;
        double steadyRateDotsPerSec = 0.01;
        double holdBandDots = 0.15;       // |error| below this -> HOLD
        double moderateDots = 0.5;        // |error| above this -> MODERATE
        double firmDots = 1.0;            // |error| above this -> FIRM
        double hysteresisDots = 0.05;
        double maxInterceptSec = 120.0;
    };

    explicit AxisGuidance(Params p) : p_(p) {}

    void reset();
    // `position` in dots (+ right/up). `valid` false clears history.
    const AxisOutput& update(bool valid, double position, double simTimeSec);
    const AxisOutput& output() const { return out_; }

private:
    Strength gradeStrength(double absError) const;

    Params p_;
    AxisOutput out_;
    bool haveHistory_ = false;
    double lastPos_ = 0.0;
    double lastTime_ = 0.0;
    double filteredError_ = 0.0;
};

}  // namespace navsight::nav
