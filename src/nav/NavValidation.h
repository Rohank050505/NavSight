#pragma once

#include <string>

#include "data/SimSnapshot.h"

// Navigation source validation: decides, per axis and per frame, whether the
// measured deviation can be shown and whether calculated (predictive) cues
// built on it may be shown.
//
// Measured vs. calculated is the key distinction:
//   measuredValid - the receiver reports a usable deviation; draw the needle,
//                   diamond and the number in the panel.
//   predictive    - trend, prediction, chevrons and correction commands are
//                   meaningful. Requires measuredValid AND a stable, unsaturated
//                   signal AND (vertical) lateral establishment AND enough
//                   groundspeed.
//
// Pure C++ (no X-Plane calls) so every rule is unit-tested.
namespace navsight::nav {

enum class SignalStatus {
    Valid,           // usable; predictive cues allowed (subject to groundspeed)
    NoSignal,        // receiver reports no deviation, or the value is implausible
    Flagged,         // receiver reports a failure flag (e.g. VOR NAV flag)
    Acquiring,       // just (re)acquired, source changed, or the value jumped:
                     // measured value shown, prediction held off while filters settle
    FullScale,       // needle at/over full scale: rate of change is meaningless
    NotEstablished   // vertical only: glidepath received but not established
                     // laterally, where glideslope signals are unreliable
};

const char* toString(SignalStatus s);

struct AxisValidity {
    SignalStatus status = SignalStatus::NoSignal;
    bool measuredValid = false;
    bool predictive = false;
};

struct NavValidity {
    AxisValidity lateral;
    AxisValidity vertical;
    bool lowGroundspeed = false;  // predictive cues suppressed (taxi, hover, slow flight)
    bool discontinuity = false;   // this frame: predictors must drop their history
    std::string discontinuityReason;
};

class NavValidator {
public:
    struct Params {
        double settleSec = 1.5;              // hold-off after (re)acquisition / jumps
        double fullScaleDots = 2.4;          // |deflection| at or above = full scale
        double maxPlausibleDots = 2.75;      // above = not a real reading (e.g. GPS -3 sentinel)
        double jumpDots = 1.0;               // per-frame step treated as a discontinuity
        double jumpWindowSec = 0.25;         // ...only between frames closer than this
        double establishedLateralDots = 2.0; // vertical needs |lateral| below this
        double minGroundspeedKt = 30.0;      // below: no predictive cues
    };

    NavValidator() = default;
    explicit NavValidator(Params p) : p_(p) {}

    const NavValidity& update(const SimSnapshot& snap);
    const NavValidity& validity() const { return v_; }
    void reset();

private:
    struct AxisTrack {
        bool hadSignal = false;
        double lastDots = 0.0;
        double acquiringUntil = -1.0;
    };

    bool stepAxis(AxisTrack& track, bool rawOk, double dots, double t, bool forceAcquire);
    AxisValidity classify(const AxisTrack& track, bool rawOk, double dots, double t) const;

    Params p_;
    NavValidity v_;
    AxisTrack lat_, vert_;
    bool haveIdentity_ = false;
    NavSource lastSource_ = NavSource::Nav1;
    int lastFrequency_ = 0;
    std::string lastLabel_;
    double lastTime_ = 0.0;
};

}  // namespace navsight::nav
