#include "nav/NavValidation.h"

#include <cmath>
#include <cstdio>

namespace navsight::nav {

const char* toString(SignalStatus s) {
    switch (s) {
        case SignalStatus::Valid: return "VALID";
        case SignalStatus::NoSignal: return "NO SIGNAL";
        case SignalStatus::Flagged: return "FLAGGED";
        case SignalStatus::Acquiring: return "ACQUIRING";
        case SignalStatus::FullScale: return "FULL SCALE";
        case SignalStatus::NotEstablished: return "NOT ESTABLISHED";
    }
    return "?";
}

void NavValidator::reset() {
    v_ = NavValidity{};
    lat_ = AxisTrack{};
    vert_ = AxisTrack{};
    haveIdentity_ = false;
    lastTime_ = 0.0;
}

// Updates the axis history. Returns true if the value jumped discontinuously.
bool NavValidator::stepAxis(AxisTrack& track, bool rawOk, double dots, double t, bool forceAcquire) {
    const double dt = t - lastTime_;
    const bool jumped = track.hadSignal && rawOk && dt > 0.0 && dt <= p_.jumpWindowSec &&
                        std::fabs(dots - track.lastDots) > p_.jumpDots;
    if (rawOk && (!track.hadSignal || forceAcquire || jumped)) track.acquiringUntil = t + p_.settleSec;
    track.hadSignal = rawOk;
    if (rawOk) track.lastDots = dots;
    return jumped;
}

AxisValidity NavValidator::classify(const AxisTrack& track, bool rawOk, double dots, double t) const {
    if (!rawOk) return {SignalStatus::NoSignal, false, false};
    if (t < track.acquiringUntil) return {SignalStatus::Acquiring, true, false};
    if (std::fabs(dots) >= p_.fullScaleDots) return {SignalStatus::FullScale, true, false};
    return {SignalStatus::Valid, true, true};
}

const NavValidity& NavValidator::update(const SimSnapshot& snap) {
    const NavSnapshot& n = snap.nav;
    const double t = snap.simTimeSec;
    v_.discontinuity = false;
    v_.discontinuityReason.clear();

    // 1. Source identity: switching NAV1/NAV2/GPS, retuning, or a new approach
    //    makes all history meaningless.
    const bool identityChanged = haveIdentity_ && (n.source != lastSource_ || n.frequency != lastFrequency_ ||
                                                   snap.approach.label != lastLabel_);
    if (identityChanged) {
        v_.discontinuity = true;
        char buf[160];
        std::snprintf(buf, sizeof(buf), "source changed (src %d->%d, freq %d->%d, '%s'->'%s')",
                      static_cast<int>(lastSource_), static_cast<int>(n.source), lastFrequency_, n.frequency,
                      lastLabel_.c_str(), snap.approach.label.c_str());
        v_.discontinuityReason = buf;
    }
    haveIdentity_ = true;
    lastSource_ = n.source;
    lastFrequency_ = n.frequency;
    lastLabel_ = snap.approach.label;

    // 2. Lateral
    const double hdef = n.hdefDots;
    const bool latRaw = n.lateralValid && n.courseValid && std::isfinite(hdef) &&
                        std::fabs(hdef) <= p_.maxPlausibleDots;
    // A VOR receiver reporting neither TO nor FROM is showing its NAV flag.
    const bool latFlagged = latRaw && snap.approach.kind == ApproachKind::Vor && n.fromTo == 0;
    const bool latJump = stepAxis(lat_, latRaw && !latFlagged, hdef, t, identityChanged);
    v_.lateral = latFlagged ? AxisValidity{SignalStatus::Flagged, false, false}
                            : classify(lat_, latRaw, hdef, t);

    // 3. Vertical: only meaningful alongside a usable lateral signal.
    const double vdef = n.vdefDots;
    const bool vertRaw = v_.lateral.measuredValid && n.verticalValid && std::isfinite(vdef) &&
                         std::fabs(vdef) <= p_.maxPlausibleDots;
    const bool vertJump = stepAxis(vert_, vertRaw, vdef, t, identityChanged);
    v_.vertical = classify(vert_, vertRaw, vdef, t);
    const bool established = v_.lateral.measuredValid && std::fabs(hdef) < p_.establishedLateralDots;
    if (v_.vertical.measuredValid && !established) v_.vertical = {SignalStatus::NotEstablished, true, false};

    if (!v_.discontinuity && (latJump || vertJump)) {
        v_.discontinuity = true;
        v_.discontinuityReason = latJump ? "lateral deviation jumped" : "vertical deviation jumped";
    }

    // 4. Groundspeed gate for calculated cues (rates in dots/s mean little when
    //    barely moving). Only applied when groundspeed is known.
    v_.lowGroundspeed = snap.groundspeedValid && snap.groundspeedKt < p_.minGroundspeedKt;
    if (v_.lowGroundspeed) {
        v_.lateral.predictive = false;
        v_.vertical.predictive = false;
    }

    lastTime_ = t;
    return v_;
}

}  // namespace navsight::nav
