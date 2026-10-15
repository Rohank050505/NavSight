#include "nav/InstrumentModel.h"

#include <algorithm>
#include <cmath>

#include "nav/Angles.h"

namespace navsight {
namespace {

nav::AxisGuidance::Params axisParams(double closureSec) {
    nav::AxisGuidance::Params p;
    p.closureTimeSec = closureSec;
    return p;
}

const char* sourceName(NavSource s) {
    switch (s) {
        case NavSource::Nav1: return "NAV1";
        case NavSource::Nav2: return "NAV2";
        case NavSource::Gps1: return "GPS1";
        case NavSource::Gps2: return "GPS2";
    }
    return "NAV";
}

// Nominal angular width per dot used before in-flight calibration. Only a
// starting point: TrajectoryPredictor reports Confidence::Prior until it has
// calibrated, and no commands are derived from a prior.
double lateralPriorDegPerDot(ApproachKind kind) {
    switch (kind) {
        case ApproachKind::Ils:
        case ApproachKind::Localizer: return 1.25;  // ~2.5 deg half-width over 2 dots
        case ApproachKind::Vor: return 4.0;         // ~10 deg half-width over 2.5 dots
        default: return 0.0;                        // unknown receiver: no estimate
    }
}

void applyTrajectory(bool use, nav::Command command, int direction, nav::Strength strength, bool crossValid,
                     double crossSec, double predicted10, nav::AxisOutput& out) {
    if (!use) return;
    out.command = command;
    out.direction = direction;
    out.strength = strength;
    out.predictedDots = predicted10;
    if (crossValid) {
        out.interceptValid = true;
        out.interceptSec = crossSec;
    }
}

}  // namespace

InstrumentModel::InstrumentModel(Settings settings)
    : settings_(settings),
      lateral_(axisParams(settings.lateralClosureSec)),
      vertical_(axisParams(settings.verticalClosureSec)) {
    state_.lateral.pipCount = 4;
    state_.vertical.pipCount = 5;
}

void InstrumentModel::setSettings(const Settings& s) {
    settings_ = s;
    lateral_ = nav::AxisGuidance(axisParams(s.lateralClosureSec));
    vertical_ = nav::AxisGuidance(axisParams(s.verticalClosureSec));
}

void InstrumentModel::reset() {
    state_ = InstrumentState{};
    state_.lateral.pipCount = 4;
    state_.vertical.pipCount = 5;
    lateral_.reset();
    vertical_.reset();
    validator_.reset();
    trajectory_.reset();
    sourceChecker_.reset();
    override_ =nav::RunwayOverride{};  // aircraft reload etc.: never carry an override over
    haveHistory_ = false;
    lastSimTime_ = 0.0;
}

void InstrumentModel::update(const SimSnapshot& snap) {
    state_.paused = snap.paused;
    updateHeading(snap);
    updateNavigation(snap);
}

void InstrumentModel::updateHeading(const SimSnapshot& snap) {
    state_.headingValid = snap.headingValid;
    if (!snap.headingValid) {
        haveHistory_ = false;  // re-acquire cleanly when data returns
        return;
    }

    const double raw = nav::wrap360(snap.magHeadingDeg);
    // Clamp dt: a long gap (sim reload, window hidden, pause) should snap, not glide.
    const double dt = haveHistory_ ? std::clamp(snap.simTimeSec - lastSimTime_, 0.0, 0.5) : 0.0;
    const double tau = settings_.headingSmoothingSec;

    if (!haveHistory_ || tau <= 0.0 || dt <= 0.0 || dt >= 0.5) {
        state_.displayHeadingDeg = raw;
    } else {
        // Exponential smoothing along the shortest arc so 359 -> 1 doesn't spin the card.
        const double alpha = 1.0 - std::exp(-dt / tau);
        const double delta = nav::shortestDelta(state_.displayHeadingDeg, raw);
        state_.displayHeadingDeg = nav::wrap360(state_.displayHeadingDeg + alpha * delta);
    }

    // Readout uses the raw value, rounded, aviation style: 000 is shown as 360.
    int rounded = static_cast<int>(std::lround(raw)) % 360;
    state_.headingReadoutDeg = rounded == 0 ? 360 : rounded;

    haveHistory_ = true;
    lastSimTime_ = snap.simTimeSec;
}

bool InstrumentModel::setRunwayOverride(const std::string& text) {
    const std::string rwy = nav::normaliseRunway(text);
    if (rwy.empty()) return false;
    override_.active = true;
    override_.runway = rwy;
    override_.boundTo = lastIdentity_;  // the source the pilot is looking at now
    return true;
}

void InstrumentModel::clearRunwayOverride() { override_ = nav::RunwayOverride{}; }

void InstrumentModel::updateNavigation(const SimSnapshot& snap) {
    const NavSnapshot& nav = snap.nav;

    // 0. Manual runway override: only for the nav source it was set on.
    overrideClearedBySourceChange_ = false;
    if (override_.active && !nav::stillApplies(override_, snap)) {
        override_ = nav::RunwayOverride{};
        overrideClearedBySourceChange_ = true;
    }
    lastIdentity_ = nav::identityOf(snap);
    // Display copy. Validation and receiver physics keep using snap.approach.
    ApproachInfo appr = snap.approach;
    // A localizer procedure flown on GPS (X-Plane's G1000 overlays a loaded
    // ILS in LNAV) is labelled as what it is, not "RNAV".
    if ((nav.source == NavSource::Gps1 || nav.source == NavSource::Gps2) && appr.kind == ApproachKind::Rnav &&
        appr.label.rfind("RNAV ", 0) == 0 && nav::procedureKind(snap) == nav::ProcedureKind::LocBased)
        appr.label = std::string(nav::locProcedureText(snap)) + appr.label.substr(4);
    nav::applyOverride(appr, override_);
    state_.runwayManual = appr.runwayManual;
    state_.runwayCourseMismatch =
        appr.runwayManual && nav.courseValid && nav::runwayMismatchesCourse(appr.runway, nav.courseDegMag);

    state_.sourceName = sourceName(nav.source);
    state_.approachLabel = appr.label;
    state_.airport = appr.airport;
    state_.runway = appr.runway;
    state_.courseValid = nav.courseValid;
    state_.courseDegMag = nav::wrap360(nav.courseDegMag);
    state_.bearingValid = nav.bearingValid;
    state_.bearingDegMag = nav::wrap360(nav.bearingDegMag);
    state_.approachMode = appr.approachMode;
    state_.activeFixId = appr.activeFixId;
    state_.activeFixKind = appr.activeFixKind;
    const bool gpsSource = nav.source == NavSource::Gps1 || nav.source == NavSource::Gps2;
    state_.groundBasedNav = !gpsSource;
    state_.distanceKind = !nav.dmeValid ? InstrumentState::DistanceKind::None
                          : gpsSource   ? InstrumentState::DistanceKind::Gps
                                        : InstrumentState::DistanceKind::Dme;
    state_.distanceNm = nav.dmeNm;

    // 1. Validate the source before using any of it.
    const nav::NavValidity& v = validator_.update(snap);
    if (v.discontinuity) {
        // Rates across a source change / jump would be fiction.
        lateral_.reset();
        vertical_.reset();
    }

    // 2. Convert X-Plane deflections to aircraft position relative to the path
    //    (+ = right of course / above path). Conventions (confirmed in-sim):
    //    hdef > 0 = aircraft left of course; vdef > 0 = aircraft above path.
    const double latSign = settings_.invertLateral ? 1.0 : -1.0;
    const double vertSign = settings_.invertVertical ? -1.0 : 1.0;

    state_.lateral.out = lateral_.update(v.lateral.measuredValid, latSign * nav.hdefDots, snap.simTimeSec);
    state_.lateral.status = v.lateral.status;
    state_.lateral.predictive = v.lateral.predictive;
    state_.vertical.out = vertical_.update(v.vertical.measuredValid, vertSign * nav.vdefDots, snap.simTimeSec);
    state_.vertical.status = v.vertical.status;
    state_.vertical.predictive = v.vertical.predictive;
    state_.lowGroundspeed = v.lowGroundspeed;
    state_.trackValid = snap.groundTrackValid;
    state_.trackDegMag = nav::wrap360(snap.groundTrackDegMag);

    // 2b. Trajectory prediction (estimated) from track, groundspeed, turn rate
    //     and vertical speed; gated by the same validity as the rate cues.
    nav::TrajectoryInput ti;
    ti.simTimeSec = snap.simTimeSec;
    ti.discontinuity = v.discontinuity;
    ti.lateralPredictive = v.lateral.predictive;
    ti.lateralDots = state_.lateral.out.positionDots;
    ti.lateralRateDotsSec = state_.lateral.out.rateDotsPerSec;
    ti.courseValid = nav.courseValid;
    ti.courseDegMag = nav.courseDegMag;
    ti.trackValid = snap.groundTrackValid;
    ti.trackDegMag = snap.groundTrackDegMag;
    ti.groundspeedValid = snap.groundspeedValid;
    ti.groundspeedKt = snap.groundspeedKt;
    ti.yawRateValid = snap.yawRateValid;
    ti.yawRateDegSec = snap.yawRateDegSec;
    if (gpsSource) {
        ti.gpsNmPerDot = nav.gpsNmPerDot;
    } else {
        if (appr.stationPosValid)
            ti.distanceNm = nav::distanceNm(snap.latitudeDeg, snap.longitudeDeg, appr.stationLatDeg, appr.stationLonDeg);
        else if (nav.dmeValid)
            ti.distanceNm = nav.dmeNm;
        ti.priorDegPerDot = lateralPriorDegPerDot(snap.approach.kind);  // the real receiver, not the override
        ti.stationAhead = nav.fromTo != 2;  // FROM: the antenna is behind
    }
    ti.verticalPredictive = v.vertical.predictive;
    ti.verticalDots = state_.vertical.out.positionDots;
    ti.verticalRateDotsSec = state_.vertical.out.rateDotsPerSec;
    ti.verticalSpeedValid = snap.verticalSpeedValid;
    ti.verticalSpeedFpm = snap.verticalSpeedFpm;
    if (gpsSource && nav.fmsVpaDeg >= 1.5f && nav.fmsVpaDeg <= 6.0f) {
        ti.pathAngleDeg = nav.fmsVpaDeg;
        ti.pathAngleAssumed = false;
    } else if (appr.glidepathDeg > 0.0f) {
        ti.pathAngleDeg = appr.glidepathDeg;
        ti.pathAngleAssumed = false;
    }
    const nav::TrajectoryOutput& tr = trajectory_.update(ti);
    state_.trajectory = tr;

    // 2c. With a trustworthy scale, corrections come from the trajectory
    //     (track vs course, standard-rate roll-out), not from dots/s alone.
    applyTrajectory(tr.lateral.commandValid && state_.lateral.predictive, tr.lateral.command, tr.lateral.direction,
                    tr.lateral.strength, tr.lateral.crossValid, tr.lateral.crossSec, tr.lateral.predicted10Dots,
                    state_.lateral.out);
    applyTrajectory(tr.vertical.commandValid && state_.vertical.predictive, tr.vertical.command, tr.vertical.direction,
                    tr.vertical.strength, tr.vertical.crossValid, tr.vertical.crossSec, tr.vertical.predicted10Dots,
                    state_.vertical.out);

    // 3. Mode follows what is actually usable, not what is tuned.
    if (!v.lateral.measuredValid) state_.mode = InstrumentMode::Heading;
    else if (appr.isApproach()) state_.mode = InstrumentMode::RunwayAlignment;
    else state_.mode = InstrumentMode::Navigation;

    state_.lateralFlagged = v.lateral.status == nav::SignalStatus::Flagged;
    state_.lateralLost = appr.isApproach() && !v.lateral.measuredValid;
    state_.verticalExpected = appr.verticalExpected;  // ILS, or GPS LPV / L/VNAV / GLS

    // 4. Aligned = both axes validly inside the window (not merely measured).
    const auto& lat = state_.lateral;
    const auto& ver = state_.vertical;
    const bool lateralIn = lat.status == nav::SignalStatus::Valid && std::fabs(lat.out.positionDots) < kAlignedDots;
    const bool verticalIn = ver.status == nav::SignalStatus::Valid
                                ? std::fabs(ver.out.positionDots) < kAlignedDots
                                : !state_.verticalExpected;
    state_.aligned = state_.mode == InstrumentMode::RunwayAlignment && lateralIn && verticalIn;

    // 5. Source cross-check on the real data (snap.approach, not the override).
    state_.sourceCheck = sourceChecker_.update(snap, state_.mode != InstrumentMode::Heading);
}

}  // namespace navsight
