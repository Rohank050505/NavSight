#pragma once

#include <string>

#include "data/SimSnapshot.h"

#include "nav/AxisGuidance.h"
#include "nav/NavValidation.h"
#include "nav/SourceCheck.h"
#include "nav/TrajectoryPredictor.h"

// Everything the renderer needs to draw one frame, already computed.
// The renderer must not do navigation math and must not read X-Plane data;
// this struct is the contract between the nav and render layers (and the
// natural payload for the future companion application).
namespace navsight {

enum class InstrumentMode {
    Heading,          // no lateral signal: compass rose + heading only
    Navigation,       // CDI against a VOR radial or GPS leg
    RunwayAlignment   // approach (ILS/LOC/RNAV): runway overlay + full guidance
};

struct GuidanceAxis {
    nav::AxisOutput out;   // position, rate, trend, prediction, command
    // Validation (NavValidator): out.valid mirrors status.measuredValid. Only
    // draw calculated cues (trend, prediction, chevrons, commands) if predictive.
    nav::SignalStatus status = nav::SignalStatus::NoSignal;
    bool predictive = false;
    double fullScaleDots = 2.5;
    int pipCount = 4;      // severity pips in the side panel (0.5 dot each)
};

struct InstrumentState {
    InstrumentMode mode = InstrumentMode::Heading;

    bool headingValid = false;
    double displayHeadingDeg = 0.0;  // smoothed, drives the rose rotation
    int headingReadoutDeg = 360;     // what the digital readout shows: 1..360

    bool paused = false;

    // Navigation source (valid in Navigation / RunwayAlignment modes)
    std::string sourceName;      // "NAV1", "GPS1", ...
    bool groundBasedNav = false; // NAV1/NAV2 receiver (ILS/LOC/VOR) vs GPS: needle colour
    std::string approachLabel;   // "ILS RWY 22", "VOR SEA", ...
    std::string airport;         // ICAO or empty
    std::string runway;          // "22", "16L" or empty
    bool runwayManual = false;          // runway set by the pilot (override) - show as MANUAL
    bool runwayCourseMismatch = false;  // manual runway > 30 deg from the selected course
    bool courseValid = false;
    double courseDegMag = 0.0;
    bool bearingValid = false;
    double bearingDegMag = 0.0;
    std::string approachMode;    // GPS approach mode ("LPV", "LNAV", ...), empty otherwise

    // Distance to the active point. GPS sources compute it from position
    // ("DIST"); only a NAV receiver's DME measures it from a ground station.
    enum class DistanceKind { None, Dme, Gps };
    DistanceKind distanceKind = DistanceKind::None;
    double distanceNm = 0.0;
    std::string activeFixId;     // "VANNU", "IMWA", ...
    FixKind activeFixKind = FixKind::Unknown;

    // Lateral: + = aircraft right of course. Vertical: + = aircraft above path.
    GuidanceAxis lateral;
    GuidanceAxis vertical;
    bool verticalExpected = false;  // ILS/RNAV: show a GS flag if vertical is missing
    bool lateralLost = false;       // approach identified but lateral signal gone
    bool lateralFlagged = false;    // receiver failure flag (VOR NAV flag)
    bool lowGroundspeed = false;    // calculated cues suppressed

    // Both axes within the alignment window (green box around the aircraft).
    bool aligned = false;

    // Measured ground track (distinct from bearing to the station).
    bool trackValid = false;
    double trackDegMag = 0.0;

    // Estimated trajectory (TrajectoryPredictor) - projected path, crossing,
    // roll-out lead, required vertical speed. Check .valid / .confidence.
    nav::TrajectoryOutput trajectory;

    // Nav source cross-check (debounced): chip + caution banner.
    nav::SourceCheckResult sourceCheck;
};

}  // namespace navsight
