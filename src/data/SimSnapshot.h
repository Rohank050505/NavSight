#pragma once

#include <string>

// Plain-data picture of the simulator state at one instant.
//
// This is the boundary between "where data comes from" and "what we do with
// it": the nav layer consumes only SimSnapshot, never X-Plane APIs. That keeps
// the logic testable offline and lets the same snapshot be fed from a recorded
// flight or streamed to/from the companion application later.
//
// Every group carries its own validity flag rather than magic sentinel values.
namespace navsight {

// Which receiver drives the pilot's HSI (X-Plane's HSI source selector).
enum class NavSource { Nav1 = 0, Nav2 = 1, Gps1 = 2, Gps2 = 3 };

// What kind of guidance the selected source is providing, resolved from the
// nav database (radios) or the GPS waypoint (GPS).
enum class ApproachKind {
    None,      // nothing identified
    Vor,       // VOR / VOR-DME radial
    Ils,       // localizer + glideslope
    Localizer, // LOC, LDA, SDF, ... (lateral only)
    Rnav,      // GPS approach to a runway
    GpsEnroute, // GPS leg that isn't to a runway
    Manual      // runway approach declared by the pilot (runway override)
};

// What the active GPS waypoint / tuned station physically is. Determines how
// distance is presented: a GPS-defined point (fix, lat/lon, runway threshold)
// has a computed GPS distance, not a DME.
enum class FixKind {
    Unknown,
    GpsFix,         // RNAV fix or lat/lon point - exists only in the database
    Runway,         // runway threshold waypoint (RWxx)
    Airport,        // airport reference point
    GroundNavaid    // VOR / NDB / DME / ILS / TACAN - a transmitter on the ground
};

struct ApproachInfo {
    ApproachKind kind = ApproachKind::None;
    std::string label;     // e.g. "ILS RWY 22", "VOR SEA", "GPS KAYLA"
    std::string runway;    // e.g. "22", "16L"; only when verified (nav DB / approach plan)
    std::string airport;   // ICAO, empty if unknown
    std::string stationId; // tuned ident / GPS waypoint
    bool runwayManual = false;      // runway set by the pilot (override), not verified data
    std::string approachMode;       // GPS approach mode, e.g. "LPV", "LNAV" (GPS only)
    bool verticalExpected = false;  // a missing glidepath should be flagged (ILS, LPV, L/VNAV, GLS)

    // Ground station position (LOC / VOR antenna) - for distance-based scaling.
    bool stationPosValid = false;
    double stationLatDeg = 0.0, stationLonDeg = 0.0;
    // Glidepath angle from the paired glideslope navaid (ILS); 0 = unknown.
    float glidepathDeg = 0.0f;

    // Active GPS waypoint / tuned station and what kind of point it is.
    std::string activeFixId;
    FixKind activeFixKind = FixKind::Unknown;

    bool isApproach() const {
        return kind == ApproachKind::Ils || kind == ApproachKind::Localizer || kind == ApproachKind::Rnav ||
               kind == ApproachKind::Manual;
    }
};

struct NavSnapshot {
    NavSource source = NavSource::Nav1;

    // Deflections exactly as X-Plane reports them, in dots. Sign conventions
    // were confirmed in-sim against X-Plane's own G1000 on 2026-10-14:
    //   hdef > 0 : CDI needle deflected right = course lies to the right
    //              (aircraft LEFT of course)
    //   vdef > 0 : GS diamond deflected down = glidepath lies below
    //              (aircraft ABOVE the path)  - note: opposite sense to hdef
    bool lateralValid = false;
    float hdefDots = 0.0f;
    bool verticalValid = false;
    float vdefDots = 0.0f;

    bool courseValid = false;
    float courseDegMag = 0.0f;  // OBS for VORs, localizer course for ILS, DTK for GPS
    bool bearingValid = false;
    float bearingDegMag = 0.0f; // to the station / waypoint

    bool dmeValid = false;
    float dmeNm = 0.0f;

    // GPS sources only: sim/cockpit/radios/gps_cdi_sensitivity
    // (0 OCN, 1 ENR, 2 TERM, 3 DPRT, 4 MAPR, 5 APR, 6 RNPAR, 7 LNAV, 8 LNAV+V,
    //  9 L/VNAV, 10 LP, 11 LPV, 12 LP+V, 13 GLS); -1 = unknown / not GPS.
    int gpsCdiMode = -1;
    float gpsNmPerDot = 0.0f;   // GPS sources: CDI sensitivity (nm per dot); 0 = unknown
    float fmsVpaDeg = 0.0f;     // GPS sources: FMS vertical path angle; 0 = unknown

    int fromTo = 0;          // 0 = flag (no indication), 1 = TO, 2 = FROM
    int frequency = 0;       // nav.dat units (MHz * 100); 0 for GPS
    std::string navId;       // ident reported by the receiver
};

// One NAV receiver, read regardless of the HSI source selector (for the
// source cross-check: is the *other* radio a better source?).
struct RadioSnapshot {
    bool available = false;    // datarefs bound
    int frequency = 0;         // nav.dat units
    bool signal = false;       // navN_display_horizontal
    float hdefDots = 0.0f;
    bool courseValid = false;
    float courseDegMag = 0.0f; // OBS for VOR, localizer course for ILS/LOC
    int fromTo = 0;
    std::string ident;         // navN_nav_id as received
    ApproachInfo station;      // resolved from the nav database (kind, runway, airport, position)
};

// The approach loaded in GPS1's approach flight plan, independent of the HSI
// source (for "approach loaded but not activated" and source mismatches).
struct GpsApproachPlanSnapshot {
    bool loaded = false;        // the plan has a runway leg
    std::string runway;         // e.g. "20"
    std::string airport;        // ICAO or empty
    bool finalValid = false;    // FAF and threshold positions known
    double fafLatDeg = 0.0, fafLonDeg = 0.0;
    double thresholdLatDeg = 0.0, thresholdLonDeg = 0.0;
    int gpsCdiMode = -1;        // GPS1 gps_cdi_sensitivity (see NavSnapshot)
    // What kind of procedure is loaded, as X-Plane's FMS reports it (raw;
    // classified by nav::procedureKind). Empty when the FMS doesn't say.
    std::string procedureName;  // final_approach_name, e.g. "I20", "ILS20", "R20"
    // fas_id / ltp_id: in X-Plane 12.4 these follow the ILS tuned on the NAV
    // radio ("IMWA" / "DME"), not the GPS procedure; only an RNAV ID in fas_id
    // ("W20A", "RNV20Y") identifies the procedure. ltp_id is kept for the log.
    std::string fasId;
    std::string ltpId;
};

struct SimSnapshot {
    double simTimeSec = 0.0;  // monotonic sim elapsed time

    bool headingValid = false;
    float magHeadingDeg = 0.0f;   // aircraft magnetic heading, [0,360)
    float trueHeadingDeg = 0.0f;  // aircraft true heading, [0,360)
    float magVariationDeg = 0.0f; // local variation (east positive)

    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;

    bool groundspeedValid = false;
    float groundspeedKt = 0.0f;
    bool groundTrackValid = false;
    float groundTrackDegMag = 0.0f;   // direction actually flown over the ground
    bool yawRateValid = false;
    float yawRateDegSec = 0.0f;       // + = turning right
    bool verticalSpeedValid = false;
    float verticalSpeedFpm = 0.0f;

    bool paused = false;

    NavSnapshot nav;
    ApproachInfo approach;

    // Source cross-check inputs.
    RadioSnapshot radios[2];            // NAV1, NAV2
    bool hsiObsValid = false;
    float hsiObsDegMag = 0.0f;          // HSI course pointer setting
    GpsApproachPlanSnapshot gpsPlan;    // GPS1 approach plan
};

}  // namespace navsight
