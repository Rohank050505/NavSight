#include "nav/SourceCheck.h"

#include <cctype>
#include <cmath>
#include <cstdio>

#include "nav/Angles.h"
#include "nav/ApproachNaming.h"
#include "nav/RunwayOverride.h"

namespace navsight::nav {
namespace {

constexpr double kFinalMaxNm = 12.0;        // established window along the final
constexpr double kFinalXtkNm = 1.0;         // ... and across it
constexpr double kFinalTrackDeg = 30.0;     // track vs final course
constexpr double kFinalMinGsKt = 40.0;      // airborne, moving
constexpr double kCourseMatchDeg = 10.0;    // "same course" / OBS tolerance
constexpr double kLocServesNm = 20.0;       // LOC near enough to be the one being flown
constexpr double kInBeamDots = 2.4;         // localizer needle off the stop

bool isGps(NavSource s) { return s == NavSource::Gps1 || s == NavSource::Gps2; }
bool isLoc(ApproachKind k) { return k == ApproachKind::Ils || k == ApproachKind::Localizer; }
// RNAV approach active with its own minima (RNP AR, LNAV ... GLS).
bool rnavFinalMode(int mode) { return mode >= 6 && mode <= 13; }
// Modes only an RNAV procedure produces. LNAV (7) is not one: X-Plane's G1000
// also shows LNAV for a loaded ILS flown as a GPS overlay.
bool rnavOnlyMode(int mode) { return mode == 6 || (mode >= 8 && mode <= 13); }
// Approach loaded at most: ocean, en route, terminal, departure.
bool notActivatedMode(int mode) { return mode >= 0 && mode <= 3; }

bool sameRunway(const std::string& a, const std::string& b) {
    const std::string na = normaliseRunway(a), nb = normaliseRunway(b);
    return !na.empty() && na == nb;
}
bool airportsDiffer(const std::string& a, const std::string& b) { return !a.empty() && !b.empty() && a != b; }

int roundCourse(double deg) {
    int c = static_cast<int>(std::lround(wrap360(deg))) % 360;
    return c == 0 ? 360 : c;
}

std::string course3(double deg) {
    char b[8];
    std::snprintf(b, sizeof b, "%03d", roundCourse(deg));
    return b;
}

const char* sourceText(NavSource s) {
    switch (s) {
        case NavSource::Nav1: return "NAV1";
        case NavSource::Nav2: return "NAV2";
        case NavSource::Gps1: return "GPS1";
        case NavSource::Gps2: return "GPS2";
    }
    return "?";
}

const char* locKindText(ApproachKind k) { return k == ApproachKind::Ils ? "ILS" : "LOC"; }

// A live ILS/LOC on this receiver that serves the procedure being flown:
// same runway as the loaded GPS approach, or (no usable plan) on the selected
// course and close by.
bool locServesProcedure(const RadioSnapshot& r, const SimSnapshot& snap) {
    if (!r.available || !r.signal || !isLoc(r.station.kind)) return false;
    const GpsApproachPlanSnapshot& plan = snap.gpsPlan;
    if (plan.loaded && !plan.runway.empty() && !r.station.runway.empty())
        return sameRunway(plan.runway, r.station.runway) && !airportsDiffer(plan.airport, r.station.airport);
    if (!snap.nav.courseValid || !r.courseValid) return false;
    if (std::fabs(wrap180(r.courseDegMag - snap.nav.courseDegMag)) > kCourseMatchDeg) return false;
    if (!r.station.stationPosValid) return false;
    return distanceNm(snap.latitudeDeg, snap.longitudeDeg, r.station.stationLatDeg, r.station.stationLonDeg) <=
           kLocServesNm;
}

std::string radioText(const RadioSnapshot& r) {
    if (!r.available) return "n/a";
    std::string s = r.station.label.empty() ? "-" : r.station.label;
    if (!r.station.airport.empty()) s += " " + r.station.airport;
    s += r.signal ? " (live)" : " (no signal)";
    return s;
}

std::string detailText(const SimSnapshot& snap, const FinalGeometry& g, bool established) {
    const GpsApproachPlanSnapshot& plan = snap.gpsPlan;
    const char* mode = gpsModeName(plan.gpsCdiMode);
    char geo[96] = "";
    if (g.valid)
        std::snprintf(geo, sizeof geo, " established=%d dist=%.1fnm xtk=%.2fnm final=%s", established ? 1 : 0,
                      g.distToThresholdNm, g.crossTrackNm, course3(g.finalCourseDegMag).c_str());
    std::string planText = plan.loaded ? "RWY " + plan.runway + (plan.airport.empty() ? "" : " " + plan.airport) : "none";
    char obs[24] = "-";
    if (snap.hsiObsValid) std::snprintf(obs, sizeof obs, "%s", course3(snap.hsiObsDegMag).c_str());
    return std::string("[hsi=") + sourceText(snap.nav.source) + " gps_mode=" + (*mode ? mode : "?") +
           " nav1=" + radioText(snap.radios[0]) + " nav2=" + radioText(snap.radios[1]) + " plan=" + planText +
           " proc=" + toString(procedureKind(snap)) + " (name='" + plan.procedureName + "' fas='" + plan.fasId +
           "' ltp='" + plan.ltpId + "')" +
           " obs=" + obs + geo + "]";
}

SourceCheckResult caution(const char* code, std::string banner, std::string chip) {
    SourceCheckResult r;
    r.status = SourceCheckStatus::Caution;
    r.code = code;
    r.banner = std::move(banner);
    r.chip = std::move(chip);
    return r;
}

}  // namespace

namespace {

std::string upperTrim(const std::string& s) {
    std::string out;
    for (char c : s)
        if (!std::isspace(static_cast<unsigned char>(c))) out += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}
bool contains(const std::string& s, const char* w) { return s.find(w) != std::string::npos; }
bool isDigit(char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; }

}  // namespace

const char* toString(ProcedureKind k) {
    switch (k) {
        case ProcedureKind::LocBased: return "LOC-based";
        case ProcedureKind::Rnav: return "RNAV";
        case ProcedureKind::Other: return "other";
        case ProcedureKind::Unknown: break;
    }
    return "unknown";
}

ProcedureKind classifyProcedure(const std::string& name, const std::string& fasId) {
    const std::string n = upperTrim(name);
    if (!n.empty()) {
        for (const char* w : {"ILS", "LOC", "LDA", "SDF", "IGS"})
            if (contains(n, w)) return ProcedureKind::LocBased;
        for (const char* w : {"RNAV", "RNP", "GPS", "GLS", "LPV"})
            if (contains(n, w)) return ProcedureKind::Rnav;
        for (const char* w : {"VOR", "NDB", "TAC"})
            if (contains(n, w)) return ProcedureKind::Other;
        // ARINC 424 approach identifier: type letter + runway ("I20", "R04L").
        if (n.size() >= 2 && isDigit(n[1])) {
            switch (n[0]) {
                case 'I': case 'L': case 'B': case 'X': case 'U': return ProcedureKind::LocBased;
                case 'R': case 'H': case 'P': case 'J': return ProcedureKind::Rnav;
                case 'V': case 'D': case 'N': case 'S': case 'Q': case 'T': return ProcedureKind::Other;
                default: break;
            }
        }
    }
    // fas_id / ltp_id: in X-Plane 12.4 (G1000) they follow the ILS *tuned on
    // the NAV radio* ('IMWA' / 'DME' while an RNAV approach was flown in LPV),
    // so a localizer ident there says nothing about the loaded procedure. Only
    // an RNAV final-segment ID is evidence.
    const std::string fas = upperTrim(fasId);
    if (fas.rfind("RNV", 0) == 0) return ProcedureKind::Rnav;
    if (fas.size() == 4 && (fas[0] == 'W' || fas[0] == 'G') && isDigit(fas[1]) && isDigit(fas[2]))
        return ProcedureKind::Rnav;  // SBAS / GBAS channel
    return ProcedureKind::Unknown;
}

ProcedureKind procedureKind(const SimSnapshot& snap) {
    return classifyProcedure(snap.gpsPlan.procedureName, snap.gpsPlan.fasId);
}

const char* locProcedureText(const SimSnapshot& snap) {
    const std::string n = upperTrim(snap.gpsPlan.procedureName);
    const bool ils = contains(n, "ILS") || (n.size() >= 2 && n[0] == 'I' && isDigit(n[1]));
    return ils ? "ILS" : "LOC";
}

int sourceCheckCodeNumber(const std::string& code) {
    static constexpr const char* kCodes[] = {"GPS_ON_LOC", "NAV_ON_RNAV", "APPR_NOT_ACTIVE", "LOC_NE_APPR",
                                             "CRS_NOT_SET"};
    for (int i = 0; i < 5; ++i)
        if (code == kCodes[i]) return i + 1;
    return 0;
}

FinalGeometry finalApproachGeometry(const GpsApproachPlanSnapshot& plan, double latDeg, double lonDeg,
                                    double magVariationDeg) {
    FinalGeometry g;
    if (!plan.loaded || !plan.finalValid) return g;
    // Flat-earth nm frame centred on the threshold (x east, y north).
    const double k = std::cos(degToRad(plan.thresholdLatDeg)) * 60.0;
    const double fx = wrap180(plan.fafLonDeg - plan.thresholdLonDeg) * k;
    const double fy = (plan.fafLatDeg - plan.thresholdLatDeg) * 60.0;
    const double len = std::sqrt(fx * fx + fy * fy);
    if (!(len > 0.1)) return g;
    const double ux = -fx / len, uy = -fy / len;  // inbound: FAF -> threshold
    const double ax = wrap180(lonDeg - plan.thresholdLonDeg) * k;
    const double ay = (latDeg - plan.thresholdLatDeg) * 60.0;
    g.valid = true;
    g.distToThresholdNm = -(ax * ux + ay * uy);
    g.crossTrackNm = std::fabs(ax * uy - ay * ux);
    g.finalCourseDegMag = wrap360(radToDeg(std::atan2(ux, uy)) - magVariationDeg);
    return g;
}

bool establishedOnFinal(const FinalGeometry& g, const SimSnapshot& snap) {
    if (!g.valid || !snap.groundTrackValid || !snap.groundspeedValid) return false;
    return g.distToThresholdNm > 0.0 && g.distToThresholdNm <= kFinalMaxNm && g.crossTrackNm <= kFinalXtkNm &&
           std::fabs(wrap180(snap.groundTrackDegMag - g.finalCourseDegMag)) <= kFinalTrackDeg &&
           snap.groundspeedKt >= kFinalMinGsKt;
}

SourceCheckResult evaluateSourceCheck(const SimSnapshot& snap, bool navContext) {
    const NavSnapshot& nav = snap.nav;
    const GpsApproachPlanSnapshot& plan = snap.gpsPlan;
    const ApproachInfo& tuned = snap.approach;  // HSI source, real (un-overridden) data
    const bool hsiGps = isGps(nav.source);
    const int gpsMode = plan.gpsCdiMode;

    // Magnetic variation from the heading pair (true - magnetic), which has an
    // unambiguous sign; the variation dataref is the fallback.
    const double variation = snap.headingValid ? wrap180(snap.trueHeadingDeg - snap.magHeadingDeg)
                                               : snap.magVariationDeg;
    const FinalGeometry geo = finalApproachGeometry(plan, snap.latitudeDeg, snap.longitudeDeg, variation);
    const bool established = establishedOnFinal(geo, snap);

    // Which procedure is being flown. When the FMS names it, that decides;
    // otherwise an RNAV-only GPS mode (LPV, L/VNAV, ...) does.
    const ProcedureKind proc = procedureKind(snap);
    const bool locProc = proc == ProcedureKind::LocBased;
    const bool flyingRnav = proc == ProcedureKind::Rnav || (!locProc && rnavOnlyMode(gpsMode));
    const bool missed = gpsMode == 4;  // MAPR: the approach is over

    // On the localizer's final: established on the plan's final segment or,
    // with no plan geometry, tracking the localizer course inside the beam.
    auto onFinal = [&](const RadioSnapshot& radio) {
        if (geo.valid) return established;
        return radio.courseValid && snap.groundTrackValid &&
               std::fabs(wrap180(snap.groundTrackDegMag - radio.courseDegMag)) <= kFinalTrackDeg &&
               std::fabs(radio.hdefDots) < kInBeamDots;
    };

    SourceCheckResult r;
    // 1. GPS on the HSI while a live localizer serves the approach being flown.
    if (hsiGps && !flyingRnav && !missed) {
        for (int i = 0; i < 2 && !r.caution(); ++i) {
            const RadioSnapshot& radio = snap.radios[i];
            if (!locServesProcedure(radio, snap) || !onFinal(radio)) continue;
            const std::string radioName = i == 0 ? "NAV1" : "NAV2";
            if (proc == ProcedureKind::Unknown && notActivatedMode(gpsMode) && established) {
                // A loaded approach, not activated, with its localizer alive:
                // either an RNAV to activate or an ILS to fly on NAV.
                r = caution("APPR_NOT_ACTIVE", "GPS APPR NOT ACTIVE - ACTIVATE OR SELECT " + radioName,
                            "APPR NOT ACTV");
                continue;
            }
            r = caution("GPS_ON_LOC",
                        std::string(locKindText(radio.station.kind)) + " APPROACH ON GPS - SELECT " + radioName,
                        "CDI GPS>" + radioName);
        }
        // 1b. A localizer procedure flown on GPS with no localizer received.
        if (!r.caution() && locProc && established) {
            r = caution("GPS_ON_LOC", std::string(locProcedureText(snap)) + " APPROACH ON GPS - TUNE AND SELECT NAV",
                        "CDI GPS>NAV");
        }
    }
    // 2. NAV on the HSI while GPS1 flies an RNAV approach the radio doesn't serve.
    if (!r.caution() && !hsiGps && !locProc && rnavFinalMode(gpsMode)) {
        bool serves = false;
        if (isLoc(tuned.kind)) {
            // A localizer to the same runway is a legitimate choice; one whose
            // runway can't be compared gets the benefit of the doubt.
            serves = tuned.runway.empty() || !plan.loaded || plan.runway.empty() ||
                     (sameRunway(tuned.runway, plan.runway) && !airportsDiffer(tuned.airport, plan.airport));
        }
        if (!serves) r = caution("NAV_ON_RNAV", "RNAV APPROACH ACTIVE - SELECT GPS", "CDI NAV>GPS");
    }
    // 3. GPS approach loaded and being flown, but not activated.
    if (!r.caution() && hsiGps && !locProc && notActivatedMode(gpsMode) && established) {
        r = caution("APPR_NOT_ACTIVE", "GPS APPROACH NOT ACTIVE - ACTIVATE APPR", "APPR NOT ACTV");
    }
    // 4. Tuned localizer serves a different runway than the loaded approach.
    if (!r.caution() && !hsiGps && isLoc(tuned.kind) && nav.lateralValid && !tuned.runway.empty() && plan.loaded &&
        !plan.runway.empty()) {
        const bool otherAirport = airportsDiffer(tuned.airport, plan.airport);
        if (otherAirport || !sameRunway(tuned.runway, plan.runway)) {
            const std::string t = otherAirport ? tuned.airport + " " + tuned.runway : "RWY " + tuned.runway;
            const std::string p = otherAirport ? plan.airport + " " + plan.runway : "RWY " + plan.runway;
            r = caution("LOC_NE_APPR",
                        std::string("TUNED ") + locKindText(tuned.kind) + " " + t + " - LOADED APPR " + p,
                        "LOC/APPR RWY");
        }
    }
    // 5. HSI course pointer not set to the localizer course.
    if (!r.caution() && !hsiGps && isLoc(tuned.kind) && nav.lateralValid && nav.courseValid && snap.hsiObsValid &&
        std::fabs(wrap180(snap.hsiObsDegMag - nav.courseDegMag)) > kCourseMatchDeg) {
        const std::string loc = course3(nav.courseDegMag);
        r = caution("CRS_NOT_SET", "SET COURSE " + loc + "* - HSI CRS " + course3(snap.hsiObsDegMag) + "*",
                    "SET CRS " + loc);
    }

    if (!r.caution()) {
        if (navContext || plan.loaded) {
            r.status = SourceCheckStatus::Ok;
            r.code = "OK";
            r.chip = "SRC OK";
        } else {
            r.status = SourceCheckStatus::NotApplicable;
            r.code = "NA";
            r.chip = "SRC --";
        }
    }
    r.detail = detailText(snap, geo, established);
    return r;
}

const SourceCheckResult& SourceChecker::update(const SimSnapshot& snap, bool navContext) {
    const double t = snap.simTimeSec;
    if (t < lastTime_) reset();  // sim reload / time reset
    lastTime_ = t;

    SourceCheckResult raw = evaluateSourceCheck(snap, navContext);
    if (raw.caution()) {
        if (shown_.caution() && shown_.code == raw.code) {
            shown_ = std::move(raw);  // same finding: refresh numbers and detail
            absentSince_ = -1.0;
            pendingCode_.clear();
            return shown_;
        }
        if (pendingCode_ != raw.code) {
            pendingCode_ = raw.code;
            pendingSince_ = t;
        }
        if (t - pendingSince_ >= kShowSec) {
            shown_ = std::move(raw);
            absentSince_ = -1.0;
            pendingCode_.clear();
            return shown_;
        }
    } else {
        pendingCode_.clear();
    }

    // No (new) finding confirmed: a shown caution lingers for kClearSec.
    if (shown_.caution()) {
        if (absentSince_ < 0.0) absentSince_ = t;
        if (t - absentSince_ < kClearSec) return shown_;
        absentSince_ = -1.0;
        if (raw.caution()) {
            // A different finding is still being confirmed; meanwhile the
            // source is at least not known to be wrong in the old way.
            SourceCheckResult plain;
            plain.status = SourceCheckStatus::Ok;
            plain.code = "OK";
            plain.chip = "SRC OK";
            plain.detail = raw.detail;
            shown_ = std::move(plain);
            return shown_;
        }
    }
    // OK / N/A apply at once; an unconfirmed caution keeps the previous state.
    if (!raw.caution()) shown_ = std::move(raw);
    return shown_;
}

void SourceChecker::reset() {
    shown_ = SourceCheckResult{};
    pendingCode_.clear();
    pendingSince_ = 0.0;
    absentSince_ = -1.0;
    lastTime_ = -1.0;
}

}  // namespace navsight::nav
