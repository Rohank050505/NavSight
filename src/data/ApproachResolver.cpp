#include "data/ApproachResolver.h"

#include <cmath>
#include <cstdio>

#include "XPLMNavigation.h"
#include "core/Log.h"
#include "core/NavLog.h"
#include "nav/Angles.h"
#include "nav/ApproachNaming.h"

namespace navsight {
namespace {

constexpr double kReresolveSec = 30.0;
constexpr double kMaxStationRangeNm = 120.0;  // farther "matches" are a different station

using nav::distanceNm;

std::string nearestAirport(double latDeg, double lonDeg) {
    float lat = static_cast<float>(latDeg), lon = static_cast<float>(lonDeg);
    const XPLMNavRef ref = XPLMFindNavAid(nullptr, nullptr, &lat, &lon, nullptr, xplm_Nav_Airport);
    if (ref == XPLM_NAV_NOT_FOUND) return {};
    char id[32] = {};
    float aLat = 0, aLon = 0;
    XPLMGetNavAidInfo(ref, nullptr, &aLat, &aLon, nullptr, nullptr, nullptr, id, nullptr, nullptr);
    return distanceNm(latDeg, lonDeg, aLat, aLon) < 15.0 ? std::string(id) : std::string();
}

// VHF nav band: 108.00-117.95 MHz (nav.dat units, MHz * 100).
bool isNavFrequency(int freq) { return freq >= 10800 && freq <= 11795; }

// ILS/LOC frequencies: 108.10-111.95 MHz with an odd tenths digit.
bool isLocalizerFrequency(int freq) {
    return freq >= 10810 && freq <= 11195 && ((freq / 10) % 10) % 2 == 1;
}

std::string formatFrequency(int freq) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%d.%02d", freq / 100, freq % 100);
    return buf;
}

const char* fixKindName(FixKind k) {
    switch (k) {
        case FixKind::GpsFix: return "GPS fix";
        case FixKind::Runway: return "runway";
        case FixKind::Airport: return "airport";
        case FixKind::GroundNavaid: return "ground navaid";
        case FixKind::Unknown: break;
    }
    return "unknown";
}

// Glidepath angle of an ILS from its paired glideslope navaid. In X-Plane's
// nav data the glideslope's "heading" field encodes angle*100000 + course
// (e.g. 300205.0 = 3.00 deg, course 205). Returns 0 if unavailable/implausible;
// the result and its source are written to the nav log.
float glideslopeAngle(int frequency, float locLat, float locLon) {
    float lat = locLat, lon = locLon;
    int freq = frequency;
    const XPLMNavRef ref = XPLMFindNavAid(nullptr, nullptr, &lat, &lon, &freq, xplm_Nav_GlideSlope);
    if (ref == XPLM_NAV_NOT_FOUND) {
        NAVS_NAV("glideslope lookup %d: not found - glidepath angle assumed 3.00", frequency);
        return 0.0f;
    }
    float gLat = 0, gLon = 0, heading = 0;
    XPLMGetNavAidInfo(ref, nullptr, &gLat, &gLon, nullptr, nullptr, &heading, nullptr, nullptr, nullptr);
    const bool paired = distanceNm(locLat, locLon, gLat, gLon) < 5.0;  // same runway's glideslope
    const float angle = std::floor(heading / 100000.0f) / 100.0f;
    const bool plausible = paired && angle >= 2.0f && angle <= 6.0f;
    NAVS_NAV("glideslope lookup %d: heading field=%.1f -> angle %.2f deg (%s)", frequency, heading, angle,
             plausible ? "used" : "rejected - assuming 3.00");
    return plausible ? angle : 0.0f;
}

// Classify a flight-plan entry by its nav type (and runway-style ident).
FixKind classifyEntry(XPLMNavType type, const std::string& id) {
    if (!nav::runwayFromGpsWaypoint(id).empty()) return FixKind::Runway;
    switch (type) {
        case xplm_Nav_Fix:
        case xplm_Nav_LatLon: return FixKind::GpsFix;
        case xplm_Nav_Airport: return FixKind::Airport;
        case xplm_Nav_VOR:
        case xplm_Nav_NDB:
        case xplm_Nav_DME:
        case xplm_Nav_ILS:
        case xplm_Nav_Localizer:
        case xplm_Nav_TACAN: return FixKind::GroundNavaid;
        default: return FixKind::Unknown;
    }
}

// What the GPS's loaded approach procedure contains (SDK 4.1 flight-plan API).
struct ApproachPlan {
    int entries = 0;
    std::string runway;        // from the RWxx leg
    std::string airport;       // airport entry, or nearest airport to the runway leg
    bool haveRunwayPos = false;
    float runwayLat = 0, runwayLon = 0;
    bool haveFaf = false;      // last positioned fix before the runway leg
    float fafLat = 0, fafLon = 0;
};

ApproachPlan readApproachPlan(XPLMNavFlightPlan plan) {
    ApproachPlan ap;
    ap.entries = XPLMCountFMSFlightPlanEntries(plan);
    bool havePrev = false;
    float prevLat = 0, prevLon = 0;
    for (int i = 0; i < ap.entries; ++i) {
        XPLMNavType type = xplm_Nav_Unknown;
        char id[256] = {};
        XPLMNavRef ref = XPLM_NAV_NOT_FOUND;  // per SDK warning: always initialise
        float lat = 0, lon = 0;
        XPLMGetFMSFlightPlanEntryInfo(plan, i, &type, id, &ref, nullptr, &lat, &lon);
        const bool positioned = !(lat == 0.0f && lon == 0.0f) && type != xplm_Nav_Airport;
        if (ap.runway.empty()) {
            const std::string rwy = nav::runwayFromGpsWaypoint(id);
            if (!rwy.empty()) {
                ap.runway = rwy;
                ap.haveRunwayPos = positioned;
                ap.runwayLat = lat;
                ap.runwayLon = lon;
                // The final approach course runs from the fix before the runway leg.
                if (havePrev && positioned && distanceNm(prevLat, prevLon, lat, lon) > 0.5) {
                    ap.haveFaf = true;
                    ap.fafLat = prevLat;
                    ap.fafLon = prevLon;
                }
            } else if (positioned) {
                havePrev = true;
                prevLat = lat;
                prevLon = lon;
            }
        }
        if (ap.airport.empty() && type == xplm_Nav_Airport) ap.airport = id;
    }
    if (ap.airport.empty() && ap.haveRunwayPos) ap.airport = nearestAirport(ap.runwayLat, ap.runwayLon);
    return ap;
}

// Find the active waypoint in a flight plan; prefer the entry the GPS is
// flying to, else any entry with the same ident.
bool findFix(XPLMNavFlightPlan plan, const std::string& ident, FixKind& kind) {
    const int n = XPLMCountFMSFlightPlanEntries(plan);
    if (n <= 0 || ident.empty()) return false;
    auto probe = [&](int i) {
        XPLMNavType type = xplm_Nav_Unknown;
        char id[256] = {};
        XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
        XPLMGetFMSFlightPlanEntryInfo(plan, i, &type, id, &ref, nullptr, nullptr, nullptr);
        if (ident != id) return false;
        kind = classifyEntry(type, id);
        return true;
    };
    const int dest = XPLMGetDestinationFMSFlightPlanEntry(plan);
    if (dest >= 0 && dest < n && probe(dest)) return true;
    for (int i = 0; i < n; ++i)
        if (probe(i)) return true;
    return false;
}

}  // namespace

GpsApproachPlanSnapshot ApproachResolver::readGpsPlan(bool copilot, int gpsCdiMode) {
    const ApproachPlan ap = readApproachPlan(copilot ? xplm_Fpl_CoPilot_Approach : xplm_Fpl_Pilot_Approach);
    GpsApproachPlanSnapshot s;
    s.gpsCdiMode = gpsCdiMode;
    s.loaded = !ap.runway.empty();
    s.runway = ap.runway;
    s.airport = ap.airport;
    s.finalValid = ap.haveRunwayPos && ap.haveFaf;
    s.thresholdLatDeg = ap.runwayLat;
    s.thresholdLonDeg = ap.runwayLon;
    s.fafLatDeg = ap.fafLat;
    s.fafLonDeg = ap.fafLon;
    return s;
}

const ApproachInfo& ApproachResolver::resolve(const NavSnapshot& nav, double lat, double lon, double t) {
    const bool changed = !haveResult_ || nav.source != lastSource_ || nav.frequency != lastFrequency_ ||
                         nav.navId != lastId_ || nav.lateralValid != lastLateralValid_ ||
                         nav.gpsCdiMode != lastGpsCdiMode_;
    if (!changed && t - lastResolveTime_ < kReresolveSec && t >= lastResolveTime_) return cached_;

    const bool isGps = nav.source == NavSource::Gps1 || nav.source == NavSource::Gps2;
    ApproachInfo next = isGps ? resolveGps(nav, lat, lon) : resolveRadio(nav, lat, lon);

    if (!haveResult_ || next.label != cached_.label || next.airport != cached_.airport ||
        next.approachMode != cached_.approachMode || next.activeFixId != cached_.activeFixId) {
        NAVS_NAV("%s resolved: '%s' airport='%s' runway='%s' mode='%s' vertical_expected=%d "
                 "fix='%s' (%s) [source=%d id=%s freq=%d gps_cdi=%d]",
                 name_, next.label.c_str(), next.airport.c_str(),
                 next.runway.empty() ? "(unverified - course reference)" : next.runway.c_str(),
                 next.approachMode.c_str(), next.verticalExpected ? 1 : 0, next.activeFixId.c_str(),
                 fixKindName(next.activeFixKind), static_cast<int>(nav.source), nav.navId.c_str(), nav.frequency,
                 nav.gpsCdiMode);
    }

    cached_ = std::move(next);
    haveResult_ = true;
    lastSource_ = nav.source;
    lastFrequency_ = nav.frequency;
    lastId_ = nav.navId;
    lastLateralValid_ = nav.lateralValid;
    lastGpsCdiMode_ = nav.gpsCdiMode;
    lastResolveTime_ = t;
    return cached_;
}

ApproachInfo ApproachResolver::resolveRadio(const NavSnapshot& nav, double lat, double lon) const {
    ApproachInfo info;
    info.stationId = nav.navId;
    if (!isNavFrequency(nav.frequency)) return info;

    float fLat = static_cast<float>(lat), fLon = static_cast<float>(lon);
    int freq = nav.frequency;
    const XPLMNavRef ref =
        XPLMFindNavAid(nullptr, nullptr, &fLat, &fLon, &freq, xplm_Nav_ILS | xplm_Nav_Localizer | xplm_Nav_VOR);

    XPLMNavType type = xplm_Nav_Unknown;
    float navLat = 0, navLon = 0;
    char id[32] = {}, name[256] = {};
    bool found = false;
    if (ref != XPLM_NAV_NOT_FOUND) {
        XPLMGetNavAidInfo(ref, &type, &navLat, &navLon, nullptr, nullptr, nullptr, id, name, nullptr);
        found = distanceNm(lat, lon, navLat, navLon) <= kMaxStationRangeNm;
    }

    std::string typeTag;
    if (found) {
        NAVS_NAV("navaid lookup %d: type=%d id='%s' name='%s'", nav.frequency, static_cast<int>(type), id, name);
        if (id[0]) info.stationId = id;
        const nav::ParsedNavaidName parsed = nav::parseNavaidName(name);
        typeTag = parsed.typeTag;
        if (type == xplm_Nav_ILS) info.kind = ApproachKind::Ils;
        else if (type == xplm_Nav_Localizer) info.kind = ApproachKind::Localizer;
        else info.kind = ApproachKind::Vor;

        if (info.kind != ApproachKind::Vor) {
            info.runway = parsed.runway;
            info.airport = parsed.airport.empty() ? nearestAirport(navLat, navLon) : parsed.airport;
        }
        // Antenna position: lateral dots are angular, so distance to it sets the scale.
        info.stationPosValid = true;
        info.stationLatDeg = navLat;
        info.stationLonDeg = navLon;
        if (info.kind == ApproachKind::Ils) info.glidepathDeg = glideslopeAngle(nav.frequency, navLat, navLon);
    } else if (isLocalizerFrequency(nav.frequency)) {
        // Not in the database, but 108.10-111.95 MHz with odd tenths is reserved
        // for localizers - a fact, not a guess. No runway is invented.
        info.kind = ApproachKind::Localizer;
    } else if (isNavFrequency(nav.frequency)) {
        // A VOR-band frequency we can't identify: show the frequency only.
        info.kind = ApproachKind::None;
        info.label = formatFrequency(nav.frequency);
        return info;
    } else {
        return info;  // not a nav frequency (e.g. unset radio): nothing to show
    }

    // The runway designator is shown only when the nav database names it;
    // otherwise the instrument shows a course reference instead.
    info.label = nav::approachLabel(info.kind, typeTag, info.runway, info.stationId);
    info.verticalExpected = info.kind == ApproachKind::Ils;
    // A NAV receiver always tracks a transmitter on the ground; distance is DME.
    info.activeFixId = info.stationId;
    info.activeFixKind = FixKind::GroundNavaid;
    return info;
}

// GPS: an approach counts as active from the GPS's own approach mode
// (gps_cdi_sensitivity APR / LNAV / LPV / ...), not from waypoint names. The
// runway and airport come from the loaded approach procedure.
ApproachInfo ApproachResolver::resolveGps(const NavSnapshot& nav, double lat, double lon) const {
    ApproachInfo info;
    info.stationId = nav.navId;
    info.activeFixId = nav.navId;

    const bool copilot = nav.source == NavSource::Gps2;
    const XPLMNavFlightPlan approachPlan = copilot ? xplm_Fpl_CoPilot_Approach : xplm_Fpl_Pilot_Approach;
    const XPLMNavFlightPlan primaryPlan = copilot ? xplm_Fpl_CoPilot_Primary : xplm_Fpl_Pilot_Primary;

    if (!findFix(approachPlan, nav.navId, info.activeFixKind) && !findFix(primaryPlan, nav.navId, info.activeFixKind))
        info.activeFixKind = nav::runwayFromGpsWaypoint(nav.navId).empty() ? FixKind::Unknown : FixKind::Runway;

    const ApproachPlan ap = readApproachPlan(approachPlan);
    const int mode = nav.gpsCdiMode;
    info.approachMode = nav::gpsModeName(mode);

    if (nav::isGpsApproachActive(mode)) {
        info.kind = ApproachKind::Rnav;
        info.runway = !ap.runway.empty() ? ap.runway : nav::runwayFromGpsWaypoint(nav.navId);
        info.airport = !ap.airport.empty() ? ap.airport : nearestAirport(lat, lon);
        info.verticalExpected = nav::gpsModeExpectsVertical(mode);
        NAVS_NAV("gps approach active: mode=%s plan_entries=%d plan_runway='%s' plan_airport='%s'",
                 info.approachMode.c_str(), ap.entries, ap.runway.c_str(), ap.airport.c_str());
    } else if (mode < 0 && !nav::runwayFromGpsWaypoint(nav.navId).empty()) {
        // GPS mode unknown (dataref unavailable): fall back to an RWxx waypoint.
        info.kind = ApproachKind::Rnav;
        info.runway = nav::runwayFromGpsWaypoint(nav.navId);
        info.airport = nearestAirport(lat, lon);
    } else {
        info.kind = nav.navId.empty() ? ApproachKind::None : ApproachKind::GpsEnroute;
    }

    info.label = nav::approachLabel(info.kind, {}, info.runway, info.stationId);
    if (nav::isGpsMissedApproach(mode)) info.label = "MISSED APPR" + (nav.navId.empty() ? "" : " " + nav.navId);
    return info;
}

}  // namespace navsight
