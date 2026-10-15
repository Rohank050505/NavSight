#pragma once

#include <string>

#include "data/SimSnapshot.h"

namespace navsight {

// Identifies what the HSI-selected source is tracking (ILS RWY 22 at KSEA,
// VOR SEA, RNAV RWY 16L, ...) using X-Plane's nav database.
//
// Nav-database searches are not free, so results are cached and re-resolved
// only when the source, frequency or ident changes, or every 30 s as the
// "nearest station on this frequency" may change during a flight.
class ApproachResolver {
public:
    // `name` labels its nav-log lines ("nav source" = the HSI-selected source,
    // "NAV1"/"NAV2" = the receivers read for the source cross-check).
    explicit ApproachResolver(const char* name = "nav source") : name_(name) {}

    const ApproachInfo& resolve(const NavSnapshot& nav, double latDeg, double lonDeg, double simTimeSec);
    void invalidate() { haveResult_ = false; }

    // GPS approach flight plan (runway, airport, FAF and threshold positions).
    static GpsApproachPlanSnapshot readGpsPlan(bool copilot, int gpsCdiMode);

private:
    const char* name_;
    ApproachInfo resolveRadio(const NavSnapshot& nav, double latDeg, double lonDeg) const;
    ApproachInfo resolveGps(const NavSnapshot& nav, double latDeg, double lonDeg) const;

    ApproachInfo cached_;
    bool haveResult_ = false;
    NavSource lastSource_ = NavSource::Nav1;
    int lastFrequency_ = 0;
    std::string lastId_;
    bool lastLateralValid_ = false;
    int lastGpsCdiMode_ = -1;
    double lastResolveTime_ = 0.0;
};

}  // namespace navsight
