#pragma once

#include <string>

#include "data/SimSnapshot.h"

// Nav source cross-check: is the pilot using the right source and procedure
// logic for what they are flying? Catches the classic setup errors:
//   GPS_ON_LOC      - HSI on GPS while a live ILS/LOC serves the approach
//   NAV_ON_RNAV     - HSI on NAV while GPS1 has an RNAV approach active
//   APPR_NOT_ACTIVE - GPS approach loaded, established on final, not activated
//   LOC_NE_APPR     - tuned localizer serves a different runway than the loaded approach
//   CRS_NOT_SET     - HSI course pointer not set to the localizer course
//
// Pure logic on SimSnapshot (the real, un-overridden data): no X-Plane calls.
namespace navsight::nav {

enum class SourceCheckStatus { NotApplicable, Ok, Caution };

struct SourceCheckResult {
    SourceCheckStatus status = SourceCheckStatus::NotApplicable;
    std::string code = "NA";      // "GPS_ON_LOC", ..., "OK", "NA"
    std::string banner;           // title-box caution (cautions only)
    std::string chip = "SRC --";  // footer status chip text
    std::string detail;  // context for the nav log

    bool caution() const { return status == SourceCheckStatus::Caution; }
};

// Where the aircraft is relative to the GPS approach's final segment
// (FAF -> runway threshold), in a flat-earth nm frame at the threshold.
struct FinalGeometry {
    bool valid = false;
    double distToThresholdNm = 0.0;  // along the final course; + = before the threshold
    double crossTrackNm = 0.0;       // unsigned
    double finalCourseDegMag = 0.0;
};

FinalGeometry finalApproachGeometry(const GpsApproachPlanSnapshot& plan, double latDeg, double lonDeg,
                                    double magVariationDeg);

// Established on final: 0 < dist <= 12 nm, xtk <= 1 nm, track within 30 deg of
// the final course, groundspeed >= 40 kt.
bool establishedOnFinal(const FinalGeometry& g, const SimSnapshot& snap);

// What kind of approach procedure is loaded in the GPS, when the FMS says.
// X-Plane's G1000 flies a loaded ILS as a GPS overlay in LNAV, so LNAV alone
// can't tell it from an RNAV approach; LPV, L/VNAV, LNAV+V, LP, LP+V, GLS and
// RNP AR are RNAV-only and can.
enum class ProcedureKind { Unknown, LocBased, Rnav, Other };
const char* toString(ProcedureKind k);

// From final_approach_name ("ILS20", "I20", "R20", ...; ARINC 424 type letter;
// empty on the G1000) and fas_id ("W20A" / "RNV20Y" = RNAV). A localizer ident
// in fas_id is ignored: X-Plane fills it from the ILS tuned on the NAV radio.
ProcedureKind classifyProcedure(const std::string& name, const std::string& fasId);
ProcedureKind procedureKind(const SimSnapshot& snap);
// "ILS" or "LOC" for a localizer-based procedure.
const char* locProcedureText(const SimSnapshot& snap);

// Numeric code for the navsight/check/code dataref: 0 none, 1 GPS_ON_LOC,
// 2 NAV_ON_RNAV, 3 APPR_NOT_ACTIVE, 4 LOC_NE_APPR, 5 CRS_NOT_SET.
int sourceCheckCodeNumber(const std::string& code);

// Instantaneous evaluation (no debounce). `navContext` = the instrument is
// showing navigation guidance (not heading-only).
SourceCheckResult evaluateSourceCheck(const SimSnapshot& snap, bool navContext);

// Debounced checker: a caution must persist kShowSec to appear and is cleared
// after it has been absent for kClearSec, so source switching never flickers.
class SourceChecker {
public:
    static constexpr double kShowSec = 2.0;
    static constexpr double kClearSec = 1.5;

    const SourceCheckResult& update(const SimSnapshot& snap, bool navContext);
    const SourceCheckResult& result() const { return shown_; }
    void reset();

private:
    SourceCheckResult shown_;
    std::string pendingCode_;
    double pendingSince_ = 0.0;
    double absentSince_ = -1.0;
    double lastTime_ = -1.0;
};

}  // namespace navsight::nav
