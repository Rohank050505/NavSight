#pragma once

#include "data/ApproachResolver.h"
#include "data/DataRef.h"
#include "data/SimDataSource.h"

namespace navsight {

// Live data from X-Plane datarefs. All dataref names used by NavSight live in
// XPlaneDataSource.cpp so they can be audited in one place.
class XPlaneDataSource final : public SimDataSource {
public:
    XPlaneDataSource();
    bool init() override;
    void sample(SimSnapshot& out) override;

private:
    void sampleNav(SimSnapshot& out);

    // Aircraft
    FloatDataRef magHeading_;
    FloatDataRef trueHeading_;
    FloatDataRef magVariation_;
    DoubleDataRef latitude_;
    DoubleDataRef longitude_;
    FloatDataRef groundspeed_;
    FloatDataRef groundTrack_;
    FloatDataRef yawRate_;
    FloatDataRef verticalSpeed_;
    FloatDataRef fmsVpa_;
    IntDataRef paused_;

    // Pilot HSI (follows the HSI source selector)
    IntDataRef hsiSource_;
    FloatDataRef hsiHdef_;
    FloatDataRef hsiVdef_;
    IntDataRef hsiHasHorizontal_;
    IntDataRef hsiHasVertical_;
    FloatDataRef hsiBearing_;
    IntDataRef hsiHasDme_;
    FloatDataRef hsiDmeNm_;
    IntDataRef hsiFromTo_;

    // Per-source details, indexed by NavSource
    FloatDataRef course_[4];
    StringDataRef navId_[4];
    IntDataRef navFrequency_[2];  // NAV1, NAV2 only
    IntDataRef gpsCdiMode_[2];    // GPS1, GPS2 only
    FloatDataRef gpsNmPerDot_[2]; // GPS1, GPS2 only

    // Source cross-check: both NAV receivers regardless of the HSI selector,
    // the HSI course pointer, and the GPS1 approach plan.
    IntDataRef radioSignal_[2];
    FloatDataRef radioHdef_[2];
    IntDataRef radioFromTo_[2];
    FloatDataRef hsiObs_;
    StringDataRef finalApproachName_;
    StringDataRef fasId_;
    StringDataRef ltpId_;
    void sampleCrossCheck(SimSnapshot& out);

    ApproachResolver resolver_;
    ApproachResolver radioResolver_[2]{ApproachResolver("NAV1"), ApproachResolver("NAV2")};
    double lastPlanRead_ = -1e9;
    int lastPlanMode_ = -99;
    GpsApproachPlanSnapshot plan_;
    bool navAvailable_ = false;
};

}  // namespace navsight
