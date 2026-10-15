#pragma once

#include "data/SimSnapshot.h"
#include "nav/AxisGuidance.h"
#include "nav/InstrumentState.h"
#include "nav/NavValidation.h"
#include "nav/RunwayOverride.h"
#include "nav/TrajectoryPredictor.h"

namespace navsight {

// Turns raw sim snapshots into display state: heading filtering, mode
// selection, and predictive lateral/vertical guidance. Stateful so it can
// filter and extrapolate over time.
class InstrumentModel {
public:
    struct Settings {
        double headingSmoothingSec = 0.05;  // first-order low-pass; 0 = raw
        bool invertLateral = false;         // escape hatches if an aircraft
        bool invertVertical = false;        // reports deflection inverted
        double lateralClosureSec = 20.0;    // ideal time to centre the localizer/CDI
        double verticalClosureSec = 10.0;   // ideal time to centre the glidepath
    };

    // Deviation (dots) inside which an axis counts as "aligned".
    static constexpr double kAlignedDots = 0.5;

    explicit InstrumentModel(Settings settings = {});

    void setSettings(const Settings& s);
    void update(const SimSnapshot& snap);
    const InstrumentState& state() const { return state_; }
    const nav::NavValidity& validity() const { return validator_.validity(); }

    // Manual runway override (bound to the current nav source; see
    // nav/RunwayOverride.h). Returns false for an invalid designator.
    bool setRunwayOverride(const std::string& text);
    void clearRunwayOverride();
    const nav::RunwayOverride& runwayOverride() const { return override_; }
    // True on the frame an override was dropped because the source changed.
    bool overrideClearedBySourceChange() const { return overrideClearedBySourceChange_; }

    void reset();

private:
    void updateHeading(const SimSnapshot& snap);
    void updateNavigation(const SimSnapshot& snap);

    Settings settings_;
    InstrumentState state_;
    nav::AxisGuidance lateral_;
    nav::AxisGuidance vertical_;
    nav::NavValidator validator_;
    nav::TrajectoryPredictor trajectory_;
    nav::SourceChecker sourceChecker_;
    nav::RunwayOverride override_;
    nav::SourceIdentity lastIdentity_;
    bool overrideClearedBySourceChange_ = false;
    bool haveHistory_ = false;
    double lastSimTime_ = 0.0;
};

}  // namespace navsight
