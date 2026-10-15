#pragma once

#include <string>
#include <vector>

#include "data/SimSnapshot.h"

// Manual runway override: the pilot declares which runway the current course
// leads to when the nav data can't verify it (LOC whose navaid name lacks the
// runway, VOR or visual approaches, GPS approach plans X-Plane doesn't
// expose). It is always shown as MANUAL, never as verified data, and it is
// bound to the nav source it was set on: retuning, switching NAV1/NAV2/GPS or
// loading another approach clears it, so it can't leak into the next approach.
//
// Pure C++ (no X-Plane calls), unit-tested.
namespace navsight::nav {

// What the override was set against (taken from the *resolved*, un-overridden
// approach info).
struct SourceIdentity {
    NavSource source = NavSource::Nav1;
    int frequency = 0;
    std::string approachLabel;
    bool operator==(const SourceIdentity& o) const {
        return source == o.source && frequency == o.frequency && approachLabel == o.approachLabel;
    }
    bool operator!=(const SourceIdentity& o) const { return !(*this == o); }
};

SourceIdentity identityOf(const SimSnapshot& snap);

struct RunwayOverride {
    bool active = false;
    std::string runway;      // normalised designator, e.g. "20", "04L"
    SourceIdentity boundTo;
};

// "20" "4l" "04L" -> "20" "04L" "04L"; empty if not a valid designator.
std::string normaliseRunway(const std::string& text);

// Nearest runway number for a magnetic course and its neighbours, e.g.
// 205 -> {"19","20","21"}, 355 -> {"35","36","01"}.
std::vector<std::string> runwayCandidates(double courseDegMag);

// Does the override still apply to this snapshot's nav source?
bool stillApplies(const RunwayOverride& o, const SimSnapshot& snap);

// Runway direction vs the selected course: a designator more than 30 deg from
// the course (e.g. "02" on a 205 deg course) is flagged.
bool runwayMismatchesCourse(const std::string& runway, double courseDegMag);

// Apply to a copy of the approach info: sets the runway, marks it manual and
// rewrites the label ("LOC" -> "LOC RWY 20", "RNAV RWY 20" -> "RNAV RWY 21").
// A non-approach source (VOR, GPS leg, unidentified) becomes a manual runway
// approach so the runway strip is drawn instead of the course band.
void applyOverride(ApproachInfo& approach, const RunwayOverride& o);

}  // namespace navsight::nav
