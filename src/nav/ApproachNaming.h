#pragma once

#include <string>

#include "data/SimSnapshot.h"

// Pure string helpers that turn nav-database names and GPS idents into the
// approach labels shown on the instrument. Kept separate from the X-Plane
// lookups so the parsing can be unit-tested.
namespace navsight::nav {

struct ParsedNavaidName {
    std::string airport;  // e.g. "KSEA" (if present in the name)
    std::string runway;   // e.g. "16L"
    std::string typeTag;  // "ILS", "LOC", "LDA", "SDF", "IGS" or empty
};

// Navaid names vary between nav-data formats, e.g. "KSEA 16L ILS-cat-III",
// "ILS-cat-I", "LDA/DME". Extract whatever is present.
ParsedNavaidName parseNavaidName(const std::string& name);

// True for "1".."36" optionally followed by L/R/C (leading zero allowed).
bool isRunwayDesignator(const std::string& token);

// GPS runway waypoints are "RW22", "RW04L"... -> "22", "04L"; else "".
std::string runwayFromGpsWaypoint(const std::string& ident);

// GPS approach state from sim/cockpit/radios/gps_cdi_sensitivity.
// Active approach = APR or any approach-minima mode (RNPAR..GLS). TERM means
// an approach may be loaded but is not yet active; MAPR is the missed approach.
bool isGpsApproachActive(int cdiMode);
bool isGpsMissedApproach(int cdiMode);
// Modes with published vertical minima, where a missing glidepath is a fault.
// Advisory-vertical modes (LNAV+V, LP+V) still show a glidepath if present.
bool gpsModeExpectsVertical(int cdiMode);
const char* gpsModeName(int cdiMode);  // "LPV", "LNAV", ... or "" if unknown

// Display label: "ILS RWY 22", "LOC RWY 16L", "RNAV RWY 22", "VOR SEA", ...
std::string approachLabel(ApproachKind kind, const std::string& typeTag, const std::string& runway,
                          const std::string& stationId);

}  // namespace navsight::nav
