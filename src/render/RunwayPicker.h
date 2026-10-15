#pragma once

#include <string>
#include <vector>

#include "render/Hit.h"

// State machine for the in-window runway-override picker: suggested runway
// numbers (from the selected course), side L / C / R, typed entry, AUTO
// (clear) and APPLY. Pure (no X-Plane, no drawing) so every interaction is
// unit-tested; InstrumentWindow feeds it clicks and keys and carries out the
// returned action.
namespace navsight::render {

struct RunwayPicker {
    enum class Action { None, Apply, Clear, Close };
    struct Result {
        Action action = Action::None;
        std::string runway;  // for Apply
    };

    bool open = false;
    std::vector<std::string> candidates;  // e.g. {"19","20","21"}
    int selected = -1;                    // candidate index, -1 = none
    char side = 0;                        // 0, 'L', 'C' or 'R'
    std::string typed;                    // typed digits (take precedence over candidates)
    std::string active;                   // runway currently overridden ("" = AUTO)
    double courseDegMag = 0.0;
    bool courseValid = false;

    void openFor(bool courseValid, double courseDegMag, const std::string& currentOverride);
    void close() { open = false; }

    // The designator APPLY would set ("" if incomplete/invalid).
    std::string designator() const;
    // Text for the entry line while composing, e.g. "2_", "20L".
    std::string entryText() const;

    Result press(Hit hit);
    // `ch` = character (may be 0), `vkey` = XPLM virtual key code.
    Result key(char ch, int vkey);
};

}  // namespace navsight::render
