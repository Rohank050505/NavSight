#pragma once

#include <string>

#include "nav/InstrumentState.h"
#include "render/Canvas.h"

// Maps guidance state to the words, colours and symbols on the instrument.
// Pure (no drawing) so the pilot-facing phrasing is unit-tested.
namespace navsight::render {

enum class Axis { Lateral, Vertical };

enum class Glyph { None, Arrow, Hold, OnTarget };

struct AxisText {
    std::string value;        // "0.4"
    std::string unit;         // "DOT R", "DOT LOW"
    std::string action;       // "TURN LEFT", "CLIMB", "HOLD HEADING", "ON COURSE"
    std::string qualifier;    // "SLIGHTLY", "EASE OFF", ...
    Color actionColor{};
    Glyph glyph = Glyph::None;
    Vec2 arrowDir{};          // unit vector for Glyph::Arrow (screen space)
    std::string trend;        // "CONVERGING"
    Color trendColor{};
    int trendArrow = 0;       // -1 = down-right (shrinking), +1 = up-right (growing), 0 = flat
    std::string prediction;   // "INTCPT 12S" / "TURN IN 8S" / "REQ VS -480" ...
    Color predictionColor{0.55f, 0.58f, 0.62f, 1.0f};  // cyan = estimate, amber = overshoot, grey = low confidence
    int litPips = 0;
};

// `traj` (optional) supplies the trajectory-based estimates; without it the
// rate-based predictions from AxisGuidance are used.
AxisText describeAxis(const GuidanceAxis& axis, Axis which, const nav::TrajectoryOutput* traj = nullptr);

// Distance read-out. A GPS distance to a database point is "DIST ... TO <fix>";
// only a NAV receiver's DME (a ground transmitter) is labelled "DME".
std::string distanceText(const InstrumentState& s);

const char* modeTitle(InstrumentMode mode);       // "RUNWAY ALIGNMENT MODE"
const char* modeShort(InstrumentMode mode);       // "RWY ALIGN"

}  // namespace navsight::render
