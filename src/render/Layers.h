#pragma once

#include "nav/InstrumentState.h"
#include "render/DisplaySmoother.h"
#include "render/GuidanceText.h"
#include "render/Palette.h"
#include "render/RenderKit.h"

// Internal to the renderer: the individual drawing layers. InstrumentRenderer
// decides the layout and calls these in painter's order.
namespace navsight::render::layers {

// Dial geometry, in dial units (face radius 0.415).
constexpr float kFaceRadius = 0.415f;
constexpr float kRimWidth = 0.014f;
constexpr float kBezelRadius = kFaceRadius + kRimWidth + 0.03f;  // 0.459, outermost
constexpr float kTickOuter = 0.405f;
constexpr float kLabelRadius = 0.320f;
constexpr float kDotSpacing = 0.075f;      // one CDI dot
constexpr float kGsDotSpacing = 0.07f;     // one glideslope dot
constexpr float kMaxNeedleDots = 2.5f;
constexpr float kStripHalfWidth = 0.10f;       // runway strip (≈1.3 dots each side)
constexpr float kCourseRefHalfWidth = 0.055f;  // neutral course-reference band
constexpr float kStripClipRadius = 0.398f;     // band is clipped just inside the ticks
constexpr float kHeadingBoxCenterR = 0.481f;
constexpr float kHeadingBoxHalfH = 0.034f;

// Needle / glidepath indicator colour by navigation source, as on common
// glass-cockpit avionics: green for a ground-based receiver (NAV1/NAV2: ILS, LOC, VOR),
// magenta for GPS.
inline Color navColor(const InstrumentState& s) { return s.groundBasedNav ? pal().kNavGreen : pal().kMagenta; }

// Dial (painter's order).
void dialBackground(Canvas& c, const Dial& d);
void runwayOverlay(Canvas& c, const Dial& d, const InstrumentState& s);
void compassRose(Canvas& c, const Dial& d, const InstrumentState& s);
void bearingPointer(Canvas& c, const Dial& d, const InstrumentState& s);
void courseDeviation(Canvas& c, const Dial& d, const InstrumentState& s);
void glideslope(Canvas& c, const Dial& d, const InstrumentState& s);
bool drawsProjectedTrack(const InstrumentState& s);
// `fx` (optional) supplies display fades; null = draw the raw state.
void projectedTrack(Canvas& c, const Dial& d, const InstrumentState& s, const DisplayFx* fx = nullptr);
void alignmentBox(Canvas& c, const Dial& d, const InstrumentState& s, const DisplayFx* fx = nullptr);
void guidanceChevrons(Canvas& c, const Dial& d, const InstrumentState& s, const DisplayFx* fx = nullptr);
void fixedMarkers(Canvas& c, const Dial& d);
void aircraftSymbol(Canvas& c, const Dial& d);
void headingBox(Canvas& c, const Dial& d, const InstrumentState& s);
void failureFlags(Canvas& c, const Dial& d, const InstrumentState& s);

// Side panel for one axis, inside `r` (canvas units); `f` gives text scale.
void sidePanel(Canvas& c, const Frame& f, const Rect& r, const InstrumentState& s, Axis axis);
// Collapsed form of a side panel: a slim edge tab keeping the measured value
// visible, with an amber border when that axis has a caution.
void collapsedTab(Canvas& c, const Frame& f, const Rect& r, const InstrumentState& s, Axis axis);

}  // namespace navsight::render::layers
