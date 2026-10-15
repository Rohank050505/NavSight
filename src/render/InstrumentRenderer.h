#pragma once

#include "nav/InstrumentState.h"
#include "render/Canvas.h"
#include "render/DisplaySmoother.h"
#include "render/Hit.h"
#include "render/Palette.h"
#include "render/RunwayPicker.h"

namespace navsight::render {

// Draws the complete NavSight face into a rectangle. Knows nothing about
// windows, X-Plane, or where the state came from - the same renderer serves
// the floating window, a popped-out OS window, and a future 3-D cockpit
// avionics device.
//
// The instrument is square (as in the reference design) and letterboxed in
// whatever rectangle it is given:
//  - Standard (side >= kStandardMinSide): title box, approach box, dial, and
//    symmetrical LATERAL / VERTICAL panels overlapping the bezel.
//  - Compact (smaller): dial with all overlays and a one-line header, because
//    panel text would be illegible at that size.
class InstrumentRenderer {
public:
    struct Viewport {
        float x = 0.0f, y = 0.0f;  // bottom-left
        float width = 0.0f, height = 0.0f;
    };

    enum class Layout { Standard, Compact };

    // Presentation choices that belong to the viewer, not the nav state.
    struct ViewOptions {
        bool showPanels = true;                     // false = slim edge tabs
        Brightness brightness = Brightness::Day;
        RunwayPicker picker;                        // runway-override picker overlay
        const DisplayFx* fx = nullptr;              // cue fades (null = draw raw)
    };

    using Hit = render::Hit;

    static constexpr float kStandardMinSide = 420.0f;  // canvas units (boxels)

    static Layout chooseLayout(float width, float height);

    void draw(Canvas& canvas, const InstrumentState& state, const Viewport& vp,
              const ViewOptions& view = ViewOptions{}) const;

    // Hit-test with exactly the geometry used by draw(). While the runway
    // picker is open it captures every click (its buttons, inside, outside).
    // Otherwise: the APPROACH box opens the picker; a side panel (or a
    // collapsed tab) toggles the panels.
    static Hit hitTest(const Viewport& vp, float x, float y, const ViewOptions& view);
};

}  // namespace navsight::render
