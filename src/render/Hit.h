#pragma once

// What a click at a point acts on (InstrumentRenderer::hitTest). Kept in its
// own header so the runway picker state machine can use it without depending
// on the renderer.
namespace navsight::render {

enum class Hit {
    None,
    TogglePanels,       // a side panel, or a collapsed edge tab
    OpenRunwayPicker,   // the APPROACH / SOURCE box
    // Runway picker (only while it is open)
    PickerCandidate0,
    PickerCandidate1,
    PickerCandidate2,
    PickerSideNone,
    PickerSideL,
    PickerSideC,
    PickerSideR,
    PickerAuto,         // clear the override
    PickerApply,
    PickerInside,       // inside the picker, not on a button
    PickerOutside       // anywhere else: closes the picker
};

}  // namespace navsight::render
