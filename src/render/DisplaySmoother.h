#pragma once

#include "nav/InstrumentState.h"

// Display-only motion smoothing and cue fades.
//
// Navigation logic stays unfiltered beyond what it needs (so tests and
// decisions are exact); this layer only eases what is *drawn*:
//  - needle / strip / GS diamond / projected track / course and bearing
//    pointers move with a critically damped response (~0.15 s, no overshoot);
//  - chevrons and the alignment box fade in/out over ~0.2 s instead of popping.
// Safety rules: anything that becomes invalid disappears immediately (no
// lingering stale guidance), and source changes / large jumps snap instead of
// gliding.
namespace navsight::render {

// Per-frame cue state for the renderer (ViewOptions::fx). Null = draw raw.
struct DisplayFx {
    struct Cue {
        bool present = false;      // a chevron cue is (or is fading) on screen
        int direction = 0;
        nav::Strength strength = nav::Strength::Slight;
        bool ease = false;         // amber
        float alpha = 0.0f;
    };
    Cue lateralChevrons;
    Cue verticalChevrons;
    float alignedAlpha = 0.0f;
    float trackAlpha = 1.0f;       // projected track fade-in
};

class DisplaySmoother {
public:
    struct Params {
        double motionTauSec = 0.15;  // overall settling time constant
        double fadeSec = 0.2;
        double snapDots = 1.5;       // larger per-frame jumps snap
    };

    DisplaySmoother() = default;
    explicit DisplaySmoother(Params p) : p_(p) {}

    // Returns the state to draw; fills `fx`. dtSec = real (wall-clock) frame time.
    InstrumentState apply(const InstrumentState& raw, double dtSec, DisplayFx& fx);
    void reset() { have_ = false; }

private:
    struct Filt {  // two cascaded first-order stages = critically damped
        double a = 0.0, b = 0.0;
        void snap(double v) { a = b = v; }
        double step(double target, double alpha) {
            a += alpha * (target - a);
            b += alpha * (a - b);
            return b;
        }
    };
    struct AngleFilt {
        Filt f;
        double unwrapped = 0.0;
        void snap(double deg) { unwrapped = deg; f.snap(deg); }
        double step(double deg, double alpha);
    };

    void fadeCue(DisplayFx::Cue& cue, const GuidanceAxis& axis, double dt) const;

    Params p_;
    bool have_ = false;
    std::string identity_;
    Filt latPos_, latPred_, vertPos_, vertPred_;
    AngleFilt course_, bearing_;
    std::array<Filt, nav::LateralPrediction::kSteps + 1> pathCross_{}, pathAlong_{};
    bool trackShown_ = false;
    DisplayFx fx_;
};

}  // namespace navsight::render
