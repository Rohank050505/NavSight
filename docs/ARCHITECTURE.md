# NavSight — Architecture and Design

How NavSight is built and why. For the math behind each algorithm see
[ALGORITHMS_AND_MATH.md](ALGORITHMS_AND_MATH.md); to build it, see
[BUILDING.md](BUILDING.md).

## Contents

- [Implementation approach](#implementation-approach)
- [Architecture](#architecture)
- [Design decisions](#design-decisions)
- [Guidance and prediction model](#guidance-and-prediction-model)
- [Development notes](#development-notes)

## Implementation approach

**Native C++ plugin on the X-Plane SDK 4.3.0 (XPLM).** XLua was evaluated and
rejected:

| Requirement | XLua | Native XPLM |
|---|---|---|
| Aircraft-independent | ✗ scripts live per aircraft | ✓ global plugin |
| Standalone movable/resizable window | ✗ no documented drawing API | ✓ `XPLMCreateWindowEx`, resize limits, pop-out |
| Custom vector graphics | ✗ | ✓ OpenGL in draw callbacks (behind a backend-neutral `Canvas`) |
| 3-D cockpit route | per-aircraft only | ✓ XPLM400 Avionics API (future host) |
| Companion app | ✗ | ✓ |

**Drawing.** X-Plane 12 renders with Vulkan but still supports plugin OpenGL
in window and avionics callbacks. Laminar's announced "Panel Graphics" API is
not in SDK 4.3. All drawing therefore goes through `render/Canvas`, so a new
backend is a single class.

## Architecture

```
 X-Plane datarefs, nav DB,          ┌─────────────── nav/ (pure C++) ───────────────┐
 approach flight plan, both radios  │ NavValidator ─► RunwayOverride ─► AxisGuidance │
        │                           │        │                         (trend, rate) │
 data/XPlaneDataSource              │        └──────► TrajectoryPredictor            │
 data/ApproachResolver ─SimSnapshot─►         (track, GS, turn rate, VS, scale)      │
                                    │ SourceCheck     InstrumentModel                │
                                    └───────────────────────┬───────────────────────┘
                                                     InstrumentState
                                                            ▼
 ui/InstrumentWindow ── DisplaySmoother (motion, fades) ─► render/InstrumentRenderer
   (XPLM window: clicks,  ViewOptions: panels, brightness,    layers ─► Canvas ─► GlCanvas
    keys, picker)         runway picker, fx                   (feathered triangles)   (vertex arrays)
 app/NavSightApp — lifecycle, flight loop, commands, menu, config, datarefs, logs
 plugin/PluginMain — XPlugin* exports + exception firewall
```

| Layer | Folder | X-Plane SDK? | Responsibility |
|---|---|---|---|
| Plugin entry | `src/plugin` | yes | `XPlugin*` exports only |
| Application | `src/app` | yes | Lifecycle, flight loop, commands, menu, published datarefs, config and log I/O |
| Data | `src/data` | yes | The **only** dataref, nav-database and flight-plan reader. Produces `SimSnapshot` |
| Navigation | `src/nav` | **no** | Validation, runway override, guidance, prediction, source check → `InstrumentState`. Pure and tested |
| Rendering | `src/render` | only `GlCanvas` | Layout, layers, text, themes, display smoothing, picker state |
| Hosts | `src/ui` | yes | The window (later: an avionics device) hosting the renderer |
| Core | `src/core` | no | Logging, nav log, config, version |

`SimSnapshot` (data → nav) and `InstrumentState` (nav → render) are plain
structs, so nav and render run headless in tests and later in a replay or
companion app.

### Source map

```
src/
├── plugin/PluginMain.cpp
├── app/NavSightApp            lifecycle, commands, menu, override/brightness wiring, logs
├── core/                      Version.h, Log, NavLog (NavSight_nav.log), Config
├── data/                      SimSnapshot.h, SimDataSource.h, DataRef, XPlaneDataSource,
│                              ApproachResolver (nav DB, GPS approach plan, fix kinds)
├── nav/                       Angles.h, InstrumentState.h, InstrumentModel,
│                              NavValidation, AxisGuidance, TrajectoryPredictor,
│                              RunwayOverride, SourceCheck, ApproachNaming
├── render/                    Canvas (+ feathering), GlCanvas, StrokeFont, RenderKit,
│                              Palette (themes), InstrumentRenderer, Layers.h,
│                              DialLayers, PanelLayers, GuidanceText,
│                              DisplaySmoother, RunwayPicker, Hit.h
└── ui/InstrumentWindow
tests/TestMain.cpp             1363 checks: unit, kinematic flight sim, headless renders
docs/                          USER_GUIDE, BUILDING, ARCHITECTURE, ALGORITHMS_AND_MATH, ACCEPTANCE, previews/
```

## Design decisions

- **Measured vs calculated.**
  - Magenta / white = measured (needle, diamond, deviation number). The dial
    needle and glidepath indicator are **green** for ground-based NAV sources
    (ILS / LOC / VOR) and **magenta** for GPS, as on common glass-cockpit avionics.
  - Cyan = calculated (advice, projected track, predictions).
  - Amber = caution.
  - Red = failure flags only.
- **Navigation validation** (`nav/NavValidation`). Each axis is classified
  every frame as VALID / NO SIGNAL / FLAGGED / ACQUIRING / FULL SCALE / NOT
  ESTABLISHED. Measured values show whenever plausible; calculated cues only
  when VALID and groundspeed ≥ 30 kt. Source changes and jumps of more than
  1 dot reset all predictors.
- **Deflection signs (confirmed in-sim against X-Plane's G1000).**
  - `hdef > 0` means the aircraft is **left** of course.
  - `vdef > 0` means the aircraft is **above** the path.

  The two axes have opposite senses. A regression test is built from the
  user's screenshots, and `invert_*` config exists only as a per-aircraft
  escape hatch.
- **Approach identification.**
  - **NAV:** `XPLMFindNavAid` finds the nearest ILS/LOC/VOR on the tuned
    frequency (≤ 120 NM); the runway comes from the navaid name, the airport
    from the name or the nearest airport.
  - **GPS:** active when `gps_cdi_sensitivity` is an approach mode; runway and
    airport come from the loaded approach plan (SDK 4.1 flight-plan API).
  - **No invention:** runway numbers are **never** estimated. When none is
    verified, the strip becomes a course-reference band.
- **Runway override** (`nav/RunwayOverride`, `render/RunwayPicker`).
  - The pilot declares the runway. It is tagged `MAN` everywhere, and
    mismatches of more than 30° from the course raise an amber caution.
  - It is bound to the source identity (source, frequency, approach label) it
    was set on and dropped when that changes.
  - It is applied to a *display copy* of the approach info: validation and
    receiver physics keep using the real data, so applying it never causes a
    false discontinuity.
- **Nav source check** (`nav/SourceCheck`). Cross-checks the HSI source,
  both NAV receivers (read regardless of the selector), the HSI course and
  GPS1's loaded approach and mode. Five rules in priority order:
  `GPS_ON_LOC`, `NAV_ON_RNAV`, `APPR_NOT_ACTIVE`, `LOC_NE_APPR`, `CRS_NOT_SET`.
  *Established on final* is computed geometrically from the plan's FAF →
  threshold segment (≤ 12 nm, ≤ 1 nm cross-track, track within 30°). Findings
  are debounced (2.0 s to show, 1.5 s to clear) and evaluated on the real,
  un-overridden data. Legitimate look-alikes (an ILS monitored during an LPV,
  RNAV backup on an ILS) are deliberately silent. Shown as a footer chip and an
  amber title-box banner; details in
  [ALGORITHMS_AND_MATH.md §10](ALGORITHMS_AND_MATH.md#10-nav-source-cross-check).
- **Fail-soft and firewall.** Missing datarefs degrade to flags. No C++
  exception crosses into X-Plane. Static CRT (`/MT`), so the `.xpl` depends
  only on `XPLM_64.dll`, `OPENGL32.dll` and `KERNEL32.dll`.
- **Vector stroke font.** It scales with the window and rotates with the
  card. `XPLMDrawString` can do neither, and isn't available in avionics
  contexts.
- **Themes.** `render/Palette` generates Day / Dim / Night from one X-macro
  colour table with luminance classes (white symbols dimmed most, coloured
  symbols less, fills least). The renderer selects the theme per draw.
- **Display smoothing** (`render/DisplaySmoother`). Display only, so nav
  logic and tests stay exact. A critically damped response (two cascaded
  first-order stages, ~0.15 s, no overshoot) runs on wall-clock time. Cue
  fades are 0.2 s. Invalid data vanishes at once; source changes, gaps and
  jumps over 1.5 dots snap.
- **Anti-aliasing.** `Canvas` adds a 1-pixel fringe fading to transparent
  around lines, circles, rings and convex shapes, and pulls the core in by
  half the fringe so stroke weight is unchanged.
- **GL backend.** One `glDrawArrays` from client memory per frame.
  `GL_ARRAY_BUFFER` and VAO bindings are saved, unbound and restored via
  `wglGetProcAddress`, with an immediate-mode fallback (`fast_rendering 0`).

## Guidance and prediction model

All of this is estimated, situational-awareness guidance, never bank or pitch
commands.

**Rate-based fallback** (`nav/AxisGuidance`, used when no trustworthy
trajectory scale exists). The filtered deviation rate is compared with an
ideal exponential closure over `T` (20 s lateral, 10 s vertical):
`error = (−dev/T − rate)·T` in dots. The result is `ON COURSE`, `HOLD`,
`TURN/CLIMB …` (graded) or `EASE OFF`, and trend and intercept time come from
the same filter. Hysteresis is applied throughout.

**Trajectory prediction** (`nav/TrajectoryPredictor`).

| Input | Source |
|---|---|
| Track-angle error TKE = track − course | `ground_track_mag_pilot`, selected course |
| Groundspeed, turn rate ω | `groundspeed`; filtered d(track)/dt (1 s), yaw rate `R` as a fallback, ±6°/s |
| Scale (nm/dot) | GPS: `gps_hdef_nm_per_dot` (**MEASURED**). LOC/VOR: `d·tan(deg/dot)`, with d from the antenna position (nav DB) or DME. deg/dot starts at a **PRIOR** (LOC 1.25, VOR 4.0) and is **CALIBRATED** in flight |
| VS, path angle γ | `vh_ind_fpm`; ILS glideslope navaid (angle×100000+course, accepted 2–6°), GPS `fms_vpa_pilot`, else 3.0° assumed |

```
Projection (20 s, 0.5 s steps; constant GS and ω):
  x(t) = x0 + ∫ GS·sin(TKE + ω·τ) dτ      along(t) = ∫ GS·cos(TKE + ω·τ) dτ
  dots = x / nm_per_dot (GPS)   |   atan(x / (d0 ∓ along)) / deg_per_dot (LOC/VOR)
Roll-out lead (standard rate 3°/s or the current rate if higher):
  R = GS/ω_std,  L = R·(1 − cos|TKE|)
  |x| > L → TURN IN (|x|−L)/closure s     |x| ≤ L → TURN NOW, overshoot (L−|x|)
Lateral command (CALIBRATED/MEASURED only):
  TKE* = −sign(x)·min(30°, 30°/nm·|x|); toward-course intercepts from TKE* up to
  max(TKE*+15°, 45°) → HOLD; otherwise TURN … graded 4/10/20° (1.5° hysteresis)
Self-calibration (straight flight, |ω| ≤ 1°/s, |TKE| ≥ 5°):
  θ' = (x'·d − x·d')/(d² + x²), x' = GS·sin TKE, d' = ∓GS·cos TKE
  deg/dot = θ'/measured dots·s⁻¹, log-space filter 4 s, CALIBRATED after 4 s
Vertical:
  required VS = −GS·101.27·tan γ        target = required − sign(dev)·min(400, 150·|dev|)
  HOLD within ±120 fpm; graded 250/500 fpm; PATH OVERSHOOT if crossing ≤ 20 s with |VS err| ≥ 250
```

**Limits.**
- No model of wind or speed changes, or of the pilot's next input.
- LOC/VOR predictions stay dim and command-free until calibrated.
- The projected track is drawn isotropically (true direction), so on fine
  scales it is clipped at the dial edge.

## Development notes

See [CHANGELOG.md](../CHANGELOG.md) for per-version detail. Key in-sim findings
that shaped the design:

- **M1:** the heading readout matched the G1000 (089°). The `A` glyph was
  illegible at small sizes and has been redrawn.
- **M2 / B:** the vertical deflection sign was inverted (KMWA ILS 20 G1000
  pixel crops: diamond 1.7 dots *below* while NavSight said LOW). Fixed and
  pinned by a test. Lateral was correct.
- **B follow-up:** a GPS approach flying to `VANNU` (LPV) showed course
  reference. Detection now uses the GPS approach mode and the approach plan
  instead of waypoint names. A `freq=1` "VOR" label from an unset NAV2 led to
  strict frequency validation.
- **C reviews:**
  - a normal 30° intercept was being told `EASE OFF`; intercept law relaxed,
    with timing left to the roll-out lead;
  - the compressed projected-track drawing pointed about 13° off the true
    track; drawing made isotropic;
  - "CLIMB" while above a descending path became `LESS DESCENT`.
- **D:**
  - feathering made strokes look bolder; the core is now pulled in by half
    the fringe;
  - runway candidates for an exactly-halfway course (205°) round up (20/21/22),
    and the localizer course (e.g. 202°) gives the expected 19/20/21.
