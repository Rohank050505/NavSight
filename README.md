# NavSight

A predictive 2-D navigation instrument plugin for **X-Plane 12** (Windows x64).
NavSight shows where the aircraft is relative to the procedure, where its
current trajectory leads, and whether it is converging, diverging or about to
overshoot. It does this with a heading-up rose, a runway-alignment strip, an
HSI needle and glideslope, a 20-second projected track, and LATERAL / VERTICAL
guidance panels. A **nav source check** also warns when the setup doesn't fit
the procedure: GPS selected on a localizer approach, an approach loaded but
not activated on final, or the HSI course not set.

- **Users:** [docs/USER_GUIDE.md](docs/USER_GUIDE.md) (install, reading the
  instrument, source check, runway override, brightness, settings,
  troubleshooting).
- **Changes:** [CHANGELOG.md](CHANGELOG.md).
- **Algorithms, math and physics (technical deep-dive):**
  [docs/ALGORITHMS_AND_MATH.md](docs/ALGORITHMS_AND_MATH.md).
- **Phase 2 acceptance evidence:** [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md).
- **Renders:** [docs/previews/](docs/previews/).

> Simulation use only. NavSight is not certified avionics or a flight
> director.

**Status — v1.0.0, first public release (Phase 2 complete).** Milestones A (visual foundation), B
(verified live navigation) and C (trajectory prediction) are done. So is D
(runway override, anti-aliasing, smoothing, brightness, performance,
packaging). The 1.0.0 release adds the nav source check. Test mode is deferred to a later release. A few behaviours still
await in-sim confirmation (see the acceptance matrix).

*Formerly **SmartCDI** (renamed in 0.3.0-a). Settings migrate automatically,
`build.ps1 -Install` removes the old plugin folder, and commands and datarefs
moved from `smartcdi/…` to `navsight/…`.*

---

## Contents

- [Implementation approach](#implementation-approach)
- [Architecture](#architecture)
- [Design decisions](#design-decisions)
- [Guidance and prediction model](#guidance-and-prediction-model)
- [Building, testing, packaging](#building-testing-packaging)
- [Installing](#installing)
- [Debugging](#debugging)
- [Roadmap](#roadmap)
- [Development notes](#development-notes)

---

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
docs/                          USER_GUIDE.md, ACCEPTANCE.md, previews/
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
  [ALGORITHMS_AND_MATH.md §10](docs/ALGORITHMS_AND_MATH.md#10-nav-source-cross-check).
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

## Building, testing, packaging

**Requirements:**
- Windows 10/11 x64.
- Visual Studio 2022+ with *Desktop development with C++* (verified with VS
  2026 18.11 / MSVC 14.51).
- X-Plane SDK 4.3.0 in `third_party/XPSDK`; `.\build.ps1 -FetchSdk`
  downloads the official zip.
- CMake is optional (`CMakeLists.txt` for IDE/CI use; not verified here).

```powershell
.\build.ps1                          # Release -> build\Release\NavSight\win_x64\NavSight.xpl
.\build.ps1 -Test                    # + 1363 unit / simulation / headless-render checks
.\build.ps1 -Configuration Debug     # SDK error callback, verbose logs
.\build.ps1 -Test -Package           # dist\NavSight-<version>-win_x64.zip (no PDB)
```

The script finds Visual Studio via `vswhere`, compiles `src/` at `/W4`, links,
and checks the five `XPlugin*` exports.

The tests (`tests/TestMain.cpp`, no X-Plane needed) cover:
- angles and the config;
- validation statuses;
- the guidance law;
- a **kinematic flight simulator** for the trajectory predictor (projection
  accuracy, `TURN NOW` geometry, calibration);
- the sign-convention regression;
- approach naming and GPS modes;
- runway override and the picker state machine;
- the source check: final-approach geometry, every rule firing and staying
  silent on its legitimate look-alikes, priority, debounce, chip and banner
  rendering;
- display smoothing and fades;
- themes;
- feathered geometry;
- hit-testing;
- headless renders of every mode, layout and variant, checked for bounds and
  NaNs.

## Installing

`.\build.ps1 -Install` (X-Plane closed):
- auto-detects X-Plane via `%LOCALAPPDATA%\x-plane_install_12.txt`, or takes
  `-XPlaneDir`;
- removes a legacy `SmartCDI` plugin folder;
- copies to `Resources\plugins\NavSight\win_x64\`.

For the release zip, see the [user guide](docs/USER_GUIDE.md#1-installing).

## Debugging

**Logs:**
- `X-Plane 12\Log.txt`: lifecycle, settings, dataref binding, errors
  (`NavSight [LEVEL]`).
- `Resources\plugins\NavSight\NavSight_nav.log` (previous session in
  `.prev.log`), timestamped and flushed per line:
  - `nav source resolved`, `gps approach active`, `navaid lookup`,
    `glideslope lookup`;
  - `nav validity` / `nav discontinuity`;
  - trajectory confidence, `TURN NOW`, vertical overshoot, glidepath angle;
  - runway override set / cleared / mismatch;
  - `source check:` on every change (status, code, banner, and context: HSI
    source, GPS mode, both radios, loaded approach, OBS, distance and
    cross-track to the final);
  - with verbose logging, 10 s nav samples and `perf:` lines (update ms,
    draw build ms, GL submit ms, triangles, GL path).

**Published datarefs** (read-only; DataRefEditor/DataRefTool are notified):
- `navsight/state/display_heading_deg`
- `navsight/state/heading_valid`
- `navsight/state/mode`
- `navsight/guidance/lateral_dots`
- `navsight/guidance/vertical_dots`
- `navsight/guidance/aligned`
- `navsight/guidance/lateral_status`
- `navsight/guidance/vertical_status`
- `navsight/check/status` (source check: 0 n/a, 1 OK, 2 caution)
- `navsight/check/code` (0 none, 1 GPS_ON_LOC, 2 NAV_ON_RNAV, 3 APPR_NOT_ACTIVE, 4 LOC_NE_APPR, 5 CRS_NOT_SET)
- `navsight/window/visible`

**Visual Studio:**
1. Run `.\build.ps1 -Configuration Debug -Install`.
2. Start X-Plane through Steam.
3. *Debug → Attach to Process → X-Plane.exe*. The PDB is installed next to
   the `.xpl`.

## Roadmap

## Development Roadmap

### v1.1 — Validation & Testing
- [ ] Implement a dedicated test mode using scripted or recorded `SimSnapshot` data.
- [ ] Create repeatable IFR scenarios for course interception, overshoot, and glidepath tracking.
- [ ] Expand in-simulator validation across aircraft types, navigation sources, wind conditions, and intercept angles.
- [ ] Improve predictive guidance accuracy, uncertainty handling, and navigation-data validation.
- [ ] Document prediction accuracy and known limitations.

### v1.2 — Advanced Procedure Visualization
Expand NavSight beyond final approach guidance to visualize holding patterns, DME arcs, and complex instrument procedure geometry.

**Holding Pattern Visualization**
- [ ] Display holding-pattern geometry, including inbound/outbound legs, holding fix, and turn direction.
- [ ] Visualize direct, parallel, and teardrop holding entries.
- [ ] Display the aircraft's current position and projected trajectory relative to the holding pattern.
- [ ] Incorporate wind correction and drift prediction.
- [ ] Anticipate inbound course interception, outbound timing, and upcoming turns.
- [ ] Provide predictive guidance for maintaining the desired holding pattern.

**DME Arc Visualization**
- [ ] Display the desired DME arc relative to its navigation station.
- [ ] Visualize aircraft position, ground track, and projected trajectory along the arc.
- [ ] Calculate radial distance deviation from the selected arc radius.
- [ ] Predict whether the aircraft is converging toward or diverging from the arc.
- [ ] Anticipate arc interception and potential overshoot.
- [ ] Visualize lead turns and transitions between the arc and subsequent approach segments.
- [ ] Incorporate groundspeed, turn rate, and wind effects into trajectory prediction.

**Procedure Integration**
- [ ] Introduce dedicated display modes for holding patterns and DME arcs.
- [ ] Expand `ApproachResolver` to identify supported procedure legs and geometry.
- [ ] Allow manual procedure configuration when reliable simulator data is unavailable.
- [ ] Add navigation validity checks and automated geometry tests for both procedure types.

### v1.3 — Flight Performance Analytics
- [ ] Record aircraft trajectories, navigation deviations, and instrument procedure events.
- [ ] Develop a dedicated 2-D approach and procedure debriefing interface.
- [ ] Calculate lateral and vertical tracking accuracy.
- [ ] Identify course interception errors, overshoots, and unstable approach conditions.
- [ ] Analyze holding-pattern accuracy and DME arc tracking performance.
- [ ] Compare performance across navigation sources, procedures, and aircraft.
- [ ] Export recorded flight data for external analysis and research.

### v2.0 — Cockpit & Companion Application Integration
- [ ] Develop a native 3-D cockpit instrument using the XPLM400 Avionics API.
- [ ] Reuse the existing `InstrumentRenderer` through a dedicated avionics display host.
- [ ] Create a companion desktop application for instrument configuration and flight analysis.
- [ ] Implement UDP/WebSocket communication for streaming navigation and instrument state.
- [ ] Develop a compact, text-only supplementary overlay for traditional cockpit instruments.
- [ ] Support external monitors and home cockpit hardware.
- [ ] Introduce customizable instrument layouts and display preferences.

### Future Development — Compatibility & Research
- [ ] Expand compatibility with custom avionics and aircraft-specific data sources.
- [ ] Support copilot-side navigation data and independent instrument configurations.
- [ ] Investigate alternative rendering backends, including the X-Plane Panel Graphics API.
- [ ] Explore macOS and Linux compatibility.
- [ ] Investigate advanced trajectory prediction incorporating aircraft performance characteristics.
- [ ] Conduct human-factors research comparing conventional navigation instruments with NavSight.
- [ ] Evaluate improvements in situational awareness, course interception accuracy, and pilot workload.

## Development notes

See [CHANGELOG.md](CHANGELOG.md) for per-version detail. Key in-sim findings
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
