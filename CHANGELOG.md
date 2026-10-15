# Changelog

All notable changes to NavSight (formerly SmartCDI). Dates are 2026.

## 1.0.0 — First public release (Oct 15)

### Changed
- The course needle, course pointer and glidepath indicator follow the
  navigation source, as on common glass-cockpit avionics: **green** for ground-based
  receivers (NAV1/NAV2: ILS, LOC, VOR), **magenta** for GPS. The green is a
  distinct yellow-green so it stays separable from the runway strip's edges.

### Added
- **Nav source check.** NavSight cross-checks the HSI source, both NAV
  radios, the HSI course and the approach loaded in GPS1, and says when the
  setup is wrong:
  - `GPS APPROACH NOT ACTIVE – ACTIVATE APPR`: an approach is loaded and
    you're established on its final, but it isn't activated;
  - `ILS/LOC APPROACH ON GPS – SELECT NAV1`: on the final with the HSI on GPS
    while a live localizer serves the approach (or `TUNE AND SELECT NAV` if
    none is received, in aircraft whose FMS reports the procedure). This also
    catches an ILS loaded in X-Plane's G1000, which flies it as a GPS overlay
    in **LNAV**. An RNAV approach is recognised by its RNAV-only modes (LPV,
    L/VNAV, LP, …) and is not flagged. Before activation, with the localizer
    alive, the caution reads `GPS APPR NOT ACTIVE – ACTIVATE OR SELECT NAV1`;
  - `RNAV APPROACH ACTIVE – SELECT GPS`: the HSI is on NAV during an active
    RNAV approach;
  - `TUNED ILS RWY 23 – LOADED APPR RWY 20`: the tuned localizer serves
    another runway;
  - `SET COURSE 202° – HSI CRS 270°`: the HSI course isn't set to the
    localizer course.

  Cautions appear in amber in the title box, after 2 s, and clear 1.5 s after
  the fix. A footer chip shows `SRC OK` (green), the caution (amber) or
  `SRC --` (grey). Legitimate setups, such as an ILS monitored during an LPV
  or the RNAV approach kept active as backup on an ILS, are not flagged. The
  check is logged to `NavSight_nav.log` and published as
  `navsight/check/status` and `navsight/check/code`.
- `PAUSED` moved to the bottom left, above the source name.
- `docs/ALGORITHMS_AND_MATH.md`: a technical reference of every algorithm,
  equation, physical model and constant, with worked examples and the
  verification method.

## 0.4.0 — Phase 2 complete (Oct 15)

Milestone D: runway override and final polish.

### Added
- **Manual runway override.**
  - **Picker:** click the APPROACH box (or *Runway Override…*). It suggests
    runway numbers around the selected course, with side L/C/R, typed entry,
    AUTO and APPLY. Keyboard: 0-9 L C R, Enter, Esc.
  - **Commands:** `navsight/runway/picker`, `next`, `previous` and `clear`.
  - **Shown as manual:** amber `MAN` in the header and on the strip. The
    title box shows an amber `CHECK RUNWAY – DOES NOT MATCH COURSE` if the
    runway is more than 30° from the course.
  - **Lifetime:** bound to the nav source it was set on and cleared
    automatically on retune, source switch, approach change or aircraft
    reload. It is never saved.
  - **Validation unaffected:** applying an override never disturbs nav
    validation, and the angular scale still uses the real receiver type.
- **Brightness** Day / Dim / Night (menu submenu, commands
  `navsight/view/brightness[_day|_dim|_night]`, config `brightness`).
- **Anti-aliasing:** feathered 1-pixel edges on lines, circles, rings and
  shapes, with stroke-weight compensation so text keeps its weight.
- **Smooth motion:** needle, glideslope diamond, strip, course and bearing
  pointers, and the projected track ease with a critically damped ~0.15 s
  response. Chevrons and the alignment box fade in and out. Invalid data
  disappears immediately; source changes and jumps snap.
- **Performance:** OpenGL vertex-array drawing (one call per frame) with safe
  VBO/VAO save and restore and an immediate-mode fallback (config
  `fast_rendering`). Verbose mode logs per-frame cost to the nav log.
- **Packaging:** `build.ps1 -Package` creates
  `dist/NavSight-<version>-win_x64.zip` (no PDB).
- **Docs:** `docs/USER_GUIDE.md`, this changelog, and `docs/ACCEPTANCE.md`.

## 0.3.0-c — Milestone C: trajectory prediction (Oct 14)
- New `TrajectoryPredictor`:
  - inputs: ground track, groundspeed, turn rate and vertical speed;
  - a 20 s projected track on the dial;
  - standard-rate roll-out lead (`TURN IN` / `TURN NOW`, overshoot);
  - a track-based lateral command law;
  - LOC/VOR scale self-calibration;
  - required VS and path overshoot.
- Collapsible side panels (click, menu or command), with caution-bordered
  edge tabs.
- Header `CRS / BRG / TRK`.

## 0.3.0-b.1 — GPS approaches, distance, nav log (Oct 14)
- Active GPS approaches (from `gps_cdi_sensitivity`) give runway alignment
  mode, with the runway and airport taken from the loaded approach plan.
- GPS distance is shown as `DIST … TO <waypoint>`; ground DME as `DME`.
- Nav diagnostics moved to `Resources/plugins/NavSight/NavSight_nav.log`.

## 0.3.0-b — Milestone B: verified live navigation (Oct 14)
- **Fixed:** the glideslope was inverted (confirmed against X-Plane's G1000).
- New `NavValidator` statuses: valid / no signal / flagged / acquiring / full
  scale / not established. Measured values are always shown; calculated
  cues only when valid.
- Groundspeed gate for calculated cues.

## 0.3.0-a — Milestone A: visual foundation (Oct 14)
- Square 600×600 layout matching the reference.
- Flat 2-D runway-alignment strip, plus a course-reference band when the
  runway is unknown. Runway numbers are no longer invented.
- Refined aircraft symbol and condensed typography.
- Renamed SmartCDI → NavSight (settings migrated; old plugin folder removed
  on install).

## 0.2.0 — Full instrument (Oct 14)
- Runway overlay, CDI needle, glideslope, LATERAL/VERTICAL panels with
  rate-based guidance, approach identification from the nav database.

## 0.1.0 — Foundation (Oct 14)
- Native XPLM plugin: movable, resizable window; compass rose; aircraft
  symbol; live magnetic heading; menu, commands, settings and logging.
