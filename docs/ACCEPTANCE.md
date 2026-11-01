# Phase 2 acceptance matrix (v1.0.0)

This matrix maps each acceptance criterion in the Phase 2 brief (§12) to its
evidence. Test mode (part of Milestone D in the brief) was deferred to a later
release at the user's request.

**Evidence key:**
- **In-sim**: confirmed by the user in X-Plane 12.4.4 (screenshots or report).
- **Tests**: automated checks in `tests/TestMain.cpp` (`.\build.ps1 -Test`).
- **Renders**: offline renders through the real renderer, in `docs/previews/`.

| # | Criterion | Evidence | Status |
|---|---|---|---|
| 1 | Visually resembles the reference instrument | User approved Milestone A. Renders: `milestone-a/`, `milestone-c/`, `milestone-d/` | ✅ In-sim + renders |
| 2 | Compass rose accurately reflects magnetic heading | M1 in-sim: readout matched the G1000 (089°). Tests: wrap/smoothing across north, readout 000→360 | ✅ In-sim + tests |
| 3 | Runway visualization responds correctly to lateral deviation | User screenshots (KMWA ILS 20 / RNAV 20) show the strip offset matching the G1000 CDI side. Tests: renderer bounds with deviation up to beyond full scale; the strip uses the same dot scale as the needle | ✅ In-sim + tests |
| 4 | Valid lateral and vertical guidance displayed accurately | Lateral sign confirmed in-sim (M2). Vertical sign bug found by the user and fixed in 0.3.0-b; the user confirmed the fix ("needed changes were done correctly"). Tests: regression pinned with the screenshot values | ✅ In-sim + tests |
| 5 | Distinguishes converging, diverging and stable | Tests: `AxisGuidance` trend with hysteresis; trajectory outcomes (CONVERGING / PARALLEL / DIVERGING / CROSSING+OVERSHOOT). User screenshot shows CONVERGING | ✅ In-sim + tests |
| 6 | Predictive guidance responds to changes in trajectory | Tests: the kinematic flight simulator checks projection accuracy (±0.02–0.1 dot), the turn-rate-aware projection, `TURN NOW` before the needle centres, track-based commands that differ from the rate law, and LOC calibration within 10% | ✅ Tests · ⏳ in-sim check of Milestone C pending |
| 7 | Invalid navigation information is clearly flagged | Tests: the `NavValidator` suite (no signal, sentinel, flags, full scale, not established, acquiring, low groundspeed). Red `NAV` / `LOC` / `GPS` / `GS` / `HDG` flags. Renders of screenshot 2 (full scale / not established). Since 1.0.0, a wrong source or procedure setup is also flagged (amber source-check banner and chip, below) | ✅ Tests + renders · ⏳ in-sim spot check |
| 8 | Movable, resizable and stable | M1 in-sim (move, resize, pop-out, persistence). Exception firewall on every callback. Tests: hit-testing, layouts, letterboxing | ✅ In-sim + tests |
| 9 | Uses real simulator data in normal operation | All data comes through `data/XPlaneDataSource` (DataRefs.txt-verified datarefs) and the SDK nav database / flight plan. No demo values in the plugin; synthetic data exists only in `tests/` and scratch preview tools. The runway override is explicit, user-initiated and tagged `MAN`. The source check reads both NAV receivers, the HSI course and GPS1's loaded approach from the same data layer, and always uses the real (un-overridden) data | ✅ Code review |
| 10 | Predictive calculations documented and independently testable | [ARCHITECTURE.md](ARCHITECTURE.md#guidance-and-prediction-model) and [ALGORITHMS_AND_MATH.md](ALGORITHMS_AND_MATH.md) (equations, assumptions, limits). `nav/TrajectoryPredictor`, `nav/AxisGuidance`, `nav/NavValidation`, `nav/RunwayOverride` and `nav/SourceCheck` are pure C++ with no X-Plane dependency, covered by 1363 checks | ✅ |

## 1.0.0 addition: nav source check

Added before the first public release at the user's request (outside the
Phase 2 brief). Design and math: [ALGORITHMS_AND_MATH.md §10](ALGORITHMS_AND_MATH.md#10-nav-source-cross-check).
User guide: [USER_GUIDE.md, *Source check*](USER_GUIDE.md#source-check).

| # | Requirement | Evidence | Status |
|---|---|---|---|
| S1 | Approach loaded but not activated while on the final course → caution | `APPR_NOT_ACTIVE`: tests for the final geometry (distance, cross-track, track, GS, variation) and for each GPS mode (ENR/TERM fire; LPV, LNAV, APR, MAPR don't). Render `release/9-source-appr-not-active.png` | ✅ Tests + renders · ⏳ in-sim |
| S2 | Wrong source for the procedure, both directions → caution | `GPS_ON_LOC` (live ILS/LOC serves the approach, HSI on GPS) and `NAV_ON_RNAV` (RNAV approach active, HSI on NAV). Tests include the no-plan fallback, NAV2, and the look-alikes that must stay silent. In-sim findings (KMWA): (1) the G1000 flies a loaded ILS on GPS in **LNAV**, which first hid the caution; (2) `fas_id`/`ltp_id` follow the ILS tuned on NAV1 (`IMWA`/`DME`) even while RNAV 20 was flown in LPV, so using them flagged a correct RNAV approach. Resolved by deciding from RNAV-only GPS modes (LPV, L/VNAV, …) and using only a procedure *name* from the FMS. Both flights are regression tests with their logged values. **Confirmed in-sim** for case 1 (ILS on GPS → caution). Renders `release/10-source-gps-on-loc.png`, `release/11-source-nav-on-rnav.png` | ✅ Tests + renders · ✅ in-sim (ILS) · ⏳ in-sim (RNAV LPV re-check) |
| S3 | Tuned localizer doesn't serve the loaded approach → caution | `LOC_NE_APPR`: different runway or airport; runway normalisation (`2` = `02`); silent without a signal or a plan | ✅ Tests · ⏳ in-sim |
| S4 | HSI course not set to the localizer course → caution | `CRS_NOT_SET`: > 10° with wrap across north; VORs never flagged; live text update. Render `release/8-source-set-course.png` | ✅ Tests + renders · ⏳ in-sim |
| S5 | Correct setup is shown as correct, nothing to check is shown as such | Green `SRC OK` / grey `SRC --` chip. Render `release/7-source-check-ok.png` | ✅ Tests + renders |
| S6 | No flicker while switching sources | Debounce tests: 2.0 s to show, 1.5 s to clear, short blips never shown, sim-time reset | ✅ Tests |
| S7 | Display fits every layout | Chip and banner checked inside the viewport (standard, compact, letterboxed); compact layout shows the chip text in the header. Render `release/12-source-compact.png` | ✅ Tests + renders |
| S8 | Auditable | `source check:` nav-log line on every change, with context; datarefs `navsight/check/status` and `navsight/check/code` | ✅ Code review · ⏳ in-sim log |

## Open in-sim confirmations

These need X-Plane and are listed in the user's test plan:

1. **Milestone C behaviour:**
   - the projected track meets the strip where the aircraft actually
     crosses;
   - `TURN NOW` appears about one turn radius early;
   - LOC calibration (`PRIOR -> CALIBRATED` in the nav log);
   - the glideslope angle decode;
   - `gps_hdef_nm_per_dot` values in LPV.
2. **Milestone D:**
   - the runway picker (open, type, apply, AUTO);
   - the override clearing on a source change;
   - the Day / Dim / Night appearance;
   - the anti-aliased rendering and smoothness;
   - the `perf:` lines in the nav log (verbose mode).
3. Whether X-Plane's G1000 exposes the loaded GPS approach plan
   (`gps approach active: … plan_entries=`).
4. **Source check (1.0.0 hotfix):**
   - RNAV approach loaded but not activated, established on final: amber
     `GPS APPROACH NOT ACTIVE` and chip `APPR NOT ACTV`;
   - HSI on GPS with the same-runway ILS tuned and alive: `SELECT NAV1`;
   - ILS on NAV with the HSI course mis-set: `SET COURSE`;
   - correct setups (ILS on NAV, activated LPV on GPS): green `SRC OK`;
   - the `source check:` lines in the nav log, including the plan runway and
     the `established` / `xtk` values;
   - whether GPS1's `gps_cdi_sensitivity` reports LPV/LNAV or APR for an
     active RNAV approach in the default G1000;
   - ✅ ILS 20 loaded in the G1000 (LNAV) flown with the CDI on GPS:
     `ILS APPROACH ON GPS - SELECT NAV1` (confirmed by the user);
   - RNAV 20 flown in LPV with ILS 20 tuned and alive: `SRC OK`. Before
     activation (TERM), established on final:
     `GPS APPR NOT ACTIVE - ACTIVATE OR SELECT NAV1`.
