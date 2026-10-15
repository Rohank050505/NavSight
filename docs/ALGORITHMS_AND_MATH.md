# NavSight — Algorithms, Math and Physics

A technical reference for everything NavSight computes: what goes in, what
comes out, the equations, the constants, and why each design choice was made.
It covers NavSight v1.0.0 (X-Plane 12.4, SDK 4.3). Every number below is the
value used in the code; module names are given so each claim can be traced to
the source (`src/…`).

> NavSight is situational-awareness software for flight simulation. None of
> the algorithms below produce bank or pitch commands, and nothing here is
> certified avionics logic.

---

## Contents

1. [Pipeline at a glance](#1-pipeline-at-a-glance)
2. [Reading the simulator](#2-reading-the-simulator)
3. [Identifying what you're flying](#3-identifying-what-youre-flying)
4. [Trusting the signal: navigation validation](#4-trusting-the-signal-navigation-validation)
5. [Heading display](#5-heading-display)
6. [Rate-based guidance (AxisGuidance)](#6-rate-based-guidance-axisguidance)
7. [Trajectory prediction (TrajectoryPredictor)](#7-trajectory-prediction-trajectorypredictor)
8. [Vertical prediction](#8-vertical-prediction)
9. [Manual runway override](#9-manual-runway-override)
10. [Nav source cross-check](#10-nav-source-cross-check)
11. [Rendering geometry](#11-rendering-geometry)
12. [Display smoothing, fades and themes](#12-display-smoothing-fades-and-themes)
13. [Anti-aliasing and the GPU path](#13-anti-aliasing-and-the-gpu-path)
14. [How it was verified](#14-how-it-was-verified)
15. [Constants reference](#15-constants-reference)
16. [Assumptions and limits](#16-assumptions-and-limits)

---

## 1. Pipeline at a glance

Every simulator frame (X-Plane flight loop, after the flight model):

```
X-Plane datarefs ─┐
nav database ─────┼─► SimSnapshot ─► NavValidator ─► RunwayOverride ─► AxisGuidance (rate, trend)
approach plan ────┘     │                                   │
                        │                                   └─► TrajectoryPredictor (track, GS, turn rate, VS)
                        │                                                │
                        └─► SourceCheck (both radios, OBS, GPS plan) ───►│
                                                                 InstrumentState
                                                                         │ (each draw)
                                     DisplaySmoother ─► InstrumentRenderer ─► Canvas ─► OpenGL
```

- **SimSnapshot** is a plain snapshot of the sim (no X-Plane types).
- **InstrumentState** is everything the display needs, already computed.
- **SourceCheck** reads the raw snapshot (never the runway override) and
  checks that the selected source and procedure fit together (§10).
- The navigation layer (`src/nav`) is pure C++ with no simulator calls, so
  every algorithm here runs headless in the test suite.
- **Two colour channels** carry meaning throughout:
  - **measured** values (needle, glideslope diamond, deviation numbers) are
    magenta or green;
  - **calculated** estimates (advice, projected track, predictions) are
    cyan;
  - amber means caution, and red is reserved for failure flags.

---

## 2. Reading the simulator

### 2.1 Inputs (`data/XPlaneDataSource`)

| Quantity | Dataref | Notes |
|---|---|---|
| Magnetic heading | `sim/flightmodel/position/mag_psi` | Flight-model truth, independent of the aircraft's gyro modelling |
| Ground track (mag) | `sim/cockpit2/gauges/indicators/ground_track_mag_pilot` | What the aircraft actually flies over the ground |
| Groundspeed | `sim/flightmodel/position/groundspeed` | m/s → kt (÷ 0.514444) |
| Vertical speed | `sim/flightmodel/position/vh_ind_fpm` | |
| Yaw rate | `sim/flightmodel/position/R` | Turn-rate fallback |
| Position | `sim/flightmodel/position/latitude/longitude` | For station distances |
| HSI source | `sim/cockpit2/radios/actuators/HSI_source_select_pilot` | 0 NAV1, 1 NAV2, 2 GPS1, 3 GPS2 |
| Lateral / vertical deflection | `…/hsi_hdef_dots_pilot`, `…/hsi_vdef_dots_pilot` | Follow the HSI source selector |
| Signal-present flags | `…/hsi_display_horizontal_pilot`, `…/hsi_display_vertical_pilot` | |
| Course | `navN_course_deg_mag_pilot` (NAV) / `gps_course_degtm` (GPS) | For an ILS this is the **localizer course**, even if the OBS knob is mis-set |
| Bearing, DME, TO/FROM | `…/hsi_bearing_deg_mag_pilot`, `…/hsi_dme_distance_nm_pilot`, `…/hsi_flag_from_to_pilot` | |
| GPS approach mode | `sim/cockpit/radios/gps_cdi_sensitivity` | Enum (see §3.2) |
| GPS CDI scale | `sim/cockpit/radios/gps_hdef_nm_per_dot` | nm per dot (measured) |
| GPS path angle | `sim/cockpit2/radios/indicators/fms_vpa_pilot` | |
| Each NAV receiver | `…/navN_display_horizontal`, `…/navN_hdef_dots_pilot`, `…/navN_flag_from_to_pilot`, `navN_frequency_hz` | Read for both radios regardless of the HSI source (§10) |
| HSI course pointer | `sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot` | Source check (§10) |
| Loaded procedure | `…/gauges/indicators/final_approach_name`, `…/radios/indicators/fas_id`, `…/ltp_id` | Procedure name if the FMS reports it; on the G1000 `fas_id`/`ltp_id` follow the tuned ILS (§10.2) |
| GPS1 approach plan | SDK `XPLMGetFMSFlightPlanEntryInfo` (approach plan) | Runway, threshold, FAF (§10) |

Every dataref was checked against X-Plane 12.4.4's `DataRefs.txt`. If one is
missing, its consumers degrade (a red flag or "no estimate") instead of
crashing.

### 2.2 Sign conventions (verified in the simulator)

X-Plane's dataref descriptions don't state the deflection signs. They were
established by comparing NavSight against X-Plane's own G1000 in pixel crops
of real screenshots (KMWA ILS 20):

| Raw value | Meaning | Evidence |
|---|---|---|
| `hdef > 0` | CDI needle right → aircraft **left** of course | Needle side matched the G1000 in both screenshots |
| `vdef > 0` | Glideslope diamond **below** centre → aircraft **above** the path | G1000 diamond 1.7 dots below centre, `vdef = +1.7` |

The two axes have **opposite senses**. Internally, NavSight converts both to
*aircraft position relative to the path*:

```
lateral  = −hdef      (+ = right of course)
vertical = +vdef      (+ = above the path)
```

A regression test pins these values.

---

## 3. Identifying what you're flying

### 3.1 Radio sources (NAV1/NAV2) — `data/ApproachResolver`

1. **Frequency sanity.** Only 108.00–117.95 MHz counts as a nav frequency.
   An unset radio (the sim reported `freq = 1`) shows nothing rather than a
   false "VOR".
2. **Database lookup.** `XPLMFindNavAid` returns the nearest ILS, LOC or VOR
   on the tuned frequency. A match more than **120 NM** away is rejected,
   because it's a different station on a reused frequency.
3. **Classification.** The navaid type decides ILS / LOC / VOR. If the navaid
   isn't in the database, the frequency itself is used: 108.10–111.95 MHz
   with an **odd tenths digit** is reserved for localizers.
4. **Runway and airport.** These are parsed from the navaid name (e.g. `KSEA
   16L ILS-cat-III`). Failing that, the airport is the nearest airport within
   15 NM of the antenna. **A runway number is never estimated**: if it can't
   be verified, the display shows a course-reference band instead.
5. **Glidepath angle.** The paired glideslope navaid on the same frequency,
   within 5 NM, stores `angle × 100000 + course` in its heading field:

   ```
   heading field 300205.0  →  angle = floor(300205 / 100000) / 100 = 3.00°
   ```

   The angle is accepted only within 2–6°, otherwise 3.0° is assumed and the
   display marks it as assumed.
6. **Caching.** Lookups run only when the source, frequency, ident, signal or
   GPS mode changes, or every 30 s.

### 3.2 GPS sources — approach detection

An approach is **active** when the GPS reports an approach mode in
`gps_cdi_sensitivity`:

| Value | Mode | Active? | Vertical expected? |
|---|---|---|---|
| 0–3 | OCN, ENR, TERM, DPRT | no (TERM = loaded, not active) | — |
| 4 | MAPR | missed approach | — |
| 5, 6 | APR, RNP AR | yes | no |
| 7 / 8 | LNAV / LNAV+V | yes | no (+V is advisory) |
| 9 | L/VNAV | yes | **yes** |
| 10 / 12 | LP / LP+V | yes | no |
| 11 | LPV | yes | **yes** |
| 13 | GLS | yes | **yes** |

- **Runway and airport** come from the loaded **approach flight plan** (SDK
  4.1 `XPLMGetFMSFlightPlanEntryInfo` on `xplm_Fpl_Pilot_Approach`): the
  `RWxx` leg and the airport entry.
- **Active-waypoint classification.** Each flight-plan entry carries a nav
  type:
  - fix or lat/lon → **GPS fix** (exists only in the database);
  - `RWxx` → runway threshold;
  - airport;
  - VOR / NDB / DME / ILS / TACAN → ground navaid.

### 3.3 Distance semantics

| Source | Display | Physics |
|---|---|---|
| NAV + DME | `DME 2.4 NM` | Slant range measured by a ground transponder |
| GPS | `DIST 4.5 NM TO VANNU` | Distance computed from GPS position to a database point |

### 3.4 Needle colour by source

Following the convention pilots already know from common glass-cockpit avionics:
- the course needle and glidepath indicator are **green** for a ground-based
  receiver (NAV1/NAV2: ILS, LOC, VOR);
- they are **magenta** for GPS.

The green is a distinct yellow-green, (0.40, 1.00, 0.18), so it stays
separable from the teal-green runway-strip edges it crosses.

---

## 4. Trusting the signal: navigation validation

`nav/NavValidation` classifies each axis every frame **before** anything is
drawn:
- **Measured** values (needle, number) are shown whenever plausible.
- **Calculated** cues (trend, prediction, chevrons, advice) are shown only
  when the axis is `VALID`.

| Status | Rule |
|---|---|
| `NO SIGNAL` | Display flag off, non-finite, or \|dots\| > **2.75** (rejects the GPS "−3 = no approach" sentinel). Vertical additionally requires a measured lateral signal |
| `FLAGGED` | VOR with neither TO nor FROM (receiver NAV flag) |
| `ACQUIRING` | For **1.5 s** after the signal appears, the source / frequency / approach changes, or the deflection jumps > **1.0 dot** between frames ≤ 0.25 s apart |
| `FULL SCALE` | \|dots\| ≥ **2.4**: the needle is pinned, so its rate carries no information |
| `NOT ESTABLISHED` | Vertical only, when \|lateral\| ≥ **2.0 dots**. Glideslope signals far off the localizer are unreliable (false lobes, coverage) |
| `VALID` | Otherwise. Calculated cues are additionally suppressed below **30 kt** groundspeed |

Every discontinuity (source change or jump) **resets all predictors**, so no
filter ever differentiates across a step and invents a huge rate.

---

## 5. Heading display

The magnetic heading drives a heading-up compass card. To avoid jitter
without lag, the displayed heading is low-passed **along the shortest arc**:

```
Δ = wrap180(raw − shown)
shown ← wrap360(shown + α·Δ),     α = 1 − e^(−dt/τ),   τ = 0.05 s
```

The filter snaps if `dt ≥ 0.5 s` (pause, reload). The readout uses the raw
value rounded aviation-style: 000 shows as **360**.

---

## 6. Rate-based guidance (AxisGuidance)

This is the fallback guidance (and the trend source) for each axis, used
whenever no trustworthy trajectory scale exists.

**Filtered rate:** `rate ← rate + α·(Δpos/dt − rate)`, with
`α = 1 − e^(−dt/1.0 s)`. A gap longer than 1 s restarts the filter.

**Ideal exponential closure.** Remove the deviation with time constant `T`
(20 s lateral, 10 s vertical):

```
desired rate = −pos / T
error (dots) = (desired rate − rate) · T          (low-passed, τ = 0.6 s)
```

| Condition | Advice |
|---|---|
| \|pos\| < 0.1 and \|rate\| < 0.03 dot/s | ON COURSE / ON GLIDEPATH |
| \|error\| < 0.15 ± 0.05 (hysteresis) | HOLD |
| error points toward the path | TURN / CLIMB …, graded at 0.5 and 1.0 dot (±0.05) |
| error points away (closing too fast) | EASE OFF |

**Trend** uses `closing = −sign(pos)·rate`. It is CONVERGING if
`closing > 0.01 dot/s` and DIVERGING if `< −0.01`, and it only leaves a state
once `closing` crosses back over 0.005, so it doesn't chatter.

**Intercept time** = `|pos| / closing` (shown up to 120 s).

**10-second extrapolation** = `pos + rate·10`.

---

## 7. Trajectory prediction (TrajectoryPredictor)

This is NavSight's main differentiator: it predicts from the aircraft's
**motion**, not from needle movement, so it sees convergence, crossings and
overshoots **before the needle moves**.

### 7.1 Track-angle error and turn rate

```
TKE = wrap180(ground track − course)          (+ = tracking right of the course)
ω   = low-pass( d(track)/dt ), τ = 1 s, clamped to ±6°/s
```

Ground track, not heading, is used, so wind correction angle is included
automatically. Below 40 kt, track is noisy, so yaw rate is used instead.

### 7.2 Lateral scale: from dots to nautical miles

A CDI dot is a **distance** on GPS and an **angle** on a localizer or VOR.

| Source | Scale | Confidence |
|---|---|---|
| GPS | `gps_hdef_nm_per_dot` | **MEASURED** |
| LOC / VOR | `nm/dot = d · tan(deg/dot)`, with d = distance to the antenna (database position, else DME) | **PRIOR** → **CALIBRATED** |

The prior is a nominal receiver width: LOC 1.25 °/dot (≈ 2.5° half-width over
2 dots), VOR 4.0 °/dot.

**In-flight self-calibration.** In straight flight (`|ω| ≤ 1°/s`,
`|TKE| ≥ 5°`, needle visibly moving), geometry predicts how fast the aircraft's
angle seen from the antenna changes. With cross-track `x` and along-track
distance `d` to the antenna:

```
θ  = atan(x / d)
θ' = (x'·d − x·d') / (d² + x²)
x' = GS · sin(TKE)                   (cross-track velocity)
d' = −GS · cos(TKE)                  (closing on the antenna; + when receding / FROM)
```

Dividing the predicted angular rate by the **measured** needle rate gives the
receiver's true width:

```
deg/dot (sample) = θ'  /  (measured dots per second)
```

- **Filtering:** samples within ¼ to 4× the prior are filtered in **log
  space** (τ = 4 s), so ratios average symmetrically.
- **Promotion:** the scale becomes **CALIBRATED** after 4 s of good samples.
- **Reset:** any source change or discontinuity drops the calibration.

The `x·d'` term matters: it is the needle drifting purely because the
aircraft is approaching the antenna. Leaving it out caused a ~13% error in an
early version; with it, simulated calibration lands within 10% of the true
width.

### 7.3 The 20-second projection

Assumption: groundspeed and turn rate stay constant ("if you keep doing what
you're doing"). Integrated over 40 steps of 0.5 s with the midpoint rule:

```
x(t)     = x₀ + ∫₀ᵗ GS · sin(TKE + ω·τ) dτ
along(t) = ∫₀ᵗ GS · cos(TKE + ω·τ) dτ
```

Converted back to dots for display:

```
GPS:       dots(t) = x(t) / nm_per_dot
LOC / VOR: dots(t) = atan( x(t) / (d₀ ∓ along(t)) ) / deg_per_dot     (angular; the scale tightens as you close)
```

From the projected path:
- **Crossing time:** the first sign change of `dots(t)`, linearly
  interpolated between steps.
- **Overshoot peak:** the largest excursion beyond the course after the
  crossing.
- **Outcome:**
  - STABILIZING: \|dots\| < 0.2 and \|TKE\| < 3°;
  - CROSSING+OVERSHOOT: peak > 0.5 dot;
  - CONVERGING / DIVERGING: closure beyond `GS·sin 2°`;
  - otherwise PARALLEL.

**Drawing.** The path is drawn on the dial in the course-aligned frame using
the **same lateral scale** as the CDI dots and the runway strip. Where the
cyan path meets the strip centreline is therefore exactly the predicted
crossing point. Along-track uses the same nm-per-pixel (isotropic), so the
path leaves the aircraft's nose in its true track direction.

### 7.4 Roll-out lead: when to start the turn

A standard-rate turn (3°/s, or the current rate if higher) has radius

```
R = GS / ω                       (GS in nm/s, ω in rad/s)
  = (120 kt / 3600) / (3° in rad) = 0.0333 / 0.05236 ≈ 0.637 nm   at 120 kt
```

Turning from track error `TKE` onto the course consumes lateral distance

```
L = R · (1 − cos|TKE|)
  30° intercept at 120 kt:  L = 0.637 · (1 − 0.866) ≈ 0.085 nm
  90° intercept at 120 kt:  L = 0.637 nm
```

| Situation | Cue |
|---|---|
| Converging, \|x\| > L | `TURN IN t s`, where t = (\|x\| − L) / closure, shown when ≤ 15 s |
| Converging, \|x\| ≤ L, not already rolling out (ω toward the course < 2°/s) | **TURN NOW** (amber), predicted overshoot = (L − \|x\|) / nm_per_dot dots |

This is applied only for \|TKE\| ≥ 10°; below that the lead distance is
negligible.

### 7.5 Lateral advice law (track-based)

This law is used only at **CALIBRATED** or **MEASURED** scale; at PRIOR the
projected path is drawn dimmed and no advice comes from it.

**Asymptotic intercept.** The desired track error shrinks as the course
approaches:

```
TKE* = −sign(x) · min(30°, 30°/nm · |x_nm|)
```

| Condition | Advice |
|---|---|
| Inside the roll-out lead (§7.4) | TURN toward the course heading — **NOW** |
| \|x\| < 0.1 dot and \|TKE\| < 3° | ON COURSE |
| Intercepting toward the course at least as steep as TKE*, up to max(TKE*+15°, 45°) | HOLD (timing is the lead's job) |
| Otherwise | TURN LEFT/RIGHT, graded by \|TKE − TKE*\|: 4° hold band, 10° moderate, 20° firm (1.5° hysteresis); EASE if the needed turn is away from the course |

The commands come from **track versus course**, not from the needle. A
parallel offset with a motionless needle still produces "turn toward the
course", and a good intercept produces HOLD even while the needle hasn't
moved yet.

---

## 8. Vertical prediction

**Required descent rate** to stay on a glidepath of angle γ at groundspeed GS
(1 kt = 6076.12 ft / 60 min = 101.27 ft/min):

```
VS_required = −GS · 101.27 · tan(γ)
  3.0° at  90 kt:  −90 · 101.27 · 0.05241 ≈ −478 fpm
  3.0° at 120 kt:  ≈ −637 fpm
```

The path angle comes from the ILS glideslope navaid (§3.1) or the GPS VPA,
otherwise 3.0° is assumed and the value is drawn grey.

**Target with correction.** The target leans toward the path in proportion to
the deviation, capped:

```
VS_target = VS_required − sign(dev) · min(400, 150 · |dev|)   fpm
```

| Condition | Advice |
|---|---|
| \|dev\| < 0.1 and \|VS − VS_required\| < 150 | ON GLIDEPATH |
| \|VS − VS_target\| < 120 ± 30 fpm | HOLD (rate) |
| Otherwise | LESS / MORE DESCENT (CLIMB / DESCEND without VS data), graded at 250 and 500 fpm; EASE if the correction is away from the path |

**Learned diamond gain.** While \|VS error\| ≥ 150 fpm and the diamond is
moving, NavSight learns `c = (dots/s) / (VS error)` (τ = 4 s; trusted after
3 s). The diamond prediction is then `dev + c·VS_error·t`; without it, the
measured rate is used.

**Path overshoot.** Flagged when the predicted crossing is ≤ 20 s away and
the VS error at the crossing (constant-VS assumption) is ≥ 250 fpm in the
crossing direction. In other words, the aircraft will punch through the
glidepath.

---

## 9. Manual runway override

The pilot can declare a runway when the data can't verify it (§3.1).
- **Suggestions:** `n = round(course / 10)` (0 → 36), plus its neighbours
  `n − 1` and `n + 1`, wrapping at 1/36. For example, a 202° course suggests
  19 / 20 / 21.
- **Validation:** `1–36` plus optional `L/C/R`, normalised to two digits
  (`4l` → `04L`).
- **Sanity check:** if `|wrap180(runway × 10 − course)| > 30°`, the display
  shows **CHECK RUNWAY – DOES NOT MATCH COURSE**.
- **Identity binding:** the override stores the `(source, frequency, approach
  label)` it was set on. Any change clears it, so it can't leak into the next
  approach.
- **Display-only:** it is applied to a *copy* of the approach info.
  Validation and the receiver-width prior keep using the real data, so
  setting it never causes a false discontinuity.

---

## 10. Nav source cross-check

`nav/SourceCheck` answers: *is the pilot using the right source and procedure
logic for what they're flying?* It runs every frame on the real
(un-overridden) data and reads more than the HSI source:
- **both NAV receivers**, whatever the HSI selector says: signal, deflection,
  course, TO/FROM and the resolved navaid (a second and third
  `ApproachResolver`, with the same caching);
- the **HSI course pointer** `hsi_obs_deg_mag_pilot`;
- the FMS's **procedure identifiers** `final_approach_name`, `fas_id` and
  `ltp_id` (§10.2);
- **GPS1's approach plan** (SDK flight-plan API, re-read every 5 s and on any
  GPS-mode change): runway, airport, threshold, and the **FAF**, the last
  positioned fix before the `RWxx` leg that is more than 0.5 nm from it;
- **GPS1's CDI mode**, even when the HSI is on NAV.

### 10.1 Established on the final approach course

A flat-earth frame in nautical miles is centred on the threshold `T`
(`x = Δlon·60·cos φ_T`, `y = Δlat·60`). With the FAF at `F` and the aircraft
at `A`:

```
u      = (T − F) / |T − F|            inbound unit vector
a      = A − T
dist   = −a · u                       along-track distance to go (+ = before the threshold)
xtk    = |a × u| = |a_x·u_y − a_y·u_x| cross-track distance
course = atan2(u_x, u_y) − variation  final course, magnetic
```

The magnetic variation is `wrap180(true heading − magnetic heading)` from the
flight model, which has an unambiguous sign. **Established** means all of:
`0 < dist ≤ 12 nm`, `xtk ≤ 1.0 nm`, `|wrap180(track − course)| ≤ 30°`, and
GS ≥ 40 kt. The 30° window accepts a normal intercept; the 1 nm and 12 nm
limits keep vectors across the final and the downwind leg from counting.

### 10.2 Which procedure is loaded

Two flights at KMWA in X-Plane 12.4.4 with the default G1000 (nav log
excerpts) settled what can and can't be known:

| Flight | GPS1 mode | `final_approach_name` | `fas_id` | `ltp_id` |
|---|---|---|---|---|
| ILS 20 loaded, flown on GPS | ENR → **LNAV** | empty | `IMWA` | `DME` |
| RNAV 20 loaded, flown on GPS | TERM → **LPV** | empty | `IMWA` | `DME` |

- The G1000 flies a loaded ILS as a GPS overlay and annunciates **LNAV**.
- `fas_id` / `ltp_id` describe the **ILS tuned on NAV1**, not the GPS
  procedure. They were empty until the localizer was received, and they
  stayed `IMWA` / `DME` while the RNAV was flown in LPV.
- `final_approach_name` is empty on the G1000.

What separates the two is the GPS mode. LPV, L/VNAV, LNAV+V, LP, LP+V, GLS
and RNP AR exist only on RNAV approaches; **LNAV** (and APR) do not.

Classification, first match wins:
1. `final_approach_name`, if an aircraft's FMS fills it: contains
   ILS/LOC/LDA/SDF/IGS → **localizer-based**; RNAV/RNP/GPS/GLS/LPV → **RNAV**;
   VOR/NDB/TACAN → **other**. Otherwise an ARINC 424 identifier (type letter
   and runway): `I` `L` `B` `X` `U` → localizer-based; `R` `H` `P` `J` → RNAV;
   `V` `D` `N` `S` `Q` `T` → other.
2. `fas_id` = `RNV…` or an SBAS/GBAS channel (`W##x`, `G##x`) → RNAV. A
   localizer ident there is ignored: it is the tuned radio, so using it would
   be circular.
3. Otherwise **unknown**.

"Flying RNAV" = an RNAV procedure, or (procedure not localizer-based) GPS1 in
an RNAV-only mode (6, 8–13). When the FMS names a localizer procedure flown on
GPS, the APPROACH label is renamed from `RNAV RWY 20` to `ILS RWY 20` (display
only).

**Known ambiguity:** with no procedure name, an RNAV approach with LNAV-only
minima flown with the same-runway ILS alive looks exactly like an ILS flown on
GPS, and is flagged.

### 10.3 Rules

Evaluated in order; the first match is reported. "Serves the approach" means
the localizer's runway equals the loaded runway (normalised, so `2` = `02`)
and the airports don't differ. With no plan, it means a localizer within
20 nm whose course is within 10° of the GPS course. "On the final" means
established (§10.1) or, with no plan geometry, tracking within 30° of the
localizer course with the needle off the stop (< 2.4 dots).

| # | Code | Condition | Display |
|---|---|---|---|
| 1 | `GPS_ON_LOC` | HSI on GPS; not flying RNAV (§10.2); not in the missed approach; NAV1 or NAV2 has a **live** ILS/LOC that serves the approach; **on the final** | `ILS APPROACH ON GPS – SELECT NAV1` |
| 1a | `APPR_NOT_ACTIVE` | As rule 1, but procedure unknown, GPS1 in OCN/ENR/TERM/DPRT and established: either an RNAV to activate or an ILS to fly on NAV | `GPS APPR NOT ACTIVE – ACTIVATE OR SELECT NAV1` |
| 1b | `GPS_ON_LOC` | HSI on GPS; FMS names a localizer-based procedure; established; no localizer received | `ILS APPROACH ON GPS – TUNE AND SELECT NAV` |
| 2 | `NAV_ON_RNAV` | HSI on NAV; procedure not localizer-based; GPS1 in an RNAV approach mode (RNP AR, LNAV, LNAV+V, L/VNAV, LP, LPV, LP+V, GLS); the tuned NAV isn't a localizer serving that runway (one whose runway is unknown gets the benefit of the doubt) | `RNAV APPROACH ACTIVE – SELECT GPS` |
| 3 | `APPR_NOT_ACTIVE` | HSI on GPS; procedure not localizer-based; GPS1 in OCN/ENR/TERM/DPRT (0–3); plan has a final segment; **established** (§10.1) | `GPS APPROACH NOT ACTIVE – ACTIVATE APPR` |
| 4 | `LOC_NE_APPR` | HSI on NAV, live ILS/LOC with a known runway; plan loaded for a different runway or airport | `TUNED ILS RWY 23 – LOADED APPR RWY 20` |
| 5 | `CRS_NOT_SET` | HSI on NAV, live ILS/LOC; `|wrap180(OBS − localizer course)| > 10°` | `SET COURSE 202° – HSI CRS 270°` |
| — | `OK` | guidance shown or an approach loaded, and nothing above | green `SRC OK` |
| — | `NA` | otherwise | grey `SRC --` |

The legitimate look-alikes are deliberately silent, and each is a unit test:
LPV active with the same-runway ILS tuned for monitoring (rule 1 needs a
procedure that isn't RNAV); an ILS flown on NAV with the same-runway RNAV
approach active as backup (rule 2); an activated approach on final (rule 3);
the localizer alive while still being vectored to the final (rule 1 needs "on
the final"); a VOR, whose OBS is the pilot's choice (rule 5). The in-sim
flights in §10.2 are regression tests with their logged values.

APR (mode 5) is treated as "activated" for rule 3 but not as an RNAV mode for
rules 1–2, because some GPS units report APR for any active approach,
including ILS.

### 10.4 Debounce

A finding must be present continuously for **2.0 s** before it is shown, and
a shown caution clears **1.5 s** after the condition goes away. This stops
switching sources (where the needle and receivers pass through ACQUIRING)
from flashing cautions. While a caution is shown, its text follows live
values (e.g. the HSI course as the knob turns). A different finding replaces
it once it has itself persisted 2.0 s. A sim-time reset starts over.

### 10.5 Outputs

`InstrumentState::sourceCheck` drives the footer chip and the title-box
banner. The banner outranks the runway-override mismatch caution. Every
change is written to the nav log with context (HSI source, GPS mode, both
radios, plan, OBS, along/cross-track to the final). Two datarefs are
published: `navsight/check/status` (0 n/a, 1 OK, 2 caution) and
`navsight/check/code` (0 none, 1–5 in the table's order).

---

## 11. Rendering geometry

- **Heading-up card.** A bearing `B` is drawn at screen angle `θ = B −
  heading` (clockwise from up): `(cx + r·sin θ, cy + r·cos θ)`. Numerals are
  rotated by `θ`, so their tops point outward.
- **Course frame.** The needle, strip and projected track live in a frame
  rotated by `course − heading`, with right = `(cos θ, −sin θ)` and along =
  `(sin θ, cos θ)`.
- **Deviation scale.** One CDI dot = 0.075 dial radii (glideslope dot 0.07),
  clamped at ±2.5 dots. The course line sits at `x = −pos · 0.075`
  (displaced toward the course).
- **Runway strip = band ∩ disc.** A constant-width band (half-width 0.10)
  clipped to the dial circle (R = 0.398). The intersection of a band and a
  disc is convex. Its outline is: up the right edge to
  `y = √(R² − x_r²)`, the arc across the top, down the left edge, and the arc
  across the bottom. It is filled as one convex polygon, with no stencil or
  clipping hardware. This is 2-D alignment, deliberately not synthetic vision:
  no perspective and no distance scaling.
- **Vector stroke font.** Glyphs are polylines on a 4 × 6 grid, drawn as
  thick lines with round joins. They scale and rotate freely, which
  X-Plane's bitmap fonts cannot. Text is condensed to 0.78 width and fitted
  by scaling height to the available width.
- **Hit-testing** uses the same layout functions as drawing, so clicks always
  land on what is shown.

---

## 12. Display smoothing, fades and themes

**Motion** (`render/DisplaySmoother`, display only; the nav math stays
exact). The needle, glideslope diamond, strip offset, course and bearing
pointers, and every projected-track point pass through **two cascaded
first-order filters**. That is a critically damped second-order response:
smooth acceleration and **no overshoot**.

```
a ← a + α(target − a);   b ← b + α(a − b);   α = 1 − e^(−dt/0.075 s)
```

Overall settling is about 0.15 s on wall-clock time, so it keeps easing while
the sim is paused.
- **Snaps:** source change, gap > 0.5 s, or a jump > 1.5 dots.
- **Angles** are unwrapped along the shortest arc before filtering.

**Fades.**
- Chevrons, the alignment box and the projected track fade over 0.2 s.
- A cue fades **out** only when the advice relaxes to HOLD or ON COURSE.
- If data becomes invalid, cues vanish **immediately**, so stale guidance
  never lingers.

**Themes.** One colour table generates Day / Dim / Night. Each colour has a
luminance class:

| Class | Dim | Night |
|---|---|---|
| White symbols | × 0.72 | × 0.45 |
| Coloured symbols (magenta, cyan, green, amber) | × 0.80 | × 0.58 |
| Fills / backgrounds | × 0.80 | × 0.60 |

Whites dim most because they dominate night glare; the coloured symbology
stays distinguishable.

---

## 13. Anti-aliasing and the GPU path

**Feathered edges.** Every line, circle, ring and convex shape gets a
1-pixel fringe whose colour fades from opaque to transparent:

- **Lines:** quads on both sides along the normal.
- **Circles and rings:** radial strips.
- **Convex polygons:** each vertex is extruded along the *average* of its
  two adjacent outward edge normals (orientation found from the signed area),
  so the fringe is continuous around corners.

To keep apparent stroke weight unchanged, the opaque core is pulled in by
half the fringe:

```
core half-width = max(0.35·w, w/2 − f/2)
circle radius   = max(0.7·r, r − f/2)
```

**Batching.** All shapes become coloured triangles in one CPU buffer; a frame
is a single draw. The OpenGL backend uses GL 1.1 client vertex arrays (one
`glDrawArrays`). Because X-Plane may leave a buffer object or vertex-array
object bound, NavSight:
1. queries `GL_ARRAY_BUFFER_BINDING` and `GL_VERTEX_ARRAY_BINDING`;
2. unbinds them via `wglGetProcAddress` entry points;
3. draws;
4. restores them.

It falls back to immediate mode if that isn't possible (or via the
`fast_rendering 0` setting). Per-frame build and submit times are logged in
verbose mode.

---

## 14. How it was verified

- **Unit and simulation tests:** **1363 checks** (`tests/TestMain.cpp`), run
  without X-Plane.
- **Kinematic flight simulator.** A point-mass aircraft flies a given
  groundspeed, track and turn rate against a course. NavSight's inputs (hdef,
  track, GS, distance) are synthesised from its true position, and the
  predictions are checked against the analytic answer:
  - 30° intercept: 10 s prediction within **±0.02 dot**, roll-out lead within
    0.002 nm, time-to-turn within 1 s;
  - close-in 90° intercept: **TURN NOW** and the predicted overshoot appear
    before the needle centres, and the crossing time is within 0.5 s;
  - a turning aircraft (1.5°/s): the turn rate is estimated within 0.15°/s,
    and the 20 s projection matches within ±0.05 dot;
  - localizer self-calibration converges within **10%** of the true width,
    and the calibrated angular projection matches within ±0.1 dot 20 s ahead;
  - track-based advice differs correctly from the rate law: HOLD on a good
    intercept, TURN on a parallel offset with a motionless needle.
- **Source check** (§10), on synthetic approaches with a known geometry
  (RWY 20, FAF 5 nm out):
  - along-track distance and cross-track within 0.02 nm, final course within
    0.2°, including magnetic variation;
  - the established window's edges: beyond 12 nm, past the threshold, 1.5 nm
    across, outbound, 35° off track, below 40 kt;
  - every rule fires on its scenario and stays silent on its legitimate
    look-alikes, in priority order;
  - procedure classification, and both in-sim flights of §10.2 replayed with
    their logged values (ILS in LNAV → caution; RNAV in LPV → none);
  - debounce timing (nothing before 2.0 s, a caution lingers 1.4 s and clears
    by 1.6 s), replacement of one caution by another, and a sim-time reset;
  - end to end through `InstrumentModel` (unaffected by a runway override),
    and the chip and banner rendered inside the viewport in every layout.
- **Simulator truth.** Pixel crops of X-Plane's own G1000 PFD established the
  deflection signs (§2.2).
- **Headless renders** of every mode, layout, brightness and failure state
  are checked to be finite and inside the viewport. Offline PNG renders were
  reviewed against the design reference.

---

## 15. Constants reference

| Area | Constant | Value |
|---|---|---|
| Validation | settle / full scale / implausible | 1.5 s / 2.4 dots / 2.75 dots |
| | jump / window / established / min GS | 1.0 dot / 0.25 s / 2.0 dots / 30 kt |
| Heading | smoothing τ | 0.05 s |
| Rate guidance | rate τ / command τ / closure T | 1.0 s / 0.6 s / 20 s lat, 10 s vert |
| | hold / moderate / firm (± hysteresis) | 0.15 / 0.5 / 1.0 dot (± 0.05) |
| Trajectory | horizon / step | 20 s / 0.5 s |
| | turn-rate τ / clamp / min GS | 1 s / ±6°/s / 40 kt |
| | standard rate / lead min TKE / turn-in window | 3°/s / 10° / 15 s |
| | intercept law max / gain | 30° / 30°/nm |
| | hold / moderate / firm (± hysteresis) | 4° / 10° / 20° (± 1.5°) |
| Calibration | min TKE / max ω / min rate / τ / required | 5° / 1°/s / 0.01 dot/s / 4 s / 4 s |
| | priors | LOC 1.25 °/dot, VOR 4.0 °/dot |
| Vertical | gain / max correction | 150 fpm/dot / 400 fpm |
| | hold / moderate / firm (± hysteresis) / overshoot | 120 / 250 / 500 (± 30) / 250 fpm |
| Resolver | max station range / airport radius / GS pairing | 120 NM / 15 NM / 5 NM |
| Override | course mismatch | > 30° |
| Source check | established: distance / cross-track / track / GS | ≤ 12 nm / ≤ 1.0 nm / ≤ 30° / ≥ 40 kt |
| | course match / localizer range (no plan) / FAF min | 10° / 20 nm / 0.5 nm |
| | show / clear / plan refresh | 2.0 s / 1.5 s / 5 s |
| Display | motion τ (total) / fade / snap | 0.15 s / 0.2 s / 1.5 dots |
| Geometry | dot / GS dot / strip half-width / clip radius | 0.075 / 0.07 / 0.10 / 0.398 dial radii |
| Rendering | feather | 1 boxel (≈ 1 px) |

---

## 16. Assumptions and limits

- **Constant inputs over the horizon.** The projection assumes groundspeed
  and turn rate stay constant for 20 s; wind shifts, speed changes and the
  pilot's next input are not modelled.
- **Localizer and VOR estimates start from a nominal width.** Until
  calibrated (a few seconds of straight flight at ≥ 5° to the course), the
  projected track is drawn dimmed and gives no advice.
- **Glidepath angle** is assumed to be 3.0° when the data can't confirm it,
  and is shown as assumed.
- **Source check depends on the GPS's flight plan.** The not-activated and
  runway-mismatch checks need the aircraft's GPS to load approaches into
  X-Plane's flight plan; without one, only the source and course checks run.
  It reads GPS1 only, and treats APR (mode 5) as activated but not as an RNAV
  mode, since some units report APR for any active approach. Its
  "established" test is geometric (FAF → threshold), so it can't see step-down
  fixes or curved finals (RF legs).
- **Standard X-Plane radios only.** NavSight reads the HSI datarefs;
  aircraft with fully custom avionics that don't drive them won't show
  guidance.
- **Not certified.** Everything here is advisory situational awareness for
  simulation, never a certified flight director.
