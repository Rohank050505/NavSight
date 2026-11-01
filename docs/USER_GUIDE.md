# NavSight — User Guide (v1.0.0)

NavSight is a predictive navigation instrument for **X-Plane 12** (Windows).
It sits in its own window and tells you where you are relative to the
procedure, where your current trajectory is taking you, and whether you are
converging, diverging or about to overshoot. It also checks that your nav
source and approach setup fit the procedure you're flying (§3, *Source check*).

> NavSight is situational-awareness software **for flight simulation only**.
> It is not certified avionics or a flight director, and it must not be used
> for real-world navigation.

---

## 1. Installing

Requires X-Plane 12.4 or newer on Windows x64.

**From the release zip.** Download `NavSight-<version>-win_x64.zip` from the
[Releases page](https://github.com/Rohank050505/NavSight/releases/latest),
unzip it, and drag the `NavSight` folder into `<X-Plane 12>\Resources\plugins\`
with X-Plane closed, so that you have:

```
X-Plane 12\Resources\plugins\NavSight\win_x64\NavSight.xpl
```

To update, replace the folder with the new one.

**From source.** Run `.\build.ps1 -Install` with X-Plane closed (see
[BUILDING.md](BUILDING.md)).

**Upgrading from SmartCDI.** NavSight was called SmartCDI before v0.3.0.
- Delete `Resources\plugins\SmartCDI` (`-Install` does this for you).
- Your settings are migrated automatically.
- Key bindings to `smartcdi/…` commands need re-binding to `navsight/…`.

**Uninstall.** Delete `Resources\plugins\NavSight`, and optionally
`Output\preferences\NavSight.prf`.

Start X-Plane and open **Plugins → NavSight → Show Instrument**.

---

## 2. The window

- **Move it:** drag the title bar.
- **Resize it:** drag an edge or corner. The instrument is **square**; spare
  space is letterboxed.
  - **Standard layout** (side ≥ 420): header boxes, dial, and the LATERAL /
    VERTICAL panels.
  - **Compact layout** (smaller): the dial with a one-line header.
- **Close it:** use the close box, or *Show Instrument* again.
- **Pop out:** *Pop Out Window* moves it to its own OS window, e.g. on a
  second monitor.
- **Lost off-screen?** Use *Reset Window Position*.
- **Remembered between sessions:** position, size, visibility, panels and
  brightness.

---

## 3. Reading the instrument

```
┌ NAVSIGHT ─────────────────────────┐   ┌ APPROACH: ILS RWY 20 KMWA ──┐
│ HSI OVERLAY - RUNWAY ALIGNMENT MODE│   │ CRS: 202°  BRG: 204°  TRK: 199°│
┌LATERAL ‹┐          [ 200° ]           ┌VERTICAL ›┐
│0.4 DOT R│        compass rose          │0.3 DOT LOW│
│ ←       │   runway strip · needle      │ ↑         │
│TURN LEFT│   projected track · ✈ · GS   │LESS DESCENT│
│SLIGHTLY │                              │SLIGHTLY   │
│TREND ↘  │                              │TREND ↘    │
│CONVERGING                              │CONVERGING │
│TURN IN 8S                              │REQ VS -480│
│● ○ ○ ○  │                              │● ○ ○ ○ ○  │
NAV1                [ SRC OK ]            DME 6.4 NM
```

### Header
- **Title box:** the current **mode**:
  - `HEADING MODE`;
  - `COURSE REFERENCE MODE`;
  - `RUNWAY ALIGNMENT MODE`;
  - or an amber **source caution**, e.g. `GPS APPROACH NOT ACTIVE – ACTIVATE APPR`
    (see *Source check* below). It takes priority over everything else;
  - or amber `CHECK RUNWAY – DOES NOT MATCH COURSE` (see §5).
- **APPROACH / SOURCE box:**
  - what you are flying, e.g. `ILS RWY 20 KMWA` or `RNAV RWY 20 LPV KMWA`;
  - `MAN` in amber when the runway is pilot-declared;
  - `CRS` is the selected course, `BRG` the bearing to the station or
    waypoint, and `TRK` your actual ground track.
- **Click the APPROACH box** to open the runway override picker (§5).

### Modes (automatic)

| Mode | When |
|---|---|
| **Heading** | The HSI source has no usable lateral signal |
| **Course reference** | A VOR radial, a GPS leg, or an approach whose runway can't be verified |
| **Runway alignment** | ILS, LOC/LDA/SDF, or an **active** GPS approach (LPV, LNAV, L/VNAV, LP, …), with a verified or pilot-declared runway |

NavSight follows the aircraft's **HSI source selector** (NAV1 / NAV2 / GPS),
so it shows whatever your HSI or PFD is coupled to.

### On the dial

| Element | Meaning |
|---|---|
| **Compass rose + top box** | Heading-up card; the box shows magnetic heading |
| **Green runway strip** | The final course drawn as a flat strip (not synthetic vision), shifted by your measured deviation. Aircraft on the dashed centreline = on course; strip upright = tracking parallel. The runway number is shown only when the nav data names it, or tagged **MAN** if you declared it |
| **Grey course band** | Course reference when no runway is known, labelled with the course |
| **Needle** | Course pointer and deviation bar (measured), as on an HSI. **Green** for a ground-based source (NAV1/NAV2: ILS, LOC, VOR), **magenta** for GPS, as on common glass-cockpit avionics |
| **Diamond (right)** | Glideslope / glidepath, same sense as the G1000: above centre means you are low. Green (ILS) or magenta (GPS), like the needle |
| **Cyan dotted path** | Where you'll be over the next **20 s** if you keep the same track and turn rate, with dots at 10 s and 20 s. Where it meets the strip centreline is where you'll cross the course. Amber beyond the course = overshoot. It is drawn dim while the estimate is still calibrating |
| **Cyan chevrons** | Which way to correct (1–3 = slight / moderate / firm). **Amber** = ease off / turn now: an overshoot is predicted |
| **Green dashed box** | Aligned: within ½ dot on both axes |
| **Cyan triangle on the rim** | Bearing to the station or waypoint |
| **Red flags** | `HDG` no heading; `NAV` receiver flag; `LOC`/`GPS` approach tuned but no lateral signal; `GS` glidepath expected but missing |

### Side panels

Each panel shows, top to bottom:
- **Measured deviation** in magenta, e.g. `0.4 DOT R`, `0.3 DOT LOW`
  (`2.5+` = full scale), with a meter bar on the left.
- **Calculated advice** in cyan:
  - `TURN LEFT / SLIGHTLY`, `HOLD / HEADING`, `ON COURSE`;
  - vertically, `LESS DESCENT` / `MORE DESCENT` when on a descending path;
  - amber `EASE OFF` or `NOW` when an overshoot is predicted.
- **Trend:** `CONVERGING`, `DIVERGING`, `STEADY` or `CENTERED`.
- **Prediction line:**
  - `TURN IN 8S`: start the turn onto the course in about 8 s;
  - `OVERSHOOT 0.4 DOT` (amber): you are already inside the turn distance;
  - `CROSS 14S`: when you'll cross the course;
  - `REQ VS -480`: the descent rate the glidepath needs at your groundspeed;
    grey if the path angle is assumed (3°);
  - `PATH OVERSHOOT` (amber).
- **Pips:** one per half dot of deviation.

When the signal can't be trusted, the measured value stays visible but the
advice is replaced by the reason:

| Panel shows | Why |
|---|---|
| `ACQUIRING` | About 1.5 s after a signal appears, a source switch, retune or sudden jump |
| `FULL SCALE / OFF COURSE` | Needle pinned; its motion means nothing |
| `NOT ESTABLISHED / LATERALLY` | 2+ dots off the localizer; glideslope there is unreliable |
| `NO COURSE` / `NO GLIDEPATH` | No signal (or an impossible value) |
| `NAV FLAG` (red) | VOR receiver failure flag |
| `STANDBY / LOW SPEED` | Groundspeed below 30 kt |

**Collapse the panels:** click either panel (or *Show Guidance Panels* in the
menu). They fold into slim edge tabs that still show the deviation and turn
arrow. A tab gets an **amber border** if that axis has a caution. Click a tab
to expand.

### Footer
- **Left:** the nav source (`PAUSED` appears just above it when the sim is paused).
- **Centre:** the **source check chip** (below).
- **Right:** distance.
  - `DME 2.4 NM`: measured from a ground station (NAV source).
  - `DIST 4.5 NM TO VANNU`: computed by the GPS to its active waypoint.

### Source check

NavSight cross-checks your **HSI source**, both **NAV radios**, the **HSI course**
and the **approach loaded in GPS1** to catch common setup errors. The chip at
the bottom centre shows the result:

| Chip | Meaning |
|---|---|
| green `SRC OK` | Guidance is shown and none of the checks below found a problem |
| grey `SRC --` | Nothing to check (heading mode, no approach loaded) |
| amber, e.g. `CDI GPS>NAV1` | A problem; the full message replaces the title-box mode text |

| Caution (title box) | Chip | When |
|---|---|---|
| `ILS/LOC APPROACH ON GPS – SELECT NAV1` | `CDI GPS>NAV1` | You're on the final with the HSI on GPS, NAV1 (or NAV2) is receiving a localizer for the loaded runway, and the GPS isn't flying an RNAV approach: it isn't in LPV, L/VNAV, LNAV+V, LP, LP+V, GLS or RNP AR (those exist only on RNAV approaches). With no approach loaded: a nearby localizer (≤ 20 nm) on the same course as the GPS (± 10°), and you're tracking it inside the beam |
| `GPS APPR NOT ACTIVE – ACTIVATE OR SELECT NAV1` | `APPR NOT ACTV` | As above, but the GPS approach isn't activated yet (ENR/TERM). NavSight can't tell whether you meant to fly the RNAV (activate it) or the ILS (select NAV1) |
| `ILS/LOC APPROACH ON GPS – TUNE AND SELECT NAV` | `CDI GPS>NAV` | The aircraft's FMS reports an ILS/LOC procedure, you're established on its final on GPS, and no localizer is being received. The default G1000 doesn't report the procedure, so this appears only in aircraft whose FMS does |
| `RNAV APPROACH ACTIVE – SELECT GPS` | `CDI NAV>GPS` | GPS1 is in an RNAV approach mode (RNP AR, LNAV, LNAV+V, L/VNAV, LP, LPV, LP+V, GLS), the loaded procedure isn't an ILS/LOC, and the HSI is on NAV without a localizer for that runway |
| `GPS APPROACH NOT ACTIVE – ACTIVATE APPR` | `APPR NOT ACTV` | A (non-ILS) approach is loaded, the HSI is on GPS, GPS1 is still in ENR/TERM (not activated), and you are **established on its final**: within 12 nm of the threshold, within 1 nm of the final course, tracking within 30° of it, above 40 kt |
| `TUNED ILS RWY 23 – LOADED APPR RWY 20` | `LOC/APPR RWY` | The HSI is on NAV, receiving a localizer for a different runway (or airport) than the approach loaded in GPS1 |
| `SET COURSE 202° – HSI CRS 270°` | `SET CRS 202` | The HSI is on a localizer but its course pointer is more than 10° from the localizer course |

Notes:
- Only one caution is shown at a time, in the order above.
- A caution must persist **2 s** before it appears and clears **1.5 s** after
  the problem is fixed, so switching sources doesn't flash warnings.
- Flying an ILS with the same-runway RNAV approach active as backup, or flying
  an LPV with the ILS tuned for monitoring, are **not** flagged.
- A VOR's course is your choice and is never checked.
- **ILS loaded in the GPS.** X-Plane's G1000 flies a loaded ILS as a GPS
  overlay and shows **LNAV**. An RNAV approach shows **LPV** (or L/VNAV, LP, …)
  instead, so NavSight treats LNAV with a live same-runway localizer on the
  final as an ILS flown on GPS.
  - **Limitation:** an RNAV approach with *LNAV-only* minima, flown with the
    same-runway ILS alive, looks identical, and is flagged. The G1000 doesn't
    tell X-Plane which approach is loaded. If your aircraft's FMS does, NavSight
    uses that instead: no false caution, and the APPROACH box reads
    `ILS RWY 20` for an ILS flown on GPS.
- The check uses the real nav data; a runway override (§5) doesn't hide it.
- The approach-plan checks need an aircraft whose GPS loads approaches into
  X-Plane's flight plan (the default G1000/GNS do). Without one, only the
  source and course checks run.
- Every change is written to `NavSight_nav.log` with details (HSI source, GPS
  mode, both radios, loaded approach, distance and cross-track to the final).

---

## 4. Brightness

**Plugins → NavSight → Brightness → Day / Dim / Night**, or the commands below.
- **Dim** and **Night** darken the whole instrument, whites most of all.
- Magenta, cyan, green and amber stay distinct at every level.

---

## 5. Runway override (manual runway)

Use it when NavSight shows a **course band** but you know which runway the
course leads to. Typical cases:
- a localizer whose nav data doesn't name the runway;
- a VOR or visual approach with the OBS set on the runway;
- a GPS approach plan that X-Plane doesn't expose.

**Open the picker:** click the **APPROACH box**, or *Runway Override…* in the
menu, or the `navsight/runway/picker` command.

```
┌──────── RUNWAY OVERRIDE  CRS 202° ────────┐
│     [ 19 ]      [ 20 ]      [ 21 ]        │   ← suggested from the course
│   [ - ]    [ L ]    [ C ]    [ R ]        │   ← side
│   ENTRY 20     (TYPE 0-9 L C R / ENTER / ESC)
│   ACTIVE AUTO (NAV DATA)                  │
│   [ AUTO ]                 [ APPLY ]      │
└───────────────────────────────────────────┘
```

- **Choose:** click a suggested number and optionally a side, or **type** it
  (e.g. `1`, `6`, `L`). Backspace corrects.
- **Apply:** **APPLY** or **Enter**.
- **Back to nav data:** **AUTO**.
- **Close without changes:** **Esc**, or click outside the picker.

Once applied:
- The strip shows the runway with an amber **MAN** tag, and the header shows
  `… RWY 20 MAN`.
- It is **never** presented as verified data.
- If the runway is more than 30° from the selected course (e.g. `02` on a
  202° course), the title box shows amber **CHECK RUNWAY – DOES NOT MATCH
  COURSE**.
- It **clears automatically** when the nav source changes (retune, NAV1 /
  NAV2 / GPS switch, a different approach, or aircraft reload), so it can't
  leak into the next approach. It isn't saved between sessions.

Without the picker: `navsight/runway/next` and `navsight/runway/previous` step
through the suggested numbers, and `navsight/runway/clear` returns to nav data.

---

## 6. Menu and commands

**Plugins → NavSight:**
- Show Instrument;
- Pop Out Window;
- Reset Window Position;
- Show Guidance Panels;
- Brightness ▸ Day / Dim / Night;
- Runway Override…;
- Clear Runway Override;
- Verbose Logging.

Bind commands under *Settings → Keyboard / Joystick* (search "NavSight"):

| Command | Action |
|---|---|
| `navsight/window/toggle` | Show / hide the instrument |
| `navsight/window/pop_out` | Pop out to an OS window |
| `navsight/window/reset_position` | Reset window placement |
| `navsight/view/toggle_panels` | Show / collapse the side panels |
| `navsight/view/brightness` | Cycle Day → Dim → Night |
| `navsight/view/brightness_day` / `_dim` / `_night` | Set brightness |
| `navsight/runway/picker` | Open / close the runway override picker |
| `navsight/runway/next` / `previous` | Step the runway override through the suggestions |
| `navsight/runway/clear` | Clear the runway override |
| `navsight/debug/toggle_logging` | Verbose nav logging |

---

## 7. Settings file

`<X-Plane 12>\Output\preferences\NavSight.prf` is written when X-Plane exits.
Edit it only while X-Plane is closed. Unknown keys are ignored, and bad values
fall back to defaults.

| Key | Default | Meaning |
|---|---|---|
| `window_visible`, `window_left/top/right/bottom` | — | Window state |
| `show_panels` | `1` | `0` = panels collapsed to edge tabs |
| `brightness` | `day` | `day` / `dim` / `night` |
| `heading_smoothing_sec` | `0.05` | Rose smoothing (0–2 s) |
| `lateral_closure_sec` / `vertical_closure_sec` | `20` / `10` | Fallback guidance aggressiveness (3–120 s; lower = firmer) |
| `invert_lateral` / `invert_vertical` | `0` | Only for an aircraft that reports deflections mirrored |
| `fast_rendering` | `1` | `0` = compatibility drawing mode if a graphics driver misbehaves |
| `debug_logging` | `0` | Verbose nav logging |

---

## 8. Troubleshooting

- **Logs.**
  - `X-Plane 12\Log.txt`: NavSight lifecycle and errors (search
    "NavSight").
  - `Resources\plugins\NavSight\NavSight_nav.log`: navigation decisions,
    such as which approach was identified, signal status changes, calibration,
    `TURN NOW` events, runway overrides and `source check:` results. The
    previous session is kept as `NavSight_nav.prev.log`.
- **A source caution you think is wrong.** Check the `source check:` line in
  the nav log. It lists the HSI source, the GPS mode, what each NAV radio is
  receiving, the loaded approach and, on a GPS final, how far along and
  across the final you are. Please report it with that line.
- **The chip never shows `APPR NOT ACTV` or `LOC/APPR RWY`.** Those checks
  need the approach loaded in the GPS's flight plan. Look for `plan=none` in
  the `source check:` line. Some aircraft GPS units don't share their plan with
  X-Plane.
- **The chip stays grey (`SRC --`).** Nothing to check: no guidance is
  shown and no approach is loaded.
- **Stuck in course-reference mode on an approach.** The runway couldn't be
  verified. Check `nav source resolved:` in the nav log, or set the runway
  with the override (§5).
- **No guidance in a custom aircraft.** NavSight reads X-Plane's standard HSI
  datarefs. Aircraft with fully custom avionics may not drive them.
- **Needle or diamond on the wrong side in one aircraft.** Set
  `invert_lateral` / `invert_vertical`, and please report it.
- **Visual glitches or blank window after a driver update.** Set
  `fast_rendering 0`.
- **Dim projected track that never brightens on an ILS/VOR.** The scale
  calibrates during a few seconds of straight flight at an intercept angle of
  5° or more. Until then the estimate stays dim and gives no turn advice.
