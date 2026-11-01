# Building NavSight from source

Most users don't need this: download the release zip and drop the `NavSight`
folder into X-Plane (see the [README](../README.md#install)). Build from source
if you want to change the code, run the tests, or try the latest commit.

## Requirements

- Windows 10/11 x64.
- Visual Studio 2022 or newer with the **Desktop development with C++**
  workload (verified with VS 2026 18.11 / MSVC 14.51). The free Community
  edition works.
- X-Plane 12.4 or newer, to run it.

You don't need to install anything else. The build script finds Visual Studio
by itself and downloads the X-Plane SDK 4.3.0 into `third_party/XPSDK` the
first time it runs. CMake is optional (see [CMake](#cmake)).

## Build and install in one step

Close X-Plane, then in PowerShell:

```powershell
git clone https://github.com/Rohank050505/NavSight.git
cd NavSight
.\build.ps1 -Install
```

This builds the plugin and copies it to
`<X-Plane 12>\Resources\plugins\NavSight\win_x64\`, the same layout as the
release zip. X-Plane is found from `%LOCALAPPDATA%\x-plane_install_12.txt`; if
that fails, pass the folder: `.\build.ps1 -Install -XPlaneDir "D:\X-Plane 12"`.

If PowerShell refuses to run the script, allow local scripts once with
`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, or run it as
`powershell -ExecutionPolicy Bypass -File .\build.ps1 -Install`.

`-Install` also removes a pre-rename `Resources\plugins\SmartCDI` install, but
only if it is unmistakably an old NavSight build.

## Build options

```powershell
.\build.ps1                          # Release -> build\Release\NavSight\win_x64\NavSight.xpl
.\build.ps1 -Test                    # + 1363 unit / simulation / headless-render checks
.\build.ps1 -Configuration Debug     # SDK error callback, verbose logs
.\build.ps1 -Test -Package           # release zip -> dist\NavSight-<version>-win_x64.zip (no PDB)
.\build.ps1 -FetchSdk                # re-download the X-Plane SDK
.\build.ps1 -Clean                   # delete build\ first
```

The script compiles `src/` at `/W4` with the static CRT (`/MT`), links, and
checks the five `XPlugin*` exports. The `.xpl` depends only on `XPLM_64.dll`,
`OPENGL32.dll` and `KERNEL32.dll`, so users need no runtime installers.

Everything the build writes stays in `build\` and `dist\`, both ignored by
git:

```
build\Release\obj\               intermediate objects (never distributed)
build\Release\NavSight\win_x64\  NavSight.xpl + NavSight.pdb (debug symbols)
build\package\NavSight\          staging folder for the release zip
dist\NavSight-<version>-win_x64.zip
```

## Making a release

1. Set the version in `src/core/Version.h` and add a `CHANGELOG.md` entry.
2. `.\build.ps1 -Test -Package`.
3. Attach `dist\NavSight-<version>-win_x64.zip` to a new GitHub release.

The zip contains exactly what goes into `Resources\plugins` and nothing else:

```
NavSight/
├── win_x64/NavSight.xpl   the plugin
├── README.txt             install steps + disclaimer
├── USER_GUIDE.md
└── CHANGELOG.md
```

## Tests

`tests/TestMain.cpp` runs without X-Plane and covers:
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
2. Start X-Plane.
3. *Debug → Attach to Process → X-Plane.exe*. The PDB is installed next to
   the `.xpl`.

## CMake

`CMakeLists.txt` is provided for IDEs and CI. It is not the verified path;
`build.ps1` is. It expects the SDK in `third_party/XPSDK` (run
`.\build.ps1 -FetchSdk` once).
