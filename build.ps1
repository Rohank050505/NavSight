<#
.SYNOPSIS
    Build, test and install the NavSight X-Plane plugin with MSVC.

.DESCRIPTION
    Self-contained build: needs only Visual Studio (2022 or newer) with the
    "Desktop development with C++" workload. The X-Plane SDK is downloaded
    into third_party\XPSDK on first use (-FetchSdk re-downloads it). CMake is
    NOT required; CMakeLists.txt is provided as an alternative for IDEs / CI.
    See docs\BUILDING.md.

.EXAMPLE
    .\build.ps1 -Install            # build and install into X-Plane (X-Plane closed)
    .\build.ps1                     # Release build -> build\Release\NavSight\win_x64\NavSight.xpl
    .\build.ps1 -Test               # also build + run host-side unit tests
    .\build.ps1 -Configuration Debug -Install
    .\build.ps1 -FetchSdk           # download X-Plane SDK 4.3.0 into third_party\XPSDK
    .\build.ps1 -Test -Package      # release zip -> dist\NavSight-<version>-win_x64.zip (no PDB)
#>
[CmdletBinding()]
param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',
    [switch]$Test,
    [switch]$Install,
    [string]$XPlaneDir,
    [switch]$Clean,
    [switch]$FetchSdk,
    [switch]$Package
)

$ErrorActionPreference = 'Stop'
$Root      = $PSScriptRoot
$SdkDir    = Join-Path $Root 'third_party\XPSDK'
$SdkUrl    = 'https://developer.x-plane.com/wp-content/plugins/code-sample-generation/sdk_zip_files/XPSDK430.zip'
$BuildDir  = Join-Path $Root "build\$Configuration"
$PluginDir = Join-Path $BuildDir 'NavSight'      # fat-plugin folder that gets installed
$XplDir    = Join-Path $PluginDir 'win_x64'
$Xpl       = Join-Path $XplDir 'NavSight.xpl'

function Write-Step($msg) { Write-Host "==> $msg" -ForegroundColor Cyan }

# ---------------------------------------------------------------- SDK
function Initialize-Sdk {
    $marker = Join-Path $SdkDir 'CHeaders\XPLM\XPLMDefs.h'
    if ((Test-Path $marker) -and -not $FetchSdk) { return }
    # A fresh clone has no SDK: fetch it, so building from source is one command.
    if (-not $FetchSdk) { Write-Step "X-Plane SDK not found in $SdkDir - downloading it (one time)" }
    Write-Step "Downloading X-Plane SDK from $SdkUrl"
    $tmp = Join-Path ([IO.Path]::GetTempPath()) ("xpsdk_" + [Guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $tmp | Out-Null
    try {
        $zip = Join-Path $tmp 'XPSDK.zip'
        Invoke-WebRequest -Uri $SdkUrl -OutFile $zip -UseBasicParsing
        $extract = Join-Path $tmp 'x'
        New-Item -ItemType Directory -Path $extract | Out-Null
        # macOS framework symlinks can't be created on Windows; tar reports them
        # but the Windows headers/libs extract fine, so check for those instead.
        & tar.exe -xf $zip -C $extract 2>$null
        if (-not (Test-Path (Join-Path $extract 'SDK\Libraries\Win\XPLM_64.lib'))) { throw "SDK archive did not contain Windows libraries" }
        if (Test-Path $SdkDir) { Remove-Item $SdkDir -Recurse -Force }
        New-Item -ItemType Directory -Force -Path (Split-Path $SdkDir) | Out-Null
        Move-Item (Join-Path $extract 'SDK') $SdkDir
        Write-Step "SDK installed to $SdkDir"
    } finally {
        Remove-Item $tmp -Recurse -Force -ErrorAction SilentlyContinue
    }
}

# ---------------------------------------------------------------- MSVC environment
function Enter-MsvcEnvironment {
    if (Get-Command cl.exe -ErrorAction SilentlyContinue) { return }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) { throw "Visual Studio not found (no vswhere.exe). Install Visual Studio with the 'Desktop development with C++' workload." }
    $vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $vs) { throw "No Visual Studio install has the MSVC x64 toolset. Add the 'Desktop development with C++' workload in Visual Studio Installer." }
    Write-Step "Using Visual Studio at $vs"
    $env:PATH = "$(Split-Path $vswhere);$env:PATH"   # VsDevCmd looks for vswhere on PATH
    Import-Module (Join-Path $vs 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll')
    Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64' | Out-Null
    if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) { throw "cl.exe still not on PATH after entering the VS dev shell" }
}

# ---------------------------------------------------------------- compile helpers
$XplmDefines = 'XPLM200','XPLM210','XPLM300','XPLM301','XPLM303','XPLM400','XPLM410','XPLM420','XPLM430' |
    ForEach-Object { "/D$_=1" }

$CommonFlags = @(
    '/nologo', '/c', '/std:c++17', '/EHsc', '/W4', '/permissive-', '/utf-8', '/Zi', '/FS', '/MP',
    "/I$Root\src",
    '/external:W0', "/external:I$SdkDir\CHeaders\XPLM",
    '/DIBM=1', '/DAPL=0', '/DLIN=0', '/D_CRT_SECURE_NO_WARNINGS', '/DWIN32_LEAN_AND_MEAN', '/DNOMINMAX'
) + $XplmDefines

# Static CRT (/MT) so the .xpl has no VC++ redistributable dependency.
if ($Configuration -eq 'Release') { $CommonFlags += '/O2', '/MT', '/DNDEBUG' }
else                              { $CommonFlags += '/Od', '/MTd', '/RTC1' }

function Invoke-Native([string]$exe, [string[]]$arguments) {
    # Out-Host: tool output is for the console, not the caller's return value.
    & $exe @arguments | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "$exe failed with exit code $LASTEXITCODE" }
}

function Invoke-Compile([string[]]$sources, [string]$objDir) {
    New-Item -ItemType Directory -Force -Path $objDir | Out-Null
    Push-Location $objDir   # cl writes .obj files to the working directory
    try {
        Invoke-Native 'cl.exe' ($CommonFlags + "/Fd$objDir\compile.pdb" + $sources)
    } finally { Pop-Location }
    # Object names are the source basenames, so sources must have unique names.
    return $sources | ForEach-Object { Join-Path $objDir ([IO.Path]::GetFileNameWithoutExtension($_) + '.obj') }
}

function Assert-UniqueBasenames([string[]]$sources) {
    $dupes = $sources | Group-Object { [IO.Path]::GetFileNameWithoutExtension($_) } | Where-Object Count -gt 1
    if ($dupes) { throw "Source files must have unique names (objects share one folder): $($dupes.Name -join ', ')" }
}

# ---------------------------------------------------------------- main
if ($Clean) {
    Write-Step "Cleaning $BuildDir"
    if (Test-Path $BuildDir) { Remove-Item $BuildDir -Recurse -Force }
}

Initialize-Sdk
Enter-MsvcEnvironment

$pluginSources = Get-ChildItem (Join-Path $Root 'src') -Recurse -Filter *.cpp | ForEach-Object FullName
Assert-UniqueBasenames $pluginSources

Write-Step "Compiling plugin ($Configuration, $($pluginSources.Count) files)"
$objs = Invoke-Compile $pluginSources (Join-Path $BuildDir 'obj')

Write-Step "Linking $Xpl"
New-Item -ItemType Directory -Force -Path $XplDir | Out-Null
$linkFlags = @('/nologo', '/DLL', '/DEBUG', '/INCREMENTAL:NO', '/MACHINE:X64',
    "/OUT:$Xpl", "/PDB:$XplDir\NavSight.pdb", "/IMPLIB:$BuildDir\obj\NavSight.lib",
    "/LIBPATH:$SdkDir\Libraries\Win", 'XPLM_64.lib', 'opengl32.lib')
if ($Configuration -eq 'Release') { $linkFlags += '/OPT:REF', '/OPT:ICF' }
Invoke-Native 'link.exe' ($linkFlags + $objs)

$exports = & dumpbin.exe /nologo /exports $Xpl | Select-String 'XPlugin\w+' | ForEach-Object { $_.Matches[0].Value }
$required = 'XPluginStart', 'XPluginStop', 'XPluginEnable', 'XPluginDisable', 'XPluginReceiveMessage'
$missing = $required | Where-Object { $exports -notcontains $_ }
if ($missing) { throw "Plugin is missing required exports: $($missing -join ', ')" }
Write-Step "Built $Xpl (exports OK: $($required -join ', '))"

if ($Test) {
    # Tests link only the X-Plane-independent modules.
    $testSources = @(
        'src\core\Log.cpp', 'src\core\NavLog.cpp', 'src\core\Config.cpp',
        'src\nav\InstrumentModel.cpp', 'src\nav\AxisGuidance.cpp', 'src\nav\ApproachNaming.cpp', 'src\nav\NavValidation.cpp', 'src\nav\TrajectoryPredictor.cpp', 'src\nav\RunwayOverride.cpp', 'src\nav\SourceCheck.cpp',
        'src\render\Palette.cpp', 'src\render\RunwayPicker.cpp', 'src\render\DisplaySmoother.cpp',
        'src\render\Canvas.cpp', 'src\render\StrokeFont.cpp', 'src\render\RenderKit.cpp',
        'src\render\GuidanceText.cpp', 'src\render\DialLayers.cpp', 'src\render\PanelLayers.cpp',
        'src\render\InstrumentRenderer.cpp',
        'tests\TestMain.cpp'
    ) | ForEach-Object { Join-Path $Root $_ }
    Write-Step "Compiling tests"
    $testObjs = Invoke-Compile $testSources (Join-Path $BuildDir 'obj_test')
    $testExe = Join-Path $BuildDir 'NavSight_tests.exe'
    Invoke-Native 'link.exe' (@('/nologo', '/DEBUG', '/INCREMENTAL:NO', "/OUT:$testExe", "/PDB:$BuildDir\NavSight_tests.pdb") + $testObjs)
    Write-Step "Running tests"
    Invoke-Native $testExe @()
}

if ($Package) {
    if ($Configuration -ne 'Release') { throw "-Package needs a Release build (drop -Configuration Debug)." }
    $version = (Select-String -Path (Join-Path $Root 'src\core\Version.h') -Pattern 'kVersionString = "([^"]+)"').Matches[0].Groups[1].Value
    $stage = Join-Path $Root 'build\package'
    $pkgRoot = Join-Path $stage 'NavSight'
    if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
    New-Item -ItemType Directory -Force -Path (Join-Path $pkgRoot 'win_x64') | Out-Null
    Copy-Item $Xpl (Join-Path $pkgRoot 'win_x64')               # the plugin only - no PDB
    Copy-Item (Join-Path $Root 'docs\USER_GUIDE.md') $pkgRoot
    Copy-Item (Join-Path $Root 'CHANGELOG.md') $pkgRoot
    @"
NavSight $version - predictive navigation instrument for X-Plane 12 (Windows x64)

INSTALL
  1. Quit X-Plane.
  2. Drag this whole NavSight folder into  <X-Plane 12>\Resources\plugins\
     You should end up with:
         X-Plane 12\Resources\plugins\NavSight\win_x64\NavSight.xpl
  3. Start X-Plane and open  Plugins > NavSight > Show Instrument.

UPDATE     Replace the NavSight folder with the new one.
UNINSTALL  Delete Resources\plugins\NavSight.
UPGRADING FROM SMARTCDI  Delete Resources\plugins\SmartCDI first.

How to read the instrument: USER_GUIDE.md.
Source code and issues: https://github.com/Rohank050505/NavSight

NavSight is situational-awareness software for flight simulation only.
It is not a certified avionics or flight-director product and must not be
used for real-world navigation.
"@ | Set-Content (Join-Path $pkgRoot 'README.txt') -Encoding utf8
    $distDir = Join-Path $Root 'dist'
    New-Item -ItemType Directory -Force -Path $distDir | Out-Null
    $zip = Join-Path $distDir "NavSight-$version-win_x64.zip"
    if (Test-Path $zip) { Remove-Item $zip -Force }
    # Write entries with standard '/' separators explicitly: Windows
    # PowerShell 5.1 (Compress-Archive and .NET Framework's ZipFile) writes
    # '\', which some unzip tools mishandle.
    Add-Type -AssemblyName System.IO.Compression
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $archive =[IO.Compression.ZipFile]::Open($zip, [IO.Compression.ZipArchiveMode]::Create)
    try {
        Get-ChildItem $pkgRoot -Recurse -File | ForEach-Object {
            $entry = 'NavSight/' + $_.FullName.Substring($pkgRoot.Length + 1).Replace('\', '/')
            [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($archive, $_.FullName, $entry,
                [IO.Compression.CompressionLevel]::Optimal) | Out-Null
        }
    } finally { $archive.Dispose() }
    $entries = (Get-ChildItem $pkgRoot -Recurse -File | ForEach-Object { $_.FullName.Substring($stage.Length + 1) }) -join ', '
    Write-Step "Packaged $zip ($entries)"
}

if ($Install) {
    if (-not $XPlaneDir) {
        $installTxt = Join-Path $env:LOCALAPPDATA 'x-plane_install_12.txt'
        if (Test-Path $installTxt) { $XPlaneDir = (Get-Content $installTxt | Select-Object -First 1).Trim() }
    }
    if (-not $XPlaneDir -or -not (Test-Path (Join-Path $XPlaneDir 'X-Plane.exe'))) {
        throw "X-Plane 12 folder not found. Pass -XPlaneDir 'C:\path\to\X-Plane 12'."
    }
    if (Get-Process -Name 'X-Plane' -ErrorAction SilentlyContinue) {
        throw "X-Plane is running and has the plugin file locked. Quit X-Plane, then run -Install again."
    }
    # The plugin was called SmartCDI before v0.3.0. Remove that install so two
    # copies don't load side by side - but only if it is unmistakably ours.
    $legacy = Join-Path $XPlaneDir 'Resources\plugins\SmartCDI'
    if (Test-Path (Join-Path $legacy 'win_x64\SmartCDI.xpl')) {
        Write-Step "Removing the pre-rename install at $legacy (SmartCDI is now NavSight)"
        Remove-Item $legacy -Recurse -Force
    } elseif (Test-Path $legacy) {
        Write-Warning "$legacy exists but doesn't look like a SmartCDI build; left untouched."
    }

    $dest = Join-Path $XPlaneDir 'Resources\plugins\NavSight'
    Write-Step "Installing to $dest"
    New-Item -ItemType Directory -Force -Path (Join-Path $dest 'win_x64') | Out-Null
    Copy-Item (Join-Path $XplDir '*') (Join-Path $dest 'win_x64') -Force
    Write-Step "Installed. Start X-Plane and open Plugins > NavSight > Show Instrument."
}
