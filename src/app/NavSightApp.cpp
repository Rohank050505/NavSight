#include "app/NavSightApp.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <exception>
#include <vector>

#include "XPLMPlugin.h"
#include "core/Log.h"
#include "core/NavLog.h"
#include "nav/RunwayOverride.h"
#include "render/GlCanvas.h"
#include "core/Version.h"
#include "data/XPlaneDataSource.h"

namespace navsight {
namespace {

// Message used by DataRefEditor / DataRefTool to learn about plugin datarefs.
constexpr int kMsgAddDataRef = 0x01000000;
constexpr const char* kDataRefViewers[] = {"xplanesdk.examples.DataRefEditor", "com.leecbaker.datareftool"};

constexpr double kDebugLogIntervalSec = 10.0;

void copyOut(char* dst, const char* src) {
    // X-Plane guarantees 256-byte buffers for name/signature/description.
    std::strncpy(dst, src, 255);
    dst[255] = '\0';
}

#ifndef NDEBUG
void onSdkError(const char* message) { NAVS_ERROR("XPLM error callback: %s", message); }
#endif

// Published dataref accessors.
float getDisplayHeading(void* refcon) {
    return static_cast<float>(static_cast<NavSightApp*>(refcon)->instrumentState().displayHeadingDeg);
}
int getHeadingValid(void* refcon) {
    return static_cast<NavSightApp*>(refcon)->instrumentState().headingValid ? 1 : 0;
}
int getWindowVisible(void* refcon) { return static_cast<NavSightApp*>(refcon)->windowVisible() ? 1 : 0; }
int getMode(void* refcon) { return static_cast<int>(static_cast<NavSightApp*>(refcon)->instrumentState().mode); }
float getLateralDots(void* refcon) {
    return static_cast<float>(static_cast<NavSightApp*>(refcon)->instrumentState().lateral.out.positionDots);
}
float getVerticalDots(void* refcon) {
    return static_cast<float>(static_cast<NavSightApp*>(refcon)->instrumentState().vertical.out.positionDots);
}
// SignalStatus as int: 0 valid, 1 no signal, 2 flagged, 3 acquiring, 4 full scale, 5 not established
int getLateralStatus(void* refcon) {
    return static_cast<int>(static_cast<NavSightApp*>(refcon)->instrumentState().lateral.status);
}
int getVerticalStatus(void* refcon) {
    return static_cast<int>(static_cast<NavSightApp*>(refcon)->instrumentState().vertical.status);
}
int getAligned(void* refcon) { return static_cast<NavSightApp*>(refcon)->instrumentState().aligned ? 1 : 0; }
// 0 not applicable, 1 OK, 2 caution
int getCheckStatus(void* refcon) {
    return static_cast<int>(static_cast<NavSightApp*>(refcon)->instrumentState().sourceCheck.status);
}
int getCheckCode(void* refcon) {
    return nav::sourceCheckCodeNumber(static_cast<NavSightApp*>(refcon)->instrumentState().sourceCheck.code);
}

}  // namespace

// ---------------------------------------------------------------- lifecycle

bool NavSightApp::start(char* outName, char* outSig, char* outDesc) {
    copyOut(outName, kPluginName);
    copyOut(outSig, kPluginSignature);
    copyOut(outDesc, kPluginDescription);

    log::setSink([](const char* line) { XPLMDebugString(line); });
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    int xpVersion = 0, xplmVersion = 0;
    XPLMHostApplicationID host{};
    XPLMGetVersions(&xpVersion, &xplmVersion, &host);
    NAVS_INFO("starting v%s (X-Plane %d, XPLM %d)", kVersionString, xpVersion, xplmVersion);

    // Nav diagnostics go to their own file in the plugin folder, not Log.txt.
    char header[128];
    std::snprintf(header, sizeof(header), "NavSight v%s nav diagnostics (X-Plane %d)", kVersionString, xpVersion);
    if (navlog::open(pluginFolder() + "NavSight_nav.log", header))
        NAVS_INFO("nav diagnostics are written to %s", navlog::path().c_str());

#ifndef NDEBUG
    XPLMSetErrorCallback(&onSdkError);
    log::setDebugEnabled(true);
    NAVS_INFO("debug build: SDK error callback installed, debug logging on");
#endif

    if (config_.load(configPath())) {
        NAVS_INFO("loaded settings from %s", configPath().c_str());
    } else if (config_.load(configPath(kLegacyConfigFile))) {
        NAVS_INFO("migrated settings from %s (plugin renamed SmartCDI -> NavSight); will save to %s",
                  configPath(kLegacyConfigFile).c_str(), configPath().c_str());
    } else {
        NAVS_INFO("no settings file yet (%s); using defaults", configPath().c_str());
    }
    if (config_.debugLogging) log::setDebugEnabled(true);
    if (config_.layoutVersion < Config::kCurrentLayoutVersion && config_.window.isSet()) {
        // The instrument grew side panels; keep the user's position, adopt the new default size.
        config_.window = InstrumentWindow::defaultSizeAt(config_.window.left, config_.window.top);
        NAVS_INFO("settings from an older layout: window resized to the wide default");
    }
    config_.layoutVersion = Config::kCurrentLayoutVersion;

    InstrumentModel::Settings ms;
    ms.headingSmoothingSec = config_.headingSmoothingSec;
    ms.lateralClosureSec = config_.lateralClosureSec;
    ms.verticalClosureSec = config_.verticalClosureSec;
    ms.invertLateral = config_.invertLateral;
    ms.invertVertical = config_.invertVertical;
    model_.setSettings(ms);
    if (ms.invertLateral || ms.invertVertical)
        NAVS_INFO("deflection inversion active: lateral=%d vertical=%d", ms.invertLateral, ms.invertVertical);

    dataSource_ = std::make_unique<XPlaneDataSource>();
    if (!dataSource_->init()) NAVS_WARN("data source incomplete - see errors above");

    registerCommands();
    createMenu();
    publishDataRefs();
    return true;  // a degraded instrument is still better than no plugin
}

void NavSightApp::stop() {
    unpublishDataRefs();
    destroyMenu();
    unregisterCommands();
    dataSource_.reset();
    NAVS_NAV("session ended");
    navlog::close();
    NAVS_INFO("stopped");
}

// Folder holding this plugin, e.g. ".../Resources/plugins/NavSight/" (the
// .xpl lives in its win_x64 sub-folder in X-Plane's fat-plugin layout).
std::string NavSightApp::pluginFolder() const {
    char path[1024] = {};
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
    std::string dir(path);
    auto parent = [](std::string& p) {
        const auto sep = p.find_last_of("/\\");
        p = sep == std::string::npos ? std::string() : p.substr(0, sep);
    };
    parent(dir);  // drop NavSight.xpl
    const auto sep = dir.find_last_of("/\\");
    const std::string leaf = sep == std::string::npos ? dir : dir.substr(sep + 1);
    if (leaf == "win_x64" || leaf == "mac_x64" || leaf == "lin_x64") parent(dir);
    return dir.empty() ? std::string() : dir + "/";
}

bool NavSightApp::enable() {
    window_ = std::make_unique<InstrumentWindow>([this]() -> const InstrumentState& { return model_.state(); });
    window_->setShowPanels(config_.showPanels);
    window_->setPanelsListener([this](bool shown) { onPanelsChanged(shown); });
    window_->setBrightness(static_cast<render::Brightness>(config_.brightness));
    window_->setRunwayListener([this](const render::RunwayPicker::Result& r) {
        applyRunwayOverride(r.action == render::RunwayPicker::Action::Apply ? r.runway : std::string());
    });
    render::GlCanvas::setFastPath(config_.fastRendering);
    if (!window_->create(config_.window, config_.windowVisible)) {
        window_.reset();
        NAVS_ERROR("enable failed: no window");
        return false;
    }

    XPLMCreateFlightLoop_t loop{};
    loop.structSize = sizeof(loop);
    loop.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    loop.callbackFunc = &NavSightApp::onFlightLoop;
    loop.refcon = this;
    flightLoop_ = XPLMCreateFlightLoop(&loop);
    XPLMScheduleFlightLoop(flightLoop_, -1.0f, 1);  // every frame

    model_.reset();
    syncMenuChecks();
    NAVS_INFO("enabled");
    return true;
}

void NavSightApp::disable() {
    captureWindowState();
    saveConfig();
    if (flightLoop_) {
        XPLMDestroyFlightLoop(flightLoop_);
        flightLoop_ = nullptr;
    }
    window_.reset();
    syncMenuChecks();
    NAVS_INFO("disabled");
}

void NavSightApp::receiveMessage(int /*from*/, int message, void* param) {
    if (message == XPLM_MSG_PLANE_LOADED && param == nullptr) {
        // User aircraft changed. Nothing aircraft-specific is cached yet, but
        // clear filters so the rose doesn't glide from the old heading.
        NAVS_DEBUG("user aircraft loaded - resetting instrument model");
        model_.reset();
    }
}

// ---------------------------------------------------------------- frame

float NavSightApp::onFlightLoop(float, float, int, void* refcon) {
    try {
        static_cast<NavSightApp*>(refcon)->update();
    } catch (const std::exception& e) {
        NAVS_ERROR("update failed: %s", e.what());
    } catch (...) {
        NAVS_ERROR("update failed: unknown exception");
    }
    return -1.0f;
}

void NavSightApp::update() {
    if (!announcedRefs_) announceDataRefsToEditors();  // other plugins are loaded by now

    using Clock = std::chrono::steady_clock;
    const Clock::time_point t0 = Clock::now();
    dataSource_->sample(snapshot_);
    model_.update(snapshot_);
    lastUpdateMs_ = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    logValidityChanges();
    logTrajectoryChanges();
    logRunwayOverrideChanges();
    logSourceCheckChanges();
    syncMenuChecks();  // the window's close box can hide it behind our back

    if (log::debugEnabled() && snapshot_.simTimeSec - lastDebugLogTime_ >= kDebugLogIntervalSec) {
        lastDebugLogTime_ = snapshot_.simTimeSec;
        const InstrumentState& s = model_.state();
        const NavSnapshot& n = snapshot_.nav;
        NAVS_DEBUG("t=%.1f hdg_mag=%.1f (valid=%d) display=%.1f readout=%03d paused=%d window=%d",
                   snapshot_.simTimeSec, snapshot_.magHeadingDeg, snapshot_.headingValid ? 1 : 0,
                   s.displayHeadingDeg, s.headingReadoutDeg, s.paused ? 1 : 0, windowVisible() ? 1 : 0);
        NAVS_NAV("t=%.1f sample src=%s '%s' raw hdef=%.2f(%d) vdef=%.2f(%d) crs=%.0f brg=%.0f dist=%.1f "
                 "gps_cdi=%d gs=%.0fkt -> mode=%d lat=%.2f rate=%.3f cmd=%d dir=%d | vert=%.2f rate=%.3f cmd=%d "
                 "dir=%d aligned=%d",
                 snapshot_.simTimeSec, s.sourceName.c_str(), s.approachLabel.c_str(), n.hdefDots,
                 n.lateralValid ? 1 : 0, n.vdefDots, n.verticalValid ? 1 : 0, n.courseDegMag, n.bearingDegMag,
                 n.dmeNm, n.gpsCdiMode, snapshot_.groundspeedKt, static_cast<int>(s.mode),
                 s.lateral.out.positionDots, s.lateral.out.rateDotsPerSec, static_cast<int>(s.lateral.out.command),
                 s.lateral.out.direction, s.vertical.out.positionDots, s.vertical.out.rateDotsPerSec,
                 static_cast<int>(s.vertical.out.command), s.vertical.out.direction, s.aligned ? 1 : 0);
        if (window_) {
            const auto& d = window_->lastDrawStats();
            NAVS_NAV("t=%.1f perf: update %.3f ms | draw build %.3f ms, GL submit %.3f ms, %zu triangles (%s)",
                     snapshot_.simTimeSec, lastUpdateMs_, d.buildMs, d.flushMs, d.triangles,
                     d.fastPath ? "vertex arrays" : "immediate mode");
        }
    }
}

// Runway override: set/cleared by the pilot, dropped on a source change, and
// runway-vs-course mismatches - all to the nav log.
void NavSightApp::logRunwayOverrideChanges() {
    const InstrumentState& s = model_.state();
    const double t = snapshot_.simTimeSec;
    if (model_.overrideClearedBySourceChange())
        NAVS_NAV("t=%.1f runway override cleared: nav source changed (now '%s' on %s)", t,
                 snapshot_.approach.label.c_str(), s.sourceName.c_str());
    if (s.runwayCourseMismatch != lastRunwayMismatch_) {
        NAVS_NAV("t=%.1f runway override %s: RWY %s vs course %.0f", t,
                 s.runwayCourseMismatch ? "DOES NOT MATCH the course" : "matches the course", s.runway.c_str(),
                 s.courseDegMag);
        lastRunwayMismatch_ = s.runwayCourseMismatch;
    }
}

// Source check: every change of the shown result (and of a caution's text).
void NavSightApp::logSourceCheckChanges() {
    const nav::SourceCheckResult& r = model_.state().sourceCheck;
    const std::string key = r.code + "|" + r.banner;
    if (key == lastSourceCheckKey_) return;
    lastSourceCheckKey_ = key;
    const char* status = r.status == nav::SourceCheckStatus::Caution ? "CAUTION"
                         : r.status == nav::SourceCheckStatus::Ok    ? "OK"
                                                                     : "n/a";
    NAVS_NAV("t=%.1f source check: %s %s%s%s %s", snapshot_.simTimeSec, status,
             r.caution() ? r.code.c_str() : "", r.caution() ? ": " : "", r.banner.c_str(), r.detail.c_str());
}

void NavSightApp::applyRunwayOverride(const std::string& runway) {
    if (runway.empty()) {
        model_.clearRunwayOverride();
        NAVS_NAV("t=%.1f runway override cleared by pilot (AUTO)", snapshot_.simTimeSec);
        return;
    }
    if (model_.setRunwayOverride(runway))
        NAVS_NAV("t=%.1f runway override set: RWY %s (manual) on %s '%s'", snapshot_.simTimeSec,
                 model_.runwayOverride().runway.c_str(), model_.state().sourceName.c_str(),
                 snapshot_.approach.label.c_str());
    else
        NAVS_WARN("runway override '%s' rejected: not a runway designator", runway.c_str());
}

// Next / previous runway number around the selected course (bindable).
void NavSightApp::cycleRunwayOverride(int step) {
    const InstrumentState& s = model_.state();
    if (!s.courseValid) {
        NAVS_NAV("t=%.1f runway override ignored: no valid course", snapshot_.simTimeSec);
        return;
    }
    const std::vector<std::string> c = nav::runwayCandidates(s.courseDegMag);
    const std::string current = model_.runwayOverride().active ? model_.runwayOverride().runway.substr(0, 2) : "";
    int idx = -1;
    for (int i = 0; i < static_cast<int>(c.size()); ++i)
        if (c[i] == current) idx = i;
    const int n = static_cast<int>(c.size());
    const int next = idx < 0 ? 1 : ((idx + step) % n + n) % n;
    applyRunwayOverride(c[static_cast<std::size_t>(next)]);
}

// Log every validation transition (not every frame) to the nav log so in-sim
// behaviour can be audited without enabling verbose logging.
void NavSightApp::logValidityChanges() {
    const nav::NavValidity& v = model_.validity();
    const double t = snapshot_.simTimeSec;
    if (v.discontinuity) NAVS_NAV("t=%.1f nav discontinuity: %s - predictors reset", t, v.discontinuityReason.c_str());

    const nav::SignalStatus lat = v.lateral.status, vert = v.vertical.status;
    if (lat != lastLateralStatus_ || vert != lastVerticalStatus_ || v.lowGroundspeed != lastLowGroundspeed_) {
        NAVS_NAV("t=%.1f nav validity: lateral=%s vertical=%s%s (hdef=%.2f vdef=%.2f gs=%.0fkt)", t,
                 nav::toString(lat), nav::toString(vert), v.lowGroundspeed ? " LOW-GS" : "", snapshot_.nav.hdefDots,
                 snapshot_.nav.vdefDots, snapshot_.groundspeedKt);
        lastLateralStatus_ = lat;
        lastVerticalStatus_ = vert;
        lastLowGroundspeed_ = v.lowGroundspeed;
    }
}

// Trajectory-estimate events: confidence changes (incl. scale calibration),
// roll-out "turn now" warnings, vertical overshoot, glidepath-angle source.
void NavSightApp::logTrajectoryChanges() {
    const nav::TrajectoryOutput& tr = model_.state().trajectory;
    const nav::LateralPrediction& L = tr.lateral;
    const nav::VerticalPrediction& V = tr.vertical;
    const double t = snapshot_.simTimeSec;

    if (L.confidence != lastLateralConfidence_) {
        NAVS_NAV("t=%.1f trajectory lateral confidence: %s -> %s (nm/dot=%.3f deg/dot=%.2f tke=%.1f gs=%.0fkt)", t,
                 nav::toString(lastLateralConfidence_), nav::toString(L.confidence), L.nmPerDot, L.degPerDot,
                 L.trackErrorDeg, snapshot_.groundspeedKt);
        lastLateralConfidence_ = L.confidence;
    }
    if (L.turnNow != lastTurnNow_) {
        if (L.turnNow)
            NAVS_NAV("t=%.1f trajectory: TURN NOW - inside roll-out lead %.2f nm, overshoot %.2f dot (tke=%.1f "
                     "lat=%.2f)",
                     t, L.leadNm, L.rolloutOvershootDots, L.trackErrorDeg, model_.state().lateral.out.positionDots);
        else
            NAVS_NAV("t=%.1f trajectory: turn-now cleared (outcome %s)", t, nav::toString(L.outcome));
        lastTurnNow_ = L.turnNow;
    }
    if (V.overshoot != lastVerticalOvershoot_) {
        NAVS_NAV("t=%.1f trajectory vertical overshoot %s (vs=%.0f req=%.0f err=%.0f fpm, cross %.1fs)", t,
                 V.overshoot ? "PREDICTED" : "cleared", snapshot_.verticalSpeedFpm, V.requiredVsFpm, V.vsErrorFpm,
                 V.crossSec);
        lastVerticalOvershoot_ = V.overshoot;
    }
    if (V.valid && (V.pathAngleAssumed != lastPathAngleAssumed_ || V.pathAngleDeg != lastPathAngleDeg_)) {
        NAVS_NAV("t=%.1f glidepath angle %.2f deg (%s)", t, V.pathAngleDeg,
                 V.pathAngleAssumed ? "assumed - no data" : "from nav data");
        lastPathAngleAssumed_ = V.pathAngleAssumed;
        lastPathAngleDeg_ = V.pathAngleDeg;
    }
}

// ---------------------------------------------------------------- commands

XPLMCommandRef NavSightApp::addCommand(const char* name, const char* description,
                                       void (NavSightApp::*action)()) {
    XPLMCommandRef ref = XPLMCreateCommand(name, description);
    if (!ref) {
        NAVS_ERROR("could not create command %s", name);
        return nullptr;
    }
    XPLMRegisterCommandHandler(ref, &NavSightApp::onCommand, 1, this);
    commands_.push_back({ref, action});
    return ref;
}

void NavSightApp::registerCommands() {
    cmdToggle_ = addCommand("navsight/window/toggle", "NavSight: show/hide instrument window",
                            &NavSightApp::cmdToggleWindow);
    addCommand("navsight/window/pop_out", "NavSight: pop instrument out to its own OS window",
               &NavSightApp::cmdPopOut);
    addCommand("navsight/window/reset_position", "NavSight: move instrument window back to default",
               &NavSightApp::cmdResetPosition);
    cmdPanels_ = addCommand("navsight/view/toggle_panels", "NavSight: show/collapse the lateral & vertical panels",
                            &NavSightApp::cmdTogglePanels);
    cmdBrightness_[0] = addCommand("navsight/view/brightness_day", "NavSight: day brightness",
                                   &NavSightApp::cmdBrightnessDay);
    cmdBrightness_[1] = addCommand("navsight/view/brightness_dim", "NavSight: dim brightness",
                                   &NavSightApp::cmdBrightnessDim);
    cmdBrightness_[2] = addCommand("navsight/view/brightness_night", "NavSight: night brightness",
                                   &NavSightApp::cmdBrightnessNight);
    addCommand("navsight/view/brightness", "NavSight: cycle brightness day / dim / night",
               &NavSightApp::cmdBrightnessCycle);
    cmdRunwayPicker_ = addCommand("navsight/runway/picker", "NavSight: open/close the runway override picker",
                                  &NavSightApp::cmdRunwayPicker);
    addCommand("navsight/runway/next", "NavSight: runway override - next runway number around the course",
               &NavSightApp::cmdRunwayNext);
    addCommand("navsight/runway/previous", "NavSight: runway override - previous runway number around the course",
               &NavSightApp::cmdRunwayPrevious);
    cmdRunwayClear_ = addCommand("navsight/runway/clear", "NavSight: clear the runway override (back to nav data)",
                                 &NavSightApp::cmdRunwayClear);
    cmdDebug_ = addCommand("navsight/debug/toggle_logging", "NavSight: toggle verbose logging to Log.txt",
                           &NavSightApp::cmdToggleDebugLogging);
}

void NavSightApp::unregisterCommands() {
    for (const CommandBinding& b : commands_) XPLMUnregisterCommandHandler(b.ref, &NavSightApp::onCommand, 1, this);
    commands_.clear();
}

int NavSightApp::onCommand(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon) {
    if (phase != xplm_CommandBegin) return 0;
    auto* self = static_cast<NavSightApp*>(refcon);
    for (const CommandBinding& b : self->commands_) {
        if (b.ref == cmd) {
            try {
                (self->*b.action)();
            } catch (...) {
                NAVS_ERROR("command handler threw");
            }
            break;
        }
    }
    return 0;  // handled; these commands are ours alone
}

void NavSightApp::cmdToggleWindow() {
    if (!window_) {
        NAVS_WARN("toggle ignored: plugin is disabled");
        return;
    }
    window_->toggle();
    syncMenuChecks();
}

void NavSightApp::cmdPopOut() {
    if (window_) window_->popOut();
}

void NavSightApp::cmdResetPosition() {
    if (window_) window_->resetPosition();
}

void NavSightApp::setBrightness(int level) {
    config_.brightness = std::clamp(level, 0, 2);
    if (window_) window_->setBrightness(static_cast<render::Brightness>(config_.brightness));
    NAVS_INFO("brightness %s", render::toString(static_cast<render::Brightness>(config_.brightness)));
    syncMenuChecks();
}
void NavSightApp::cmdBrightnessDay() { setBrightness(0); }
void NavSightApp::cmdBrightnessDim() { setBrightness(1); }
void NavSightApp::cmdBrightnessNight() { setBrightness(2); }
void NavSightApp::cmdBrightnessCycle() { setBrightness((config_.brightness + 1) % 3); }

void NavSightApp::cmdRunwayPicker() {
    if (window_) window_->toggleRunwayPicker();
}
void NavSightApp::cmdRunwayNext() { cycleRunwayOverride(+1); }
void NavSightApp::cmdRunwayPrevious() { cycleRunwayOverride(-1); }
void NavSightApp::cmdRunwayClear() { applyRunwayOverride({}); }

void NavSightApp::cmdTogglePanels() {
    // Works while the plugin is disabled too: the choice is applied on enable.
    if (window_) window_->setShowPanels(!window_->showPanels());  // listener persists + syncs menu
    else onPanelsChanged(!config_.showPanels);
}

void NavSightApp::onPanelsChanged(bool shown) {
    config_.showPanels = shown;
    NAVS_INFO("guidance panels %s", shown ? "shown" : "collapsed");
    syncMenuChecks();
}

void NavSightApp::cmdToggleDebugLogging() {
    config_.debugLogging = !config_.debugLogging;
    log::setDebugEnabled(config_.debugLogging);
    lastDebugLogTime_ = 0.0;
    NAVS_INFO("debug logging %s", config_.debugLogging ? "ON" : "OFF");
    syncMenuChecks();
}

// ---------------------------------------------------------------- menu

namespace {
void menuHandler(void*, void*) {}  // all items are command-bound
}  // namespace

void NavSightApp::createMenu() {
    XPLMMenuID plugins = XPLMFindPluginsMenu();
    pluginsMenuItem_ = XPLMAppendMenuItem(plugins, kPluginName, nullptr, 0);
    menu_ = XPLMCreateMenu(kPluginName, plugins, pluginsMenuItem_, &menuHandler, nullptr);
    if (!menu_) {
        NAVS_ERROR("could not create Plugins > NavSight menu (commands still work)");
        return;
    }
    toggleMenuItem_ = XPLMAppendMenuItemWithCommand(menu_, "Show Instrument", cmdToggle_);
    XPLMAppendMenuItemWithCommand(menu_, "Pop Out Window", XPLMFindCommand("navsight/window/pop_out"));
    XPLMAppendMenuItemWithCommand(menu_, "Reset Window Position",
                                  XPLMFindCommand("navsight/window/reset_position"));
    XPLMAppendMenuSeparator(menu_);
    panelsMenuItem_ = XPLMAppendMenuItemWithCommand(menu_, "Show Guidance Panels", cmdPanels_);
    const int brightnessItem = XPLMAppendMenuItem(menu_, "Brightness", nullptr, 0);
    brightnessMenu_ = XPLMCreateMenu("Brightness", menu_, brightnessItem, &menuHandler, nullptr);
    if (brightnessMenu_) {
        static constexpr const char* kNames[] = {"Day", "Dim", "Night"};
        for (int i = 0; i < 3; ++i)
            brightnessItems_[i] = XPLMAppendMenuItemWithCommand(brightnessMenu_, kNames[i], cmdBrightness_[i]);
    }
    XPLMAppendMenuSeparator(menu_);
    runwayMenuItem_ = XPLMAppendMenuItemWithCommand(menu_, "Runway Override...", cmdRunwayPicker_);
    XPLMAppendMenuItemWithCommand(menu_, "Clear Runway Override", cmdRunwayClear_);
    debugMenuItem_ = XPLMAppendMenuItemWithCommand(menu_, "Verbose Logging", cmdDebug_);
    lastMenuVisible_ = !windowVisible();  // force first sync
    syncMenuChecks();
}

void NavSightApp::destroyMenu() {
    if (brightnessMenu_) XPLMDestroyMenu(brightnessMenu_);  // submenu first
    brightnessMenu_ = nullptr;
    if (menu_) XPLMDestroyMenu(menu_);
    if (pluginsMenuItem_ >= 0) XPLMRemoveMenuItem(XPLMFindPluginsMenu(), pluginsMenuItem_);
    menu_ = nullptr;
    pluginsMenuItem_ = toggleMenuItem_ = debugMenuItem_ = panelsMenuItem_ = runwayMenuItem_ = -1;
}

void NavSightApp::syncMenuChecks() {
    if (!menu_) return;
    const bool visible = windowVisible();
    if (visible != lastMenuVisible_) {
        XPLMCheckMenuItem(menu_, toggleMenuItem_, visible ? xplm_Menu_Checked : xplm_Menu_Unchecked);
        lastMenuVisible_ = visible;
    }
    XPLMCheckMenuItem(menu_, debugMenuItem_, log::debugEnabled() ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(menu_, panelsMenuItem_, config_.showPanels ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    if (brightnessMenu_)
        for (int i = 0; i < 3; ++i)
            XPLMCheckMenuItem(brightnessMenu_, brightnessItems_[i],
                              config_.brightness == i ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(menu_, runwayMenuItem_,
                      model_.runwayOverride().active ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}

// ---------------------------------------------------------------- datarefs

void NavSightApp::publishDataRefs() {
    struct Def {
        const char* name;
        XPLMDataTypeID type;
        XPLMGetDatai_f geti;
        XPLMGetDataf_f getf;
    };
    const Def defs[] = {
        {"navsight/state/display_heading_deg", xplmType_Float, nullptr, &getDisplayHeading},
        {"navsight/state/heading_valid", xplmType_Int, &getHeadingValid, nullptr},
        {"navsight/window/visible", xplmType_Int, &getWindowVisible, nullptr},
        {"navsight/state/mode", xplmType_Int, &getMode, nullptr},
        {"navsight/guidance/lateral_dots", xplmType_Float, nullptr, &getLateralDots},
        {"navsight/guidance/vertical_dots", xplmType_Float, nullptr, &getVerticalDots},
        {"navsight/guidance/aligned", xplmType_Int, &getAligned, nullptr},
        {"navsight/guidance/lateral_status", xplmType_Int, &getLateralStatus, nullptr},
        {"navsight/guidance/vertical_status", xplmType_Int, &getVerticalStatus, nullptr},
        {"navsight/check/status", xplmType_Int, &getCheckStatus, nullptr},
        {"navsight/check/code", xplmType_Int, &getCheckCode, nullptr},
    };
    for (const Def& d : defs) {
        XPLMDataRef ref = XPLMRegisterDataAccessor(d.name, d.type, 0, d.geti, nullptr, d.getf, nullptr, nullptr,
                                                   nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                                                   this, nullptr);
        if (!ref) {
            NAVS_WARN("could not publish dataref %s", d.name);
            continue;
        }
        publishedRefs_.push_back(ref);
        publishedNames_.emplace_back(d.name);
    }
}

void NavSightApp::unpublishDataRefs() {
    for (XPLMDataRef ref : publishedRefs_) XPLMUnregisterDataAccessor(ref);
    publishedRefs_.clear();
    publishedNames_.clear();
}

void NavSightApp::announceDataRefsToEditors() {
    announcedRefs_ = true;
    for (const char* sig : kDataRefViewers) {
        const XPLMPluginID id = XPLMFindPluginBySignature(sig);
        if (id == XPLM_NO_PLUGIN_ID) continue;
        for (const std::string& name : publishedNames_)
            XPLMSendMessageToPlugin(id, kMsgAddDataRef, const_cast<char*>(name.c_str()));
        NAVS_DEBUG("announced %zu datarefs to %s", publishedNames_.size(), sig);
    }
}

// ---------------------------------------------------------------- config

std::string NavSightApp::configPath(const char* fileName) const {
    char buf[1024] = {};
    XPLMGetPrefsPath(buf);  // full path of X-Plane's own prefs file
    std::string path(buf);
    const auto sep = path.find_last_of("/\\");
    path = (sep == std::string::npos) ? std::string() : path.substr(0, sep + 1);
    return path + fileName;
}

void NavSightApp::captureWindowState() {
    if (!window_) return;
    config_.windowVisible = window_->isVisible();
    const WindowGeometry g = window_->geometry();
    if (g.isSet()) config_.window = g;
}

void NavSightApp::saveConfig() {
    if (config_.save(configPath())) NAVS_DEBUG("settings saved to %s", configPath().c_str());
    else NAVS_WARN("could not save settings to %s", configPath().c_str());
}

}  // namespace navsight
