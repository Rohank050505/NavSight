#pragma once

#include <memory>
#include <string>
#include <vector>

#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "core/Config.h"
#include "data/SimDataSource.h"
#include "data/SimSnapshot.h"
#include "nav/InstrumentModel.h"
#include "ui/InstrumentWindow.h"

namespace navsight {

// Composition root. Owns every subsystem and maps X-Plane's plugin lifecycle
// onto them:
//
//   start   -> config, logging, data source, commands, menu, published datarefs
//   enable  -> instrument window, flight loop
//   disable -> save config, tear down window + flight loop
//   stop    -> unregister commands, menu, datarefs
//
// Per frame: flight loop samples SimDataSource -> InstrumentModel; the window's
// draw callback renders the model's InstrumentState.
class NavSightApp {
public:
    bool start(char* outName, char* outSig, char* outDesc);
    void stop();
    bool enable();
    void disable();
    void receiveMessage(int from, int message, void* param);

    bool windowVisible() const { return window_ && window_->isVisible(); }
    const InstrumentState& instrumentState() const { return model_.state(); }

private:
    // Commands
    struct CommandBinding {
        XPLMCommandRef ref = nullptr;
        void (NavSightApp::*action)() = nullptr;
    };
    void registerCommands();
    void unregisterCommands();
    XPLMCommandRef addCommand(const char* name, const char* description, void (NavSightApp::*action)());
    static int onCommand(XPLMCommandRef cmd, XPLMCommandPhase phase, void* refcon);

    void cmdToggleWindow();
    void cmdPopOut();
    void cmdResetPosition();
    void cmdToggleDebugLogging();
    void cmdTogglePanels();
    void onPanelsChanged(bool shown);
    void setBrightness(int level);
    void cmdBrightnessDay();
    void cmdBrightnessDim();
    void cmdBrightnessNight();
    void cmdBrightnessCycle();
    void cmdRunwayPicker();
    void cmdRunwayNext();
    void cmdRunwayPrevious();
    void cmdRunwayClear();
    void applyRunwayOverride(const std::string& runway);  // "" = clear (AUTO)
    void cycleRunwayOverride(int step);
    void logRunwayOverrideChanges();
    void logSourceCheckChanges();

    // Menu
    void createMenu();
    void destroyMenu();
    void syncMenuChecks();

    // Published datarefs (debugging / future companion app)
    void publishDataRefs();
    void unpublishDataRefs();
    void announceDataRefsToEditors();

    // Frame update
    static float onFlightLoop(float sinceLast, float sinceLastLoop, int counter, void* refcon);
    void update();
    void logValidityChanges();
    void logTrajectoryChanges();

    // Config
    // Settings file in X-Plane's preferences folder. The plugin was called
    // SmartCDI before v0.3.0; its settings file is read once if no NavSight
    // file exists yet (and left in place).
    static constexpr const char* kConfigFile = "NavSight.prf";
    static constexpr const char* kLegacyConfigFile = "SmartCDI.prf";
    std::string configPath(const char* fileName = kConfigFile) const;
    std::string pluginFolder() const;
    void captureWindowState();
    void saveConfig();

    Config config_;
    std::unique_ptr<SimDataSource> dataSource_;
    SimSnapshot snapshot_;
    InstrumentModel model_;
    std::unique_ptr<InstrumentWindow> window_;

    XPLMFlightLoopID flightLoop_ = nullptr;
    std::vector<CommandBinding> commands_;
    XPLMCommandRef cmdToggle_ = nullptr;
    XPLMCommandRef cmdDebug_ = nullptr;

    XPLMMenuID menu_ = nullptr;
    int pluginsMenuItem_ = -1;
    int toggleMenuItem_ = -1;
    int debugMenuItem_ = -1;
    int panelsMenuItem_ = -1;
    XPLMCommandRef cmdPanels_ = nullptr;
    XPLMCommandRef cmdBrightness_[3] = {};
    XPLMMenuID brightnessMenu_ = nullptr;
    int brightnessItems_[3] = {-1, -1, -1};
    XPLMCommandRef cmdRunwayPicker_ = nullptr;
    XPLMCommandRef cmdRunwayClear_ = nullptr;
    int runwayMenuItem_ = -1;
    bool lastRunwayMismatch_ = false;
    std::string lastSourceCheckKey_;
    double lastUpdateMs_ = 0.0;
    bool lastMenuVisible_ = false;

    std::vector<XPLMDataRef> publishedRefs_;
    std::vector<std::string> publishedNames_;
    bool announcedRefs_ = false;
    double lastDebugLogTime_ = 0.0;
    nav::SignalStatus lastLateralStatus_ = nav::SignalStatus::NoSignal;
    nav::SignalStatus lastVerticalStatus_ = nav::SignalStatus::NoSignal;
    bool lastLowGroundspeed_ = false;
    nav::Confidence lastLateralConfidence_ = nav::Confidence::None;
    bool lastTurnNow_ = false;
    bool lastVerticalOvershoot_ = false;
    bool lastPathAngleAssumed_ = true;
    double lastPathAngleDeg_ = 0.0;
};

}  // namespace navsight
