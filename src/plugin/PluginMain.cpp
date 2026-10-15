// X-Plane plugin entry points. Kept deliberately thin: everything is delegated
// to NavSightApp, and no C++ exception is allowed to cross into X-Plane.

#include <cstdio>
#include <exception>
#include <memory>

#include "XPLMDefs.h"
#include "XPLMUtilities.h"
#include "app/NavSightApp.h"

#if !defined(XPLM301)
#error "NavSight requires the XPLM301+ API (X-Plane 11.20 or newer); check build defines"
#endif

namespace {

std::unique_ptr<navsight::NavSightApp> g_app;

void reportFatal(const char* where, const char* what) {
    char line[512];
    std::snprintf(line, sizeof(line), "NavSight [FATAL] exception in %s: %s\n", where, what);
    XPLMDebugString(line);
}

template <typename Fn>
auto guarded(const char* where, Fn&& fn, decltype(fn()) onError) -> decltype(fn()) {
    try {
        return fn();
    } catch (const std::exception& e) {
        reportFatal(where, e.what());
    } catch (...) {
        reportFatal(where, "unknown");
    }
    return onError;
}

}  // namespace

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    return guarded("XPluginStart", [&] {
        g_app = std::make_unique<navsight::NavSightApp>();
        return g_app->start(outName, outSig, outDesc) ? 1 : 0;
    }, 0);
}

PLUGIN_API void XPluginStop(void) {
    guarded("XPluginStop", [] {
        if (g_app) g_app->stop();
        g_app.reset();
        return 0;
    }, 0);
}

PLUGIN_API int XPluginEnable(void) {
    return guarded("XPluginEnable", [] { return g_app && g_app->enable() ? 1 : 0; }, 0);
}

PLUGIN_API void XPluginDisable(void) {
    guarded("XPluginDisable", [] {
        if (g_app) g_app->disable();
        return 0;
    }, 0);
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void* inParam) {
    guarded("XPluginReceiveMessage", [&] {
        if (g_app) g_app->receiveMessage(inFrom, inMsg, inParam);
        return 0;
    }, 0);
}
