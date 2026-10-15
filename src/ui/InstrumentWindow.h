#pragma once

#include <chrono>
#include <cstddef>
#include <functional>

#include "XPLMDisplay.h"
#include "core/Config.h"
#include "render/GlCanvas.h"
#include "render/InstrumentRenderer.h"

namespace navsight {

struct InstrumentState;

// The floating NavSight window: an X-Plane "modern" window (XPLMCreateWindowEx)
// with X-Plane's own decoration, so the user gets a title bar to drag, a close
// button and resize handles for free. It can also be popped out into a
// separate OS window (e.g. for a second monitor).
//
// The window only hosts the renderer; it never touches sim data.
class InstrumentWindow {
public:
    using StateProvider = std::function<const InstrumentState&()>;

    explicit InstrumentWindow(StateProvider provider) : provider_(std::move(provider)) {}
    ~InstrumentWindow() { destroy(); }
    InstrumentWindow(const InstrumentWindow&) = delete;
    InstrumentWindow& operator=(const InstrumentWindow&) = delete;

    // Create the X-Plane window. `saved` geometry is used if it is still on
    // screen, otherwise a default position is chosen.
    bool create(const WindowGeometry& saved, bool visible);
    void destroy();
    bool exists() const { return id_ != nullptr; }

    bool isVisible() const;
    void setVisible(bool visible);
    void toggle() { setVisible(!isVisible()); }

    void popOut();
    void resetPosition();

    // Current in-sim geometry (unchanged while popped out, since popped-out
    // windows use OS coordinates that we don't persist).
    WindowGeometry geometry() const;

    // Default window size anchored at a top-left corner.
    static WindowGeometry defaultSizeAt(int left, int top);

    // LATERAL / VERTICAL guidance panels: shown, or collapsed to edge tabs.
    // Toggled by clicking a panel/tab or from the menu; `listener` is told
    // about every change (to persist it and update the menu check mark).
    bool showPanels() const { return view_.showPanels; }
    void setShowPanels(bool show);
    void setPanelsListener(std::function<void(bool)> listener) { panelsListener_ = std::move(listener); }

    render::Brightness brightness() const { return view_.brightness; }
    void setBrightness(render::Brightness b) { view_.brightness = b; }

    // Runway-override picker (opened by clicking the APPROACH box or by
    // command). Apply/Clear results go to the listener.
    using RunwayListener = std::function<void(const render::RunwayPicker::Result&)>;
    void setRunwayListener(RunwayListener listener) { runwayListener_ = std::move(listener); }
    void toggleRunwayPicker();
    bool runwayPickerOpen() const { return view_.picker.open; }

    struct DrawStats {
        std::size_t triangles = 0;
        double buildMs = 0.0;   // renderer + canvas geometry
        double flushMs = 0.0;   // GL submission
        bool fastPath = false;
    };
    const DrawStats& lastDrawStats() const { return stats_; }

private:
    static void onDraw(XPLMWindowID id, void* refcon);
    static int onMouseClick(XPLMWindowID id, int x, int y, XPLMMouseStatus status, void* refcon);
    static int onRightClick(XPLMWindowID id, int x, int y, XPLMMouseStatus status, void* refcon);
    static void onKey(XPLMWindowID id, char key, XPLMKeyFlags flags, char vkey, void* refcon, int losingFocus);
    static XPLMCursorStatus onCursor(XPLMWindowID id, int x, int y, void* refcon);
    static int onWheel(XPLMWindowID id, int x, int y, int wheel, int clicks, void* refcon);

    void draw();
    void click(int x, int y);
    void key(char ch, int vkey);
    void openRunwayPicker();
    void closeRunwayPicker();
    void handlePickerResult(const render::RunwayPicker::Result& r);
    render::InstrumentRenderer::Viewport viewport() const;
    static WindowGeometry defaultGeometry();
    static bool isOnScreen(const WindowGeometry& g);

    StateProvider provider_;
    XPLMWindowID id_ = nullptr;
    render::GlCanvas canvas_;
    render::InstrumentRenderer renderer_;
    render::InstrumentRenderer::ViewOptions view_;
    render::DisplaySmoother smoother_;
    render::DisplayFx fx_;
    std::chrono::steady_clock::time_point lastDraw_{};
    bool haveDrawn_ = false;
    DrawStats stats_;
    std::function<void(bool)> panelsListener_;
    RunwayListener runwayListener_;
    WindowGeometry lastInSimGeometry_;
    bool drawErrorLogged_ = false;
};

}  // namespace navsight
