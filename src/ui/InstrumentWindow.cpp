#include "ui/InstrumentWindow.h"

#include <algorithm>
#include <exception>

#include "core/Log.h"
#include "nav/InstrumentState.h"
#include "render/Palette.h"

namespace navsight {
namespace {

constexpr int kDefaultWidth = 600;   // boxels; the instrument is square
constexpr int kDefaultHeight = 600;
constexpr int kMinSize = 200;
constexpr int kMaxSize = 2000;
constexpr const char* kTitle = "NavSight";

}  // namespace

bool InstrumentWindow::create(const WindowGeometry& saved, bool visible) {
    if (id_) return true;

    WindowGeometry g = saved;
    if (!g.isSet() || !isOnScreen(g)) {
        if (g.isSet()) NAVS_INFO("saved window position is off-screen, using default placement");
        g = defaultGeometry();
    }

    XPLMCreateWindow_t params{};
    params.structSize = sizeof(params);
    params.left = g.left;
    params.top = g.top;
    params.right = g.right;
    params.bottom = g.bottom;
    params.visible = visible ? 1 : 0;
    params.drawWindowFunc = &InstrumentWindow::onDraw;
    params.handleMouseClickFunc = &InstrumentWindow::onMouseClick;
    params.handleRightClickFunc = &InstrumentWindow::onRightClick;
    params.handleKeyFunc = &InstrumentWindow::onKey;
    params.handleCursorFunc = &InstrumentWindow::onCursor;
    params.handleMouseWheelFunc = &InstrumentWindow::onWheel;
    params.refcon = this;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    params.layer = xplm_WindowLayerFloatingWindows;

    id_ = XPLMCreateWindowEx(&params);
    if (!id_) {
        NAVS_ERROR("XPLMCreateWindowEx failed - instrument window unavailable");
        return false;
    }
    XPLMSetWindowTitle(id_, kTitle);
    XPLMSetWindowResizingLimits(id_, kMinSize, kMinSize, kMaxSize, kMaxSize);
    XPLMSetWindowPositioningMode(id_, xplm_WindowPositionFree, -1);
    lastInSimGeometry_ = g;
    NAVS_INFO("window created at L%d T%d R%d B%d (%dx%d), visible=%d", g.left, g.top, g.right, g.bottom,
              g.width(), g.height(), visible ? 1 : 0);
    return true;
}

void InstrumentWindow::destroy() {
    if (!id_) return;
    if (view_.picker.open) closeRunwayPicker();
    XPLMDestroyWindow(id_);
    id_ = nullptr;
}

bool InstrumentWindow::isVisible() const { return id_ && XPLMGetWindowIsVisible(id_) != 0; }

void InstrumentWindow::setVisible(bool visible) {
    if (!id_) return;
    XPLMSetWindowIsVisible(id_, visible ? 1 : 0);
    NAVS_DEBUG("window %s", visible ? "shown" : "hidden");
}

void InstrumentWindow::popOut() {
    if (!id_) return;
    setVisible(true);
    XPLMSetWindowPositioningMode(id_, xplm_WindowPopOut, -1);
    NAVS_INFO("window popped out to OS window");
}

void InstrumentWindow::resetPosition() {
    if (!id_) return;
    if (XPLMWindowIsPoppedOut(id_)) XPLMSetWindowPositioningMode(id_, xplm_WindowPositionFree, -1);
    const WindowGeometry g = defaultGeometry();
    XPLMSetWindowGeometry(id_, g.left, g.top, g.right, g.bottom);
    lastInSimGeometry_ = g;
    setVisible(true);
    NAVS_INFO("window position reset");
}

WindowGeometry InstrumentWindow::geometry() const {
    if (!id_ || XPLMWindowIsPoppedOut(id_)) return lastInSimGeometry_;
    WindowGeometry g;
    XPLMGetWindowGeometry(id_, &g.left, &g.top, &g.right, &g.bottom);
    return g;
}

WindowGeometry InstrumentWindow::defaultGeometry() {
    int l = 0, t = 0, r = 0, b = 0;
    XPLMGetScreenBoundsGlobal(&l, &t, &r, &b);
    WindowGeometry g;
    g.left = l + 80;
    g.top = t - 140;
    g.right = g.left + kDefaultWidth;
    g.bottom = g.top - kDefaultHeight;
    return g;
}

WindowGeometry InstrumentWindow::defaultSizeAt(int left, int top) {
    return {left, top, left + kDefaultWidth, top - kDefaultHeight};
}

bool InstrumentWindow::isOnScreen(const WindowGeometry& g) {
    int l = 0, t = 0, r = 0, b = 0;
    XPLMGetScreenBoundsGlobal(&l, &t, &r, &b);
    // Require the title-bar area to be reachable so the user can drag it back.
    const int overlapX = std::min(g.right, r) - std::max(g.left, l);
    return overlapX >= 60 && g.top <= t && g.top >= b + 60;
}

// ---- Callbacks: never let a C++ exception unwind into X-Plane --------------

void InstrumentWindow::onDraw(XPLMWindowID, void* refcon) {
    auto* self = static_cast<InstrumentWindow*>(refcon);
    try {
        self->draw();
    } catch (const std::exception& e) {
        if (!self->drawErrorLogged_) NAVS_ERROR("draw failed: %s", e.what());
        self->drawErrorLogged_ = true;
    } catch (...) {
        if (!self->drawErrorLogged_) NAVS_ERROR("draw failed: unknown exception");
        self->drawErrorLogged_ = true;
    }
}

render::InstrumentRenderer::Viewport InstrumentWindow::viewport() const {
    int l = 0, t = 0, r = 0, b = 0;
    XPLMGetWindowGeometry(id_, &l, &t, &r, &b);
    return {static_cast<float>(l), static_cast<float>(b), static_cast<float>(r - l), static_cast<float>(t - b)};
}

void InstrumentWindow::draw() {
    const render::InstrumentRenderer::Viewport vp = viewport();
    if (!XPLMWindowIsPoppedOut(id_)) {
        lastInSimGeometry_ = {static_cast<int>(vp.x), static_cast<int>(vp.y + vp.height),
                              static_cast<int>(vp.x + vp.width), static_cast<int>(vp.y)};
    }
    if (vp.width <= 0.0f || vp.height <= 0.0f) return;

    // Display smoothing runs on wall-clock time (keeps easing while paused).
    using Clock = std::chrono::steady_clock;
    const Clock::time_point now = Clock::now();
    const double dt = haveDrawn_ ? std::chrono::duration<double>(now - lastDraw_).count() : 1.0;
    lastDraw_ = now;
    haveDrawn_ = true;
    const InstrumentState shown = smoother_.apply(provider_(), dt, fx_);
    view_.fx = &fx_;

    // ~1 boxel feathered edges (anti-aliasing). The renderer picks
    // standard/compact layout from the size and letterboxes.
    canvas_.setFeather(1.0f);
    const Clock::time_point t0 = Clock::now();
    renderer_.draw(canvas_, shown, vp, view_);
    const Clock::time_point t1 = Clock::now();
    canvas_.flush();
    const Clock::time_point t2 = Clock::now();
    stats_.triangles = canvas_.lastTriangles();
    stats_.buildMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.flushMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    stats_.fastPath = canvas_.lastUsedFastPath();
}

void InstrumentWindow::toggleRunwayPicker() {
    if (!id_) return;
    if (view_.picker.open) closeRunwayPicker();
    else openRunwayPicker();
}

void InstrumentWindow::openRunwayPicker() {
    const InstrumentState& s = provider_();
    view_.picker.openFor(s.courseValid, s.courseDegMag, s.runwayManual ? s.runway : std::string());
    setVisible(true);
    XPLMTakeKeyboardFocus(id_);  // typed entry: 0-9 L C R, Enter, Esc
    NAVS_DEBUG("runway picker opened");
}

void InstrumentWindow::closeRunwayPicker() {
    view_.picker.close();
    if (id_ && XPLMHasKeyboardFocus(id_)) XPLMTakeKeyboardFocus(nullptr);  // give keys back to X-Plane
}

void InstrumentWindow::handlePickerResult(const render::RunwayPicker::Result& r) {
    using Action = render::RunwayPicker::Action;
    if (r.action == Action::None) return;
    closeRunwayPicker();
    if ((r.action == Action::Apply || r.action == Action::Clear) && runwayListener_) runwayListener_(r);
}

void InstrumentWindow::key(char ch, int vkey) {
    if (!view_.picker.open) return;
    handlePickerResult(view_.picker.key(ch, vkey));
}

void InstrumentWindow::setShowPanels(bool show) {
    if (view_.showPanels == show) return;
    view_.showPanels = show;
    NAVS_DEBUG("guidance panels %s", show ? "shown" : "collapsed");
    if (panelsListener_) panelsListener_(show);
}

// Mouse coordinates arrive in the same boxel space the window draws in, so the
// renderer's hit test matches what is on screen (also when popped out).
void InstrumentWindow::click(int x, int y) {
    using Hit = render::Hit;
    const Hit hit = render::InstrumentRenderer::hitTest(viewport(), static_cast<float>(x), static_cast<float>(y), view_);
    if (view_.picker.open) {
        handlePickerResult(view_.picker.press(hit));
        if (view_.picker.open) XPLMTakeKeyboardFocus(id_);  // keep typing after a click
        return;
    }
    if (hit == Hit::OpenRunwayPicker) openRunwayPicker();
    else if (hit == Hit::TogglePanels) setShowPanels(!view_.showPanels);
}

int InstrumentWindow::onMouseClick(XPLMWindowID, int x, int y, XPLMMouseStatus status, void* refcon) {
    if (status == xplm_MouseDown) {
        try {
            static_cast<InstrumentWindow*>(refcon)->click(x, y);
        } catch (...) {
            NAVS_ERROR("click handler failed");
        }
    }
    return 1;  // consume clicks so they don't fall through to the cockpit
}

int InstrumentWindow::onRightClick(XPLMWindowID, int, int, XPLMMouseStatus, void*) {
    return 1;  // consumed, no action
}

void InstrumentWindow::onKey(XPLMWindowID, char ch, XPLMKeyFlags flags, char vkey, void* refcon, int losingFocus) {
    auto* self = static_cast<InstrumentWindow*>(refcon);
    try {
        if (losingFocus) {  // focus taken elsewhere: just stop typing
            self->view_.picker.typed.clear();
            return;
        }
        if (flags & xplm_DownFlag) self->key(ch, static_cast<unsigned char>(vkey));
    } catch (...) {
        NAVS_ERROR("key handler failed");
    }
}

XPLMCursorStatus InstrumentWindow::onCursor(XPLMWindowID, int, int, void*) { return xplm_CursorDefault; }

int InstrumentWindow::onWheel(XPLMWindowID, int, int, int, int, void*) {
    return 1;  // consume so scrolling over the instrument doesn't zoom the view
}

}  // namespace navsight
