#include "core/Config.h"

#include <fstream>
#include <sstream>

#include "core/Log.h"

namespace navsight {
namespace {

bool parseBool(const std::string& v, bool& out) {
    if (v == "1" || v == "true") { out = true; return true; }
    if (v == "0" || v == "false") { out = false; return true; }
    return false;
}

bool parseBrightness(const std::string& v, int& out) {
    if (v == "day" || v == "0") { out = 0; return true; }
    if (v == "dim" || v == "1") { out = 1; return true; }
    if (v == "night" || v == "2") { out = 2; return true; }
    return false;
}

template <typename T>
bool parseNumber(const std::string& v, T& out) {
    std::istringstream ss(v);
    T tmp{};
    if (!(ss >> tmp)) return false;
    out = tmp;
    return true;
}

}  // namespace

bool Config::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) return false;

    layoutVersion = 1;  // files written before the key existed
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (const auto hash = line.find('#'); hash != std::string::npos) line.erase(hash);
        std::istringstream ss(line);
        std::string key, value;
        if (!(ss >> key)) continue;
        ss >> value;

        bool ok = true;
        if (key == "layout_version") ok = parseNumber(value, layoutVersion);
        else if (key == "window_visible") ok = parseBool(value, windowVisible);
        else if (key == "show_panels") ok = parseBool(value, showPanels);
        else if (key == "brightness") ok = parseBrightness(value, brightness);
        else if (key == "fast_rendering") ok = parseBool(value, fastRendering);
        else if (key == "window_left") ok = parseNumber(value, window.left);
        else if (key == "window_top") ok = parseNumber(value, window.top);
        else if (key == "window_right") ok = parseNumber(value, window.right);
        else if (key == "window_bottom") ok = parseNumber(value, window.bottom);
        else if (key == "heading_smoothing_sec") ok = parseNumber(value, headingSmoothingSec);
        else if (key == "lateral_closure_sec") ok = parseNumber(value, lateralClosureSec);
        else if (key == "vertical_closure_sec") ok = parseNumber(value, verticalClosureSec);
        else if (key == "invert_lateral") ok = parseBool(value, invertLateral);
        else if (key == "invert_vertical") ok = parseBool(value, invertVertical);
        else if (key == "debug_logging") ok = parseBool(value, debugLogging);
        else NAVS_DEBUG("config line %d: unknown key '%s' ignored", lineNo, key.c_str());

        if (!ok) NAVS_WARN("config line %d: bad value '%s' for '%s', keeping default",
                           lineNo, value.c_str(), key.c_str());
    }

    if (headingSmoothingSec < 0.0f || headingSmoothingSec > 2.0f) {
        NAVS_WARN("heading_smoothing_sec %.3f out of range [0,2], using 0.05", headingSmoothingSec);
        headingSmoothingSec = 0.05f;
    }
    if (lateralClosureSec < 3.0f || lateralClosureSec > 120.0f) {
        NAVS_WARN("lateral_closure_sec %.1f out of range [3,120], using 20", lateralClosureSec);
        lateralClosureSec = 20.0f;
    }
    if (verticalClosureSec < 3.0f || verticalClosureSec > 120.0f) {
        NAVS_WARN("vertical_closure_sec %.1f out of range [3,120], using 10", verticalClosureSec);
        verticalClosureSec = 10.0f;
    }
    return true;
}

bool Config::save(const std::string& path) const {
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;
    out << "# NavSight settings - written by the plugin, safe to edit while X-Plane is closed\n"
        << "layout_version " << kCurrentLayoutVersion << "\n"
        << "window_visible " << (windowVisible ? 1 : 0) << "\n"
        << "show_panels " << (showPanels ? 1 : 0) << "\n"
        << "brightness " << (brightness == 2 ? "night" : brightness == 1 ? "dim" : "day") << "\n"
        << "fast_rendering " << (fastRendering ? 1 : 0) << "\n"
        << "window_left " << window.left << "\n"
        << "window_top " << window.top << "\n"
        << "window_right " << window.right << "\n"
        << "window_bottom " << window.bottom << "\n"
        << "heading_smoothing_sec " << headingSmoothingSec << "\n"
        << "lateral_closure_sec " << lateralClosureSec << "\n"
        << "vertical_closure_sec " << verticalClosureSec << "\n"
        << "invert_lateral " << (invertLateral ? 1 : 0) << "\n"
        << "invert_vertical " << (invertVertical ? 1 : 0) << "\n"
        << "debug_logging " << (debugLogging ? 1 : 0) << "\n";
    return static_cast<bool>(out);
}

}  // namespace navsight
