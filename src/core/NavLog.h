#pragma once

#include <string>

// Navigation diagnostics log: a dedicated file inside the plugin folder
// (Resources/plugins/NavSight/NavSight_nav.log) for the high-volume nav
// decisions - source/approach resolution, validity transitions,
// discontinuities, periodic nav samples - so they don't flood X-Plane's
// Log.txt. Lifecycle messages and errors stay in Log.txt (core/Log.h).
//
// The file is recreated each session; the previous session is kept as
// NavSight_nav.prev.log. Every line is timestamped (local wall clock) and
// flushed immediately so it survives a sim crash. If the file can't be opened,
// lines fall back to Log.txt so nothing is lost.
namespace navsight::navlog {

bool open(const std::string& path, const char* header);
void close();
bool isOpen();
const std::string& path();

void write(const char* fmt, ...);

}  // namespace navsight::navlog

#define NAVS_NAV(...) ::navsight::navlog::write(__VA_ARGS__)
