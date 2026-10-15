#pragma once

#include <functional>

// Logging front-end. Messages go to a pluggable sink: inside X-Plane the sink
// is XPLMDebugString (=> X-Plane 12/Log.txt); in unit tests it is stderr.
// Every line is prefixed with "NavSight" so it is easy to grep Log.txt.
namespace navsight::log {

enum class Level { Debug, Info, Warn, Error };

using Sink = std::function<void(const char* line)>;

void setSink(Sink sink);
void setDebugEnabled(bool enabled);
bool debugEnabled();

void write(Level level, const char* fmt, ...);

}  // namespace navsight::log

#define NAVS_DEBUG(...) ::navsight::log::write(::navsight::log::Level::Debug, __VA_ARGS__)
#define NAVS_INFO(...) ::navsight::log::write(::navsight::log::Level::Info, __VA_ARGS__)
#define NAVS_WARN(...) ::navsight::log::write(::navsight::log::Level::Warn, __VA_ARGS__)
#define NAVS_ERROR(...) ::navsight::log::write(::navsight::log::Level::Error, __VA_ARGS__)
