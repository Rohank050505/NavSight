#include "core/Log.h"

#include <cstdarg>
#include <cstdio>
#include <utility>

namespace navsight::log {
namespace {

Sink g_sink = [](const char* line) { std::fputs(line, stderr); };
bool g_debug = false;

const char* levelTag(Level level) {
    switch (level) {
        case Level::Debug: return "DEBUG";
        case Level::Info: return "INFO";
        case Level::Warn: return "WARN";
        case Level::Error: return "ERROR";
    }
    return "?";
}

}  // namespace

void setSink(Sink sink) { g_sink = std::move(sink); }
void setDebugEnabled(bool enabled) { g_debug = enabled; }
bool debugEnabled() { return g_debug; }

void write(Level level, const char* fmt, ...) {
    if (level == Level::Debug && !g_debug) return;

    char body[1024];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);

    char line[1100];
    std::snprintf(line, sizeof(line), "NavSight [%s] %s\n", levelTag(level), body);
    if (g_sink) g_sink(line);
}

}  // namespace navsight::log
