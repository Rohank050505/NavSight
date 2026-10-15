#include "core/NavLog.h"

#include <cstdarg>
#include <cstdio>
#include <ctime>

#include "core/Log.h"

namespace navsight::navlog {
namespace {

std::FILE* g_file = nullptr;
std::string g_path;

void timestamp(char* out, std::size_t size) {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    std::strftime(out, size, "%Y-%m-%d %H:%M:%S", &local);
}

}  // namespace

bool open(const std::string& path, const char* header) {
    close();
    // Keep the previous session for comparison.
    const std::string prev = path.substr(0, path.rfind('.')) + ".prev.log";
    std::remove(prev.c_str());
    std::rename(path.c_str(), prev.c_str());

    g_file = std::fopen(path.c_str(), "w");
    if (!g_file) {
        NAVS_WARN("could not create nav log %s - nav diagnostics go to Log.txt", path.c_str());
        return false;
    }
    g_path = path;
    char ts[32];
    timestamp(ts, sizeof(ts));
    std::fprintf(g_file, "# %s - session started %s\n", header, ts);
    std::fflush(g_file);
    return true;
}

void close() {
    if (g_file) {
        std::fclose(g_file);
        g_file = nullptr;
    }
}

bool isOpen() { return g_file != nullptr; }
const std::string& path() { return g_path; }

void write(const char* fmt, ...) {
    char body[1024];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);

    if (!g_file) {
        NAVS_INFO("%s", body);
        return;
    }
    char ts[32];
    timestamp(ts, sizeof(ts));
    std::fprintf(g_file, "%s  %s\n", ts, body);
    std::fflush(g_file);
}

}  // namespace navsight::navlog
