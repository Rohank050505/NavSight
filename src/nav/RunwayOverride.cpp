#include "nav/RunwayOverride.h"

#include <cctype>
#include <cmath>
#include <cstdio>

#include "nav/Angles.h"
#include "nav/ApproachNaming.h"

namespace navsight::nav {

SourceIdentity identityOf(const SimSnapshot& snap) {
    return {snap.nav.source, snap.nav.frequency, snap.approach.label};
}

std::string normaliseRunway(const std::string& text) {
    std::string t;
    for (char c : text)
        if (!std::isspace(static_cast<unsigned char>(c))) t += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (!isRunwayDesignator(t)) return {};
    // Two-digit number: "4L" -> "04L", "9" -> "09".
    if (t.size() == 1 || !std::isdigit(static_cast<unsigned char>(t[1]))) t = "0" + t;
    return t;
}

std::vector<std::string> runwayCandidates(double courseDegMag) {
    int n = static_cast<int>(std::lround(wrap360(courseDegMag) / 10.0));
    if (n == 0) n = 36;
    std::vector<std::string> out;
    for (int d : {-1, 0, 1}) {
        int v = n + d;
        if (v < 1) v += 36;
        if (v > 36) v -= 36;
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02d", v);
        out.emplace_back(buf);
    }
    return out;
}

bool stillApplies(const RunwayOverride& o, const SimSnapshot& snap) {
    return o.active && identityOf(snap) == o.boundTo;
}

bool runwayMismatchesCourse(const std::string& runway, double courseDegMag) {
    if (runway.size() < 2 || !std::isdigit(static_cast<unsigned char>(runway[0])) ||
        !std::isdigit(static_cast<unsigned char>(runway[1])))
        return false;
    const int n = (runway[0] - '0') * 10 + (runway[1] - '0');
    return std::fabs(shortestDelta(courseDegMag, n * 10.0)) > 30.0;
}

void applyOverride(ApproachInfo& a, const RunwayOverride& o) {
    if (!o.active || o.runway.empty()) return;
    a.runway = o.runway;
    a.runwayManual = true;
    const std::string rwy = "RWY " + o.runway;
    if (!a.isApproach()) {
        // VOR / GPS leg / unidentified: the pilot declares a runway approach.
        a.kind = ApproachKind::Manual;
        a.label = rwy;
        return;
    }
    const auto pos = a.label.find("RWY ");
    a.label = pos == std::string::npos ? (a.label.empty() ? rwy : a.label + " " + rwy) : a.label.substr(0, pos) + rwy;
}

}  // namespace navsight::nav
