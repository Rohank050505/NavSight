#include "nav/ApproachNaming.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <sstream>

#include "nav/Angles.h"

namespace navsight::nav {
namespace {

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

bool isIcaoLike(const std::string& t) {
    if (t.size() != 4) return false;
    for (char c : t)
        if (!std::isalnum(static_cast<unsigned char>(c))) return false;
    return std::isalpha(static_cast<unsigned char>(t[0])) != 0;
}

}  // namespace

bool isRunwayDesignator(const std::string& token) {
    const std::string t = upper(token);
    std::size_t digits = 0;
    while (digits < t.size() && std::isdigit(static_cast<unsigned char>(t[digits]))) ++digits;
    if (digits == 0 || digits > 2) return false;
    const int num = std::stoi(t.substr(0, digits));
    if (num < 1 || num > 36) return false;
    if (t.size() == digits) return true;
    return t.size() == digits + 1 && (t.back() == 'L' || t.back() == 'R' || t.back() == 'C');
}

ParsedNavaidName parseNavaidName(const std::string& name) {
    ParsedNavaidName out;
    const std::string u = upper(name);
    for (const char* tag : {"LDA", "SDF", "IGS", "LOC", "ILS"}) {
        if (u.find(tag) != std::string::npos) {
            out.typeTag = tag;
            break;
        }
    }
    std::istringstream ss(u);
    std::string tok;
    while (ss >> tok) {
        if (out.runway.empty() && isRunwayDesignator(tok)) {
            out.runway = tok.size() == 1 || (tok.size() == 2 && !std::isdigit(static_cast<unsigned char>(tok[1])))
                             ? "0" + tok
                             : tok;
        } else if (out.airport.empty() && out.runway.empty() && isIcaoLike(tok) && tok != out.typeTag) {
            out.airport = tok;
        }
    }
    return out;
}

std::string runwayFromGpsWaypoint(const std::string& ident) {
    const std::string u = upper(ident);
    if (u.size() < 3 || u.compare(0, 2, "RW") != 0) return {};
    const std::string rest = u.substr(2);
    return isRunwayDesignator(rest) ? rest : std::string();
}

bool isGpsApproachActive(int m) { return m == 5 || (m >= 6 && m <= 13); }
bool isGpsMissedApproach(int m) { return m == 4; }
bool gpsModeExpectsVertical(int m) { return m == 9 || m == 11 || m == 13; }

const char* gpsModeName(int m) {
    static constexpr const char* kNames[] = {"OCN", "ENR",   "TERM",   "DPRT", "MAPR", "APR",  "RNP AR",
                                             "LNAV", "LNAV+V", "L/VNAV", "LP",   "LPV",  "LP+V", "GLS"};
    return (m >= 0 && m < static_cast<int>(sizeof(kNames) / sizeof(kNames[0]))) ? kNames[m] : "";
}

std::string approachLabel(ApproachKind kind, const std::string& typeTag, const std::string& runway,
                          const std::string& stationId) {
    const std::string rwy = runway.empty() ? std::string() : " RWY " + runway;
    switch (kind) {
        case ApproachKind::Ils: return "ILS" + rwy;
        case ApproachKind::Localizer:
            return (typeTag.empty() || typeTag == "ILS" ? std::string("LOC") : typeTag) + rwy;
        case ApproachKind::Rnav: return "RNAV" + rwy;
        case ApproachKind::Vor: return stationId.empty() ? "VOR" : "VOR " + stationId;
        case ApproachKind::GpsEnroute: return stationId.empty() ? "GPS" : "GPS " + stationId;
        case ApproachKind::Manual: return runway.empty() ? std::string("RWY") : "RWY " + runway;
        case ApproachKind::None: break;
    }
    return stationId;
}

}  // namespace navsight::nav
