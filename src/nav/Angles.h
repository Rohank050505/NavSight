#pragma once

#include <cmath>

// Angle helpers shared by data, nav and render layers. Header-only, no
// X-Plane dependency. All angles in degrees unless the name says otherwise.
namespace navsight::nav {

constexpr double kPi = 3.14159265358979323846;

constexpr double degToRad(double deg) { return deg * kPi / 180.0; }
constexpr double radToDeg(double rad) { return rad * 180.0 / kPi; }

// Normalise to [0, 360).
inline double wrap360(double deg) {
    if (!std::isfinite(deg)) return 0.0;
    double r = std::fmod(deg, 360.0);
    if (r < 0.0) r += 360.0;
    return r >= 360.0 ? 0.0 : r;  // guard -tiny + 360 rounding up to 360
}

// Normalise to [-180, 180).
inline double wrap180(double deg) {
    return wrap360(deg + 180.0) - 180.0;
}

// Signed shortest rotation from `from` to `to`, in [-180, 180).
inline double shortestDelta(double from, double to) {
    return wrap180(to - from);
}

// Distance between two positions in nautical miles (equirectangular; accurate
// to well under 1% at the < 150 NM ranges used for nav stations).
inline double distanceNm(double lat1, double lon1, double lat2, double lon2) {
    const double dLat = (lat2 - lat1) * 60.0;
    const double dLon = wrap180(lon2 - lon1) * 60.0 * std::cos(degToRad((lat1 + lat2) * 0.5));
    return std::sqrt(dLat * dLat + dLon * dLon);
}

}  // namespace navsight::nav
