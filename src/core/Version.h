#pragma once

// Single source of truth for plugin identity. The signature must stay stable
// across releases: X-Plane and other plugins (and the future companion app)
// use it to find this plugin.
namespace navsight {

constexpr const char* kPluginName = "NavSight";
constexpr const char* kPluginSignature = "navsight.instrument";
constexpr const char* kPluginDescription =
    "NavSight predictive navigation instrument: HSI overlay with runway alignment and guidance.";
constexpr const char* kVersionString = "1.0.0";  // first public release

}  // namespace navsight
