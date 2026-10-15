#include "data/XPlaneDataSource.h"

#include <algorithm>
#include <cmath>

#include "XPLMProcessing.h"
#include "core/Log.h"
#include "nav/Angles.h"

namespace navsight {

// Dataref names verified against X-Plane 12.4.4 Resources/plugins/DataRefs.txt.
//
// Heading: mag_psi is the aircraft's actual magnetic heading from the flight
// model (not an indicated instrument value), so NavSight is independent of the
// aircraft's own gyro/AHRS modelling.
//
// Navigation: the hsi_* datarefs follow X-Plane's pilot HSI source selector
// (NAV1/NAV2/GPS1/GPS2), so NavSight shows whatever the pilot has coupled to
// the HSI, in any aircraft that uses X-Plane's radios. Course and ident come
// from the selected source's own datarefs: for an ILS, navN_course_deg_mag is
// the localizer course regardless of how the OBS knob is set.
XPlaneDataSource::XPlaneDataSource()
    : magHeading_("sim/flightmodel/position/mag_psi"),
      trueHeading_("sim/flightmodel/position/psi"),
      magVariation_("sim/flightmodel/position/magnetic_variation"),
      latitude_("sim/flightmodel/position/latitude"),
      longitude_("sim/flightmodel/position/longitude"),
      groundspeed_("sim/flightmodel/position/groundspeed"),  // m/s
      groundTrack_("sim/cockpit2/gauges/indicators/ground_track_mag_pilot"),
      yawRate_("sim/flightmodel/position/R"),               // deg/s
      verticalSpeed_("sim/flightmodel/position/vh_ind_fpm"),
      fmsVpa_("sim/cockpit2/radios/indicators/fms_vpa_pilot"),
      paused_("sim/time/paused"),
      hsiSource_("sim/cockpit2/radios/actuators/HSI_source_select_pilot"),
      hsiHdef_("sim/cockpit2/radios/indicators/hsi_hdef_dots_pilot"),
      hsiVdef_("sim/cockpit2/radios/indicators/hsi_vdef_dots_pilot"),
      hsiHasHorizontal_("sim/cockpit2/radios/indicators/hsi_display_horizontal_pilot"),
      hsiHasVertical_("sim/cockpit2/radios/indicators/hsi_display_vertical_pilot"),
      hsiBearing_("sim/cockpit2/radios/indicators/hsi_bearing_deg_mag_pilot"),
      hsiHasDme_("sim/cockpit2/radios/indicators/hsi_has_dme_pilot"),
      hsiDmeNm_("sim/cockpit2/radios/indicators/hsi_dme_distance_nm_pilot"),
      hsiFromTo_("sim/cockpit2/radios/indicators/hsi_flag_from_to_pilot"),
      course_{FloatDataRef("sim/cockpit2/radios/actuators/nav1_course_deg_mag_pilot"),
              FloatDataRef("sim/cockpit2/radios/actuators/nav2_course_deg_mag_pilot"),
              FloatDataRef("sim/cockpit/radios/gps_course_degtm"),
              FloatDataRef("sim/cockpit/radios/gps2_course_degtm")},
      navId_{StringDataRef("sim/cockpit2/radios/indicators/nav1_nav_id"),
             StringDataRef("sim/cockpit2/radios/indicators/nav2_nav_id"),
             StringDataRef("sim/cockpit2/radios/indicators/gps_nav_id"),
             StringDataRef("sim/cockpit2/radios/indicators/gps2_nav_id")},
      navFrequency_{IntDataRef("sim/cockpit2/radios/actuators/nav1_frequency_hz"),
                    IntDataRef("sim/cockpit2/radios/actuators/nav2_frequency_hz")},
      gpsCdiMode_{IntDataRef("sim/cockpit/radios/gps_cdi_sensitivity"),
                  IntDataRef("sim/cockpit/radios/gps2_cdi_sensitivity")},
      gpsNmPerDot_{FloatDataRef("sim/cockpit/radios/gps_hdef_nm_per_dot"),
                   FloatDataRef("sim/cockpit/radios/gps2_hdef_nm_per_dot")},
      radioSignal_{IntDataRef("sim/cockpit2/radios/indicators/nav1_display_horizontal"),
                   IntDataRef("sim/cockpit2/radios/indicators/nav2_display_horizontal")},
      radioHdef_{FloatDataRef("sim/cockpit2/radios/indicators/nav1_hdef_dots_pilot"),
                 FloatDataRef("sim/cockpit2/radios/indicators/nav2_hdef_dots_pilot")},
      radioFromTo_{IntDataRef("sim/cockpit2/radios/indicators/nav1_flag_from_to_pilot"),
                   IntDataRef("sim/cockpit2/radios/indicators/nav2_flag_from_to_pilot")},
      hsiObs_("sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot"),
      // Loaded procedure type: the GPS mode alone can't tell an ILS flown as a
      // GPS overlay (X-Plane's G1000 shows LNAV) from an RNAV approach.
      finalApproachName_("sim/cockpit2/gauges/indicators/final_approach_name"),
      fasId_("sim/cockpit2/radios/indicators/fas_id"),
      ltpId_("sim/cockpit2/radios/indicators/ltp_id") {}

bool XPlaneDataSource::init() {
    const bool headingOk = magHeading_.bind();
    trueHeading_.bind();
    magVariation_.bind();
    latitude_.bind();
    longitude_.bind();
    groundspeed_.bind();
    groundTrack_.bind();
    yawRate_.bind();
    verticalSpeed_.bind();
    fmsVpa_.bind();
    paused_.bind();

    // Navigation is optional: without these the instrument stays in HDG mode.
    const bool sourceOk = hsiSource_.bind();
    const bool hdefOk = hsiHdef_.bind();
    const bool horizontalOk = hsiHasHorizontal_.bind();
    navAvailable_ = sourceOk && hdefOk && horizontalOk;
    hsiVdef_.bind();
    hsiHasVertical_.bind();
    hsiBearing_.bind();
    hsiHasDme_.bind();
    hsiDmeNm_.bind();
    hsiFromTo_.bind();
    for (auto& r : course_) r.bind();
    for (auto& r : navId_) r.bind();
    for (auto& r : navFrequency_) r.bind();
    for (auto& r : gpsCdiMode_) r.bind();
    for (auto& r : gpsNmPerDot_) r.bind();
    for (auto& r : radioSignal_) r.bind();
    for (auto& r : radioHdef_) r.bind();
    for (auto& r : radioFromTo_) r.bind();
    hsiObs_.bind();
    finalApproachName_.bind();
    fasId_.bind();
    ltpId_.bind();

    if (!headingOk) NAVS_ERROR("magnetic heading unavailable - instrument will show HDG flag");
    if (!navAvailable_) NAVS_ERROR("HSI datarefs unavailable - navigation guidance disabled");
    return headingOk;
}

void XPlaneDataSource::sample(SimSnapshot& out) {
    out.simTimeSec = XPLMGetElapsedTime();
    out.headingValid = magHeading_.valid();
    out.magHeadingDeg = static_cast<float>(nav::wrap360(magHeading_.get()));
    out.trueHeadingDeg = static_cast<float>(nav::wrap360(trueHeading_.get()));
    out.magVariationDeg = magVariation_.get();
    out.latitudeDeg = latitude_.get();
    out.longitudeDeg = longitude_.get();
    constexpr float kMpsToKt = 1.0f / 0.514444f;
    out.groundspeedValid = groundspeed_.valid();
    out.groundspeedKt = groundspeed_.get() * kMpsToKt;
    out.groundTrackValid = groundTrack_.valid();
    out.groundTrackDegMag = static_cast<float>(nav::wrap360(groundTrack_.get()));
    out.yawRateValid = yawRate_.valid();
    out.yawRateDegSec = yawRate_.get();
    out.verticalSpeedValid = verticalSpeed_.valid();
    out.verticalSpeedFpm = verticalSpeed_.get();
    out.paused = paused_.get() != 0;
    sampleNav(out);
}

void XPlaneDataSource::sampleNav(SimSnapshot& out) {
    NavSnapshot& n = out.nav;
    if (!navAvailable_) {
        n = NavSnapshot{};
        out.approach = ApproachInfo{};
        return;
    }

    const int src = hsiSource_.get();
    const int idx = (src >= 0 && src <= 3) ? src : 0;
    n.source = static_cast<NavSource>(idx);

    n.lateralValid = hsiHasHorizontal_.get() != 0;
    n.hdefDots = hsiHdef_.get();
    n.verticalValid = hsiHasVertical_.valid() && hsiHasVertical_.get() != 0;
    n.vdefDots = hsiVdef_.get();
    n.courseValid = course_[idx].valid();
    n.courseDegMag = course_[idx].get();
    n.bearingValid = hsiBearing_.valid() && n.lateralValid;
    n.bearingDegMag = hsiBearing_.get();
    n.dmeValid = hsiHasDme_.get() != 0;
    n.dmeNm = hsiDmeNm_.get();
    n.fromTo = hsiFromTo_.get();
    n.frequency = idx < 2 ? navFrequency_[idx].get() : 0;
    n.gpsCdiMode = (idx >= 2 && gpsCdiMode_[idx - 2].valid()) ? gpsCdiMode_[idx - 2].get() : -1;
    n.gpsNmPerDot = idx >= 2 ? std::max(0.0f, gpsNmPerDot_[idx - 2].get()) : 0.0f;
    n.fmsVpaDeg = idx >= 2 ? std::fabs(fmsVpa_.get()) : 0.0f;
    n.navId = navId_[idx].get();

    out.approach = resolver_.resolve(n, out.latitudeDeg, out.longitudeDeg, out.simTimeSec);
    sampleCrossCheck(out);
}

// Inputs for the source cross-check (nav/SourceCheck): what each NAV receiver
// is receiving, how the HSI course is set, and what approach GPS1 has loaded.
void XPlaneDataSource::sampleCrossCheck(SimSnapshot& out) {
    for (int i = 0; i < 2; ++i) {
        RadioSnapshot& r = out.radios[i];
        r.available = radioSignal_[i].valid() && navFrequency_[i].valid();
        if (!r.available) {
            r = RadioSnapshot{};
            continue;
        }
        r.frequency = navFrequency_[i].get();
        r.signal = radioSignal_[i].get() != 0;
        r.hdefDots = radioHdef_[i].get();
        r.courseValid = course_[i].valid();
        r.courseDegMag = course_[i].get();
        r.fromTo = radioFromTo_[i].get();
        r.ident = navId_[i].get();
        NavSnapshot probe;  // resolve this receiver's station like an HSI source
        probe.source = static_cast<NavSource>(i);
        probe.frequency = r.frequency;
        probe.lateralValid = r.signal;
        probe.courseValid = r.courseValid;
        probe.courseDegMag = r.courseDegMag;
        probe.navId = r.ident;
        r.station = radioResolver_[i].resolve(probe, out.latitudeDeg, out.longitudeDeg, out.simTimeSec);
    }

    out.hsiObsValid = hsiObs_.valid();
    out.hsiObsDegMag = hsiObs_.get();

    // GPS1 approach plan: re-read every 5 s or when GPS1's mode changes.
    const int mode = gpsCdiMode_[0].valid() ? gpsCdiMode_[0].get() : -1;
    if (mode != lastPlanMode_ || out.simTimeSec - lastPlanRead_ >= 5.0 || out.simTimeSec < lastPlanRead_) {
        plan_ = ApproachResolver::readGpsPlan(false, mode);
        lastPlanRead_ = out.simTimeSec;
        lastPlanMode_ = mode;
    }
    out.gpsPlan = plan_;
    out.gpsPlan.gpsCdiMode = mode;
    out.gpsPlan.procedureName = finalApproachName_.get();
    out.gpsPlan.fasId = fasId_.get();
    out.gpsPlan.ltpId = ltpId_.get();
}

}  // namespace navsight
