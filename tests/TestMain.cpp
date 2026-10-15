// Host-side unit tests: everything that doesn't need X-Plane running.
// Covers the nav math, config persistence, and the renderer (drawn into a
// recording canvas so layout can be checked without OpenGL).
//
// Built and run by:  .\build.ps1 -Test

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "core/Config.h"
#include "core/Log.h"
#include "core/NavLog.h"
#include "nav/Angles.h"
#include "nav/ApproachNaming.h"
#include "nav/AxisGuidance.h"
#include "nav/InstrumentModel.h"
#include "nav/RunwayOverride.h"
#include "render/DisplaySmoother.h"
#include "render/Palette.h"
#include "render/RunwayPicker.h"
#include "render/GuidanceText.h"
#include "render/InstrumentRenderer.h"
#include "render/StrokeFont.h"

using namespace navsight;

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++g_checks;                                                              \
        if (!(cond)) {                                                           \
            ++g_failures;                                                        \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) <= (eps))

class RecordingCanvas : public render::Canvas {
public:
    void flush() override {
        flushedVertices += batch_.size();
        for (const Vertex& v : batch_) {
            if (!std::isfinite(v.pos.x) || !std::isfinite(v.pos.y)) ++nonFinite;
            minX = std::fmin(minX, v.pos.x); maxX = std::fmax(maxX, v.pos.x);
            minY = std::fmin(minY, v.pos.y); maxY = std::fmax(maxY, v.pos.y);
        }
        batch_.clear();
    }
    std::size_t flushedVertices = 0;
    int nonFinite = 0;
    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
};

void testAngles() {
    std::puts("angles");
    CHECK_NEAR(nav::wrap360(370.0), 10.0, 1e-9);
    CHECK_NEAR(nav::wrap360(-10.0), 350.0, 1e-9);
    CHECK_NEAR(nav::wrap360(360.0), 0.0, 1e-9);
    CHECK_NEAR(nav::wrap360(-1e-15), 0.0, 1e-9);
    CHECK_NEAR(nav::wrap360(NAN), 0.0, 1e-9);
    CHECK_NEAR(nav::wrap180(190.0), -170.0, 1e-9);
    CHECK_NEAR(nav::shortestDelta(350.0, 10.0), 20.0, 1e-9);
    CHECK_NEAR(nav::shortestDelta(10.0, 350.0), -20.0, 1e-9);
}

void testModel() {
    std::puts("instrument model");
    InstrumentModel m({0.0});
    SimSnapshot s;
    s.headingValid = true;

    s.magHeadingDeg = 218.4f; s.simTimeSec = 1.0;
    m.update(s);
    CHECK(m.state().headingValid);
    CHECK_NEAR(m.state().displayHeadingDeg, 218.4, 1e-4);
    CHECK(m.state().headingReadoutDeg == 218);

    s.magHeadingDeg = 0.2f; m.update(s);
    CHECK(m.state().headingReadoutDeg == 360);  // aviation convention
    s.magHeadingDeg = 359.6f; m.update(s);
    CHECK(m.state().headingReadoutDeg == 360);
    s.magHeadingDeg = 0.6f; m.update(s);
    CHECK(m.state().headingReadoutDeg == 1);

    // Smoothing goes the short way across north.
    InstrumentModel sm({0.1});
    s.magHeadingDeg = 355.0f; s.simTimeSec = 0.0; sm.update(s);
    s.magHeadingDeg = 5.0f; s.simTimeSec = 0.05; sm.update(s);
    const double d = sm.state().displayHeadingDeg;
    CHECK(d > 355.0 || d < 5.0);
    CHECK(std::fabs(nav::shortestDelta(355.0, d)) < 10.0);
    // Converges.
    for (int i = 0; i < 100; ++i) { s.simTimeSec += 0.02; sm.update(s); }
    CHECK_NEAR(sm.state().displayHeadingDeg, 5.0, 0.01);

    // Long gap snaps instead of gliding.
    s.magHeadingDeg = 90.0f; s.simTimeSec += 5.0; sm.update(s);
    CHECK_NEAR(sm.state().displayHeadingDeg, 90.0, 1e-4);

    // Invalid data propagates.
    s.headingValid = false; sm.update(s);
    CHECK(!sm.state().headingValid);
}

void testConfig() {
    std::puts("config");
    const auto path = (std::filesystem::temp_directory_path() / "navsight_test.prf").string();
    Config a;
    a.windowVisible = false;
    a.window = {100, 900, 480, 520};
    a.headingSmoothingSec = 0.2f;
    a.debugLogging = true;
    CHECK(a.save(path));

    Config b;
    CHECK(b.load(path));
    CHECK(!b.windowVisible);
    CHECK(b.window.left == 100 && b.window.top == 900 && b.window.right == 480 && b.window.bottom == 520);
    CHECK_NEAR(b.headingSmoothingSec, 0.2f, 1e-6f);
    CHECK(b.debugLogging);

    // Garbage and unknown keys keep defaults instead of failing.
    {
        FILE* f = std::fopen(path.c_str(), "w");
        std::fputs("# comment\nwindow_left banana\nfuture_key 42\nheading_smoothing_sec 99\n", f);
        std::fclose(f);
    }
    Config c;
    CHECK(c.load(path));
    CHECK(c.window.left == 0);
    CHECK_NEAR(c.headingSmoothingSec, 0.05f, 1e-6f);

    // Files from before layout_version existed are flagged for migration.
    CHECK(c.layoutVersion == 1);
    CHECK(b.layoutVersion == Config::kCurrentLayoutVersion);

    Config missing;
    CHECK(!missing.load(path + ".does-not-exist"));
    std::filesystem::remove(path);
}

void testFont() {
    std::puts("stroke font");
    CHECK(render::measureText("", 12.0f) == 0.0f);
    CHECK(render::measureText("N", 12.0f) > 0.0f);
    CHECK_NEAR(render::measureText("33", 24.0f), 2.0f * render::measureText("33", 12.0f), 1e-4f);
    RecordingCanvas c;
    render::TextStyle st;
    render::drawText(c, "NAVSIGHT 0123456789*", {0, 0}, st);
    CHECK(c.pendingVertexCount() > 0);
    CHECK(c.pendingVertexCount() % 3 == 0);
}

// Feed an axis a position history sampled at 20 Hz.
template <typename PosFn>
nav::AxisOutput runAxis(nav::AxisGuidance& g, double seconds, PosFn pos, double t0 = 0.0) {
    nav::AxisOutput out;
    for (double t = t0; t <= t0 + seconds + 1e-9; t += 0.05) out = g.update(true, pos(t), t);
    return out;
}

void testAxisGuidance() {
    std::puts("axis guidance");
    using nav::Command;
    using nav::Trend;
    nav::AxisGuidance::Params p;  // closure 20 s

    {   // Static offset right of course: not converging -> correct LEFT.
        nav::AxisGuidance g(p);
        const auto o = runAxis(g, 5.0, [](double) { return 1.0; });
        CHECK(o.valid);
        CHECK(o.trend == Trend::Steady);
        CHECK(o.command == Command::Correct);
        CHECK(o.direction == -1);
        CHECK(std::fabs(o.rateDotsPerSec) < 1e-6);
    }
    {   // Closing at roughly the ideal rate (pos/20 per second) -> HOLD.
        nav::AxisGuidance g(p);
        const auto o = runAxis(g, 6.0, [](double t) { return 1.0 * std::exp(-t / 20.0); });
        CHECK(o.trend == Trend::Converging);
        CHECK(o.command == Command::Hold);
        CHECK(o.interceptValid);
        CHECK(o.interceptSec > 10.0 && o.interceptSec < 30.0);
    }
    {   // Closing much faster than ideal from the right -> EASE (turn right).
        nav::AxisGuidance g(p);
        const auto o = runAxis(g, 4.0, [](double t) { return 2.0 - 0.3 * t; });
        CHECK(o.trend == Trend::Converging);
        CHECK(o.command == Command::Ease);
        CHECK(o.direction == +1);
        CHECK(o.predictedDots < o.positionDots);
    }
    {   // Drifting away below the path -> DIVERGING, correct UP, firm.
        nav::AxisGuidance g(p);
        const auto o = runAxis(g, 4.0, [](double t) { return -0.5 - 0.1 * t; });
        CHECK(o.trend == Trend::Diverging);
        CHECK(o.command == Command::Correct);
        CHECK(o.direction == +1);
        CHECK(o.strength == nav::Strength::Firm);
        CHECK(!o.interceptValid);
    }
    {   // Centred and stable -> ON TARGET.
        nav::AxisGuidance g(p);
        const auto o = runAxis(g, 3.0, [](double) { return 0.03; });
        CHECK(o.command == Command::OnTarget);
    }
    {   // Invalid clears; long gap re-acquires without a rate spike.
        nav::AxisGuidance g(p);
        runAxis(g, 2.0, [](double) { return 1.0; });
        CHECK(!g.update(false, 0.0, 2.1).valid);
        const auto o = g.update(true, -1.0, 2.2);
        CHECK(o.valid && o.rateDotsPerSec == 0.0);
        const auto o2 = g.update(true, 1.0, 10.0);  // 7.8 s gap
        CHECK(o2.rateDotsPerSec == 0.0);
    }
}

void testApproachNaming() {
    std::puts("approach naming");
    auto a = nav::parseNavaidName("KSEA 16L ILS-cat-III");
    CHECK(a.airport == "KSEA" && a.runway == "16L" && a.typeTag == "ILS");
    auto b = nav::parseNavaidName("ILS-cat-I");
    CHECK(b.airport.empty() && b.runway.empty() && b.typeTag == "ILS");
    auto c = nav::parseNavaidName("KJAC 19 LDA/DME");
    CHECK(c.runway == "19" && c.typeTag == "LDA");
    auto d = nav::parseNavaidName("EGLL 9L ILS");
    CHECK(d.runway == "09L");
    CHECK(nav::isRunwayDesignator("22") && nav::isRunwayDesignator("04R") && !nav::isRunwayDesignator("37"));
    CHECK(!nav::isRunwayDesignator("0") && !nav::isRunwayDesignator("22X") && !nav::isRunwayDesignator("ILS"));
    CHECK(nav::runwayFromGpsWaypoint("RW22") == "22");
    CHECK(nav::runwayFromGpsWaypoint("RW04L") == "04L");
    CHECK(nav::runwayFromGpsWaypoint("KAYLA").empty());
    CHECK(nav::approachLabel(ApproachKind::Ils, "ILS", "22", "IABC") == "ILS RWY 22");
    CHECK(nav::approachLabel(ApproachKind::Localizer, "LDA", "19", "") == "LDA RWY 19");
    CHECK(nav::approachLabel(ApproachKind::Localizer, "ILS", "", "") == "LOC");
    CHECK(nav::approachLabel(ApproachKind::Vor, "", "", "SEA") == "VOR SEA");
    CHECK(nav::approachLabel(ApproachKind::Rnav, "", "16L", "RW16L") == "RNAV RWY 16L");

    // GPS approach state from gps_cdi_sensitivity.
    for (int m : {5, 6, 7, 8, 9, 10, 11, 12, 13}) CHECK(nav::isGpsApproachActive(m));
    for (int m : {-1, 0, 1, 2, 3, 4, 14}) CHECK(!nav::isGpsApproachActive(m));
    CHECK(nav::isGpsMissedApproach(4) && !nav::isGpsMissedApproach(11));
    CHECK(nav::gpsModeExpectsVertical(11) && nav::gpsModeExpectsVertical(9) && nav::gpsModeExpectsVertical(13));
    CHECK(!nav::gpsModeExpectsVertical(7) && !nav::gpsModeExpectsVertical(8) && !nav::gpsModeExpectsVertical(12));
    CHECK(std::string(nav::gpsModeName(11)) == "LPV" && std::string(nav::gpsModeName(2)) == "TERM");
    CHECK(std::string(nav::gpsModeName(-1)).empty() && std::string(nav::gpsModeName(99)).empty());
}

SimSnapshot ilsSnapshot(float hdef, float vdef, double t);  // defined below

void testNavLog() {
    std::puts("nav log file");
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "navsight_navlog_test";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const std::string path = (dir / "NavSight_nav.log").string();

    auto readAll = [](const fs::path& p) {
        std::string s;
        if (FILE* f = std::fopen(p.string().c_str(), "r")) {
            char buf[512];
            while (std::fgets(buf, sizeof(buf), f)) s += buf;
            std::fclose(f);
        }
        return s;
    };

    CHECK(navlog::open(path, "session one"));
    NAVS_NAV("t=%.1f nav validity: lateral=%s", 12.5, "VALID");
    navlog::close();
    const std::string first = readAll(path);
    CHECK(first.find("# session one") != std::string::npos);
    CHECK(first.find("t=12.5 nav validity: lateral=VALID") != std::string::npos);
    // Each entry line starts with a "YYYY-MM-DD HH:MM:SS" timestamp.
    const auto entry = first.find("t=12.5");
    const auto lineStart = first.rfind('\n', entry) + 1;
    CHECK(entry != std::string::npos && entry - lineStart == 21);
    CHECK(std::isdigit(static_cast<unsigned char>(first[lineStart])) && first[lineStart + 4] == '-');

    CHECK(navlog::open(path, "session two"));  // previous session rotated
    navlog::close();
    CHECK(readAll(dir / "NavSight_nav.prev.log").find("session one") != std::string::npos);
    CHECK(readAll(path).find("session two") != std::string::npos);
    CHECK(readAll(path).find("nav validity") == std::string::npos);
    fs::remove_all(dir);
}

// ---------------------------------------------------------------------------
// Kinematic simulator for the trajectory predictor: a point mass flying at a
// given groundspeed / track / turn rate relative to a course. NavSight's inputs
// (hdef, track, groundspeed, distance) are synthesised from the true position,
// so predictions can be checked against the analytic answer.
struct FlightSim {
    double t = 0.0;
    double xNm = 0.0;          // true cross-track, + = right of course
    double courseDeg = 205.0;
    double trackDeg = 205.0;
    double gsKt = 120.0;
    double turnRateDegSec = 0.0;
    bool angular = false;      // false: GPS (nm per dot); true: localizer (deg per dot)
    double nmPerDot = 1.0;
    double degPerDot = 0.8;
    double distNm = 8.0;       // to the localizer antenna (angular only)
    bool dmeAvailable = true;

    void step(double dt) {
        trackDeg += turnRateDegSec * dt;
        const double tke = nav::degToRad(trackDeg - courseDeg);
        xNm += gsKt / 3600.0 * std::sin(tke) * dt;
        distNm -= gsKt / 3600.0 * std::cos(tke) * dt;
        t += dt;
    }
    double dots() const {
        return angular ? nav::radToDeg(std::atan(xNm / distNm)) / degPerDot : xNm / nmPerDot;
    }
    SimSnapshot snapshot() const {
        SimSnapshot s;
        s.simTimeSec = t;
        s.headingValid = true;
        s.magHeadingDeg = static_cast<float>(nav::wrap360(trackDeg));
        s.groundTrackValid = true;
        s.groundTrackDegMag = static_cast<float>(nav::wrap360(trackDeg));
        s.groundspeedValid = true;
        s.groundspeedKt = static_cast<float>(gsKt);
        s.nav.lateralValid = true;
        s.nav.hdefDots = static_cast<float>(std::clamp(-dots(), -2.5, 2.5));  // hdef > 0 = aircraft left
        s.nav.courseValid = true;
        s.nav.courseDegMag = static_cast<float>(courseDeg);
        s.nav.fromTo = 1;
        if (angular) {
            s.nav.source = NavSource::Nav1;
            s.nav.frequency = 11190;
            s.nav.dmeValid = dmeAvailable;
            s.nav.dmeNm = static_cast<float>(distNm);
            s.approach.kind = ApproachKind::Ils;
            s.approach.label = "ILS RWY 20";
        } else {
            s.nav.source = NavSource::Gps1;
            s.nav.gpsNmPerDot = static_cast<float>(nmPerDot);
            s.nav.gpsCdiMode = 11;
            s.approach.kind = ApproachKind::Rnav;
            s.approach.label = "RNAV RWY 20";
        }
        s.approach.runway = "20";
        return s;
    }
};

void fly(InstrumentModel& m, FlightSim& sim, double seconds) {
    for (double end = sim.t + seconds; sim.t < end - 1e-9;) {
        sim.step(0.05);
        m.update(sim.snapshot());
    }
}

void testTrajectoryPredictor() {
    std::puts("trajectory predictor");
    using nav::Command;
    using nav::Confidence;
    const double radius120 = nav::TrajectoryPredictor::standardRateRadiusNm(120.0, 3.0);
    CHECK_NEAR(radius120, 0.6366, 0.001);  // 120 kt at 3 deg/s

    {   // A. Straight 30 deg intercept from ~1 nm (GPS, 1 nm/dot): converging,
        //    10 s prediction matches geometry, roll-out lead ahead, track error ~ desired -> HOLD.
        FlightSim sim;
        sim.xNm = 1.0;
        sim.trackDeg = 175.0;  // 30 deg left of course 205, toward it
        InstrumentModel m;
        fly(m, sim, 3.0);
        const auto& L = m.state().trajectory.lateral;
        const double closure = 120.0 / 3600.0 * std::sin(nav::degToRad(30.0));
        CHECK(L.valid && L.confidence == Confidence::Measured);
        CHECK(L.outcome == nav::LateralOutcome::Converging);
        CHECK_NEAR(L.trackErrorDeg, -30.0, 0.01);
        CHECK_NEAR(L.predicted10Dots, sim.xNm - 10.0 * closure, 0.02);
        CHECK(!L.crossValid);  // ~57 s away, beyond the 20 s horizon
        const double lead = radius120 * (1.0 - std::cos(nav::degToRad(30.0)));
        CHECK_NEAR(L.leadNm, lead, 0.002);
        CHECK(L.turnInValid && std::fabs(L.turnInSec - (sim.xNm - lead) / closure) < 1.0);
        CHECK(!L.turnNow);
        CHECK(L.commandValid && L.command == Command::Hold);
        CHECK(m.state().lateral.out.command == Command::Hold);  // track-based law drives the panel
    }
    {   // B. Close-in 90 deg intercept: inside the standard-rate roll-out
        //    distance -> TURN NOW (right) with predicted overshoot, before the
        //    needle reaches centre.
        FlightSim sim;
        sim.xNm = 0.3;
        sim.trackDeg = 115.0;  // 90 deg left of course 205
        InstrumentModel m;
        fly(m, sim, 3.0);
        const auto& L = m.state().trajectory.lateral;
        CHECK(sim.xNm > 0.15);                 // still well right of course
        CHECK(L.turnNow);
        CHECK_NEAR(L.rolloutOvershootDots, (radius120 - sim.xNm) / 1.0, 0.03);
        CHECK(L.command == Command::Ease && L.direction == +1);  // turn right onto the course heading
        CHECK(L.crossValid && std::fabs(L.crossSec - sim.xNm / (120.0 / 3600.0)) < 0.5);
        const auto txt = render::describeAxis(m.state().lateral, render::Axis::Lateral, &m.state().trajectory);
        CHECK(txt.action == "TURN RIGHT" && txt.qualifier == "NOW");
        CHECK(txt.prediction.rfind("OVERSHOOT", 0) == 0);
    }
    {   // C. Same position, different track: the command follows track vs
        //    course, not the deviation rate. 0.5 nm right at a 15 deg intercept
        //    is the desired intercept -> HOLD; parallel (0 deg) -> TURN LEFT.
        FlightSim sim;
        sim.xNm = 0.5;
        sim.trackDeg = 190.0;
        InstrumentModel m;
        fly(m, sim, 3.0);
        const auto& L = m.state().trajectory.lateral;
        CHECK_NEAR(L.desiredTrackErrorDeg, -30.0 * sim.xNm, 0.2);
        CHECK(L.command == Command::Hold);

        FlightSim par;
        par.xNm = 0.5;
        par.trackDeg = 205.0;
        InstrumentModel mp;
        fly(mp, par, 3.0);
        const auto& P = mp.state().trajectory.lateral;
        CHECK(P.outcome == nav::LateralOutcome::Parallel);
        CHECK(P.command == Command::Correct && P.direction == -1 && P.strength == nav::Strength::Moderate);

        // A normal 30 deg intercept 0.55 nm out is acceptable (HOLD), even
        // though the asymptotic law alone would prefer ~16 deg there.
        FlightSim std30;
        std30.xNm = 0.6;
        std30.trackDeg = 175.0;
        InstrumentModel ms;
        fly(ms, std30, 3.0);
        CHECK(ms.state().trajectory.lateral.command == Command::Hold);
        // Too shallow (5 deg) at the same distance -> steepen (turn left, toward).
        FlightSim shallow;
        shallow.xNm = 0.6;
        shallow.trackDeg = 200.0;
        InstrumentModel mh;
        fly(mh, shallow, 3.0);
        CHECK(mh.state().trajectory.lateral.command == Command::Correct);
        CHECK(mh.state().trajectory.lateral.direction == -1);
    }
    {   // D. Turning: the turn rate is estimated from the track and used in the
        //    projection (checked against an independent integration).
        FlightSim sim;
        sim.xNm = 1.2;
        sim.trackDeg = 160.0;
        sim.turnRateDegSec = 1.5;  // turning right, toward the course heading
        InstrumentModel m;
        fly(m, sim, 4.0);
        const auto& L = m.state().trajectory.lateral;
        CHECK_NEAR(L.turnRateDegSec, 1.5, 0.15);
        FlightSim ref = sim;
        for (int i = 0; i < 400; ++i) ref.step(0.05);  // fly 20 s ahead exactly
        CHECK_NEAR(L.predicted20Dots, ref.dots(), 0.05);
    }
    {   // E. Localizer: prior scale first (no commands), then in-flight
        //    calibration of the angular width converges to the true 0.8 deg/dot.
        FlightSim sim;
        sim.angular = true;
        sim.xNm = 0.15;
        sim.trackDeg = 197.0;  // 8 deg intercept: enough geometry, slow closure
        InstrumentModel m;
        fly(m, sim, 2.0);
        CHECK(m.state().trajectory.lateral.confidence == Confidence::Prior);
        CHECK(!m.state().trajectory.lateral.commandValid);
        fly(m, sim, 10.0);
        const auto& L = m.state().trajectory.lateral;
        CHECK(L.confidence == Confidence::Calibrated);
        CHECK(std::fabs(L.degPerDot - 0.8) / 0.8 < 0.10);
        CHECK(L.commandValid);
        // Calibrated angular projection (distance closing) matches the truth 20 s ahead.
        FlightSim ref = sim;
        for (int i = 0; i < 400; ++i) ref.step(0.05);
        CHECK_NEAR(L.predicted20Dots, ref.dots(), 0.1);

        // A source change (retune) is a discontinuity: calibration is dropped.
        SimSnapshot retuned = sim.snapshot();
        retuned.nav.frequency = 10930;
        retuned.simTimeSec += 0.05;
        m.update(retuned);
        CHECK(m.state().trajectory.lateral.confidence != Confidence::Calibrated);
    }
    {   // F. No way to relate dots to distance (no DME, no station position):
        //    no trajectory estimate; rate-based cues remain.
        FlightSim sim;
        sim.angular = true;
        sim.dmeAvailable = false;
        sim.xNm = 0.15;
        sim.trackDeg = 197.0;
        InstrumentModel m;
        fly(m, sim, 3.0);
        CHECK(!m.state().trajectory.lateral.valid);
        CHECK(m.state().trajectory.lateral.confidence == Confidence::None);
        CHECK(m.state().lateral.out.valid);
    }
    {   // G. Vertical: required descent rate, overshoot, VS-based command.
        CHECK_NEAR(nav::TrajectoryPredictor::requiredVerticalSpeedFpm(90.0, 3.0), -477.6, 1.0);
        nav::TrajectoryPredictor tp;
        nav::TrajectoryInput in;
        in.verticalPredictive = true;
        in.verticalSpeedValid = true;
        in.groundspeedValid = true;
        in.groundspeedKt = 90.0;
        in.verticalDots = 0.8;           // high
        in.verticalRateDotsSec = -0.08;  // coming down fast
        in.verticalSpeedFpm = -1100.0;
        auto V = tp.update(in).vertical;
        CHECK(V.valid && V.crossValid && std::fabs(V.crossSec - 10.0) < 0.1);
        CHECK(V.overshoot);
        CHECK(V.command == Command::Ease && V.direction == +1);  // less descent

        in.simTimeSec = 0.05;
        in.verticalDots = -0.5;          // low
        in.verticalRateDotsSec = 0.0;
        in.verticalSpeedFpm = -700.0;    // descending more steeply than the path
        V = tp.update(in).vertical;
        CHECK(!V.overshoot);
        CHECK(V.command == Command::Correct && V.direction == +1 && V.strength == nav::Strength::Moderate);

        in.simTimeSec = 0.1;
        in.verticalSpeedFpm = -420.0;    // about the target (required + 75 fpm correction)
        V = tp.update(in).vertical;
        CHECK(V.command == Command::Hold);

        // Wording on a descending path: a vertical correction is phrased as a
        // change of descent rate, never "CLIMB" while above the path.
        nav::TrajectoryOutput tr;
        tr.vertical.valid = true;
        tr.vertical.commandValid = true;
        tr.vertical.targetVsFpm = -598.0;
        GuidanceAxis high;
        high.status = nav::SignalStatus::Valid;
        high.predictive = true;
        high.out.valid = true;
        high.out.positionDots = 0.8;
        high.out.command = Command::Ease;
        high.out.direction = +1;
        CHECK(render::describeAxis(high, render::Axis::Vertical, &tr).action == "LESS DESCENT");
        high.out.direction = -1;
        high.out.command = Command::Correct;
        CHECK(render::describeAxis(high, render::Axis::Vertical, &tr).action == "MORE DESCENT");
    }
}

void testGpsApproachAndDistance() {
    std::puts("gps approach + distance");
    // RNAV (GPS) RWY 20, LPV active, flying to VANNU (a GPS fix) - the
    // situation in the in-sim report that showed COURSE REFERENCE MODE.
    InstrumentModel m;
    double t = 0.0;
    for (; t <= 3.0; t += 0.05) {
        SimSnapshot s = ilsSnapshot(0.1f, 0.6f, t);
        s.nav.source = NavSource::Gps1;
        s.nav.gpsCdiMode = 11;
        s.nav.dmeValid = true;
        s.nav.dmeNm = 4.5f;
        s.approach.kind = ApproachKind::Rnav;
        s.approach.label = "RNAV RWY 20";
        s.approach.runway = "20";
        s.approach.airport = "KMWA";
        s.approach.approachMode = "LPV";
        s.approach.verticalExpected = true;
        s.approach.activeFixId = "VANNU";
        s.approach.activeFixKind = FixKind::GpsFix;
        m.update(s);
    }
    const InstrumentState& st = m.state();
    CHECK(st.mode == InstrumentMode::RunwayAlignment);
    CHECK(st.approachMode == "LPV" && st.runway == "20");
    CHECK(st.distanceKind == InstrumentState::DistanceKind::Gps);
    CHECK(st.activeFixKind == FixKind::GpsFix);
    CHECK(render::distanceText(st) == "DIST 4.5 NM TO VANNU");

    // Ground-based DME on a NAV source keeps the DME label.
    InstrumentModel mi;
    SimSnapshot ils = ilsSnapshot(0.1f, 0.1f, 0.0);
    ils.nav.dmeValid = true;
    ils.nav.dmeNm = 2.4f;
    mi.update(ils);
    CHECK(mi.state().distanceKind == InstrumentState::DistanceKind::Dme);
    CHECK(render::distanceText(mi.state()) == "DME 2.4 NM");

    // No distance -> nothing displayed.
    ils.nav.dmeValid = false;
    ils.simTimeSec = 0.05;
    mi.update(ils);
    CHECK(render::distanceText(mi.state()).empty());

    // LNAV (no vertical minima): a missing glidepath is not flagged.
    InstrumentModel ml;
    SimSnapshot lnav = ilsSnapshot(0.1f, 0.0f, 0.0);
    lnav.nav.verticalValid = false;
    lnav.approach.kind = ApproachKind::Rnav;
    lnav.approach.verticalExpected = false;
    ml.update(lnav);
    CHECK(ml.state().mode == InstrumentMode::RunwayAlignment && !ml.state().verticalExpected);
}

SimSnapshot ilsSnapshot(float hdef, float vdef, double t) {
    SimSnapshot s;
    s.simTimeSec = t;
    s.headingValid = true;
    s.magHeadingDeg = 215.0f;
    s.nav.lateralValid = true;
    s.nav.hdefDots = hdef;
    s.nav.verticalValid = true;
    s.nav.vdefDots = vdef;
    s.nav.courseValid = true;
    s.nav.courseDegMag = 218.0f;
    s.approach.kind = ApproachKind::Ils;
    s.approach.label = "ILS RWY 22";
    s.approach.runway = "22";
    s.approach.verticalExpected = true;
    s.nav.fromTo = 1;
    s.groundspeedValid = true;
    s.groundspeedKt = 110.0f;
    return s;
}

// Feed the model/validator a steady signal for `seconds` at 20 Hz.
template <typename Sink>
double feed(Sink&& sink, float hdef, float vdef, double t0, double seconds) {
    double t = t0;
    for (; t <= t0 + seconds + 1e-9; t += 0.05) sink(ilsSnapshot(hdef, vdef, t));
    return t;
}

void testNavValidator() {
    std::puts("nav validation");
    using nav::SignalStatus;
    nav::NavValidator v;
    auto upd = [&](const SimSnapshot& s) { return v.update(s); };

    // Acquisition: measured immediately, predictive only after settling.
    auto r = v.update(ilsSnapshot(0.5f, 0.2f, 0.0));
    CHECK(r.lateral.status == SignalStatus::Acquiring && r.lateral.measuredValid && !r.lateral.predictive);
    CHECK(r.vertical.status == SignalStatus::Acquiring && r.vertical.measuredValid);
    double t = feed(upd, 0.5f, 0.2f, 0.05, 2.0);
    r = v.validity();
    CHECK(r.lateral.status == SignalStatus::Valid && r.lateral.predictive);
    CHECK(r.vertical.status == SignalStatus::Valid && r.vertical.predictive);
    CHECK(!r.discontinuity);

    // Not established laterally (>= 2 dots): GS measured but no vertical guidance.
    for (float h = 0.5f; h <= 2.2f; h += 0.05f, t += 0.05) v.update(ilsSnapshot(h, 0.2f, t));
    r = v.validity();
    CHECK(r.lateral.status == SignalStatus::Valid);
    CHECK(r.vertical.status == SignalStatus::NotEstablished && r.vertical.measuredValid && !r.vertical.predictive);

    // Full scale: rate is meaningless -> measured only.
    for (float h = 2.2f; h <= 2.5f; h += 0.05f, t += 0.05) v.update(ilsSnapshot(h, 0.2f, t));
    r = v.validity();
    CHECK(r.lateral.status == SignalStatus::FullScale && r.lateral.measuredValid && !r.lateral.predictive);

    // Back on course after settling.
    for (float h = 2.5f; h >= 0.3f; h -= 0.05f, t += 0.05) v.update(ilsSnapshot(h, 0.2f, t));
    t = feed(upd, 0.3f, 0.2f, t, 2.0);
    CHECK(v.validity().lateral.status == SignalStatus::Valid);

    // Implausible value (GPS "-3" sentinel) and missing display flag = no signal.
    auto sentinel = ilsSnapshot(0.3f, -3.0f, t);
    CHECK(v.update(sentinel).vertical.status == SignalStatus::NoSignal);
    auto noGs = ilsSnapshot(0.3f, 0.2f, t + 0.05);
    noGs.nav.verticalValid = false;
    CHECK(v.update(noGs).vertical.status == SignalStatus::NoSignal);
    t = feed(upd, 0.3f, 0.2f, t + 0.1, 2.0);

    // Vertical is never valid without lateral.
    auto noLoc = ilsSnapshot(0.3f, 0.2f, t);
    noLoc.nav.lateralValid = false;
    r = v.update(noLoc);
    CHECK(r.lateral.status == SignalStatus::NoSignal && r.vertical.status == SignalStatus::NoSignal);
    r = v.update(ilsSnapshot(0.3f, 0.2f, t + 0.05));  // regained: acquire again
    CHECK(r.lateral.status == SignalStatus::Acquiring);
    t = feed(upd, 0.3f, 0.2f, t + 0.1, 2.0);

    // A jump between consecutive frames is a discontinuity.
    r = v.update(ilsSnapshot(-1.2f, 0.2f, t));
    CHECK(r.discontinuity && r.lateral.status == SignalStatus::Acquiring);
    t = feed(upd, -1.2f, 0.2f, t + 0.05, 2.0);

    // Source change (NAV1 -> NAV2) is a discontinuity.
    auto nav2 = ilsSnapshot(-1.2f, 0.2f, t);
    nav2.nav.source = NavSource::Nav2;
    r = v.update(nav2);
    CHECK(r.discontinuity && !r.discontinuityReason.empty());
    CHECK(r.lateral.status == SignalStatus::Acquiring);

    // VOR without TO/FROM indication is a NAV flag.
    nav::NavValidator vv;
    auto vor = ilsSnapshot(0.4f, 0.0f, 0.0);
    vor.approach.kind = ApproachKind::Vor;
    vor.nav.fromTo = 0;
    r = vv.update(vor);
    CHECK(r.lateral.status == SignalStatus::Flagged && !r.lateral.measuredValid);

    // Low groundspeed: valid measurement, no calculated cues.
    nav::NavValidator vs;
    double ts = 0.0;
    for (; ts <= 2.5; ts += 0.05) {
        auto s = ilsSnapshot(0.4f, 0.1f, ts);
        s.groundspeedKt = 12.0f;
        vs.update(s);
    }
    r = vs.validity();
    CHECK(r.lowGroundspeed && r.lateral.status == SignalStatus::Valid && !r.lateral.predictive);
}

void testNavModel() {
    std::puts("nav model");
    InstrumentModel m;
    auto upd = [&](const SimSnapshot& s) { m.update(s); };
    // X-Plane conventions, confirmed in-sim against X-Plane's G1000:
    //   hdef < 0 : CDI needle left       => aircraft RIGHT of course
    //   vdef < 0 : GS diamond above ctr  => aircraft BELOW the path
    double t = feed(upd, -0.4f, -0.3f, 1.0, 2.5);
    const InstrumentState& s = m.state();
    CHECK(s.mode == InstrumentMode::RunwayAlignment);
    CHECK_NEAR(s.lateral.out.positionDots, 0.4, 1e-6);
    CHECK_NEAR(s.vertical.out.positionDots, -0.3, 1e-6);
    CHECK(s.lateral.status == nav::SignalStatus::Valid && s.lateral.predictive);
    CHECK(s.vertical.status == nav::SignalStatus::Valid && s.vertical.predictive);
    CHECK(s.aligned);
    CHECK(s.verticalExpected);

    // Regression for the in-sim report (KMWA ILS 20): the G1000 showed the GS
    // diamond 1.7 dots BELOW centre (vdef = +1.7) while NavSight said LOW.
    t = feed(upd, -0.4f, 1.7f, t, 2.5);
    CHECK_NEAR(m.state().vertical.out.positionDots, 1.7, 1e-6);
    CHECK(render::describeAxis(m.state().vertical, render::Axis::Vertical).unit == "DOT HIGH");
    CHECK(m.state().vertical.out.direction <= 0);  // never "climb" when high
    CHECK(!m.state().aligned);

    SimSnapshot vor = ilsSnapshot(0.5f, 0.0f, t);
    vor.nav.verticalValid = false;
    vor.approach.kind = ApproachKind::Vor;
    m.update(vor);
    CHECK(m.state().mode == InstrumentMode::Navigation);
    CHECK(!m.state().vertical.out.valid);

    vor.nav.fromTo = 0;  // NAV flag
    vor.simTimeSec += 0.05;
    m.update(vor);
    CHECK(m.state().mode == InstrumentMode::Heading && m.state().lateralFlagged);

    SimSnapshot lost = ilsSnapshot(0.0f, 0.0f, t + 1.0);
    lost.nav.lateralValid = false;
    m.update(lost);
    CHECK(m.state().mode == InstrumentMode::Heading);
    CHECK(m.state().lateralLost);

    InstrumentModel::Settings inv;
    inv.invertLateral = true;
    inv.invertVertical = true;
    InstrumentModel mi(inv);
    mi.update(ilsSnapshot(-0.4f, 1.0f, 1.0));
    CHECK_NEAR(mi.state().lateral.out.positionDots, -0.4, 1e-6);
    CHECK_NEAR(mi.state().vertical.out.positionDots, -1.0, 1e-6);
}

void testGuidanceText() {
    std::puts("guidance text");
    GuidanceAxis lat;
    lat.status = nav::SignalStatus::Valid;
    lat.predictive = true;
    lat.out.valid = true;
    lat.out.positionDots = 0.4;
    lat.out.command = nav::Command::Correct;
    lat.out.direction = -1;
    lat.out.strength = nav::Strength::Slight;
    lat.out.trend = nav::Trend::Converging;
    lat.out.interceptValid = true;
    lat.out.interceptSec = 12.3;
    auto t = render::describeAxis(lat, render::Axis::Lateral);
    CHECK(t.value == "0.4" && t.unit == "DOT R");
    CHECK(t.action == "TURN LEFT" && t.qualifier == "SLIGHTLY");
    CHECK(t.trend == "CONVERGING" && t.prediction == "INTCPT 12S");
    CHECK(t.glyph == render::Glyph::Arrow && t.arrowDir.x < 0);
    CHECK(t.litPips == 1);

    GuidanceAxis ver;
    ver.status = nav::SignalStatus::Valid;
    ver.predictive = true;
    ver.out.valid = true;
    ver.out.positionDots = -1.3;
    ver.out.command = nav::Command::Ease;
    ver.out.direction = -1;
    ver.out.trend = nav::Trend::Converging;
    ver.pipCount = 5;
    t = render::describeAxis(ver, render::Axis::Vertical);
    CHECK(t.unit == "DOT LOW" && t.action == "DESCEND" && t.qualifier == "EASE OFF");
    CHECK(t.litPips == 3);

    t = render::describeAxis(GuidanceAxis{}, render::Axis::Vertical);
    CHECK(t.value == "---" && t.action == "NO GLIDEPATH");

    // Measured but not predictive: number shown, no calculated cues.
    GuidanceAxis fs = lat;
    fs.status = nav::SignalStatus::FullScale;
    fs.predictive = false;
    fs.out.positionDots = 2.5;
    t = render::describeAxis(fs, render::Axis::Lateral);
    CHECK(t.value == "2.5+" && t.action == "FULL SCALE");
    CHECK(t.glyph == render::Glyph::None && t.trend == "--" && t.prediction.empty());

    GuidanceAxis ne = ver;
    ne.status = nav::SignalStatus::NotEstablished;
    ne.predictive = false;
    t = render::describeAxis(ne, render::Axis::Vertical);
    CHECK(t.value == "1.3" && t.action == "NOT ESTABLISHED" && t.glyph == render::Glyph::None);

    GuidanceAxis acq = lat;
    acq.status = nav::SignalStatus::Acquiring;
    acq.predictive = false;
    CHECK(render::describeAxis(acq, render::Axis::Lateral).action == "ACQUIRING");

    GuidanceAxis flagged;
    flagged.status = nav::SignalStatus::Flagged;
    t = render::describeAxis(flagged, render::Axis::Lateral);
    CHECK(t.value == "---" && t.action == "NAV FLAG");
}

InstrumentState sampleState(InstrumentMode mode, double hdg, bool valid, bool runwayKnown = true,
                            float hdef = -0.8f) {
    InstrumentModel m;
    SimSnapshot s = ilsSnapshot(hdef, 0.4f, 1.0);
    s.magHeadingDeg = static_cast<float>(hdg);
    s.headingValid = valid;
    if (!runwayKnown) s.approach.runway.clear();
    if (mode == InstrumentMode::Heading) s.nav.lateralValid = false;
    if (mode == InstrumentMode::Navigation) s.approach.kind = ApproachKind::Vor;
    // Track + distance so the projected-track layer is drawn too (prior scale).
    s.groundTrackValid = true;
    s.groundTrackDegMag = 190.0f;
    s.nav.dmeValid = true;
    s.nav.dmeNm = 6.0f;
    s.verticalSpeedValid = true;
    s.verticalSpeedFpm = -700.0f;
    for (; s.simTimeSec < 4.0; s.simTimeSec += 0.05) {  // past acquisition: predictive cues drawn
        s.nav.hdefDots -= 0.002f;
        m.update(s);
    }
    return m.state();
}

void testRenderer() {
    std::puts("renderer");
    render::InstrumentRenderer r;
    using Layout = render::InstrumentRenderer::Layout;
    CHECK(render::InstrumentRenderer::chooseLayout(600, 600) == Layout::Standard);
    CHECK(render::InstrumentRenderer::chooseLayout(900, 450) == Layout::Standard);
    CHECK(render::InstrumentRenderer::chooseLayout(400, 400) == Layout::Compact);
    CHECK(render::InstrumentRenderer::chooseLayout(900, 300) == Layout::Compact);

    const render::InstrumentRenderer::Viewport viewports[] = {
        {100.0f, 50.0f, 400.0f, 400.0f},   // compact
        {0.0f, 0.0f, 600.0f, 600.0f},      // standard, reference size
        {100.0f, 50.0f, 700.0f, 500.0f},   // standard, letterboxed
        {0.0f, 0.0f, 1200.0f, 300.0f},     // very wide, small: compact letterboxed
    };
    for (const auto& vp : viewports) {
        for (InstrumentMode mode :
             {InstrumentMode::Heading, InstrumentMode::Navigation, InstrumentMode::RunwayAlignment}) {
            for (double hdg : {0.0, 47.3, 218.0, 359.9}) {
                for (int variant = 0; variant < 4; ++variant) {
                    const bool valid = variant != 1;
                    const bool runwayKnown = variant != 2;      // 2 = approach without verified runway
                    const float hdef = variant == 3 ? -2.7f : -0.8f;  // 3 = full scale (needle clamped)
                    RecordingCanvas c;
                    r.draw(c, sampleState(mode, hdg, valid, runwayKnown, hdef), vp);
                    c.flush();
                    CHECK(c.flushedVertices > 1000);
                    CHECK(c.nonFinite == 0);
                    // Everything stays inside the viewport (1px tolerance).
                    CHECK(c.minX >= vp.x - 1.0f && c.maxX <= vp.x + vp.width + 1.0f);
                    CHECK(c.minY >= vp.y - 1.0f && c.maxY <= vp.y + vp.height + 1.0f);
                }
            }
        }
    }
    RecordingCanvas tiny;
    r.draw(tiny, InstrumentState{}, {0, 0, 2.0f, 2.0f});
    CHECK(tiny.pendingVertexCount() == 0);
}

void testCollapsiblePanels() {
    std::puts("collapsible panels");
    using R = render::InstrumentRenderer;
    const R::Viewport vp{0.0f, 0.0f, 600.0f, 600.0f};
    R::ViewOptions shown, collapsed;
    collapsed.showPanels = false;

    // Clicks on a panel collapse; on the dial or header do nothing.
    CHECK(R::hitTest(vp, 60.0f, 300.0f, shown) == R::Hit::TogglePanels);   // LATERAL panel
    CHECK(R::hitTest(vp, 540.0f, 300.0f, shown) == R::Hit::TogglePanels);  // VERTICAL panel
    CHECK(R::hitTest(vp, 300.0f, 300.0f, shown) == R::Hit::None);          // dial
    CHECK(R::hitTest(vp, 300.0f, 560.0f, shown) == R::Hit::None);          // header
    // Collapsed: only the slim edge tabs respond; the panel area is now dial/bezel.
    CHECK(R::hitTest(vp, 20.0f, 270.0f, collapsed) == R::Hit::TogglePanels);
    CHECK(R::hitTest(vp, 580.0f, 270.0f, collapsed) == R::Hit::TogglePanels);
    CHECK(R::hitTest(vp, 100.0f, 300.0f, collapsed) == R::Hit::None);
    // Compact layout has no panels: nothing to toggle.
    CHECK(R::hitTest({0, 0, 380, 380}, 20.0f, 190.0f, shown) == R::Hit::None);
    // Letterboxed viewport: hit-test follows the drawn geometry.
    // (square centred at x = 200..800; LATERAL panel at x = 207..318)
    CHECK(R::hitTest({100, 0, 800, 600}, 260.0f, 300.0f, shown) == R::Hit::TogglePanels);
    CHECK(R::hitTest({100, 0, 800, 600}, 160.0f, 300.0f, shown) == R::Hit::None);  // letterbox margin

    // Collapsed draws fewer triangles, stays inside the viewport, and still
    // shows the measured value (tabs) - covered across modes and cautions.
    R r;
    for (InstrumentMode mode : {InstrumentMode::Heading, InstrumentMode::RunwayAlignment}) {
        for (float hdef : {-0.8f, -2.7f}) {
            const InstrumentState s = sampleState(mode, 200.0, true, true, hdef);
            RecordingCanvas full, tabs;
            r.draw(full, s, vp, shown);
            full.flush();
            r.draw(tabs, s, vp, collapsed);
            tabs.flush();
            CHECK(tabs.flushedVertices < full.flushedVertices);
            CHECK(tabs.nonFinite == 0);
            CHECK(tabs.minX >= -1.0f && tabs.maxX <= 601.0f && tabs.minY >= -1.0f && tabs.maxY <= 601.0f);
        }
    }

    // Persisted.
    const auto path = (std::filesystem::temp_directory_path() / "navsight_panels.prf").string();
    Config c;
    c.showPanels = false;
    c.brightness = 2;
    c.fastRendering = false;
    CHECK(c.save(path));
    Config d;
    CHECK(d.load(path) && !d.showPanels && d.brightness == 2 && !d.fastRendering);
    std::filesystem::remove(path);
}

// Counts flushed vertices of one exact colour (alpha ignored).
class ColorCountCanvas : public render::Canvas {
public:
    explicit ColorCountCanvas(render::Color c) : want_(c) {}
    void flush() override {
        for (const Vertex& v : batch_)
            if (v.color.r == want_.r && v.color.g == want_.g && v.color.b == want_.b) ++count;
        batch_.clear();
    }
    std::size_t count = 0;

private:
    render::Color want_;
};

void testNeedleColourBySource() {
    std::puts("needle colour by source");
    const render::Theme day = render::makeTheme(render::Brightness::Day);
    render::InstrumentRenderer r;
    // NAV1 ILS (ground based): needle and GS indicator in NAV green.
    InstrumentState ils = sampleState(InstrumentMode::RunwayAlignment, 210.0, true);
    CHECK(ils.groundBasedNav);
    ColorCountCanvas greenNav(day.kNavGreen);
    r.draw(greenNav, ils, {0, 0, 600, 600});
    greenNav.flush();
    CHECK(greenNav.count > 50);
    // Same picture on GPS: no NAV green anywhere; the needle is magenta again.
    InstrumentState gps = ils;
    gps.groundBasedNav = false;
    ColorCountCanvas greenGps(day.kNavGreen);
    r.draw(greenGps, gps, {0, 0, 600, 600});
    greenGps.flush();
    CHECK(greenGps.count == 0);
    ColorCountCanvas magNav(day.kMagenta), magGps(day.kMagenta);
    r.draw(magNav, ils, {0, 0, 600, 600});
    magNav.flush();
    r.draw(magGps, gps, {0, 0, 600, 600});
    magGps.flush();
    CHECK(magGps.count > magNav.count + 50);  // needle + GS moved from magenta to green
    // Model: NAV sources are ground based, GPS sources are not.
    InstrumentModel m;
    SimSnapshot s = ilsSnapshot(0.2f, 0.1f, 0.0);
    s.nav.source = NavSource::Nav2;
    m.update(s);
    CHECK(m.state().groundBasedNav);
    s.nav.source = NavSource::Gps1;
    s.simTimeSec = 0.05;
    m.update(s);
    CHECK(!m.state().groundBasedNav);
}

void testRunwayOverride() {
    std::puts("runway override");
    CHECK(nav::normaliseRunway("20") == "20" && nav::normaliseRunway("4l") == "04L");
    CHECK(nav::normaliseRunway(" 9 r ") == "09R" && nav::normaliseRunway("37").empty());
    CHECK(nav::normaliseRunway("ABC").empty() && nav::normaliseRunway("").empty());
    CHECK((nav::runwayCandidates(203.0) == std::vector<std::string>{"19", "20", "21"}));
    CHECK((nav::runwayCandidates(205.0) == std::vector<std::string>{"20", "21", "22"}));  // 20.5 rounds up
    CHECK((nav::runwayCandidates(355.0) == std::vector<std::string>{"35", "36", "01"}));
    CHECK((nav::runwayCandidates(4.0) == std::vector<std::string>{"35", "36", "01"}));
    CHECK(!nav::runwayMismatchesCourse("20", 205.0) && !nav::runwayMismatchesCourse("36", 5.0));
    CHECK(nav::runwayMismatchesCourse("02", 205.0) && nav::runwayMismatchesCourse("24", 205.0));

    // Label rewriting / kinds.
    nav::RunwayOverride o;
    o.active = true;
    o.runway = "21";
    ApproachInfo loc;
    loc.kind = ApproachKind::Localizer;
    loc.label = "LOC";
    nav::applyOverride(loc, o);
    CHECK(loc.label == "LOC RWY 21" && loc.runway == "21" && loc.runwayManual);
    ApproachInfo rnav;
    rnav.kind = ApproachKind::Rnav;
    rnav.label = "RNAV RWY 20";
    nav::applyOverride(rnav, o);
    CHECK(rnav.label == "RNAV RWY 21");
    ApproachInfo vor;
    vor.kind = ApproachKind::Vor;
    vor.label = "VOR SEA";
    nav::applyOverride(vor, o);
    CHECK(vor.kind == ApproachKind::Manual && vor.label == "RWY 21" && vor.isApproach());

    // Model: an unverified LOC shows a course band; the override turns it into
    // a runway strip marked manual - without disturbing validation.
    InstrumentModel m;
    auto upd = [&](const SimSnapshot& s) { m.update(s); };
    auto locSnap = [](float hdef, double t) {
        SimSnapshot s = ilsSnapshot(hdef, 0.1f, t);
        s.approach.kind = ApproachKind::Localizer;
        s.approach.label = "LOC";
        s.approach.runway.clear();
        s.approach.verticalExpected = false;
        s.nav.verticalValid = false;
        s.nav.frequency = 11190;
        return s;
    };
    double t = 0.0;
    for (; t < 2.5; t += 0.05) upd(locSnap(-0.3f, t));
    CHECK(m.state().runway.empty() && !m.state().runwayManual);
    CHECK(m.setRunwayOverride("20"));
    CHECK(!m.setRunwayOverride("XX"));  // rejected, previous kept
    upd(locSnap(-0.3f, t));
    t += 0.05;
    CHECK(m.state().runway == "20" && m.state().runwayManual);
    CHECK(m.state().mode == InstrumentMode::RunwayAlignment && m.state().approachLabel == "LOC RWY 20");
    CHECK(m.state().lateral.status == nav::SignalStatus::Valid);  // no false discontinuity
    CHECK(!m.state().runwayCourseMismatch);  // 20 vs course 218: within 30 deg
    m.setRunwayOverride("02");
    upd(locSnap(-0.3f, t));
    t += 0.05;
    CHECK(m.state().runwayCourseMismatch);
    // Retune: the override is dropped (bound to the source it was set on).
    SimSnapshot retuned = locSnap(-0.3f, t);
    retuned.nav.frequency = 10930;
    upd(retuned);
    CHECK(m.overrideClearedBySourceChange() && !m.runwayOverride().active && !m.state().runwayManual);
    // Clearing explicitly.
    m.setRunwayOverride("20");
    upd(retuned);
    m.clearRunwayOverride();
    upd(retuned);
    CHECK(!m.state().runwayManual);
    // Aircraft reload clears too.
    m.setRunwayOverride("20");
    m.reset();
    CHECK(!m.runwayOverride().active);
}

void testRunwayPicker() {
    std::puts("runway picker");
    using Act = render::RunwayPicker::Action;
    render::RunwayPicker p;
    p.openFor(true, 203.0, "");
    CHECK(p.open && p.candidates.size() == 3 && p.selected == 1 && p.designator() == "20");
    p.press(render::Hit::PickerCandidate0);
    p.press(render::Hit::PickerSideL);
    CHECK(p.designator() == "19L" && p.entryText() == "19L");
    auto r = p.press(render::Hit::PickerApply);
    CHECK(r.action == Act::Apply && r.runway == "19L" && !p.open);

    // Typing: digits + side, backspace, Enter.
    p.openFor(true, 203.0, "");
    p.key('1', 0);
    CHECK(p.entryText() == "01");  // a single digit is already a valid designator
    p.key('6', 0);
    p.key('r', 0);
    CHECK(p.designator() == "16R");
    p.key(0, 0x08);  // backspace removes the side first
    CHECK(p.designator() == "16");
    r = p.key(0, 0x0D);  // Enter
    CHECK(r.action == Act::Apply && r.runway == "16");

    // Invalid typed number: APPLY does nothing; Esc closes; AUTO clears.
    p.openFor(true, 203.0, "");
    p.key('3', 0);
    p.key('7', 0);
    CHECK(p.designator().empty() && p.press(render::Hit::PickerApply).action == Act::None && p.open);
    CHECK(p.key(0, 0x1B).action == Act::Close && !p.open);
    p.openFor(true, 203.0, "20");
    CHECK(p.press(render::Hit::PickerAuto).action == Act::Clear);
    p.openFor(true, 203.0, "");
    CHECK(p.press(render::Hit::PickerOutside).action == Act::Close);

    // Re-opening pre-selects the active override.
    p.openFor(true, 203.0, "21R");
    CHECK(p.selected == 2 && p.side == 'R' && p.designator() == "21R");
    p.openFor(true, 203.0, "04");  // not among the candidates: shown as typed
    CHECK(p.designator() == "04");
    // No course: no candidates, typing still works.
    p.openFor(false, 0.0, "");
    CHECK(p.candidates.empty() && p.designator().empty());
    p.key('2', 0);
    p.key('2', 0);
    CHECK(p.designator() == "22");

    // Hit-testing matches the drawn picker (standard and compact).
    using R = render::InstrumentRenderer;
    R::ViewOptions view;
    for (const R::Viewport vp : {R::Viewport{0, 0, 600, 600}, R::Viewport{0, 0, 380, 380}}) {
        view.picker.open = false;
        const float s = vp.width;
        CHECK(R::hitTest(vp, s * 0.8f, s * 0.95f, view) == render::Hit::OpenRunwayPicker);
        view.picker.openFor(true, 205.0, "");
        CHECK(R::hitTest(vp, s * 0.5f, s * 0.5f, view) != render::Hit::PickerOutside);
        CHECK(R::hitTest(vp, s * 0.02f, s * 0.02f, view) == render::Hit::PickerOutside);
        // Sweep the picker: every button kind is reachable.
        bool seen[16] = {};
        for (float y = 0; y < s; y += 2.0f)
            for (float x = 0; x < s; x += 2.0f) seen[static_cast<int>(R::hitTest(vp, x, y, view))] = true;
        for (render::Hit h : {render::Hit::PickerCandidate0, render::Hit::PickerCandidate1, render::Hit::PickerCandidate2,
                              render::Hit::PickerSideNone, render::Hit::PickerSideL, render::Hit::PickerSideC,
                              render::Hit::PickerSideR, render::Hit::PickerAuto, render::Hit::PickerApply})
            CHECK(seen[static_cast<int>(h)]);
        CHECK(!seen[static_cast<int>(render::Hit::TogglePanels)]);  // picker captures clicks
    }
}

void testPolish() {
    std::puts("polish: feathering, smoothing, themes");
    using R = render::InstrumentRenderer;
    // Feathered (anti-aliased) geometry: fringe vertices are transparent and
    // the whole instrument still stays inside the viewport.
    {
        RecordingCanvas c;
        c.setFeather(1.0f);
        c.fillCircle({50, 50}, 10.0f, {1, 1, 1, 1}, 16);
        c.flush();
        CHECK(c.flushedVertices == 16 * 3 * 3);  // fan + 2 fringe triangles per segment
        R r;
        RecordingCanvas full;
        full.setFeather(1.0f);
        R::ViewOptions v;
        v.picker.openFor(true, 218.0, "20");
        r.draw(full, sampleState(InstrumentMode::RunwayAlignment, 200.0, true), {0, 0, 600, 600}, v);
        full.flush();
        CHECK(full.nonFinite == 0 && full.minX >= -2.0f && full.maxX <= 602.0f && full.minY >= -2.0f &&
              full.maxY <= 602.0f);
    }
    // Display smoother: eases a step without overshoot, snaps on jumps / invalid.
    {
        render::DisplaySmoother sm;
        render::DisplayFx fx;
        InstrumentState s = sampleState(InstrumentMode::RunwayAlignment, 200.0, true);
        s.lateral.out.positionDots = 0.0;
        sm.apply(s, 0.016, fx);
        s.lateral.out.positionDots = 1.0;
        double prev = 0.0, v = 0.0;
        bool monotone = true;
        for (int i = 0; i < 60; ++i) {  // 1 s at 60 fps
            v = sm.apply(s, 1.0 / 60.0, fx).lateral.out.positionDots;
            if (v < prev - 1e-9 || v > 1.0 + 1e-9) monotone = false;
            if (i == 5) CHECK(v > 0.05 && v < 0.9);  // moving, not jumped
            prev = v;
        }
        CHECK(monotone);
        CHECK_NEAR(v, 1.0, 0.01);
        s.lateral.out.positionDots = -1.0;  // 2-dot jump: snaps
        CHECK_NEAR(sm.apply(s, 1.0 / 60.0, fx).lateral.out.positionDots, -1.0, 1e-9);

        // Chevron cue: fades in, fades out on HOLD, vanishes at once when invalid.
        sm.reset();
        s.lateral.predictive = true;
        s.lateral.out.command = nav::Command::Hold;
        sm.apply(s, 0.05, fx);  // first frame: no cue
        s.lateral.out.command = nav::Command::Correct;
        s.lateral.out.direction = -1;
        sm.apply(s, 0.05, fx);
        CHECK(fx.lateralChevrons.present && fx.lateralChevrons.alpha > 0.1f && fx.lateralChevrons.alpha < 0.5f);
        for (int i = 0; i < 10; ++i) sm.apply(s, 0.05, fx);
        CHECK_NEAR(fx.lateralChevrons.alpha, 1.0f, 1e-6f);
        s.lateral.out.command = nav::Command::Hold;
        sm.apply(s, 0.05, fx);
        CHECK(fx.lateralChevrons.present && fx.lateralChevrons.alpha < 1.0f && fx.lateralChevrons.direction == -1);
        for (int i = 0; i < 10; ++i) sm.apply(s, 0.05, fx);
        CHECK(!fx.lateralChevrons.present);
        s.lateral.out.command = nav::Command::Correct;
        for (int i = 0; i < 10; ++i) sm.apply(s, 0.05, fx);
        s.lateral.predictive = false;
        sm.apply(s, 0.05, fx);
        CHECK(!fx.lateralChevrons.present && fx.lateralChevrons.alpha == 0.0f);
        // Source change snaps everything.
        s.approachLabel = "ILS RWY 04";
        s.lateral.out.positionDots = 0.7;
        CHECK_NEAR(sm.apply(s, 0.016, fx).lateral.out.positionDots, 0.7, 1e-9);
    }
    // Themes: Day = reference colours; Dim/Night progressively darker; every
    // brightness renders inside the viewport.
    {
        const render::Theme day = render::makeTheme(render::Brightness::Day);
        const render::Theme dim = render::makeTheme(render::Brightness::Dim);
        const render::Theme night = render::makeTheme(render::Brightness::Night);
        CHECK_NEAR(day.kCyan.g, 0.82f, 1e-6f);
        auto lum = [](const render::Color& c) { return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b; };
        for (auto pick : {&render::Theme::kScale, &render::Theme::kCyan, &render::Theme::kAmber,
                          &render::Theme::kMagenta, &render::Theme::kPanelBg})
            CHECK(lum(day.*pick) > lum(dim.*pick) && lum(dim.*pick) > lum(night.*pick));
        CHECK(lum(night.kScale) < lum(night.kCyan) * 1.2f);  // whites tamed most at night
        CHECK(render::nextBrightness(render::Brightness::Night) == render::Brightness::Day);
        R r;
        for (auto b : {render::Brightness::Day, render::Brightness::Dim, render::Brightness::Night}) {
            R::ViewOptions v;
            v.brightness = b;
            RecordingCanvas c;
            r.draw(c, sampleState(InstrumentMode::RunwayAlignment, 120.0, true), {0, 0, 600, 600}, v);
            c.flush();
            CHECK(c.flushedVertices > 1000 && c.nonFinite == 0 && c.maxX <= 601.0f);
            CHECK(render::brightness() == b);
        }
        render::setBrightness(render::Brightness::Day);
    }
}

// ---- Source check fixtures: RNAV/ILS RWY 20 at KMWA, final course 200 true,
// FAF 5 nm out. Positions are placed along / across the final in nm.
constexpr double kThrLat = 37.75, kThrLon = -89.0;

void placeOnFinal(SimSnapshot& s, double distNm, double rightNm) {
    const double inbound = nav::degToRad(200.0);
    // Outbound from the threshold along 020 true, then right of the inbound course.
    const double north = -distNm * std::cos(inbound) + rightNm * -std::sin(inbound);
    const double east = -distNm * std::sin(inbound) + rightNm * std::cos(inbound);
    s.latitudeDeg = kThrLat + north / 60.0;
    s.longitudeDeg = kThrLon + east / (60.0 * std::cos(nav::degToRad(kThrLat)));
}

RadioSnapshot liveIls(const std::string& runway, const std::string& airport = "KMWA", float course = 200.0f) {
    RadioSnapshot r;
    r.available = true;
    r.frequency = 10970;
    r.signal = true;
    r.courseValid = true;
    r.courseDegMag = course;
    r.fromTo = 1;
    r.ident = "IMWA";
    r.station.kind = ApproachKind::Ils;
    r.station.label = "ILS RWY " + runway;
    r.station.runway = runway;
    r.station.airport = airport;
    r.station.stationPosValid = true;
    r.station.stationLatDeg = kThrLat - 0.03;  // localizer antenna past the far end
    r.station.stationLonDeg = kThrLon - 0.01;
    return r;
}

// HSI on GPS1, GPS1 in `gpsMode` with RWY 20 loaded, established 6 nm out.
SimSnapshot sourceSnap(int gpsMode, double t = 1.0) {
    SimSnapshot s;
    s.simTimeSec = t;
    s.headingValid = true;
    s.magHeadingDeg = 200.0f;
    s.trueHeadingDeg = 200.0f;  // variation 0
    s.groundspeedValid = true;
    s.groundspeedKt = 110.0f;
    s.groundTrackValid = true;
    s.groundTrackDegMag = 200.0f;
    placeOnFinal(s, 6.0, 0.0);
    s.nav.source = NavSource::Gps1;
    s.nav.lateralValid = true;
    s.nav.courseValid = true;
    s.nav.courseDegMag = 200.0f;
    s.approach.kind = ApproachKind::Rnav;
    s.approach.label = "RNAV RWY 20";
    s.approach.runway = "20";
    s.approach.airport = "KMWA";
    GpsApproachPlanSnapshot& p = s.gpsPlan;
    p.loaded = true;
    p.runway = "20";
    p.airport = "KMWA";
    p.finalValid = true;
    p.thresholdLatDeg = kThrLat;
    p.thresholdLonDeg = kThrLon;
    SimSnapshot faf;
    placeOnFinal(faf, 5.0, 0.0);
    p.fafLatDeg = faf.latitudeDeg;
    p.fafLonDeg = faf.longitudeDeg;
    p.gpsCdiMode = gpsMode;
    s.radios[0].available = s.radios[1].available = true;
    return s;
}

// Same, but the HSI on NAV1 with the given localizer/ILS (or other station) tuned.
SimSnapshot navSourceSnap(int gpsMode, ApproachKind kind, const std::string& runway,
                          const std::string& airport = "KMWA") {
    SimSnapshot s = sourceSnap(gpsMode);
    s.nav.source = NavSource::Nav1;
    s.approach = ApproachInfo{};
    s.approach.kind = kind;
    s.approach.runway = runway;
    s.approach.airport = airport;
    s.approach.label = kind == ApproachKind::Vor ? "VOR FAM" : "ILS RWY " + runway;
    s.radios[0] = liveIls(runway, airport);
    s.radios[0].station = s.approach;
    s.hsiObsValid = true;
    s.hsiObsDegMag = 200.0f;
    return s;
}

std::string checkCode(const SimSnapshot& s, bool navContext = true) {
    return nav::evaluateSourceCheck(s, navContext).code;
}

void testSourceCheck() {
    std::puts("source check");
    constexpr int kTerm = 2, kEnr = 1, kLpv = 11, kLnav = 7, kApr = 5, kMapr = 4;

    // --- Established-on-final geometry.
    {
        SimSnapshot s = sourceSnap(kTerm);
        nav::FinalGeometry g = nav::finalApproachGeometry(s.gpsPlan, s.latitudeDeg, s.longitudeDeg, 0.0);
        CHECK(g.valid);
        CHECK_NEAR(g.distToThresholdNm, 6.0, 0.02);
        CHECK_NEAR(g.crossTrackNm, 0.0, 0.02);
        CHECK_NEAR(g.finalCourseDegMag, 200.0, 0.2);
        CHECK(nav::establishedOnFinal(g, s));
        // Variation 3 W: magnetic course is 3 deg more than true.
        g = nav::finalApproachGeometry(s.gpsPlan, s.latitudeDeg, s.longitudeDeg, -3.0);
        CHECK_NEAR(g.finalCourseDegMag, 203.0, 0.2);
        placeOnFinal(s, 4.0, 0.6);
        g = nav::finalApproachGeometry(s.gpsPlan, s.latitudeDeg, s.longitudeDeg, 0.0);
        CHECK_NEAR(g.distToThresholdNm, 4.0, 0.02);
        CHECK_NEAR(g.crossTrackNm, 0.6, 0.02);
        CHECK(nav::establishedOnFinal(g, s));
        auto established = [](SimSnapshot x) {
            return nav::establishedOnFinal(
                nav::finalApproachGeometry(x.gpsPlan, x.latitudeDeg, x.longitudeDeg, 0.0), x);
        };
        SimSnapshot x = sourceSnap(kTerm);
        placeOnFinal(x, 4.0, 1.5);
        CHECK(!established(x));  // too far across
        placeOnFinal(x, 13.0, 0.0);
        CHECK(!established(x));  // beyond 12 nm
        placeOnFinal(x, -0.5, 0.0);
        CHECK(!established(x));  // past the threshold (missed approach)
        x = sourceSnap(kTerm);
        x.groundTrackDegMag = 20.0f;
        CHECK(!established(x));  // outbound on the final course
        x.groundTrackDegMag = 235.0f;
        CHECK(!established(x));  // 35 deg off
        x.groundTrackDegMag = 225.0f;
        CHECK(established(x));   // intercepting at 25 deg counts
        x.groundspeedKt = 30.0f;
        CHECK(!established(x));  // taxiing / not flying
        x = sourceSnap(kTerm);
        x.gpsPlan.finalValid = false;
        CHECK(!nav::finalApproachGeometry(x.gpsPlan, x.latitudeDeg, x.longitudeDeg, 0.0).valid);
        // Variation from the heading pair: true 197 / magnetic 200 = 3 W.
        x = sourceSnap(kTerm);
        x.trueHeadingDeg = 197.0f;
        x.groundTrackDegMag = 203.0f;
        const nav::SourceCheckResult r = nav::evaluateSourceCheck(x, true);
        CHECK(r.detail.find("final=203") != std::string::npos);
    }

    // --- APPR_NOT_ACTIVE: loaded, flying the final on GPS, not activated.
    {
        nav::SourceCheckResult r = nav::evaluateSourceCheck(sourceSnap(kTerm), true);
        CHECK(r.caution() && r.code == "APPR_NOT_ACTIVE");
        CHECK(r.banner == "GPS APPROACH NOT ACTIVE - ACTIVATE APPR" && r.chip == "APPR NOT ACTV");
        CHECK(checkCode(sourceSnap(kEnr)) == "APPR_NOT_ACTIVE");
        CHECK(checkCode(sourceSnap(kLpv)) == "OK");   // activated: LPV
        CHECK(checkCode(sourceSnap(kLnav)) == "OK");
        CHECK(checkCode(sourceSnap(kApr)) == "OK");   // APR
        CHECK(checkCode(sourceSnap(kMapr)) == "OK");  // missed approach
        SimSnapshot s = sourceSnap(kTerm);
        s.groundTrackDegMag = 290.0f;                  // vectored across the final
        CHECK(checkCode(s) == "OK");
        s = sourceSnap(kTerm);
        placeOnFinal(s, 20.0, 0.0);                    // still far out
        CHECK(checkCode(s) == "OK");
    }

    // --- GPS_ON_LOC: a live ILS serves the loaded approach, HSI on GPS.
    {
        // G1000 flying a loaded ILS as an overlay: LNAV.
        SimSnapshot s = sourceSnap(kLnav);
        s.radios[0] = liveIls("20");
        nav::SourceCheckResult r = nav::evaluateSourceCheck(s, true);
        CHECK(r.code == "GPS_ON_LOC");
        CHECK(r.banner == "ILS APPROACH ON GPS - SELECT NAV1" && r.chip == "CDI GPS>NAV1");
        s.radios[0] = RadioSnapshot{};
        s.radios[1] = liveIls("20");
        s.radios[1].station.kind = ApproachKind::Localizer;
        r = nav::evaluateSourceCheck(s, true);
        CHECK(r.code == "GPS_ON_LOC" && r.banner == "LOC APPROACH ON GPS - SELECT NAV2");
        CHECK(checkCode(sourceSnap(kApr)) == "OK");
        // Not activated, unknown procedure, localizer alive: either fix applies.
        s = sourceSnap(kTerm);
        s.radios[0] = liveIls("20");
        r = nav::evaluateSourceCheck(s, true);
        CHECK(r.code == "APPR_NOT_ACTIVE" && r.banner == "GPS APPR NOT ACTIVE - ACTIVATE OR SELECT NAV1" &&
              r.chip == "APPR NOT ACTV");
        s.gpsPlan.gpsCdiMode = kEnr;
        CHECK(checkCode(s) == "APPR_NOT_ACTIVE");
        // Look-alike: flying the RNAV (LPV active) with the same-runway ILS tuned.
        s = sourceSnap(kLpv);
        s.radios[0] = liveIls("20");
        CHECK(checkCode(s) == "OK");
        // An ILS that serves another runway / airport doesn't count.
        s = sourceSnap(kTerm);
        s.radios[0] = liveIls("02");
        CHECK(checkCode(s) == "APPR_NOT_ACTIVE");
        s.radios[0] = liveIls("20", "KCGI");
        CHECK(checkCode(s) == "APPR_NOT_ACTIVE");
        // No signal: not "live".
        s.radios[0] = liveIls("20");
        s.radios[0].signal = false;
        CHECK(checkCode(s) == "APPR_NOT_ACTIVE");
        // No approach plan: course match + nearby localizer.
        s = sourceSnap(kEnr);
        s.gpsPlan = GpsApproachPlanSnapshot{};
        s.gpsPlan.gpsCdiMode = kEnr;
        s.radios[0] = liveIls("20");
        CHECK(checkCode(s) == "GPS_ON_LOC");
        s.radios[0].courseDegMag = 240.0f;  // a different localizer course
        CHECK(checkCode(s) == "OK");
        s.radios[0].courseDegMag = 200.0f;
        s.radios[0].station.stationLatDeg = kThrLat + 1.0;  // 60 nm away
        CHECK(checkCode(s) == "OK");
    }

    // --- Procedure type: an ILS loaded in the GPS (X-Plane's G1000 flies it in LNAV).
    {
        using PK = nav::ProcedureKind;
        CHECK(nav::classifyProcedure("I20", "") == PK::LocBased);
        CHECK(nav::classifyProcedure("ILS20", "") == PK::LocBased);
        CHECK(nav::classifyProcedure("ILS RWY 20", "") == PK::LocBased);
        CHECK(nav::classifyProcedure("L16R", "") == PK::LocBased);
        CHECK(nav::classifyProcedure("R20", "") == PK::Rnav);
        CHECK(nav::classifyProcedure("RNAV20", "") == PK::Rnav);
        CHECK(nav::classifyProcedure("H04L", "") == PK::Rnav);
        CHECK(nav::classifyProcedure("D33", "") == PK::Other);
        CHECK(nav::classifyProcedure("VOR-A", "") == PK::Other);
        CHECK(nav::classifyProcedure("", "W20A") == PK::Rnav);
        CHECK(nav::classifyProcedure("", "RNV20Y") == PK::Rnav);
        // In-sim: fas_id follows the ILS tuned on NAV1, whatever the GPS has loaded.
        CHECK(nav::classifyProcedure("", "IMWA") == PK::Unknown);
        CHECK(nav::classifyProcedure("", "") == PK::Unknown);
        CHECK(nav::classifyProcedure("R20", "IMWA") == PK::Rnav);  // the name decides

        // In-sim case 1: ILS 20 loaded, G1000 in LNAV, CDI back on GPS,
        // ILS 20 alive on NAV1, short final -> select NAV1.
        SimSnapshot s = sourceSnap(kLnav);
        placeOnFinal(s, 0.9, 0.0);
        s.radios[0] = liveIls("20");
        s.gpsPlan.fasId = "IMWA";  // as logged
        s.gpsPlan.ltpId = "DME";
        nav::SourceCheckResult r = nav::evaluateSourceCheck(s, true);
        CHECK(r.code == "GPS_ON_LOC" && r.banner == "ILS APPROACH ON GPS - SELECT NAV1" && r.chip == "CDI GPS>NAV1");
        CHECK(r.detail.find("proc=unknown (name='' fas='IMWA' ltp='DME')") != std::string::npos);
        // In-sim case 2: RNAV 20 loaded and flown in LPV, same ILS alive on
        // NAV1 with the same fas_id/ltp_id -> nothing wrong.
        s.gpsPlan.gpsCdiMode = kLpv;
        CHECK(checkCode(s) == "OK");
        for (int mode : {6, 8, 9, 10, 12, 13}) {  // every RNAV-only mode
            s.gpsPlan.gpsCdiMode = mode;
            CHECK(checkCode(s) == "OK");
        }
        // Known limitation: an LNAV-only RNAV approach flown with the
        // same-runway ILS alive looks exactly like case 1, unless the FMS says.
        s.gpsPlan.gpsCdiMode = kLnav;
        s.gpsPlan.fasId = "W20A";
        CHECK(checkCode(s) == "OK");
        s.gpsPlan.fasId = "IMWA";
        s.gpsPlan.procedureName = "I20";
        CHECK(checkCode(s) == "GPS_ON_LOC");
        s.gpsPlan.gpsCdiMode = kLpv;  // whatever the GPS mode says
        CHECK(checkCode(s) == "GPS_ON_LOC");
        s.gpsPlan.gpsCdiMode = kMapr;  // missed approach: not flagged
        CHECK(checkCode(s) == "OK");
        // Not on the final (vectored across it): no caution yet.
        s = sourceSnap(kLnav);
        s.gpsPlan.procedureName = "I20";
        s.radios[0] = liveIls("20");
        s.groundTrackDegMag = 290.0f;
        CHECK(checkCode(s) == "OK");
        // ILS loaded, on final on GPS, localizer not tuned/received.
        s = sourceSnap(kTerm);
        s.gpsPlan.procedureName = "I20";
        r = nav::evaluateSourceCheck(s, true);
        CHECK(r.code == "GPS_ON_LOC" && r.banner == "ILS APPROACH ON GPS - TUNE AND SELECT NAV" &&
              r.chip == "CDI GPS>NAV");  // not "activate APPR": that's not the fix
        s.gpsPlan.procedureName = "L20";
        CHECK(nav::evaluateSourceCheck(s, true).banner == "LOC APPROACH ON GPS - TUNE AND SELECT NAV");
        // Flying that ILS correctly on NAV1 with the G1000 in LNAV: fine.
        s = navSourceSnap(kLnav, ApproachKind::Ils, "20");
        s.gpsPlan.procedureName = "I20";
        CHECK(checkCode(s) == "OK");
        // ILS loaded, NAV on a VOR, GPS in LNAV: not "RNAV approach active".
        s = navSourceSnap(kLnav, ApproachKind::Vor, "");
        s.gpsPlan.procedureName = "I20";
        CHECK(checkCode(s) != "NAV_ON_RNAV");
        // A named RNAV procedure: the same-runway ILS never triggers rule 1,
        // even before activation (then "activate" is the right advice).
        s = sourceSnap(kTerm);
        s.gpsPlan.procedureName = "R20";
        s.radios[0] = liveIls("20");
        CHECK(checkCode(s) == "APPR_NOT_ACTIVE");
        s.gpsPlan.gpsCdiMode = kLpv;
        CHECK(checkCode(s) == "OK");

        // Model: the header names the procedure flown on GPS - only when the
        // FMS names it (the tuned ILS's fas_id doesn't relabel an RNAV).
        InstrumentModel m;
        SimSnapshot g = sourceSnap(kLnav);
        g.gpsPlan.procedureName = "I20";
        g.radios[0] = liveIls("20");
        m.update(g);
        CHECK(m.state().approachLabel == "ILS RWY 20");
        g.gpsPlan.procedureName = "R20";
        g.simTimeSec = 1.05;
        m.update(g);
        CHECK(m.state().approachLabel == "RNAV RWY 20");
        g.gpsPlan.procedureName.clear();
        g.gpsPlan.fasId = "IMWA";
        g.gpsPlan.ltpId = "DME";
        g.gpsPlan.gpsCdiMode = kLpv;
        g.simTimeSec = 1.1;
        m.update(g);
        CHECK(m.state().approachLabel == "RNAV RWY 20");
    }

    // --- NAV_ON_RNAV: GPS1 flying an RNAV approach, HSI on NAV.
    {
        CHECK(checkCode(navSourceSnap(kLpv, ApproachKind::Vor, "")) == "NAV_ON_RNAV");
        const nav::SourceCheckResult r = nav::evaluateSourceCheck(navSourceSnap(kLnav, ApproachKind::None, ""), true);
        CHECK(r.code == "NAV_ON_RNAV" && r.banner == "RNAV APPROACH ACTIVE - SELECT GPS" && r.chip == "CDI NAV>GPS");
        CHECK(checkCode(navSourceSnap(kLpv, ApproachKind::Ils, "02")) == "NAV_ON_RNAV");  // ILS to another runway
        // Look-alike: the ILS to the same runway, GPS approach active as backup.
        CHECK(checkCode(navSourceSnap(kLpv, ApproachKind::Ils, "20")) == "OK");
        CHECK(checkCode(navSourceSnap(kLpv, ApproachKind::Localizer, "20")) == "OK");
        CHECK(checkCode(navSourceSnap(kLpv, ApproachKind::Ils, "")) == "OK");  // runway unknown: benefit of the doubt
        CHECK(checkCode(navSourceSnap(kTerm, ApproachKind::Vor, "")) == "OK");  // GPS not on an approach
    }

    // --- LOC_NE_APPR: tuned localizer serves a different runway / airport.
    {
        nav::SourceCheckResult r = nav::evaluateSourceCheck(navSourceSnap(kTerm, ApproachKind::Ils, "23"), true);
        CHECK(r.code == "LOC_NE_APPR");
        CHECK(r.banner == "TUNED ILS RWY 23 - LOADED APPR RWY 20" && r.chip == "LOC/APPR RWY");
        r = nav::evaluateSourceCheck(navSourceSnap(kTerm, ApproachKind::Localizer, "20", "KCGI"), true);
        CHECK(r.code == "LOC_NE_APPR" && r.banner == "TUNED LOC KCGI 20 - LOADED APPR KMWA 20");
        SimSnapshot s = navSourceSnap(kTerm, ApproachKind::Ils, "23");
        s.nav.lateralValid = false;  // not receiving it: nothing to compare yet
        CHECK(checkCode(s) == "OK");
        s = navSourceSnap(kTerm, ApproachKind::Ils, "23");
        s.gpsPlan = GpsApproachPlanSnapshot{};
        CHECK(checkCode(s) == "OK");  // nothing loaded
        s = navSourceSnap(kTerm, ApproachKind::Ils, "2");
        s.gpsPlan.runway = "02";
        CHECK(checkCode(s) == "OK");  // "2" and "02" are the same runway
    }

    // --- CRS_NOT_SET: HSI course pointer vs the localizer course.
    {
        SimSnapshot s = navSourceSnap(kTerm, ApproachKind::Ils, "20");
        CHECK(checkCode(s) == "OK");
        s.hsiObsDegMag = 270.0f;
        nav::SourceCheckResult r = nav::evaluateSourceCheck(s, true);
        CHECK(r.code == "CRS_NOT_SET" && r.banner == "SET COURSE 200* - HSI CRS 270*" && r.chip == "SET CRS 200");
        s.hsiObsDegMag = 208.0f;  // within 10 deg
        CHECK(checkCode(s) == "OK");
        s.hsiObsDegMag = 5.0f;
        s.nav.courseDegMag = 355.0f;  // across north: 10 deg apart
        CHECK(checkCode(s) == "OK");
        s.hsiObsValid = false;
        s.hsiObsDegMag = 270.0f;
        CHECK(checkCode(s) == "OK");  // aircraft without an HSI OBS
        // A VOR's OBS is the pilot's choice: never flagged.
        s = navSourceSnap(kTerm, ApproachKind::Vor, "");
        s.hsiObsDegMag = 90.0f;
        CHECK(checkCode(s) == "OK");
        // Priority: a wrong-runway localizer is reported before its course.
        s = navSourceSnap(kTerm, ApproachKind::Ils, "23");
        s.hsiObsDegMag = 90.0f;
        CHECK(checkCode(s) == "LOC_NE_APPR");
    }

    // --- OK / not applicable.
    {
        SimSnapshot s;
        s.simTimeSec = 1.0;
        nav::SourceCheckResult r = nav::evaluateSourceCheck(s, false);
        CHECK(r.status == nav::SourceCheckStatus::NotApplicable && r.chip == "SRC --" && !r.caution());
        r = nav::evaluateSourceCheck(s, true);
        CHECK(r.status == nav::SourceCheckStatus::Ok && r.chip == "SRC OK");
        s.gpsPlan.loaded = true;  // heading mode, but an approach is loaded
        CHECK(nav::evaluateSourceCheck(s, false).status == nav::SourceCheckStatus::Ok);
        CHECK(nav::sourceCheckCodeNumber("OK") == 0 && nav::sourceCheckCodeNumber("GPS_ON_LOC") == 1 &&
              nav::sourceCheckCodeNumber("CRS_NOT_SET") == 5);
        CHECK(nav::SourceCheckResult{}.chip == "SRC --");
    }

    // --- Debounce: 2.0 s to show, 1.5 s to clear.
    {
        nav::SourceChecker chk;
        SimSnapshot bad = sourceSnap(kTerm);
        SimSnapshot good = sourceSnap(kLpv);
        double t = 10.0;
        for (; t < 10.0 + 1.9; t += 0.05) {
            bad.simTimeSec = t;
            CHECK(!chk.update(bad, true).caution());
        }
        for (; t < 10.0 + 2.1; t += 0.05) {
            bad.simTimeSec = t;
            chk.update(bad, true);
        }
        CHECK(chk.result().code == "APPR_NOT_ACTIVE");
        const double cleared = t;
        for (; t < cleared + 1.4; t += 0.05) {
            good.simTimeSec = t;
            CHECK(chk.update(good, true).caution());  // lingers
        }
        for (; t < cleared + 1.6; t += 0.05) {
            good.simTimeSec = t;
            chk.update(good, true);
        }
        CHECK(chk.result().code == "OK" && chk.result().status == nav::SourceCheckStatus::Ok);
        // A blip shorter than 2 s never shows.
        for (double end = t + 1.0; t < end; t += 0.05) {
            bad.simTimeSec = t;
            CHECK(!chk.update(bad, true).caution());
        }
        good.simTimeSec = t;
        chk.update(good, true);
        for (double end = t + 1.5; t < end; t += 0.05) {
            bad.simTimeSec = t;
            CHECK(!chk.update(bad, true).caution());  // timer restarted after the gap
        }
        // Sim reload (time goes backwards): starts over.
        bad.simTimeSec = 1.0;
        CHECK(!chk.update(bad, true).caution());
        // One caution replaced by another after confirmation.
        nav::SourceChecker chk2;
        SimSnapshot a = sourceSnap(kTerm);
        SimSnapshot b = sourceSnap(kLnav);
        b.radios[0] = liveIls("20");
        for (t = 0.0; t < 2.5; t += 0.05) a.simTimeSec = t, chk2.update(a, true);
        CHECK(chk2.result().code == "APPR_NOT_ACTIVE");
        for (; t < 3.0; t += 0.05) b.simTimeSec = t, chk2.update(b, true);
        CHECK(chk2.result().code == "APPR_NOT_ACTIVE");  // new finding not confirmed yet
        for (; t < 5.0; t += 0.05) b.simTimeSec = t, chk2.update(b, true);
        CHECK(chk2.result().code == "GPS_ON_LOC");
        // The caution text follows the live numbers.
        nav::SourceChecker chk3;
        SimSnapshot c = navSourceSnap(kTerm, ApproachKind::Ils, "20");
        c.hsiObsDegMag = 270.0f;
        for (t = 0.0; t < 2.5; t += 0.05) c.simTimeSec = t, chk3.update(c, true);
        CHECK(chk3.result().chip == "SET CRS 200");
        c.hsiObsDegMag = 250.0f;
        c.simTimeSec = t;
        chk3.update(c, true);
        CHECK(chk3.result().banner == "SET COURSE 200* - HSI CRS 250*");
    }

    // --- Model: runs on the real data; the runway override doesn't hide it.
    {
        InstrumentModel m;
        SimSnapshot s = ilsSnapshot(0.2f, 0.1f, 0.0);
        s.hsiObsValid = true;
        s.hsiObsDegMag = 218.0f;
        for (double t = 0.0; t < 3.0; t += 0.05) s.simTimeSec = t, m.update(s);
        CHECK(m.state().sourceCheck.status == nav::SourceCheckStatus::Ok);
        s.hsiObsDegMag = 300.0f;
        for (double t = 3.0; t < 6.0; t += 0.05) s.simTimeSec = t, m.update(s);
        CHECK(m.state().sourceCheck.code == "CRS_NOT_SET");
        CHECK(m.setRunwayOverride("23"));
        s.simTimeSec = 6.1;
        m.update(s);
        CHECK(m.state().sourceCheck.code == "CRS_NOT_SET");
        m.reset();
        CHECK(m.state().sourceCheck.status == nav::SourceCheckStatus::NotApplicable);
    }

    // --- Display: chip + banner, inside the viewport in every layout.
    {
        const render::Theme day = render::makeTheme(render::Brightness::Day);
        render::InstrumentRenderer r;
        InstrumentState ok = sampleState(InstrumentMode::RunwayAlignment, 200.0, true);
        CHECK(ok.sourceCheck.status == nav::SourceCheckStatus::Ok);
        InstrumentState bad = ok;
        bad.sourceCheck = nav::evaluateSourceCheck(sourceSnap(kTerm), true);
        bad.runwayCourseMismatch = true;  // the source caution wins the banner slot
        InstrumentState na = sampleState(InstrumentMode::Heading, 200.0, true);
        na.sourceCheck = nav::SourceCheckResult{};
        for (const render::InstrumentRenderer::Viewport& vp :
             {render::InstrumentRenderer::Viewport{0, 0, 600, 600}, render::InstrumentRenderer::Viewport{0, 0, 380, 380},
              render::InstrumentRenderer::Viewport{100, 50, 900, 600}}) {
            for (const InstrumentState* s : {&ok, &bad, &na}) {
                RecordingCanvas c;
                r.draw(c, *s, vp);
                c.flush();
                CHECK(c.nonFinite == 0);
                CHECK(c.minX >= vp.x - 1.0f && c.maxX <= vp.x + vp.width + 1.0f);
                CHECK(c.minY >= vp.y - 1.0f && c.maxY <= vp.y + vp.height + 1.0f);
            }
            ColorCountCanvas amberOk(day.kAmber), amberBad(day.kAmber);
            r.draw(amberOk, ok, vp);
            amberOk.flush();
            r.draw(amberBad, bad, vp);
            amberBad.flush();
            CHECK(amberBad.count > amberOk.count + 100);  // amber chip text/border + banner
        }
        ColorCountCanvas greenOk(day.kGreen), greenNa(day.kGreen);
        r.draw(greenOk, ok, {0, 0, 600, 600});
        greenOk.flush();
        r.draw(greenNa, na, {0, 0, 600, 600});
        greenNa.flush();
        CHECK(greenOk.count > greenNa.count);  // green SRC OK chip
        CHECK(render::measureText(">", 10.0f) > 0.0f);
    }
}

}  // namespace

int main() {
    log::setSink([](const char*) {});  // keep test output clean
    testAngles();
    testModel();
    testConfig();
    testFont();
    testAxisGuidance();
    testApproachNaming();
    testNavValidator();
    testNavModel();
    testGpsApproachAndDistance();
    testTrajectoryPredictor();
    testNavLog();
    testGuidanceText();
    testRenderer();
    testCollapsiblePanels();
    testNeedleColourBySource();
    testRunwayOverride();
    testRunwayPicker();
    testPolish();
    testSourceCheck();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
