// Tests for crash detection and the crash sounds (crash.hpp, and the events through the controller). Self-contained; build and run from this folder:
//   g++ -std=c++17 -O1 -I.. crash_test.cpp -o crash_test && ./crash_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>
#include "core.hpp"
#include "audio.hpp"

using namespace ffx;
static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)

constexpr double G = crash::kG;
constexpr double dt = 1.0 / 60.0;

// Drives a detector at 60 Hz. Each step gives surge and sway in g and the speed in m/s. Returns the first hit (or a miss), and how many hits there were.
struct Drive {
    crash::Detector d; crash::Result first; int hits = 0; double t = 0;
    void step(double surgeG, double swayG, double speed, bool suppress = false, const crash::Params& p = crash::Params()) {
        const crash::Result r = d.update(dt, surgeG * G, swayG * G, speed, suppress, p); t += dt;
        if (r.hit) { if (!hits) first = r; ++hits; }
    }
    void cruise(double seconds, double speed, double surgeG = 0, double swayG = 0) { for (int i = 0; i < (int)(seconds * 60); ++i) step(surgeG, swayG, speed); }
};

TEST(driving_hard_is_not_a_crash) {
    Drive d;
    d.cruise(1.0, 25);
    for (int i = 0; i < 90; ++i) d.step(-1.3, 0.0, 25 - i * 0.1);          // hard braking, 1.3 g
    for (int i = 0; i < 90; ++i) d.step(0.0, 1.6, 20);                      // hard cornering, 1.6 g
    CHECK(d.hits == 0);
}

TEST(a_slow_build_up_to_a_big_number_is_not_a_crash) {
    Drive d;
    d.cruise(1.0, 25);
    for (int i = 0; i < 120; ++i) d.step(-0.05 * i / 6.0, 0.0, 25);         // ramps to 1 g over two seconds
    CHECK(d.hits == 0);
}

TEST(head_on_into_a_tree_is_a_heavy_frontal_crash) {
    Drive d;
    d.cruise(1.0, 22);
    d.step(-14, 0.2, 14); d.step(-9, 0, 6); d.step(-3, 0, 2); d.step(0, 0, 0.5);
    CHECK(d.hits == 1);
    CHECK(d.first.hit && d.first.frontal);
    CHECK(d.first.peakG > 12.0 && d.first.severity > 0.9);
}

TEST(a_very_hard_spike_counts_at_once_without_waiting_for_the_speed) {
    Drive d;
    d.cruise(1.0, 20);
    d.step(-9, 0, 20);                                                       // the speed field has not moved yet
    CHECK(d.hits == 1);
}

TEST(a_moderate_spike_needs_the_speed_to_drop) {
    Drive kerb;
    kerb.cruise(1.0, 20);
    kerb.step(-4, 0, 20); kerb.step(-3.5, 0, 20); kerb.cruise(0.5, 20);
    CHECK(kerb.hits == 0);                                                   // a jolt, the speed did not change
    Drive wall;
    wall.cruise(1.0, 20);
    wall.step(-4, 0, 19.5); wall.step(-3.5, 0, 17); wall.step(-1, 0, 15); wall.cruise(0.3, 15);
    CHECK(wall.hits == 1);
    CHECK(wall.first.severity > 0.0 && wall.first.severity < 0.5);
}

TEST(a_hit_from_the_side_is_not_frontal) {
    Drive d;
    d.cruise(1.0, 20);
    d.step(0.5, 10, 15); d.step(0, 6, 12); d.cruise(0.2, 12);
    CHECK(d.hits == 1 && !d.first.frontal);
}

TEST(harder_is_more_severe) {
    double sev[3];
    const double g[3] = {4.5, 8.0, 14.0}, lost[3] = {2.5, 7.0, 16.0};     // the harder ones also lose more speed
    for (int k = 0; k < 3; ++k) { Drive d; d.cruise(1.0, 25); d.step(-g[k], 0, 25 - lost[k] * 0.5); d.step(-g[k] * 0.5, 0, 25 - lost[k]); d.cruise(0.3, 25 - lost[k]); sev[k] = d.first.severity; CHECK(d.hits == 1); }
    CHECK(sev[0] < sev[1] && sev[1] < sev[2] && sev[2] <= 1.0 && sev[0] >= 0.0);
}

TEST(the_return_to_normal_after_a_suppressed_spike_is_not_a_crash) {
    Drive d;
    d.cruise(1.0, 25);
    for (int i = 0; i < 40; ++i) d.step(-15, 0, 25 - i * 0.3, true);       // a violent landing shakes the body for two thirds of a second
    d.cruise(1.0, 13);
    CHECK(d.hits == 0);
}

TEST(one_crash_is_one_event_and_the_next_one_counts_later) {
    Drive d;
    d.cruise(1.0, 25);
    d.step(-12, 0, 10);
    for (int i = 0; i < 20; ++i) d.step(-8, 0, 3);                           // scraping on: no machine gun
    CHECK(d.hits == 1);
    d.cruise(1.5, 6);
    d.step(-12, 0, 2);
    CHECK(d.hits == 2);
}

TEST(nothing_counts_in_the_air_or_just_after_landing) {
    Drive d;
    d.cruise(1.0, 25);
    for (int i = 0; i < 10; ++i) d.step(-15, 0, 10, true);
    CHECK(d.hits == 0);
    d.cruise(0.3, 10);
    CHECK(d.hits == 0);                                                      // the spike was ignored, not saved up for later
}

TEST(too_slow_is_never_a_crash) {
    Drive d;
    d.cruise(1.0, 1.0);
    d.step(-12, 0, 0.0);
    CHECK(d.hits == 0);
}

TEST(the_settings_move_the_threshold) {
    crash::Params strict; strict.minG = 6.0; strict.strongG = 14.0;
    Drive a; a.cruise(1.0, 20); a.step(-5, 0, 16); a.step(-4, 0, 13); a.cruise(0.2, 12);
    Drive b;
    b.cruise(1.0, 20);
    b.step(-5, 0, 16, false, strict); b.step(-4, 0, 13, false, strict); for (int i = 0; i < 12; ++i) b.step(0, 0, 12, false, strict);
    CHECK(a.hits == 1 && b.hits == 0);
}

TEST(bad_numbers_do_not_break_it) {
    crash::Detector d;
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 30; ++i) {
        const crash::Result r = d.update(i % 3 ? 0.0 : nan, i % 2 ? nan : inf, -inf, i % 5 ? nan : -3.0, false);
        CHECK(std::isfinite(r.severity) && std::isfinite(r.peakG));
    }
    d.update(dt, 0, 0, 20, false);
    CHECK(true);
}

TEST(layers_grow_with_severity) {
    const crash::Layers a = crash::PlanLayers(0.10, true, 0.1, 0.1), b = crash::PlanLayers(0.45, true, 0.9, 0.1), c = crash::PlanLayers(0.95, true, 0.1, 0.1);
    CHECK(a.thump == 0 && !a.crumple && !a.glass && !a.metal);
    CHECK(b.thump == 1 && b.metal && b.crumple && !b.branch && !b.glass);   // branch roll lost
    CHECK(c.thump == 2 && c.metal && c.crumple && !c.branch && c.glass);    // too big for a branch
    CHECK(crash::PlanLayers(0.3, true, 0.1, 0.9).branch);                   // nose first, moderate, lucky roll: twigs
    CHECK(!crash::PlanLayers(0.3, false, 0.1, 0.9).branch);                 // from the side: no
    CHECK(!crash::PlanLayers(0.3, true, 0.9, 0.9).branch);
    CHECK(crash::PlanLayers(std::numeric_limits<double>::quiet_NaN(), true, 0, 0).thump == 0);
    CHECK(crash::PlanLayers(5.0, true, 0, 0).thump == 2);
}

// ------------------------------------------------------------------ through the controller
struct CrashSink : IAudioSink {
    int light = 0, medium = 0, heavy = 0, metal = 0, crumple = 0, branch = 0, glass = 0, other = 0;
    void setWheelLoop(int, int, float, float, float) override {}
    void setSlideLoop(int, int, float, float, float) override {}
    void setSquealLoop(int, int, float, float, float) override {}
    void setRumble(float, float) override {}
    void trigger(SoundEvent k, int, float, float, float) override {
        switch (k) {
            case SoundEvent::CrashLight: ++light; break; case SoundEvent::CrashMedium: ++medium; break; case SoundEvent::CrashHeavy: ++heavy; break;
            case SoundEvent::CrashMetal: ++metal; break; case SoundEvent::CrashCrumple: ++crumple; break; case SoundEvent::CrashBranch: ++branch; break;
            case SoundEvent::CrashGlass: ++glass; break; default: ++other; break;
        }
    }
    void silence() override {}
    int thumps() const { return light + medium + heavy; }
};
static TelemetryFrame fr(double t, double speedMs, double surgeMs2) {
    TelemetryFrame f; f.time = t; f.speed = (float)(speedMs * 3.6 * 1.609); f.velSurge = (float)speedMs; f.accSurge = (float)surgeMs2; f.throttle = 0.5f;
    for (auto& w : f.wheels) w.strutForce = -3500.f;
    return f;
}
static int driveCrash(Settings s, double spikeMs2, CrashSink& sink) {
    AudioController c(s, sink, 1);
    double t = 0, v = 25;
    for (int i = 0; i < 120; ++i) { t += dt; c.onFrame(fr(t, v, 0)); }
    t += dt; v = 14; c.onFrame(fr(t, v, spikeMs2));
    t += dt; v = 6; c.onFrame(fr(t, v, spikeMs2 * 0.5));
    for (int i = 0; i < 30; ++i) { t += dt; v = 2; c.onFrame(fr(t, v, 0)); }
    return sink.thumps();
}

TEST(a_crash_plays_a_thump_and_layers_through_the_controller) {
    CrashSink sink; Settings s;
    CHECK(driveCrash(s, -12 * G, sink) == 1);
    CHECK(sink.heavy == 1 && sink.metal == 1 && sink.crumple == 1);
}

TEST(the_controller_counts_crashes_for_the_log) {
    CrashSink sink; Settings s;
    AudioController c(s, sink, 1);
    double t = 0;
    for (int i = 0; i < 120; ++i) { t += dt; c.onFrame(fr(t, 25, 0)); }
    CHECK(c.debris.crashCount == 0);
    t += dt; c.onFrame(fr(t, 14, -12 * G)); t += dt; c.onFrame(fr(t, 6, -6 * G));
    for (int i = 0; i < 30; ++i) { t += dt; c.onFrame(fr(t, 2, 0)); }
    CHECK(c.debris.crashCount == 1 && c.debris.crashLastG > 10.0 && c.debris.crashLastSeverity > 0.8);
    CHECK(c.analyzer.speedMs >= 0.0);
}

TEST(a_bump_or_hard_braking_plays_no_crash) {
    CrashSink sink; Settings s;
    CHECK(driveCrash(s, -1.2 * G, sink) == 0);
    CHECK(sink.metal == 0 && sink.glass == 0 && sink.branch == 0);
}

TEST(crashes_can_be_switched_off_and_tuned) {
    CrashSink off; Settings s; s.enableCrashes = false;
    CHECK(driveCrash(s, -12 * G, off) == 0);
    CrashSink strict; Settings t; t.crashMinG = 12.0;
    CHECK(driveCrash(t, -6 * G, strict) == 0);
    CHECK(Settings().enableCrashes && Settings().crashMinG == 3.0 && Settings().crashLevel == 1.0);
}

TEST(settings_are_read_from_the_ini_text_and_limited) {
    Settings s;
    applySetting(s, "EnableCrashes", "0"); CHECK(!s.enableCrashes);
    applySetting(s, "EnableCrashes", "1"); CHECK(s.enableCrashes);
    applySetting(s, "CrashMinG", "5"); CHECK(s.crashMinG == 5.0);
    applySetting(s, "CrashMinG", "0.1"); CHECK(s.crashMinG == 1.0);       // limited: below 1 g ordinary driving would count
    applySetting(s, "CrashMinG", "99"); CHECK(s.crashMinG == 12.0);
    applySetting(s, "CrashLevel", "2.5"); CHECK(s.crashLevel == 2.5);
    applySetting(s, "CrashLevel", "-3"); CHECK(s.crashLevel == 0.0);
    applySetting(s, "CrashLevel", "50"); CHECK(s.crashLevel == 6.0);
}

// ------------------------------------------------------------------ the sounds
TEST(crash_events_find_their_sounds_and_fall_back) {
    Settings s; MaterialTable table;
    SoundBank bank;
    bank.add("debris_impact", std::vector<float>(2000, 0.2f)); bank.add("debris_landing", std::vector<float>(2000, 0.2f));
    SoundSetMap map(bank, table, s); Mixer mixer; MixerSink sink(mixer, bank, s, &map);
    sink.trigger(SoundEvent::CrashLight, 0, 1, 1, 1); CHECK(mixer.activeShots() == 1);     // no crash_light: the bump sound stands in
    sink.trigger(SoundEvent::CrashHeavy, 0, 1, 1, 1); CHECK(mixer.activeShots() == 2);     // no crash_heavy: the landing sound
    sink.trigger(SoundEvent::CrashMetal, 0, 1, 1, 1); CHECK(mixer.activeShots() == 2);     // layers have no stand-in: silent
    sink.trigger(SoundEvent::CrashBranch, 0, 1, 1, 1); sink.trigger(SoundEvent::CrashGlass, 0, 1, 1, 1); sink.trigger(SoundEvent::CrashCrumple, 0, 1, 1, 1);
    CHECK(mixer.activeShots() == 2);
    bank.add("crash_metal", std::vector<float>(2000, 0.2f)); bank.add("crash_branch", std::vector<float>(2000, 0.2f));
    bank.add("crash_crumple", std::vector<float>(2000, 0.2f)); bank.add("crash_glass", std::vector<float>(2000, 0.2f));
    sink.trigger(SoundEvent::CrashMetal, 0, 1, 1, 1); sink.trigger(SoundEvent::CrashBranch, 0, 1, 1, 1);
    sink.trigger(SoundEvent::CrashGlass, 0, 1, 1, 1); sink.trigger(SoundEvent::CrashCrumple, 0, 1, 1, 1);
    CHECK(mixer.activeShots() == 6);
}

TEST(own_crash_sounds_win_and_the_level_scales_them) {
    Settings s; MaterialTable table;
    SoundBank bank;
    bank.add("crash_heavy", std::vector<float>(2000, 0.5f));
    SoundSetMap map(bank, table, s); Mixer mixer; MixerSink sink(mixer, bank, s, &map);
    sink.trigger(SoundEvent::CrashHeavy, 0, 1, 1, 1); CHECK(mixer.activeShots() == 1);
    Settings quiet; quiet.crashLevel = 0.0;
    SoundSetMap map2(bank, table, quiet); Mixer m2; MixerSink sink2(m2, bank, quiet, &map2);
    sink2.trigger(SoundEvent::CrashHeavy, 0, 1, 1, 1);
    CHECK(m2.activeShots() == 1);                                                           // still started, at zero gain (a level of 0 mutes, it does not skip)
}

int main() {
    for (const Reg& r : regs()) { const int before = g_fail; r.fn(); std::printf("%s %s\n", g_fail == before ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d tests, %d checks, %d failures\n", (int)regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
