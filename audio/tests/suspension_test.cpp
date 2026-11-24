// Tests for the suspension creaks and bottoming-out thumps: rare events that follow the telemetry, never a stream.
//   g++ -std=c++17 -O1 -I.. suspension_test.cpp -o suspension_test && ./suspension_test
#include <cstdio>
#include <functional>
#include <random>
#include <vector>
#include "core.hpp"

using namespace ffx;

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)

struct Ev { SoundEvent kind; double t; float gl, gr; };
struct FakeSink : IAudioSink {
    double now = 0; std::vector<Ev> ev;
    void setWheelLoop(int, int, float, float, float) override {}
    void setSlideLoop(int, int, float, float, float) override {}
    void setSquealLoop(int, int, float, float, float) override {}
    void setRumble(float, float) override {}
    void trigger(SoundEvent k, int, float gl, float gr, float) override { ev.push_back({k, now, gl, gr}); }
    void silence() override {}
    int count(SoundEvent k, double from = 0, double to = 1e9) const { int n = 0; for (const Ev& e : ev) if (e.kind == k && e.t >= from && e.t < to) ++n; return n; }
    std::vector<double> times(SoundEvent k) const { std::vector<double> v; for (const Ev& e : ev) if (e.kind == k) v.push_back(e.t); return v; }
};

// A drive at a steady speed. load(w, t) is the strut force of each tyre and piston(w, t) its damper piston velocity.
struct Drive {
    Settings st; FakeSink sink; AudioController c; double t = 0; double speed;
    explicit Drive(double speedMs = 25.0, Settings s = Settings()) : st(s), c(st, sink, 7), speed(speedMs) {}
    void run(double seconds, const std::function<float(int, double)>& load, const std::function<float(int, double)>& piston) {
        for (int i = 0, n = (int)(seconds * 60); i < n; ++i) {
            t += 1.0 / 60; sink.now = t;
            TelemetryFrame f; f.time = t; f.speed = (float)(speed * 3.6 / 1.609 * 1.609); f.velSurge = (float)speed; f.throttle = 0.5f;
            for (int w = 0; w < 4; ++w) { f.wheels[w].strutForce = -load(w, t); f.wheels[w].pistonVelocity = piston(w, t); }
            c.onFrame(f);
        }
    }
};
static float steady(int, double) { return 3500.0f; }
static float quiet(int, double) { return 0.0f; }

TEST(smooth_driving_makes_no_creaks_or_thumps) {
    Drive d; std::mt19937 g(1); std::normal_distribution<float> n(0, 1);
    d.run(90.0, [&](int, double) { return 3500.0f * (1.0f + 0.01f * n(g)); }, [&](int, double) { return 0.005f * n(g); });
    CHECK(d.sink.count(SoundEvent::SuspSqueak) == 0 && d.sink.count(SoundEvent::SuspBottom) == 0);
}
TEST(rough_ground_with_fast_strokes_makes_no_creaks_and_no_thumps) {
    Drive d; std::mt19937 g(2); std::normal_distribution<float> n(0, 1);
    d.run(12.0, steady, quiet);                                                       // learn the normal load
    d.run(90.0, [&](int, double) { return 3500.0f * (1.0f + 0.5f * n(g)); }, [&](int, double) { return 0.5f * n(g); });
    CHECK(d.sink.count(SoundEvent::SuspSqueak) == 0);                                   // strokes are far too fast to creak
    CHECK(d.sink.count(SoundEvent::SuspBottom) <= 1);                                   // constant roughness is not a bump-stop hit
}
TEST(a_hard_corner_makes_a_few_well_spaced_creaks) {
    Drive d;
    d.run(12.0, steady, quiet);
    const double t0 = d.t;
    // weight moves to the outside tyres for 10 s with the body rolling slowly at first
    d.run(10.0, [](int w, double) { return (w % 2 == 0) ? 4900.0f : 2100.0f; }, [&](int, double t) { return t - t0 < 4.0 ? 0.08f : 0.0f; });
    const int n = d.sink.count(SoundEvent::SuspSqueak);
    CHECK(n >= 1 && n <= 6);
    const std::vector<double> ts = d.sink.times(SoundEvent::SuspSqueak);
    for (size_t i = 1; i < ts.size(); ++i) CHECK(ts[i] - ts[i - 1] >= 1.4);             // never a stream
}
TEST(a_corner_that_never_loads_the_suspension_makes_no_creak) {
    Drive d; d.run(12.0, steady, quiet);
    d.run(30.0, [](int w, double) { return (w % 2 == 0) ? 3800.0f : 3200.0f; }, [](int, double) { return 0.08f; });   // 9% shift only
    CHECK(d.sink.count(SoundEvent::SuspSqueak) == 0);
}
TEST(slow_stroke_without_weight_transfer_makes_no_creak) {
    Drive d; d.run(12.0, steady, quiet);
    d.run(30.0, steady, [](int, double) { return 0.08f; });                              // moving, but carrying its normal load
    CHECK(d.sink.count(SoundEvent::SuspSqueak) == 0);
}
TEST(crawling_makes_no_creak) {
    Drive d(0.5); d.run(12.0, steady, quiet);
    d.run(20.0, [](int w, double) { return (w % 2 == 0) ? 4900.0f : 2100.0f; }, [](int, double) { return 0.08f; });
    CHECK(d.sink.count(SoundEvent::SuspSqueak) == 0);
}
TEST(one_big_hit_is_one_thump_and_replaces_the_impact_sound) {
    Drive d; d.run(12.0, steady, quiet);
    const double t0 = d.t;
    d.run(0.05, [](int w, double) { return w == 0 ? 14000.0f : 3500.0f; }, [](int w, double) { return w == 0 ? 3.0f : 0.0f; });   // 4x load, fast compression
    d.run(2.0, steady, quiet);
    CHECK(d.sink.count(SoundEvent::SuspBottom, t0, 1e9) == 1);
    CHECK(d.sink.count(SoundEvent::Impact, t0, t0 + 0.5) == 0);                          // the thump stands in for it
}
TEST(thumps_are_spaced_and_never_stack_up) {
    Drive d; d.run(12.0, steady, quiet);
    for (int k = 0; k < 6; ++k) {                                                        // six hits 0.1 s apart
        d.run(0.03, [](int, double) { return 14000.0f; }, [](int, double) { return 3.0f; });
        d.run(0.07, steady, quiet);
    }
    CHECK(d.sink.count(SoundEvent::SuspBottom) <= 2);
    const std::vector<double> ts = d.sink.times(SoundEvent::SuspBottom);
    for (size_t i = 1; i < ts.size(); ++i) CHECK(ts[i] - ts[i - 1] >= 0.45);
}
TEST(a_moderate_bump_is_not_a_bottom_out) {
    Drive d; d.run(12.0, steady, quiet);
    d.run(0.05, [](int w, double) { return w == 0 ? 8750.0f : 3500.0f; }, [](int w, double) { return w == 0 ? 3.0f : 0.0f; });   // 2.5x: a hard bump, not the stop
    d.run(2.0, steady, quiet);
    CHECK(d.sink.count(SoundEvent::SuspBottom) == 0);
}
TEST(a_load_spike_without_a_fast_stroke_is_not_a_hit) {
    Drive d; d.run(12.0, steady, quiet);
    d.run(0.05, [](int w, double) { return w == 0 ? 14000.0f : 3500.0f; }, [](int, double) { return 0.02f; });
    d.run(2.0, steady, quiet);
    CHECK(d.sink.count(SoundEvent::SuspBottom) == 0);
}
TEST(switching_the_suspension_sounds_off_silences_both) {
    Settings s; s.enableSuspension = false; Drive d(25.0, s); d.run(12.0, steady, quiet);
    d.run(0.05, [](int w, double) { return w == 0 ? 14000.0f : 3500.0f; }, [](int w, double) { return w == 0 ? 3.0f : 0.0f; });
    d.run(10.0, [](int w, double) { return (w % 2 == 0) ? 4900.0f : 2100.0f; }, [](int, double) { return 0.08f; });
    CHECK(d.sink.count(SoundEvent::SuspBottom) == 0 && d.sink.count(SoundEvent::SuspSqueak) == 0);
}
TEST(there_are_no_rate_settings_and_old_keys_are_ignored) {
    std::string p = "/tmp/susp_legacy.ini";
    { FILE* f = fopen(p.c_str(), "w"); fputs("SqueakSensitivity = 4\nBottomOutRatio = 1.2\nSqueakLevel = 0.9\n", f); fclose(f); }
    Settings s; CHECK(loadSettings(p, s)); CHECK(s.squeakLevel > 0.89 && s.squeakLevel < 0.91);
    Drive a(25.0, s), b; a.run(12.0, steady, quiet); b.run(12.0, steady, quiet);
    auto corner = [](Drive& d) { d.run(10.0, [](int w, double) { return (w % 2 == 0) ? 4900.0f : 2100.0f; }, [](int, double) { return 0.08f; }); };
    corner(a); corner(b);
    CHECK(a.sink.count(SoundEvent::SuspSqueak) == b.sink.count(SoundEvent::SuspSqueak));
    remove(p.c_str());
}
TEST(default_levels_keep_them_under_the_loops) {
    Settings s; CHECK(s.squeakLevel <= 0.7 && s.bottomLevel <= 1.0);
}

int main() {
    for (const Reg& r : regs()) { int before = g_fail; r.fn(); std::printf("[%s] %s\n", g_fail == before ? " ok " : "FAIL", r.name); }
    std::printf("\n%zu tests, %d checks, %d failures\n", regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
