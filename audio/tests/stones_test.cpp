// Tests for the telemetry-driven stone spray. Self-contained; build and run from this folder:
//   g++ -std=c++17 -O1 -I.. stones_test.cpp -o stones_test && ./stones_test
#include <cstdio>
#include <functional>
#include <random>
#include "core.hpp"

using namespace rsa;

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

struct FakeSink : IAudioSink {
    int stones = 0;
    float lastStoneGain = 0, maxStoneGain = 0;
    void setWheelLoop(int, int, float, float, float) override {}
    void setSlideLoop(int, int, float, float, float) override {}
    void setSquealLoop(int, int, float, float, float) override {}
    void setRumble(float, float) override {}
    void trigger(SoundEvent k, int, float gl, float gr, float) override {
        if (k == SoundEvent::StoneHit) { ++stones; lastStoneGain = gl + gr; maxStoneGain = std::max(maxStoneGain, gl + gr); }
    }
    void silence() override {}
};

// Same frame shape the real game gives: the velocity vector is the ground speed in m/s, the Speed field is the driven-wheel speedometer.
static TelemetryFrame gframe(double t, double groundMs, double drivenMs, float strut, float throttle) {
    TelemetryFrame f; f.time = t; f.speed = (float)(drivenMs * 3.6 * 1.609); f.throttle = throttle; f.velSurge = (float)groundMs;
    for (auto& w : f.wheels) w.strutForce = strut;
    return f;
}

// Drives for `seconds` at a steady speed. rough = 0 is a smooth strut force (tarmac); rough > 0 adds noise (loose ground).
static int run(double groundMs, double drivenMs, float throttle, float rough, double seconds, Settings st = Settings(), uint32_t seed = 1) {
    FakeSink sink; SurfaceAudioController c(st, sink, seed); double t = 0;
    std::mt19937 g(9); std::normal_distribution<float> n(0, 1);
    for (int i = 0, N = (int)(seconds * 60); i < N; ++i) { t += 1.0 / 60; c.onFrame(gframe(t, groundMs, drivenMs, -3500.f * (1.f + rough * n(g)), throttle)); }
    return sink.stones;
}

TEST(loose_ground_throws_stones_at_low_speed_too) {
    // 14 km/h on rough ground: the old fixed 25 km/h cut-off made this silent
    CHECK(run(4.0, 4.0, 0.5f, 0.5f, 15.0) > 0);
}
TEST(clean_smooth_tarmac_throws_no_stones_at_any_speed) {
    CHECK(run(30.0, 30.0, 0.5f, 0.0f, 15.0) == 0);
    CHECK(run(8.0, 8.0, 0.5f, 0.0f, 15.0) == 0);
}
TEST(stone_rate_grows_smoothly_with_speed) {
    int slow = run(5.0, 5.0, 0.5f, 0.5f, 30.0), mid = run(15.0, 15.0, 0.5f, 0.5f, 30.0), fast = run(30.0, 30.0, 0.5f, 0.5f, 30.0);
    CHECK(slow > 0 && slow < mid && mid < fast);
}
TEST(stones_are_quieter_at_low_surface_speed) {
    FakeSink slow, fast; Settings st;
    SurfaceAudioController a(st, slow, 3), b(st, fast, 3); double t = 0;
    std::mt19937 g(9); std::normal_distribution<float> n(0, 1);
    for (int i = 0; i < 1800; ++i) {
        t += 1.0 / 60; float strut = -3500.f * (1.f + 0.5f * n(g));
        a.onFrame(gframe(t, 4.0, 4.0, strut, 0.5f)); b.onFrame(gframe(t, 30.0, 30.0, strut, 0.5f));
    }
    CHECK(slow.stones > 0 && fast.stones > 0);
    CHECK(slow.maxStoneGain < fast.maxStoneGain);
}
TEST(standing_wheelspin_still_throws_stones) {
    // car nearly stationary, driven wheels at 14 m/s: the spinning tyre flings material from a standing start
    CHECK(run(1.0, 14.0, 1.0f, 0.5f, 15.0) > 0);
}
TEST(standing_still_with_no_wheelspin_throws_nothing) {
    CHECK(run(0.0, 0.0, 0.0f, 0.5f, 10.0) == 0);
}
TEST(stones_can_be_switched_off_and_the_rate_scales_with_the_setting) {
    Settings off; off.enableStones = false;
    CHECK(run(25.0, 25.0, 0.5f, 0.5f, 15.0, off) == 0);
    Settings none; none.stoneRate = 0.0;
    CHECK(run(25.0, 25.0, 0.5f, 0.5f, 15.0, none) == 0);
    Settings few, many; few.stoneRate = 3.0; many.stoneRate = 20.0;
    CHECK(run(25.0, 25.0, 0.5f, 0.5f, 30.0, few) < run(25.0, 25.0, 0.5f, 0.5f, 30.0, many));
}
TEST(no_per_surface_setting_is_involved) {
    // the same telemetry gives the same stones whatever the sound names say: nothing in Settings maps a surface to a stone level
    Settings a, b; b.surfaceNames[3] = "ice"; b.classVolume[3] = 0.1f;
    CHECK(run(20.0, 20.0, 0.5f, 0.5f, 15.0, a, 5) == run(20.0, 20.0, 0.5f, 0.5f, 15.0, b, 5));
}

int main() {
    for (const Reg& r : regs()) { int before = g_fail; r.fn(); std::printf("[%s] %s\n", g_fail == before ? " ok " : "FAIL", r.name); }
    std::printf("\n%zu tests, %d checks, %d failures\n", regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
