// Tests for the weight transfer scalings (weight_transfer.hpp). Self-contained:
//   g++ -std=c++17 -O1 weight_transfer_test.cpp -o weight_transfer_test && ./weight_transfer_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>
#include "../weight_transfer.hpp"

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)

static const float kNaN = std::numeric_limits<float>::quiet_NaN();
using namespace WeightTransfer;

// A car on a test track at 100 Hz. The side load is given as a function of time; the lateral acceleration follows it (0.4 of side load is about 0.7 g).
struct Rig {
    Lateral t; float time = 0, speed = 20.0f, air = 0.0f; bool latOk = true; unsigned seed = 1;
    float noise() { seed = seed * 1664525u + 1013904223u; return (float)((seed >> 8) & 0xFFFF) / 32767.5f - 1.0f; }
    // Runs for `secs`; fn(time since the call began) gives the side load. Returns the last output, and the largest swell and surge on the way.
    Out run(const std::function<float(float)>& fn, float secs, float* maxSwell = nullptr, float* maxSurge = nullptr, float noiseAmp = 0.0f, float latGOverride = -1.0f) {
        Out o; float ms = 0, mg = 0; const float t0 = time;
        for (int i = 0; i < (int)(secs * 100 + 0.5f); ++i) {
            const float v = fn(time - t0);
            In in; in.lateral = v + noiseAmp * noise(); in.speed = speed; in.airborne = air; in.latOk = latOk;
            in.latG = latGOverride >= 0.0f ? latGOverride : std::fabs(v) * 1.8f;
            o = t.Update(in, 0.01f); time += 0.01f;
            ms = std::max(ms, o.swell); mg = std::max(mg, o.surge);
        }
        if (maxSwell) *maxSwell = ms;
        if (maxSurge) *maxSurge = mg;
        return o;
    }
};

// ---- front load: one multiplier for both directions
TEST(the_front_load_is_light_when_unloaded_and_heavy_when_loaded) {
    AccelLight::Out lightOut; lightOut.drop = 0.3f;
    AccelLight::Out heavyOut; heavyOut.heavy = 1.0f;
    CHECK(FrontMul(lightOut, 1.0f, 1.0f) < 0.75f);
    CHECK(FrontMul(heavyOut, 1.0f, 1.0f) > 1.25f);
    CHECK(FrontMul(AccelLight::Out(), 1.0f, 1.0f) == 1.0f);                 // nothing moving: exactly RBR's force
}
TEST(the_front_load_gains_switch_each_direction_off) {
    AccelLight::Out lightOut; lightOut.drop = 0.3f;
    AccelLight::Out heavyOut; heavyOut.heavy = 1.0f;
    CHECK(FrontMul(lightOut, 0.0f, 1.0f) == 1.0f);
    CHECK(FrontMul(heavyOut, 1.0f, 0.0f) == 1.0f);
    CHECK(std::fabs(FrontMul(heavyOut, 1.0f, 1.0f) - BrakeFx::HeavyMul(1.0f, 1.0f)) < 1e-6f);     // the same numbers as before they were combined
    CHECK(std::fabs(FrontMul(lightOut, 1.0f, 1.0f) - AccelLight::Mul(lightOut, 1.0f)) < 1e-6f);
}

// ---- side load
TEST(a_straight_road_adds_nothing) {
    Rig r; float sw, sg;
    const Out o = r.run([](float) { return 0.0f; }, 3.0f, &sw, &sg);
    CHECK(sw == 0.0f && sg == 0.0f && LateralMul(o, 1.0f) == 1.0f);
}
TEST(a_held_corner_swells_the_force_and_it_eases_off_afterwards) {
    Rig r; r.run([](float) { return 0.0f; }, 1.0f);
    const Out o = r.run([](float) { return 0.4f; }, 2.0f);
    CHECK(o.swell > 0.9f && o.surge < 0.05f);
    const float m = LateralMul(o, 1.0f);
    CHECK(m > 1.2f && m <= 1.0f + kMaxAdd + 1e-6f);
    const Out e = r.run([](float) { return 0.0f; }, 2.0f);
    CHECK(e.swell < 0.02f);
}
TEST(the_swell_comes_in_a_moment_after_the_load_not_at_once) {
    Rig r; r.run([](float) { return 0.0f; }, 1.0f);
    const Out first = r.run([](float) { return 0.4f; }, 0.03f);
    CHECK(first.swell < 0.3f);
}
TEST(a_direction_change_surges_and_a_plain_corner_does_not) {
    Rig corner; corner.run([](float) { return 0.0f; }, 1.0f);
    float sw, sg;
    corner.run([](float) { return 0.4f; }, 2.0f, &sw, &sg);
    CHECK(sg < 0.05f);                                                     // building up and holding a corner is not a surge
    Rig flick; flick.run([](float) { return 0.4f; }, 2.0f);
    float swF, sgF;
    flick.run([](float) { return 0.4f; }, 0.01f);
    flick.run([](float s) { return std::max(-0.4f, 0.4f - 0.8f * s / 0.3f); }, 1.0f, &swF, &sgF);     // the load crosses the car in 0.3 s
    CHECK(sgF > 0.5f);
}
TEST(a_slow_swing_surges_less_than_a_quick_one) {
    auto swing = [](float secs) { Rig r; r.run([](float) { return 0.4f; }, 2.0f); float sg; r.run([secs](float s) { return std::max(-0.4f, 0.4f - 0.8f * s / secs); }, 3.0f, nullptr, &sg); return sg; };
    CHECK(swing(0.3f) > swing(1.5f) + 0.5f && swing(1.5f) < 0.1f);
}
TEST(it_works_the_same_whichever_side_the_load_goes_to) {
    Rig a, b; a.run([](float) { return 0.0f; }, 1.0f); b.run([](float) { return 0.0f; }, 1.0f);
    const Out oa = a.run([](float) { return 0.4f; }, 2.0f), ob = b.run([](float) { return -0.4f; }, 2.0f);
    CHECK(std::fabs(oa.swell - ob.swell) < 0.01f);
}
TEST(rough_road_noise_on_a_straight_does_nothing) {
    for (float amp : {0.1f, 0.3f, 0.6f}) {
        Rig r; float sw, sg;
        r.run([](float) { return 0.0f; }, 1.0f);
        r.run([](float) { return 0.0f; }, 10.0f, &sw, &sg, amp, 0.02f);       // noisy strut forces, barely any lateral acceleration
        CHECK(sw == 0.0f && sg == 0.0f);
    }
}
TEST(the_standing_offset_is_taken_off) {
    Rig r; r.speed = 0.0f;
    r.run([](float) { return 0.12f; }, 1.5f);                              // the standing car leans a little to one side
    r.speed = 20.0f;
    float sw, sg;
    r.run([](float) { return 0.12f; }, 3.0f, &sw, &sg, 0.0f, 0.5f);       // driving straight with the same lean
    CHECK(sw < 0.02f && sg < 0.02f);
}
TEST(nothing_is_added_without_known_units_in_the_air_or_at_a_crawl) {
    { Rig r; r.latOk = false; float sw; r.run([](float) { return 0.4f; }, 2.0f, &sw); CHECK(sw == 0.0f); }
    { Rig r; r.air = 1.0f; float sw; r.run([](float) { return 0.4f; }, 2.0f, &sw); CHECK(sw == 0.0f); }
    { Rig r; r.speed = 1.0f; float sw; r.run([](float) { return 0.4f; }, 2.0f, &sw); CHECK(sw == 0.0f); }
}
TEST(the_gain_scales_it_and_never_goes_past_the_limit) {
    Out o; o.swell = 1.0f; o.surge = 1.0f;
    CHECK(LateralMul(o, 0.0f) == 1.0f);
    CHECK(LateralMul(o, 2.0f) <= 1.0f + kMaxAdd + 1e-6f);
    Out h; h.swell = 0.5f;
    CHECK(std::fabs(LateralMul(h, 1.0f) - (1.0f + 0.5f * kSwellAdd)) < 1e-6f);
    CHECK(LateralMul(h, 2.0f) > LateralMul(h, 1.0f));
}
TEST(broken_numbers_change_nothing) {
    Rig r; r.run([](float) { return 0.0f; }, 1.0f);
    Out o; In in; in.lateral = kNaN; in.speed = kNaN; in.airborne = kNaN; in.latG = kNaN; in.latOk = true;
    o = r.t.Update(in, kNaN);
    CHECK(std::isfinite(o.swell) && std::isfinite(o.surge) && o.swell == 0.0f && o.surge == 0.0f);
    Out bad; bad.swell = kNaN; bad.surge = kNaN;
    CHECK(LateralMul(bad, 1.0f) == 1.0f && LateralMul(Out(), kNaN) == 1.0f);
    AccelLight::Out ao; ao.drop = kNaN; ao.heavy = kNaN;
    CHECK(FrontMul(ao, kNaN, kNaN) == 1.0f);
}
TEST(a_reset_forgets_everything) {
    Rig r; r.run([](float) { return 0.4f; }, 2.0f);
    r.t.Reset();
    const Out o = r.t.Update(In(), 0.01f);
    CHECK(o.swell == 0.0f && o.surge == 0.0f);
}

int main() {
    for (const Reg& r : regs()) { const int before = g_fail; r.fn(); std::printf("%s  %s\n", g_fail == before ? "ok  " : "FAIL", r.name); }
    std::printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
