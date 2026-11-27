// Tests for the wheel's reaction to a crash (crash_fx.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. crash_fx_test.cpp -o crash_fx_test && ./crash_fx_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>

#include "effects/crash_fx.hpp"
#include "tests/mini_test.hpp"

static const float kNaN = std::numeric_limits<float>::quiet_NaN();
using CrashFx::Envelope;

static float peakOver(Envelope& e, unsigned seq, float sev, float secs, float gain = 1.0f) {
    float pk = 0; for (int i = 0; i < (int)(secs * 100); ++i) pk = std::max(pk, e.Step(seq, sev, 0.01f, gain)); return pk;
}
TEST(nothing_happens_without_an_event) { Envelope e; CHECK(peakOver(e, 0, 0.0f, 2.0f) == 0.0f); CHECK(peakOver(e, 0, 1.0f, 2.0f) == 0.0f); }
TEST(an_event_gives_a_jolt_that_scales_with_severity) {
    Envelope a, b; const float soft = peakOver(a, 1, 0.3f, 0.5f), hard = peakOver(b, 1, 1.0f, 0.5f);
    CHECK(soft > 0.25f && soft <= 0.31f); CHECK(hard > 0.95f && hard <= 1.0f);
}
TEST(it_is_over_in_a_fraction_of_a_second) {
    Envelope e; e.Step(1, 1.0f, 0.01f, 1.0f); float v = 1;
    for (int i = 0; i < 60; ++i) v = e.Step(1, 1.0f, 0.01f, 1.0f);
    CHECK(v < 0.05f);
    for (int i = 0; i < 100; ++i) v = e.Step(1, 1.0f, 0.01f, 1.0f);
    CHECK(v == 0.0f);
}
TEST(it_holds_then_decays) {
    Envelope e; float a = e.Step(1, 1.0f, 0.01f, 1.0f), b = a, c = a;
    for (int i = 0; i < 10; ++i) b = e.Step(1, 1.0f, 0.01f, 1.0f);
    for (int i = 0; i < 20; ++i) c = e.Step(1, 1.0f, 0.01f, 1.0f);
    CHECK(a > 0.99f); CHECK(b < a); CHECK(c < b);
}
TEST(a_second_crash_restarts_it_and_a_weak_one_does_not_cut_a_strong_one) {
    Envelope e; peakOver(e, 1, 1.0f, 0.1f);
    const float w = e.Step(2, 0.2f, 0.01f, 1.0f);
    CHECK(w > 0.5f);                                    // still carrying the big one
    Envelope f; peakOver(f, 1, 0.3f, 1.2f);             // finished
    CHECK(peakOver(f, 2, 0.9f, 0.3f) > 0.85f);
}
TEST(the_live_gain_scales_it_and_zero_mutes_it) {
    Envelope a, b, c; CHECK(std::fabs(peakOver(a, 1, 1.0f, 0.3f, 0.5f) - 0.5f) < 0.02f); CHECK(peakOver(b, 1, 1.0f, 0.3f, 0.0f) == 0.0f); CHECK(peakOver(c, 1, 1.0f, 0.3f, 7.0f) <= 1.0f);
}
TEST(a_new_stage_that_resets_the_counter_is_not_a_crash) {
    Envelope e; peakOver(e, 5, 1.0f, 1.5f);
    CHECK(peakOver(e, 0, 0.0f, 1.0f) == 0.0f);
}
TEST(multiplier_and_push_follow_the_level_and_are_bounded) {
    CHECK(CrashFx::Mul(0.0f, 1.0f) == 1.0f); CHECK(std::fabs(CrashFx::Mul(1.0f, 0.8f) - 1.8f) < 1e-5f); CHECK(CrashFx::Mul(1.0f, 99.0f) <= 4.0f);
    CHECK(CrashFx::Add(1.0f, 0.15f) == 0.15f); CHECK(CrashFx::Add(0.0f, 0.5f) == 0.0f); CHECK(CrashFx::Add(1.0f, 9.0f) <= 1.0f);
    CHECK(CrashFx::Mul(kNaN, kNaN) == 1.0f); CHECK(CrashFx::Add(kNaN, kNaN) == 0.0f);
}
TEST(bad_numbers_never_poison_it) {
    Envelope e; float v = e.Step(1, kNaN, kNaN, kNaN); CHECK(std::isfinite(v));
    v = peakOver(e, 2, 1.0f, 0.3f); CHECK(std::isfinite(v) && v > 0.9f);
}

int main() { return mini::RunAll(); }

