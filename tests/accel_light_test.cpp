// Tests for light steering from the front load (accel_light.hpp). Self-contained:
//   g++ -std=c++17 -O1 accel_light_test.cpp -o accel_light_test && ./accel_light_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>
#include "../accel_light.hpp"

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)

using AccelLight::In; using AccelLight::Out;

static In standing(float share = 0.52f) { In in; in.frontShare = share; in.speed = 0.0f; in.throttle = 0.0f; return in; }
static In cruise(float share = 0.52f) { In in; in.frontShare = share; in.throttle = 0.2f; in.speed = 25; in.accOk = true; in.accMag = 0.3f; return in; }
static In moving(float share, float thr = 1.0f, float speed = 6.0f) { In in; in.frontShare = share; in.throttle = thr; in.speed = speed; in.accOk = true; in.accMag = 6.0f; return in; }
static Out run(AccelLight::Tracker& t, In in, float secs) { Out o; for (int i = 0; i < (int)(secs * 100); ++i) o = t.Update(in, 0.01f); return o; }

TEST(the_standing_share_is_known_from_the_start_line) {
    AccelLight::Tracker t;
    const Out o = run(t, standing(0.54f), 0.6f);
    CHECK(o.learned && o.fromRest && std::fabs(o.base - 0.54f) < 0.01f);
    CHECK(o.drop == 0.0f && o.rawDrop == 0.0f);                        // standing: nothing is lost
    AccelLight::Tracker t2;
    CHECK(!run(t2, standing(), 0.2f).learned);                          // a fifth of a second is not enough to trust it
}

TEST(the_launch_is_lightened_at_once_with_no_wait) {
    AccelLight::Tracker t;
    run(t, standing(0.52f), 3.0f);                                      // sitting on the start line
    float first = 1.0f; int ms = -1;
    for (int i = 0; i < 100; ++i) {
        const Out o = t.Update(moving(0.40f, 1.0f, 0.2f + i * 0.2f), 0.01f);   // the launch: the front unloads as the car starts to move
        if (ms < 0 && AccelLight::Mul(o, 1.0f) < 0.9f) { ms = i * 10; first = AccelLight::Mul(o, 1.0f); }
    }
    CHECK(ms >= 0 && ms <= 200);                                        // within the first fifth of a second of the launch
    CHECK(first < 0.9f);
}

TEST(the_steering_loses_force_in_proportion_to_the_front_load_lost) {
    AccelLight::Tracker t;
    run(t, standing(0.50f), 2.0f);
    const Out o = run(t, moving(0.40f), 2.0f);                          // the front carries 20% less than standing
    CHECK(o.rawDrop > 0.19f && o.rawDrop < 0.21f);
    CHECK(std::fabs(AccelLight::Mul(o, 1.0f) - (1.0f - (0.20f - AccelLight::kDeadband))) < 0.01f);   // proportional, past the small dead band
    CHECK(AccelLight::Mul(o, 0.5f) > AccelLight::Mul(o, 1.0f) && AccelLight::Mul(o, 1.0f) > AccelLight::Mul(o, 2.0f));
    CHECK(AccelLight::Mul(o, 0.0f) == 1.0f);                            // off: exactly RBR's force
}

TEST(the_pedals_do_not_decide_the_load_does) {
    AccelLight::Tracker t;
    run(t, standing(0.50f), 2.0f);
    const Out o = run(t, moving(0.40f, 0.0f, 20.0f), 2.0f);             // no throttle, front unloaded (a steep uphill, a crest): the steering is still light
    CHECK(AccelLight::Mul(o, 1.0f) < 0.85f);
    AccelLight::Tracker t2;
    run(t2, standing(0.50f), 2.0f);
    CHECK(AccelLight::Mul(run(t2, moving(0.50f, 1.0f, 20.0f), 2.0f), 1.0f) == 1.0f);   // flat out but the front keeps its load: nothing
}

TEST(more_weight_on_the_front_is_left_to_rbr) {
    AccelLight::Tracker t;
    run(t, standing(0.50f), 2.0f);
    const Out o = run(t, moving(0.62f, 0.0f, 20.0f), 2.0f);             // braking or downhill
    CHECK(o.drop == 0.0f && o.rawDrop == 0.0f && AccelLight::Mul(o, 2.0f) == 1.0f);
}

TEST(never_more_than_the_limit_and_never_in_the_air) {
    AccelLight::Tracker t;
    run(t, standing(0.50f), 2.0f);
    const Out o = run(t, moving(0.10f), 2.0f);                          // almost no front load at all
    CHECK(AccelLight::Mul(o, 2.0f) >= 1.0f - AccelLight::kMaxCut - 1e-4f);
    In air = moving(0.10f); air.airborne = 1.0f;
    CHECK(run(t, air, 1.0f).drop < 0.01f);
}

TEST(a_bump_does_not_flicker_the_steering) {
    AccelLight::Tracker t;
    run(t, standing(0.50f), 2.0f);
    run(t, cruise(0.50f), 0.5f);
    float worst = 1.0f;
    for (int i = 0; i < 3; ++i) worst = std::min(worst, AccelLight::Mul(t.Update(moving(0.30f), 0.01f), 1.0f));   // 30 ms of the front unloading by 40%
    for (int i = 0; i < 50; ++i) worst = std::min(worst, AccelLight::Mul(t.Update(cruise(0.50f), 0.01f), 1.0f));
    CHECK(worst > 0.9f);
}

TEST(it_fades_in_and_out_and_never_snaps) {
    AccelLight::Tracker t;
    run(t, standing(0.50f), 2.0f);
    float prev = 1.0f, maxStep = 0.0f, low = 1.0f;
    for (int i = 0; i < 300; ++i) { const float m = AccelLight::Mul(t.Update(moving(0.38f), 0.01f), 1.0f); maxStep = std::max(maxStep, std::fabs(m - prev)); prev = m; low = std::min(low, m); }
    for (int i = 0; i < 300; ++i) { const float m = AccelLight::Mul(t.Update(cruise(0.50f), 0.01f), 1.0f); maxStep = std::max(maxStep, std::fabs(m - prev)); prev = m; }
    CHECK(low < 0.85f && maxStep < 0.03f && prev > 0.99f);
}

TEST(revving_on_the_line_does_not_teach_it_the_squat) {
    AccelLight::Tracker t;
    run(t, standing(0.52f), 2.0f);
    In rev = standing(0.45f); rev.throttle = 0.9f;                      // revving against the brake: the car squats, the front share falls
    const Out o = run(t, rev, 3.0f);
    CHECK(std::fabs(o.base - 0.52f) < 0.01f);
    CHECK(AccelLight::Mul(o, 1.0f) < 1.0f);                             // and the lightness is measured against the idle share
}

TEST(standing_again_refreshes_the_standing_share) {
    AccelLight::Tracker t;
    run(t, standing(0.50f), 2.0f);
    run(t, moving(0.40f), 2.0f);
    const Out o = run(t, standing(0.46f), 4.0f);                        // a restart, or another car: a different standing share
    CHECK(std::fabs(o.base - 0.46f) < 0.01f && o.drop < 0.01f);
}

TEST(a_car_already_moving_learns_from_steady_driving_after_a_second) {
    AccelLight::Tracker t;
    CHECK(!run(t, cruise(0.52f), 0.8f).learned);
    const Out o = run(t, cruise(0.52f), 0.5f);
    CHECK(o.learned && !o.fromRest && std::fabs(o.base - 0.52f) < 0.01f);
    CHECK(AccelLight::Mul(run(t, moving(0.42f), 1.0f), 1.0f) < 0.9f);
}

TEST(a_start_on_a_hill_is_corrected_once_the_car_rolls_on_the_flat) {
    AccelLight::Tracker up;                                              // standing uphill: weight to the rear, the standing front share reads low
    run(up, standing(0.49f), 3.0f);
    run(up, cruise(0.52f), 25.0f);                                       // then driving steadily on the flat
    CHECK(std::fabs(run(up, cruise(0.52f), 0.1f).base - 0.52f) < 0.01f);
    AccelLight::Tracker down;                                            // standing downhill: reads high, so a flat road looks like an unloaded front at first
    run(down, standing(0.55f), 3.0f);
    const Out early = run(down, cruise(0.52f), 1.0f);
    CHECK(AccelLight::Mul(early, 1.0f) > 0.95f);                         // a few percent at most, and only until it has caught up
    const Out late = run(down, cruise(0.52f), 25.0f);
    CHECK(std::fabs(late.base - 0.52f) < 0.01f && AccelLight::Mul(late, 1.0f) == 1.0f);
}

TEST(the_slope_error_is_small_next_to_a_launch) {
    AccelLight::Tracker flat, hill;
    run(flat, standing(0.52f), 3.0f);
    run(hill, standing(0.50f), 3.0f);                                    // a steep start: the standing share two points lower
    const float f = AccelLight::Mul(run(flat, moving(0.43f), 2.0f), 1.0f), h = AccelLight::Mul(run(hill, moving(0.43f), 2.0f), 1.0f);
    CHECK(f < 0.85f && h < 0.90f && std::fabs(f - h) < 0.06f);           // the launch is still clearly light either way
}

TEST(a_long_hard_launch_does_not_teach_the_normal) {
    AccelLight::Tracker t;
    run(t, standing(0.52f), 3.0f);
    run(t, moving(0.43f), 12.0f);                                        // twelve seconds of hard acceleration
    const Out o = run(t, moving(0.43f), 0.1f);
    CHECK(std::fabs(o.base - 0.52f) < 0.01f && AccelLight::Mul(o, 1.0f) < 0.85f);
}

TEST(cornering_does_not_teach_a_wrong_normal) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 3.0f);
    In corner = cruise(0.45f); corner.accMag = 7.0f;
    run(t, corner, 6.0f);
    CHECK(std::fabs(run(t, cruise(0.52f), 0.2f).base - 0.52f) < 0.01f);
}

TEST(nothing_before_anything_is_known_and_after_a_reset) {
    AccelLight::Tracker t;
    const Out o = t.Update(moving(0.30f), 0.01f);                       // the very first reading is a car in the middle of a launch: no normal to compare with
    CHECK(!o.learned && AccelLight::Mul(o, 2.0f) == 1.0f);
    AccelLight::Tracker t2;
    run(t2, standing(0.50f), 2.0f);
    t2.Reset();
    CHECK(AccelLight::Mul(t2.Update(moving(0.30f), 0.01f), 2.0f) == 1.0f);
}

TEST(bad_input_never_makes_nan) {
    AccelLight::Tracker t;
    const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
    run(t, standing(), 1.0f);
    In bad; bad.frontShare = nan; bad.throttle = inf; bad.brake = nan; bad.speed = -inf; bad.airborne = nan; bad.accMag = nan; bad.accOk = true;
    for (int i = 0; i < 50; ++i) {
        const Out o = t.Update(bad, i % 2 ? nan : -1.0f);
        CHECK(std::isfinite(o.base) && std::isfinite(o.share) && std::isfinite(o.drop) && std::isfinite(o.rawDrop) && o.drop >= 0.0f);
    }
    CHECK(AccelLight::Mul(t.Update(bad, 0.01f), nan) == 1.0f);
}

int main() {
    for (const Reg& r : regs()) { const int before = g_fail; r.fn(); std::printf("%s %s\n", g_fail == before ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d tests, %d checks, %d failures\n", (int)regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
