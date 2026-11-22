// Tests for light steering under hard acceleration (accel_light.hpp). Self-contained:
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

static In cruise(float share = 0.52f) { In in; in.frontShare = share; in.throttle = 0.2f; in.speed = 25; in.accOk = true; in.accMag = 0.3f; return in; }
static In launch(float share, float thr = 1.0f) { In in; in.frontShare = share; in.throttle = thr; in.speed = 20; in.accOk = true; in.accMag = 6.0f; return in; }
static Out run(AccelLight::Tracker& t, In in, float secs) { Out o; for (int i = 0; i < (int)(secs * 100); ++i) o = t.Update(in, 0.01f); return o; }

TEST(nothing_happens_until_the_normal_share_is_learned) {
    AccelLight::Tracker t;
    const Out o = run(t, launch(0.40f), 3.0f);                          // hard acceleration from the first second: nothing to compare with
    CHECK(!o.learned && o.effect == 0.0f && o.drop == 0.0f);
    AccelLight::Tracker t2;
    CHECK(!run(t2, cruise(), 1.0f).learned);                            // 1 s of steady driving is not yet enough
    CHECK(run(t2, cruise(), 1.0f).learned);
}

TEST(the_normal_share_is_learned_from_steady_driving) {
    AccelLight::Tracker t;
    const Out o = run(t, cruise(0.55f), 5.0f);
    CHECK(o.learned && std::fabs(o.base - 0.55f) < 0.01f);
    CHECK(o.drop == 0.0f && o.rawDrop == 0.0f && o.effect == 0.0f);     // at normal: nothing
}

TEST(hard_acceleration_that_unloads_the_front_lightens_the_steering) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 4.0f);
    const Out o = run(t, launch(0.42f), 1.5f);                          // front share down by about 19% of normal
    CHECK(o.learned && o.drop > 0.12f && o.effect > 0.9f);
    CHECK(o.rawDrop > 0.17f && o.rawDrop < 0.21f);                      // the measured drop, 0.42 against 0.52, is reported as it is
    CHECK(std::fabs(AccelLight::Mul(o, 0.3f) - 0.7f) < 0.03f);
    CHECK(AccelLight::Mul(o, 0.0f) == 1.0f);                            // off: exactly RBR's force
}

TEST(a_small_unloading_is_a_small_effect) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 4.0f);
    const Out o = run(t, launch(0.49f), 1.5f);                          // 6% of normal
    CHECK(o.effect > 0.2f && o.effect < 0.7f);
}

TEST(loading_the_front_more_does_nothing_and_neither_does_braking) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 4.0f);
    CHECK(run(t, launch(0.60f), 1.0f).effect == 0.0f);                  // weight forward
    In brake = launch(0.40f, 0.0f); brake.brake = 0.8f;                 // front share down but on the brakes (a crest, say)
    CHECK(run(t, brake, 1.0f).effect == 0.0f);
}

TEST(a_crest_without_throttle_does_nothing) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 4.0f);
    In crest = launch(0.38f, 0.1f); crest.accMag = 1.0f;
    CHECK(run(t, crest, 1.0f).effect < 0.01f);
}

TEST(not_in_the_air_and_not_when_crawling) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 4.0f);
    In air = launch(0.40f); air.airborne = 1.0f;
    CHECK(run(t, air, 1.0f).effect == 0.0f);
    In slow = launch(0.40f); slow.speed = 1.0f;
    CHECK(run(t, slow, 1.0f).effect == 0.0f);
}

TEST(it_fades_in_and_out_and_never_snaps) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 4.0f);
    float prev = 0, maxStep = 0, peak = 0;
    for (int i = 0; i < 300; ++i) { const Out o = t.Update(launch(0.40f), 0.01f); maxStep = std::max(maxStep, std::fabs(o.effect - prev)); prev = o.effect; peak = std::max(peak, o.effect); }
    for (int i = 0; i < 300; ++i) { const Out o = t.Update(cruise(0.52f), 0.01f); maxStep = std::max(maxStep, std::fabs(o.effect - prev)); prev = o.effect; }
    CHECK(peak > 0.9f && maxStep < 0.1f && prev < 0.02f);
}

TEST(cornering_does_not_teach_it_a_wrong_normal) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 4.0f);
    In corner = cruise(0.45f); corner.accMag = 7.0f;                    // a hard corner shifts the share, and is not steady driving
    run(t, corner, 6.0f);
    const Out o = run(t, cruise(0.52f), 0.5f);
    CHECK(std::fabs(o.base - 0.52f) < 0.01f);
}

TEST(without_known_units_the_pedals_alone_decide_what_is_steady) {
    AccelLight::Tracker t;
    In a = cruise(0.52f); a.accOk = false;
    run(t, a, 4.0f);
    In l = launch(0.42f); l.accOk = false;
    CHECK(run(t, l, 1.5f).effect > 0.9f);
}

TEST(the_amount_is_limited) {
    Out full; full.effect = 1.0f;
    CHECK(AccelLight::Mul(full, 5.0f) >= 0.4f - 1e-4f);                 // never more than 60% off
    CHECK(AccelLight::Mul(full, -1.0f) == 1.0f);
    Out half; half.effect = 0.5f;
    CHECK(std::fabs(AccelLight::Mul(half, 0.4f) - 0.8f) < 1e-4f);
    Out none;
    CHECK(AccelLight::Mul(none, 0.6f) == 1.0f);
}

TEST(bad_input_never_makes_nan) {
    AccelLight::Tracker t;
    const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
    run(t, cruise(), 3.0f);
    In bad; bad.frontShare = nan; bad.throttle = inf; bad.brake = nan; bad.speed = -inf; bad.airborne = nan; bad.accMag = nan; bad.accOk = true;
    for (int i = 0; i < 50; ++i) {
        const Out o = t.Update(bad, i % 2 ? nan : -1.0f);
        CHECK(std::isfinite(o.base) && std::isfinite(o.drop) && std::isfinite(o.effect) && o.effect >= 0.0f && o.effect <= 1.0f);
    }
    CHECK(AccelLight::Mul(t.Update(bad, 0.01f), nan) == 1.0f);
}

TEST(reset_forgets_everything) {
    AccelLight::Tracker t;
    run(t, cruise(0.52f), 4.0f);
    t.Reset();
    const Out o = run(t, launch(0.40f), 1.0f);
    CHECK(!o.learned && o.effect == 0.0f);
}

int main() {
    for (const Reg& r : regs()) { const int before = g_fail; r.fn(); std::printf("%s %s\n", g_fail == before ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d tests, %d checks, %d failures\n", (int)regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
