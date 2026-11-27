// Tests for light steering under hard acceleration (accel_light.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. accel_light_test.cpp -o accel_light_test && ./accel_light_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>
#include "effects/accel_light.hpp"
#include "tests/mini_test.hpp"

using AccelLight::In; using AccelLight::Out;

// A car on a test track: the tracker is fed at 100 Hz with the front share, and the ground speed follows the acceleration asked for (in g).
struct Rig {
    AccelLight::Tracker t; float v = 0; unsigned seed = 1;
    float noise() { seed = seed * 1664525u + 1013904223u; return (float)((seed >> 8) & 0xFFFF) / 32767.5f - 1.0f; }
    Out run(float share, float aG, float secs, float noiseAmp = 0.0f, float throttle = -1.0f, float airborne = 0.0f) {
        Out o;
        for (int i = 0; i < (int)(secs * 100 + 0.5f); ++i) {
            v = std::max(0.0f, v + aG * 9.81f * 0.01f);
            In in; in.frontShare = share + noiseAmp * noise(); in.speed = v; in.airborne = airborne;
            in.throttle = throttle >= 0.0f ? throttle : (aG > 0.05f ? 1.0f : (aG < -0.05f ? 0.0f : 0.2f));
            in.accOk = true; in.accMag = std::fabs(aG) * 9.81f;
            o = t.Update(in, 0.01f);
        }
        return o;
    }
};

TEST(the_standing_share_is_known_from_the_start_line) {
    Rig r;
    const Out o = r.run(0.54f, 0.0f, 0.6f);
    CHECK(o.learned && o.fromRest && std::fabs(o.base - 0.54f) < 0.01f);
    CHECK(o.drop == 0.0f && o.rawDrop == 0.0f);
    Rig r2;
    CHECK(!r2.run(0.52f, 0.0f, 0.2f).learned);                          // a fifth of a second is not enough to trust it
}

TEST(the_launch_is_lightened_at_once_with_no_wait) {
    Rig r;
    r.run(0.52f, 0.0f, 3.0f);                                           // sitting on the start line
    int ms = -1;
    for (int i = 0; i < 100; ++i) {
        const Out o = r.run(0.40f, 0.5f, 0.01f);                        // the launch: half a g, the front unloads
        if (ms < 0 && AccelLight::Mul(o, 1.0f) < 0.9f) ms = i * 10;
    }
    CHECK(ms >= 0 && ms <= 300);                                        // inside the first third of a second
}

TEST(the_steering_loses_force_in_proportion_to_the_front_load_lost) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    const Out o = r.run(0.40f, 0.5f, 2.0f);                             // the front carries 20% less than standing
    CHECK(o.rawDrop > 0.19f && o.rawDrop < 0.21f && o.gate > 0.99f);
    CHECK(std::fabs(AccelLight::Mul(o, 1.0f) - (1.0f - (0.20f - AccelLight::kDeadband))) < 0.01f);
    CHECK(AccelLight::Mul(o, 0.5f) > AccelLight::Mul(o, 1.0f) && AccelLight::Mul(o, 1.0f) > AccelLight::Mul(o, 2.0f));
    CHECK(AccelLight::Mul(o, 0.0f) == 1.0f);                            // off: exactly RBR's force
}

TEST(the_acceleration_is_measured_from_the_speed) {
    Rig r;
    r.run(0.50f, 0.0f, 1.0f);
    CHECK(std::fabs(r.run(0.50f, 0.5f, 2.0f).accelG - 0.5f) < 0.05f);
    CHECK(std::fabs(r.run(0.50f, 0.0f, 3.0f).accelG) < 0.03f);
    CHECK(r.run(0.50f, -0.8f, 1.0f).accelG < -0.5f);
}

TEST(only_while_accelerating_not_at_a_steady_speed_whatever_the_load_does) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    r.run(0.50f, 0.5f, 3.0f);                                           // up to speed
    const Out o = r.run(0.38f, 0.0f, 3.0f);                             // cruising, the front share low (a steep uphill, a crest)
    CHECK(o.gate < 0.05f && AccelLight::Mul(o, 2.0f) > 0.99f);
}

TEST(a_rough_road_does_not_flicker_the_steering_on_the_straights) {
    Rig r;
    r.run(0.52f, 0.0f, 2.0f);
    r.run(0.52f, 0.5f, 3.0f);
    float early = 1.0f, worst = 1.0f, sum = 0.0f; int n = 0;
    for (int i = 0; i < 1100; ++i) {                                     // 11 s of load noise, 20 points either way (very rough ground), right after a hard acceleration
        const float m = AccelLight::Mul(r.run(0.52f, 0.0f, 0.01f, 0.20f), 1.0f);
        if (i < 100) early = std::min(early, m);                         // the first second: the acceleration is still fading out of the gate
        else { worst = std::min(worst, m); sum += m; ++n; }
    }
    CHECK(early > 0.93f);
    CHECK(worst > 0.995f && sum / n > 0.999f);
}

TEST(braking_and_more_weight_on_the_front_are_left_to_rbr) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    r.run(0.50f, 0.5f, 3.0f);
    CHECK(AccelLight::Mul(r.run(0.38f, -0.8f, 1.0f), 2.0f) > 0.99f);    // braking: even a low front share is not lightness
    CHECK(AccelLight::Mul(r.run(0.62f, 0.4f, 2.0f), 2.0f) > 0.999f);    // accelerating with the front carrying more than usual
}

TEST(never_more_than_the_limit_and_never_in_the_air) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    const Out o = r.run(0.10f, 0.5f, 2.0f);
    CHECK(AccelLight::Mul(o, 2.0f) >= 1.0f - AccelLight::kMaxCut - 1e-4f);
    CHECK(r.run(0.10f, 0.5f, 1.0f, 0.0f, -1.0f, 1.0f).drop < 0.01f);
}

TEST(it_fades_in_and_out_and_never_snaps) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    float prev = 1.0f, maxStep = 0.0f, low = 1.0f;
    for (int i = 0; i < 300; ++i) { const float m = AccelLight::Mul(r.run(0.38f, 0.5f, 0.01f), 1.0f); maxStep = std::max(maxStep, std::fabs(m - prev)); prev = m; low = std::min(low, m); }
    for (int i = 0; i < 300; ++i) { const float m = AccelLight::Mul(r.run(0.50f, 0.0f, 0.01f), 1.0f); maxStep = std::max(maxStep, std::fabs(m - prev)); prev = m; }
    CHECK(low < 0.85f && maxStep < 0.03f && prev > 0.99f);
}

TEST(a_jump_in_the_speed_is_not_a_launch) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    r.v = 40.0f;                                                        // a respawn or a crash: the speed changes by 40 m/s in one step
    CHECK(AccelLight::Mul(r.run(0.38f, 0.0f, 1.0f), 1.0f) > 0.95f);
}

TEST(revving_on_the_line_does_not_teach_the_squat_or_lighten_a_standing_car) {
    Rig r;
    r.run(0.52f, 0.0f, 2.0f);
    const Out o = r.run(0.45f, 0.0f, 3.0f, 0.0f, 0.9f);                 // revving against the brake: the car squats, the front share falls, nothing moves
    CHECK(std::fabs(o.base - 0.52f) < 0.01f);
    CHECK(AccelLight::Mul(o, 1.0f) == 1.0f);
}

TEST(standing_again_refreshes_the_standing_share) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    r.run(0.40f, 0.5f, 2.0f);
    r.v = 0.0f;                                                         // stopped
    const Out o = r.run(0.46f, 0.0f, 4.0f);                             // a restart, or another car: a different standing share
    CHECK(std::fabs(o.base - 0.46f) < 0.01f && o.drop < 0.01f);
}

TEST(a_car_already_moving_learns_from_steady_driving_after_a_second) {
    Rig r; r.v = 25.0f;
    CHECK(!r.run(0.52f, 0.0f, 0.8f).learned);
    const Out o = r.run(0.52f, 0.0f, 0.5f);
    CHECK(o.learned && !o.fromRest && std::fabs(o.base - 0.52f) < 0.01f);
    CHECK(AccelLight::Mul(r.run(0.42f, 0.4f, 1.0f), 1.0f) < 0.9f);
}

TEST(a_start_on_a_hill_is_corrected_once_the_car_rolls_on_the_flat) {
    Rig up;                                                              // standing uphill: weight to the rear, the standing front share reads low
    up.run(0.49f, 0.0f, 3.0f);
    up.run(0.49f, 0.4f, 3.0f); up.run(0.52f, 0.0f, 25.0f);              // then driving steadily on the flat
    CHECK(std::fabs(up.run(0.52f, 0.0f, 0.1f).base - 0.52f) < 0.01f);
    Rig down;                                                            // standing downhill: reads high, so a flat road looks like an unloaded front
    down.run(0.55f, 0.0f, 3.0f);
    down.run(0.52f, 0.4f, 3.0f);
    CHECK(AccelLight::Mul(down.run(0.52f, 0.0f, 1.0f), 1.0f) > 0.99f);  // not accelerating, so nothing, whatever the baseline
    down.run(0.52f, 0.0f, 25.0f);
    CHECK(std::fabs(down.run(0.52f, 0.0f, 0.1f).base - 0.52f) < 0.01f);
}

TEST(the_slope_error_is_small_next_to_a_launch) {
    Rig flat, hill;
    flat.run(0.52f, 0.0f, 3.0f);
    hill.run(0.50f, 0.0f, 3.0f);                                         // a steep start: the standing share two points lower
    const float f = AccelLight::Mul(flat.run(0.43f, 0.5f, 2.0f), 1.0f), h = AccelLight::Mul(hill.run(0.43f, 0.5f, 2.0f), 1.0f);
    CHECK(f < 0.85f && h < 0.90f && std::fabs(f - h) < 0.06f);
}

TEST(a_long_hard_launch_does_not_teach_the_normal) {
    Rig r;
    r.run(0.52f, 0.0f, 3.0f);
    r.run(0.43f, 0.3f, 12.0f);                                           // twelve seconds of hard acceleration
    const Out o = r.run(0.43f, 0.3f, 0.1f);
    CHECK(std::fabs(o.base - 0.52f) < 0.01f && AccelLight::Mul(o, 1.0f) < 0.85f);
}

TEST(cornering_does_not_teach_a_wrong_normal) {
    Rig r; r.v = 25.0f;
    r.run(0.52f, 0.0f, 3.0f);
    for (int i = 0; i < 600; ++i) { In in; in.frontShare = 0.45f; in.speed = 25.0f; in.throttle = 0.2f; in.accOk = true; in.accMag = 7.0f; r.t.Update(in, 0.01f); }
    CHECK(std::fabs(r.run(0.52f, 0.0f, 0.2f).base - 0.52f) < 0.01f);
}

TEST(nothing_before_anything_is_known_and_after_a_reset) {
    Rig r;
    const Out o = r.run(0.30f, 0.5f, 0.05f);                            // the very first reading is a car in the middle of a launch: no normal to compare with
    CHECK(!o.learned && AccelLight::Mul(o, 2.0f) == 1.0f);
    Rig r2;
    r2.run(0.50f, 0.0f, 2.0f);
    r2.t.Reset();
    CHECK(AccelLight::Mul(r2.run(0.30f, 0.5f, 0.05f), 2.0f) == 1.0f);
}

TEST(bad_input_never_makes_nan) {
    Rig r;
    const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
    r.run(0.52f, 0.0f, 1.0f);
    In bad; bad.frontShare = nan; bad.throttle = inf; bad.brake = nan; bad.speed = -inf; bad.airborne = nan; bad.accMag = nan; bad.accOk = true;
    for (int i = 0; i < 50; ++i) {
        const Out o = r.t.Update(bad, i % 2 ? nan : -1.0f);
        CHECK(std::isfinite(o.base) && std::isfinite(o.share) && std::isfinite(o.drop) && std::isfinite(o.rawDrop) && std::isfinite(o.accelG) && std::isfinite(o.gate) && o.drop >= 0.0f);
    }
    CHECK(AccelLight::Mul(r.t.Update(bad, 0.01f), nan) == 1.0f);
}

// ---- braking side of the same tracker: dive and heavy
// A car that stood on the line (so "normal" is learned), then drives at 25 m/s and brakes at `decelG`, with the front share at `share`.
struct BrakeRig {
    AccelLight::Tracker t; float v = 0; Out last;
    void stand(float share) { for (int i = 0; i < 100; ++i) { In in; in.frontShare = share; in.throttle = 0.1f; last = t.Update(in, 0.01f); } }
    void drive(float share, float secs, float v0) { v = v0; for (int i = 0; i < (int)(secs * 100); ++i) { In in; in.frontShare = share; in.speed = v; in.throttle = 0.3f; in.accOk = true; in.accMag = 0.2f; last = t.Update(in, 0.01f); } }
    void brake(float share, float decelG, float pedal, float secs) {
        for (int i = 0; i < (int)(secs * 100); ++i) {
            v = std::max(0.0f, v - decelG * 9.81f * 0.01f);
            In in; in.frontShare = share; in.speed = v; in.brake = pedal; in.accOk = true; in.accMag = decelG * 9.81f; last = t.Update(in, 0.01f);
        }
    }
};
TEST(dive_follows_the_weight_moving_onto_the_front_while_braking) {
    BrakeRig r; r.stand(0.50f); r.drive(0.50f, 2.0f, 25.0f);
    CHECK(r.last.dive < 0.02f);
    r.brake(0.58f, 0.8f, 1.0f, 1.0f);
    CHECK(r.last.dive > 0.7f);                          // 8 points of share is most of the 10 that make 1.0
    r.brake(0.62f, 0.8f, 1.0f, 0.5f);
    CHECK(r.last.dive > 0.95f);
}
TEST(dive_needs_the_brake_pedal_and_is_zero_without_it) {
    BrakeRig r; r.stand(0.50f); r.drive(0.50f, 2.0f, 25.0f);
    r.brake(0.60f, 0.8f, 0.0f, 1.0f);                   // share high, pedal off (engine braking, a downhill)
    CHECK(r.last.dive < 0.02f);
}
TEST(heavy_braking_needs_the_car_to_really_slow_down) {
    BrakeRig a; a.stand(0.50f); a.drive(0.50f, 2.0f, 25.0f);
    a.brake(0.60f, 0.02f, 1.0f, 1.5f);                  // pedal and weight, but the speed hardly changes (a short tap)
    CHECK(a.last.dive > 0.8f); CHECK(a.last.heavy < 0.05f);
    BrakeRig b; b.stand(0.50f); b.drive(0.50f, 2.0f, 25.0f);
    b.brake(0.60f, 0.8f, 1.0f, 1.5f);
    CHECK(b.last.heavy > 0.8f);
}
TEST(braking_never_teaches_the_normal_share) {
    BrakeRig r; r.stand(0.50f); r.drive(0.50f, 2.0f, 25.0f);
    r.brake(0.62f, 0.8f, 1.0f, 3.0f);
    CHECK(std::fabs(r.last.base - 0.50f) < 0.01f);
}
TEST(dive_is_zero_before_the_normal_is_known_and_in_the_air) {
    AccelLight::Tracker t; Out o;
    for (int i = 0; i < 20; ++i) { In in; in.frontShare = 0.7f; in.speed = 20.0f; in.brake = 1.0f; in.throttle = 0.8f; o = t.Update(in, 0.01f); }
    CHECK(o.dive == 0.0f && o.heavy == 0.0f);
    BrakeRig r; r.stand(0.50f); r.drive(0.50f, 2.0f, 25.0f);
    for (int i = 0; i < 100; ++i) { In in; in.frontShare = 0.62f; in.speed = 10.0f; in.brake = 1.0f; in.airborne = 1.0f; in.accOk = true; in.accMag = 8.0f; r.last = r.t.Update(in, 0.01f); }
    CHECK(r.last.heavy < 0.01f);
}
TEST(dive_and_heavy_survive_bad_input) {
    AccelLight::Tracker t; Out o;
    for (int i = 0; i < 50; ++i) { In in; in.frontShare = i % 2 ? std::numeric_limits<float>::quiet_NaN() : 0.6f; in.speed = i % 3 ? 20.0f : std::numeric_limits<float>::infinity(); in.brake = 1.0f; o = t.Update(in, i % 5 ? 0.01f : std::numeric_limits<float>::quiet_NaN()); }
    CHECK(std::isfinite(o.dive) && std::isfinite(o.heavy) && o.dive >= 0.0f && o.dive <= 1.0f && o.heavy >= 0.0f && o.heavy <= 1.0f);
}

int main() { return mini::RunAll(); }
