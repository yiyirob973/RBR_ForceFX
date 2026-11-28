// Tests for weight transfer (effects/weight_transfer.hpp): the tracker of the load along and across the car, and the one function that turns it into a
// multiplier for RBR's steering force. Four parts: acceleration (lighter), braking (heavier), cornering (swell and surge), and the function itself. Self-contained:
//   g++ -std=c++17 -O1 -I.. weight_transfer_test.cpp -o weight_transfer_test && ./weight_transfer_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>
#include "effects/weight_transfer.hpp"
#include "tests/mini_test.hpp"

static const float kNaN = std::numeric_limits<float>::quiet_NaN();

// ---------------------------------------------------------------- along the car: acceleration (lighter) and braking (heavier)
using WeightTransfer::In; using WeightTransfer::Out; using WeightTransfer::Gains;

// A front share with the rest spread evenly behind it and nothing across the car.
static void Front(In& in, float share) { in.wload[0] = in.wload[1] = share * 0.5f; in.wload[2] = in.wload[3] = (1.0f - share) * 0.5f; }
// RBR's force times the weight transfer function with only the lightness on, only the heaviness on, only the cornering on.
static float LightMul(const Out& o, float g) { Gains k; k.light = g; k.heavy = 0.0f; k.corner = 0.0f; return WeightTransfer::Mul(o, k); }
static float HeavyMul(const Out& o, float g) { Gains k; k.light = 0.0f; k.heavy = g; k.corner = 0.0f; return WeightTransfer::Mul(o, k); }
static float CornerMul(const Out& o, float g) { Gains k; k.light = 0.0f; k.heavy = 0.0f; k.corner = g; return WeightTransfer::Mul(o, k); }
static float Drop(const Out& o) { return std::max(0.0f, -o.rawChange); }     // the front load lost right now, 0.2 = 20% less than standing

// A car on a test track: the tracker is fed at 100 Hz with the front share, and the ground speed follows the acceleration asked for (in g).
struct Rig {
    WeightTransfer::Tracker t; float v = 0; unsigned seed = 1;
    float noise() { seed = seed * 1664525u + 1013904223u; return (float)((seed >> 8) & 0xFFFF) / 32767.5f - 1.0f; }
    Out run(float share, float aG, float secs, float noiseAmp = 0.0f, float throttle = -1.0f, float airborne = 0.0f) {
        Out o;
        for (int i = 0; i < (int)(secs * 100 + 0.5f); ++i) {
            v = std::max(0.0f, v + aG * 9.81f * 0.01f);
            In in; Front(in, share + noiseAmp * noise()); in.speed = v; in.airborne = airborne;
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
    CHECK(o.light == 0.0f && Drop(o) == 0.0f);
    Rig r2;
    CHECK(!r2.run(0.52f, 0.0f, 0.2f).learned);                          // a fifth of a second is not enough to trust it
}

TEST(the_launch_is_lightened_at_once_with_no_wait) {
    Rig r;
    r.run(0.52f, 0.0f, 3.0f);                                           // sitting on the start line
    int ms = -1;
    for (int i = 0; i < 100; ++i) {
        const Out o = r.run(0.40f, 0.5f, 0.01f);                        // the launch: half a g, the front unloads
        if (ms < 0 && LightMul(o, 1.0f) < 0.9f) ms = i * 10;
    }
    CHECK(ms >= 0 && ms <= 300);                                        // inside the first third of a second
}

TEST(the_steering_loses_force_in_proportion_to_the_front_load_lost) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    const Out o = r.run(0.40f, 0.5f, 2.0f);                             // the front carries 20% less than standing
    CHECK(Drop(o) > 0.19f && Drop(o) < 0.21f && o.gate > 0.99f);
    CHECK(std::fabs(LightMul(o, 1.0f) - (1.0f - (0.20f - WeightTransfer::kDeadband))) < 0.01f);
    CHECK(LightMul(o, 0.5f) > LightMul(o, 1.0f) && LightMul(o, 1.0f) > LightMul(o, 2.0f));
    CHECK(LightMul(o, 0.0f) == 1.0f);                            // off: exactly RBR's force
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
    CHECK(o.gate < 0.05f && LightMul(o, 2.0f) > 0.99f);
}

TEST(a_rough_road_does_not_flicker_the_steering_on_the_straights) {
    Rig r;
    r.run(0.52f, 0.0f, 2.0f);
    r.run(0.52f, 0.5f, 3.0f);
    float early = 1.0f, worst = 1.0f, sum = 0.0f; int n = 0;
    for (int i = 0; i < 1100; ++i) {                                     // 11 s of load noise, 20 points either way (very rough ground), right after a hard acceleration
        const float m = LightMul(r.run(0.52f, 0.0f, 0.01f, 0.20f), 1.0f);
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
    CHECK(LightMul(r.run(0.38f, -0.8f, 1.0f), 2.0f) > 0.99f);    // braking: even a low front share is not lightness
    CHECK(LightMul(r.run(0.62f, 0.4f, 2.0f), 2.0f) > 0.999f);    // accelerating with the front carrying more than usual
}

TEST(never_more_than_the_limit_and_never_in_the_air) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    const Out o = r.run(0.10f, 0.5f, 2.0f);
    CHECK(LightMul(o, 2.0f) >= 1.0f - WeightTransfer::kMaxCut - 1e-4f);
    CHECK(r.run(0.10f, 0.5f, 1.0f, 0.0f, -1.0f, 1.0f).light < 0.01f);
}

TEST(it_fades_in_and_out_and_never_snaps) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    float prev = 1.0f, maxStep = 0.0f, low = 1.0f;
    for (int i = 0; i < 300; ++i) { const float m = LightMul(r.run(0.38f, 0.5f, 0.01f), 1.0f); maxStep = std::max(maxStep, std::fabs(m - prev)); prev = m; low = std::min(low, m); }
    for (int i = 0; i < 300; ++i) { const float m = LightMul(r.run(0.50f, 0.0f, 0.01f), 1.0f); maxStep = std::max(maxStep, std::fabs(m - prev)); prev = m; }
    CHECK(low < 0.85f && maxStep < 0.03f && prev > 0.99f);
}

TEST(a_jump_in_the_speed_is_not_a_launch) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    r.v = 40.0f;                                                        // a respawn or a crash: the speed changes by 40 m/s in one step
    CHECK(LightMul(r.run(0.38f, 0.0f, 1.0f), 1.0f) > 0.95f);
}

TEST(revving_on_the_line_does_not_teach_the_squat_or_lighten_a_standing_car) {
    Rig r;
    r.run(0.52f, 0.0f, 2.0f);
    const Out o = r.run(0.45f, 0.0f, 3.0f, 0.0f, 0.9f);                 // revving against the brake: the car squats, the front share falls, nothing moves
    CHECK(std::fabs(o.base - 0.52f) < 0.01f);
    CHECK(LightMul(o, 1.0f) == 1.0f);
}

TEST(standing_again_refreshes_the_standing_share) {
    Rig r;
    r.run(0.50f, 0.0f, 2.0f);
    r.run(0.40f, 0.5f, 2.0f);
    r.v = 0.0f;                                                         // stopped
    const Out o = r.run(0.46f, 0.0f, 4.0f);                             // a restart, or another car: a different standing share
    CHECK(std::fabs(o.base - 0.46f) < 0.01f && o.light < 0.01f);
}

TEST(a_car_already_moving_learns_from_steady_driving_after_a_second) {
    Rig r; r.v = 25.0f;
    CHECK(!r.run(0.52f, 0.0f, 0.8f).learned);
    const Out o = r.run(0.52f, 0.0f, 0.5f);
    CHECK(o.learned && !o.fromRest && std::fabs(o.base - 0.52f) < 0.01f);
    CHECK(LightMul(r.run(0.42f, 0.4f, 1.0f), 1.0f) < 0.9f);
}

TEST(a_start_on_a_hill_is_corrected_once_the_car_rolls_on_the_flat) {
    Rig up;                                                              // standing uphill: weight to the rear, the standing front share reads low
    up.run(0.49f, 0.0f, 3.0f);
    up.run(0.49f, 0.4f, 3.0f); up.run(0.52f, 0.0f, 25.0f);              // then driving steadily on the flat
    CHECK(std::fabs(up.run(0.52f, 0.0f, 0.1f).base - 0.52f) < 0.01f);
    Rig down;                                                            // standing downhill: reads high, so a flat road looks like an unloaded front
    down.run(0.55f, 0.0f, 3.0f);
    down.run(0.52f, 0.4f, 3.0f);
    CHECK(LightMul(down.run(0.52f, 0.0f, 1.0f), 1.0f) > 0.99f);  // not accelerating, so nothing, whatever the baseline
    down.run(0.52f, 0.0f, 25.0f);
    CHECK(std::fabs(down.run(0.52f, 0.0f, 0.1f).base - 0.52f) < 0.01f);
}

TEST(the_slope_error_is_small_next_to_a_launch) {
    Rig flat, hill;
    flat.run(0.52f, 0.0f, 3.0f);
    hill.run(0.50f, 0.0f, 3.0f);                                         // a steep start: the standing share two points lower
    const float f = LightMul(flat.run(0.43f, 0.5f, 2.0f), 1.0f), h = LightMul(hill.run(0.43f, 0.5f, 2.0f), 1.0f);
    CHECK(f < 0.85f && h < 0.90f && std::fabs(f - h) < 0.06f);
}

TEST(a_long_hard_launch_does_not_teach_the_normal) {
    Rig r;
    r.run(0.52f, 0.0f, 3.0f);
    r.run(0.43f, 0.3f, 12.0f);                                           // twelve seconds of hard acceleration
    const Out o = r.run(0.43f, 0.3f, 0.1f);
    CHECK(std::fabs(o.base - 0.52f) < 0.01f && LightMul(o, 1.0f) < 0.85f);
}

TEST(cornering_does_not_teach_a_wrong_normal) {
    Rig r; r.v = 25.0f;
    r.run(0.52f, 0.0f, 3.0f);
    for (int i = 0; i < 600; ++i) { In in; Front(in, 0.45f); in.speed = 25.0f; in.throttle = 0.2f; in.accOk = true; in.accMag = 7.0f; r.t.Update(in, 0.01f); }
    CHECK(std::fabs(r.run(0.52f, 0.0f, 0.2f).base - 0.52f) < 0.01f);
}

TEST(nothing_before_anything_is_known_and_after_a_reset) {
    Rig r;
    const Out o = r.run(0.30f, 0.5f, 0.05f);                            // the very first reading is a car in the middle of a launch: no normal to compare with
    CHECK(!o.learned && LightMul(o, 2.0f) == 1.0f);
    Rig r2;
    r2.run(0.50f, 0.0f, 2.0f);
    r2.t.Reset();
    CHECK(LightMul(r2.run(0.30f, 0.5f, 0.05f), 2.0f) == 1.0f);
}

TEST(bad_input_never_makes_nan) {
    Rig r;
    const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
    r.run(0.52f, 0.0f, 1.0f);
    In bad; Front(bad, nan); bad.throttle = inf; bad.brake = nan; bad.speed = -inf; bad.airborne = nan; bad.accMag = nan; bad.accOk = true;
    for (int i = 0; i < 50; ++i) {
        const Out o = r.t.Update(bad, i % 2 ? nan : -1.0f);
        CHECK(std::isfinite(o.base) && std::isfinite(o.share) && std::isfinite(o.light) && std::isfinite(o.rawChange) && std::isfinite(o.accelG) && std::isfinite(o.gate) && o.light >= 0.0f);
    }
    CHECK(LightMul(r.t.Update(bad, 0.01f), nan) == 1.0f);
}



// ---- braking side of the same tracker: dive and heavy
// A car that stood on the line (so "normal" is learned), then drives at 25 m/s and brakes at `decelG`, with the front share at `share`.
struct BrakeRig {
    WeightTransfer::Tracker t; float v = 0; Out last;
    void stand(float share) { for (int i = 0; i < 100; ++i) { In in; Front(in, share); in.throttle = 0.1f; last = t.Update(in, 0.01f); } }
    void drive(float share, float secs, float v0) { v = v0; for (int i = 0; i < (int)(secs * 100); ++i) { In in; Front(in, share); in.speed = v; in.throttle = 0.3f; in.accOk = true; in.accMag = 0.2f; last = t.Update(in, 0.01f); } }
    void brake(float share, float decelG, float pedal, float secs) {
        for (int i = 0; i < (int)(secs * 100); ++i) {
            v = std::max(0.0f, v - decelG * 9.81f * 0.01f);
            In in; Front(in, share); in.speed = v; in.brake = pedal; in.accOk = true; in.accMag = decelG * 9.81f; last = t.Update(in, 0.01f);
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
    r.brake(0.60f, 0.0f, 0.0f, 1.0f);                   // share high, pedal off, speed steady (a downhill)
    CHECK(r.last.dive < 0.02f);
}
TEST(the_steering_gains_force_in_proportion_to_the_front_load_gained_when_the_car_really_slows_down) {
    BrakeRig r; r.stand(0.50f); r.drive(0.50f, 2.0f, 25.0f);
    r.brake(0.60f, 0.8f, 1.0f, 2.0f);                   // the front carries 20% more than standing
    CHECK(r.last.rawChange > 0.19f && r.last.rawChange < 0.21f);
    CHECK(std::fabs(HeavyMul(r.last, 1.0f) - (1.0f + (0.20f - WeightTransfer::kDeadband))) < 0.01f);
    CHECK(HeavyMul(r.last, 2.0f) > HeavyMul(r.last, 1.0f) && HeavyMul(r.last, 1.0f) > HeavyMul(r.last, 0.5f));
    CHECK(HeavyMul(r.last, 0.0f) == 1.0f && LightMul(r.last, 2.0f) == 1.0f);   // off: exactly RBR's force; and braking is never lightness
}
TEST(heavy_braking_needs_the_car_to_really_slow_down_but_not_the_pedal) {
    BrakeRig a; a.stand(0.50f); a.drive(0.50f, 2.0f, 25.0f);
    a.brake(0.60f, 0.02f, 1.0f, 1.5f);                  // pedal and weight, but the speed hardly changes (a short tap)
    CHECK(a.last.dive > 0.8f); CHECK(a.last.heavy < 0.05f);
    BrakeRig b; b.stand(0.50f); b.drive(0.50f, 2.0f, 25.0f);
    b.brake(0.60f, 0.8f, 1.0f, 1.5f);
    CHECK(b.last.heavy > 0.15f);
    BrakeRig c; c.stand(0.50f); c.drive(0.50f, 2.0f, 25.0f);
    c.brake(0.60f, 0.4f, 0.0f, 1.5f);                   // engine braking: slowing down with weight on the front and no pedal at all
    CHECK(c.last.heavy > 0.15f && c.last.dive < 0.02f);
}
TEST(acceleration_is_never_heaviness_and_braking_is_never_lightness) {
    BrakeRig r; r.stand(0.50f); r.drive(0.50f, 2.0f, 25.0f);
    r.brake(0.40f, 0.8f, 1.0f, 1.5f);                   // slowing down with the front share low (a crest)
    CHECK(LightMul(r.last, 2.0f) > 0.999f && HeavyMul(r.last, 2.0f) > 0.999f && r.last.light == 0.0f);
    Rig a; a.run(0.50f, 0.0f, 2.0f);
    const Out o = a.run(0.62f, 0.5f, 2.0f);             // accelerating with the front carrying more than usual
    CHECK(HeavyMul(o, 2.0f) < 1.001f && LightMul(o, 2.0f) > 0.999f);
}
TEST(braking_never_teaches_the_normal_share) {
    BrakeRig r; r.stand(0.50f); r.drive(0.50f, 2.0f, 25.0f);
    r.brake(0.62f, 0.8f, 1.0f, 3.0f);
    CHECK(std::fabs(r.last.base - 0.50f) < 0.01f);
}
TEST(braking_gains_are_zero_before_the_normal_is_known_and_in_the_air) {
    WeightTransfer::Tracker t; Out o;
    for (int i = 0; i < 20; ++i) { In in; Front(in, 0.7f); in.speed = 20.0f; in.brake = 1.0f; in.throttle = 0.8f; o = t.Update(in, 0.01f); }
    CHECK(o.dive == 0.0f && o.heavy == 0.0f);
    BrakeRig r; r.stand(0.50f); r.drive(0.50f, 2.0f, 25.0f);
    for (int i = 0; i < 100; ++i) { In in; Front(in, 0.62f); in.speed = 10.0f; in.brake = 1.0f; in.airborne = 1.0f; in.accOk = true; in.accMag = 8.0f; r.last = r.t.Update(in, 0.01f); }
    CHECK(r.last.heavy < 0.01f);
}
TEST(dive_and_heavy_survive_bad_input) {
    WeightTransfer::Tracker t; Out o;
    for (int i = 0; i < 50; ++i) { In in; Front(in, i % 2 ? std::numeric_limits<float>::quiet_NaN() : 0.6f); in.speed = i % 3 ? 20.0f : std::numeric_limits<float>::infinity(); in.brake = 1.0f; o = t.Update(in, i % 5 ? 0.01f : std::numeric_limits<float>::quiet_NaN()); }
    CHECK(std::isfinite(o.dive) && std::isfinite(o.heavy) && o.dive >= 0.0f && o.dive <= 1.0f && o.heavy >= 0.0f);
}

// A side load with the front share kept at one half: the right tyres carry (1 + T) / 2 of the weight, the left ones (1 - T) / 2.
static void Side(In& in, float t) { in.wload[0] = in.wload[2] = (1.0f - t) * 0.25f; in.wload[1] = in.wload[3] = (1.0f + t) * 0.25f; }

// A car on a test track at 100 Hz. The side load is given as a function of time; the lateral acceleration follows it (0.4 of side load is about 0.7 g).
struct SideRig {
    WeightTransfer::Tracker t; float time = 0, speed = 20.0f, air = 0.0f; bool latOk = true; unsigned seed = 1;
    float noise() { seed = seed * 1664525u + 1013904223u; return (float)((seed >> 8) & 0xFFFF) / 32767.5f - 1.0f; }
    // Runs for `secs`; fn(time since the call began) gives the side load. Returns the last output, and the largest swell and surge on the way.
    Out run(const std::function<float(float)>& fn, float secs, float* maxSwell = nullptr, float* maxSurge = nullptr, float noiseAmp = 0.0f, float latGOverride = -1.0f) {
        Out o; float ms = 0, mg = 0; const float t0 = time;
        for (int i = 0; i < (int)(secs * 100 + 0.5f); ++i) {
            const float v = fn(time - t0);
            In in; Side(in, v + noiseAmp * noise()); in.speed = speed; in.airborne = air; in.latOk = latOk;
            in.latG = latGOverride >= 0.0f ? latGOverride : std::fabs(v) * 1.8f;
            o = t.Update(in, 0.01f); time += 0.01f;
            ms = std::max(ms, o.swell); mg = std::max(mg, o.surge);
        }
        if (maxSwell) *maxSwell = ms;
        if (maxSurge) *maxSurge = mg;
        return o;
    }
};

// ---------------------------------------------------------------- across the car: swell in a corner, surge in a direction change
TEST(a_straight_road_adds_nothing) {
    SideRig r; float sw, sg;
    const Out o = r.run([](float) { return 0.0f; }, 3.0f, &sw, &sg);
    CHECK(sw == 0.0f && sg == 0.0f && CornerMul(o, 1.0f) == 1.0f);
}
TEST(a_held_corner_swells_the_force_and_it_eases_off_afterwards) {
    SideRig r; r.run([](float) { return 0.0f; }, 1.0f);
    const Out o = r.run([](float) { return 0.4f; }, 2.0f);
    CHECK(o.swell > 0.9f && o.surge < 0.05f);
    const float m = CornerMul(o, 1.0f);
    CHECK(m > 1.2f && m <= 1.0f + WeightTransfer::kMaxCorner + 1e-6f);
    const Out e = r.run([](float) { return 0.0f; }, 2.0f);
    CHECK(e.swell < 0.02f);
}
TEST(the_swell_comes_in_a_moment_after_the_load_not_at_once) {
    SideRig r; r.run([](float) { return 0.0f; }, 1.0f);
    const Out first = r.run([](float) { return 0.4f; }, 0.03f);
    CHECK(first.swell < 0.3f);
}
TEST(a_direction_change_surges_and_a_plain_corner_does_not) {
    SideRig corner; corner.run([](float) { return 0.0f; }, 1.0f);
    float sw, sg;
    corner.run([](float) { return 0.4f; }, 2.0f, &sw, &sg);
    CHECK(sg < 0.05f);                                                     // building up and holding a corner is not a surge
    SideRig flick; flick.run([](float) { return 0.4f; }, 2.0f);
    float swF, sgF;
    flick.run([](float) { return 0.4f; }, 0.01f);
    flick.run([](float s) { return std::max(-0.4f, 0.4f - 0.8f * s / 0.3f); }, 1.0f, &swF, &sgF);     // the load crosses the car in 0.3 s
    CHECK(sgF > 0.5f);
}
TEST(a_slow_swing_surges_less_than_a_quick_one) {
    auto swing = [](float secs) { SideRig r; r.run([](float) { return 0.4f; }, 2.0f); float sg; r.run([secs](float s) { return std::max(-0.4f, 0.4f - 0.8f * s / secs); }, 3.0f, nullptr, &sg); return sg; };
    CHECK(swing(0.3f) > swing(1.5f) + 0.5f && swing(1.5f) < 0.1f);
}
TEST(it_works_the_same_whichever_side_the_load_goes_to) {
    SideRig a, b; a.run([](float) { return 0.0f; }, 1.0f); b.run([](float) { return 0.0f; }, 1.0f);
    const Out oa = a.run([](float) { return 0.4f; }, 2.0f), ob = b.run([](float) { return -0.4f; }, 2.0f);
    CHECK(std::fabs(oa.swell - ob.swell) < 0.01f);
}
TEST(rough_road_noise_on_a_straight_does_nothing) {
    for (float amp : {0.1f, 0.3f, 0.6f}) {
        SideRig r; float sw, sg;
        r.run([](float) { return 0.0f; }, 1.0f);
        r.run([](float) { return 0.0f; }, 10.0f, &sw, &sg, amp, 0.02f);       // noisy strut forces, barely any lateral acceleration
        CHECK(sw == 0.0f && sg == 0.0f);
    }
}
TEST(the_standing_offset_is_taken_off) {
    SideRig r; r.speed = 0.0f;
    r.run([](float) { return 0.12f; }, 1.5f);                              // the standing car leans a little to one side
    r.speed = 20.0f;
    float sw, sg;
    r.run([](float) { return 0.12f; }, 3.0f, &sw, &sg, 0.0f, 0.5f);       // driving straight with the same lean
    CHECK(sw < 0.02f && sg < 0.02f);
}
TEST(nothing_is_added_without_known_units_in_the_air_or_at_a_crawl) {
    { SideRig r; r.latOk = false; float sw; r.run([](float) { return 0.4f; }, 2.0f, &sw); CHECK(sw == 0.0f); }
    { SideRig r; r.air = 1.0f; float sw; r.run([](float) { return 0.4f; }, 2.0f, &sw); CHECK(sw == 0.0f); }
    { SideRig r; r.speed = 1.0f; float sw; r.run([](float) { return 0.4f; }, 2.0f, &sw); CHECK(sw == 0.0f); }
}
TEST(the_gain_scales_it_and_never_goes_past_the_limit) {
    Out o; o.swell = 1.0f; o.surge = 1.0f;
    CHECK(CornerMul(o, 0.0f) == 1.0f);
    CHECK(CornerMul(o, 2.0f) <= 1.0f + WeightTransfer::kMaxCorner + 1e-6f);
    Out h; h.swell = 0.5f;
    CHECK(std::fabs(CornerMul(h, 1.0f) - (1.0f + 0.5f * WeightTransfer::kSwellAdd)) < 1e-6f);
    CHECK(CornerMul(h, 2.0f) > CornerMul(h, 1.0f));
}
TEST(broken_numbers_change_nothing) {
    SideRig r; r.run([](float) { return 0.0f; }, 1.0f);
    Out o; In in; for (int w = 0; w < 4; ++w) in.wload[w] = kNaN;
    in.speed = kNaN; in.airborne = kNaN; in.latG = kNaN; in.latOk = true; in.throttle = kNaN; in.brake = kNaN; in.accMag = kNaN; in.accOk = true;
    o = r.t.Update(in, kNaN);
    CHECK(std::isfinite(o.swell) && std::isfinite(o.surge) && o.swell == 0.0f && o.surge == 0.0f);
    Out bad; bad.swell = kNaN; bad.surge = kNaN; bad.light = kNaN; bad.heavy = kNaN;
    CHECK(WeightTransfer::Mul(bad, Gains()) == 1.0f);
    Gains nan; nan.light = nan.heavy = nan.corner = kNaN;
    CHECK(WeightTransfer::Mul(Out(), nan) == 1.0f);
}
TEST(a_reset_forgets_everything) {
    SideRig r; r.run([](float) { return 0.4f; }, 2.0f);
    r.t.Reset();
    const Out o = r.t.Update(In(), 0.01f);
    CHECK(o.swell == 0.0f && o.surge == 0.0f);
}


// ---------------------------------------------------------------- the one function
// Out as the tracker would hand it on: the front load lost or gained, and the cornering weight.
static Out Made(float light, float heavy, float swell = 0.0f, float surge = 0.0f) { Out o; o.light = light; o.heavy = heavy; o.swell = swell; o.surge = surge; return o; }

TEST(nothing_moving_is_exactly_rbrs_force_whatever_the_gains) {
    Gains g; g.light = g.heavy = g.corner = 2.0f;
    CHECK(WeightTransfer::Mul(Out(), g) == 1.0f);
    CHECK(WeightTransfer::Mul(Out(), Gains()) == 1.0f);
}
TEST(one_proportion_for_both_directions_lighter_when_unloaded_heavier_when_loaded) {
    Gains g; g.light = 1.0f; g.heavy = 1.0f; g.corner = 0.0f;
    CHECK(std::fabs(WeightTransfer::Mul(Made(0.2f, 0.0f), g) - 0.8f) < 1e-6f);      // the front lost 20% of its load: the steering loses 20%
    CHECK(std::fabs(WeightTransfer::Mul(Made(0.0f, 0.2f), g) - 1.2f) < 1e-6f);      // the front gained 20%: the steering gains 20%
}
TEST(the_function_is_continuous_and_rises_with_the_front_load_all_the_way_through_normal) {
    Gains g; g.light = 1.0f; g.heavy = 1.0f; g.corner = 0.0f;
    float prev = -1.0f, maxStep = 0.0f;
    for (int i = -50; i <= 50; ++i) {
        const float x = (float)i * 0.01f;                                            // the front load against normal, -50% .. +50%
        const float m = WeightTransfer::Mul(Made(std::max(0.0f, -x), std::max(0.0f, x)), g);
        if (prev >= 0.0f) { CHECK(m >= prev - 1e-6f); maxStep = std::max(maxStep, m - prev); }
        prev = m;
    }
    CHECK(maxStep < 0.0101f);
}
TEST(the_two_front_gains_are_independent_and_the_limits_hold) {
    Gains light; light.light = 1.0f; light.heavy = 0.0f; light.corner = 0.0f;
    Gains heavy; heavy.light = 0.0f; heavy.heavy = 1.0f; heavy.corner = 0.0f;
    CHECK(WeightTransfer::Mul(Made(0.0f, 0.3f), light) == 1.0f && WeightTransfer::Mul(Made(0.3f, 0.0f), heavy) == 1.0f);
    CHECK(WeightTransfer::Mul(Made(0.3f, 0.0f), light) < 0.75f && WeightTransfer::Mul(Made(0.0f, 0.3f), heavy) > 1.25f);
    Gains big; big.light = 2.0f; big.heavy = 2.0f; big.corner = 0.0f;
    CHECK(WeightTransfer::Mul(Made(0.9f, 0.0f), big) >= 1.0f - WeightTransfer::kMaxCut - 1e-6f);
    CHECK(WeightTransfer::Mul(Made(0.0f, 0.9f), big) <= 1.0f + WeightTransfer::kMaxHeavy + 1e-6f);
}
TEST(trail_braking_is_the_front_load_and_the_cornering_weight_together) {
    Gains g; g.light = 1.0f; g.heavy = 1.0f; g.corner = 1.0f;
    const float braking = WeightTransfer::Mul(Made(0.0f, 0.2f), g), corner = WeightTransfer::Mul(Made(0.0f, 0.0f, 1.0f), g);
    CHECK(std::fabs(corner - (1.0f + WeightTransfer::kSwellAdd)) < 1e-6f);
    CHECK(std::fabs(WeightTransfer::Mul(Made(0.0f, 0.2f, 1.0f), g) - braking * corner) < 1e-6f);
}
TEST(the_cornering_weight_never_goes_past_its_limit_whatever_the_gain) {
    Gains g; g.light = 0.0f; g.heavy = 0.0f; g.corner = 2.0f;
    CHECK(WeightTransfer::Mul(Made(0.0f, 0.0f, 1.0f, 1.0f), g) <= 1.0f + WeightTransfer::kMaxCorner + 1e-6f);
    g.corner = 0.0f;
    CHECK(WeightTransfer::Mul(Made(0.0f, 0.0f, 1.0f, 1.0f), g) == 1.0f);
}
TEST(a_drive_through_acceleration_braking_and_a_corner_stays_smooth) {
    // Stand, accelerate (the front unloads), cruise, brake hard (the front loads up) while turning in: the force goes below 1, back through 1, above 1, never snapping.
    WeightTransfer::Tracker t; float v = 0.0f, time = 0.0f, prev = 1.0f, lo = 1.0f, hi = 1.0f, maxStep = 0.0f; Gains g; g.heavy = 1.0f;
    auto step = [&](float share, float aG, float side, float thr, float brk) {
        v = std::max(0.0f, v + aG * 9.81f * 0.01f);
        In in; Front(in, share); in.wload[0] -= side * 0.25f; in.wload[2] -= side * 0.25f; in.wload[1] += side * 0.25f; in.wload[3] += side * 0.25f;
        in.speed = v; in.throttle = thr; in.brake = brk; in.accOk = true; in.accMag = std::fabs(aG) * 9.81f; in.latOk = true; in.latG = std::fabs(side) * 1.8f;
        const float m = WeightTransfer::Mul(t.Update(in, 0.01f), g); time += 0.01f;
        maxStep = std::max(maxStep, std::fabs(m - prev)); prev = m; lo = std::min(lo, m); hi = std::max(hi, m);
    };
    for (int i = 0; i < 200; ++i) step(0.52f, 0.0f, 0.0f, 0.1f, 0.0f);
    for (int i = 0; i < 400; ++i) step(0.42f, 0.5f, 0.0f, 1.0f, 0.0f);
    CHECK(lo < 0.85f);
    for (int i = 0; i < 300; ++i) step(0.52f, 0.0f, 0.0f, 0.3f, 0.0f);
    const float cruise = prev;
    for (int i = 0; i < 300; ++i) step(0.64f, -0.8f, 0.3f, 0.0f, 1.0f);
    CHECK(hi > 1.2f && cruise > 0.98f && cruise < 1.02f && maxStep < 0.05f);
}

int main() { return mini::RunAll(); }
