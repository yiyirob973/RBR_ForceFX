// WEIGHT TRANSFER: where the car's weight is, felt as a change in how heavy RBR's own steering force is. Pure logic, no Windows or game access,
// tested in tests/weight_transfer_test.cpp. Nothing here adds a force of its own or pulls the wheel in a direction: RBR's force is only scaled,
// so it always points the way RBR sent it.
//
//   FRONT LOAD   weight moving along the car. FrontMul combines the two directions in one multiplier: lighter when acceleration unloads the front
//                (accel_light.hpp) and heavier in hard braking (brake_fx.hpp). It is the same measurement either way up, and each side only
//                switches on while the ground speed says the car is really accelerating or slowing, so it is continuous through zero.
//   SIDE LOAD    weight moving across the car, read from the four strut forces as (right share - left share). Which side it is does not matter,
//                only how far it has moved, so there is nothing to calibrate but the small offset the car has standing still.
//     swell      the weight that has built up on the outside tyres in a corner: RBR's force gets a little heavier, following the body roll
//                a moment after the steering, and eases off as the corner ends.
//     surge      the weight swinging from one side to the other in a direction change (the pendulum of a flick): RBR's force swells while
//                the load crosses the car. It is detected as the quick average of the side load having the opposite sign to the slow one.
//   Both side effects need real lateral acceleration (from 0.08 g, fully at 0.25 g), because the strut forces are noisy on a rough road and the
//   noise alone would otherwise swell the steering on a straight. They are off in the air and fade in with speed.
#pragma once
#include <algorithm>
#include <cmath>
#include "accel_light.hpp"
#include "brake_fx.hpp"

namespace WeightTransfer {

constexpr float kSwellAdd = 0.25f;     // at gain 1 and full cornering load, RBR's force grows by this share ...
constexpr float kSurgeAdd = 0.35f;     // ... and by this much while the load swings across the car
constexpr float kMaxAdd = 0.5f;        // never more than this share of RBR's force from the side load, whatever the gain

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
inline float alphaFor(float dt, float tau) { return 1.0f - std::exp(-dt / tau); }
inline float smooth01(float a, float b, float x) { const float t = c01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
inline float gain(float g) { return std::isfinite(g) && g > 0.0f ? (g > 2.0f ? 2.0f : g) : 0.0f; }
}  // namespace detail

// RBR's force times this, for the front load: below 1 while acceleration unloads the front, above 1 in hard braking. Each gain: 0 = off, 1 = normal, at most 2.
inline float FrontMul(const AccelLight::Out& o, float lightGain, float heavyGain) {
    return AccelLight::Mul(o, lightGain) * BrakeFx::HeavyMul(o.heavy, heavyGain);
}

struct In {
    float lateral = 0.0f;      // (share of the weight on the right tyres) - (share on the left tyres), -1..1
    float speed = 0.0f;        // m/s over the ground
    float airborne = 0.0f;     // 0 on the ground .. 1 in the air
    float latG = 0.0f;         // lateral acceleration in g (absolute) ...
    bool  latOk = false;       // ... when its units are known; without it the side load is not trusted and nothing is added
};
struct Out {
    float load = 0.0f;         // the side load now, quick average, minus the standing offset (for the log)
    float swell = 0.0f;        // 0..1: weight built up on the outside tyres, smoothed
    float surge = 0.0f;        // 0..1: weight swinging across the car, smoothed
    float gate = 0.0f;         // 0..1: how far the lateral acceleration and the speed let the effect through
};

class Lateral {
public:
    void Reset() { *this = Lateral(); }

    Out Update(const In& in, float dt) {
        using namespace detail;
        Out o;
        if (!std::isfinite(dt) || dt <= 0.0f) dt = 0.01f;
        if (dt > 0.1f) dt = 0.1f;
        const float raw = std::isfinite(in.lateral) ? std::max(-1.0f, std::min(1.0f, in.lateral)) : 0.0f;
        const float spd = std::isfinite(in.speed) && in.speed > 0.0f ? in.speed : 0.0f;
        const float air = c01(in.airborne);

        // The standing car is not perfectly even (fuel, driver, a slope): that offset is taken off, learned only while the car stands.
        if (spd < 0.5f && air < 0.1f) { if (!haveOff_) { off_ = raw; haveOff_ = true; } else off_ += (raw - off_) * alphaFor(dt, 0.5f); }
        const float x = raw - off_;
        if (!haveSm_) { fast_ = slow_ = x; haveSm_ = true; }
        else { fast_ += (x - fast_) * alphaFor(dt, 0.10f); slow_ += (x - slow_) * alphaFor(dt, 0.35f); }

        // Lateral acceleration, held over about a third of a second so it stays on while it passes through zero in a direction change.
        latG_ += ((in.latOk && std::isfinite(in.latG) ? in.latG : 0.0f) - latG_) * alphaFor(dt, 0.3f);
        const float gate = smooth01(0.08f, 0.25f, latG_) * smooth01(3.0f, 10.0f, spd) * (1.0f - air);
        o.gate = gate; o.load = fast_;

        const float swellT = smooth01(0.05f, 0.45f, std::fabs(fast_)) * gate;
        const float cross = std::sqrt(std::max(0.0f, -fast_ * slow_));      // > 0 only while the quick and the slow average disagree in sign
        const float surgeT = smooth01(0.03f, 0.18f, cross) * gate;
        swell_ += (swellT - swell_) * alphaFor(dt, swellT > swell_ ? 0.10f : 0.20f);   // follows the body roll, a little behind the steering
        surge_ += (surgeT - surge_) * alphaFor(dt, surgeT > surge_ ? 0.05f : 0.15f);
        o.swell = swell_; o.surge = surge_;
        return o;
    }

private:
    bool haveOff_ = false, haveSm_ = false;
    float off_ = 0.0f, fast_ = 0.0f, slow_ = 0.0f, latG_ = 0.0f, swell_ = 0.0f, surge_ = 0.0f;
};

// RBR's force times this (>= 1) for the side load. gain: 0 = off, 1 = normal, at most 2.
inline float LateralMul(const Out& o, float g) {
    const float add = detail::gain(g) * (kSwellAdd * detail::c01(o.swell) + kSurgeAdd * detail::c01(o.surge));
    return 1.0f + (add > kMaxAdd ? kMaxAdd : add);
}

}  // namespace WeightTransfer
