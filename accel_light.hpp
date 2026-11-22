// Light steering under hard acceleration: the front axle unloads as weight transfers to the rear, so the tyres have less to push against and
// the steering goes light. This scales RBR's own steering force down by how far the front load has dropped below what it normally is, and never
// adds a force of its own. Pure logic, no Windows or game access, tested in tests/accel_light_test.cpp.
//
// RBR's physics already unloads the front under acceleration, so its force may already go light; that is why the effect is capped, adjustable and
// has its own switch, and why Out reports the measured front-load drop (the plugin logs it, so you can see how much RBR does by itself).
//   - The normal front share of the car's weight is learned while driving steadily (no hard acceleration, braking or cornering, on the ground).
//     Until it is learned (about 1.5 s of steady driving) the effect is off.
//   - Only a drop below normal counts, and only while the throttle is down and the brakes are off: a crest or a bump that unloads the front for a
//     moment under no throttle does nothing, and neither does braking (that moves the weight forward).
//   - It fades in over about a fifth of a second and out over a third, so it never snaps.
#pragma once
#include <algorithm>
#include <cmath>

namespace AccelLight {

constexpr float kFullDrop = 0.15f;     // front share this far below normal (relative, 0.15 = 15%) is the full effect
constexpr float kMaxAmount = 0.6f;     // the setting never takes more than this share of RBR's force away

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
inline float smooth01(float a, float b, float x) { const float t = c01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
inline float alphaFor(float dt, float tau) { return 1.0f - std::exp(-dt / tau); }
}  // namespace detail

struct In {
    float frontShare = 0.5f;   // share of the car's weight on the two front tyres, 0..1
    float throttle = 0.0f, brake = 0.0f;
    float speed = 0.0f;        // m/s over the ground
    float airborne = 0.0f;     // 0 on the ground .. 1 in the air
    float accMag = 0.0f;       // the car's acceleration over the ground in m/s^2 ...
    bool  accOk = false;       // ... when the units are known (without it, steady driving is judged by the pedals alone)
};
struct Out {
    bool  learned = false;     // the normal front share is known
    float base = 0.0f;         // the normal front share
    float drop = 0.0f;         // how far the front share is below normal right now, relative, smoothed, and at most kFullDrop (0 = at normal or above)
    float rawDrop = 0.0f;      // the same, as measured: not smoothed and not limited (0.2 = the front carries 20% less of the weight than normal). For the log
    float effect = 0.0f;       // 0..1: how much of the full effect is on (drop, throttle and the other conditions together)
};

class Tracker {
public:
    static constexpr float kLearnNeeded = 1.5f;   // seconds of steady driving before the normal share is trusted

    void Reset() { *this = Tracker(); }

    Out Update(const In& in, float dt) {
        using namespace detail;
        Out o;
        if (!std::isfinite(dt) || dt <= 0.0f) dt = 0.01f;
        if (dt > 0.1f) dt = 0.1f;
        const float share = std::isfinite(in.frontShare) ? std::min(0.95f, std::max(0.05f, in.frontShare)) : 0.5f;
        const float thr = c01(in.throttle), brk = c01(in.brake), air = c01(in.airborne);
        const float spd = std::isfinite(in.speed) && in.speed > 0.0f ? in.speed : 0.0f;

        // Learn what the front share normally is: steady driving on the ground, gentle on the pedals, not cornering or accelerating hard.
        const bool steady = spd > 3.0f && air < 0.1f && thr < 0.35f && brk < 0.05f && (!in.accOk || (std::isfinite(in.accMag) && in.accMag < 1.5f));
        if (steady) {
            learnT_ += dt;
            const float tau = learnT_ < 1.0f ? 0.3f : 4.0f;                      // quick at first, then slow and steady
            if (!haveBase_) { base_ = share; haveBase_ = true; }
            else base_ += (share - base_) * alphaFor(dt, tau);
        }
        o.learned = haveBase_ && learnT_ >= kLearnNeeded;
        o.base = base_;

        float raw = 0.0f;
        if (o.learned && base_ > 0.05f) {
            o.rawDrop = std::max(0.0f, (base_ - share) / base_);
            raw = c01(o.rawDrop / kFullDrop);                                     // 1 = the full drop or more
        }
        const float k = alphaFor(dt, raw > drop_ ? 0.20f : 0.35f);
        drop_ += (raw - drop_) * k;
        if (!(drop_ > 0.0f)) drop_ = 0.0f;
        o.drop = drop_ * kFullDrop;

        const float gate = smooth01(0.35f, 0.80f, thr) * (1.0f - smooth01(0.05f, 0.30f, brk)) * (1.0f - air) * smooth01(2.0f, 8.0f, spd);
        const float target = raw * gate;
        effect_ += (target - effect_) * alphaFor(dt, target > effect_ ? 0.20f : 0.35f);   // lifting off the throttle fades out too, it does not snap
        if (!(effect_ > 0.0f)) effect_ = 0.0f;
        o.effect = effect_ > 1.0f ? 1.0f : effect_;
        return o;
    }

private:
    bool haveBase_ = false;
    float base_ = 0.5f, learnT_ = 0.0f, drop_ = 0.0f, effect_ = 0.0f;
};

// RBR's force times this (<= 1). `amount` is the setting: 0 = off, at most 0.6 (never more than 60% off).
inline float Mul(const Out& o, float amount) {
    const float a = std::isfinite(amount) && amount > 0.0f ? (amount > kMaxAmount ? kMaxAmount : amount) : 0.0f;
    return 1.0f - a * detail::c01(o.effect);
}

}  // namespace AccelLight
