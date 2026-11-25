// The wheel's reaction to a crash: a short jolt on RBR's own force, started by a crash event with a severity (the detector is audio/crash.hpp,
// which also drives the crash sounds). Same path as a landing: the force is multiplied and gets a minimum extra push for a moment, there is no
// effect of its own. Pure logic, tested in tests/crash_fx_test.cpp.
//   - a fast attack, a short hold, then a quick decay (a hit is over in a fraction of a second);
//   - a new event restarts it, a weaker one never cuts a stronger one short;
//   - the live gain scales it, and no gain, no severity or a broken number gives nothing.
#pragma once
#include <algorithm>
#include <cmath>

namespace CrashFx {

constexpr float kHold = 0.03f, kDecay = 0.15f, kLife = 1.0f;   // seconds

class Envelope {
public:
    void Reset() { *this = Envelope(); }
    // seq: goes up by one per crash (0 = no telemetry). sev: 0..1. Returns the level, 0..1.
    float Step(unsigned seq, float sev, float dt, float userGain) {
        if (!std::isfinite(dt) || dt <= 0.0f) dt = 0.01f;
        const float s = std::isfinite(sev) ? std::max(0.0f, std::min(1.0f, sev)) : 0.0f;
        if (seq != seen_) {
            seen_ = seq;
            if (seq != 0 && s > 0.0f) { age_ = 0.0f; amp_ = std::max(amp_ * env(age_), s); }
        }
        age_ = std::min(age_ + dt, kLife);
        const float g = std::isfinite(userGain) ? std::max(0.0f, std::min(1.0f, userGain)) : 0.0f;
        return age_ < kLife ? std::min(1.0f, amp_ * env(age_)) * g : 0.0f;
    }
private:
    static float env(float age) { return age < kHold ? 1.0f : std::exp(-(age - kHold) / kDecay); }
    unsigned seen_ = 0; float age_ = kLife, amp_ = 0.0f;
};

inline float Mul(float level, float boost) {      // RBR's force times this (>= 1)
    const float b = std::isfinite(boost) ? std::max(0.0f, std::min(3.0f, boost)) : 0.0f;
    return 1.0f + b * std::max(0.0f, std::min(1.0f, level));
}
inline float Add(float level, float minPush) {    // extra push, a share of full torque
    const float p = std::isfinite(minPush) ? std::max(0.0f, std::min(1.0f, minPush)) : 0.0f;
    return p * std::max(0.0f, std::min(1.0f, level));
}

}  // namespace CrashFx
