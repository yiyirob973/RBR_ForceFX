// How the plugin's scalings of RBR's own steering force are combined, and kept in bounds. Pure logic, no Windows or game access, tested in
// tests/force_scale_test.cpp.
//
// Landing, water, the weight (front load and side load), brake lock-up and crashes all scale RBR's own force. Each is bounded on its own,
// but together their product has no limit, and none of it is seen by the total-output ceiling or the thermal derate (those look at the effects the
// plugin adds). So the product gets its own bounds here:
//   - it is never below kMinMul (the wheel is never left nearly empty) and never above the cap (MaxForceMultiplier, itself never above kHardMulMax);
//   - it can not jump: it rises at most kRiseRate per second and falls at most kFallRate per second, which is quicker than any single scaling moves
//     at the defaults, so it only holds back an extreme setting;
//   - the minimum extra push is capped at kHardAddMax of full torque.
#pragma once
#include <algorithm>
#include <cmath>

namespace ForceScale {

constexpr float kHardMulMax = 4.0f;    // the cap can never be set above this
constexpr float kDefaultCap = 3.0f;
constexpr float kMinMul = 0.25f;       // the force is never scaled below this
constexpr float kHardAddMax = 0.5f;    // the extra push never exceeds this share of full torque
constexpr float kRiseRate = 150.0f;    // the multiplier rises at most this much per second
constexpr float kFallRate = 40.0f;     // ... and falls at most this much per second

struct Parts {                          // each is 1 = no change
    float landing = 1, waterKick = 1, waterHeavy = 1, frontLoad = 1, lateral = 1, brakeLock = 1, crash = 1;
};

namespace detail {
inline float one(float v) { return std::isfinite(v) && v > 0.0f ? v : 1.0f; }       // a broken part counts as "no change"
}

inline float Product(const Parts& p) {
    using namespace detail;
    return one(p.landing) * one(p.waterKick) * one(p.waterHeavy) * one(p.frontLoad) * one(p.lateral) * one(p.brakeLock) * one(p.crash);
}

inline float Cap(float cap) { return std::isfinite(cap) ? std::max(1.0f, std::min(kHardMulMax, cap)) : kDefaultCap; }

inline float Add(float add) { return std::isfinite(add) && add > 0.0f ? std::min(add, kHardAddMax) : 0.0f; }

class Limiter {
public:
    void Reset() { cur_ = 1.0f; }
    float Value() const { return cur_; }
    // target: the product of the parts. Returns the multiplier to use now.
    float Step(float target, float dt, float cap) {
        const float hi = Cap(cap);
        const float t = std::isfinite(target) ? std::max(kMinMul, std::min(hi, target)) : 1.0f;
        const float d = (std::isfinite(dt) && dt > 0.0f) ? std::min(dt, 0.1f) : 0.01f;
        cur_ += std::max(-kFallRate * d, std::min(kRiseRate * d, t - cur_));
        cur_ = std::max(kMinMul, std::min(hi, cur_));                              // a lower cap takes hold at once
        return cur_;
    }
private:
    float cur_ = 1.0f;
};

}  // namespace ForceScale
