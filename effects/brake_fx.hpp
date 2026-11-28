// Brake feel, built from the same signals as the rest of the plugin. Pure logic, no Windows or game access, tested in tests/brake_fx_test.cpp.
//
// Two parts:
//   JUDDER     the front-end shake under braking (strength from the pedal, the weight that has moved to the front tyres and any lock-up; the
//              pitch from the front wheels, or a lock-up scrub whose pitch follows the ground). It rides on RBR's own force like the road
//              vibration does, as one more band of it, instead of being a separate effect on the wheel.
//   LOCK-UP    when the front tyres lock, they stop gripping and the steering goes light. LockMul scales RBR's force down by how far the front
//              tyres are locked, the same way the front load does in weight_transfer.hpp (which also owns the heavier steering in hard braking).
#pragma once
#include <algorithm>
#include <cmath>

namespace BrakeFx {

constexpr float kLockCut = 0.5f;       // at gain 1 and both front tyres fully locked, RBR's force loses this share ...
constexpr float kLockCutMax = 0.7f;    // ... and never more than this, whatever the gain

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
inline float gain(float g) { return std::isfinite(g) && g > 0.0f ? (g > 2.0f ? 2.0f : g) : 0.0f; }
}  // namespace detail

// How far the front tyres are locked, 0..1, from the per-tyre lock-up (FL, FR, RL, RR). The rear tyres are felt through the car, not the steering.
inline float FrontLock(const float lock[4]) { return detail::c01(0.5f * (lock[0] + lock[1])); }

// RBR's force times this (<= 1) while the front tyres are locked. gain: 0 = off, 1 = normal, at most 2.
inline float LockMul(float frontLock, float gain) {
    const float cut = detail::gain(gain) * kLockCut * detail::c01(frontLock);
    return 1.0f - (cut > kLockCutMax ? kLockCutMax : cut);
}

// The pitch of the lock-up scrub (Hz). Hard ground chatters higher, loose and soft ground scrubs lower and coarser; r is a random number in -1..1.
// On gravel (hard 0.4, loose 0.6) this centres on about 35 Hz, which is where the old fixed 38 Hz sat.
inline float LockHz(float hard, float loose, float r) {
    const float h = detail::c01(hard), l = detail::c01(loose), rr = std::isfinite(r) ? std::max(-1.0f, std::min(1.0f, r)) : 0.0f;
    return std::max(25.0f, std::min(70.0f, 36.0f + 24.0f * (h - 0.4f) - 8.0f * (l - 0.5f) + 25.0f * rr));
}

// Strength of the judder, 0..1: the pedal, the weight on the front tyres and lock-up (already scaled by the ground).
inline float JudderAmp(float brake, float dive, float lockU) {
    using namespace detail;
    return c01(std::pow(c01(brake), 1.2f) * (0.3f + 0.7f * c01(dive)) * 0.6f + c01(lockU));
}

// Pitch of the judder (Hz): a lock-up scrub, or else twice the front wheel's turn rate. hzScale is the setting.
inline float JudderHz(float lockU, float frontRev, float hard, float loose, float r, float hzScale) {
    const float hz = lockU > 0.1f ? LockHz(hard, loose, r)
                                  : std::max(12.0f, std::min(60.0f, std::isfinite(frontRev) ? frontRev * 2.0f : 12.0f));
    return hz * (std::isfinite(hzScale) && hzScale > 0.0f ? hzScale : 1.0f);
}

}  // namespace BrakeFx
