// Road guard: keeps the road vibration from turning into a push or pull on the wheel. Pure logic, no Windows or game access, so it
// can be tested on its own (see tests/road_guard_test.cpp). Included in RBR_ForceFX.cpp before the effect hook, which needs it.
#pragma once
#include <cmath>

namespace Road {

// ---------------------------------------------------------------- keeping the road from pushing or pulling the wheel
// The road is meant to be a vibration only. Three things can turn it into a push that is felt as resistance when you steer, and each
// has a guard here.

// 1. A low cut. Random strength and pitch (the grain of the ground) leak a little slow movement into the sines, and a slow push
//    is a shove, not a texture. Two gentle high-pass stages at kLowCutHz remove it; the road range (25 Hz and up) loses about 5%.
constexpr float kLowCutHz = 6.0f;
struct LowCut {
    float x1[2] = {0.0f, 0.0f}, y1[2] = {0.0f, 0.0f};
    bool init = false;
    void Reset() { x1[0] = x1[1] = y1[0] = y1[1] = 0.0f; init = false; }
    // dt is the real time since the last call, so uneven sending does not change the cut-off.
    float Step(float in, float dt) {
        if (!std::isfinite(in)) in = 0.0f;
        if (!std::isfinite(dt) || dt < 0.0005f) dt = 0.0005f;
        if (dt > 0.03f) dt = 0.03f;
        if (!init) { init = true; x1[0] = x1[1] = in; y1[0] = y1[1] = 0.0f; return 0.0f; }
        // bilinear transform: the corner stays where it is however uneven dt is, and the road range keeps its full gain
        const float k = std::tan(3.14159265f * kLowCutHz * dt), g = 1.0f / (1.0f + k), p = (1.0f - k) / (1.0f + k);
        float v = in;
        for (int i = 0; i < 2; ++i) { const float y = g * (v - x1[i]) + p * y1[i]; x1[i] = v; y1[i] = y; v = y; }
        return v;
    }
};

// 2. Headroom. RBR's force plus the vibration must fit in the wheel's range. Cutting off only the top of the sum shifts the average
//    force; limiting the vibration itself, equally in both directions, keeps it centred on RBR's force.
inline int FitHeadroom(int base, int vib, int fullScale = 10000) {
    int head = fullScale - (base < 0 ? -base : base);
    if (head < 0) head = 0;
    return vib > head ? head : (vib < -head ? -head : vib);
}

// 3. A ceiling on very rough ground. The strength of the three bands together (their combined RMS) passes unchanged up to
//    kCeilKnee of the road gain and is squeezed smoothly towards kCeilMax of it, so stones, kerbs and washboard can stack up
//    without the road growing without limit. All three bands are scaled by the same factor, so the character does not change.
constexpr int kBands = 3;   // same value as in road_bands.hpp
constexpr float kCeilKnee = 0.45f, kCeilMax = 0.70f;
inline float CeilingScale(const float mag[kBands], float gain) {
    if (!(gain > 0.0f) || !std::isfinite(gain)) return 1.0f;
    float e = 0.0f;
    for (int b = 0; b < kBands; ++b) { const float m = (std::isfinite(mag[b]) && mag[b] > 0.0f) ? mag[b] : 0.0f; e += m * m; }
    const float rms = std::sqrt(e), k = kCeilKnee * gain, L = kCeilMax * gain;
    if (rms <= k) return 1.0f;
    return (k + (L - k) * std::tanh((rms - k) / (L - k))) / rms;
}

// 4. The landing boost is for jumps. Rough ground drops the load on all four wheels for a moment, and bumps and washboard fired
//    boost after boost, which is felt as a wheel that is heavy to turn. A landing only counts after this much time off the ground.
constexpr float kBoostMinAir = 0.15f;
inline bool LandingBoostWorthy(float airSeconds) { return std::isfinite(airSeconds) && airSeconds >= kBoostMinAir; }

}  // namespace Road
