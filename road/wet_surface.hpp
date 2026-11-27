// What damp and wet ground do to the tyre sounds, worked out in code from the dry recordings. Pure logic, no Windows or audio access,
// tested in tests/wet_surface_test.cpp.
//
// RBR's sound files have one loop per surface (gravel, tarmac, ...), so damp and wet gravel used to play the dry gravel loop. The game does
// say how wet the ground under each tyre is (the material names end in _Damp or _Wet), and what wet ground does to the sound is simple:
//   - loose ground loses its crunch: the stones stick together, so the loop gets duller and a little quieter;
//   - water on the surface adds a soft hiss that grows with speed (the spray under the tyre), most clearly on hard ground;
//   - flung stones are fewer, softer and dull, and some of them land in water.
// A recording of wet ground (gravel_wet_loop.wav, ...) already contains all that, so none of it is added on top of one: `nativeShare` is how
// much of the tyre's sound comes from such a file. The stones are the exception, they are separate sounds either way.
#pragma once
#include <algorithm>
#include <cmath>

namespace WetSurface {

struct Params {
    bool on = true;           // WetTreatment
    float tone = 0.6f;        // how dull loose ground gets when fully wet (0 = not at all, 1 = as dull as the mixer can make it)
    float crunch = 0.3f;      // how much quieter the dry loop gets on loose ground when fully wet
    float hiss = 0.5f;        // level of the spray hiss, as a share of the loop level
    float stones = 0.4f;      // how much fewer stones are thrown from loose ground when fully wet
};

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
inline float level(float v) { return std::isfinite(v) ? std::max(0.0f, std::min(2.0f, v)) : 1.0f; }
inline float smooth(float a, float b, float x) { const float t = c01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
}  // namespace detail

// How wet: the game's 0 dry, 1 damp, 2 wet as 0, 0.5, 1.
inline float Amount(int wet) { return wet >= 2 ? 1.0f : (wet == 1 ? 0.5f : 0.0f); }

struct Plan {
    float tone = 0.0f;        // added to the tyre's tone: negative = duller (the mixer takes -1 .. +1)
    float gain = 1.0f;        // multiplies the dry loop's level
    float hiss = 0.0f;        // level of the spray hiss loop, 0..1, relative to the loop level
    float stones = 1.0f;      // multiplies how much loose material there is to throw
    float wetStone = 0.0f;    // chance 0..1 that a thrown stone also makes the small splash of one landing in water
};

// wet: the game's 0/1/2. loose: how much loose material the ground has, 0..1. speedKph: the tyre's surface speed. nativeShare: 0..1.
// liveLevel: the live multiplier from the menu, 0 (none) .. 2 (twice).
inline Plan Make(int wet, float loose, float speedKph, float nativeShare, float liveLevel, const Params& p) {
    using namespace detail;
    Plan out;
    if (!p.on || Amount(wet) <= 0.0f) return out;
    const float lf = c01(loose);
    const float lv = detail::level(liveLevel);
    const float ws = std::min(1.0f, Amount(wet) * lv);                                // the strength the menu level gives
    const float treat = ws * (1.0f - c01(nativeShare));                               // a real wet recording needs none of it
    out.tone = -std::min(1.0f, c01(p.tone) * treat * lf);
    out.gain = 1.0f - std::min(0.6f, c01(p.crunch) * treat * lf);
    out.hiss = c01(p.hiss * treat * smooth(8.0f, 100.0f, speedKph) * (1.0f - 0.4f * lf));
    out.stones = 1.0f - std::min(0.8f, c01(p.stones) * ws * lf);
    out.wetStone = c01(0.6f * ws * lf) * (1.0f - c01(nativeShare));
    return out;
}

}  // namespace WetSurface
