// Shared by the plugin and its portable core (config.hpp, synth.hpp): the hard ceilings, a few small math helpers and the log call.
// Nothing here needs Windows. Inside the plugin these headers are included in its anonymous namespace (see src/RBR_ForceFX.cpp), so everything
// stays private to the DLL; the tests include them at global scope and supply their own ForceFX::Logf.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace ForceFX {

// Hard ceilings: the .ini can never push past these, whatever it says.
constexpr int   kHardTotalMax = 5000;    // sum of all layer magnitudes (0..10000 DirectInput units)
constexpr float kHardMinHz    = 12.0f;   // below this a vibration becomes a visible wheel swing
constexpr float kHardMaxHz    = 120.0f;

inline float clampf(float v, float lo, float hi) { return !(v >= lo) ? lo : (v > hi ? hi : v); }   // NaN comes out as lo, never passed on
inline float clamp01(float v) { return clampf(v, 0.0f, 1.0f); }
inline float smooth01(float e0, float e1, float x) { float t = clamp01((x - e0) / (e1 - e0)); return t * t * (3.0f - 2.0f * t); }
inline float gauss(float x, float c, float w) { float d = (x - c) / w; return std::exp(-d * d); }
inline float alphaFor(float dt, float tau) { return 1.0f - std::exp(-dt / tau); }   // EMA coefficient
inline float hpCoef(float dt, float tau) { return tau / (tau + dt); }               // 1st order high-pass
inline float rnd() { return (rand() % 2001 - 1000) / 1000.0f; }                    // -1..1

// Writes one line to ForceFX.log. The plugin defines it (src/RBR_ForceFX.cpp); a test that includes the core defines its own.
void Logf(const char* fmt, ...);

}  // namespace ForceFX
