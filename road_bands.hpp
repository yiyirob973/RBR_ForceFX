// Road bands: the road vibration as three sine effects, each carrying one band of the road's spectrum (low, mid, high).
// Pure logic, no Windows or game access, so it can be tested on its own (see tests/road_bands_test.cpp).
//
// A wheel can only play periodic effects, and one periodic effect is a single pitch. Road noise has many pitches at once, so the
// road is split over three sine effects, each covering a third (in pitch) of the road frequency range. This follows the usual way
// of building a road surface from its spectrum:
//   - A road is made of bumps of every length. The pitch you feel from a bump is the speed divided by its length, so each band
//     follows the speed of the car: faster means higher. A band stays inside its own third of the range, because the wheel can
//     not play beyond it.
//   - How strong the road is overall still comes from the telemetry (how rough the tyres measure). What the ground decides is
//     how that strength is spread over the three bands: soft ground and snow are low and dull, loose ground is flat and grainy,
//     glazed ground is bright.
//   - A tyre does not follow tiny bumps: it rolls over everything shorter than its contact patch. At low speed the high bands
//     are therefore weak; the cut-off is where the bump length equals the patch length (speed / patch length).
//   - Both sides add up as energy, never as plus and minus, so left and right can not cancel each other out.
//
// Include after surface_feel.hpp (Surf::Feel, Surf::RidgeHz). In RBR_ForceFX.cpp it is included inside the anonymous namespace,
// right after road_feel.hpp.
#pragma once
#include <algorithm>
#include <cmath>

namespace Road {

constexpr int   kBands    = 3;
constexpr float kPatchLen = 0.13f;    // length of a tyre's contact patch in metres
constexpr float kRefSpeed = 20.0f;    // m/s: the speed at which each band sits in the middle of its third
constexpr float kPatchMin = 0.05f;    // the least a band is let through by the contact patch, so no band is ever completely dead
// Scale of the three sine effects together, kept from the earlier road feel so the road gain means the same as before.
constexpr float kSineEq   = 0.816f;

// The range of band i: the road range [minHz, maxHz] cut into three equal parts on a pitch (logarithmic) scale.
inline void BandRange(float minHz, float maxHz, int i, float& lo, float& hi) {
    if (!(minHz > 1.0f) || !std::isfinite(minHz)) minHz = 25.0f;
    if (!(maxHz > minHz) || !std::isfinite(maxHz)) maxHz = minHz * 4.0f;
    const float r = maxHz / minHz;
    i = std::max(0, std::min(i, kBands - 1));
    lo = minHz * std::pow(r, (float)i / kBands);
    hi = minHz * std::pow(r, (float)(i + 1) / kBands);
}

// Pitch of band i at this speed: the middle of its third at kRefSpeed, proportional to speed, held inside the third.
inline float BandPitch(float speed, float minHz, float maxHz, int i) {
    float lo, hi; BandRange(minHz, maxHz, i, lo, hi);
    const float mid = std::sqrt(lo * hi);
    const float v = (std::isfinite(speed) && speed > 0.0f) ? speed : 0.0f;
    const float f = mid * v / kRefSpeed;
    return f < lo ? lo : (f > hi ? hi : f);
}

// How much of a vibration of this pitch the tyre lets through at this speed: the main lobe of a moving average over the contact
// patch (a sinc). 1 for bumps much longer than the patch, falling to the floor where the bump is as short as the patch.
inline float PatchGain(float hz, float speed) {
    if (!std::isfinite(hz) || !(hz > 0.0f)) return 1.0f;
    if (!std::isfinite(speed) || !(speed > 0.1f)) return kPatchMin;
    const float x = kPatchLen * hz / speed;                 // patch length over bump length; 1 = the first null of the sinc
    if (x >= 1.0f) return kPatchMin;
    if (x < 1e-4f) return 1.0f;
    const float px = 3.14159265f * x;
    return std::max(kPatchMin, std::sin(px) / px);
}

// How the ground tilts the road spectrum: the strength of a band goes with (pitch / middle pitch) ^ p. Feel::hz is how high or low
// the ground sounds (soft ground 0.4, tarmac 1, loose ground 1.3, glaze 1.7), and p follows it: about -2 for dull ground (the low
// band dominates), -0.5 for plain ground, 0 for loose ground (flat, like gravel noise), +0.4 for glaze (the high band leads).
inline float TiltExponent(float feelHz) {
    const float h = (std::isfinite(feelHz) && feelHz > 0.05f) ? feelHz : 1.0f;
    const float p = -0.5f + 1.2f * std::log2(h);
    return p < -3.0f ? -3.0f : (p > 1.5f ? 1.5f : p);
}

// The share of one side's road strength carried by each band, for that side's ground. Before the contact patch, the shares
// squared add up to 1, so the ground changes where the strength sits and not how much of it there is; the contact patch then takes
// away what a tyre can not follow at this speed.
inline void BandShares(const Surf::Feel& fe, const float hz[kBands], float speed, float fc, float share[kBands]) {
    const float p = TiltExponent(fe.hz);
    float w[kBands], sum2 = 0.0f;
    for (int b = 0; b < kBands; ++b) {
        const float r = (hz[b] > 0.0f && fc > 0.0f) ? hz[b] / fc : 1.0f;
        w[b] = std::pow(r, p);
        if (!std::isfinite(w[b])) w[b] = 1.0f;
        sum2 += w[b] * w[b];
    }
    const float norm = sum2 > 1e-12f ? 1.0f / std::sqrt(sum2) : 0.0f;
    for (int b = 0; b < kBands; ++b) share[b] = w[b] * norm * PatchGain(hz[b], speed);
}

struct SideIn {
    Surf::Feel fe = Surf::kNeutral;   // the ground character of this side (Road::Derive)
    float strength = 0.0f;            // how strong this side's road is, in effect units (>= 0)
};
struct Targets {
    float hz[kBands] = {};            // pitch of each band (shared by both sides: one effect per band)
    float share[2][kBands] = {};      // per side, how much of its strength each band carries
};

// Pitch and shares for the two sides ([0] = left, [1] = right) at this speed. Ridged ground (cobbles, planks) also pulls the band
// whose third holds the ridge pitch (speed / spacing) onto exactly that pitch, and makes that band stronger and the others weaker, so the
// hits come at the rate the ridges pass under the tyre.
inline Targets Plan(const SideIn side[2], float speed, float minHz, float maxHz) {
    Targets t;
    float lo0, hi0, lo2, hi2;
    BandRange(minHz, maxHz, 0, lo0, hi0); BandRange(minHz, maxHz, kBands - 1, lo2, hi2);
    const float fc = std::sqrt(lo0 * hi2);
    for (int b = 0; b < kBands; ++b) t.hz[b] = BandPitch(speed, minHz, maxHz, b);

    // Ridges: the side that feels them most decides the ridge pitch.
    auto mixOf = [&](int s) { const float m = side[s].fe.ridgeMix; return std::isfinite(m) ? std::max(0.0f, std::min(1.0f, m)) : 0.0f; };
    auto strOf = [&](int s) { const float v = side[s].strength; return std::isfinite(v) && v > 0.0f ? v : 0.0f; };
    const int rs = strOf(0) * mixOf(0) >= strOf(1) * mixOf(1) ? 0 : 1;
    int lock = -1; float ridgeHz = 0.0f;
    if (strOf(rs) * mixOf(rs) > 0.0f && side[rs].fe.ridge > 0.0f) {
        ridgeHz = Surf::RidgeHz(speed, side[rs].fe.ridge, lo0, hi2);
        lock = kBands - 1;                                              // the band whose third the ridge pitch falls in
        for (int b = 0; b < kBands - 1; ++b) { float lo, hi; BandRange(minHz, maxHz, b, lo, hi); if (ridgeHz < hi) { lock = b; break; } }
    }
    for (int s = 0; s < 2; ++s) {
        BandShares(side[s].fe, t.hz, speed, fc, t.share[s]);
        if (lock >= 0 && mixOf(s) > 0.0f)
            for (int b = 0; b < kBands; ++b) t.share[s][b] *= (b == lock) ? 1.0f + 1.5f * mixOf(s) : 1.0f - 0.5f * mixOf(s);
    }
    if (lock >= 0) t.hz[lock] += (ridgeHz - t.hz[lock]) * mixOf(rs);
    return t;
}

// The strength of each band's effect: the two sides add as energy (square root of the sum of squares), never as plus and minus,
// so they can not cancel. `jit` is a random factor per side and band (1 = none), chosen by the caller every few milliseconds.
inline void Combine(const SideIn side[2], const float share[2][kBands], const float jit[2][kBands], float mag[kBands]) {
    for (int b = 0; b < kBands; ++b) {
        float e = 0.0f;
        for (int s = 0; s < 2; ++s) {
            const float st = std::isfinite(side[s].strength) && side[s].strength > 0.0f ? side[s].strength : 0.0f;
            const float j = std::isfinite(jit[s][b]) && jit[s][b] > 0.0f ? jit[s][b] : 0.0f;
            const float v = st * share[s][b] * j;
            e += v * v;
        }
        mag[b] = std::isfinite(e) ? kSineEq * std::sqrt(e) : 0.0f;
    }
}

}  // namespace Road
