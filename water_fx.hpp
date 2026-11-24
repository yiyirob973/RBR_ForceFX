// Water feel: what the wheel does when the tyres go through standing water (fords, puddles, flooded road).
// Pure logic, no Windows or game access, so it can be tested on its own (see tests/water_fx_test.cpp).
//
// Three things happen in real water, and each has its own effect here:
//   - Splash:   the moment a tyre goes in, the water hits it. A short, heavy thump that grows with speed (the water has to be
//               pushed aside, and that takes force that rises with the square of the speed). A tyre coming out lets go, a smaller
//               echo of the same. A landing in water is a splash too.
//   - Wading:   as long as the tyres are in water the front end is churned and slapped. A low, turbulent rumble that grows with
//               speed and with how many tyres are in the water, stronger on the side that is wet.
//   - Heavy steering: with the front tyres pushing through water the steering gets much heavier (the water drags on the turned tyres
//               and wants to keep them straight). This scales RBR's own force up while the front tyres are in water; it never adds a force of its own.
//
// Only the material the surface reader found under each tyre decides whether there is water: Water_Surface (id 157) in NGP's table,
// plus any ids listed in WaterIds= in ForceFX.ini (some stages may use other ids for their fords). Wet tarmac and wet gravel are
// a film, not standing water, and are left to the ordinary road feel.
//
// The vibration is delivered the same way as the road feel (three zero-mean sine bands riding on RBR's own force), and the kick on
// entry is delivered the way the landing boost is (RBR's own force, scaled up for a moment), so the wheel only ever gets more or
// less of what RBR already sends.
#pragma once
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

namespace Water {

constexpr int kBands = 3;
constexpr int kWaterMaterialId = 157;     // "Water_Surface" in NGP's material table
constexpr float kRef = 1800.0f;           // wheel units of a full splash at gain 1 (a full splash is a car at full speed hitting deep water)
constexpr float kSloshMax = 0.35f;        // wading never gets above this share of a full splash: it lasts, a splash does not
constexpr float kKick = 0.45f;            // extra RBR force at the peak of a full splash, at gain 1 (0.45 = +45%)

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
inline float smooth01(float a, float b, float x) { const float t = c01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
inline float alphaFor(float dt, float tau) { return 1.0f - std::exp(-dt / tau); }
}  // namespace detail

// ---------------------------------------------------------------- which ids are water
inline bool* ExtraIds() { static bool e[256] = {}; return e; }

// "200,201,210": more material ids to treat as standing water. Returns how many were applied; entries that are not numbers from 0 to 255
// are skipped and, when `rejected` is given, listed there so the plugin can say so in the log.
inline int ParseIds(const std::string& list, std::vector<std::string>* rejected = nullptr) {
    int applied = 0;
    size_t p = 0;
    while (p < list.size()) {
        size_t q = list.find(',', p);
        std::string t = list.substr(p, q == std::string::npos ? std::string::npos : q - p);
        p = q == std::string::npos ? list.size() : q + 1;
        t.erase(std::remove_if(t.begin(), t.end(), [](char ch) { return ch == ' ' || ch == '\t' || ch == '\r'; }), t.end());
        if (t.empty()) continue;
        if (t.size() > 3 || !std::all_of(t.begin(), t.end(), [](char ch) { return ch >= '0' && ch <= '9'; }) || atoi(t.c_str()) > 255) {
            if (rejected) rejected->push_back(t + " (the id must be a number from 0 to 255)");
            continue;
        }
        ExtraIds()[atoi(t.c_str())] = true; ++applied;
    }
    return applied;
}
inline void ClearIds() { for (int i = 0; i < 256; ++i) ExtraIds()[i] = false; }

// 0 = not standing water, 1 = standing water.
inline float DepthOfId(int id) {
    if (id < 0 || id > 255) return 0.0f;
    return (id == kWaterMaterialId || ExtraIds()[id]) ? 1.0f : 0.0f;
}

// ---------------------------------------------------------------- how hard the water hits
// Force on a tyre pushing water aside rises with the square of the speed. Nothing below walking pace, full at about 32 m/s (115 km/h).
inline float SpeedFactor(float speedMs) {
    if (!std::isfinite(speedMs) || speedMs <= 3.0f) return 0.0f;
    return std::pow(detail::c01((speedMs - 3.0f) / 29.0f), 1.4f);
}
// How much of the wading rumble a speed gives: a faint one even when creeping through, then growing towards full speed.
inline float SloshSpeed(float speedMs, float maxSpeed) {
    if (!std::isfinite(speedMs) || speedMs <= 0.0f) return 0.0f;
    const float ms = (std::isfinite(maxSpeed) && maxSpeed > 1.0f) ? maxSpeed : 40.0f;
    return 0.10f * detail::smooth01(1.0f, 4.0f, speedMs) + 0.90f * std::pow(detail::c01(speedMs / ms), 1.5f);
}

// ---------------------------------------------------------------- the tracker
// Wheel order everywhere: 0 front left, 1 front right, 2 rear left, 3 rear right. Side 0 = left (tyres 0 and 2), side 1 = right (1 and 3).
struct In {
    float depth[4] = {0, 0, 0, 0};   // DepthOfId of each tyre's material, 0..1 (use 0 when the ground is not known)
    float speed = 0.0f;              // m/s over the ground
    float maxSpeed = 40.0f;          // m/s that counts as full speed
    float airborne = 0.0f;           // 0 on the ground .. 1 in the air
    bool  landed = false;            // a landing was detected on this step
    float landSev = 0.0f;            // its severity, 0..1
};
struct Out {
    float splash[2] = {0, 0};        // per side, 0..1: the thump on entry (and exit), dying away over a fraction of a second
    float slosh[2]  = {0, 0};        // per side, 0..1: the wading rumble (already scaled by kSloshMax)
    float kick = 0.0f;               // 0..1: front-weighted splash, for scaling RBR's force up for a moment
    float heavy = 0.0f;              // 0..1: front tyres pushing through standing water (how much of the heavy steering is on)
    float entry = 0.0f;              // 0..1: the hardest splash that started on THIS step (0 on all others): one event for sound and glass
    int   entryTyre = -1;            // the tyre that made it
    float immersion[4] = {0, 0, 0, 0};   // each tyre's smoothed share in the water
};

class Tracker {
public:
    static constexpr float kMinEvent = 0.05f;     // splashes weaker than this are not events
    static constexpr float kExitShare = 0.30f;    // a tyre leaving the water splashes this much of what it would on the way in
    static constexpr float kCooldown = 0.15f;     // seconds before the same tyre can make another event (puddle edges flicker)

    void Reset() { *this = Tracker(); }

    Out Update(const In& in, float dt) {
        using detail::c01; using detail::alphaFor;
        Out o;
        if (!std::isfinite(dt) || dt <= 0.0f) dt = 0.01f;
        if (dt > 0.1f) dt = 0.1f;
        const float spd = std::isfinite(in.speed) && in.speed > 0.0f ? in.speed : 0.0f;
        const float sf = SpeedFactor(spd);
        const float air = c01(in.airborne);

        float ev[4] = {0, 0, 0, 0};
        for (int w = 0; w < 4; ++w) {
            const float d = c01(in.depth[w]);
            cool_[w] = std::max(0.0f, cool_[w] - dt);
            if (init_) {
                const bool wasIn = prev_[w] >= 0.5f, isIn = d >= 0.5f;
                if (!wasIn && isIn && cool_[w] <= 0.0f) ev[w] = d * sf;                       // a tyre goes in
                else if (wasIn && !isIn && cool_[w] <= 0.0f) ev[w] = kExitShare * prev_[w] * sf;   // a tyre comes out
                if (in.landed && isIn) ev[w] = std::max(ev[w], std::max(0.4f * sf, c01(0.5f + 0.5f * in.landSev) * std::max(sf, 0.55f)));   // a landing in water
                if (ev[w] >= kMinEvent) cool_[w] = kCooldown;
            }
            prev_[w] = d;
            // each tyre's share in the water: quick to go in, a little slower to drain
            imm_[w] += (d - imm_[w]) * alphaFor(dt, d > imm_[w] ? 0.05f : 0.20f);
            if (!(imm_[w] > 0.0f)) imm_[w] = 0.0f;
            o.immersion[w] = imm_[w];
            env_[w] = std::max(env_[w] * std::exp(-dt / 0.22f), ev[w] >= kMinEvent ? c01(ev[w]) : 0.0f);
            if (ev[w] >= kMinEvent && ev[w] > o.entry) { o.entry = c01(ev[w]); o.entryTyre = w; }
        }
        init_ = true;

        // The front tyre of a side is wheel s, the rear tyre is wheel 2 + s. The front meets the water first and is what the steering feels.
        for (int s = 0; s < 2; ++s) {
            o.splash[s] = std::max(env_[s], 0.7f * env_[2 + s]);
            const float sideImm = 0.65f * imm_[s] + 0.35f * imm_[2 + s];
            o.slosh[s] = kSloshMax * sideImm * SloshSpeed(spd, in.maxSpeed) * (1.0f - air);
        }
        o.kick = std::max(std::max(env_[0], env_[1]), 0.5f * std::max(env_[2], env_[3]));
        // The steering is heavy from a walking-pace crawl upwards: the water resists a turned tyre long before it is going fast.
        o.heavy = 0.5f * (imm_[0] + imm_[1]) * detail::smooth01(1.0f, 12.0f, spd) * (1.0f - air);
        return o;
    }

private:
    bool init_ = false;
    float prev_[4] = {0, 0, 0, 0}, imm_[4] = {0, 0, 0, 0}, env_[4] = {0, 0, 0, 0}, cool_[4] = {0, 0, 0, 0};
};

// ---------------------------------------------------------------- delivery
// Share of a splash and of wading that each of the three road bands (low, mid, high) carries. Both are squared sums of 1.
// A splash is a low thump with a short rattle on top; wading is almost all low.
constexpr float kSplashShare[kBands] = {0.753f, 0.565f, 0.339f};
constexpr float kSloshShare[kBands]  = {0.845f, 0.507f, 0.169f};

// The strength of each band's sine in wheel units. `jit` is a random factor per side and band for the wading (1 = none), picked by the
// caller every few milliseconds. The two sides add as energy, never as plus and minus, like the road, so they can not cancel.
inline void BandMagnitudes(const Out& o, float gain, const float jit[2][kBands], float mag[kBands]) {
    const float g = std::isfinite(gain) && gain > 0.0f ? (gain > 2.0f ? 2.0f : gain) : 0.0f;
    for (int b = 0; b < kBands; ++b) {
        float e = 0.0f;
        for (int s = 0; s < 2; ++s) {
            const float j = jit && std::isfinite(jit[s][b]) && jit[s][b] > 0.0f ? jit[s][b] : 0.0f;
            const float v = kRef * g * (detail::c01(o.splash[s]) * kSplashShare[b] + (std::isfinite(o.slosh[s]) ? std::min(std::max(o.slosh[s], 0.0f), kSloshMax) : 0.0f) * kSloshShare[b] * j);
            e += v * v;
        }
        mag[b] = std::isfinite(e) ? std::sqrt(e) : 0.0f;
        if (mag[b] > kRef * g) mag[b] = kRef * g;                                 // never above one full splash, however the sides add up
    }
}

// RBR's force times this, for a moment, on a splash (>= 1).
inline float KickMul(const Out& o, float gain) {
    const float g = std::isfinite(gain) && gain > 0.0f ? (gain > 2.0f ? 2.0f : gain) : 0.0f;
    return 1.0f + kKick * g * detail::c01(o.kick);
}

// RBR's force times this while the front tyres push through water (>= 1). `heaviness` is the setting: 0 = off, 1 = double the force at full
// effect, at most 2 (never more than three times).
inline float HeavyMul(const Out& o, float heaviness) {
    const float h = std::isfinite(heaviness) && heaviness > 0.0f ? (heaviness > 2.0f ? 2.0f : heaviness) : 0.0f;
    return 1.0f + h * detail::c01(o.heavy);
}

}  // namespace Water
