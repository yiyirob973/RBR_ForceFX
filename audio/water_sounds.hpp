// Water sounds made in code, for when the sound folder has none of its own.
//
// RBR's sound files have a rolling-through-water sound (Surfaces\\Normal\\water.wav, copied as water_loop) but no splash and no
// stone-in-water sound, so without these nothing happened when a tyre hit the water. When the bank has RBR's water_loop, the splash is made
// from that recording (a slice of it with a hit-and-fade envelope, under a low thump), so it sounds like the game's own water; with no
// water_loop it is made from noise. Each sound is only made when the bank has no file of that name, so dropping your own
// water_splash.wav / water_loop.wav / water_hit.wav (or the debris_splash.wav / water_surface_* names) into the sounds folder
// replaces it. They are plain filtered noise, a short low thump and a few small droplets, the same every run.
//
//   water_splash   a tyre hits standing water: a crash that darkens quickly, a low thump under it, a few droplets after
//   water_loop     the tyre rolling through water: a soft, slapping hiss
//   water_hit      what flying stones sound like in water: a small plip
// The Water_Surface material finds water_loop and water_hit by its name, so a ford gets its own sound set.
//
// Portable (no Windows APIs), tested in tests/water_sounds_test.cpp.
#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include "audio.hpp"

namespace ffx {
namespace wsnd {

constexpr double kTwoPi = 6.28318530717958647692;

// Small repeatable noise source (the sounds are the same on every run and every machine).
struct Noise {
    uint32_t s;
    explicit Noise(uint32_t seed) : s(seed * 2654435761u + 12345u) {}
    float next() { s = s * 1664525u + 1013904223u; return (float)((s >> 8) & 0xFFFF) / 32767.5f - 1.0f; }   // -1 .. 1
    float unit() { return 0.5f + 0.5f * next(); }                                                                // 0 .. 1
};

inline void peak(std::vector<float>& d, float target) {
    float pk = 0.0f;
    for (float x : d) pk = std::max(pk, std::fabs(x));
    if (pk < 1e-6f) return;
    const float g = target / pk;
    for (float& x : d) x *= g;
}

// A droplet: a short sine that rises in pitch, which is what a small bubble of water does. Added into d at frame `at`.
inline void droplet(std::vector<float>& d, size_t at, double hz0, double hz1, double len, float amp) {
    const size_t n = (size_t)(len * kSampleRate);
    double ph = 0.0;
    for (size_t i = 0; i < n && at + i < d.size(); ++i) {
        const double u = (double)i / (double)n;
        ph += kTwoPi * (hz0 + (hz1 - hz0) * u) / kSampleRate;
        d[at + i] += (float)(std::sin(ph) * amp * std::exp(-5.0 * u) * std::min(1.0, u * 40.0));
    }
}

// A tyre hits the water. About 1 second. Variants differ in their noise and droplets.
inline std::vector<float> makeSplash(int variant) {
    const size_t n = (size_t)(1.0 * kSampleRate);
    std::vector<float> d(n, 0.0f);
    Noise r((uint32_t)variant + 1u);
    double l1 = 0.0, l2 = 0.0, hp = 0.0, ph = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double t = (double)i / kSampleRate;
        const double env = t < 0.004 ? t / 0.004 : std::exp(-(t - 0.004) / 0.20);
        const double fc = 400.0 + 4200.0 * std::exp(-t / 0.12);                   // bright at the crash, darker as the water falls back
        const double a = 1.0 - std::exp(-kTwoPi * fc / kSampleRate);
        const double x = r.next();
        l1 += a * (x - l1); l2 += a * (l1 - l2);                                  // two poles
        hp += 0.02 * (l2 - hp);                                                   // the noise keeps out of the low end: the thump carries it
        ph += kTwoPi * (45.0 + 40.0 * std::exp(-t / 0.08)) / kSampleRate;
        const double thump = std::sin(ph) * std::exp(-t / 0.10) * std::min(1.0, t / 0.003);
        d[i] = (float)(1.6 * env * (l2 - hp) + 0.7 * thump);
    }
    for (int k = 0; k < 5; ++k)                                                   // drops falling back
        droplet(d, (size_t)((0.22 + 0.65 * r.unit()) * kSampleRate), 900.0 + 500.0 * r.unit(), 1700.0 + 900.0 * r.unit(), 0.03 + 0.02 * r.unit(), 0.10f + 0.10f * r.unit());
    peak(d, 0.8f);
    return d;
}

// What a flung stone sounds like in water: a small plip over a short band of noise. About 0.2 seconds.
inline std::vector<float> makeWaterHit(int variant) {
    const size_t n = (size_t)(0.2 * kSampleRate);
    std::vector<float> d(n, 0.0f);
    Noise r((uint32_t)variant + 101u);
    double l1 = 0.0, l2 = 0.0, hp = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double t = (double)i / kSampleRate;
        const double env = t < 0.002 ? t / 0.002 : std::exp(-(t - 0.002) / 0.035);
        const double a = 1.0 - std::exp(-kTwoPi * 2600.0 / kSampleRate);
        l1 += a * (r.next() - l1); l2 += a * (l1 - l2);
        hp += 0.12 * (l2 - hp);                                                   // takes the low end off: a splat, not a thud
        d[i] = (float)(env * (l2 - hp));
    }
    droplet(d, (size_t)(0.004 * kSampleRate), 650.0 + 300.0 * r.unit(), 1300.0 + 500.0 * r.unit(), 0.06, 0.5f);
    peak(d, 0.6f);
    return d;
}

// A tyre rolling through water: a soft hiss whose strength slaps up and down a few times a second. The slaps repeat a whole
// number of times in the length of the loop, so it joins up with itself.
inline std::vector<float> makeWaterLoop() {
    const double dur = 2.4;
    const size_t n = (size_t)(dur * kSampleRate);
    std::vector<float> d(n, 0.0f);
    Noise r(777u), m(4242u);
    double l1 = 0.0, l2 = 0.0, hp = 0.0, slow = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double t = (double)i / kSampleRate;
        const double a = 1.0 - std::exp(-kTwoPi * 1800.0 / kSampleRate);
        l1 += a * (r.next() - l1); l2 += a * (l1 - l2);
        hp += 0.03 * (l2 - hp);
        slow += 0.0015 * (m.next() - slow);                                       // irregular lumps, a few per second
        const double am = 0.55 + 0.20 * std::sin(kTwoPi * 5.0 * t / dur + 0.7) + 0.12 * std::sin(kTwoPi * 11.0 * t / dur + 2.1) + 3.0 * slow;
        d[i] = (float)((l2 - hp) * std::max(0.15, am));
    }
    makeSeamless(d);
    normalizeLoudness(d, targetRmsFor("water_loop"));
    return d;
}

// A splash made from RBR's own water recording: a different slice of it for each variant, hit hard and fading over about a second, with the same
// low thump under it as the noise splash. About 1 second.
inline std::vector<float> makeSplashFromLoop(const std::vector<float>& loop, int variant) {
    const size_t n = (size_t)(0.95 * kSampleRate);
    if (loop.size() < 64) return makeSplash(variant);
    const size_t off = (size_t)((0.10 + 0.37 * (double)variant) * kSampleRate) % loop.size();
    double e = 0.0;                                                               // the slice's loudness, so every recording ends up at the same level
    for (size_t i = 0; i < n; ++i) { const double x = loop[(off + i) % loop.size()]; e += x * x; }
    const double rms = std::sqrt(e / (double)n);
    const double g = rms > 1e-5 ? 0.30 / rms : 0.0;
    std::vector<float> d(n, 0.0f);
    double ph = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double t = (double)i / kSampleRate;
        const double env = t < 0.005 ? t / 0.005 : std::exp(-(t - 0.005) / 0.30);
        ph += kTwoPi * (45.0 + 40.0 * std::exp(-t / 0.08)) / kSampleRate;
        const double thump = std::sin(ph) * std::exp(-t / 0.10) * std::min(1.0, t / 0.003);
        const double x = std::isfinite((double)loop[(off + i) % loop.size()]) ? loop[(off + i) % loop.size()] : 0.0;
        d[i] = (float)(3.0 * g * x * env + 0.55 * thump);
    }
    peak(d, 0.8f);
    return d;
}

// Adds whichever of the three the bank does not have already. Returns how many were added (0 = the bank has its own water sounds).
// `splashFromRbr` (optional) says whether the splash was made from the bank's own water_loop rather than from noise.
inline int addWaterFallbacks(SoundBank& bank, bool* splashFromRbr = nullptr) {
    int added = 0;
    if (splashFromRbr) *splashFromRbr = false;
    auto has = [&](std::initializer_list<const char*> keys) { for (const char* k : keys) if (bank.get(k)) return true; return false; };
    const SoundBank::Variants* realLoop = bank.get("water_loop");
    if (!realLoop) realLoop = bank.get("water_surface_loop");
    if (!has({"water_splash", "debris_splash"})) {
        if (realLoop && !realLoop->empty() && (*realLoop)[0].size() >= 64) {
            const std::vector<float> src = (*realLoop)[0];                           // a copy: adding to the bank may move what realLoop points at
            for (int v = 0; v < 3; ++v) bank.add("water_splash", makeSplashFromLoop(src, v));
            if (splashFromRbr) *splashFromRbr = true;
        } else for (int v = 0; v < 3; ++v) bank.add("water_splash", makeSplash(v));
        ++added;
    }
    if (!has({"water_loop", "water_surface_loop"})) { bank.add("water_loop", makeWaterLoop()); ++added; }
    if (!has({"water_hit", "water_surface_hit"})) { for (int v = 0; v < 4; ++v) bank.add("water_hit", makeWaterHit(v)); ++added; }
    return added;
}

// The spray of wet ground under a tyre: a soft, steady hiss with the low end and the very top taken off, so it sits above the tyre's own noise
// without covering it. wet_surface.hpp mixes it in under each tyre in proportion to how wet the ground is and how fast the tyre goes. About 3 seconds,
// made to join up with itself. A recording named wet_hiss_loop.wav in the sounds folder is used instead.
inline std::vector<float> makeWetHiss() {
    const double dur = 3.0;
    const size_t n = (size_t)(dur * kSampleRate);
    std::vector<float> d(n, 0.0f);
    Noise r(31337u), m(2718u);
    double l1 = 0.0, l2 = 0.0, hp = 0.0, slow = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double t = (double)i / kSampleRate;
        const double a = 1.0 - std::exp(-kTwoPi * 6500.0 / kSampleRate);          // top end off above about 6.5 kHz
        l1 += a * (r.next() - l1); l2 += a * (l1 - l2);
        hp += 0.20 * (l2 - hp);                                                   // low end off below about 1.6 kHz
        slow += 0.0010 * (m.next() - slow);                                       // a little unevenness, a few swells a second
        const double am = 0.85 + 0.10 * std::sin(kTwoPi * 3.0 * t / dur + 0.4) + 2.0 * slow;
        d[i] = (float)((l2 - hp) * std::max(0.3, am));
    }
    makeSeamless(d);
    normalizeLoudness(d, targetRmsFor("wet_hiss_loop"));
    return d;
}

// Adds the wet hiss if the bank has none of its own. Returns true if it was made.
inline bool addWetFallbacks(SoundBank& bank) {
    if (bank.get("wet_hiss_loop")) return false;
    bank.add("wet_hiss_loop", makeWetHiss());
    return true;
}

}  // namespace wsnd
}  // namespace ffx
