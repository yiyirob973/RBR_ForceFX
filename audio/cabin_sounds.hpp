// The interior noises, made in code: no sound files are needed. Pure DSP (no Windows access), tested in audio/tests/cabin_test.cpp.
//   cabin_fan_loop    the heater blower: airy noise with a faint motor tone
//   cabin_wind_loop   air rushing past the glass: low and gusty
//   cabin_buzz_loop   trim and panels buzzing with the engine: pulses at 100 Hz (the engine's second order at 3000 rpm) with two panel
//                     resonances, a little irregular; the mixer plays it faster or slower with the engine speed
//   cabin_rattle      short rattles (a few variants): small loose things knocking
// Files with these names in the sounds folder (cabin_fan_loop.wav, ...) are used instead.
#pragma once
#include "water_sounds.hpp"

namespace ffx {
namespace wsnd {

// One-pole high-pass and low-pass helpers on a running sample.
struct OnePole {
    double lp = 0;
    double low(double x, double fc) { lp += (1.0 - std::exp(-kTwoPi * fc / kSampleRate)) * (x - lp); return lp; }
};

inline std::vector<float> makeCabinFan() {
    const double dur = 4.0;
    const size_t n = (size_t)(dur * kSampleRate);
    std::vector<float> d(n, 0.0f);
    Noise r(1117u);
    OnePole a1, a2, h1;
    for (size_t i = 0; i < n; ++i) {
        const double t = (double)i / kSampleRate;
        double x = r.next();
        x = a2.low(a1.low(x, 2600.0), 2600.0);                                    // top end off
        const double air = x - h1.low(x, 220.0);                                  // low end off
        const double tone = 0.10 * std::sin(kTwoPi * 205.0 * t) + 0.04 * std::sin(kTwoPi * 410.0 * t + 0.7);   // the motor, felt more than heard
        const double wob = 1.0 + 0.03 * std::sin(kTwoPi * 0.5 * t);
        d[i] = (float)((air + tone) * wob);
    }
    makeSeamless(d);
    normalizeLoudness(d, targetRmsFor("cabin_fan_loop"));
    return d;
}

inline std::vector<float> makeCabinWind() {
    const double dur = 4.0;
    const size_t n = (size_t)(dur * kSampleRate);
    std::vector<float> d(n, 0.0f);
    Noise r(2203u), g(991u);
    OnePole a1, a2, a3, gust;
    for (size_t i = 0; i < n; ++i) {
        double x = r.next();
        x = a3.low(a2.low(a1.low(x, 1300.0), 1300.0), 1300.0);                    // low and dull: air round the glass heard from inside
        const double gs = gust.low(g.next(), 1.2);                                // gusts a second or so long
        d[i] = (float)(x * std::max(0.35, 0.8 + 6.0 * gs));
    }
    makeSeamless(d);
    normalizeLoudness(d, targetRmsFor("cabin_wind_loop"));
    return d;
}

inline std::vector<float> makeCabinBuzz() {
    const double dur = 3.0, f0 = 100.0;
    const size_t n = (size_t)(dur * kSampleRate);
    const size_t period = (size_t)std::llround(kSampleRate / f0);
    std::vector<float> d(n, 0.0f);
    Noise r(4409u), j(7301u);
    // two panel resonances, each a damped sine rung by every pulse
    struct Res { double f, q, y1 = 0, y2 = 0; } res[2] = {{720.0, 9.0}, {1850.0, 7.0}};
    double amp = 1.0;
    for (size_t i = 0; i < n; ++i) {
        double in = 0.0;
        if (i % period == 0) { amp = 0.75 + 0.25 * (j.next() * 0.5 + 0.5); in = amp; }   // a little uneven, like loose trim
        in += 0.04 * r.next();
        double y = 0.0;
        for (Res& s : res) {
            const double w = kTwoPi * s.f / kSampleRate, rr = std::exp(-w / (2.0 * s.q));
            const double ny = in + 2.0 * rr * std::cos(w) * s.y1 - rr * rr * s.y2;
            s.y2 = s.y1; s.y1 = ny; y += 0.5 * ny * (s.f < 1000.0 ? 1.0 : 0.6);
        }
        d[i] = (float)y;
    }
    // a whole number of pulse periods, so the loop joins up on its own; no trimming
    d.resize((n / period) * period);
    normalizeLoudness(d, targetRmsFor("cabin_buzz_loop"));
    return d;
}

// One rattle: a few small knocks close together, each a short burst of mid and upper noise.
inline std::vector<float> makeCabinRattle(unsigned seed) {
    const double dur = 0.22;
    const size_t n = (size_t)(dur * kSampleRate);
    std::vector<float> d(n, 0.0f);
    Noise r(seed * 977u + 13u), p(seed * 313u + 5u);
    const int knocks = 3 + (int)(seed % 3);
    double t0 = 0.0;
    for (int k = 0; k < knocks; ++k) {
        const double at = t0 + 0.012 + 0.028 * (p.next() * 0.5 + 0.5);
        t0 = at;
        const double f = 1100.0 + 1900.0 * (p.next() * 0.5 + 0.5);
        const double gain = (0.9 - 0.15 * k) * (0.7 + 0.3 * (p.next() * 0.5 + 0.5));
        OnePole lp; 
        const size_t s0 = (size_t)(at * kSampleRate);
        for (size_t i = s0; i < n; ++i) {
            const double t = (double)(i - s0) / kSampleRate;
            const double env = std::exp(-t / 0.011);
            if (env < 0.002) break;
            const double ring = std::sin(kTwoPi * f * t) * 0.6 + lp.low(r.next(), 5200.0) * 0.7;
            d[i] += (float)(ring * env * gain);
        }
    }
    normalizeLoudness(d, 1e9, 0.7);                                               // a one-shot: peak-normalised
    return d;
}

// Adds whatever interior sounds the bank does not have. Returns how many were made.
inline int addCabinFallbacks(SoundBank& bank) {
    int made = 0;
    if (!bank.get("cabin_fan_loop"))  { bank.add("cabin_fan_loop", makeCabinFan()); ++made; }
    if (!bank.get("cabin_wind_loop")) { bank.add("cabin_wind_loop", makeCabinWind()); ++made; }
    if (!bank.get("cabin_buzz_loop")) { bank.add("cabin_buzz_loop", makeCabinBuzz()); ++made; }
    if (!bank.get("cabin_rattle"))    { for (unsigned v = 1; v <= 4; ++v) bank.add("cabin_rattle", makeCabinRattle(v)); ++made; }
    return made;
}

}  // namespace wsnd
}  // namespace ffx
