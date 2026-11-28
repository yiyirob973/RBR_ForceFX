// Tests for the synthesis (core/synth.hpp): scripted drives at 100 Hz through every effect, with several settings, checked two ways.
//   1. The whole output of all the drives is hashed and compared with the hash of the last accepted behaviour. A refactor must leave it alone; a change
//      that is meant to alter the feel makes the test fail and print the new hash to put in kGolden (after listening to it, of course).
//   2. Whatever the drive, every output stays finite and inside its limits.
// Self-contained:
//   g++ -std=c++17 -O1 -I.. synth_golden_test.cpp -o synth_golden_test && ./synth_golden_test
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <string>
#include "core/synth.hpp"
#include "tests/mini_test.hpp"

namespace ForceFX {
static std::string g_log;      // what the synthesis wrote to the log
void Logf(const char* fmt, ...) { char b[512]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); g_log += b; g_log += "\n"; }
}  // namespace ForceFX

using namespace ForceFX;

static const unsigned long long kGolden = 0x396cfe2e7946c0dfULL;

static unsigned g_seed = 12345;
static float rf() { g_seed = g_seed * 1664525u + 1013904223u; return (float)((g_seed >> 8) & 0xFFFF) / 65535.0f; }

// What a step produced, for the limit checks.
struct Seen { bool finite = true; int steps = 0; int magMax = 0; float hzMax = 0, mulMin = 1e9f, mulMax = -1e9f, addMin = 1e9f, addMax = -1e9f; };

// One scripted drive: a start from rest, speeding up, rev limiter, braking, handbrake, a jump, water, a hit, changing ground, sideways load swinging side to side.
// Every third step is appended to `text` (that is what gets hashed); every step is checked against the limits.
static void Drive(const Config& c, int variant, std::string& text, Seen& seen, bool breakTelemetry = false) {
    srand(7); g_seed = 12345u + (unsigned)variant;
    Synth sy; Signals s;
    unsigned evt = 0, crash = 0; double t = 0; float vPrev = 0.0f;
    char b[256];
    for (int i = 0; i < 6000; ++i, t += 0.01) {
        const float ph = (float)i * 0.01f;
        s.rpm = i < 100 ? 900.0f : 2500.0f + 4500.0f * (0.5f + 0.5f * std::sin(ph * 0.7f));
        if (i % 2400 > 2300) s.rpm = 7600.0f;
        const float ramp = std::min(1.0f, std::max(0.0f, (ph - 2.0f) / 6.0f));          // away from the line, then speeding up and slowing down by about 0.8 g, over and over
        s.speed = ramp * (25.0f + 18.0f * std::sin((ph - 2.0f) * 0.45f));
        const float lonG = i == 0 ? 0.0f : (s.speed - vPrev) / 0.01f / 9.81f; vPrev = s.speed;
        s.ground = s.speed * (i % 900 < 600 ? 1.0f : 0.0f);
        s.throttle = clampf(0.5f + 0.5f * std::sin(ph * 0.9f), 0.0f, 1.0f);
        s.brake = lonG < -0.2f ? 0.8f : 0.0f;
        s.handbrake = (i / 700) % 5 == 4 ? 1.0f : 0.0f;
        s.texture = 0.5f + 0.4f * std::sin(ph * 1.3f); s.texL = clamp01(s.texture + 0.1f * std::sin(ph * 3)); s.texR = clamp01(s.texture - 0.1f * std::sin(ph * 3));
        s.bump = rf() < 0.02f ? rf() : s.bump * 0.9f;
        s.loadL = 1.0f + 0.5f * std::sin(ph * 1.1f); s.loadR = 1.0f - 0.5f * std::sin(ph * 1.1f); s.loadF = 1.0f + 0.3f * std::sin(ph * 0.6f);
        s.airborne = (i % 1500 > 1420) ? 1.0f : 0.0f;
        const float lat = 0.4f * std::sin(ph * 1.7f), lon = -0.15f * std::max(-1.0f, std::min(1.0f, lonG / 0.8f));   // acceleration unloads the front, braking loads it
        s.wload[0] = 0.27f + 0.5f * lat * 0.5f + lon * 0.2f; s.wload[1] = 0.27f - 0.5f * lat * 0.5f + lon * 0.2f;
        s.wload[2] = 0.23f + 0.5f * lat * 0.5f - lon * 0.2f; s.wload[3] = 0.23f - 0.5f * lat * 0.5f - lon * 0.2f;
        s.accOk = i > 300; s.accMag = std::fabs(lonG) * 9.81f; s.latG = s.accOk ? std::fabs(lat) * 1.8f : 0.0f;
        s.dive = (s.brake > 0.1f) ? 0.4f : 0.0f; s.lock = (i % 1000 > 960) ? 0.8f : 0.0f;
        s.frontRev = 8.0f + s.speed * 0.5f;
        for (int w = 0; w < 4; ++w) {
            s.surf[w] = (i / 800 + w) % 6; s.mat[w] = ((i / 500) % 4 == 3) ? 157 : (int)((i / 600 + w * 7) % 30) - 1;
            s.ratio[w] = (w < 2 ? 0.0f : 0.3f * std::sin(ph + w)); s.ratioOk[w] = i % 400 < 350; s.angle[w] = 0.2f * std::sin(ph * 1.7f + w);
        }
        s.angleOk = true;
        if (i % 1500 == 1450) { ++evt; s.evtSev = 0.3f + 0.7f * rf(); }
        if (i % 2200 == 2100) { ++crash; s.crashSev = 0.2f + 0.8f * rf(); }
        s.evtSeq = evt; s.crashSeq = crash; s.thumpL = (i % 1500 > 1450) ? 0.6f : 0.0f; s.thumpR = s.thumpL * 0.5f;
        s.rpmMaxTelem = (variant & 1) ? 7500.0f : 0.0f;
        if (breakTelemetry && i % 97 < 3) {                      // now and then a field is not a number
            const float bad = (i / 97) % 2 ? std::numeric_limits<float>::quiet_NaN() : std::numeric_limits<float>::infinity();
            switch ((i / 97) % 5) { case 0: s.rpm = bad; break; case 1: s.speed = bad; break; case 2: s.wload[1] = bad; break; case 3: s.texture = bad; break; default: s.latG = bad; break; }
        }
        int mag[S_COUNT]; float hz[S_COUNT];
        sy.Run(c, s, t, 0.01f, variant == 3 ? 0.4f : 0.85f, mag, hz);
        ++seen.steps;
        for (int k = 0; k < S_COUNT; ++k) {
            seen.finite = seen.finite && std::isfinite(hz[k]);
            seen.magMax = std::max(seen.magMax, mag[k]); seen.hzMax = std::max(seen.hzMax, hz[k]);
            if (mag[k] < 0) seen.magMax = 1 << 30;               // a negative strength counts as a failure too
        }
        seen.finite = seen.finite && std::isfinite(sy.boostMul) && std::isfinite(sy.boostAdd);
        seen.mulMin = std::min(seen.mulMin, sy.boostMul); seen.mulMax = std::max(seen.mulMax, sy.boostMul);
        seen.addMin = std::min(seen.addMin, sy.boostAdd); seen.addMax = std::max(seen.addMax, sy.boostAdd);
        if (i % 3 == 0) {
            std::snprintf(b, sizeof b, "%d", i); text += b;
            for (int k = 0; k < S_COUNT; ++k) { std::snprintf(b, sizeof b, " %d %.9g", mag[k], (double)hz[k]); text += b; }
            std::snprintf(b, sizeof b, " %.9g %.9g\n", (double)sy.boostMul, (double)sy.boostAdd); text += b;
        }
    }
}

static Config Variant(int v) {
    Config c;
    if (v == 1) c.fxWeight = 0;
    if (v == 2) { c.fxBrake = 0; c.fxWater = 0; c.fxSurface = 0; }
    if (v == 3) { c.weightHeavy = 1.0f; c.weightLight = 1.5f; c.weightCorner = 2.0f; }
    if (v == 4) { c.fxCrash = 0; c.fxLandBoost = 0; c.fxRoad = 0; c.fxEngine = 0; }
    if (v == 5) { c.weightLight = 0.0f; c.fxBrake = 1; c.weightHeavy = 2.0f; }
    if (v == 6) { c.maxForceMul = 1.2f; c.weightCorner = 2.0f; }
    return c;
}

static unsigned long long Fnv1a(const std::string& s) {
    unsigned long long h = 0xcbf29ce484222325ULL;
    for (unsigned char ch : s) { h ^= ch; h *= 0x100000001b3ULL; }
    return h;
}

TEST(the_output_of_the_scripted_drives_is_unchanged) {
    std::string text; Seen seen; g_log.clear();
    for (int v = 0; v < 8; ++v) { char h[32]; std::snprintf(h, sizeof h, "# variant %d\n", v); text += h; Drive(Variant(v), v, text, seen); }
    text += "# log\n" + g_log;
    const unsigned long long h = Fnv1a(text);
    if (h != kGolden) std::printf("  the output hash is now 0x%016llxULL (expected 0x%016llxULL): if the change in feel is intended, put the new one in kGolden\n", h, kGolden);
    CHECK(h == kGolden);
}
TEST(every_output_stays_finite_and_inside_its_limits) {
    for (int v = 0; v < 8; ++v) {
        std::string text; Seen seen; const Config c = Variant(v);
        Drive(c, v, text, seen);
        CHECK(seen.finite && seen.steps == 6000);
        CHECK(seen.magMax <= kHardTotalMax);
        CHECK(seen.hzMax <= kHardMaxHz + 1e-3f);
        CHECK(seen.mulMin >= 0.0f && seen.mulMax <= c.maxForceMul + 1e-4f);
    }
}
TEST(fields_that_are_not_numbers_never_reach_the_output) {
    for (int v : {0, 3, 6}) {
        std::string text; Seen seen;
        Drive(Variant(v), v, text, seen, true);
        CHECK(seen.finite);
        CHECK(seen.magMax <= kHardTotalMax);
    }
}
TEST(the_same_drive_gives_the_same_output_twice) {
    std::string a, b; Seen s1, s2;
    Drive(Variant(0), 0, a, s1); Drive(Variant(0), 0, b, s2);
    CHECK(a == b);
}

int main() { return mini::RunAll(); }
