// Surface feel: which kinds of ground the plugin tells apart and how each one reshapes the road vibration.
// Pure logic, no Windows or game access, so it can be tested on its own (see tests/surface_feel_test.cpp).
//
// RBR_ForceFX.cpp includes this file INSIDE its anonymous namespace at the place the code used to live, so the
// Surf namespace here is the same one the surface reader extends. The standard headers are already included there.
#pragma once
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

namespace Surf {

namespace detail {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

enum Cls { C_NONE = 0, C_TARMAC, C_GRAVEL, C_SNOW, C_ICE, C_SOFT, C_COBBLE, C_WOOD, C_COUNT };
inline const char* ClsName(int c) {
    static const char* const n[C_COUNT] = {"unknown", "tarmac", "gravel", "snow", "ice", "soft", "cobble", "wood"};
    return c >= 0 && c < C_COUNT ? n[c] : "?";
}
// Names used for the keys in the [Surface] section of ForceFX.ini ("GravelAmp", "CobbleRidge", ...).
inline const char* ClsIniName(int c) {
    static const char* const n[C_COUNT] = {"", "Tarmac", "Gravel", "Snow", "Ice", "Soft", "Cobble", "Wood"};
    return c >= 0 && c < C_COUNT ? n[c] : "";
}

struct Feel {
    float amp;        // multiplier on the road strength
    float grainAdd;   // added to RoadGrain (0..1): how random the strength and pitch are
    float dropout;    // extra drop of the weakest hits (0..0.6): gaps between grains
    float hz;         // multiplier on the road pitch
    float atk;        // how fast the effect rises (0.05..1)
    float rel;        // multiplier on how fast it dies away (large = short, sharp hits)
    float base;       // multiplier on the everyday strength between hits
    float spark;      // chance per 12 ms and side of a single hard hit (a stone, a crack)
    float sparkMul;   // strength of that hit
    float floor;      // the least texture level: smooth ground still gives a faint feel
    float ridge;      // metres between ridges (cobbles, planks). 0 = none. With a ridge the pitch is locked to speed: hits per second = speed / ridge
    float ridgeMix;   // blended result only, never set from the ini: how much of the pitch comes from the ridges (0..1)
};
constexpr Feel kNeutral = {1.0f, 0.0f, 0.0f, 1.0f, 0.55f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};

constexpr int kFeelParams = 11;     // values that can be set per surface (the ini keys); the 11th is Ridge
constexpr int kBlendParams = 10;    // the first ten blend linearly; Ridge is combined separately

constexpr float kRidgeMin = 0.03f, kRidgeMax = 1.0f;   // metres: the range the ini may set

// Default values per class. Hard ground that is not smooth gets a ridge so its pitch follows speed:
//   cobble: small stones, a hit every ~12 cm, crisp and regular, strong even with little suspension movement (floor)
//   wood:   planks across the road, a hit every ~25 cm, rounder and a little hollow
inline Feel* DefaultFeels() {
    static Feel f[C_COUNT] = {
        kNeutral,                                                                                      // unknown
        kNeutral,                                                                                      // tarmac
        {1.00f,  0.10f, 0.25f, 1.10f, 0.60f, 1.8f, 1.00f, 0.07f, 1.6f, 0.20f, 0.0f, 0.0f},           // gravel
        {0.55f, -0.70f, 0.00f, 0.50f, 0.12f, 0.3f, 0.90f, 0.00f, 1.0f, 0.22f, 0.0f, 0.0f},           // snow
        {0.90f, -0.40f, 0.00f, 1.35f, 0.90f, 2.2f, 0.30f, 0.10f, 2.4f, 0.30f, 0.0f, 0.0f},           // ice
        {0.90f, -0.20f, 0.05f, 0.80f, 0.35f, 0.5f, 1.00f, 0.00f, 1.0f, 0.18f, 0.0f, 0.0f},           // soft
        {1.25f, -0.60f, 0.00f, 1.00f, 0.90f, 2.5f, 1.00f, 0.04f, 1.5f, 0.40f, 0.12f, 0.0f},          // cobble
        {1.00f, -0.50f, 0.00f, 1.00f, 0.70f, 1.2f, 1.00f, 0.00f, 1.0f, 0.30f, 0.25f, 0.0f}};         // wood
    return f;
}

inline const char* const* FeelKeys() {
    static const char* const k[kFeelParams] = {"Amp", "Grain", "Dropout", "Hz", "Attack", "Release", "Base", "Spark", "SparkMul", "Floor", "Ridge"};
    return k;
}
inline float Feel::* const* FeelMembers() {
    static float Feel::* const m[kFeelParams] = {&Feel::amp, &Feel::grainAdd, &Feel::dropout, &Feel::hz, &Feel::atk,
                                                 &Feel::rel, &Feel::base, &Feel::spark, &Feel::sparkMul, &Feel::floor, &Feel::ridge};
    return m;
}

// Which class a material id belongs to. The gravel and tarmac families up to 55 are verified against the game's material table;
// the rest come from the table's order and from driving, so [Surface] Map= can correct any of them.
// Anything not listed is "unknown" and leaves the feel unchanged. The game's own material table has no cobblestone or wooden
// bridge entries, so those are assigned by id with Map= (the log names every unmapped id the car drives on).
inline int DefaultClass(int id) {
    auto in = [id](int a, int b) { return id >= a && id <= b; };
    if (id == 88) return C_SNOW;                                                                                  // snow wall base: only seen on snow stages
    if (in(1, 23) || in(33, 35) || in(41, 43) || in(49, 51) || in(153, 155) || in(177, 199)) return C_GRAVEL;   // gravel (US2 / BR2 sets too)
    if (in(133, 135) || in(141, 143) || in(149, 151) || in(161, 163)) return C_GRAVEL;                            // dirt / gravel roads
    if (in(37, 39) || in(45, 47) || in(53, 55) || in(65, 87)) return C_TARMAC;                                    // six tarmac families of three: clean, sprinkled, covered
    if (in(101, 103)) return C_SNOW;                                                                              // snow on gravel (shallow, medium, deep)
    if (in(117, 119)) return C_ICE;                                                                               // snow on ice (shallow, medium, deep)
    if (in(93, 99) || in(105, 115) || in(121, 131) || in(137, 139)) return C_SOFT;                                // grass, rough verge
    return C_NONE;                                                                                                // unknown, trees, rocks, walls, water: nothing changes
}

inline int* IdMap() { static int m[256] = {}; return m; }       // per-id override of the class, stored as class+1 (0 = use the built-in table)
inline int ClassOfId(int id) {
    if (id < 0 || id > 255) return C_NONE;
    return IdMap()[id] > 0 ? IdMap()[id] - 1 : DefaultClass(id);
}

// A surface type typed in Map=, lower case. Accepts the class names and a few obvious alternatives. -1 = not a type.
inline int ClassFromName(const std::string& n) {
    struct Alias { const char* name; int cls; };
    static const Alias a[] = {
        {"none", C_NONE}, {"unknown", C_NONE},
        {"tarmac", C_TARMAC}, {"asphalt", C_TARMAC}, {"road", C_TARMAC},
        {"gravel", C_GRAVEL}, {"dirt", C_GRAVEL},
        {"snow", C_SNOW}, {"ice", C_ICE}, {"soft", C_SOFT}, {"grass", C_SOFT},
        {"cobble", C_COBBLE}, {"cobbles", C_COBBLE}, {"cobblestone", C_COBBLE}, {"cobblestones", C_COBBLE},
        {"wood", C_WOOD}, {"wooden", C_WOOD}, {"bridge", C_WOOD}, {"woodenbridge", C_WOOD}, {"planks", C_WOOD}, {"timber", C_WOOD}};
    for (const Alias& x : a) if (n == x.name) return x.cls;
    return -1;
}

// "88:snow,92:none,129:soft,200:cobble": correct the class of single ids. Returns how many entries were applied.
// Entries that can't be used (no number, a number outside 0..255, a type that doesn't exist) are skipped and, when
// `rejected` is given, listed there so the plugin can say so in the log instead of ignoring them silently.
inline int ParseIdMap(const std::string& m, std::vector<std::string>* rejected = nullptr) {
    int applied = 0;
    size_t p = 0;
    while (p < m.size()) {
        size_t q = m.find(',', p);
        std::string t = m.substr(p, q == std::string::npos ? std::string::npos : q - p);
        p = q == std::string::npos ? m.size() : q + 1;
        const size_t c = t.find(':');
        if (c == std::string::npos) { if (rejected && t.find_first_not_of(" \t\r") != std::string::npos) rejected->push_back(t + " (needs id:type)"); continue; }
        std::string idText = t.substr(0, c);
        idText.erase(std::remove_if(idText.begin(), idText.end(), [](char ch) { return ch == ' ' || ch == '\t' || ch == '\r'; }), idText.end());
        if (idText.empty() || idText.size() > 3 || !std::all_of(idText.begin(), idText.end(), [](char ch) { return ch >= '0' && ch <= '9'; })) { if (rejected) rejected->push_back(t + " (the id must be a number from 0 to 255)"); continue; }
        const int id = atoi(idText.c_str());
        std::string n = t.substr(c + 1);
        n.erase(std::remove_if(n.begin(), n.end(), [](char ch) { return ch == ' ' || ch == '\t' || ch == '\r'; }), n.end());
        for (char& ch : n) ch = (char)tolower((unsigned char)ch);
        const int k = ClassFromName(n);
        if (k < 0) { if (rejected) rejected->push_back(t + " (unknown type '" + n + "')"); continue; }
        if (id > 255) { if (rejected) rejected->push_back(t + " (the id must be a number from 0 to 255)"); continue; }
        IdMap()[id] = k + 1; ++applied;
    }
    return applied;
}

// Weighted mix of the per-class values (w sums to 1), then scaled by the strength (0 = no change at all, 1 = as tuned, up to 1.5).
// Ridges don't average (half a cobble and half tarmac is not a 6 cm ridge): the ridge spacing is the weighted spacing of the
// classes that have one, and ridgeMix says how much of the mix they make up, so the pitch fades between random and speed-locked.
inline Feel Blend(const float w[C_COUNT], float strength) {
    using detail::clampf;
    const Feel* gf = DefaultFeels();
    Feel r = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float Feel::* const* m = FeelMembers();
    for (int p = 0; p < kBlendParams; ++p) {
        float v = 0;
        for (int i = 0; i < C_COUNT; ++i) v += w[i] * gf[i].*m[p];
        r.*m[p] = kNeutral.*m[p] + (v - kNeutral.*m[p]) * strength;
    }
    float rw = 0, rs = 0;
    for (int i = 0; i < C_COUNT; ++i) if (gf[i].ridge > 0.0f) { rw += w[i]; rs += w[i] * gf[i].ridge; }
    r.ridge = rw > 0.0f ? clampf(rs / rw, kRidgeMin, kRidgeMax) : 0.0f;
    r.ridgeMix = clampf(rw * strength, 0.0f, 1.0f);
    r.atk = clampf(r.atk, 0.05f, 1.0f); r.rel = clampf(r.rel, 0.05f, 4.0f); r.hz = clampf(r.hz, 0.3f, 2.0f);
    r.dropout = clampf(r.dropout, 0.0f, 0.6f); r.spark = clampf(r.spark, 0.0f, 0.5f); r.floor = clampf(r.floor, 0.0f, 0.6f);
    r.amp = std::max(0.0f, r.amp); r.base = std::max(0.0f, r.base); r.sparkMul = clampf(r.sparkMul, 0.5f, 3.0f);
    return r;
}

// Pitch of a ridged surface: hits per second = speed / spacing. When that is above what the wheel can play it is folded down
// by octaves (halved until it fits) rather than pinned at the ceiling, so the pitch keeps following speed.
inline float RidgeHz(float speedMs, float ridgeM, float minHz, float maxHz) {
    if (ridgeM <= 0.0f || speedMs <= 0.0f) return minHz;
    float hz = speedMs / ridgeM;
    for (int i = 0; i < 8 && hz > maxHz; ++i) hz *= 0.5f;
    return detail::clampf(hz, minHz, maxHz);
}

// Spots cobbles (and anything else hard but made of separate ridges) on ground the game calls tarmac, from the telemetry alone:
// how rough the tyres' strut force is for the speed the car is doing. A probe drive on a cobbled stage (ground id 77) against plain
// tarmac (id 37) gave a rough-per-speed level (x100) of about 0.4 on tarmac (75% of it under 0.5) and about 1.0 on cobbles
// (75% of it above 0.8), the two cleanly apart once smoothed over about a second, so 0.5 to 1.0 is the default ramp.
// Roughness means little at crawling speed and nothing in the air, so below 8 m/s the detector resets and in the air it holds.
struct CobbleDetector {
    float ema = 0.0f;
    bool init = false;
    static constexpr float kMinSpeedMs = 8.0f;
    static float Ramp(float x, float lo, float hi) {
        if (!(hi > lo)) hi = lo + 0.01f;
        return x <= lo ? 0.0f : (x >= hi ? 1.0f : (x - lo) / (hi - lo));
    }
    // Returns 0 (plain tarmac) .. 1 (cobbles). dt in seconds, tau is the smoothing time.
    float update(float rough, float speedMs, bool airborne, float dt, float lo, float hi, float tau = 0.8f) {
        if (!std::isfinite(rough) || !std::isfinite(speedMs) || !(speedMs >= kMinSpeedMs)) { ema = 0.0f; init = false; return 0.0f; }
        if (!airborne) {
            const float x = std::max(0.0f, rough) / speedMs * 100.0f;
            if (!init) { ema = x; init = true; }
            else if (dt > 0.0f && std::isfinite(dt)) ema += (x - ema) * (1.0f - std::exp(-dt / std::max(0.05f, tau)));
        }
        return init ? Ramp(ema, lo, hi) : 0.0f;
    }
};

}  // namespace Surf
