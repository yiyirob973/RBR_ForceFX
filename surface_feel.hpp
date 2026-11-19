// Surface classes: which kinds of ground the plugin tells apart, which class each game material id belongs to, and the checks on
// the telemetry that spot ridged ground. How the road actually feels is worked out in road_feel.hpp.
// Pure logic, no Windows or game access, so it can be tested on its own (see tests/surface_feel_test.cpp).
//
// RBR_ForceFX.cpp includes this file inside its anonymous namespace, so the Surf namespace here is the same one the
// surface reader extends. The standard headers are already included there.
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
// The values the road effect uses for one side of the car, worked out by Road::Derive (road_feel.hpp).
struct Feel {
    float amp;        // multiplier on the road strength
    float grainAdd;   // added to the base grain (-1..1): how random the strength and pitch are
    float dropout;    // extra drop of the weakest hits (0..0.6): gaps between grains
    float hz;         // multiplier on the road pitch
    float atk;        // how fast the effect rises (0.05..1)
    float rel;        // multiplier on how fast it dies away (large = short, sharp hits)
    float base;       // multiplier on the everyday strength between hits
    float spark;      // chance per 12 ms and side of a single hard hit (a stone, a crack)
    float sparkMul;   // strength of that hit
    float floor;      // the least texture level: smooth ground still gives a faint feel
    float ridge;      // metres between ridges (cobbles, planks). 0 = none. With a ridge the pitch is locked to speed: hits per second = speed / ridge
    float ridgeMix;   // how much of the pitch comes from the ridges (0..1)
};
constexpr Feel kNeutral = {1.0f, 0.0f, 0.0f, 1.0f, 0.55f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};

// Which class a material id belongs to. The gravel and tarmac families up to 55 are verified against the game's material table;
// the rest come from the table's order and from driving, so [Surface] Map= can correct any of them (the class also picks the ground when an id is not in the game's material table).
// Anything not listed is "unknown": the ground then comes from the material name, or is plain hard ground. The game's own material table has no cobblestone or wooden
// bridge entries, so those are assigned by id with Map= (the log names every unmapped id the car drives on).
inline int DefaultClass(int id) {
    auto in = [id](int a, int b) { return id >= a && id <= b; };
    if (id == 88) return C_SNOW;                                                                                  // snow wall base: only seen on snow stages
    if (in(1, 23) || in(33, 35) || in(41, 43) || in(49, 51) || in(153, 155) || in(177, 199)) return C_GRAVEL;   // gravel (US2 / BR2 sets too)
    if (in(133, 135) || in(141, 143) || in(149, 151) || in(161, 163)) return C_GRAVEL;                            // dirt / gravel roads
    if (in(37, 39) || in(45, 47) || in(53, 55) || in(65, 87)) return C_TARMAC;                                    // six tarmac families of three: clean, sprinkled, covered
    if (in(101, 103)) return C_SNOW;                                                                              // snow on gravel (shallow, medium, deep)
    if (in(117, 119)) return C_ICE;                                                                               // snow on ice (shallow, medium, deep)
    if (in(93, 99) || in(105, 115) || in(121, 131) || in(137, 139) || id == 157) return C_SOFT;                  // grass, rough verge, water
    return C_NONE;                                                                                                // unknown, trees, rocks, walls: nothing changes
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
    float ema = 0.0f;                    // smoothed rough-per-speed level (x100)
    float score = 0.0f;                  // the last result, 0..1, kept so the plugin can show and log it
    bool init = false;
    static constexpr float kMinSpeedMs = 8.0f;
    static float Ramp(float x, float lo, float hi) {
        if (!(hi > lo)) hi = lo + 0.01f;
        return x <= lo ? 0.0f : (x >= hi ? 1.0f : (x - lo) / (hi - lo));
    }
    // Returns 0 (plain tarmac) .. 1 (cobbles). dt in seconds, tau is the smoothing time.
    float update(float rough, float speedMs, bool airborne, float dt, float lo, float hi, float tau = 0.8f) {
        if (!std::isfinite(rough) || !std::isfinite(speedMs) || !(speedMs >= kMinSpeedMs)) { ema = 0.0f; init = false; score = 0.0f; return 0.0f; }
        if (!airborne) {
            const float x = std::max(0.0f, rough) / speedMs * 100.0f;
            if (!init) { ema = x; init = true; }
            else if (dt > 0.0f && std::isfinite(dt)) ema += (x - ema) * (1.0f - std::exp(-dt / std::max(0.05f, tau)));
        }
        score = init ? Ramp(ema, lo, hi) : 0.0f;
        return score;
    }
};

}  // namespace Surf
