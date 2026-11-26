// Surface classes: which kinds of ground the plugin tells apart and which class each game material id belongs to.
// How the road actually feels is worked out in road_feel.hpp.
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

enum Cls { C_NONE = 0, C_TARMAC, C_GRAVEL, C_SNOW, C_ICE, C_SOFT, C_COUNT };
inline const char* ClsName(int c) {
    static const char* const n[C_COUNT] = {"unknown", "tarmac", "gravel", "snow", "ice", "soft"};
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
};
constexpr Feel kNeutral = {1.0f, 0.0f, 0.0f, 1.0f, 0.55f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f};

// Which class a material id belongs to. The gravel and tarmac families up to 55 are verified against the game's material table;
// the rest come from the table's order and from driving, so [Surface] Map= can correct any of them (the class also picks the ground when an id is not in the game's material table).
// Anything not listed is "unknown": the ground then comes from the material name, or is plain hard ground.
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
        {"snow", C_SNOW}, {"ice", C_ICE}, {"soft", C_SOFT}, {"grass", C_SOFT}};
    for (const Alias& x : a) if (n == x.name) return x.cls;
    return -1;
}

// "88:snow,92:none,129:soft": correct the class of single ids. Returns how many entries were applied.
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

}  // namespace Surf
