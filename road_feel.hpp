// Road feel: how the road vibration is shaped by the ground under a tyre and by what the car is doing.
// Pure logic, no Windows or game access, so it can be tested on its own (see tests/road_feel_test.cpp).
//
// There is no "gravel feels like this, tarmac feels like that" table. Each ground is described by a few physical properties
// (how hard it is, how much loose material lies on it, how coarse its grain is, how slippery it is), worked out from the
// game's own material names ("Gravel_Coarse_Deep_Wet", "SnowOnIce_Shallow", ...). The way the road feels then follows from
// those properties together with what the telemetry says the car is doing: how fast it goes, how much the wheels slip, and
// how rough the tyres measure. So the same gravel feels calm when you roll over it gently and violent when you slide on it.
//
// Include after surface_feel.hpp (Surf::Feel, Surf::Cls) and materials.hpp (ffx::kMaterials). In RBR_ForceFX.cpp it is
// included inside the anonymous namespace, right after surface_feel.hpp. How the result reaches the wheel (three sine bands) is in
// road_bands.hpp.
#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include "surface_model.hpp"

namespace Road {

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }        // NaN comes out as 0
inline float clampf(float v, float lo, float hi) { return !(v > lo) ? lo : (v > hi ? hi : v); }   // NaN comes out as lo
inline float smooth01(float a, float b, float x) { const float t = c01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
}  // namespace detail

constexpr float kBaseGrain = 0.9f;       // how random the road strength and pitch are on ground with no special character (0 smooth .. 1 coarse)

// What a tyre is rolling on, each 0..1.
struct Ground {
    float hard = 1.0f;       // how stiff the surface is: tarmac 1, hard dirt .8, deep gravel .4, snow .15 to .35
    float loose = 0.0f;      // loose material on top that can be thrown or ploughed: clean tarmac 0, gravel .75 to 1
    float coarse = 0.5f;     // grain size: fine .25, medium .55, coarse .9
    float slick = 0.0f;      // how little grip there is: dry 0, wet tarmac .15, snow on ice 1
};

inline Ground Mix(const Ground& a, const Ground& b, float t) {
    Ground r;
    r.hard = a.hard + (b.hard - a.hard) * t;       r.loose = a.loose + (b.loose - a.loose) * t;
    r.coarse = a.coarse + (b.coarse - a.coarse) * t; r.slick = a.slick + (b.slick - a.slick) * t;
    return r;
}

// ---------------------------------------------------------------- ground from the game's material names
namespace detail {
inline bool Has(const std::string& n, const char* w) { return n.find(w) != std::string::npos; }

// wet: 0 dry, 1 damp, 2 wet (from the game's table). False for anything that is not ground.
inline bool FromName(const std::string& n, int wet, Ground& g) {
    const float w = wet >= 2 ? 1.0f : (wet == 1 ? 0.5f : 0.0f);
    g = Ground();
    const float grain = Has(n, "Coarse") ? 0.9f : (Has(n, "Medium") ? 0.55f : 0.25f);
    if (Has(n, "SnowOnGravel")) {
        g.hard = Has(n, "Deep") ? 0.15f : (Has(n, "Medium") ? 0.25f : 0.35f);
        g.loose = Has(n, "Deep") ? 0.7f : (Has(n, "Medium") ? 0.6f : 0.5f);
        g.coarse = 0.1f; g.slick = 0.5f;
    } else if (Has(n, "SnowOnIce")) {
        g.hard = Has(n, "Deep") ? 0.35f : (Has(n, "Medium") ? 0.5f : 0.65f);
        g.loose = 0.2f; g.coarse = 0.05f;
        g.slick = Has(n, "Deep") ? 0.8f : (Has(n, "Medium") ? 0.9f : 1.0f);
    } else if (Has(n, "Tarmac")) {
        g.hard = 1.0f; g.coarse = grain;
        g.loose = Has(n, "Covered") ? 0.75f : (Has(n, "Sprinkled") ? 0.35f : 0.0f);
        g.slick = 0.15f * w;
    } else if (Has(n, "Gravel")) {
        g.hard = Has(n, "Deeper") ? 0.4f : (Has(n, "Deep") ? 0.55f : 0.75f);
        g.loose = Has(n, "Deeper") ? 1.0f : (Has(n, "Deep") ? 0.9f : 0.75f);
        g.coarse = grain; g.slick = 0.20f * w;
    } else if (Has(n, "Dirt")) {
        g.hard = Has(n, "Soft") ? 0.4f : (Has(n, "Medium") ? 0.6f : 0.8f);
        g.loose = Has(n, "Soft") ? 0.75f : (Has(n, "Medium") ? 0.6f : 0.45f);
        g.coarse = 0.35f; g.slick = 0.35f * w;
    } else if (Has(n, "Grass")) {
        g.hard = Has(n, "Soft") ? 0.3f : (Has(n, "Medium") ? 0.4f : 0.5f);
        g.loose = Has(n, "Soft") ? 0.45f : (Has(n, "Medium") ? 0.3f : 0.2f);
        g.coarse = 0.2f; g.slick = 0.25f * w;
    } else if (Has(n, "Rough")) {
        g.hard = 0.45f; g.loose = Has(n, "VeryRough") ? 0.8f : 0.65f;
        g.coarse = Has(n, "VeryRough") ? 1.0f : 0.8f; g.slick = 0.20f * w;
    } else if (Has(n, "Water")) {
        g.hard = 0.3f; g.loose = 0.25f; g.coarse = 0.1f; g.slick = 0.6f;
    } else {
        return false;
    }
    return true;
}

struct Entry { Ground g; bool known = false; int wet = 0; };
inline Entry* Table() {
    static Entry t[256];
    static bool built = false;
    if (!built) {
        built = true;
        for (const ffx::MaterialInfo& m : ffx::kMaterials)
            if (m.id >= 0 && m.id < 256 && m.cls >= 0) { t[m.id].known = FromName(m.name, m.wet, t[m.id].g); t[m.id].wet = m.wet; }
    }
    return t;
}
}  // namespace detail

// The ground for a game material id. False for objects, undefined ids and anything that is not in the table.
inline bool GroundOfId(int id, Ground& out) {
    if (id < 0 || id > 255) return false;
    const detail::Entry& e = detail::Table()[id];
    if (!e.known) return false;
    out = e.g;
    return true;
}

// How wet the ground is for a game material id: 0 dry, 1 damp, 2 wet (from the game's table, or a materials.csv override). 0 for anything unknown.
inline int WetOfId(int id) {
    if (id < 0 || id > 255) return 0;
    return detail::Table()[id].wet;
}

// The materials.csv overrides (the file the audio reads, ForceFXAudio\materials.csv) for the road too, so one file corrects both: for every id the
// override changes, the wetness and the ground are redone, and the reader's class follows the override unless [Surface] Map= already set one.
// base: the built-in table; ovr: the same table after loadOverrides. Returns how many ids it changed.
inline int ApplyMaterialOverrides(const ffx::MaterialTable& base, const ffx::MaterialTable& ovr) {
    int changed = 0;
    for (int id = 0; id < 256; ++id) {
        if (ovr.cls(id) == base.cls(id) && ovr.wet(id) == base.wet(id)) continue;
        ++changed;
        detail::Entry& e = detail::Table()[id];
        e.wet = ovr.wet(id);
        const std::string name = base.name(id);
        if (ovr.cls(id) < 0) { e.known = false; continue; }                                  // the override says it is not ground
        if (name == "unknown") e.known = false;                                              // a new id: its class gives the ground
        else e.known = detail::FromName(name, e.wet, e.g);                                   // a known material, wetter or drier
        if (ovr.cls(id) != base.cls(id) && Surf::IdMap()[id] == 0) Surf::IdMap()[id] = SurfaceModel::SurfClassOfAudio(ovr.cls(id)) + 1;
    }
    return changed;
}

// A rough ground for a surface class, for ids the material table does not have.
inline Ground GroundOfClass(int cls) {
    Ground g;
    switch (cls) {
        case Surf::C_TARMAC: g.hard = 1.0f; g.coarse = 0.55f; break;
        case Surf::C_GRAVEL: g.hard = 0.55f; g.loose = 0.85f; g.coarse = 0.55f; break;
        case Surf::C_SNOW:   g.hard = 0.25f; g.loose = 0.6f; g.coarse = 0.1f; g.slick = 0.5f; break;
        case Surf::C_ICE:    g.hard = 0.65f; g.loose = 0.2f; g.coarse = 0.05f; g.slick = 1.0f; break;
        case Surf::C_SOFT:   g.hard = 0.4f; g.loose = 0.35f; g.coarse = 0.2f; g.slick = 0.1f; break;
        default: break;
    }
    return g;
}

// What to use under one tyre. `id` is the material id the reader found (-1 = none) and `cls` the class it settled on.
// A Map= entry for the id wins, then the material's own name, then the class; with nothing known the ground is plain
// hard ground, which adds no special character.
inline Ground GroundForTyre(int id, int cls) {
    Ground g;
    const bool mapped = id >= 0 && id < 256 && Surf::IdMap()[id] > 0;
    if (!mapped && GroundOfId(id, g)) return g;
    if (cls != Surf::C_NONE) return GroundOfClass(cls);
    return Ground();
}

// ---------------------------------------------------------------- the road character
struct Drive {
    float activity = 0.0f;   // 0..1: how hard the tyres are working the ground (speed and wheel slip)
    float spin = 0.0f;       // 0..1: wheelspin on the driven tyres (a tyre faster than the ground under power)
    float slide = 0.0f;      // 0..1: tyres sliding sideways or locked under braking (Tyre::Classify, measured against this ground's grip limit)
};

// Turns a ground and what the car is doing into the values the road effect uses (see Surf::Feel). Smooth in every input:
// ground that is part gravel and part snow feels part way between, and nothing jumps when a property changes a little.
inline Surf::Feel Derive(const Ground& g, const Drive& d) {
    using detail::c01; using detail::clampf; using detail::smooth01;
    const float hard = c01(g.hard), soft = 1.0f - hard, loose = c01(g.loose), coarse = c01(g.coarse), slick = c01(g.slick);
    const float act = c01(d.activity);
    const float hs = hard * slick;                                    // hard and slippery: ice, wet glaze
    const float lc = loose * coarse;                                  // loose and coarse: stones that rattle

    Surf::Feel f = Surf::kNeutral;
    f.amp      = (1.0f - 0.55f * soft * (1.0f - 0.5f * loose)) * (1.0f - 0.12f * slick);   // soft ground absorbs
    f.grainAdd = 0.24f * lc - 0.85f * soft * (1.0f - loose) - 0.45f * hs;                                  // stones are random, soft ground and ice are smooth
    f.dropout  = 0.35f * lc + 0.10f * soft * loose;                                                           // gaps between grains of loose stuff
    f.hz       = 1.0f - 0.65f * soft + 0.45f * loose + 0.70f * hs;                                            // soft ground is low and dull, loose and glazed ground is high
    f.atk      = 0.12f + 0.50f * hard + 0.60f * hs + 0.30f * lc;                                              // soft ground rises slowly
    f.rel      = 0.30f + 0.80f * hard + 1.80f * lc + 1.40f * hs;                                           // hard and stony ground lets go fast: short, sharp hits
    f.base     = 1.0f - 1.00f * hs;                                                                           // glaze is quiet between its cracks
    f.spark    = 0.08f * loose * act + 0.12f * hs;                                                            // a stone flung up, a crack in the glaze
    f.sparkMul = 1.0f + 0.8f * loose * act + 1.4f * hs;
    f.floor    = 0.17f * soft + 0.10f * loose + 0.15f * slick;                                                // ground that is never silent while the car moves

    // Wheelspin and slides work the ground differently. Both dig into loose ground (strength, flung stones) and scrub hard ground
    // (a higher chatter, never silent), but a tyre sliding on ice has little to grab, so it stays quiet.
    const float sp = c01(d.spin), sl = c01(d.slide) * (1.0f - 0.7f * hs);
    f.amp     *= 1.0f + (0.50f * sl + 0.30f * sp) * (0.5f + 0.5f * loose);
    f.hz      *= 1.0f + 0.25f * sl + 0.10f * sp;
    f.grainAdd += 0.15f * sl + 0.05f * sp;
    f.spark   += (0.10f * sl + 0.06f * sp) * loose;
    f.sparkMul += (0.6f * sl + 0.3f * sp) * loose;
    f.floor   += 0.20f * sl + 0.10f * sp;

    f.atk = clampf(f.atk, 0.05f, 1.0f); f.rel = clampf(f.rel, 0.05f, 4.0f); f.hz = clampf(f.hz, 0.3f, 2.0f);
    f.dropout = clampf(f.dropout, 0.0f, 0.6f); f.spark = clampf(f.spark, 0.0f, 0.5f); f.floor = clampf(f.floor, 0.0f, 0.6f);
    f.amp = std::max(0.0f, f.amp); f.base = std::max(0.0f, f.base); f.sparkMul = clampf(f.sparkMul, 0.5f, 3.0f);
    f.grainAdd = clampf(f.grainAdd, -1.0f, 1.0f);
    return f;
}

// How hard the tyres are working the ground, 0..1, from the car's speed (0..1 of full speed) and the wheels' slip (0..1).
// Standing still is no activity at all. Wheels slipping count most, because that is what throws stones and chatters.
inline float Activity(float speedN, float slip) {
    return detail::c01(0.45f * detail::c01(speedN) + 1.0f * detail::c01(slip));
}


}  // namespace Road
