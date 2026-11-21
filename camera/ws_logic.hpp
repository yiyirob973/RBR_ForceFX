// WindscreenFX logic that needs neither Windows nor Direct3D, so it can be unit-tested anywhere (tests/ws_logic_test.cpp).
//
//  * Ground      what the tyres are on (from the surface scan's material ids) -> how much is thrown at the glass, what it is
//                (mud, dust, slush, road spray, grass) and how wet it is.
//  * Palette     the colour of each kind of dirt.
//  * Blade       where a wiper blade has been between two frames: the area it swept, from the blade's real two end points.
//  * Extent      the length and direction of a wiper mesh, measured from its vertices.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include "../materials.hpp"

namespace ffx {
namespace cam {
namespace wsl {

// ================================================================================================ palette
enum Tint { kMud = 0, kSlush = 1, kDust = 2, kSpray = 3, kGrass = 4, kTints = 5 };
struct Rgb { float r, g, b; };

inline float mix1(float a, float b, float t) { return a + (b - a) * t; }
inline Rgb mixRgb(Rgb a, Rgb b, float t) { return {mix1(a.r, b.r, t), mix1(a.g, b.g, t), mix1(a.b, b.b, t)}; }

// Base colour of each kind of dirt (before the per-splat shade). 'mud' is the colour from the ini, the others follow it, so a red
// clay stage gives red dust and a red-brown spray instead of fixed grey ones.
inline Rgb tintColour(int tint, Rgb mud) {
    switch (tint) {
        case kSlush: return {0.88f, 0.90f, 0.94f};
        case kDust:  return mixRgb(mud, {0.80f, 0.74f, 0.64f}, 0.62f);
        case kSpray: return mixRgb(mud, {0.27f, 0.27f, 0.29f}, 0.80f);
        case kGrass: return mixRgb({mud.r * 0.80f, mud.g * 0.80f, mud.b * 0.80f}, {0.25f, 0.32f, 0.14f}, 0.50f);
        default:     return mud;
    }
}
// Colour of the frosted patch behind a splat.
inline Rgb frostColour(int tint, Rgb mud) {
    if (tint == kSlush) return {0.90f, 0.92f, 0.96f};
    const Rgb c = tintColour(tint, mud);
    return {std::min(1.f, 0.50f + c.r * 0.9f), std::min(1.f, 0.50f + c.g * 0.9f), std::min(1.f, 0.50f + c.b * 0.9f)};
}

// ================================================================================================ ground
struct GroundCfg { float mudDry = 0.15f, mudDamp = 0.7f, mudWet = 1.f, mudSnow = 0.6f; };

struct Ground {
    bool  ok = false;           // at least one tyre is on real ground
    float loose = 0.f;          // 0..1 how much loose material the tyres can throw (clean tarmac 0, deep gravel 1)
    float mudMul = 0.f;         // multiplier for clods from how wet that loose material is (the ini's MudDry / MudDamp / MudWet / MudSnow)
    float spray = 0.f;          // 0..1 water the tyres throw up (wet tarmac, puddles, wet dirt)
    float dust = 0.f;           // 0..1 dry dust and grit
    float snow = 0.f;           // 0..1 share of the tyres that are on snow
    float clod[kTints] = {0, 0, 0, 0, 0};   // what the clods are, as shares that add up to 1
    float fine[kTints] = {0, 0, 0, 0, 0};   // what the fine spray / dust is, as shares that add up to 1
};

// ids: the material id under FL, FR, RL, RR. Front tyres count a little more (they are closest to the glass line).
inline Ground groundFromIds(const int ids[4], const rsa::MaterialTable& tab, const GroundCfg& cfg) {
    static const float kW[4] = {0.30f, 0.30f, 0.20f, 0.20f};
    Ground g;
    float wSum = 0.f, looseSum = 0.f, mudSum = 0.f, spraySum = 0.f, dustSum = 0.f, snowSum = 0.f, clodW[kTints] = {0, 0, 0, 0, 0}, fineW[kTints] = {0, 0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) {
        const int id = ids[i];
        const int cls = tab.cls(id);
        if (cls < 0) continue;                                         // not ground (objects, unknown id): this tyre says nothing
        const double lo = tab.looseness(id);
        const float L = lo < 0.0 ? 0.f : (float)lo, w = kW[i];
        const int wet = tab.wet(id);
        const float wetN = wet <= 0 ? 0.f : (wet == 1 ? 0.5f : 1.f);
        const bool water = std::strstr(tab.name(id), "Water") != nullptr;
        wSum += w; looseSum += w * L;
        float mudI = wet <= 0 ? cfg.mudDry : (wet == 1 ? cfg.mudDamp : cfg.mudWet);
        float sprayI = 0.f, dustI = 0.f; int clodT = kMud, fineT = kMud;
        if (cls == 3) {                                                // snow
            mudI = cfg.mudSnow; clodT = kSlush; fineT = kSlush; dustI = L * 0.6f; snowSum += w;
        } else if (cls == 0) {                                         // tarmac: clean road throws water when it is wet and (almost) nothing otherwise
            clodT = kDust; fineT = wet > 0 ? kSpray : kDust;
            sprayI = wetN * 0.9f; dustI = wet > 0 ? 0.f : L * 0.5f;
        } else if (cls == 1) {                                         // gravel, dirt, water
            if (water) { clodT = kSpray; fineT = kSpray; sprayI = 1.f; }
            else if (wet <= 0) { clodT = kDust; fineT = kDust; dustI = L * 0.6f; }
            else { clodT = kMud; fineT = kMud; sprayI = wetN * 0.8f; }
        } else {                                                       // grass, rough ground
            clodT = wet > 0 ? kGrass : kDust; fineT = clodT; sprayI = wetN * 0.5f; dustI = wet > 0 ? 0.f : L * 0.4f;
        }
        mudSum += w * L * mudI; spraySum += w * sprayI; dustSum += w * dustI;
        clodW[clodT] += w * L * mudI;
        fineW[fineT] += w * (sprayI + dustI);
    }
    if (wSum <= 0.f) return g;
    g.ok = true;
    g.loose = looseSum / wSum;
    g.mudMul = looseSum > 1e-5f ? mudSum / looseSum : 0.f;
    g.spray = spraySum / wSum; g.dust = dustSum / wSum; g.snow = snowSum / wSum;
    float cs = 0.f, fs = 0.f; for (int t = 0; t < kTints; ++t) { cs += clodW[t]; fs += fineW[t]; }
    for (int t = 0; t < kTints; ++t) { g.clod[t] = cs > 1e-6f ? clodW[t] / cs : 0.f; g.fine[t] = fs > 1e-6f ? fineW[t] / fs : 0.f; }
    return g;
}

// Picks one kind out of a set of shares with u in 0..1. Returns 'fallback' when the shares are all zero.
inline int pickTint(const float shares[kTints], float u, int fallback) {
    float s = 0.f; for (int t = 0; t < kTints; ++t) s += shares[t];
    if (s < 1e-6f) return fallback;
    float acc = 0.f; const float x = u * s;
    for (int t = 0; t < kTints; ++t) { acc += shares[t]; if (x < acc) return t; }
    return fallback;
}

// ================================================================================================ wiper sweep
// The rubber part of a blade as two end points in pixels. Everything is worked out from these two points, so it holds whatever the
// blade's length, the glass rake or the perspective do to its apparent shape.
struct Seg { float ix, iy, ox, oy; };

inline float segLen(const Seg& s) { return std::sqrt((s.ox - s.ix) * (s.ox - s.ix) + (s.oy - s.iy) * (s.oy - s.iy)); }

// 0 = it has not moved, 1 = it moved normally, 2 = it jumped (a different blade, a camera cut): do not sweep anything this frame.
inline int bladeMotion(const Seg& a, const Seg& b, float maxJumpPx) {
    const float d = std::max(std::max(std::fabs(a.ix - b.ix), std::fabs(a.iy - b.iy)), std::max(std::fabs(a.ox - b.ox), std::fabs(a.oy - b.oy)));
    if (d < 0.02f) return 0;
    return d > maxJumpPx ? 2 : 1;
}

inline float distPointSeg(float px, float py, float ax, float ay, float bx, float by) {
    const float dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    const float t = l2 > 1e-9f ? std::max(0.f, std::min(1.f, ((px - ax) * dx + (py - ay) * dy) / l2)) : 0.f;
    const float cx = ax + dx * t - px, cy = ay + dy * t - py; return std::sqrt(cx * cx + cy * cy);
}
inline bool inQuad(float px, float py, const float x[4], const float y[4]) {      // crossing number: works for any simple quadrilateral
    bool in = false;
    for (int i = 0, j = 3; i < 4; j = i++)
        if (((y[i] > py) != (y[j] > py)) && (px < (x[j] - x[i]) * (py - y[i]) / (y[j] - y[i]) + x[i])) in = !in;
    return in;
}

// Is the disc at (X, Y) with radius R (px) inside the area the blade swept going from 'a' to 'b'? 'margin' is the blade's half width.
// The area is the quadrilateral between the two positions, widened a little for the bulge of the real arc when the step was big.
// On a hit (mx, my) is how far the blade moved at that point (the direction a smear is dragged).
inline bool sweptBy(const Seg& a, const Seg& b, float X, float Y, float R, float margin, float& mx, float& my) {
    const float big = std::max(std::max(std::fabs(a.ix - b.ix), std::fabs(a.iy - b.iy)), std::max(std::fabs(a.ox - b.ox), std::fabs(a.oy - b.oy)));
    const float L = std::max(segLen(b), 1.f);
    const float reach = R + margin + big * big / (8.f * L);           // sagitta of the arc the blade really followed between the two positions
    const float qx[4] = {a.ix, a.ox, b.ox, b.ix}, qy[4] = {a.iy, a.oy, b.oy, b.iy};
    bool hit = inQuad(X, Y, qx, qy);
    for (int e = 0; e < 4 && !hit; ++e) { const int f = (e + 1) & 3; hit = distPointSeg(X, Y, qx[e], qy[e], qx[f], qy[f]) <= reach; }
    if (!hit) return false;
    const float dx = b.ox - b.ix, dy = b.oy - b.iy, l2 = dx * dx + dy * dy;
    const float f = l2 > 1e-9f ? std::max(0.f, std::min(1.f, ((X - b.ix) * dx + (Y - b.iy) * dy) / l2)) : 0.5f;
    mx = mix1(b.ix - a.ix, b.ox - a.ox, f); my = mix1(b.iy - a.iy, b.oy - a.oy, f);
    return true;
}

// ================================================================================================ wiper mesh extent
struct Extent { bool ok = false; int axis = 0; float lo = 0.f, hi = 0.f; };   // along 'axis' (0..2) in model units, signed

// Position of every vertex is three floats at the start of the vertex. Finds the long axis of the mesh and how far it reaches.
// A mesh that is not a thin long thing (a wiper arm or blade is at least 5 times longer than it is wide) or that has implausible
// numbers is rejected, so a wrong guess about the vertex layout never reaches the sweep.
inline Extent measureExtent(const uint8_t* data, int count, int strideBytes) {
    Extent e;
    if (!data || count < 3 || strideBytes < 12 || strideBytes > 256) return e;
    float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f}; int used = 0;
    for (int i = 0; i < count; ++i) {
        float p[3]; std::memcpy(p, data + (size_t)i * (size_t)strideBytes, 12);
        if (!(std::fabs(p[0]) < 20.f && std::fabs(p[1]) < 20.f && std::fabs(p[2]) < 20.f)) continue;     // also rejects NaN
        for (int k = 0; k < 3; ++k) { mn[k] = std::min(mn[k], p[k]); mx[k] = std::max(mx[k], p[k]); }
        ++used;
    }
    if (used < count * 9 / 10) return e;
    float range[3]; for (int k = 0; k < 3; ++k) range[k] = mx[k] - mn[k];
    int ax = 0; for (int k = 1; k < 3; ++k) if (range[k] > range[ax]) ax = k;
    float other = 0.f; for (int k = 0; k < 3; ++k) if (k != ax) other = std::max(other, range[k]);
    if (range[ax] < 0.12f || range[ax] > 1.5f || other * 5.f > range[ax]) return e;
    e.ok = true; e.axis = ax; e.lo = mn[ax]; e.hi = mx[ax];
    return e;
}

// The part of that mesh that wipes, from the end nearer the pivot to the far end (signed positions along the axis). A mesh that reaches
// back to its own origin carries the arm, and the rubber only starts a fraction of the way out ('inner'). A mesh that starts further
// out is the blade alone, and one centred on its origin is taken as the blade alone too, since the pivot end is then unknown.
inline void rubberSpan(const Extent& e, float inner, float& a, float& b) {
    const float len = e.hi - e.lo;
    const bool centred = e.lo < 0.f && e.hi > 0.f;
    const float nearEnd = std::fabs(e.lo) <= std::fabs(e.hi) ? e.lo : e.hi, farEnd = std::fabs(e.lo) <= std::fabs(e.hi) ? e.hi : e.lo;
    const bool hasArm = !centred && std::fabs(nearEnd) < 0.15f * len;
    if (hasArm) { a = nearEnd + inner * (farEnd - nearEnd); b = farEnd; }
    else { a = e.lo; b = e.hi; }
}

// ================================================================================================ smoothing
// Moves 'cur' towards 'tgt' by k (0..1). The first time (cur not ok yet) it simply takes the target.
inline void smoothGround(Ground& cur, const Ground& tgt, float k) {
    if (!tgt.ok) { cur.ok = false; return; }
    if (!cur.ok) { cur = tgt; return; }
    k = std::max(0.f, std::min(1.f, k));
    cur.loose = mix1(cur.loose, tgt.loose, k); cur.mudMul = mix1(cur.mudMul, tgt.mudMul, k); cur.spray = mix1(cur.spray, tgt.spray, k);
    cur.dust = mix1(cur.dust, tgt.dust, k); cur.snow = mix1(cur.snow, tgt.snow, k);
    for (int t = 0; t < kTints; ++t) { cur.clod[t] = mix1(cur.clod[t], tgt.clod[t], k); cur.fine[t] = mix1(cur.fine[t], tgt.fine[t], k); }
}

// ================================================================================================ mud between stages
// Mud and dirt stay on the glass from one stage to the next of the same rally, so that cleaning it costs time. Water does not (it runs off while
// the car is in the service park). The glass is cleaned fully when this is a new rally or a new car. The plugin is never told when a rally
// starts, so a new rally is recognised by what it looks like from here.
struct StageHistory {
    bool have = false;              // a stage has been started before in this session
    bool finished = false;          // and it was finished (not retired, restarted or left for the menu)
    int map = -1;                   // which stage that was
    std::string player;             // and who drove it
    float secondsSinceFinish = 0.f;
};
// nullptr: keep the mud. Otherwise the reason to clean the glass, for the log.
inline const char* cleanReason(bool carryOn, const StageHistory& h, int map, const std::string& player, float carryMinutes) {
    if (!carryOn) return "CarryOver is off";
    if (!h.have) return "first stage since the game started";
    if (!h.finished) return "the last stage was not finished (retired, restarted or left for the menu)";
    if (map == h.map) return "the same stage again";
    if (player != h.player) return "a different driver";
    if (carryMinutes > 0.f && h.secondsSinceFinish > carryMinutes * 60.f) return "a long break since the last stage, taken as a new rally";
    return nullptr;
}
// Is this a different car from the one the glass was last dirtied in? Empty keys say nothing.
inline bool carChanged(const std::string& last, const std::string& now) { return !last.empty() && !now.empty() && last != now; }

}  // namespace wsl
}  // namespace cam
}  // namespace ffx
