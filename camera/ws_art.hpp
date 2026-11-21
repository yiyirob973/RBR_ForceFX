// Artwork of the water drops in WindscreenFX, without Windows or Direct3D so it can be rendered and looked at offline (tests/ws_art_test.cpp).
//
// What a real drop on a car windscreen looks like (and what the first version got wrong):
//  * Standing drops are nearly round and small, a bit flatter at the bottom. Big flat grey pools and oval "pebbles" do not occur on a moving car.
//  * The inside is the world seen upside down and magnified. It is NOT darker or greyer than the world around it, if anything slightly brighter.
//  * The edge is a thin, soft dark line on the upper side (where the glass is seen through the thick part) and a thin bright line on the lower
//    side (sky refracted in). There is no heavy outline.
//  * A drop that runs is a little longer than wide and rounded at both ends. It leaves a thin clear track; it does not carry a tail with it.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ffx {
namespace cam {
namespace wsl {
namespace art {

struct Px { float c[3] = {0, 0, 0}; float a = 0; };                    // premultiplied accumulation, same as the plugin's WsPx
inline float cl01(float v) { return std::max(0.f, std::min(1.f, v)); }
inline float smooth(float a, float b, float x) { const float t = cl01((x - a) / (b - a)); return t * t * (3.f - 2.f * t); }
inline void over(Px& p, float r, float g, float b, float a) {
    a = cl01(a); const float col[3] = {r, g, b};
    for (int i = 0; i < 3; ++i) p.c[i] = col[i] * a + p.c[i] * (1.f - a);
    p.a = a + p.a * (1.f - a);
}
inline void scale(Px& p, float s) { p.a *= s; for (int i = 0; i < 3; ++i) p.c[i] *= s; }

// ---- height fields on a unit cell (u right, v down). Cell 0: a standing drop, cell 1: a drop that is running (head at the bottom).
inline float heightStanding(float u, float v) {                         // a round bead, a little flatter at the bottom edge
    const float dx = u - 0.5f, dy = v - 0.5f, k = dy > 0.f ? 1.08f : 1.f;
    const float d = std::sqrt(dx * dx + dy * dy * k * k) / 0.44f;
    return d < 1.f ? std::sqrt(1.f - d * d) : 0.f;
}
inline float heightRunning(float u, float v) {                          // egg: round at both ends, fuller at the bottom, about 1.5 times as long as wide
    const float dx = u - 0.5f, dy = (v - 0.5f) / 0.46f;                 // dy in -1 .. 1 along the drop
    if (std::fabs(dy) >= 1.f) return 0.f;
    const float full = 0.30f + 0.12f * smooth(-1.f, 1.f, dy);           // half width: narrower at the top, fuller at the bottom
    const float w = full * std::sqrt(1.f - dy * dy) * 1.0f + 0.0f;       // elliptical envelope
    const float s = std::fabs(dx) / std::max(w, 1e-4f);
    if (s >= 1.f) return 0.f;
    return std::sqrt(1.f - s * s) * std::sqrt(1.f - dy * dy * 0.8f);
}
inline float height(int cell, float u, float v) { return cell == 0 ? heightStanding(u, v) : heightRunning(u, v); }

// ---- refraction mask (alpha of the lens)
inline float mask(int cell, float u, float v) { return smooth(0.f, 0.14f, height(cell, u, v)) * 0.97f; }

// ---- shading laid over the refracted view: no body colour, only a soft edge and a small highlight
inline void shade(int cell, float u, float v, Px& p) {
    const float h = height(cell, u, v); if (h <= 0.f) return;
    const float e = 1.f / 64.f, S = 0.30f / (2.f * e);
    const float hx = height(cell, u + e, v) - height(cell, u - e, v), hy = height(cell, u, v + e) - height(cell, u, v - e);
    float nx = -hx * S, ny = -hy * S, nz = 1.f; const float nl = std::sqrt(nx * nx + ny * ny + nz * nz); nx /= nl; ny /= nl; nz /= nl;
    const float rim = 1.f - nz;                                          // 0 in the middle, towards 1 at the edge
    const float up = std::max(0.f, -ny), down = std::max(0.f, ny);        // which side of the drop this part of the edge faces (v grows downwards)
    // thin soft dark line on the upper edge, only there
    over(p, 0.04f, 0.05f, 0.06f, std::pow(rim, 2.2f) * up * 0.50f);
    // bright refracted sky on the lower edge
    over(p, 0.95f, 0.97f, 1.00f, std::pow(rim, 1.6f) * down * 0.40f);
    // a faint lift in the middle so the drop reads as clear water, not as a hole
    over(p, 0.90f, 0.95f, 1.00f, (1.f - rim) * 0.035f);
    // small specular dot, upper left
    const float sx = u - 0.40f, sy = v - 0.36f;
    over(p, 1.f, 1.f, 1.f, std::exp(-(sx * sx + sy * sy) / 0.0045f) * 0.75f * smooth(0.f, 0.25f, h));
    scale(p, smooth(0.f, 0.10f, h));
}

// ---- small drops that are drawn as plain sprites (too small to refract)
inline void sprite(float u, float v, Px& p) {                           // u, v in -1 .. 1
    const float dd = std::sqrt(u * u + v * v);
    if (dd >= 1.f) return;
    over(p, 0.85f, 0.90f, 0.96f, 0.10f);
    over(p, 0.05f, 0.07f, 0.09f, smooth(0.60f, 0.98f, dd) * std::max(0.f, -v + 0.2f) * 0.32f);
    over(p, 0.95f, 0.97f, 1.f, smooth(0.55f, 0.95f, dd) * std::max(0.f, v) * 0.30f);
    const float hx = u + 0.34f, hy = v + 0.38f;
    over(p, 1.f, 1.f, 1.f, smooth(0.26f, 0.04f, std::sqrt(hx * hx + hy * hy)) * 0.85f);
    scale(p, smooth(1.0f, 0.86f, dd));
}
inline void micro(float u, float v, Px& p) {                            // tiny speck: faint body, one bright point
    const float d = std::sqrt(u * u + v * v); if (d >= 1.f) return;
    over(p, 0.20f, 0.24f, 0.28f, smooth(1.0f, 0.5f, d) * 0.22f);
    const float sx = u + 0.28f, sy = v + 0.32f;
    over(p, 1.f, 1.f, 1.f, std::exp(-(sx * sx + sy * sy) / 0.07f) * 0.85f);
}

}  // namespace art
}  // namespace wsl
}  // namespace cam
}  // namespace ffx
