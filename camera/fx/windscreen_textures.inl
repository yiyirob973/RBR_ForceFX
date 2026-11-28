// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ================================================================================================ Windscreen effects
// Rain, snow and mud on the glass of the cockpit view, drawn through the D3D9 device (pre-transformed vertices, procedural textures).
//  - Drops are refractive: a copy of the frame is sampled upside down and magnified inside each drop, like a real bead of water. Moving drops
//    become teardrops, leave a streak and swallow the drops they hit. Rain falls at a rate set by the game's own sky type and your speed.
//  - Snow flakes stick, melt slowly into water drops, and turn thrown mud into grey slush.
//  - Mud arrives as splatters (rough, wet, fast) and landings; wipers smear it and sweep drops away.
//  - Everything lives on the glass plane and follows the head transform, so it stays on the car.
//  - HideGameFx suppresses the game's own wet-windscreen drawing so only these effects remain.
struct WsVert  { float x, y, z, rhw; DWORD color; float u, v; };
struct WsVert2 { float x, y, z, rhw; DWORD color; float u0, v0, u1, v1; };
constexpr DWORD kWsFvf1 = D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX1;
constexpr DWORD kWsFvf2 = D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX2;
struct WsDrop  { float x, y, r, vx, vy, age, thr, tx, ty, tr, melt, melt0, rot, asp; int kind, var; bool dead, sliding; };   // x,y: 0..1 of the viewport; r: fraction of viewport height; kind 0 water, 1 snow flake, 2 micro droplet; var / asp: pool shape
struct WsMud   { float x, y, r, rot, age, fade, stretch, shade, ox, oy, dur; int variant, tint; bool dying; };   // age < 0: waiting; ox, oy, dur > 0: thrown outwards from (ox, oy) over dur seconds; tint 0 mud, 1 slush
// A particle on its way to the glass (outside it). kind 0 rain bead, 1 mud clod, 2 snow flake, 3 dirty road spray. Positions are viewport fractions; t runs 0..1
// over dur seconds (negative = not started yet). mode 0: perspective approach from a far point (rain, snow, mud thrown by a car ahead); mode 1: thrown up from
// below (mud off your own wheels), fast at first and slowing as it arrives.
struct WsFly   { float x0, y0, x1, y1, t, dur, r, arc, shade; int kind, mode, variant, tint; };
struct WsTrail { float x0, y0, x1, y1, w0, w1, age; };                                                   // positions as x,y fractions, half widths as fractions of height

using EndSceneFn = HRESULT(__stdcall*)(void* dev);
using ResetFn    = HRESULT(__stdcall*)(void* dev, void* pp);
using DrawPrimFn      = HRESULT(__stdcall*)(void* dev, DWORD type, UINT start, UINT prims);
using DrawIdxPrimFn   = HRESULT(__stdcall*)(void* dev, DWORD type, INT baseVtx, UINT minVtx, UINT numVtx, UINT startIdx, UINT prims);
using DrawPrimUPFn    = HRESULT(__stdcall*)(void* dev, DWORD type, UINT prims, const void* data, UINT stride);
using DrawIdxPrimUPFn = HRESULT(__stdcall*)(void* dev, DWORD type, UINT minVtx, UINT numVtx, UINT prims, const void* idx, DWORD idxFmt, const void* vtx, UINT stride);
EndSceneFn g_origEndScene = nullptr;
ResetFn    g_origReset = nullptr;
DrawPrimFn g_origDrawPrim = nullptr; DrawIdxPrimFn g_origDrawIdxPrim = nullptr; DrawPrimUPFn g_origDrawPrimUP = nullptr; DrawIdxPrimUPFn g_origDrawIdxPrimUP = nullptr;
volatile bool g_wsSuppress = false, g_wsDrawing = false;          // g_wsSuppress: dropping the game's own windscreen draws; g_wsDrawing: our own draws always pass
std::atomic<long> g_wsHidden{0}, g_wsWiped{0};      // draw calls dropped / things cleared by the wipers, counted per second in the log

IDirect3DDevice9*     g_wsDev = nullptr;
IDirect3DTexture9    *g_wsPoolMaskTex = nullptr, *g_wsPoolShadeTex = nullptr, *g_wsMicroTex = nullptr, *g_wsSpriteTex = nullptr, *g_wsMaskTex = nullptr, *g_wsShadeTex = nullptr, *g_wsMudTex = nullptr, *g_wsTrailTex = nullptr,
                     *g_wsSplashTex = nullptr, *g_wsFlakeTex = nullptr, *g_wsWhiteTex = nullptr, *g_wsStreakTex = nullptr;
IDirect3DTexture9*    g_wsSceneTex = nullptr; IDirect3DSurface9* g_wsSceneSurf = nullptr; UINT g_wsSceneW = 0, g_wsSceneH = 0; int g_wsSceneFmt = 0;
IDirect3DTexture9*    g_wsBlurTex[2] = {nullptr, nullptr}; IDirect3DSurface9* g_wsBlurSurf[2] = {nullptr, nullptr};   // 1/2 and 1/4 of the scene copy: the veil blur
bool                  g_wsRefrFailed = false, g_wsRefrUsed = false, g_wsVeilFailed = false, g_wsVeilUsed = false;
IDirect3DStateBlock9* g_wsSB = nullptr;
std::vector<WsDrop>   g_wsDrops;
std::vector<WsMud>    g_wsMud;
std::vector<WsTrail>  g_wsTrails;
std::vector<WsFly>    g_wsFly;
std::vector<WsVert>   g_wsVSprite, g_wsVShade, g_wsVShadePool, g_wsVMud, g_wsVTrail, g_wsVFlake, g_wsVSplash, g_wsVDbg, g_wsVMicro, g_wsVVeil, g_wsVDirt, g_wsVGlow, g_wsVFlyMud, g_wsVFlyFlake, g_wsVStreak, g_wsVWBlur;
std::vector<WsVert2>  g_wsVRefr, g_wsVRefrPool, g_wsVFrost;
volatile bool g_wsArmed = false, g_wsHooked = false;
bool          g_wsReset = true, g_wsResetWater = false, g_wsTexFailed = false;
wsl::StageHistory g_wsHist; std::string g_wsLastCar; DWORD g_wsFinishTick = 0, g_wsCarCheckUntil = 0;   // for carrying mud from one stage to the next
DWORD         g_wsStamp = 0;
long          g_wsFrameNo = 0, g_wsLastDrawn = -1;
std::atomic<long> g_wsDrawn{0}, g_wsSkipped{0};
UINT          g_wsVpW = 0, g_wsVpH = 0;
uint32_t      g_wsRng = 0x9E3779B9u;
float         g_wsSprayAcc = 0.f, g_wsRainAcc = 0.f, g_wsSnowAcc = 0.f, g_wsMudAcc = 0.f, g_wsMicroAcc = 0.f, g_wsAspect = 16.f / 9.f, g_wsSnowEff = 0.f;
std::atomic<long>  g_wsGameCalls{0};      // calls of the game's own wet-windscreen function (0x452020), counted per second in the log
std::atomic<DWORD> g_wsGameTick{0};       // tick of the last such call
float         g_wsRough = 0.f, g_wsLoose = 0.f, g_wsSpdN = 0.f, g_wsDbgRain = 0.f, g_wsDbgMud = 0.f;   // telemetry-derived factors (logged for calibration)
float         g_wsWet = 1.f;              // 0..1 rain rate scale; follows the game's wet-windscreen calls when [Windscreen] WetFromGame=1
// the car's real wiper blades, found among the 3D draw calls by the hunter (see wsHuntReport) and turned into a pose every frame
struct WsModelBlade { bool ok; float px, py, len, ang; wsl::Seg seg; };      // seg: the two ends of the rubber on the glass, in pixels            // pivot (fraction of width / height), length (fraction of height), angle (rad, 0 = right, CCW with y up)
WsModelBlade g_wsModelBlade[2] = {{false, 0, 0, 0, 0, {0, 0, 0, 0}}, {false, 0, 0, 0, 0, {0, 0, 0, 0}}};
wsl::Seg g_wsSegPrev[2] = {{0, 0, 0, 0}, {0, 0, 0, 0}}; bool g_wsSegHave[2] = {false, false};     // where each rubber was in the last frame, to know what it swept
struct WsMeasure { bool tried = false; wsl::Extent ext; };                                        // the real length and direction of a locked blade mesh, read from its vertices once
WsMeasure g_wsMeas[2];
int g_wsDrawLock = -1;                                                // which locked blade the draw call being hooked is (-1 = none)
bool g_wsHuntLocked = false;
std::atomic<long> g_wsHiddenWipers{0};
void wsHuntPose(float W, float H);                                   // defined with the hunter
void wsHuntReset();
void wsHuntTick();
// wipers (emulated sweep, see wsWiperUpdate)
float g_wsBladeAng[2] = {0, 0}, g_wsBladePrev[2] = {0, 0}, g_wsWiperW = 0.f; bool g_wsBladeInit = false; DWORD g_wsWipeT0 = 0;

float wsRnd() { g_wsRng ^= g_wsRng << 13; g_wsRng ^= g_wsRng >> 17; g_wsRng ^= g_wsRng << 5; return (float)(g_wsRng & 0xFFFFFF) / 16777216.0f; }
float wsRange(float a, float b) { return a + (b - a) * wsRnd(); }
float wsSmooth(float a, float b, float x) { const float t = clamp01((x - a) / (b - a)); return t * t * (3.f - 2.f * t); }   // works for a > b too
float wsHash(int x, int y, int seed) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + (uint32_t)seed * 2147483647u + 1013904223u;
    h = (h ^ (h >> 13)) * 1274126177u; h ^= h >> 16; return (float)(h & 0xFFFFFF) / 16777216.f;
}
float wsVNoise(float x, float y, int seed) {
    const int xi = (int)std::floor(x), yi = (int)std::floor(y); float fx = x - (float)xi, fy = y - (float)yi;
    fx = fx * fx * (3.f - 2.f * fx); fy = fy * fy * (3.f - 2.f * fy);
    const float a = wsHash(xi, yi, seed), b = wsHash(xi + 1, yi, seed), c = wsHash(xi, yi + 1, seed), d = wsHash(xi + 1, yi + 1, seed);
    return (a + (b - a) * fx) * (1.f - fy) + (c + (d - c) * fx) * fy;
}
float wsFbm(float x, float y, int seed, int oct) {
    float s = 0.f, a = 0.5f, f = 1.f, n = 0.f;
    for (int i = 0; i < oct; ++i) { s += a * wsVNoise(x * f, y * f, seed + i * 17); n += a; a *= 0.5f; f *= 2.03f; }
    return s / n;
}

// Glass coverage grids (viewport fractions): film = water haze from rain / snow impacts, dirt = mud haze. They blur the view behind the glass in proportion,
// fill up while it rains, and the wipers clear them (dirt only partly, so it smears).
constexpr int kGX = 80, kGY = 45;
float g_wsFilm[kGX * kGY], g_wsDirt[kGX * kGY];
float g_wsBow[2] = {0.f, 0.f}, g_wsGhost[2][2] = {{0.f, 0.f}, {0.f, 0.f}};      // wiper: water ridge pushed ahead of each blade, motion-blur ghost angles
void wsGridAdd(float* g, float x, float y, float rc, float amount) {
    const float cx = x * (float)kGX - 0.5f, cy = y * (float)kGY - 0.5f;
    const int x0 = std::max(0, (int)std::floor(cx - rc)), x1 = std::min(kGX - 1, (int)std::ceil(cx + rc)), y0 = std::max(0, (int)std::floor(cy - rc)), y1 = std::min(kGY - 1, (int)std::ceil(cy + rc));
    for (int j = y0; j <= y1; ++j) for (int i = x0; i <= x1; ++i) {
        const float dx = (float)i - cx, dy = (float)j - cy, dd = std::sqrt(dx * dx + dy * dy);
        if (dd >= rc) continue;
        const float w = 1.f - dd / rc; float& c = g[j * kGX + i]; c = std::min(1.f, c + amount * w * w * (3.f - 2.f * w));
    }
}
float wsGridMax(const float* g) { float m = 0.f; for (int i = 0; i < kGX * kGY; ++i) m = std::max(m, g[i]); return m; }

// The game's resolved stage conditions. NGPCarMenu documents RBRMapSettingsEx at the fixed address 0x8938F8: the Good / Random / Bad weather preset
// has already been turned into a sky type (0 crisp, 1 hazy, 2 no rain, 3 light rain, 4 heavy rain, 5 no snow, 6 light snow, 7 heavy snow, 8 light fog,
// 9 heavy fog), a surface wetness (0 dry, 1 damp, 2 wet), a cloud type and a time of day. That is what the game itself keys rain and wipers on.
struct WsGameWeather { bool ok = false; int track = -1, cloud = -1, wet = -1, tod = -1, sky = -1; };
const char* wsSkyName(int s) {
    static const char* n[10] = {"crisp", "hazy", "no rain", "light rain", "heavy rain", "no snow", "light snow", "heavy snow", "light fog", "heavy fog"};
    return (s >= 0 && s < 10) ? n[s] : "?";
}
WsGameWeather wsReadGameWeather() {
    WsGameWeather g;
    const uintptr_t a = absAddr(0x8938F8);
    if (!readableRange(a, 0x40)) return g;
    const int32_t* w = (const int32_t*)a;
    g.track = w[2]; g.cloud = w[4]; g.wet = w[5]; g.tod = w[14]; g.sky = w[15];     // offsets 0x08, 0x10, 0x14, 0x38, 0x3C
    g.ok = (g.sky >= 0 && g.sky <= 9 && g.wet >= 0 && g.wet <= 2);
    return g;
}
WsGameWeather g_wsGW;                                              // refreshed a few times a second from wsStep

// ---- what the tyres are on (from the surface scan), and what that means for the glass ----
ffx::MaterialTable g_wsMat;                                        // material id -> class / wetness / looseness
wsl::Ground g_wsGround;                                            // smoothed over about half a second so one tyre touching a verge does not flip the glass
wsl::Rgb    g_wsDirtCol{0.52f, 0.40f, 0.28f};                      // colour of the haze on the glass: drifts towards whatever is landing on it
int         g_wsGroundIds[4] = {-1, -1, -1, -1}; DWORD g_wsGroundTick = 0, g_wsGroundGood = 0; bool g_wsGroundSeen = false;
wsl::Rgb wsMudRgb() { return {g_cfg.wsMudR, g_cfg.wsMudG, g_cfg.wsMudB}; }

// ---- procedural textures ----
struct WsPx { float c[3] = {0, 0, 0}; float a = 0; };            // premultiplied accumulation
void wsOver(WsPx& p, float r, float g, float b, float a) {
    a = clamp01(a); const float col[3] = {r, g, b};
    for (int i = 0; i < 3; ++i) p.c[i] = col[i] * a + p.c[i] * (1.f - a);
    p.a = a + p.a * (1.f - a);
}
void wsScale(WsPx& p, float s) { p.a *= s; for (int i = 0; i < 3; ++i) p.c[i] *= s; }
DWORD wsPack(const WsPx& p) {
    if (p.a <= 1e-4f) return 0;
    auto ch = [&](float c) { return (DWORD)(clamp01(c / p.a) * 255.f + 0.5f); };
    return ((DWORD)(clamp01(p.a) * 255.f + 0.5f) << 24) | (ch(p.c[0]) << 16) | (ch(p.c[1]) << 8) | ch(p.c[2]);
}
template <class F> bool wsMakeTex(IDirect3DDevice9* d, IDirect3DTexture9** t, int w, int h, F fill) {
    if (FAILED(d->CreateTexture(w, h, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, t, nullptr)) || !*t) return false;
    D3DLOCKED_RECT lr;
    if (FAILED((*t)->LockRect(0, &lr, nullptr, 0))) return false;
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) { WsPx p; fill(x, y, p); *(DWORD*)((BYTE*)lr.pBits + y * lr.Pitch + x * 4) = wsPack(p); }
    (*t)->UnlockRect(0);
    return true;
}

// Water bead artwork lives in ws_art.hpp (pure code, tested in tests/ws_art_test.cpp). Cell 0 is a standing drop, cell 1 a running one.
void wsFromArt(const wsl::art::Px& q, WsPx& p) { for (int i = 0; i < 3; ++i) p.c[i] = q.c[i]; p.a = q.a; }
void wsBeadShade(int cell, float u, float v, WsPx& p) { wsl::art::Px q; wsl::art::shade(cell, u, v, q); wsFromArt(q, p); }
// Mud / slush atlas: 4 x 2 cells of 128 px (luminance in rgb, tinted by the vertex colour). Cells 0-3 are thrown-mud splats like the ones on a rally car's
// windscreen: ragged spiky outline, translucent lacy interior, darker rim, wet highlights, elongated satellite drops flung outwards. 4 smear (long, streaky),
// 5 and 7 spray clumps, 6 fine speckle of dirty road water.
constexpr int kMudSmear = 4, kMudSpray = 5, kMudSpeckle = 6, kMudSpray2 = 7;
inline bool wsIsSprayV(int v) { return v >= 5; }
float wsAngDiff(float a, float b) { return std::fmod(a - b + 9.42478f, 6.28318f) - 3.14159f; }
void wsMudPx(int variant, float u, float v, WsPx& p) {
    const int seed = 11 + variant * 37;
    const float d = std::sqrt(u * u + v * v), ang = std::atan2(v, u);
    auto dot = [&](float cx, float cy, float rad, float stretch, float a, float lum) {      // ellipse elongated along the direction from the cell centre
        const float dx = u - cx, dy = v - cy, cl = std::sqrt(cx * cx + cy * cy) + 1e-4f, ax = cx / cl, ay = cy / cl;
        const float al = dx * ax + dy * ay, pe = -dx * ay + dy * ax, dd = std::sqrt((al / stretch) * (al / stretch) + pe * pe);
        if (dd < rad) wsOver(p, lum, lum, lum, wsSmooth(rad, rad * 0.45f, dd) * a);
    };
    if (variant == kMudSpray || variant == kMudSpray2) {                // a loose clump of clods: centre-weighted, uneven sizes
        for (int k = 0; k < 10; ++k) {
            const float a = wsHash(k, 1, seed) * 6.2832f, rr = std::pow(wsHash(k, 2, seed), 1.4f) * 0.62f, rad = 0.05f + 0.20f * wsHash(k, 3, seed) * wsHash(k, 3, seed);
            const float cx = std::cos(a) * rr, cy = std::sin(a) * rr;
            const float dx = u - cx, dy = v - cy, dd = std::sqrt(dx * dx + dy * dy);
            if (dd < rad) { const float lum = 0.34f + 0.30f * wsHash(k, 4, seed) + 0.22f * std::exp(-((dx + 0.3f * rad) * (dx + 0.3f * rad) + (dy + 0.35f * rad) * (dy + 0.35f * rad)) / (rad * rad * 0.18f)); wsOver(p, lum, lum, lum, wsSmooth(rad, rad * 0.5f, dd) * 0.95f); }
        }
        return;
    }
    if (variant == kMudSpeckle) {                                      // fine dirty spray: many tiny droplets, some elongated
        for (int k = 0; k < 34; ++k) {
            const float a = wsHash(k, 1, seed) * 6.2832f, rr = std::pow(wsHash(k, 2, seed), 0.8f) * 0.85f, rad = 0.018f + 0.050f * std::pow(wsHash(k, 3, seed), 2.f);
            dot(std::cos(a) * rr, std::sin(a) * rr, rad, 1.f + 1.2f * wsHash(k, 8, seed), 0.55f + 0.4f * wsHash(k, 9, seed), 0.30f + 0.35f * wsHash(k, 4, seed));
        }
        return;
    }
    if (variant == kMudSmear) {                                        // smear: long along v, streaky, translucent
        const float uu = u * 1.25f, vv = v * 0.55f, dd = std::sqrt(uu * uu + vv * vv);
        const float edge = 0.58f + 0.2f * wsFbm(ang * 1.2f + 3.f, 1.f, seed, 3);
        const float n = wsFbm(u * 9.f + 4.f, v * 1.6f, seed, 4);        // striations run along v
        const float dens = (edge - dd) / 0.30f + (n - 0.5f) * 0.9f;
        const float a = wsSmooth(0.0f, 0.65f, dens) * (0.50f + 0.25f * n);
        const float lum = 0.55f + 0.30f * (n - 0.5f);
        wsOver(p, lum, lum, lum, a);
        return;
    }
    // ---- splat
    const int ns = 3 + variant % 3;                                    // major lobes: 3-5 short, wide ones
    float R = 0.36f + 0.09f * (wsFbm(std::cos(ang) * 2.2f + (float)seed, std::sin(ang) * 2.2f + (float)seed * 1.7f, seed, 3) - 0.5f) * 2.f;
    for (int k = 0; k < ns; ++k) {
        const float ak = wsHash(k, 1, seed) * 6.2832f - 3.14159f, w = 0.14f + 0.14f * wsHash(k, 2, seed), len = 0.10f + 0.20f * std::pow(wsHash(k, 3, seed), 1.3f);
        const float x = wsAngDiff(ang, ak) / w; R += len * std::exp(-x * x);
    }
    for (int k = 0; k < 9; ++k) {                                      // minor jagged points all around the edge
        const float ak = wsHash(k, 11, seed) * 6.2832f - 3.14159f, w = 0.06f + 0.07f * wsHash(k, 12, seed), len = 0.035f + 0.075f * wsHash(k, 13, seed);
        const float x = wsAngDiff(ang, ak) / w; R += len * std::exp(-x * x);
    }
    R = std::min(R, 0.84f);
    const float jit = (wsFbm(u * 9.f + (float)seed, v * 9.f, seed + 3, 3) - 0.5f) * 0.09f;
    const float sd = d - R + jit;                                      // negative inside
    const float inside = wsSmooth(0.025f, -0.035f, sd);
    const float grain = wsFbm(u * 6.5f + (float)seed * 0.3f, v * 6.5f + (float)seed * 0.9f, seed + 5, 4), speck = wsVNoise(u * 34.f, v * 34.f, seed + 9);
    if (inside > 0.f) {
        const float core = wsSmooth(R, 0.f, d);
        float a = inside * (0.42f + 0.32f * core + 0.34f * (grain - 0.5f));
        a *= 0.72f + 0.28f * wsSmooth(0.22f, 0.55f, grain);            // lacy, thinner patches
        float lum = 0.72f + 0.60f * (grain - 0.5f) - 0.34f * wsSmooth(-0.16f, 0.0f, sd) + 0.14f * (speck - 0.5f);
        const float hx = u + 0.15f, hy = v + 0.19f; lum += 0.32f * std::exp(-(hx * hx + hy * hy) / 0.012f);        // wet sheen
        const float h2 = (u - 0.10f) * 1.0f, h3 = v + 0.30f; lum += 0.16f * std::exp(-(h2 * h2 * 6.f + h3 * h3 * 90.f));
        wsOver(p, clamp01(lum), clamp01(lum), clamp01(lum), clamp01(a));
        const float kr = (sd + 0.02f) / 0.028f; wsOver(p, 0.16f, 0.16f, 0.16f, std::exp(-kr * kr) * 0.55f * inside);   // dark rim where the mud is thickest
    }
    for (int k = 0; k < 10; ++k) {                                     // satellites flung outwards
        const float ak = wsHash(k, 5, seed) * 6.2832f, rr = std::min(0.90f, R * 0.85f + 0.14f + 0.34f * wsHash(k, 6, seed)), rad = 0.022f + 0.060f * std::pow(wsHash(k, 7, seed), 1.8f);
        dot(std::cos(ak) * rr, std::sin(ak) * rr, rad, 1.f + 1.6f * wsHash(k, 8, seed), 0.88f, 0.34f + 0.22f * wsHash(k, 9, seed));
    }
}


// Flat irregular pools (the big drops in real photos of a rainy windscreen): low profile, so a darker flat interior, a thin dark contact line on the
// upper side and a thin bright arc just inside the lower side, no big highlight. Four shapes in a 2x2 atlas, cell coordinates in [-1,1].
float wsPoolSD(int variant, float u, float v, float& nx, float& ny) {      // signed distance (negative inside) and outward direction
    const float asp = 0.80f + 0.12f * (float)variant;
    const float uu = u / asp, vv = v * asp, d = std::sqrt(uu * uu + vv * vv) + 1e-5f, ang = std::atan2(vv, uu);
    const int seed = 101 + variant * 29;
    const float n = wsFbm(std::cos(ang) * 1.4f + (float)seed, std::sin(ang) * 1.4f + (float)seed * 1.3f, seed, 3);
    float R = 0.62f * (1.f + 0.11f * (n - 0.5f) * 2.f) + 0.05f * std::sin(2.f * ang + (float)variant) + 0.014f * std::sin(3.f * ang + 2.f * (float)variant);
    R = std::max(0.38f, std::min(0.86f, R));
    nx = uu / d; ny = vv / d;
    return d - R;
}
void wsPoolPx(int variant, float u, float v, bool shade, WsPx& p) {
    float nx, ny; const float sd = wsPoolSD(variant, u, v, nx, ny);
    if (!shade) { wsOver(p, 1.f, 1.f, 1.f, wsSmooth(0.04f, -0.04f, sd) * 0.97f); return; }
    if (sd > 0.20f) return;
    const float inside = wsSmooth(0.03f, -0.03f, sd);
    const float mott = wsFbm(u * 3.5f + (float)variant * 7.f, v * 3.5f, 61 + variant, 3);
    const float lowSide = std::max(0.f, nx * -0.30f + ny * 0.95f), upSide = std::max(0.f, ny * -0.95f + nx * 0.20f);
    wsOver(p, 0.08f, 0.11f, 0.10f, inside * (0.27f + 0.14f * mott));                              // flat, darker interior
    wsOver(p, 0.04f, 0.06f, 0.05f, inside * wsSmooth(-0.30f, 0.0f, sd) * upSide * 0.20f);        // the thick part of the film near the upper edge
    wsOver(p, 0.85f, 0.90f, 0.90f, inside * 0.10f * wsSmooth(0.55f, 0.85f, mott));               // faint lighter reflection patches
    const float kd = (sd - 0.005f) / 0.050f, kb = (sd + 0.050f) / 0.055f;
    wsOver(p, 0.05f, 0.07f, 0.06f, std::exp(-kd * kd) * (0.25f + 0.60f * upSide));             // dark contact line
    wsOver(p, 0.93f, 0.96f, 0.95f, std::exp(-kb * kb) * (0.08f + 0.80f * lowSide) * 0.62f);    // bright arc inside the lower edge
}
void wsMicroPx(float u, float v, WsPx& p) { wsl::art::Px q; wsl::art::micro(u, v, q); wsFromArt(q, p); }   // a tiny drop: faint body, one bright point

bool wsMakeTextures(IDirect3DDevice9* d) {
    const int N = 64;
    const bool a = wsMakeTex(d, &g_wsSpriteTex, N, N, [&](int x, int y, WsPx& p) {          // small drops: plain alpha sprite
        wsl::art::Px q; wsl::art::sprite((x + 0.5f) / N * 2.f - 1.f, (y + 0.5f) / N * 2.f - 1.f, q); wsFromArt(q, p);
    });
    const bool b = wsMakeTex(d, &g_wsMaskTex, 2 * N, N, [&](int x, int y, WsPx& p) {         // refraction mask: cell 0 round, cell 1 comet
        const int cell = x / N; const float u = ((x % N) + 0.5f) / N, v = (y + 0.5f) / N;
        wsOver(p, 1.f, 1.f, 1.f, wsl::art::mask(cell, u, v));
    });
    const bool c = wsMakeTex(d, &g_wsShadeTex, 2 * N, N, [&](int x, int y, WsPx& p) {        // rim / specular overlay for the same two cells
        const int cell = x / N; wsBeadShade(cell, ((x % N) + 0.5f) / N, (y + 0.5f) / N, p);
    });
    const bool e = wsMakeTex(d, &g_wsMudTex, 512, 256, [&](int x, int y, WsPx& p) {          // 4x2 atlas of mud cells
        const int cx = x / 128, cy = y / 128;
        wsMudPx(cy * 4 + cx, ((x % 128) + 0.5f) / 128.f * 2.f - 1.f, ((y % 128) + 0.5f) / 128.f * 2.f - 1.f, p);
    });
    const bool m = wsMakeTex(d, &g_wsStreakTex, 16, 16, [&](int x, int, WsPx& p) {           // soft streak: u across, uniform along v (rain / dirt in flight)
        const float t = std::fabs(((x + 0.5f) / 16.f) * 2.f - 1.f); wsOver(p, 1.f, 1.f, 1.f, std::exp(-t * t * 5.f) * wsSmooth(1.0f, 0.75f, t));
    });
    const bool f = wsMakeTex(d, &g_wsTrailTex, 16, 16, [&](int x, int, WsPx& p) {            // wet streak: u across, uniform along v
        const float u = (x + 0.5f) / 16.f, t = std::fabs(2.f * u - 1.f), k1 = (t - 0.80f) / 0.12f, k2 = (u - 0.30f) / 0.09f;
        wsOver(p, 0.80f, 0.88f, 0.96f, 0.05f);
        wsOver(p, 0.08f, 0.11f, 0.15f, 0.16f * std::exp(-k1 * k1));
        wsOver(p, 1.f, 1.f, 1.f, 0.20f * std::exp(-k2 * k2));
        wsScale(p, wsSmooth(1.0f, 0.82f, t));
    });
    const bool g = wsMakeTex(d, &g_wsSplashTex, N, N, [&](int x, int y, WsPx& p) {           // impact flash: thin ring with spikes
        const float u = (x + 0.5f) / N * 2.f - 1.f, v = (y + 0.5f) / N * 2.f - 1.f, dd = std::sqrt(u * u + v * v), ang = std::atan2(v, u);
        if (dd >= 1.f) return;
        const float k = (dd - 0.62f) / 0.07f, ring = std::exp(-k * k) * 0.65f;
        const float spike = std::pow(std::max(0.f, std::cos(8.f * ang + 0.4f)), 6.f) * wsSmooth(0.15f, 0.4f, dd) * wsSmooth(0.98f, 0.6f, dd) * 0.55f;
        wsOver(p, 0.92f, 0.96f, 1.f, ring + spike);
        wsScale(p, wsSmooth(1.0f, 0.9f, dd));
    });
    const bool h = wsMakeTex(d, &g_wsFlakeTex, N, N, [&](int x, int y, WsPx& p) {            // snow flake: soft core with six feathery arms
        const float u = (x + 0.5f) / N * 2.f - 1.f, v = (y + 0.5f) / N * 2.f - 1.f, dd = std::sqrt(u * u + v * v), ang = std::atan2(v, u);
        if (dd >= 1.f) return;
        const float edge = 0.46f + 0.30f * wsFbm(std::cos(ang) * 1.5f + 3.f, std::sin(ang) * 1.5f + 8.f, 43, 3);
        const float n = wsFbm(u * 3.0f + 5.f, v * 3.0f + 9.f, 41, 4);
        const float dens = (edge - dd) / 0.32f + (n - 0.5f) * 0.9f;
        float al = wsSmooth(0.0f, 0.75f, dens) * 0.96f;
        float lum = 0.84f + 0.16f * n - 0.16f * wsSmooth(0.0f, 0.9f, (u + v) * 0.5f + 0.25f);     // soft shadow towards the lower right
        const float hx = u + 0.22f, hy = v + 0.26f; lum += 0.12f * std::exp(-(hx * hx + hy * hy) / 0.05f);
        if (wsHash(x, y, 77) > 0.975f) lum = 1.f;                      // a few ice glints
        lum = clamp01(lum);
        wsOver(p, lum * 0.96f, lum * 0.985f, lum, al);
        wsScale(p, wsSmooth(1.0f, 0.86f, dd));
    });
    const bool i = wsMakeTex(d, &g_wsWhiteTex, 4, 4, [&](int, int, WsPx& p) { wsOver(p, 1.f, 1.f, 1.f, 1.f); });
    const bool j = wsMakeTex(d, &g_wsPoolMaskTex, 256, 256, [&](int x, int y, WsPx& p) { wsPoolPx((y / 128) * 2 + (x / 128), ((x % 128) + 0.5f) / 128.f * 2.f - 1.f, ((y % 128) + 0.5f) / 128.f * 2.f - 1.f, false, p); });
    const bool k = wsMakeTex(d, &g_wsPoolShadeTex, 256, 256, [&](int x, int y, WsPx& p) { wsPoolPx((y / 128) * 2 + (x / 128), ((x % 128) + 0.5f) / 128.f * 2.f - 1.f, ((y % 128) + 0.5f) / 128.f * 2.f - 1.f, true, p); });
    const bool l = wsMakeTex(d, &g_wsMicroTex, 16, 16, [&](int x, int y, WsPx& p) { wsMicroPx(((x + 0.5f) / 16.f) * 2.f - 1.f, ((y + 0.5f) / 16.f) * 2.f - 1.f, p); });
    return a && b && c && e && f && g && h && i && j && k && l && m;
}

void wsReleaseScene() {
    for (int i = 0; i < 2; ++i) {
        if (g_wsBlurSurf[i]) { g_wsBlurSurf[i]->Release(); g_wsBlurSurf[i] = nullptr; }
        if (g_wsBlurTex[i]) { g_wsBlurTex[i]->Release(); g_wsBlurTex[i] = nullptr; }
    }
    if (g_wsSceneSurf) { g_wsSceneSurf->Release(); g_wsSceneSurf = nullptr; }
    if (g_wsSceneTex) { g_wsSceneTex->Release(); g_wsSceneTex = nullptr; }
    g_wsSceneW = g_wsSceneH = 0;
}
bool wsEnsureResources(IDirect3DDevice9* d) {
    if (!g_wsSpriteTex && !wsMakeTextures(d)) { g_wsTexFailed = true; Note("Windscreen: texture creation failed, effect disabled"); return false; }
    if (!g_wsSB && FAILED(d->CreateStateBlock(D3DSBT_ALL, &g_wsSB))) { g_wsSB = nullptr; return false; }
    return true;
}
void wsReleaseDeviceObjects() { if (g_wsSB) { g_wsSB->Release(); g_wsSB = nullptr; } wsReleaseScene(); }   // the textures are managed: they survive Reset

bool wsOnBackbuffer(IDirect3DDevice9* d) {                             // skip draws that belong to render-to-texture passes
    IDirect3DSurface9 *rt = nullptr, *bb = nullptr; bool same = false;
    if (SUCCEEDED(d->GetRenderTarget(0, &rt)) && rt) {
        if (SUCCEEDED(d->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb) { same = (rt == bb); bb->Release(); }
        rt->Release();
    }
    return same;
}

// Copy of the frame so far (half resolution, or full size when the back buffer is multisampled), sampled by the refractive drops.
bool wsCopyScene(IDirect3DDevice9* d, const D3DVIEWPORT9& vp, bool needBlur) {
    if (g_wsRefrFailed) return false;
    IDirect3DSurface9* rt = nullptr;
    if (FAILED(d->GetRenderTarget(0, &rt)) || !rt) return false;
    D3DSURFACE_DESC desc{}; rt->GetDesc(&desc);
    const bool msaa = desc.MultiSampleType != D3DMULTISAMPLE_NONE;
    const UINT tw = msaa ? (UINT)vp.Width : std::max<UINT>(64u, (UINT)(vp.Width / 2)), th = msaa ? (UINT)vp.Height : std::max<UINT>(64u, (UINT)(vp.Height / 2));
    if (!g_wsSceneTex || g_wsSceneW != tw || g_wsSceneH != th || g_wsSceneFmt != (int)desc.Format) {
        wsReleaseScene();
        if (FAILED(d->CreateTexture(tw, th, 1, D3DUSAGE_RENDERTARGET, desc.Format, D3DPOOL_DEFAULT, &g_wsSceneTex, nullptr)) || !g_wsSceneTex ||
            FAILED(g_wsSceneTex->GetSurfaceLevel(0, &g_wsSceneSurf)) || !g_wsSceneSurf) {
            wsReleaseScene(); rt->Release(); g_wsRefrFailed = true; Note("Windscreen: refraction disabled (cannot create the scene copy, format %d)", (int)desc.Format); return false;
        }
        g_wsSceneW = tw; g_wsSceneH = th; g_wsSceneFmt = (int)desc.Format;
    }
    RECT src = { (LONG)vp.X, (LONG)vp.Y, (LONG)(vp.X + vp.Width), (LONG)(vp.Y + vp.Height) };
    const HRESULT hr = d->StretchRect(rt, &src, g_wsSceneSurf, nullptr, msaa ? D3DTEXF_NONE : D3DTEXF_LINEAR);
    rt->Release();
    if (FAILED(hr)) { g_wsRefrFailed = true; Note("Windscreen: refraction disabled (StretchRect failed 0x%08X, multisampled=%d)", (unsigned)hr, msaa ? 1 : 0); return false; }
    if (needBlur && !g_wsVeilFailed) {                                 // two more halvings: the veil is a heavily blurred copy of the same frame
        if (!g_wsBlurTex[1]) {
            UINT bw = tw, bh = th; bool ok = true;
            for (int i = 0; i < 2 && ok; ++i) {
                bw = std::max(16u, bw / 2); bh = std::max(16u, bh / 2);
                ok = SUCCEEDED(d->CreateTexture(bw, bh, 1, D3DUSAGE_RENDERTARGET, desc.Format, D3DPOOL_DEFAULT, &g_wsBlurTex[i], nullptr)) && g_wsBlurTex[i] &&
                     SUCCEEDED(g_wsBlurTex[i]->GetSurfaceLevel(0, &g_wsBlurSurf[i])) && g_wsBlurSurf[i];
            }
            if (!ok) { for (int i = 0; i < 2; ++i) { if (g_wsBlurSurf[i]) { g_wsBlurSurf[i]->Release(); g_wsBlurSurf[i] = nullptr; } if (g_wsBlurTex[i]) { g_wsBlurTex[i]->Release(); g_wsBlurTex[i] = nullptr; } }
                       g_wsVeilFailed = true; Note("Windscreen: veil disabled (cannot create the blur copies)"); return true; }
        }
        if (FAILED(d->StretchRect(g_wsSceneSurf, nullptr, g_wsBlurSurf[0], nullptr, D3DTEXF_LINEAR)) || FAILED(d->StretchRect(g_wsBlurSurf[0], nullptr, g_wsBlurSurf[1], nullptr, D3DTEXF_LINEAR)))
        { g_wsVeilFailed = true; Note("Windscreen: veil disabled (StretchRect for the blur failed)"); }
    }
    return true;
}

// depth (0..1 z-buffer value) of the glass plane, from the scene projection: z = s*c33 + c43/d for a point d metres in front of the eye
float wsGlassZ() {
    float s = g_wsPzs, c33 = g_wsPc33, c43 = g_wsPc43;
    if (std::fabs(s) < 0.5f) {
        if (!g_pValid || std::fabs(g_P.m[3][2]) < 0.5f) return -1.f;
        s = g_P.m[3][2]; c33 = g_P.m[2][2]; c43 = g_P.m[2][3];
    }
    return clamp01(s * c33 + c43 / std::max(0.05f, g_cfg.wsGlassDist));
}

