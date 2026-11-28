// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ================================================================================================ hook 1: set_camera_target
// Declared exactly like openRBRVR declares it (RBR.hpp), default calling convention.
using SetCamTargetFn = void* (*)(void* a, float* camPos, float* camTarget);
SetCamTargetFn g_orig = nullptr;
constexpr uint32_t kSeatMoveMagic = 0x0DDB411;                         // openRBRVR's marker: a camera type the game never uses
std::atomic<long> g_natCalls{0}, g_applies{0};
float g_inPos[3] = {0}, g_inTgt[3] = {0};                              // last natural incoming values (for the log)
bool g_warnedInsane = false;

// Method 1 cache: camera object and base values from the game's own (natural) camera setup calls.
void*    g_camObj = nullptr;
float    g_basePos[3] = {0}, g_baseTgt[3] = {0};
uint32_t g_baseType = 0xFFFFFFFF;
bool     g_haveBase = false;
void*    g_pendObj = nullptr; float g_pendPos[3] = {0}, g_pendTgt[3] = {0};
long     g_pendSeq = 0, g_pendConsumed = 0;                            // natural-call counter / last one accepted
uint32_t g_prevType = 0xFFFFFFFE;
volatile bool g_inDirect = false;                                      // set while WE call the original (hook passes through)
volatile bool g_forcing = false;                                       // method 2: set while we force a camera change
bool g_haveLast = false; float g_lastPosIn[3], g_lastTgtIn[3], g_lastPosOut[3], g_lastTgtOut[3];

bool nearEq(const float* a, const float* b) {
    for (int i = 0; i < 3; ++i) if (std::fabs(a[i] - b[i]) > 1e-6f) return false;
    return true;
}

void* hkSetCamTarget(void* a, float* camPos, float* camTarget) {
    if (g_inDirect) return g_orig(a, camPos, camTarget);               // our own direct call: untouched
    const bool ok = sane3(camPos) && sane3(camTarget);
    if (!g_forcing) {
        g_natCalls++;
        if (ok) { memcpy(g_inPos, camPos, 12); memcpy(g_inTgt, camTarget, 12); }
        else if (!g_warnedInsane) { Note("WARNING: cam_pos/cam_target look invalid, not applying (hook args wrong?)"); g_warnedInsane = true; }
        // NOTE: the camera-type slot still holds the PREVIOUS type here. Remember object + values as "pending"; the
        // render hook accepts them once the camera type has actually switched to a cockpit type (method 1).
        if (ok) {
            g_pendObj = a; memcpy(g_pendPos, camPos, 12); memcpy(g_pendTgt, camTarget, 12); g_pendSeq++;
            if (g_haveBase && readCameraType() == g_baseType) { g_camObj = a; memcpy(g_basePos, camPos, 12); memcpy(g_baseTgt, camTarget, 12); }
        }
    } else if (ok) {
        // Method 2: we forced the camera change; add the head offset here. If the engine hands back the values we
        // modified last time (camera data persisted), restart from the unmodified inputs so offsets never accumulate.
        if (g_haveLast && nearEq(camPos, g_lastPosOut) && nearEq(camTarget, g_lastTgtOut)) {
            memcpy(camPos, g_lastPosIn, 12); memcpy(camTarget, g_lastTgtIn, 12);
        }
        memcpy(g_lastPosIn, camPos, 12); memcpy(g_lastTgtIn, camTarget, 12);
        applyHead(camPos, camTarget);
        memcpy(g_lastPosOut, camPos, 12); memcpy(g_lastTgtOut, camTarget, 12); g_haveLast = true;
    }
    return g_orig(a, camPos, camTarget);
}

// ================================================================================================ hook 2: per-frame render
using ChangeCameraFn  = void(__thiscall*)(void* cam, int type, uint32_t a);
using PrepareCameraFn = void(__thiscall*)(void* self, uint32_t a);
using RenderFn        = void(__fastcall*)(void* p);
RenderFn g_origRender = nullptr;

// Method 3 (wiper-friendly, experimental): call the original set_camera_target ourselves with base + offset, then run only
// apply_camera_position. change_camera (which Method 2 runs every frame) is skipped: it is the likely cause of the wipers
// stopping. POD only (SEH). g_inDirect keeps the hook out of the way for both calls.
bool applyDirect(void* p) {
    __try {
        float pos[3], tgt[3];
        memcpy(pos, g_basePos, 12); memcpy(tgt, g_baseTgt, 12);
        applyHead(pos, tgt);
        g_inDirect = true;
        g_orig(g_camObj, pos, tgt);
        ((PrepareCameraFn)absAddr(0x4825B0))(p, 0);
        g_inDirect = false;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) { g_inDirect = false; return false; }
}

// Method 2 (default, confirmed to move the camera, but stops the wipers): openRBRVR's force_camera_change: put the magic value in the type slot, then change_camera(p, realType).
bool forceCameraChange(void* p, uint32_t type) {
    __try {
        const uintptr_t carInfo = *(uintptr_t*)absAddr(0x165FC68);   if (!carInfo) return false;
        const uintptr_t camData = *(uintptr_t*)(carInfo + 0x758);    if (!camData) return false;
        void* camInfo = *(void**)(camData + 0x10);                   if (!camInfo) return false;
        void* fovThis = *(void**)((uintptr_t)p + 0xCF4);
        *(uint32_t*)camInfo = kSeatMoveMagic;
        g_forcing = true;
        ((ChangeCameraFn)absAddr(0x487680))(camInfo, (int)type, 1);
        ((PrepareCameraFn)absAddr(0x4825B0))(p, 0);
        if (fovThis) ((PrepareCameraFn)absAddr(0x4BF690))(fovThis, 0);
        g_forcing = false;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) { g_forcing = false; return false; }
}

// ---------------------------------------------------------------- diagnostics: what does the forced camera change modify?
// With Diag=1, on 3 consecutive forced camera changes (after DiagAfter of them, so the wipers have been running) the plugin
// snapshots several game structures before and after the change and logs every 32-bit word that differs. A wiper timer /
// state that gets reset shows up as a value that jumps back (often to 0). Send me the log from a rain stage.
struct CamPtrs { uintptr_t p = 0, carInfo = 0, camData = 0, camInfo = 0, fovThis = 0; };
struct Snap { std::string name; uintptr_t addr; std::vector<uint8_t> data; };

bool getCamPtrs(void* p, CamPtrs& o) {                                  // POD only (SEH)
    __try {
        o.p = (uintptr_t)p;
        o.carInfo = *(uintptr_t*)absAddr(0x165FC68);                 if (!o.carInfo) return false;
        o.camData = *(uintptr_t*)(o.carInfo + 0x758);                if (!o.camData) return false;
        o.camInfo = *(uintptr_t*)(o.camData + 0x10);                 if (!o.camInfo) return false;
        o.fovThis = *(uintptr_t*)((uintptr_t)p + 0xCF4);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}
bool readableRange(uintptr_t a, size_t n) {
    const uintptr_t end = a + n;
    while (a < end) {
        MEMORY_BASIC_INFORMATION mbi;
        if (!VirtualQuery((LPCVOID)a, &mbi, sizeof(mbi))) return false;
        if (mbi.State != MEM_COMMIT || (mbi.Protect & (PAGE_NOACCESS | PAGE_GUARD))) return false;
        a = (uintptr_t)mbi.BaseAddress + mbi.RegionSize;
    }
    return true;
}
// Snapshot the camera structures; with deep=true also every heap object that those structures point to (256 bytes each).
std::vector<Snap> takeSnaps(const CamPtrs& c, bool deep) {
    struct R { const char* n; uintptr_t a; size_t sz; } rs[] = {
        {"camInfo", c.camInfo, 0x400}, {"camData", c.camData, 0x400}, {"carInfo", c.carInfo, 0x1000},
        {"p", c.p, 0x2000}, {"fov", c.fovThis, 0x400}};
    std::vector<Snap> out;
    for (const R& r : rs) {
        if (!r.a) continue;
        size_t sz = r.sz;
        while (sz >= 0x40 && !readableRange(r.a, sz)) sz /= 2;          // shrink until readable
        if (sz < 0x40) continue;
        Snap s; s.name = r.n; s.addr = r.a; s.data.assign((const uint8_t*)r.a, (const uint8_t*)r.a + sz);
        out.push_back(std::move(s));
    }
    if (deep) {
        const size_t baseCount = out.size();
        std::set<uintptr_t> seen; std::vector<Snap> extra;
        for (size_t i = 0; i < baseCount && extra.size() < 3000; ++i) {
            for (size_t o = 0; o + 4 <= out[i].data.size() && extra.size() < 3000; o += 4) {
                uint32_t w; memcpy(&w, &out[i].data[o], 4);
                if (w < 0x00100000u || w >= 0x7FF00000u || (w & 3)) continue;     // plausible user-space pointer
                const uintptr_t t = (uintptr_t)w & ~(uintptr_t)0xF;
                if (!seen.insert(t).second) continue;
                bool inBase = false;
                for (size_t k = 0; k < baseCount; ++k) if (t >= out[k].addr && t < out[k].addr + out[k].data.size()) { inBase = true; break; }
                if (inBase || !readableRange(t, 0x100)) continue;
                Snap e; char nm[64]; snprintf(nm, sizeof(nm), "%s+0x%X->%08X", out[i].name.c_str(), (unsigned)o, (unsigned)t);
                e.name = nm; e.addr = t; e.data.assign((const uint8_t*)t, (const uint8_t*)t + 0x100);
                extra.push_back(std::move(e));
            }
        }
        for (Snap& e : extra) out.push_back(std::move(e));
    }
    return out;
}
// s0 and s1 are taken back to back BEFORE the forced change, s2 right after it. A word is reported only if it changed
// between s1 and s2 but NOT between s0 and s1 (so plain time-passing counters and timers are filtered out).
void logDiff3(const std::vector<Snap>& s0, const std::vector<Snap>& s1, const std::vector<Snap>& s2) {
    std::map<uintptr_t, const Snap*> m0, m2;
    for (const Snap& s : s0) m0[s.addr] = &s;
    for (const Snap& s : s2) m2[s.addr] = &s;
    Logf("DIFF ---- forced camera change over %u regions: words changed by the call (not by plain time passing)", (unsigned)s1.size());
    int printed = 0, regionsHit = 0;
    for (const Snap& a : s1) {
        const auto i0 = m0.find(a.addr), i2 = m2.find(a.addr);
        if (i0 == m0.end() || i2 == m2.end()) continue;
        const Snap& z = *i0->second; const Snap& b = *i2->second;
        const size_t n = std::min(std::min(a.data.size(), z.data.size()), b.data.size());
        int shown = 0, total = 0;
        for (size_t o = 0; o + 4 <= n; o += 4) {
            uint32_t x0, x1, x2; memcpy(&x0, &z.data[o], 4); memcpy(&x1, &a.data[o], 4); memcpy(&x2, &b.data[o], 4);
            if (x1 == x2 || x0 != x1) continue;
            ++total;
            if (shown < 12 && printed < 400) {
                float f1, f2; memcpy(&f1, &x1, 4); memcpy(&f2, &x2, 4);
                Logf("DIFF %s @%08X +0x%03X: %08X -> %08X  (float %g -> %g)", a.name.c_str(), (unsigned)a.addr, (unsigned)o, x1, x2, f1, f2);
                ++shown; ++printed;
            }
        }
        if (total) ++regionsHit;
    }
    Logf("DIFF done: %d regions changed, %d lines shown", regionsHit, printed);
}
int g_diagCount = 0;

// ================================================================================================ Method 4: render-matrix patch
// This is what openRBRVR does for head tracking (and why it does not disturb the wipers): the game's camera is never touched.
// Instead the Direct3D 9 matrices are rewritten at draw time:
//   * SetTransform(D3DTS_PROJECTION) is observed to learn the projection P (and its inverse).
//   * SetVertexShaderConstantF(register 0, 4 vectors) carries the MVP of every shader-drawn 3D object:  MVP' = P * Hinv * P^-1 * MVP
//     where Hinv is the head pose inverse in view space. Equivalent to moving the camera, so cabin and world move correctly
//     relative to each other, and it allows roll.
//   * Optionally (Render.FixedFunction=1) SetTransform(D3DTS_VIEW) for fixed-function draws (BTB / custom tracks).
// Hooks are installed by hooking Direct3DCreate9 -> IDirect3D9::CreateDevice -> device vtable, exactly like openRBRVR.
// Matrices below are plain row-major math matrices in the COLUMN-vector convention (clip = M * v).
struct M4 { float m[4][4]; };
M4 m4Ident() { M4 r{}; for (int i = 0; i < 4; ++i) r.m[i][i] = 1.f; return r; }
M4 m4Mul(const M4& a, const M4& b) {
    M4 r{};
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) { float s = 0; for (int k = 0; k < 4; ++k) s += a.m[i][k] * b.m[k][j]; r.m[i][j] = s; }
    return r;
}
M4 m4T(const M4& a) { M4 r{}; for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) r.m[i][j] = a.m[j][i]; return r; }
bool m4Inv(const M4& a, M4& out) {                                     // Gauss-Jordan with partial pivoting
    double g[4][8];
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) { g[i][j] = a.m[i][j]; g[i][4 + j] = (i == j) ? 1.0 : 0.0; }
    for (int c = 0; c < 4; ++c) {
        int piv = c; for (int r = c + 1; r < 4; ++r) if (std::fabs(g[r][c]) > std::fabs(g[piv][c])) piv = r;
        if (std::fabs(g[piv][c]) < 1e-12) return false;
        if (piv != c) for (int j = 0; j < 8; ++j) std::swap(g[c][j], g[piv][j]);
        const double d = g[c][c]; for (int j = 0; j < 8; ++j) g[c][j] /= d;
        for (int r = 0; r < 4; ++r) if (r != c) { const double f = g[r][c]; if (f != 0.0) for (int j = 0; j < 8; ++j) g[r][j] -= f * g[c][j]; }
    }
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) out.m[i][j] = (float)g[i][4 + j];
    return true;
}
void mul3(const float a[3][3], const float b[3][3], float r[3][3]) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) { float s = 0; for (int k = 0; k < 3; ++k) s += a[i][k] * b[k][j]; r[i][j] = s; }
}

// Inverse of the head pose in view space (applied to the world). Head frame in a left-handed view space (x right, y up, z forward):
// + pitch = look down, + yaw = look right, + roll = tilt right (right ear down). zSign < 0 converts to a right-handed view space.
M4 buildHinv(float F, float R, float U, float pitchRad, float yawRad, float rollRad, int zSign) {
    const float cp = std::cos(pitchRad), sp = std::sin(pitchRad), cy = std::cos(yawRad), sy = std::sin(yawRad),
                cr = std::cos(rollRad), sr = std::sin(rollRad);
    const float Rx[3][3] = {{1, 0, 0}, {0, cp, -sp}, {0, sp, cp}};
    const float Ry[3][3] = {{cy, 0, sy}, {0, 1, 0}, {-sy, 0, cy}};
    const float Rz[3][3] = {{cr, sr, 0}, {-sr, cr, 0}, {0, 0, 1}};
    float A[3][3], Rh[3][3]; mul3(Ry, Rx, A); mul3(A, Rz, Rh);         // head orientation = Ry * Rx * Rz
    const float t[3] = {R, U, F};
    M4 H = m4Ident();
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) H.m[i][j] = Rh[j][i];                 // Rh^T
    for (int i = 0; i < 3; ++i) H.m[i][3] = -(Rh[0][i] * t[0] + Rh[1][i] * t[1] + Rh[2][i] * t[2]); // -Rh^T * t
    if (zSign < 0) for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) if ((i == 2) != (j == 2)) H.m[i][j] = -H.m[i][j];
    return H;
}

using D3DCreate9Fn   = void* (WINAPI*)(UINT);
using CreateDeviceFn = HRESULT(__stdcall*)(void* self, UINT adapter, UINT devType, HWND hwnd, DWORD flags, void* pp, void** ppDev);
using SetTransformFn = HRESULT(__stdcall*)(void* dev, DWORD state, const float* m16);
using SetVSConstFFn  = HRESULT(__stdcall*)(void* dev, UINT start, const float* data, UINT count);
D3DCreate9Fn   g_origD3DCreate9 = nullptr;
CreateDeviceFn g_origCreateDevice = nullptr;
SetTransformFn g_origSetTransform = nullptr;
SetVSConstFFn  g_origSetVSConst = nullptr;
bool g_createDeviceHooked = false, g_devHooked = false;
std::string g_d3dPath;

M4 g_P = m4Ident(), g_Pinv = m4Ident(), g_Hinv = m4Ident(), g_Comb = m4Ident();
bool g_pValid = false, g_combDirty = true; int g_zSign = 1;
volatile bool g_in3D = false, g_inWindscreenFx = false, g_patchActive = false;
M4 g_lastMvp = m4Ident(); bool g_lastMvpOk = false;                    // the game's own MVP of the draw about to happen (before the head patch), for the Windscreen wiper hunter
M4 g_wsP = m4Ident(), g_wsPinv = m4Ident(); bool g_wsPok = false;      // full scene projection (+ inverse) captured during the 3D pass, for Windscreen
float g_wsPzs = 0.f, g_wsPc33 = 0.f, g_wsPc43 = 0.f;                   // scene projection terms for the glass depth (Windscreen), captured during the 3D pass
std::atomic<long> g_patched{0};

HRESULT __stdcall hkSetTransform(void* dev, DWORD state, const float* m) {
    if (state == 3) {                                                  // D3DTS_PROJECTION
        if (g_in3D && m) { g_wsPzs = m[11]; g_wsPc33 = m[10]; g_wsPc43 = m[14]; }
        M4 pc; for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) pc.m[r][c] = m[4 * c + r];   // D3D row-vector -> column-vector
        M4 inv;
        if (m4Inv(pc, inv)) { g_P = pc; g_Pinv = inv; g_pValid = true; g_zSign = (pc.m[3][2] > 0.f) ? 1 : -1; g_combDirty = true; if (g_in3D) { g_wsP = pc; g_wsPinv = inv; g_wsPok = true; } }
    } else if (state == 2 && g_patchActive && g_in3D && !g_inWindscreenFx && g_cfg.fixedFunction && m) {   // D3DTS_VIEW (fixed-function draws)
        M4 v; for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) v.m[r][c] = m[4 * r + c];
        const M4 out = m4Mul(v, m4T(g_Hinv));                          // row-vector convention: V' = V * Hinv^T
        float buf[16]; for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) buf[4 * r + c] = out.m[r][c];
        g_patched++;
        return g_origSetTransform(dev, state, buf);
    }
    return g_origSetTransform(dev, state, m);
}

HRESULT __stdcall hkSetVSConst(void* dev, UINT start, const float* data, UINT count) {
    if (start == 0 && count == 4 && g_in3D && !g_inWindscreenFx && data) { for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) g_lastMvp.m[r][c] = data[4 * r + c]; g_lastMvpOk = true; }
    if (start == 0 && count == 4 && g_patchActive && g_in3D && !g_inWindscreenFx && g_pValid && data) {
        if (g_combDirty) { g_Comb = m4Mul(g_P, m4Mul(g_Hinv, g_Pinv)); g_combDirty = false; }
        M4 mvp; for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) mvp.m[r][c] = data[4 * r + c];   // rows of the column-vector MVP
        const M4 out = m4Mul(g_Comb, mvp);                             // MVP' = P * Hinv * P^-1 * MVP
        float buf[16]; for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) buf[4 * r + c] = out.m[r][c];
        g_patched++;
        return g_origSetVSConst(dev, start, buf, count);
    }
    return g_origSetVSConst(dev, start, data, count);
}

