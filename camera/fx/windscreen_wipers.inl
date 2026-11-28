// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ---- wiper pose statistics ----
// Tracks how a value swings back and forth (clean, regular reversals every 0.25-4 s are what a wiper blade does); the wiper hunter below uses it.
struct WsScanStat { float minV, maxV, hi, lo, hiT, loT, lastRevT, sumHP, sumHP2; int dir, n; bool init, bad; };
void wsScanUpdate(WsScanStat& s, float v, float t) {
    if (!std::isfinite(v) || std::fabs(v) > 1.0e6f) { s.bad = true; return; }
    if (!s.init) { s = WsScanStat{}; s.init = true; s.minV = s.maxV = s.hi = s.lo = v; s.hiT = s.loT = t; return; }
    s.minV = std::min(s.minV, v); s.maxV = std::max(s.maxV, v);
    const float thr = std::max(0.10f * (s.maxV - s.minV), 1e-3f);
    auto rev = [&](float at) { if (s.lastRevT > 0.f) { const float hp = at - s.lastRevT; s.n++; s.sumHP += hp; s.sumHP2 += hp * hp; } s.lastRevT = at; };
    if (s.dir == 0) {
        s.hi = std::max(s.hi, v); s.lo = std::min(s.lo, v);
        if (s.hi - s.lo > thr) { s.dir = (v >= 0.5f * (s.hi + s.lo)) ? 1 : -1; s.hi = s.lo = v; s.hiT = s.loT = t; }
    } else if (s.dir == 1) {
        if (v > s.hi) { s.hi = v; s.hiT = t; }
        else if (s.hi - v > thr) { rev(s.hiT); s.dir = -1; s.lo = v; s.loT = t; }
    } else {
        if (v < s.lo) { s.lo = v; s.loT = t; }
        else if (v - s.lo > thr) { rev(s.loT); s.dir = 1; s.hi = v; s.hiT = t; }
    }
}
// ---- wiper hunter ----
// The car's wipers are an ordinary 3D model. Every draw call of the cockpit pass carries its matrix (MVP = P * V * M through shader constants 0-3), and
// P^-1 * MVP is the object's placement in camera space. A wiper blade is the placement that swings back and forth with a regular rhythm: the steering
// wheel, pedals and needles move irregularly, the rest of the car is still, the world only moves one way. The hunter watches every draw (its mesh
// signature plus its occurrence index in the frame), and when two 6 s windows agree it locks onto up to two blades and reads their pose every frame.
struct WsHuntEnt { WsScanStat st[12]; float T[12]; float ext[9][2][9]; long lastFrame; };     // T = 3x3 rotation (row-major) then translation; ext = rotation at each extreme of each element
std::unordered_map<uint64_t, WsHuntEnt> g_wsHunt;
struct WsHuntSlot { uint32_t sig; int cnt; long frame; };
WsHuntSlot g_wsSlots[2048];
struct WsLockE { bool on; uint64_t key; float theta, hp; };
WsLockE g_wsLock[2] = {{false, 0, 0, 0}, {false, 0, 0, 0}};
uint64_t g_wsPrevSel[2] = {0, 0}; float g_wsHuntT0 = 0.f;

uint32_t wsSig(DWORD type, UINT a, UINT b, UINT c, UINT d, UINT e) {
    uint32_t h = 2166136261u; const uint32_t v[6] = {(uint32_t)type, (uint32_t)a, (uint32_t)b, (uint32_t)c, (uint32_t)d, (uint32_t)e};
    for (int i = 0; i < 6; ++i) { h ^= v[i]; h *= 16777619u; h ^= h >> 15; }
    return h;
}
inline bool wsHuntActive() { return g_in3D && g_wsArmed && !g_wsDrawing && (g_cfg.wsWipers && g_cfg.wsWiperSource); }
float wsRotAngle(const float* Ra, const float* Rb) {                  // angle between two rotations: acos((tr(Ra^T Rb) - 1) / 2)
    float tr = 0.f; for (int i = 0; i < 9; ++i) tr += Ra[i] * Rb[i];
    return std::acos(std::max(-1.f, std::min(1.f, (tr - 1.f) * 0.5f)));
}
void wsHuntReset() {
    g_wsHunt.clear(); memset(g_wsSlots, 0, sizeof(g_wsSlots));
    g_wsLock[0].on = g_wsLock[1].on = false; g_wsHuntLocked = false; g_wsPrevSel[0] = g_wsPrevSel[1] = 0; g_wsHuntT0 = 0.f;
    g_wsModelBlade[0].ok = g_wsModelBlade[1].ok = false; g_wsMeas[0] = g_wsMeas[1] = WsMeasure{}; g_wsSegHave[0] = g_wsSegHave[1] = false; g_wsDrawLock = -1;
}
// Called for every 3D draw call while hunting or while a blade is locked. Returns true when the draw is a locked blade that should be hidden.
bool wsHuntDraw(uint32_t sig) {
    g_wsDrawLock = -1;
    if (!g_lastMvpOk || !g_wsPok) return false;
    WsHuntSlot* sl = nullptr;                                          // occurrence index of this mesh in the frame (two wipers can share a mesh)
    for (int p = 0; p < 8 && !sl; ++p) { WsHuntSlot& s = g_wsSlots[(sig + (uint32_t)p) & 2047u]; if (s.frame == g_wsFrameNo && s.sig == sig) sl = &s; }
    for (int p = 0; p < 8 && !sl; ++p) { WsHuntSlot& s = g_wsSlots[(sig + (uint32_t)p) & 2047u]; if (s.frame != g_wsFrameNo) { sl = &s; s.sig = sig; s.cnt = 0; s.frame = g_wsFrameNo; } }
    if (!sl) return false;
    const int k = std::min(sl->cnt++, 255);
    const uint64_t key = ((uint64_t)sig << 8) | (uint64_t)k;
    const bool l0 = g_wsLock[0].on && g_wsLock[0].key == key, l1 = g_wsLock[1].on && g_wsLock[1].key == key;
    g_wsDrawLock = l0 ? 0 : (l1 ? 1 : -1);
    const bool hunting = !g_wsHuntLocked;
    if (!hunting && !l0 && !l1) return false;
    const M4 T = m4Mul(g_wsPinv, g_lastMvp);                           // V * M: the object's placement in camera space
    float v[12]; for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) v[r * 3 + c] = T.m[r][c];
    v[9] = T.m[0][3]; v[10] = T.m[1][3]; v[11] = T.m[2][3];
    if (g_wsHunt.size() > 3000 && g_wsHunt.find(key) == g_wsHunt.end()) return l0 || l1;
    WsHuntEnt& e = g_wsHunt[key];
    memcpy(e.T, v, sizeof(v)); e.lastFrame = g_wsFrameNo;
    if (hunting) {
        const float t = (float)GetTickCount() / 1000.f;
        for (int i = 0; i < 12; ++i) {
            WsScanStat& s = e.st[i]; const bool was = s.init; const float pmin = s.minV, pmax = s.maxV;
            wsScanUpdate(s, v[i], t);
            if (i < 9 && !s.bad) {
                if (!was || v[i] < pmin) memcpy(e.ext[i][0], v, 9 * sizeof(float));
                if (!was || v[i] > pmax) memcpy(e.ext[i][1], v, 9 * sizeof(float));
            }
        }
    }
    return (l0 || l1) && g_cfg.wsHideGameWipers;
}
// Reads the vertices of a locked wiper blade's draw call once, and measures the mesh: its long axis and how far it reaches (see wsl::measureExtent).
// This replaces the guess in the ini (WiperAxis, WiperLenM). If the vertex buffer cannot be read, or the mesh does not look like a wiper,
// nothing changes and the ini values stay in use.
void wsMeasureBlade(void* devp, int lockIdx, INT baseVtx, UINT minVtx, UINT numVtx) {
    if (lockIdx < 0 || lockIdx > 1 || g_wsMeas[lockIdx].tried) return;
    WsMeasure& m = g_wsMeas[lockIdx]; m.tried = true;
    IDirect3DDevice9* d = (IDirect3DDevice9*)devp; IDirect3DVertexBuffer9* vb = nullptr; UINT off = 0, stride = 0;
    if (!d || FAILED(d->GetStreamSource(0, &vb, &off, &stride)) || !vb) { Logf("WiperHunt: blade %d: cannot read the vertex stream, using WiperAxis / WiperLenM", lockIdx + 1); return; }
    D3DVERTEXBUFFER_DESC desc{}; vb->GetDesc(&desc);
    const long first = (long)baseVtx + (long)minVtx;
    if (first < 0 || stride < 12 || numVtx < 3 || numVtx > 20000 || (unsigned long long)off + (unsigned long long)(first + numVtx) * stride > desc.Size) {
        Logf("WiperHunt: blade %d: unexpected vertex range (base %d min %u count %u stride %u), using WiperAxis / WiperLenM", lockIdx + 1, (int)baseVtx, minVtx, numVtx, stride); vb->Release(); return;
    }
    void* ptr = nullptr; HRESULT hr = vb->Lock(off + (UINT)first * stride, numVtx * stride, &ptr, D3DLOCK_READONLY);
    if (SUCCEEDED(hr) && ptr) { m.ext = wsl::measureExtent((const uint8_t*)ptr, (int)numVtx, (int)stride); vb->Unlock(); }
    vb->Release();
    if (m.ext.ok) Logf("WiperHunt: blade %d measured: axis %d, %.3f .. %.3f model units (%d vertices, stride %u). Rubber and direction now come from the mesh", lockIdx + 1, m.ext.axis, m.ext.lo, m.ext.hi, (int)numVtx, stride);
    else Logf("WiperHunt: blade %d: the mesh could not be measured (lock 0x%08X, %d vertices, stride %u), using WiperAxis / WiperLenM", lockIdx + 1, (unsigned)hr, (int)numVtx, stride);
}

bool wsProjectCam(float x, float y, float z, float W, float H, float& ox, float& oy) {    // camera-space point -> pixels of the base-pose frame
    const float cx = g_wsP.m[0][0] * x + g_wsP.m[0][1] * y + g_wsP.m[0][2] * z + g_wsP.m[0][3];
    const float cy = g_wsP.m[1][0] * x + g_wsP.m[1][1] * y + g_wsP.m[1][2] * z + g_wsP.m[1][3];
    const float cw = g_wsP.m[3][0] * x + g_wsP.m[3][1] * y + g_wsP.m[3][2] * z + g_wsP.m[3][3];
    if (cw < 1e-4f) return false;
    ox = (cx / cw * 0.5f + 0.5f) * W; oy = (0.5f - cy / cw * 0.5f) * H; return true;
}
void wsHuntPose(float W, float H) {                                    // the locked blades' pivot, length, angle and rubber end points on the glass, from their current matrices
    for (int i = 0; i < 2; ++i) {
        WsModelBlade& mb = g_wsModelBlade[i]; mb.ok = false;
        if (!g_cfg.wsWiperSource || !g_wsLock[i].on || !g_wsPok) continue;
        const auto it = g_wsHunt.find(g_wsLock[i].key); if (it == g_wsHunt.end()) continue;
        const WsHuntEnt& e = it->second; if (g_wsFrameNo - e.lastFrame > 30) continue;      // not drawn recently
        // Direction and extent: measured from the mesh's own vertices when that worked (WiperMeasure=1), else the ini's WiperAxis / WiperLenM.
        int ax = std::max(0, std::min(2, g_cfg.wsWiperAxis)); float rA, rB, tip;           // rubber from rA to rB, tip = far end, positions along the axis in model units
        const float sgn = g_cfg.wsWiperAxisSign < 0 ? -1.f : 1.f, Lm = g_cfg.wsWiperLenM;
        const bool measured = g_cfg.wsWiperMeasure && g_wsMeas[i].ext.ok;
        if (measured) { ax = g_wsMeas[i].ext.axis; wsl::rubberSpan(g_wsMeas[i].ext, g_cfg.wsWiperInner, rA, rB); tip = (std::fabs(g_wsMeas[i].ext.hi) >= std::fabs(g_wsMeas[i].ext.lo)) ? g_wsMeas[i].ext.hi : g_wsMeas[i].ext.lo; }
        else { rA = sgn * Lm * std::max(0.f, std::min(0.9f, g_cfg.wsWiperInner)); rB = sgn * Lm; tip = sgn * Lm; }
        const float cx = e.T[0 * 3 + ax], cy = e.T[1 * 3 + ax], cz = e.T[2 * 3 + ax];      // the model axis in camera space
        float s0x, s0y, sTx, sTy, sAx, sAy, sBx, sBy;
        if (!wsProjectCam(e.T[9], e.T[10], e.T[11], W, H, s0x, s0y)) continue;
        if (!wsProjectCam(e.T[9] + cx * tip, e.T[10] + cy * tip, e.T[11] + cz * tip, W, H, sTx, sTy)) continue;
        if (!wsProjectCam(e.T[9] + cx * rA, e.T[10] + cy * rA, e.T[11] + cz * rA, W, H, sAx, sAy)) continue;
        if (!wsProjectCam(e.T[9] + cx * rB, e.T[10] + cy * rB, e.T[11] + cz * rB, W, H, sBx, sBy)) continue;
        mb.ok = true; mb.px = s0x / W; mb.py = s0y / H; mb.len = std::sqrt((sTx - s0x) * (sTx - s0x) + (sTy - s0y) * (sTy - s0y)) / H; mb.ang = std::atan2(s0y - sTy, sTx - s0x);
        mb.seg = {sAx, sAy, sBx, sBy};
    }
}
void wsHuntReport() {
    struct Cand { uint64_t key; int nPass, idx; float cv, hp, theta, px, py; };
    std::vector<Cand> c; int tracked = 0;
    const float W = (float)std::max(1u, (unsigned)g_wsVpW), H = (float)std::max(1u, (unsigned)g_wsVpH);
    for (auto& kv : g_wsHunt) {
        const WsHuntEnt& e = kv.second; if (g_wsFrameNo - e.lastFrame > 120) continue;
        ++tracked;
        int nPass = 0, idx = -1; float cvSum = 0.f, best = 0.f, hp = 0.f;
        for (int i = 0; i < 9; ++i) {                                  // the 9 rotation elements: a swinging blade makes several of them swing with one rhythm
            const WsScanStat& s = e.st[i]; if (!s.init || s.bad || s.n < 4) continue;
            const float mean = s.sumHP / (float)s.n, var = std::max(0.f, s.sumHP2 / (float)s.n - mean * mean), cv = std::sqrt(var) / std::max(mean, 1e-3f), range = s.maxV - s.minV;
            if (mean < 0.25f || mean > 4.f || cv >= 0.25f || range < 0.08f) continue;
            ++nPass; cvSum += cv; if (range > best) { best = range; idx = i; hp = mean; }
        }
        if (nPass < 3 || idx < 0) continue;
        const float th = wsRotAngle(e.ext[idx][0], e.ext[idx][1]); if (th < 0.35f) continue;   // it must really sweep through an angle
        Cand cd{kv.first, nPass, idx, cvSum / (float)nPass, hp, th, 0.f, 0.f};
        wsProjectCam(e.T[9], e.T[10], e.T[11], W, H, cd.px, cd.py); cd.px /= W; cd.py /= H;
        c.push_back(cd);
    }
    std::sort(c.begin(), c.end(), [](const Cand& a, const Cand& b) { return a.cv < b.cv; });
    Logf("WiperHunt: %d draws tracked, %d swinging candidate(s) in the last 6 s%s", tracked, (int)c.size(), g_wsHuntLocked ? " (locked)" : "");
    for (size_t k = 0; k < c.size() && k < 6; ++k)
        Logf("WiperHunt candidate: mesh %08X #%d  half-period %.2fs  irregularity %.2f  sweep %.0f deg  swinging elements %d  pivot at %.2f,%.2f of the screen",
             (unsigned)(c[k].key >> 8), (int)(c[k].key & 255u), c[k].hp, c[k].cv, c[k].theta * 57.2958f, c[k].nPass, c[k].px, c[k].py);
    if (!g_wsHuntLocked && g_cfg.wsWipers && g_cfg.wsWiperSource) {
        std::vector<Cand> sel;                                         // up to two blades: different pivots (an arm and its blade share one), the same rhythm
        for (const Cand& cd : c) {
            if (sel.size() >= 2) break;
            bool ok = true;
            for (const Cand& s : sel) if (std::fabs(cd.px - s.px) + std::fabs(cd.py - s.py) < 0.04f || std::fabs(cd.hp - s.hp) > 0.25f * s.hp) ok = false;
            if (ok) sel.push_back(cd);
        }
        bool agree = !sel.empty();                                     // only lock when the previous window picked the same meshes
        for (const Cand& s : sel) if (s.key != g_wsPrevSel[0] && s.key != g_wsPrevSel[1]) agree = false;
        g_wsPrevSel[0] = sel.size() > 0 ? sel[0].key : 0; g_wsPrevSel[1] = sel.size() > 1 ? sel[1].key : 0;
        if (agree) {
            for (size_t i = 0; i < sel.size(); ++i) {
                g_wsLock[i] = {true, sel[i].key, sel[i].theta, sel[i].hp}; g_wsMeas[i] = WsMeasure{}; g_wsSegHave[i] = false;
                Logf("WiperHunt: LOCKED blade %d = mesh %08X #%d, sweep %.0f deg, stroke %.2fs, pivot %.2f,%.2f. WiperAxis=%d WiperLenM=%.2f decide the blade direction and length (WiperDebug=1 shows it)",
                     (int)i + 1, (unsigned)(sel[i].key >> 8), (int)(sel[i].key & 255u), sel[i].theta * 57.2958f, sel[i].hp * 2.f, sel[i].px, sel[i].py, g_cfg.wsWiperAxis, g_cfg.wsWiperLenM);
            }
            g_wsHuntLocked = true;
        }
    }
    for (auto it = g_wsHunt.begin(); it != g_wsHunt.end();) {          // start a fresh window; forget meshes that are no longer drawn
        if (g_wsFrameNo - it->second.lastFrame > 300) it = g_wsHunt.erase(it);
        else { for (WsScanStat& s : it->second.st) s = WsScanStat{}; ++it; }
    }
}
void wsHuntTick() {                                                    // called once per drawn frame
    if (!(g_cfg.wsWipers && g_cfg.wsWiperSource)) return;
    const float t = (float)GetTickCount() / 1000.f;
    if (g_wsHuntT0 == 0.f) g_wsHuntT0 = t;
    if (g_wsHuntLocked) return;
    if (t - g_wsHuntT0 < 6.f) return;
    g_wsHuntT0 = t; wsHuntReport();
}

