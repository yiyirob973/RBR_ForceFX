// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ---- hooks ----
// Draw once per frame, from whichever hook point is selected. The stamp check stops a stale flag from drawing over menus.
void wsDrawFrom(int point, void* dev) {
    if (g_cfg.wsDrawAt != point || !g_wsArmed || !dev || g_wsLastDrawn == g_wsFrameNo || (GetTickCount() - g_wsStamp) >= 100) return;
    if (wsDraw((IDirect3DDevice9*)dev)) g_wsLastDrawn = g_wsFrameNo;
}
HRESULT __stdcall hkEndScene(void* dev) { wsDrawFrom(0, dev); return g_origEndScene(dev); }          // DrawAt=0: end of frame (over the HUD)
HRESULT __stdcall hkReset(void* dev, void* pp) { wsReleaseDeviceObjects(); return g_origReset(dev, pp); }
// While the game's own wet-windscreen function runs (HideGameFx=1) its draw calls are dropped here; its logic still runs, so the wipers keep working.
HRESULT __stdcall hkDrawPrim(void* dev, DWORD t, UINT s, UINT p) {
    if (g_wsSuppress && !g_wsDrawing) { g_wsHidden++; return 0; }
    if (wsHuntActive()) {
        const bool hide = wsHuntDraw(wsSig(t, 0, 0, 0, s, p));
        if (g_wsDrawLock >= 0 && !g_wsMeas[g_wsDrawLock].tried) { g_wsMeas[g_wsDrawLock].tried = true; Logf("WiperHunt: blade %d is drawn without an index buffer and is not measured, using WiperAxis / WiperLenM", g_wsDrawLock + 1); }
        if (hide) { g_wsHiddenWipers++; return 0; }
    }
    return g_origDrawPrim(dev, t, s, p);
}
HRESULT __stdcall hkDrawIdxPrim(void* dev, DWORD t, INT b, UINT mn, UINT nv, UINT si, UINT p) {
    if (g_wsSuppress && !g_wsDrawing) { g_wsHidden++; return 0; }
    if (wsHuntActive()) {
        const bool hide = wsHuntDraw(wsSig(t, (UINT)b, mn, nv, si, p));
        if (g_wsDrawLock >= 0 && g_cfg.wsWiperMeasure) wsMeasureBlade(dev, g_wsDrawLock, b, mn, nv);     // once per locked blade
        if (hide) { g_wsHiddenWipers++; return 0; }
    }
    return g_origDrawIdxPrim(dev, t, b, mn, nv, si, p);
}
HRESULT __stdcall hkDrawPrimUP(void* dev, DWORD t, UINT p, const void* data, UINT stride) { if (g_wsSuppress && !g_wsDrawing) { g_wsHidden++; return 0; } return g_origDrawPrimUP(dev, t, p, data, stride); }
HRESULT __stdcall hkDrawIdxPrimUP(void* dev, DWORD t, UINT mn, UINT nv, UINT p, const void* idx, DWORD fmt, const void* vtx, UINT stride) { if (g_wsSuppress && !g_wsDrawing) { g_wsHidden++; return 0; } return g_origDrawIdxPrimUP(dev, t, mn, nv, p, idx, fmt, vtx, stride); }


HRESULT __stdcall hkCreateDevice(void* self, UINT adapter, UINT devType, HWND hwnd, DWORD flags, void* pp, void** ppDev) {
    const HRESULT hr = g_origCreateDevice(self, adapter, devType, hwnd, flags, pp, ppDev);
    if (SUCCEEDED(hr) && ppDev && *ppDev && !g_devHooked) {
        void** vt = *(void***)(*ppDev);                                // IDirect3DDevice9 vtable: SetTransform = 44, SetVertexShaderConstantF = 94
        const bool a = MH_CreateHook(vt[44], (void*)&hkSetTransform, (void**)&g_origSetTransform) == MH_OK && MH_EnableHook(vt[44]) == MH_OK;
        const bool b = MH_CreateHook(vt[94], (void*)&hkSetVSConst, (void**)&g_origSetVSConst) == MH_OK && MH_EnableHook(vt[94]) == MH_OK;
        g_devHooked = a && b;
        g_wsDev = (IDirect3DDevice9*)*ppDev;
        if (g_cfg.wsEnable) {                                          // Windscreen: EndScene = 42, Reset = 16
            const bool c = MH_CreateHook(vt[42], (void*)&hkEndScene, (void**)&g_origEndScene) == MH_OK && MH_EnableHook(vt[42]) == MH_OK;
            const bool r = MH_CreateHook(vt[16], (void*)&hkReset, (void**)&g_origReset) == MH_OK && MH_EnableHook(vt[16]) == MH_OK;
            g_wsHooked = c && r;
            Note("Windscreen: EndScene hook %s, Reset hook %s", c ? "OK" : "FAILED", r ? "OK" : "FAILED");
            if (g_cfg.wsHideGameFx == 1 || g_cfg.wsWiperSource || g_cfg.wsHideGameWipers) {   // DrawPrimitive = 81, DrawIndexedPrimitive = 82, DrawPrimitiveUP = 83, DrawIndexedPrimitiveUP = 84
                const bool h1 = MH_CreateHook(vt[81], (void*)&hkDrawPrim, (void**)&g_origDrawPrim) == MH_OK && MH_EnableHook(vt[81]) == MH_OK;
                const bool h2 = MH_CreateHook(vt[82], (void*)&hkDrawIdxPrim, (void**)&g_origDrawIdxPrim) == MH_OK && MH_EnableHook(vt[82]) == MH_OK;
                const bool h3 = MH_CreateHook(vt[83], (void*)&hkDrawPrimUP, (void**)&g_origDrawPrimUP) == MH_OK && MH_EnableHook(vt[83]) == MH_OK;
                const bool h4 = MH_CreateHook(vt[84], (void*)&hkDrawIdxPrimUP, (void**)&g_origDrawIdxPrimUP) == MH_OK && MH_EnableHook(vt[84]) == MH_OK;
                Note("Windscreen: draw-call hooks for HideGameFx: %s %s %s %s", h1 ? "OK" : "FAILED", h2 ? "OK" : "FAILED", h3 ? "OK" : "FAILED", h4 ? "OK" : "FAILED");
            }
        }
        Note("D3D device created: SetTransform hook %s, SetVertexShaderConstantF hook %s", a ? "OK" : "FAILED", b ? "OK" : "FAILED");
    }
    return hr;
}

void* WINAPI hkDirect3DCreate9(UINT sdk) {
    void* d3d = g_origD3DCreate9(sdk);
    if (d3d && !g_createDeviceHooked) {
        void** vt = *(void***)d3d;                                     // IDirect3D9 vtable: CreateDevice = 16
        if (MH_CreateHook(vt[16], (void*)&hkCreateDevice, (void**)&g_origCreateDevice) == MH_OK && MH_EnableHook(vt[16]) == MH_OK) {
            g_createDeviceHooked = true; Note("Direct3DCreate9 called: CreateDevice hook installed");
        } else Note("CreateDevice hook FAILED");
    }
    return d3d;
}

bool installD3DHooks() {
    HMODULE d3d = GetModuleHandleA("d3d9.dll");
    if (!d3d) d3d = LoadLibraryA("d3d9.dll");
    if (!d3d) { Note("d3d9.dll not found"); return false; }
    char path[MAX_PATH] = {0}; GetModuleFileNameA(d3d, path, MAX_PATH); g_d3dPath = path;
    Logf("d3d9.dll = %s", path);
    void* f = (void*)GetProcAddress(d3d, "Direct3DCreate9");
    if (!f) { Note("Direct3DCreate9 not found"); return false; }
    if (MH_CreateHook(f, (void*)&hkDirect3DCreate9, (void**)&g_origD3DCreate9) != MH_OK || MH_EnableHook(f) != MH_OK) {
        Note("Direct3DCreate9 hook FAILED (is another plugin, such as openRBRVR, hooking it too?)"); return false;
    }
    Note("Direct3DCreate9 hook installed (waiting for the game to create its device)");
    return true;
}

// The game's wet-windscreen overlay is a screen-space quad drawn by this function (openRBRVR hooks it for the same reason):
// it must not be head-transformed.
using WindscreenFxFn = void(__fastcall*)(void* a);
WindscreenFxFn g_origWindscreenFx = nullptr;
void __fastcall hkWindscreenFx(void* a) {
    g_wsGameCalls++; g_wsGameTick = GetTickCount();
    wsDrawFrom(2, g_wsDev);                                            // DrawAt=2: just before the game would draw its own wet overlay
    const bool hide = g_wsOn.load() && g_wsArmed && g_wsHooked && g_cfg.wsHideGameFx != 0;      // only while our own effect is what is on screen
    if (hide && g_cfg.wsHideGameFx == 2) { wsDrawFrom(1, g_wsDev); return; }                      // 2: the game's function does not run at all
    g_wsSuppress = hide;                                               // 1: it runs (so the wipers keep working) but its draw calls are dropped
    g_inWindscreenFx = true; g_origWindscreenFx(a); g_inWindscreenFx = false;
    g_wsSuppress = false;
    wsDrawFrom(1, g_wsDev);                                            // DrawAt=1: just after it (inside the 3D pass, before the HUD)
}

bool openRBRVRLoaded() { return GetModuleHandleA("openRBRVR.dll") != nullptr; }
// Method 4 hooks Direct3DCreate9 like openRBRVR does, so the two cannot share a game. For VR users (openRBRVR present) and
// for installs where the D3D hooks never become active, fall back to Method 2 (forced camera change).
void fallbackToMethod2(const char* why) {
    Note("AUTO-FALLBACK: %s -> using Method 2 (forced camera change; no head roll; may affect wipers)", why);
    g_cfg.method = 2; g_dirty = false; g_patchActive = false; resetHead();
}

bool ownForeground() {
    HWND w = GetForegroundWindow(); if (!w) return false;
    DWORD pid = 0; GetWindowThreadProcessId(w, &pid); return pid == GetCurrentProcessId();
}
void pollKeys() {
    static bool prevT = false, prevR = false, prevW = false, prevS = false;
    const bool ctrl = !g_cfg.needCtrl || (GetAsyncKeyState(VK_CONTROL) & 0x8000);
    const bool fg = ownForeground();
    const bool dT = fg && ctrl && g_cfg.toggleKey && (GetAsyncKeyState(g_cfg.toggleKey) & 0x8000);
    const bool dR = fg && ctrl && g_cfg.reloadKey && (GetAsyncKeyState(g_cfg.reloadKey) & 0x8000);
    if (dT && !prevT) { g_enabled = !g_enabled.load(); Logf("effect %s (hot key)", g_enabled ? "ON" : "OFF"); }
    if (dR && !prevR) { loadConfig(); Logf("config reloaded (hot key)"); }
    const bool dW = fg && ctrl && g_cfg.wsToggleKey && (GetAsyncKeyState(g_cfg.wsToggleKey) & 0x8000);
    if (dW && !prevW) { g_wsOn = !g_wsOn.load(); Logf("windscreen fx %s (hot key)", g_wsOn ? "ON" : "OFF"); }
    const bool dS = fg && ctrl && g_cfg.wsWiperSyncKey && (GetAsyncKeyState(g_cfg.wsWiperSyncKey) & 0x8000);
    if (dS && !prevS) wsSyncWipers();
    prevT = dT; prevR = dR; prevW = dW; prevS = dS;
}

std::atomic<long> g_frames{0};
DWORD g_lastReport = 0; bool g_loggedFirst = false; int g_lastTestPhase = -1;

void logStatus(DWORD now, uint32_t type, const Drive& d) {
    if (g_cfg.method == 4)
        Logf("render: createDeviceHook=%d deviceHooks=%d P=%d zSign=%d patched/s=%ld head F/R/U mm=%.1f/%.1f/%.1f pitch/yaw/roll deg=%.2f/%.2f/%.2f carRoll raw peak=%.3f",
             g_createDeviceHooked ? 1 : 0, g_devHooked ? 1 : 0, g_pValid ? 1 : 0, g_zSign, g_patched.exchange(0),
             g_hF * 1000.f, g_hR * 1000.f, g_hU * 1000.f, g_hPitchDeg, g_hYawDeg, g_hRollDeg, g_pkCarRoll.exchange(0));
    if (g_cfg.wsEnable) {
        int flakes = 0, micro = 0; for (const WsDrop& d : g_wsDrops) { if (d.kind == 1) ++flakes; else if (d.kind == 2) ++micro; }
        Logf("windscreen: on=%d hooked=%d drawn/s=%ld skippedNotBackbuffer/s=%ld gameWetFnCalls/s=%ld gameDrawsHidden/s=%ld refraction=%d veil=%d | water=%d micro=%d flakes=%d mud=%d viewport=%ux%u | stage %d sky=%d(%s) wet=%d rain=%.2f snow=%.2f mudx=%.2f | wiper=%.0f%% src=%d locked=%d%d hiddenWiperDraws/s=%ld wiped/s=%ld | telemetry: spdN=%.2f rough=%.3f loose=%.2f rain/s=%.1f mud/min=%.1f | film=%.2f dirt=%.2f inflight=%d bow=%.2f/%.2f",
             g_wsOn.load() ? 1 : 0, g_wsHooked ? 1 : 0, g_wsDrawn.exchange(0), g_wsSkipped.exchange(0), g_wsGameCalls.exchange(0), g_wsHidden.exchange(0), g_wsRefrUsed ? 1 : 0, g_wsVeilUsed ? 1 : 0,
             (int)g_wsDrops.size() - flakes - micro, micro, flakes, (int)g_wsMud.size(), g_wsVpW, g_wsVpH, g_wsStageId, g_wsGW.sky, wsSkyName(g_wsGW.sky), g_wsGW.wet, g_wsRainEff, g_wsSnowEff, g_wsMudMul,
             g_wsWiperW * 100.f, g_cfg.wsWiperSource, g_wsLock[0].on ? 1 : 0, g_wsLock[1].on ? 1 : 0, g_wsHiddenWipers.exchange(0), g_wsWiped.exchange(0), g_wsSpdN, g_wsRough, g_wsLoose, g_wsDbgRain, g_wsDbgMud, wsGridMax(g_wsFilm), wsGridMax(g_wsDirt), (int)g_wsFly.size(), g_wsBow[0], g_wsBow[1]);
    }
    Logf("air=%d load=%.2f shake=%.2f", g_airborne.load() ? 1 : 0, g_loadFrac.load(), g_shake.amp);
    Logf("fps=%ld applies/s=%ld natcam/s=%ld type=%u base=%d en=%d | in S/W/H=%.2f/%.2f/%.2f bump=%.2f yawN=%.2f steerN=%.2f spd=%.0f brk=%.2f gear=%d | "
         "raw peak S/W/H=%.2f/%.2f/%.2f yaw=%.2f steer=%.2f | head mm F/R/U=%.1f/%.1f/%.1f ang pitch/yaw=%.2f/%.2f | pkt_age_ms=%u",
         g_frames.exchange(0), g_applies.exchange(0), g_natCalls.exchange(0), type, g_haveBase ? 1 : 0, g_enabled ? 1 : 0,
         d.gS, d.gSw, d.gH, d.gHp, d.yawN, d.steerN, d.speed, d.brake, g_gear.load(),
         g_pkSurge.exchange(0), g_pkSway.exchange(0), g_pkHeave.exchange(0), g_pkYaw.exchange(0), g_pkSteer.exchange(0),
         g_offPos[g_cfg.axFwd] * g_cfg.sgFwd * 1000.f, g_offPos[g_cfg.axRight] * g_cfg.sgRight * 1000.f, g_offPos[g_cfg.axUp] * g_cfg.sgUp * 1000.f,
         g_angPitch * 57.29578f, g_angYaw * 57.29578f, (unsigned)(now - g_lastPacketTick.load()));
}

void __fastcall hkRender(void* p) {
    g_frames++;
    const DWORD now = GetTickCount();
    const uint32_t type = readCameraType();
    pollKeys();

    if (g_cfg.method == 4 && g_cfg.autoFallback) {                      // checked once per second
        static DWORD lastFb = 0; static int noHookSecs = 0;
        if (now - lastFb > 1000) {
            lastFb = now;
            if (openRBRVRLoaded()) fallbackToMethod2("openRBRVR.dll detected");
            else if (isCockpit(type) && !(g_devHooked && g_pValid)) { if (++noHookSecs >= 10) fallbackToMethod2("D3D hooks not active after 10 s in a cockpit camera"); }
            else noHookSecs = 0;
        }
    }

    if (type != g_prevType) {                                           // camera type just changed
        g_prevType = type;
        if (isCockpit(type) && g_pendSeq != g_pendConsumed) {           // the natural call that configured this camera
            g_camObj = g_pendObj; memcpy(g_basePos, g_pendPos, 12); memcpy(g_baseTgt, g_pendTgt, 12);
            g_baseType = type; g_haveBase = true; g_pendConsumed = g_pendSeq; resetHead(); g_haveLast = false;
            Logf("base captured for camera type %u: pos=(%.3f %.3f %.3f) tgt=(%.3f %.3f %.3f)", type,
                 g_basePos[0], g_basePos[1], g_basePos[2], g_baseTgt[0], g_baseTgt[1], g_baseTgt[2]);
        } else { g_haveBase = false; if (g_dirty) { g_dirty = false; resetHead(); g_haveLast = false; } }
    }

    Drive d;                                                            // current inputs
    d.gS = g_in.surge; d.gSw = g_in.sway; d.gH = g_in.heave; d.gHp = g_in.heaveHp;
    d.yawN = g_in.yawN; d.steerN = g_in.steerN; d.speed = g_in.speed; d.brake = g_in.brake; d.carRoll = g_in.carRoll;
    const bool test = g_cfg.test != 0;
    if (test) {
        int ph = 0; const char* msg = testDrive(now, d, ph);
        if (ph != g_lastTestPhase) { g_lastTestPhase = ph; Logf("TEST: %s", msg); }
    }

    if (g_cfg.log && now - g_lastReport > 1000) { logStatus(now, type, d); g_lastReport = now; }

    if (g_cfg.apply && isCockpit(type) && (g_cfg.method != 3 || g_haveBase)) {
        const bool active = g_enabled.load();
        const bool fresh = test || (now - g_lastPacketTick.load()) < 300;   // NGP only sends while actually driving
        LARGE_INTEGER q; QueryPerformanceCounter(&q);
        const float dt = (float)(q.QuadPart - g_qlast.QuadPart) / (float)g_qf.QuadPart; g_qlast = q;

        bool moving = false;
        if (active) {
            if (dt > 0.f && dt < 0.1f) {
                if (!fresh) { d = Drive(); }                           // no telemetry: let the head settle to rest
                stepHead(dt, d);
            } else resetHead();                                         // pause / first frame / camera switch
            const float mag = std::fabs(g_hF) + std::fabs(g_hR) + std::fabs(g_hU) +
                              (std::fabs(g_hPitchDeg) + std::fabs(g_hYawDeg) + std::fabs(g_hRollDeg)) * 0.017453293f;
            moving = mag > 1e-5f;
        } else { g_head.reset(); zeroOffsets(); }

        if (g_cfg.method == 4) {
            // Method 4: no camera calls at all. Hand the head pose to the D3D matrix hooks for this frame.
            g_patchActive = false;
            if (g_devHooked && g_pValid && active && moving) {
                g_Hinv = buildHinv(g_hF * g_cfg.rSgFwd, g_hR * g_cfg.rSgRight, g_hU * g_cfg.rSgUp,
                                   g_hPitchDeg * 0.017453293f * g_cfg.rSgPitch, g_hYawDeg * 0.017453293f * g_cfg.rSgYaw,
                                   g_hRollDeg * 0.017453293f * g_cfg.rSgRoll, g_zSign);
                g_combDirty = true; g_patchActive = true;
                if (!g_loggedFirst) { Logf("first frame with patched matrices (method 4, zSign=%d)", g_zSign); g_loggedFirst = true; }
            }
        } else {
        static LONGLONG lastApplyQ = 0;
        const bool allow = g_cfg.maxApplyHz <= 0 || (q.QuadPart - lastApplyQ) >= g_qf.QuadPart / g_cfg.maxApplyHz;
        if (allow && ((active && (fresh || moving)) || g_dirty)) {
            CamPtrs cp; std::vector<Snap> s0, s1; bool diagNow = false;
            if (g_cfg.diag && g_cfg.method == 2) {
                ++g_diagCount;
                if (g_diagCount >= g_cfg.diagAfter && g_diagCount < g_cfg.diagAfter + 3 && getCamPtrs(p, cp)) {
                    s0 = takeSnaps(cp, g_cfg.diagDeep != 0); s1 = takeSnaps(cp, g_cfg.diagDeep != 0); diagNow = true;   // two back-to-back = noise control
                }
            }
            const bool ok = (g_cfg.method == 2) ? forceCameraChange(p, type) : applyDirect(p);
            if (diagNow && ok) logDiff3(s0, s1, takeSnaps(cp, g_cfg.diagDeep != 0));
            if (ok) {
                lastApplyQ = q.QuadPart;
                g_applies++;
                if (!g_loggedFirst) { Logf("first apply OK (method %d)", g_cfg.method); g_loggedFirst = true; }
                g_dirty = moving;                                       // one last apply after settling restores the base camera
            } else {
                Note("apply FAILED (exception or null pointer): disabling apply");
                g_cfg.apply = 0;
            }
        }
        }   // method 2 / 3
    } else if (g_dirty) {
        g_dirty = false; resetHead();
    }
    g_wsArmed = g_wsOn.load() && g_wsHooked && g_cfg.wsEnable && isCockpit(type);   // Windscreen draws from the EndScene hook
    g_wsFrameNo++; g_wsStamp = now;
    g_in3D = true;
    g_origRender(p);
    g_in3D = false; g_patchActive = false;
}

} // namespace

// The game's current camera type (the cockpit family is 3 to 6, 5 is the interior view), or -1 when it cannot be read: not the RallySimFans
// executable, or the game has no car yet. Read-only, no hooks, so it works whether or not the camera module is on. The audio uses it to know
// whether the sound is heard from inside the car.
inline int CameraType() {
    static const bool rsf = isRsfExe();
    if (!rsf) return -1;
    const uint32_t t = readCameraType();
    return t == 0xFFFFFFFFu || t > 99u ? -1 : (int)t;
}

