// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ================================================================================================ the module
class Camera {
public:
    struct Hooks {
        std::function<void(const std::string&)> log;
        std::function<std::string()> carKey;       // identifies the car in use ("name|id|slot"), empty while unknown
        std::function<bool(int[4])> readIds;       // the material id under each tyre (FL, FR, RL, RR) from the surface scan; false until all four are known
    };

    // Reads the camera sections of ForceFX.ini and installs the hooks. cameraOn / windscreenOn are the switches from ForceFX.ini. The windscreen
    // hooks are only installed when it is on here, so a restart is needed to switch it on later.
    void configure(const std::string& dir, Hooks h, bool cameraOn, bool windscreenOn) {
        if (m_started) return;
        m_started = true;
        g_dir = dir; g_hostLog = h.log; g_hostIds = h.readIds; g_hostCarKey = h.carKey;
        g_enabled = cameraOn; g_wsOn = windscreenOn; g_cfg.wsEnable = windscreenOn ? 1 : 0;
        loadConfig();
        if (g_cfg.log) g_log = fopen((g_dir + "ForceFXCamera.log").c_str(), "w");
        if (!g_cfg.anyExe && !isRsfExe()) {
            Note("not the RallySimFans executable (RichardBurnsRally_SSE.exe), so the camera and windscreen effects stay off. Set AnyExe=1 in the [Camera] section of ForceFX.ini to try anyway");
            return;
        }
        QueryPerformanceFrequency(&g_qf); QueryPerformanceCounter(&g_qlast);
        if (MH_Initialize() != MH_OK) { Note("MH_Initialize failed, the camera effects stay off"); return; }
        m_mh = true;
        void* t1 = (void*)absAddr(0x4663F0);
        void* t2 = (void*)absAddr(0x47E1E0);
        if (MH_CreateHook(t1, (void*)&hkSetCamTarget, (void**)&g_orig) == MH_OK && MH_EnableHook(t1) == MH_OK) {
            m_setCam = t1; Logf("hook set_camera_target installed @ %p", t1);
        } else Note("hook set_camera_target FAILED @ %p", t1);
        if (MH_CreateHook(t2, (void*)&hkRender, (void**)&g_origRender) == MH_OK && MH_EnableHook(t2) == MH_OK) {
            m_render = t2; Logf("hook render installed @ %p", t2);
        } else Note("hook render FAILED @ %p", t2);
        if (g_cfg.method == 4 && g_cfg.autoFallback && openRBRVRLoaded()) fallbackToMethod2("openRBRVR.dll detected at startup");
        const bool wantD3D = (g_cfg.method == 4) || (g_cfg.wsEnable && !openRBRVRLoaded());   // Windscreen also needs the device hooks
        if (wantD3D && installD3DHooks()) Logf("D3D hooks requested (method %d, windscreen enable=%d)", g_cfg.method, g_cfg.wsEnable);
        if (wantD3D) {                                                  // also needed by Windscreen: counts the game's own wet-overlay calls
            void* t3 = (void*)absAddr(0x452020);
            if (MH_CreateHook(t3, (void*)&hkWindscreenFx, (void**)&g_origWindscreenFx) == MH_OK && MH_EnableHook(t3) == MH_OK) {
                m_wind = t3; Logf("hook windscreen effects installed @ %p", t3);
            } else Note("hook windscreen effects FAILED @ %p", t3);
        }
        if (g_rawMethod < 2 || g_rawMethod > 4) Note("Method=%d in the ini is not valid (use 2, 3 or 4); using %d", g_rawMethod, g_cfg.method);
        m_ok = m_setCam && m_render;
        if (m_ok) { g_ready = true; Note("active (method %d, camera %s, windscreen %s)", g_cfg.method, cameraOn ? "on" : "off", windscreenOn ? "on" : "off"); }
        else Note("a hook failed, so the camera effects stay off");
    }

    void setActive(bool on) { g_enabled = on; }
    void setWindscreen(bool on) { g_wsOn = on; }

    // A tyre hit standing water at this strength (0..1): the windscreen throws a splash of water, the same way a landing throws mud.
    // Ignored while the windscreen effects are off, so a stale splash can not appear when they are switched on later.
    void splash(float strength) {
        if (g_ready && g_wsOn.load() && strength > 0.0f) storeMax(g_wsWaterHit, strength < 1.0f ? strength : 1.0f);
    }

    // One NGP telemetry packet. Safe to call from any thread, one packet at a time.
    void feed(const uint8_t* data, int len) {
        if (g_ready && data && len > 0) processPacket((const char*)data, len);
    }
    // Where the packet fields are (the plugin's one layout from [ForceFX]). Call before the first packet; it is not changed while the game runs.
    void setLayout(const Telem::Layout& l) { g_lay = l; }

    // The stage is starting: forget the old head pose and the last stage's events.
    void stageStarted(int iMap, const char* player, bool) {
        if (!g_ready) return;
        const WsGameWeather gw = wsReadGameWeather();
        Logf("StageStarted: map id=%d | game conditions: sky=%d (%s) surfaceWetness=%d cloud=%d timeOfDay=%d valid=%d", iMap, gw.sky, wsSkyName(gw.sky), gw.wet, gw.cloud, gw.tod, gw.ok ? 1 : 0);
        g_wsGW = gw; g_wsStageId = iMap; wsApplyStage();
        g_stageStartTick = GetTickCount();
        {   // Mud and dirt carry on from the last stage of the same rally; a new rally, a new driver or a stage that was not finished starts clean (new car: see wsStep)
            const std::string who = player ? player : "";
            g_wsHist.secondsSinceFinish = g_wsHist.finished ? (float)(GetTickCount() - g_wsFinishTick) / 1000.f : 0.f;
            const char* why = wsl::cleanReason(g_cfg.wsCarryOver != 0, g_wsHist, iMap, who, g_cfg.wsCarryMinutes);
            if (why) { g_wsReset = true; Logf("windscreen: glass cleaned for this stage: %s", why); }
            else { g_wsResetWater = true; Logf("windscreen: mud and dirt carried over from the last stage, water cleared"); }
            g_wsHist.have = true; g_wsHist.finished = false; g_wsHist.map = iMap; g_wsHist.player = who; g_wsCarCheckUntil = GetTickCount() + 12000;
        }
        g_evCrashSurge = 0; g_evCrashSway = 0; g_evLand = 0;
        resetHead(); g_haveBase = false; g_haveLast = false;
    }

    // The player finished the stage (finishTime > 0) or failed it (0). Only a finished stage lets the mud carry on to the next one.
    void stageFinished(float finishTime) {
        if (!g_ready || finishTime <= 0.f) return;
        g_wsHist.finished = true; g_wsFinishTick = GetTickCount();
    }

    // One line for the diagnostics page.
    std::string status() {
        if (!m_started) return std::string();
        char b[256];
        if (!g_ready) { snprintf(b, sizeof(b), "camera: off (see ForceFX.log)"); return b; }
        const unsigned age = (unsigned)(GetTickCount() - g_lastPacketTick.load());
        snprintf(b, sizeof(b), "camera: %s, method %d, %s | windscreen: %s | telemetry %s",
                 g_enabled.load() ? "on" : "off", g_cfg.method,
                 g_cfg.method == 4 ? (g_devHooked && g_pValid ? "D3D hooks active" : "waiting for D3D hooks") : "forced camera change",
                 !g_wsOn.load() ? "off" : (g_wsHooked ? "on" : (g_cfg.wsEnable ? "starting" : "on after a restart")),
                 age < 1000 ? "flowing" : "not seen");
        return b;
    }

    void shutdown() {
        if (!m_started || m_down) return;
        m_down = true; g_ready = false;
        if (m_wind) { MH_DisableHook(m_wind); MH_RemoveHook(m_wind); }
        if (m_render) { MH_DisableHook(m_render); MH_RemoveHook(m_render); }
        if (m_setCam) { MH_DisableHook(m_setCam); MH_RemoveHook(m_setCam); }
        if (m_mh) MH_Uninitialize();
        if (g_log) { fclose(g_log); g_log = nullptr; }
    }

private:
    bool m_started = false, m_down = false, m_ok = false, m_mh = false;
    void* m_setCam = nullptr; void* m_render = nullptr; void* m_wind = nullptr;
};

