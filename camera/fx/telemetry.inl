// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ================================================================================================ telemetry
// Where the packet fields are: the plugin's one telemetry layout (the Offset* keys in [ForceFX]), handed over once at start (ffx::Camera::setLayout).
Telem::Layout g_lay;
constexpr int kMinSuspPacket = 560, kMinPacket = 136;

struct Inputs {                                                       // written by processPacket(), read by the render thread
    std::atomic<float> surge{0}, sway{0}, heave{0}, heaveHp{0}, yawN{0}, steerN{0}, speed{0}, brake{0}, carRoll{0};
} g_in;
std::atomic<float>    g_pkSurge{0}, g_pkSway{0}, g_pkHeave{0}, g_pkYaw{0}, g_pkSteer{0}, g_pkCarRoll{0};
std::atomic<float>    g_wsSplash{0};                                       // landing / impact strength handed to Windscreen (mud splatter)
std::atomic<float>    g_wsWaterHit{0};                                     // a tyre hit standing water, 0..1: Windscreen throws a wash of water at the glass
std::atomic<float>    g_evCrashSurge{0}, g_evCrashSway{0}, g_evLand{0};   // impact events (signed excess g / landing strength), consumed by the render thread
std::atomic<bool>     g_airborne{false};
std::atomic<float>    g_loadFrac{1.f};
std::atomic<uint32_t> g_stageStartTick{0};
std::atomic<float>    g_shiftImpulse{0};                              // +1 per upshift, -1 per downshift (consumed by render)
std::atomic<int>      g_gear{0};
std::atomic<uint32_t> g_lastPacketTick{0};

void storeMax(std::atomic<float>& a, float v) { v = std::fabs(v); float o = a.load(); while (v > o && !a.compare_exchange_weak(o, v)) {} }
void addTo(std::atomic<float>& a, float v) { float o = a.load(); while (!a.compare_exchange_weak(o, o + v)) {} }
float rdf(const char* b, int off) { float f; memcpy(&f, b + off, 4); return f; }
int   rdi(const char* b, int off) { int v; memcpy(&v, b + off, 4); return v; }
float clean(float v) { return (std::isfinite(v) && std::fabs(v) < 1.0e6f) ? v : 0.f; }
float clampf(float v, float m) { return std::max(-m, std::min(m, v)); }
float clamp01(float v) { return std::max(0.f, std::min(1.f, v)); }
float deadZone(float v, float dz) { const float a = std::fabs(v) - dz; return a <= 0.f ? 0.f : (v < 0.f ? -a : a); }   // soft deadzone

// State that lives between packets (the packets arrive one at a time, never from two threads at once: feed() holds a lock).
struct PktState {
    LARGE_INTEGER qf{}, qlast{};
    float fs = 0, fy = 0, fh = 0, slowH = 0, yawEnv = 0, steerEnv = 0, fyaw = 0, fsteer = 0, fcr = 0;
    bool first = true, timing = false; int lastGear = -999;
    double tsec = 0; float loadEnv = 0, lastCrashT = -10.f, airT0 = -1.f; bool airborne = false, airDisabled = false; int badAir = 0;
};
PktState g_ps;
std::mutex g_psMtx;

// One NGP telemetry packet.
void processPacket(const char* buf, int n) {
    if (n < kMinPacket) return;
    std::lock_guard<std::mutex> lock(g_psMtx);
    PktState& P = g_ps;
    LARGE_INTEGER& qf = P.qf; LARGE_INTEGER& qlast = P.qlast;
    if (!P.timing) { QueryPerformanceFrequency(&qf); QueryPerformanceCounter(&qlast); P.timing = true; }
    float& fs = P.fs; float& fy = P.fy; float& fh = P.fh; float& slowH = P.slowH; float& yawEnv = P.yawEnv; float& steerEnv = P.steerEnv;
    float& fyaw = P.fyaw; float& fsteer = P.fsteer; float& fcr = P.fcr; bool& first = P.first; int& lastGear = P.lastGear;
    double& tsec = P.tsec; float& loadEnv = P.loadEnv; float& lastCrashT = P.lastCrashT; float& airT0 = P.airT0;
    bool& airborne = P.airborne; bool& airDisabled = P.airDisabled; int& badAir = P.badAir;
    LARGE_INTEGER qn; QueryPerformanceCounter(&qn);
    float dt = (float)(qn.QuadPart - qlast.QuadPart) / (float)qf.QuadPart; qlast = qn;
    if (!(dt > 0.0005f && dt < 0.25f)) dt = 0.01f;
    tsec += dt;

    const Cfg c = g_cfg;                                           // snapshot (config can be reloaded live)
    const float su = clean(rdf(buf, g_lay.accSurge)), sw = clean(rdf(buf, g_lay.accSway)), he = clean(rdf(buf, g_lay.accHeave));
    const float yr = clean(rdf(buf, g_lay.yawRate)), st = clean(rdf(buf, g_lay.steering));
    const float spd = clean(rdf(buf, g_lay.speed)), brk = clamp01(clean(rdf(buf, g_lay.brake)));
    const int   gear = rdi(buf, g_lay.gear);
    const float cr = clean(rdf(buf, g_lay.carRoll));

    // ---- impact events: crashes (large horizontal acceleration) and jumps / landings (all struts unloaded, then loaded again)
    const bool settled = (GetTickCount() - g_stageStartTick.load()) > 3000;          // ignore the stage-start teleport spikes
    if (c.impactScale > 0.f && settled) {
        const float aS = su * c.invSurge, aW = sw * c.invSway;
        const float ma = std::max(std::fabs(aS), std::fabs(aW));
        if (spd > 3.f && ma > c.crashThr && ma < c.crashIgnore && (tsec - lastCrashT) > c.crashCooldown) {
            lastCrashT = (float)tsec;
            const float eS = std::fabs(aS) > c.crashThr ? (aS > 0 ? 1.f : -1.f) * (std::fabs(aS) - c.crashThr) : 0.f;
            const float eW = std::fabs(aW) > c.crashThr ? (aW > 0 ? 1.f : -1.f) * (std::fabs(aW) - c.crashThr) : 0.f;
            addTo(g_evCrashSurge, eS); addTo(g_evCrashSway, eW);
            Logf("EVENT crash: surge=%.2f sway=%.2f (excess %.2f/%.2f) speed=%.0f", aS, aW, eS, eW, spd);
        }
        if (n >= kMinSuspPacket) {
            float ls = 0.f;
            for (int i = 0; i < 4; ++i) ls += std::fabs(clean(rdf(buf, g_lay.suspBase[i] + g_lay.strutForce)));
            loadEnv = std::max(ls, loadEnv * std::exp(-dt / 30.f));
            const float frac = ls / std::max(loadEnv, 1e-3f);
            g_loadFrac = frac;
            if (airDisabled) {}
            else if (!airborne) {
                if (frac < c.airFrac && spd > 20.f) {
                    if (airT0 < 0.f) airT0 = (float)tsec;
                    if ((tsec - airT0) * 1000.0 >= c.airMs) { airborne = true; Logf("EVENT airborne (load fraction %.3f)", frac); }
                } else airT0 = -1.f;
            } else if (tsec - airT0 > 4.0) {                                   // never lands: the strut signal is probably not a load
                airborne = false; airT0 = -1.f;
                if (++badAir >= 3) { airDisabled = true; Logf("EVENT airborne detection disabled: struts never reloaded (3 timeouts)"); }
            } else if (frac > c.landFrac) {
                const float airTime = (float)tsec - airT0;
                airborne = false; airT0 = -1.f;
                const float S = std::min(c.landMaxS, std::max(airTime / c.airRef, std::fabs(he) / c.landRefG));
                if (S >= c.landMinS) { addTo(g_evLand, S); Logf("EVENT landing: air=%.2fs heave=%.2f strength=%.2f", airTime, he, S); }
            }
        }
    } else { airborne = false; airT0 = -1.f; }
    g_airborne = airborne;

    const float cs = deadZone(clampf(su * c.invSurge, c.accelClamp), c.deadZone),
                cy = deadZone(clampf(sw * c.invSway,  c.accelClamp), c.deadZone),
                ch = deadZone(clampf(he * c.invHeave, c.accelClamp), c.deadZone * 0.5f);
    const float k = 1.f - c.smooth;                                // one-pole low-pass on the accelerations
    if (first) { fs = cs; fy = cy; fh = ch; slowH = ch; first = false; }
    fs += k * (cs - fs); fy += k * (cy - fy); fh += k * (ch - fh);
    slowH += (1.f - std::exp(-dt / c.bumpTau)) * (ch - slowH);     // slow heave; (raw - slow) = bump (high-pass)
    const float hp = deadZone(ch - slowH, c.bumpThreshold);        // only real bumps, not road noise

    // yaw rate / steering: units are undocumented -> normalise by a slowly decaying peak envelope
    const float decay = std::exp(-dt / 60.f);
    yawEnv = std::max(std::fabs(yr), yawEnv * decay);       steerEnv = std::max(std::fabs(st), steerEnv * decay);
    const float yn = clampf(yr * c.invYaw / std::max(yawEnv, c.yawFloor), 1.f);
    const float sn = clampf(st * c.invSteer / std::max(steerEnv, c.steerFloor), 1.f);
    fyaw += k * (yn - fyaw); fsteer += k * (sn - fsteer); fcr += k * (cr - fcr);

    if (lastGear != -999 && gear != lastGear && gear >= 1 && lastGear >= 1 && gear <= 9 && lastGear <= 9)
        addTo(g_shiftImpulse, gear > lastGear ? 1.f : -1.f);
    lastGear = gear; g_gear = gear;

    g_in.surge = fs; g_in.sway = fy; g_in.heave = fh; g_in.heaveHp = hp;
    g_in.yawN = fyaw; g_in.steerN = fsteer; g_in.speed = spd; g_in.brake = brk; g_in.carRoll = fcr;
    storeMax(g_pkSurge, su); storeMax(g_pkSway, sw); storeMax(g_pkHeave, he); storeMax(g_pkYaw, yr); storeMax(g_pkSteer, st); storeMax(g_pkCarRoll, cr);
    g_lastPacketTick = GetTickCount();
}

