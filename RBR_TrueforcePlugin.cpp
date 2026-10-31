// RBR_TrueforcePlugin.cpp  (v5: safety limits, live gain, directional granular road, weight/load model, per-car profiles)
//
// Layers extra "Trueforce-like" vibration on top of Richard Burns Rally's own force feedback,
// driven by NGP's UDP telemetry (rbr.telemetry.data.TelemetryData.h, NGP 7.4.778, 664 bytes).
//
// HOW IT REACHES THE WHEEL
//   RBR/NGP already hold the wheel exclusively, so a second device object could never create effects.
//   We patch the IDirectInputDevice8 vtable (Acquire / Unacquire / GetDeviceState / GetDeviceData / Poll),
//   capture RBR's own device object (matched by instance GUID) and add our effects to it. The wheel sums
//   them with RBR's own force.
//
// EFFECT LAYERS (4 DirectInput effects, all zero-mean vibration: they never push the wheel to one side)
//   ENGINE   sine, firing frequency compressed into the wheel's usable band (monotonic, no pitch jumps).
//            Amplitude is shaped by RPM (idle lump, resonance bumps, high-rev buzz), throttle load and a
//            frequency compensation. Rev limiter stutters.
//   LUMP     second, low sine (half order) = rough idle / beating against the main engine tone.
//   ROAD     sawtooth with random frequency + amplitude jitter. Driven by the roughness of the FRONT tyres
//            (high-passed strut force / mean load, as in RbrSurfaceAudio) and damper piston velocity.
//            Symmetric vibration only: RBR's own steering force is never overridden.
//            Smooth tarmac = nothing, gravel = grit, rough / dirt / kerbs = violent and sharp.
//   BRAKE    sine: front-end judder under braking, grows with brake pedal and front load transfer (dive);
//            front wheel lock-up (from the tyre segment counter vs car speed) makes it harsh.
//
// BUILD: 32-bit (RBR is x86), static CRT:
//   cl /std:c++17 /EHsc /LD /MT /O2 RBR_TrueforcePlugin.cpp /Fe:RbrTrueforce.dll /link /DEF:RbrTrueforce.def
// NGP: udpTelemetry=1, udpTelemetryEndpoints=127.0.0.1:6776,127.0.0.1:6777,127.0.0.1:6778
// LIVE GAIN (while driving): Ctrl + Numpad+ / Numpad- change the overall vibration level in equal-loudness steps (x or / 0.85),
//   Ctrl + Numpad* mutes / unmutes. Only active while RBR is the foreground window. The level can only go DOWN
//   from the configured ceilings (0..100%), never above them. Keys are configurable in the .ini.
// SAFETY: hard compile-time ceilings (kHardTotalMax etc.), frequency floor, slow thermal derate,
//   loopback-only UDP, fade-to-zero on shutdown, exception guard on the worker thread.
// WEIGHT: per-wheel strut load relative to the car's own static load drives road strength (loaded side transmits more),
//   a compression/landing thump, quiet when airborne, and braking judder. TrueforceCars.ini gives each car index its own
//   engine character (Preset=boxer4 ...) and WeightFeel.
// FILES (next to the DLL): TrueforceRumble.ini (shipped defaults; written on first run if missing), TrueforceCars.ini
//   (per-car profiles), .log, _loaded.txt, _packet.txt

#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define DIRECTINPUT_VERSION 0x0800
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <dinput.h>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <algorithm>

#define __SUPPORT_PLUGIN__ 1
#include "IRBRGame.h"
#include "IPlugin.h"

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")

namespace {

typedef std::chrono::steady_clock Clock;

// Hard ceilings: the .ini can never push past these, whatever it says.
constexpr int   kHardTotalMax = 5000;    // sum of all layer magnitudes (0..10000 DirectInput units)
constexpr float kHardMinHz    = 12.0f;   // below this a vibration becomes a visible wheel swing
constexpr float kHardMaxHz    = 120.0f;
constexpr int   kMagDeadband  = 60;      // skip USB updates for changes smaller than this
constexpr float kApplyPeriod  = 0.010f;  // seconds between effect updates (~100 Hz)

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float clamp01(float v) { return clampf(v, 0.0f, 1.0f); }
inline float smooth01(float e0, float e1, float x) { float t = clamp01((x - e0) / (e1 - e0)); return t * t * (3.0f - 2.0f * t); }
inline float gauss(float x, float c, float w) { float d = (x - c) / w; return std::exp(-d * d); }
inline float alphaFor(float dt, float tau) { return 1.0f - std::exp(-dt / tau); }   // EMA coefficient
inline float hpCoef(float dt, float tau) { return tau / (tau + dt); }               // 1st order high-pass
inline float rnd() { return (rand() % 2001 - 1000) / 1000.0f; }                    // -1..1

// ---------------------------------------------------------------- paths/log
std::string g_dir;

std::string pluginDirectory() {
    HMODULE h = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       (LPCSTR)&pluginDirectory, &h);
    char path[MAX_PATH] = {0};
    GetModuleFileNameA(h, path, MAX_PATH);
    std::string p(path);
    size_t slash = p.find_last_of("\\/");
    return slash == std::string::npos ? std::string(".\\") : p.substr(0, slash + 1);
}

void Logf(const char* fmt, ...) {
    char b[512];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    FILE* f = fopen((g_dir + "TrueforceRumble.log").c_str(), "a");
    if (f) { fprintf(f, "%s\n", b); fclose(f); }
}

// ------------------------------------------------------------------- config
struct Config {
    int   udpPort = 6778;            // 6776 = SimHub, 6777 = RbrSurfaceAudio. Never share a port.
    int   minPacket = 664;
    int   cylinders = 4;             // engine pulses per rev = cylinders / 2

    // per-layer ceilings (DirectInput units 0..10000), a fixed total ceiling, and one live level (userGain)
    int   engineMaxMag = 1200, roadMaxMag = 3000, brakeMaxMag = 3000;
    int   totalMaxMag = 4000;        // all layers together are scaled down to this (hard ceiling kHardTotalMax)
    float sustainedLimit = 0.10f;    // thermal derate: long-term mean-square level (0.10 ~ 32% RMS-equivalent)

    // engine
    float engineMinHz = 14.0f, engineMaxHz = 100.0f;
    float rpmMax = 0.0f;             // 0 = learn from the data (starts at RpmMaxInit)
    float rpmMaxInit = 7500.0f;
    float engineGrit = 0.5f;         // 0 = clean sine at high revs, 1 = rough, beating, irregular

    // road
    float roadMinHz = 30.0f, roadMaxHz = 95.0f;
    float roughLow = 0.08f, roughHigh = 0.60f, roughGamma = 1.4f;   // tyre roughness -> 0..1 level (tarmac below roughLow)
    float pistonScale = 1.0f;        // RMS damper piston velocity -> 0..1 (absolute hits, kerbs)
    float bumpScale = 0.1f;          // heave acceleration spike -> 0..1
    float roadGrain = 0.6f;          // 0 = smooth buzz, 1 = coarse, irregular grain (amplitude + frequency randomness)
    float roadDirectional = 0.7f;    // 0 = both sides identical, 1 = left/right impacts fully separate
    int   roadSideInvert = 0;        // 1 = swap which side pushes which way (wheel/driver polarity differs)
    float maxSpeed = 40.0f;          // m/s that counts as full speed

    // weight / suspension load (all relative to the car's own static load, so units never matter)
    float loadInfluence = 0.6f;      // 0 = ignore tyre load, 1 = strongly favour the more heavily loaded side
    float thumpGain = 0.8f;          // compression / landing thump strength
    float airborneFade = 1.0f;       // 1 = road and engine go quiet when the wheels are off the ground

    // per-car character (the values below are the globals; a car profile can override them)
    float weightFeel = 1.0f;         // >1 heavier: stronger thump, slower decay, lower road pitch
    float lumpScale = 1.0f;          // idle lump / beat strength
    float unevenness = 0.0f;         // uneven firing (boxer, five-cylinder, V8 cross-plane)
    int   carProfiles = 1;           // use TrueforceCars.ini
    int   offCarIndex = 56;          // NGP Car.Index

    // braking
    float tyreCircumference = 1.95f; // m, until the plugin has learned it while coasting
    float lockThreshold = 0.12f;     // relative wheel speed deficit where lock-up starts

    // NGP packet offsets (float32 unless noted)
    int   offThrottle = 28, offBrake = 32, offSpeed = 60, offAccHeave = 120, offRpm = 136;
    int   offSuspBase = 152, suspStride = 128, offStrut = 16, offPiston = 28, offSegment = 60;   // segment = uint32
    int   speedIsKph = 0;
    float rpmScale = 1.0f;

    // live gain (hotkeys, polled while RBR is the foreground window). VK codes, 0 = disabled.
    float userGain = 1.0f;           // persisted live level 0..1 (can only reduce)
    float gainRatio = 0.85f;         // each hotkey press multiplies/divides the level by this (~ -1.4 dB per step)
    int   keyGainUp = 0x6B, keyGainDown = 0x6D, keyMute = 0x6A;   // Numpad + / - / *
    int   keyModifier = 0;                                        // 0 = no modifier (0x11 = Ctrl)
    int   gainBeep = 1;

    int         autoConnect = 1;
    std::string wheel = "SIMAGIC Alpha EVO Wheelbase";   // default from the shipped ini; the menu overwrites it
};

void SyncConfig(Config& c, const std::string& path, bool write) {
    const char* S = "Trueforce";
    auto F = [&](const char* k, float& v) {
        char b[64];
        if (write) { snprintf(b, sizeof(b), "%g", v); WritePrivateProfileStringA(S, k, b, path.c_str()); }
        else { GetPrivateProfileStringA(S, k, "", b, sizeof(b), path.c_str()); if (b[0]) v = (float)atof(b); }
    };
    auto I = [&](const char* k, int& v) {
        char b[64];
        if (write) { snprintf(b, sizeof(b), "%d", v); WritePrivateProfileStringA(S, k, b, path.c_str()); }
        else { GetPrivateProfileStringA(S, k, "", b, sizeof(b), path.c_str()); if (b[0]) v = atoi(b); }
    };
    I("UdpPort", c.udpPort);  I("MinPacketBytes", c.minPacket);  I("Cylinders", c.cylinders);
    I("EngineMaxMagnitude", c.engineMaxMag); I("RoadMaxMagnitude", c.roadMaxMag);
    I("BrakeMaxMagnitude", c.brakeMaxMag); I("TotalMaxMagnitude", c.totalMaxMag);
    F("EngineMinHz", c.engineMinHz); F("EngineMaxHz", c.engineMaxHz);
    F("RpmMax", c.rpmMax); F("RpmMaxInit", c.rpmMaxInit);
    F("RoadMinHz", c.roadMinHz); F("RoadMaxHz", c.roadMaxHz);
    F("RoughLow", c.roughLow); F("RoughHigh", c.roughHigh); F("RoughGamma", c.roughGamma);
    F("RoadGrain", c.roadGrain); F("RoadDirectional", c.roadDirectional); I("RoadSideInvert", c.roadSideInvert);
    F("EngineGrit", c.engineGrit);
    F("LoadInfluence", c.loadInfluence); F("ThumpGain", c.thumpGain); F("AirborneFade", c.airborneFade);
    F("WeightFeel", c.weightFeel); F("LumpScale", c.lumpScale); F("Unevenness", c.unevenness);
    I("CarProfiles", c.carProfiles); I("OffsetCarIndex", c.offCarIndex);
    F("PistonScale", c.pistonScale); F("BumpScale", c.bumpScale); F("MaxSpeed", c.maxSpeed);
    F("TyreCircumference", c.tyreCircumference); F("LockThreshold", c.lockThreshold);
    I("OffsetThrottle", c.offThrottle); I("OffsetBrake", c.offBrake); I("OffsetSpeed", c.offSpeed);
    I("OffsetAccHeave", c.offAccHeave); I("OffsetRpm", c.offRpm);
    I("OffsetSuspBase", c.offSuspBase); I("SuspStride", c.suspStride); I("OffsetStrutForce", c.offStrut);
    I("OffsetPistonVel", c.offPiston); I("OffsetTyreSegment", c.offSegment);
    I("SpeedIsKph", c.speedIsKph); F("RpmScale", c.rpmScale);
    F("SustainedLimit", c.sustainedLimit);
    F("UserGain", c.userGain); F("GainRatio", c.gainRatio);
    I("KeyGainUp", c.keyGainUp); I("KeyGainDown", c.keyGainDown); I("KeyMute", c.keyMute);
    I("KeyModifier", c.keyModifier); I("GainBeep", c.gainBeep);
    I("AutoConnect", c.autoConnect);
    char b[260];
    if (write) WritePrivateProfileStringA(S, "Wheel", c.wheel.c_str(), path.c_str());
    else { GetPrivateProfileStringA(S, "Wheel", "", b, sizeof(b), path.c_str()); c.wheel = b; }

    if (!write) {
        c.cylinders = std::max(1, std::min(c.cylinders, 16));
        c.engineMaxMag = std::max(0, std::min(c.engineMaxMag, 10000));
        c.roadMaxMag = std::max(0, std::min(c.roadMaxMag, 10000));
        c.brakeMaxMag = std::max(0, std::min(c.brakeMaxMag, 10000));
        c.totalMaxMag = std::max(500, std::min(c.totalMaxMag, kHardTotalMax));
        c.userGain   = clampf(c.userGain,   0.0f, 1.0f);
        c.roadGrain = clampf(c.roadGrain, 0.0f, 1.0f);
        c.roadDirectional = clampf(c.roadDirectional, 0.0f, 1.0f);
        c.engineGrit = clampf(c.engineGrit, 0.0f, 1.0f);
        c.loadInfluence = clampf(c.loadInfluence, 0.0f, 1.0f);
        c.thumpGain = clampf(c.thumpGain, 0.0f, 2.0f);
        c.airborneFade = clampf(c.airborneFade, 0.0f, 1.0f);
        c.weightFeel = clampf(c.weightFeel, 0.5f, 2.0f);
        c.lumpScale = clampf(c.lumpScale, 0.0f, 2.0f);
        c.unevenness = clampf(c.unevenness, 0.0f, 1.0f);
        c.gainRatio  = clampf(c.gainRatio, 0.5f, 0.95f);
        c.sustainedLimit = clampf(c.sustainedLimit, 0.02f, 0.20f);
        c.engineMinHz = clampf(c.engineMinHz, kHardMinHz, kHardMaxHz);
        c.engineMaxHz = clampf(c.engineMaxHz, kHardMinHz, kHardMaxHz);
        c.roadMinHz = clampf(c.roadMinHz, 20.0f, 150.0f);
        c.roadMaxHz = clampf(c.roadMaxHz, c.roadMinHz + 5.0f, 150.0f);
        if (c.engineMaxHz < c.engineMinHz + 5.0f) c.engineMaxHz = c.engineMinHz + 5.0f;
        if (c.roughHigh < c.roughLow + 0.05f) c.roughHigh = c.roughLow + 0.05f;
        if (c.maxSpeed < 1.0f) c.maxSpeed = 1.0f;
        if (c.tyreCircumference < 0.5f) c.tyreCircumference = 1.95f;
    }
}

// ------------------------------------------------------------ per-car profiles
// TrueforceCars.ini, one section per NGP Car.Index:  [Car3]  Name=...  Preset=boxer4  Cylinders=4  RpmMax=7800 ...
// Anything a profile does not set falls back to the [Trueforce] values. Presets give an engine character in one word.
struct CarProfile {
    bool found = false;
    std::string name, preset;
    int   cylinders = 0;             // 0 = keep global
    float rpmMax = 0;                // 0 = keep global / learn
    float engineScale = 1.0f, roadScale = 1.0f;
    float lumpScale = -1, grit = -1, unevenness = -1, weightFeel = -1;   // negative = keep global
};

struct Preset { const char* name; int cyl; float lump, grit, uneven, engine; };
const Preset kPresets[] = {
    { "inline4",      4, 1.00f, 0.50f, 0.00f, 1.00f },   // plain four
    { "inline4turbo", 4, 0.80f, 0.65f, 0.05f, 1.10f },   // turbo four: rougher, less lumpy
    { "boxer4",       4, 1.50f, 0.45f, 0.45f, 1.10f },   // unequal-length headers: burble and sway at low revs
    { "inline5",      5, 1.20f, 0.50f, 0.30f, 1.05f },   // five-cylinder: uneven, gravelly
    { "v6",           6, 0.60f, 0.30f, 0.10f, 0.95f },
    { "flat6",        6, 1.00f, 0.40f, 0.20f, 1.00f },
    { "v8",           8, 0.70f, 0.30f, 0.25f, 1.00f },
};

std::string CarsIniPath() { return g_dir + "TrueforceCars.ini"; }

std::string LowerStr(std::string v) { for (char& ch : v) ch = (char)tolower((unsigned char)ch); return v; }

bool LoadCarProfile(int idx, CarProfile& p) {
    char sec[32]; snprintf(sec, sizeof(sec), "Car%d", idx);
    const std::string path = CarsIniPath();
    char raw[1024];
    if (GetPrivateProfileSectionA(sec, raw, sizeof(raw), path.c_str()) == 0) return false;
    p.found = true;
    char b[128];
    auto GS = [&](const char* k, std::string& v) { GetPrivateProfileStringA(sec, k, "", b, sizeof(b), path.c_str()); if (b[0]) v = b; };
    auto GF = [&](const char* k, float& v) { GetPrivateProfileStringA(sec, k, "", b, sizeof(b), path.c_str()); if (b[0]) v = (float)atof(b); };
    GS("Name", p.name); GS("Preset", p.preset);
    // preset first, explicit keys override it
    for (const Preset& pr : kPresets) if (LowerStr(p.preset) == pr.name) {
        p.cylinders = pr.cyl; p.lumpScale = pr.lump; p.grit = pr.grit; p.unevenness = pr.uneven; p.engineScale = pr.engine;
    }
    GetPrivateProfileStringA(sec, "Cylinders", "", b, sizeof(b), path.c_str()); if (b[0]) p.cylinders = atoi(b);
    GF("RpmMax", p.rpmMax); GF("EngineScale", p.engineScale); GF("RoadScale", p.roadScale);
    GF("LumpScale", p.lumpScale); GF("Grit", p.grit); GF("Unevenness", p.unevenness); GF("WeightFeel", p.weightFeel);
    return true;
}

void ApplyProfile(const Config& base, const CarProfile& p, Config& e) {
    e = base;
    if (!p.found) return;
    if (p.cylinders > 0) e.cylinders = std::max(1, std::min(p.cylinders, 16));
    if (p.rpmMax > 500.0f) e.rpmMax = p.rpmMax;
    e.engineMaxMag = std::max(0, std::min((int)(base.engineMaxMag * clampf(p.engineScale, 0.0f, 2.0f)), 10000));
    e.roadMaxMag   = std::max(0, std::min((int)(base.roadMaxMag * clampf(p.roadScale, 0.0f, 2.0f)), 10000));
    if (p.lumpScale >= 0)   e.lumpScale  = clampf(p.lumpScale, 0.0f, 2.0f);
    if (p.grit >= 0)        e.engineGrit = clampf(p.grit, 0.0f, 1.0f);
    if (p.unevenness >= 0)  e.unevenness = clampf(p.unevenness, 0.0f, 1.0f);
    if (p.weightFeel > 0)   e.weightFeel = clampf(p.weightFeel, 0.5f, 2.0f);
}

// -------------------------------------------------- tap on RBR's own device
// IDirectInputDevice8 vtable slots
enum { VT_ACQUIRE = 7, VT_UNACQUIRE = 8, VT_GETDEVICESTATE = 9, VT_GETDEVICEDATA = 10, VT_POLL = 25 };

typedef HRESULT (WINAPI *PFN_Acquire)(IDirectInputDevice8A*);
typedef HRESULT (WINAPI *PFN_GetState)(IDirectInputDevice8A*, DWORD, LPVOID);
typedef HRESULT (WINAPI *PFN_GetData)(IDirectInputDevice8A*, DWORD, LPDIDEVICEOBJECTDATA, LPDWORD, DWORD);
typedef HRESULT (WINAPI *PFN_Poll)(IDirectInputDevice8A*);

bool GetInstanceGuid(IDirectInputDevice8A* d, bool wide, GUID& g) {
    if (wide) {
        DIDEVICEINSTANCEW di{}; di.dwSize = sizeof(di);
        if (FAILED(reinterpret_cast<IDirectInputDevice8W*>(d)->GetDeviceInfo(&di))) return false;
        g = di.guidInstance;
    } else {
        DIDEVICEINSTANCEA di{}; di.dwSize = sizeof(di);
        if (FAILED(d->GetDeviceInfo(&di))) return false;
        g = di.guidInstance;
    }
    return true;
}

class GameDeviceTap {
public:
    static GameDeviceTap& I() { static GameDeviceTap t; return t; }

    bool Install(LPDIRECTINPUT8A diA, const GUID& anyDeviceGuid);   // defined below Tap<>
    void Remove();                                                    // defined below Tap<>

    // Start looking for the game's device object with this instance GUID.
    void Arm(const GUID& g) {
        ReleaseCaptured();
        std::lock_guard<std::mutex> lk(m_mtx);
        m_target = g; m_seen.clear(); m_reacquired = false; m_drop = false; m_armed = true;
    }
    void Disarm() { m_armed = false; ReleaseCaptured(); }

    IDirectInputDevice8A* Captured() const { return m_captured.load(); }

    bool ConsumeReacquired() { return m_reacquired.exchange(false); }
    bool ConsumeDropRequest() { return m_drop.exchange(false); }

    void ReleaseCaptured() {
        IDirectInputDevice8A* d = m_captured.exchange(nullptr);
        if (d) d->Release();
    }

    // called from hooks (game thread)
    void OnCall(IDirectInputDevice8A* self, bool wide) {
        if (!m_armed.load() || m_captured.load()) return;
        GUID target;
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            if (std::find(m_seen.begin(), m_seen.end(), self) != m_seen.end()) return;
            if (m_seen.size() < 64) m_seen.push_back(self);
            target = m_target;
        }
        GUID g;
        if (!GetInstanceGuid(self, wide, g)) return;
        if (IsEqualGUID(g, target)) {
            self->AddRef();
            m_captured = self;
            Logf("Captured game's DirectInput device %p (%s)", (void*)self, wide ? "unicode" : "ansi");
        }
    }
    void OnAcquire(IDirectInputDevice8A* self) {
        if (self == m_captured.load()) { m_reacquired = true; return; }
        // A device that was rejected/unacquired earlier gets evaluated again after a fresh acquire.
        std::lock_guard<std::mutex> lk(m_mtx);
        m_seen.erase(std::remove(m_seen.begin(), m_seen.end(), self), m_seen.end());
    }
    // The game gave the wheel up (menu change, device reset). Our effects died with that acquire, and if we kept
    // holding a reference the game's next exclusive acquire could be blocked, so let the worker thread let go.
    void OnUnacquire(IDirectInputDevice8A* self) {
        if (self == m_captured.load()) m_drop = true;
    }

private:
    std::atomic<bool> m_armed{false};
    std::atomic<bool> m_reacquired{false};
    std::atomic<bool> m_drop{false};
    std::atomic<IDirectInputDevice8A*> m_captured{nullptr};
    std::mutex m_mtx;
    GUID m_target{};
    std::vector<IDirectInputDevice8A*> m_seen;
    bool m_installed = false;
    friend struct TapAccess;
    bool m_didA = false, m_didW = false;
};

template <int V>
struct Tap {
    static void** vtbl;
    static void*  orig[5];

    static HRESULT WINAPI HAcquire(IDirectInputDevice8A* self) {
        HRESULT hr = ((PFN_Acquire)orig[0])(self);
        if (SUCCEEDED(hr)) GameDeviceTap::I().OnAcquire(self);
        return hr;
    }
    static HRESULT WINAPI HUnacquire(IDirectInputDevice8A* self) {
        GameDeviceTap::I().OnUnacquire(self);
        return ((PFN_Poll)orig[4])(self);
    }
    static HRESULT WINAPI HGetState(IDirectInputDevice8A* self, DWORD cb, LPVOID p) {
        GameDeviceTap::I().OnCall(self, V == 1);
        return ((PFN_GetState)orig[1])(self, cb, p);
    }
    static HRESULT WINAPI HGetData(IDirectInputDevice8A* self, DWORD cb, LPDIDEVICEOBJECTDATA d, LPDWORD n, DWORD f) {
        GameDeviceTap::I().OnCall(self, V == 1);
        return ((PFN_GetData)orig[2])(self, cb, d, n, f);
    }
    static HRESULT WINAPI HPoll(IDirectInputDevice8A* self) {
        GameDeviceTap::I().OnCall(self, V == 1);
        return ((PFN_Poll)orig[3])(self);
    }

    static bool PatchSlot(int idx, void* hook, void** origOut) {
        DWORD old;
        if (!VirtualProtect(&vtbl[idx], sizeof(void*), PAGE_READWRITE, &old)) return false;
        *origOut = vtbl[idx];
        vtbl[idx] = hook;
        VirtualProtect(&vtbl[idx], sizeof(void*), old, &old);
        return true;
    }
    static bool Patch(void** vt) {
        if (vtbl == vt) return true;           // already patched
        if (vtbl) return false;
        vtbl = vt;
        bool ok = PatchSlot(VT_ACQUIRE,        (void*)&HAcquire,  &orig[0]) &&
                  PatchSlot(VT_GETDEVICESTATE, (void*)&HGetState, &orig[1]) &&
                  PatchSlot(VT_GETDEVICEDATA,  (void*)&HGetData,  &orig[2]) &&
                  PatchSlot(VT_POLL,           (void*)&HPoll,     &orig[3]) &&
                  PatchSlot(VT_UNACQUIRE,      (void*)&HUnacquire,&orig[4]);
        if (!ok) vtbl = nullptr;
        return ok;
    }
    // Returns false if another tool hooked on top of ours (we must then stay loaded).
    static bool Unpatch() {
        if (!vtbl) return true;
        const int idx[5] = { VT_ACQUIRE, VT_GETDEVICESTATE, VT_GETDEVICEDATA, VT_POLL, VT_UNACQUIRE };
        void* const mine[5] = { (void*)&HAcquire, (void*)&HGetState, (void*)&HGetData, (void*)&HPoll, (void*)&HUnacquire };
        bool clean = true;
        for (int i = 0; i < 5; ++i) {
            if (vtbl[idx[i]] != mine[i]) { clean = false; continue; }   // not ours any more: leave it alone
            DWORD old;
            if (orig[i] && VirtualProtect(&vtbl[idx[i]], sizeof(void*), PAGE_READWRITE, &old)) {
                vtbl[idx[i]] = orig[i];
                VirtualProtect(&vtbl[idx[i]], sizeof(void*), old, &old);
            }
        }
        if (clean) vtbl = nullptr;
        return clean;
    }
};
template <int V> void** Tap<V>::vtbl = nullptr;
template <int V> void*  Tap<V>::orig[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};

bool GameDeviceTap::Install(LPDIRECTINPUT8A diA, const GUID& g) {
    if (m_installed) return true;
    bool ok = false;

    IDirectInputDevice8A* a = nullptr;
    if (diA && SUCCEEDED(diA->CreateDevice(g, &a, NULL)) && a) {
        ok |= Tap<0>::Patch(*reinterpret_cast<void***>(a));
        a->Release();
    }
    // RBR might use the unicode interface; its vtable is a different table.
    LPDIRECTINPUT8W diW = nullptr;
    if (SUCCEEDED(DirectInput8Create(GetModuleHandle(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8W,
                                     (VOID**)&diW, NULL)) && diW) {
        IDirectInputDevice8W* w = nullptr;
        if (SUCCEEDED(diW->CreateDevice(g, &w, NULL)) && w) {
            ok |= Tap<1>::Patch(*reinterpret_cast<void***>(w));
            w->Release();
        }
        diW->Release();
    }
    m_installed = ok;
    Logf("Hook install: %s", ok ? "ok" : "FAILED");
    return ok;
}

void GameDeviceTap::Remove() {
    m_armed = false;
    ReleaseCaptured();
    bool clean = Tap<0>::Unpatch();
    clean = Tap<1>::Unpatch() && clean;
    if (!clean) {
        // Somebody chained their hook after ours; unloading would leave a pointer into freed code. Pin the DLL.
        HMODULE h = nullptr;
        GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN, (LPCSTR)&pluginDirectory, &h);
        Logf("Hook chain detected on exit: DLL pinned in memory");
    }
    m_installed = false;
}

// ----------------------------------------------------------- effect layer
BOOL CALLBACK FindFFAxisCb(const DIDEVICEOBJECTINSTANCEA* o, VOID* ref) {
    // Reads only fields that sit before the name strings: safe for the ANSI and unicode variants.
    if (o->dwFlags & DIDOI_FFACTUATOR) {
        *static_cast<DWORD*>(ref) = o->dwType;
        return DIENUM_STOP;
    }
    return DIENUM_CONTINUE;
}

enum Slot { S_ENG1 = 0, S_ENG2, S_ROAD, S_ROADR, S_BRAKE, S_COUNT };   // S_ROAD = sawtooth-up (sharp edge toward negative/left), S_ROADR = sawtooth-down
const char* const kSlotName[S_COUNT] = {"engine", "lump", "road-A", "road-B", "brake"};

class FFLayer {
public:
    ~FFLayer() { Destroy(); }      // also runs if the worker thread unwinds from an exception
    bool Ready() const {
        for (int i = 0; i < S_COUNT; ++i) if (m_fx[i]) return true;
        return false;
    }

    HRESULT Create(IDirectInputDevice8A* dev) {
        Destroy();
        m_dev = dev;

        DWORD axisId = 0xFFFFFFFF;
        dev->EnumObjects(FindFFAxisCb, &axisId, DIDFT_AXIS);
        if (axisId == 0xFFFFFFFF) return E_NOTIMPL;      // no force feedback actuator
        m_axisId = axisId;

        const GUID* guids[S_COUNT] = { &GUID_Sine, &GUID_Sine, &GUID_SawtoothUp, &GUID_SawtoothDown, &GUID_Sine };
        HRESULT firstErr = DI_OK;
        int ok = 0;
        for (int i = 0; i < S_COUNT; ++i) {
            HRESULT hr = Make(i, *guids[i]);
            if (SUCCEEDED(hr)) { ++ok; m_fx[i]->Start(1, 0); }
            else {
                Logf("CreateEffect(%s) failed: 0x%08X", kSlotName[i], (unsigned)hr);
                if (SUCCEEDED(firstErr)) firstErr = hr;
            }
            m_lastMag[i] = -100000; m_lastPer[i] = 0;
        }
        m_fail = 0;
        if (!ok) { Destroy(); return firstErr; }
        Logf("Effects created: %d of %d", ok, (int)S_COUNT);
        return DI_OK;
    }

    void Destroy() {
        for (int i = 0; i < S_COUNT; ++i)
            if (m_fx[i]) { m_fx[i]->Stop(); m_fx[i]->Release(); m_fx[i] = nullptr; }
        m_dev = nullptr;
    }

    // RBR re-acquired the device: make sure everything is playing again.
    void Restart() {
        for (int i = 0; i < S_COUNT; ++i) if (m_fx[i]) { m_fx[i]->Start(1, 0); m_lastMag[i] = -100000; }
    }

    // mag: 0..10000. per: period in microseconds.
    bool Push(const int mag[S_COUNT], const DWORD per[S_COUNT]) {
        bool bad = false;
        for (int i = 0; i < S_COUNT; ++i) {
            if (!m_fx[i]) continue;
            HRESULT hr = Update(i, mag[i], per[i]);
            if (FAILED(hr)) { bad = true; m_lastHr = hr; }
        }
        if (bad) {
            for (int i = 0; i < S_COUNT; ++i) if (m_fx[i]) { m_fx[i]->Start(1, 0); m_lastMag[i] = -100000; }
            return ++m_fail < 100;
        }
        m_fail = 0;
        return true;
    }
    HRESULT LastHr() const { return m_lastHr; }

private:
    IDirectInputDevice8A* m_dev = nullptr;
    IDirectInputEffect* m_fx[S_COUNT] = {};
    int m_lastMag[S_COUNT];
    DWORD m_lastPer[S_COUNT];
    DWORD m_axisId = 0;
    int m_fail = 0;
    HRESULT m_lastHr = DI_OK;

    HRESULT Make(int slot, const GUID& g) {
        DIPERIODIC per{};
        per.dwPeriod = 20000;
        LONG dir = 0;
        DIEFFECT e{};
        e.dwSize = sizeof(DIEFFECT);
        e.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTIDS;   // axis by object id: independent of RBR's data format
        e.dwDuration = INFINITE;
        e.dwGain = DI_FFNOMINALMAX;
        e.dwTriggerButton = DIEB_NOTRIGGER;
        e.cAxes = 1;
        e.rgdwAxes = &m_axisId;
        e.rglDirection = &dir;
        e.cbTypeSpecificParams = sizeof(per);
        e.lpvTypeSpecificParams = &per;
        return m_dev->CreateEffect(g, &e, &m_fx[slot], NULL);
    }

    HRESULT Update(int slot, int mag, DWORD period) {
        DIEFFECT e{};
        e.dwSize = sizeof(DIEFFECT);
        mag = std::max(0, std::min(mag, 10000));
        bool magChanged = std::abs(mag - m_lastMag[slot]) >= kMagDeadband || (mag == 0 && m_lastMag[slot] != 0);
        bool perChanged = m_lastPer[slot] == 0 ||
            (period > m_lastPer[slot] ? period - m_lastPer[slot] : m_lastPer[slot] - period) * 100 > m_lastPer[slot] * 2;
        if (!magChanged && !perChanged) return DI_OK;
        DIPERIODIC p{};
        p.dwMagnitude = (DWORD)mag; p.lOffset = 0; p.dwPhase = 0; p.dwPeriod = period;
        e.cbTypeSpecificParams = sizeof(p); e.lpvTypeSpecificParams = &p;
        HRESULT hr = m_fx[slot]->SetParameters(&e, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);
        if (SUCCEEDED(hr)) { m_lastMag[slot] = mag; m_lastPer[slot] = period; }
        return hr;
    }
};

// ------------------------------------------------------------------ telemetry
// Car.Index may be a uint32 or an integral float depending on the NGP build: accept either.
inline bool readIndex(const uint8_t* b, int len, int off, int& out) {
    if (off < 0 || off + 4 > len) return false;
    uint32_t u; std::memcpy(&u, b + off, 4);
    if (u <= 255u) { out = (int)u; return true; }
    float f; std::memcpy(&f, b + off, 4);
    if (std::isfinite(f) && f >= 0.0f && f <= 255.0f && f == std::floor(f)) { out = (int)f; return true; }
    return false;
}

inline bool readF(const uint8_t* b, int len, int off, float& out) {
    if (off < 0 || off + 4 > len) return false;
    float v; std::memcpy(&v, b + off, 4);
    if (!std::isfinite(v)) return false;
    out = v;
    return true;
}

struct Signals {
    float rpm = 0, speed = 0 /* m/s */, throttle = 0, brake = 0;
    float rough = 0;      // raw front-weighted tyre roughness
    float level = 0;      // roughness mapped through low/high/gamma
    float piston = 0;     // damper piston velocity level
    float texture = 0;    // combined road texture 0..1
    float texL = 0, texR = 0;   // the same, per side (left / right tyres)
    float bump = 0;       // heave acceleration spike envelope
    float dive = 0;       // front load transfer under braking 0..1
    float lock = 0;       // front wheel lock-up 0..1
    float frontRev = 0;   // front wheel revolutions per second
    int   carIndex = -1;  // NGP Car.Index (-1 = unknown)
    // weight / suspension load, relative to this car's own static load (1 = at rest)
    float loadL = 1, loadR = 1, loadF = 1;
    float airborne = 0;   // 0 = wheels loaded, 1 = off the ground
    float thumpL = 0, thumpR = 0;   // compression / landing envelope per side
    float staticLoad = 0; // learned static strut load, all four wheels (units of the telemetry)
};

struct Hp  { float y = 0, x = 0; bool init = false;
             float step(float in, float a) { if (!init) { x = in; init = true; y = 0; return 0; } y = a * (y + in - x); x = in; return y; } };
struct Ema { float v = 0; bool init = false;
             float step(float in, float a) { if (!init) { v = in; init = true; } else v += (in - v) * a; return v; } };

// Per-packet analysis state. Wheel order in NGP: LF, RF, LB, RB.
struct Dsp {
    Hp  sfHp[4], pistonHp[4], accHp;
    Ema sfVar[4], pistonVar[4], loadShort[4], rev[4];
    Ema loadRef, rollRatio, baseFront;
    uint32_t lastSeg[4] = {0, 0, 0, 0};
    bool haveSeg[4] = {false, false, false, false};
    float bumpEnv = 0, diveSm = 0, lockSm = 0;
    Ema statRef[4], rLsm, rRsm, rFsm;
    float thumpL = 0, thumpR = 0;
};

void ParsePacket(const Config& c, const uint8_t* b, int n, Signals& s, Dsp& d, float dt) {
    dt = clampf(dt, 0.002f, 0.05f);
    float v;
    { int ci; if (readIndex(b, n, c.offCarIndex, ci)) s.carIndex = ci; }
    if (readF(b, n, c.offRpm, v))   s.rpm = std::max(0.0f, v * c.rpmScale);
    if (readF(b, n, c.offSpeed, v)) s.speed = std::fabs(v) * (c.speedIsKph ? (1.0f / 3.6f) : 0.44704f);
    s.throttle = readF(b, n, c.offThrottle, v) ? clamp01(v) : 0.6f;
    s.brake    = readF(b, n, c.offBrake, v)    ? clamp01(v) : 0.0f;

    const float aHp = hpCoef(dt, 0.032f);          // ~5 Hz: removes weight transfer, keeps texture and hits
    const float aVar = alphaFor(dt, 0.10f);        // RMS window
    const float aLoad = alphaFor(dt, 0.3f);

    float rough[4] = {0, 0, 0, 0}, absSf[4] = {0, 0, 0, 0}, pRms[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4 && c.offSuspBase >= 0; ++i) {
        const int base = c.offSuspBase + i * c.suspStride;
        float sf = 0, pv = 0;
        if (readF(b, n, base + c.offStrut, sf)) {
            absSf[i] = std::fabs(sf);
            float ls = d.loadShort[i].step(absSf[i], aLoad);
            float hpSf = d.sfHp[i].step(sf, aHp);
            float var = d.sfVar[i].step(hpSf * hpSf, aVar);
            float denom = std::max(ls, 0.3f * d.loadRef.v);
            rough[i] = denom > 1e-6f ? std::sqrt(std::max(var, 0.0f)) / denom : 0.0f;
        }
        if (readF(b, n, base + c.offPiston, pv)) {
            float hp = d.pistonHp[i].step(pv, hpCoef(dt, 0.04f));
            pRms[i] = std::sqrt(std::max(d.pistonVar[i].step(hp * hp, aVar), 0.0f));
        }
        int so = base + c.offSegment;
        if (so >= 0 && so + 4 <= n) {
            uint32_t seg; std::memcpy(&seg, b + so, 4);
            uint32_t sg = seg % 8u;
            if (d.haveSeg[i]) {
                int delta = (int)((sg + 8u - d.lastSeg[i]) % 8u);
                d.rev[i].step(delta / 8.0f / dt, alphaFor(dt, 0.15f));
            }
            d.lastSeg[i] = sg; d.haveSeg[i] = true;
        }
    }
    const float meanLoad = 0.25f * (absSf[0] + absSf[1] + absSf[2] + absSf[3]);
    if (s.speed > 3.0f && meanLoad > 0) d.loadRef.step(meanLoad, alphaFor(dt, 8.0f));

    // ---- weight / suspension load model. Everything is relative to this car's own static load (learned slowly,
    //      reset at stage start when the car stands still), so the telemetry units and the car's mass don't matter.
    {
        const float tot = absSf[0] + absSf[1] + absSf[2] + absSf[3];
        float statTot = 0; bool seeded = true;
        for (int i = 0; i < 4; ++i) { statTot += d.statRef[i].v; seeded = seeded && d.statRef[i].init; }
        if (!seeded || tot > 0.5f * statTot)                       // never learn while airborne
            for (int i = 0; i < 4; ++i) d.statRef[i].step(absSf[i], alphaFor(dt, 30.0f));
        statTot = 0;
        for (int i = 0; i < 4; ++i) statTot += d.statRef[i].v;
        s.staticLoad = statTot;
        if (statTot > 1e-3f) {
            float rw[4];
            for (int i = 0; i < 4; ++i)
                rw[i] = clampf(absSf[i] / std::max(d.statRef[i].v, 0.0375f * statTot), 0.0f, 3.5f);
            const float rl = 0.65f * rw[0] + 0.35f * rw[2], rr2 = 0.65f * rw[1] + 0.35f * rw[3];
            const float aR = alphaFor(dt, 0.08f);
            s.loadL = d.rLsm.step(rl, aR);
            s.loadR = d.rRsm.step(rr2, aR);
            s.loadF = d.rFsm.step(0.5f * (rw[0] + rw[1]), aR);
            s.airborne = s.speed > 3.0f ? 1.0f - smooth01(0.06f, 0.35f, tot / statTot) : 0.0f;
            const float eDecay = std::exp(-dt / 0.14f);               // compression / landing thump per side
            d.thumpL = std::max(clamp01((rl  - 1.7f) / 1.2f), d.thumpL * eDecay);
            d.thumpR = std::max(clamp01((rr2 - 1.7f) / 1.2f), d.thumpR * eDecay);
            s.thumpL = d.thumpL; s.thumpR = d.thumpR;
        }
    }

    // ---- road texture: FRONT tyres dominate (they are what the driver feels through the rack)
    const float rF = std::max(rough[0], rough[1]), rR = std::max(rough[2], rough[3]);
    s.rough = 0.75f * rF + 0.25f * rR;
    s.level = std::pow(clamp01((s.rough - c.roughLow) / (c.roughHigh - c.roughLow)), c.roughGamma);
    s.piston = std::pow(clamp01(std::max(pRms[0], pRms[1]) * c.pistonScale), 1.2f);
    s.texture = clamp01(std::max(s.level, 0.8f * s.piston));

    // ---- per side (NGP wheel order LF, RF, LB, RB): same mapping, fronts weigh most
    auto side = [&](int front, int back) {
        const float r = 0.75f * rough[front] + 0.25f * rough[back];
        const float lv = std::pow(clamp01((r - c.roughLow) / (c.roughHigh - c.roughLow)), c.roughGamma);
        const float pi = std::pow(clamp01(pRms[front] * c.pistonScale), 1.2f);
        return clamp01(std::max(lv, 0.8f * pi));
    };
    s.texL = side(0, 2);
    s.texR = side(1, 3);

    // ---- bumps / landings from heave acceleration
    if (readF(b, n, c.offAccHeave, v)) {
        float hp = d.accHp.step(v, hpCoef(dt, 0.05f));
        d.bumpEnv = std::max(std::fabs(hp) * c.bumpScale, d.bumpEnv * std::exp(-dt / 0.12f));
    }
    s.bump = clamp01(d.bumpEnv);

    // ---- braking: front load transfer ("dive") and front wheel lock-up
    const float tot = absSf[0] + absSf[1] + absSf[2] + absSf[3];
    if (tot > 1e-3f) {
        float share = (absSf[0] + absSf[1]) / tot;
        if (s.brake < 0.05f && s.speed > 3.0f) d.baseFront.step(share, alphaFor(dt, 2.0f));
        float target = (s.brake > 0.1f && d.baseFront.init) ? clamp01((share - d.baseFront.v) / 0.10f) : 0.0f;
        d.diveSm += (target - d.diveSm) * alphaFor(dt, 0.1f);
    }
    s.dive = d.diveSm;

    const float rr = 0.5f * (d.rev[0].v + d.rev[1].v);
    s.frontRev = rr;
    if (s.throttle < 0.05f && s.brake < 0.02f && s.speed > 8.0f && rr > 0.5f)
        d.rollRatio.step(rr / s.speed, alphaFor(dt, 1.0f));      // learn tyre rev/s per m/s while coasting
    const float ratio = d.rollRatio.init ? d.rollRatio.v : 1.0f / c.tyreCircumference;
    float lockT = 0;
    if (s.speed > 4.0f && ratio > 1e-3f && s.brake > 0.2f) {
        float slip = rr / (ratio * s.speed) - 1.0f;
        lockT = clamp01((-slip - c.lockThreshold) / 0.5f);
    }
    d.lockSm += (lockT - d.lockSm) * alphaFor(dt, lockT > d.lockSm ? 0.05f : 0.15f);
    s.lock = d.lockSm;
}

// -------------------------------------------------------------- synthesis
// Turns the analysed signals into effect magnitudes / frequencies. Runs on the worker thread, ~125 Hz.
struct Synth {
    float sm[S_COUNT] = {};
    float thrSm = 0, lope = 0, rpmPeak = 0, heat = 0;
    float roadJit[2] = {1.0f, 1.0f}, ampJit[2] = {1.0f, 1.0f}, engJit = 1.0f, engHzJit = 1.0f, unevPhase = 0;
    double nextJit = 0;

    void Run(const Config& c, const Signals& s, double t, float dt, float userGain, int mag[S_COUNT], float hz[S_COUNT]) {
        thrSm += (s.throttle - thrSm) * alphaFor(dt, 0.12f);

        // ---------------- ENGINE: gain AND frequency both follow the engine
        rpmPeak = std::max(rpmPeak, s.rpm);
        const float rmax = c.rpmMax > 0 ? c.rpmMax : std::max(rpmPeak, c.rpmMaxInit);
        const float x = clamp01(s.rpm / rmax);
        const float fire = s.rpm / 60.0f * c.cylinders * 0.5f;
        const float fireMax = rmax / 60.0f * c.cylinders * 0.5f;
        const float k = fireMax > c.engineMaxHz ? c.engineMaxHz / fireMax : 1.0f;     // linear compression: no pitch jumps
        const float buzzW = smooth01(0.50f, 1.0f, x);                                 // how much of the high-rev range we are in
        const float grit = c.engineGrit * buzzW;
        hz[S_ENG1] = clampf(fire * k * (1.0f + 0.04f * grit * engHzJit), c.engineMinHz, c.engineMaxHz);

        const float idle = 1.0f - smooth01(0.08f, 0.45f, x);                          // lumpy idle
        const float res = gauss(x, 0.55f, 0.08f) + 0.8f * gauss(x, 0.80f, 0.07f);    // engine-order resonances
        const float buzz = smooth01(0.60f, 1.0f, x);                                  // high-rev buzz
        const float A1 = clamp01(0.22f + 0.45f * idle + 0.35f * res + 0.30f * buzz);
        const float loadF = 0.45f + 0.55f * thrSm;                                    // engine under load shakes more
        const float comp = clampf(1.4f - 0.009f * (hz[S_ENG1] - c.engineMinHz), 0.55f, 1.4f);   // wheel is weaker at high Hz
        const float engAir  = 1.0f - 0.35f * c.airborneFade * s.airborne;                       // revving in the air feels lighter
        const float unevMod = 1.0f - 0.35f * c.unevenness * (0.5f + 0.5f * std::sin(unevPhase)); // uneven firing sways the amplitude
        float t1 = s.rpm > 200 ? c.engineMaxMag * clamp01(A1 * loadF * comp * unevMod * engAir * (1.0f - 0.45f * grit * (0.5f - 0.5f * engJit))) : 0.0f;   // irregular firing strength
        if (s.rpm > 200 && x > 0.985f && thrSm > 0.8f) t1 *= (std::fmod(t, 0.08) < 0.04 ? 1.0f : 0.25f);   // rev limiter

        lope += (rnd() - lope) * 0.05f;                                               // uneven firing at low rpm
        // half-order lump at low revs; at high revs the same effect becomes a detuned second oscillator (x1.37)
        // whose beating against the main one is what makes a high-rev engine feel rough instead of a clean sine
        const float xover = smooth01(0.45f, 0.75f, x) * c.engineGrit;
        const float hzLump = clampf(fire * 0.5f, 7.0f, 42.0f);
        const float hzBeat = clampf(hz[S_ENG1] * 1.37f, c.engineMinHz, kHardMaxHz);
        hz[S_ENG2] = hzLump + (hzBeat - hzLump) * xover;
        unevPhase += 6.2831853f * hzLump * dt;
        if (unevPhase > 6.2831853f) unevPhase -= 6.2831853f;
        const float A2 = (0.15f + 0.85f * idle) * (0.5f + 0.5f * thrSm) * (1.0f + 0.25f * lope * (0.2f + idle))
                       + 0.55f * grit * (0.6f + 0.4f * thrSm);
        float t2 = s.rpm > 200 ? c.engineMaxMag * 0.8f * clamp01(A2 * c.lumpScale) * engAir : 0.0f;

        // ---------------- ROAD: two sawtooth effects with opposite sharp edges (zero mean, so they never shift
        // RBR's own force). S_ROAD's sharp edge snaps toward negative (left), S_ROADR's toward positive (right);
        // each is fed by one side's tyre roughness, so an impact on one side is felt as a one-directional snap.
        const float spN = clamp01(s.speed / c.maxSpeed);
        const float gSpd = smooth01(1.0f, 10.0f, s.speed);
        const float texMono = clamp01(s.texture + 0.9f * s.bump);
        const float wf = clampf(c.weightFeel, 0.5f, 2.0f);
        const float thump[2] = { s.thumpL, s.thumpR };
        const float sideTex[2] = { s.texL, s.texR };
        const float sideLoad[2] = { s.loadL, s.loadR };
        float texSide[2], loadGain[2];                                                // [0] = left, [1] = right
        for (int k2 = 0; k2 < 2; ++k2) {
            const float tm = texMono + (clamp01(sideTex[k2] + 0.9f * s.bump) - texMono) * c.roadDirectional;
            texSide[k2] = clamp01(tm + c.thumpGain * wf * thump[k2]);                 // heavier car: harder landing thump
            loadGain[k2] = 1.0f + c.loadInfluence * (clampf(sideLoad[k2], 0.4f, 1.8f) - 1.0f);   // loaded tyres transmit more
        }
        const float roadAir = 1.0f - c.airborneFade * s.airborne;                     // off the ground: no road
        if (t >= nextJit) {                                                           // ~12 ms grain: new random amplitude + pitch per side
            for (int k2 = 0; k2 < 2; ++k2) {
                const float tx = texSide[k2];
                roadJit[k2] = 1.0f + (0.10f + 0.40f * c.roadGrain * (tx > 0.3f ? 1.0f : 0.5f)) * rnd();
                const float lo = 1.0f - 0.65f * c.roadGrain;                          // grain: amplitude varies lo..1
                ampJit[k2] = lo + (1.0f - lo) * (rnd() * 0.5f + 0.5f);
            }
            engJit = rnd(); engHzJit = rnd();
            nextJit = t + 0.012;
        }
        float t3[2];
        for (int k2 = 0; k2 < 2; ++k2) {
            const float spd = c.roadMinHz + (c.roadMaxHz - c.roadMinHz) * clamp01(0.35f * spN + 0.65f * texSide[k2]);
            hz[k2 == 0 ? S_ROAD : S_ROADR] = spd * roadJit[k2] * (k2 ? 1.31f : 1.0f) / std::sqrt(wf);   // detuned sides; heavier = lower pitch
            t3[k2] = c.roadMaxMag * 0.7f * texSide[k2] * gSpd * ampJit[k2] * loadGain[k2] * roadAir;     // two effects share the road budget
        }
        // A = sharp edge to the left (negative), B = to the right. Default: an impact on the left tyre snaps left.
        const int sA = c.roadSideInvert ? 1 : 0, sB = 1 - sA;
        const float tRoadA = t3[sA], tRoadB = t3[sB];
        { const float hzA = hz[sA == 0 ? S_ROAD : S_ROADR], hzB = hz[sB == 0 ? S_ROAD : S_ROADR];
          hz[S_ROAD] = hzA; hz[S_ROADR] = hzB; }

        // ---------------- BRAKE: front judder, harsh on lock-up
        const float gB = smooth01(2.0f, 8.0f, s.speed);
        const float Ab = clamp01(std::pow(s.brake, 1.2f) * (0.3f + 0.7f * s.dive) * 0.6f + s.lock);
        hz[S_BRAKE] = s.lock > 0.1f ? clampf(38.0f + 25.0f * rnd(), 25.0f, 70.0f) : clampf(s.frontRev * 2.0f, 12.0f, 60.0f);
        float t4 = c.brakeMaxMag * Ab * gB * clampf(0.65f + 0.35f * s.loadF, 0.5f, 1.3f);   // more load on the fronts = more judder

        // attack / release: road is deliberately fast (sharpness), engine is rounder
        auto follow = [&](int i, float target, float up, float down) {
            sm[i] += (target - sm[i]) * (target > sm[i] ? up : down);
        };
        follow(S_ENG1, t1, 0.35f, 0.15f);
        follow(S_ENG2, t2, 0.25f, 0.12f);
        const float roadDown = clampf(0.40f / wf, 0.15f, 0.70f);                     // heavier = the thump rings out longer
        follow(S_ROAD,  tRoadA, 0.55f, roadDown);
        follow(S_ROADR, tRoadB, 0.55f, roadDown);
        follow(S_BRAKE, t4, 0.50f, 0.20f);

        float sum = 0;
        for (int i = 0; i < S_COUNT; ++i) sum += std::fabs(sm[i]);
        const float scale = sum > c.totalMaxMag ? c.totalMaxMag / sum : 1.0f;

        // Slow thermal derate on the level actually delivered to the wheel (live gain included): turning the
        // live gain down therefore relieves the derate instead of stacking with it.
        const float ug = clampf(userGain, 0.0f, 1.0f);
        const float lvl = std::min(sum * scale, (float)kHardTotalMax) / 10000.0f * ug;
        heat += (lvl * lvl - heat) * alphaFor(dt, 20.0f);
        const float derate = heat > c.sustainedLimit ? std::sqrt(c.sustainedLimit / heat) : 1.0f;

        const float g = ug * derate;
        for (int i = 0; i < S_COUNT; ++i) {
            float m = sm[i] * scale * g;
            if (!std::isfinite(m)) { m = 0.0f; sm[i] = 0.0f; }          // never let a NaN latch into the filters
            mag[i] = std::max(0, std::min((int)m, kHardTotalMax));
        }
    }
};

// ----------------------------------------------------------------- plugin
struct DeviceInfo { GUID guid; char name[MAX_PATH]; };

class TrueforcePlugin : public IPlugin {
private:
    IRBRGame* game_;
    LPDIRECTINPUT8A m_pDI = nullptr;
    std::vector<DeviceInfo> m_devices;
    int m_sel = 0;
    int m_connectedIdx = -1;
    Config m_cfg;
    std::string m_iniPath;

    // worker thread + shared state
    std::thread m_thread;
    std::atomic<bool> m_run{false};
    std::atomic<bool> m_test{false};
    std::atomic<int>  m_testPhase{0};
    std::atomic<long long> m_packets{0};
    std::atomic<float> m_mag[S_COUNT], m_hz[S_COUNT];
    std::atomic<float> m_dRpm{0}, m_dSpeed{0}, m_dThr{0}, m_dBrk{0}, m_dTex{0}, m_dRough{0}, m_dBump{0},
                       m_dDive{0}, m_dLock{0};
    std::atomic<bool> m_ffActive{false};
    std::atomic<float> m_userGain{1.0f};   // live level 0..1, changed by hotkeys
    std::atomic<bool>  m_muted{false};
    std::atomic<float> m_dHeat{1.0f};      // shown in the menu: 1 = no derate
    std::atomic<int>   m_dCar{-1};
    std::atomic<float> m_dStat{0}, m_dLoadL{1}, m_dLoadR{1}, m_dAir{0};
    std::atomic<bool>  m_stageReset{false};
    std::string m_carName, m_carPreset;    // guarded by m_mtx
    std::mutex m_mtx;
    std::string m_status;
    uint8_t m_pkt[2048];
    int m_pktLen = 0;
    int m_tableBase = 0;           // UI thread only

    void SetStatus(const char* fmt, ...) {
        char b[256];
        va_list ap; va_start(ap, fmt);
        vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        std::lock_guard<std::mutex> lk(m_mtx);
        m_status = b;
    }

    // ------------------------------------------------------------ devices
    static BOOL CALLBACK EnumCb(LPCDIDEVICEINSTANCEA di, LPVOID ref) {
        auto* self = static_cast<TrueforcePlugin*>(ref);
        DeviceInfo info{};
        info.guid = di->guidInstance;
        lstrcpynA(info.name, di->tszProductName, MAX_PATH);
        self->m_devices.push_back(info);
        return DIENUM_CONTINUE;
    }

    void EnumerateDevices() {
        m_devices.clear();
        if (!m_pDI &&
            FAILED(DirectInput8Create(GetModuleHandle(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8A,
                                      (VOID**)&m_pDI, NULL))) {
            SetStatus("DirectInput8Create failed");
            return;
        }
        // Prefer force feedback devices; fall back to everything if none report FFB.
        m_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumCb, this, DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK);
        if (m_devices.empty())
            m_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumCb, this, DIEDFL_ATTACHEDONLY);
        if (m_sel >= (int)m_devices.size()) m_sel = 0;
    }

    static std::string lower(std::string s) {
        for (auto& c : s) c = (char)tolower((unsigned char)c);
        return s;
    }

    // ------------------------------------------------------- connect logic
    void Connect() {
        Disconnect();
        if (m_devices.empty()) { SetStatus("No force feedback devices found"); return; }

        const DeviceInfo& dev = m_devices[m_sel];
        m_cfg.wheel = dev.name;
        WritePrivateProfileStringA("Trueforce", "Wheel", dev.name, m_iniPath.c_str());

        if (!GameDeviceTap::I().Install(m_pDI, dev.guid)) {
            SetStatus("Could not hook DirectInput (see log)");
            return;
        }
        GameDeviceTap::I().Arm(dev.guid);
        m_connectedIdx = m_sel;
        m_packets = 0;
        m_pktLen = 0;
        m_run = true;
        m_thread = std::thread(&TrueforcePlugin::Loop, this);
        SetStatus("Waiting for RBR to use %s ...", dev.name);
    }

    void Disconnect() {
        m_run = false;
        if (m_thread.joinable()) m_thread.join();
        GameDeviceTap::I().Disarm();
        m_test = false;
        m_ffActive = false;
        m_connectedIdx = -1;
    }

    void DumpPacket(const uint8_t* b, int n) {
        FILE* f = fopen((g_dir + "TrueforceRumble_packet.txt").c_str(), "w");
        if (!f) return;
        fprintf(f, "packet length = %d bytes\noffset  float32          raw hex     raw uint32\n", n);
        for (int i = 0; i + 4 <= n; i += 4) {
            float v; uint32_t u;
            std::memcpy(&v, b + i, 4); std::memcpy(&u, b + i, 4);
            fprintf(f, "0x%03X   %-16.6g 0x%08X  %u\n", i, (double)v, u, u);
        }
        fclose(f);
    }

    // Synthetic signals for the TEST mode: 16 s loop through every layer, no telemetry needed.
    void TestSignals(double t, Signals& s) {
        s = Signals();
        s.speed = 25.0f; s.throttle = 1.0f;
        double ph = std::fmod(t, 16.0);
        if (ph < 4.0) {                                   // 0 engine sweep
            m_testPhase = 0; s.rpm = (float)(1000.0 + 6500.0 * (ph / 4.0));
        } else if (ph < 7.0) {                            // 1 gravel
            m_testPhase = 1; s.rpm = 3500; s.texture = 0.35f;
        } else if (ph < 10.0) {                           // 2 rough / dirt
            m_testPhase = 2; s.rpm = 4500; s.texture = 0.9f;
            const bool leftHit = std::fmod(ph - 7.0, 3.0) < 1.5;                      // 1.5 s left-side hits, then 1.5 s right-side
            s.texL = leftHit ? 0.9f : 0.15f; s.texR = leftHit ? 0.15f : 0.9f;
        } else if (ph < 13.0) {                           // 3 hard braking, lock-up after 1.5 s
            m_testPhase = 3; s.rpm = 3000; s.throttle = 0; s.brake = 1.0f; s.dive = 1.0f; s.frontRev = 15.0f;
            if (ph > 11.5) s.lock = 0.9f;
        } else {                                          // 4 idle
            m_testPhase = 4; s.rpm = 900; s.throttle = 0; s.speed = 0;
        }
        if (m_testPhase != 2) s.texL = s.texR = s.texture;
    }

    // ------------------------------------------------------------ live gain hotkeys
    static bool RbrHasFocus() {
        HWND w = GetForegroundWindow();
        if (!w) return false;
        DWORD pid = 0; GetWindowThreadProcessId(w, &pid);
        return pid == GetCurrentProcessId();
    }
    static bool KeyDown(int vk) { return vk > 0 && (GetAsyncKeyState(vk) & 0x8000) != 0; }
    static void BeepAsync(int hz) { std::thread([hz] { Beep((DWORD)hz, 50); }).detach(); }

    // Called from the worker thread every cycle. Edge-triggered, with a small auto-repeat for held keys.
    void PollHotkeys(Clock::time_point now) {
        static bool up = false, dn = false, mu = false;
        static Clock::time_point nextRepeat{};
        if (!RbrHasFocus() || (m_cfg.keyModifier && !KeyDown(m_cfg.keyModifier))) { up = dn = mu = false; return; }
        const bool bu = KeyDown(m_cfg.keyGainUp), bd = KeyDown(m_cfg.keyGainDown), bm = KeyDown(m_cfg.keyMute);
        bool changed = false;
        float g = m_userGain.load();
        const float r = m_cfg.gainRatio;
        if (bu && (!up || now >= nextRepeat)) { g = g < 0.02f ? 0.05f : g / r; changed = true; }
        if (bd && (!dn || now >= nextRepeat)) { g = g * r; if (g < 0.03f) g = 0.0f; changed = true; }
        if (changed) {
            nextRepeat = now + (((bu && up) || (bd && dn)) ? std::chrono::milliseconds(120) : std::chrono::milliseconds(350));
            if (g > 0.98f) g = 1.0f;
            g = clampf(g, 0.0f, 1.0f);
            m_userGain = g;
            m_muted = false;
            char b[32]; snprintf(b, sizeof(b), "%g", g);
            WritePrivateProfileStringA("Trueforce", "UserGain", b, m_iniPath.c_str());
            if (m_cfg.gainBeep) BeepAsync(300 + (int)(g * 900.0f));          // pitch tells you the level
            Logf("Live gain -> %.0f%%", g * 100.0f);
        }
        if (bm && !mu) {
            m_muted = !m_muted;
            if (m_cfg.gainBeep) BeepAsync(m_muted ? 200 : 700);
            Logf("Vibration %s", m_muted ? "MUTED" : "unmuted");
        }
        up = bu; dn = bd; mu = bm;
    }

    // ------------------------------------------------------------ main loop
    void Loop() {
        try { LoopImpl(); }
        catch (...) { Logf("Worker thread exception caught; effects released"); m_ffActive = false; }
    }

    void LoopImpl() {
        WSADATA wsa;
        bool wsaOk = WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
        SOCKET sock = INVALID_SOCKET;
        if (wsaOk) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock != INVALID_SOCKET) {
                sockaddr_in a{};
                a.sin_family = AF_INET;
                a.sin_port = htons((u_short)m_cfg.udpPort);
                a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   // telemetry is local only
                if (bind(sock, (sockaddr*)&a, sizeof(a)) == SOCKET_ERROR) {
                    Logf("UDP bind to port %d failed (WSA %d). Another app on that port?", m_cfg.udpPort, WSAGetLastError());
                    closesocket(sock); sock = INVALID_SOCKET;
                } else {
                    DWORD to = 20;
                    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
                }
            }
        }
        const bool haveSock = sock != INVALID_SOCKET;
        if (!haveSock) Logf("No UDP socket: only TEST mode will work");

        std::vector<uint8_t> buf(2048);
        FFLayer ff;
        Dsp dsp;
        Synth synth;
        Signals live, sig;
        Config eff = m_cfg;                    // m_cfg + the active car profile
        int curCar = -2;
        CarProfile prof;
        bool dumped = false, prevTest = false;
        double testT0 = 0;

        const auto t0 = Clock::now();
        auto lastRx = t0 - std::chrono::seconds(10);
        auto lastPkt = t0, lastApply = t0, nextTry = t0;

        while (m_run) {
            const auto now = Clock::now();
            bool got = false;

            if (haveSock) {
                int n = recvfrom(sock, (char*)buf.data(), (int)buf.size(), 0, nullptr, nullptr);
                if (n >= m_cfg.minPacket) {
                    m_packets++;
                    got = true;
                    float pdt = std::chrono::duration<float>(now - lastPkt).count();
                    lastPkt = now; lastRx = now;
                    if (m_stageReset.exchange(false)) {                 // car is standing at the start line: re-learn static load
                        for (int i = 0; i < 4; ++i) dsp.statRef[i] = Ema();
                        dsp.thumpL = dsp.thumpR = 0;
                    }
                    ParsePacket(eff, buf.data(), n, live, dsp, pdt);
                    if (live.carIndex >= 0 && live.carIndex != curCar) {   // new car: load its profile, forget what the last one taught us
                        curCar = live.carIndex;
                        prof = CarProfile();
                        if (m_cfg.carProfiles) LoadCarProfile(curCar, prof);
                        ApplyProfile(m_cfg, prof, eff);
                        dsp = Dsp(); synth.rpmPeak = 0;
                        { std::lock_guard<std::mutex> lk(m_mtx);
                          m_carName = prof.found ? (prof.name.empty() ? "(unnamed profile)" : prof.name) : "no profile";
                          m_carPreset = prof.preset; }
                        m_dCar = curCar;
                        Logf("Car index %d: %s  preset=%s  cyl=%d rpmMax=%.0f weightFeel=%.2f", curCar,
                             prof.found ? "profile loaded" : "no profile in TrueforceCars.ini", prof.preset.c_str(),
                             eff.cylinders, eff.rpmMax, eff.weightFeel);
                    }
                    {
                        std::lock_guard<std::mutex> lk(m_mtx);
                        m_pktLen = n;
                        std::memcpy(m_pkt, buf.data(), n);
                    }
                    if (!dumped) { DumpPacket(buf.data(), n); dumped = true; }
                }
            } else {
                Sleep(10);
            }

            const double t = std::chrono::duration<double>(now - t0).count();
            const float sinceApply = std::chrono::duration<float>(now - lastApply).count();
            if (sinceApply < kApplyPeriod) continue;
            lastApply = now;
            const float dt = clampf(sinceApply, 0.002f, 0.05f);

            if (m_test) {
                TestSignals(t, sig);
            } else if (std::chrono::duration<double>(now - lastRx).count() < 0.4) {
                sig = live;
            } else {
                sig = Signals();                   // no telemetry (stage over / paused) -> fade to zero
            }

            m_dRpm = sig.rpm; m_dSpeed = sig.speed; m_dThr = sig.throttle; m_dBrk = sig.brake;
            m_dTex = sig.texture; m_dRough = sig.rough; m_dBump = sig.bump;
            m_dDive = sig.dive; m_dLock = sig.lock;
            m_dStat = sig.staticLoad; m_dLoadL = sig.loadL; m_dLoadR = sig.loadR; m_dAir = sig.airborne;

            PollHotkeys(now);
            const bool testNow = m_test.load();
            if (testNow && !prevTest) testT0 = t;
            prevTest = testNow;
            // TEST mode fades in over one second so it never starts with a jolt.
            const float testRamp = testNow ? clamp01((float)(t - testT0)) : 1.0f;
            const float live_gain = m_muted.load() ? 0.0f : m_userGain.load() * testRamp;

            int mag[S_COUNT]; float hz[S_COUNT];
            synth.Run(eff, sig, t, dt, live_gain, mag, hz);
            m_dHeat = synth.heat > m_cfg.sustainedLimit ? std::sqrt(m_cfg.sustainedLimit / synth.heat) : 1.0f;
            DWORD per[S_COUNT];
            for (int i = 0; i < S_COUNT; ++i) {
                hz[i] = std::isfinite(hz[i]) ? clampf(hz[i], kHardMinHz, kHardMaxHz) : kHardMinHz;
                m_mag[i] = (float)mag[i]; m_hz[i] = hz[i];
                per[i] = (DWORD)(1000000.0f / hz[i]);
            }

            // ------------------------------------------------ push to wheel
            if (!ff.Ready()) {
                if (now >= nextTry) {
                    nextTry = now + std::chrono::seconds(1);
                    IDirectInputDevice8A* d = GameDeviceTap::I().Captured();
                    if (!d) {
                        SetStatus("Waiting for RBR to use the wheel (is FFB enabled in RBR?)");
                    } else {
                        HRESULT hr = ff.Create(d);
                        if (SUCCEEDED(hr)) {
                            m_ffActive = true;
                            SetStatus("ACTIVE: layered on RBR's FFB. Select = TEST vibration");
                        } else {
                            m_ffActive = false;
                            Logf("Create failed: 0x%08X", (unsigned)hr);
                            if (hr == DIERR_NOTEXCLUSIVEACQUIRED || hr == DIERR_OTHERAPPHASPRIO) {
                                // Not the object that holds the wheel exclusively (RBR/NGP may use several).
                                GameDeviceTap::I().ReleaseCaptured();
                                SetStatus("Skipped a non-exclusive device object (0x%08X), looking for RBR's", (unsigned)hr);
                            } else {
                                SetStatus("CreateEffect failed: 0x%08X (E_NOTIMPL = device has no FFB)", (unsigned)hr);
                            }
                        }
                    }
                }
            } else {
                if (GameDeviceTap::I().ConsumeDropRequest()) {
                    ff.Destroy();
                    GameDeviceTap::I().ReleaseCaptured();
                    m_ffActive = false;
                    SetStatus("RBR released the wheel; waiting for it to come back...");
                    continue;
                }
                if (GameDeviceTap::I().ConsumeReacquired()) ff.Restart();
                if (!ff.Push(mag, per)) {
                    Logf("Effects lost (0x%08X), recreating", (unsigned)ff.LastHr());
                    SetStatus("Effects lost (0x%08X), retrying", (unsigned)ff.LastHr());
                    ff.Destroy();
                    m_ffActive = false;
                }
            }
        }

        if (ff.Ready()) {                       // settle to zero before the effects are torn down
            int z[S_COUNT] = {};
            DWORD p[S_COUNT];
            for (int i = 0; i < S_COUNT; ++i) p[i] = 20000;
            ff.Push(z, p);
            Sleep(30);
        }
        ff.Destroy();
        if (sock != INVALID_SOCKET) closesocket(sock);
        if (wsaOk) WSACleanup();
    }

public:
    explicit TrueforcePlugin(IRBRGame* game) : game_(game) {
        g_dir = pluginDirectory();
        m_iniPath = g_dir + "TrueforceRumble.ini";
        std::memset(m_pkt, 0, sizeof(m_pkt));
        for (int i = 0; i < S_COUNT; ++i) { m_mag[i] = 0; m_hz[i] = 0; }
        bool exists = GetFileAttributesA(m_iniPath.c_str()) != INVALID_FILE_ATTRIBUTES;
        if (!exists) SyncConfig(m_cfg, m_iniPath, true);     // write a template with every key
        SyncConfig(m_cfg, m_iniPath, false);
        m_userGain = m_cfg.userGain;

        SetStatus("Pick the wheel with Left/Right, press Select");
        EnumerateDevices();

        if (!m_cfg.wheel.empty()) {
            std::string want = lower(m_cfg.wheel);
            for (int i = 0; i < (int)m_devices.size(); ++i) {
                if (lower(m_devices[i].name).find(want) != std::string::npos) {
                    m_sel = i;
                    if (m_cfg.autoConnect) Connect();
                    break;
                }
            }
        }
    }

    ~TrueforcePlugin() {
        Disconnect();
        GameDeviceTap::I().Remove();       // restore the vtable before our DLL goes away
        if (m_pDI) { m_pDI->Release(); m_pDI = nullptr; }
    }

    const char* GetName() override { return "Trueforce Rumble"; }

    void DrawFrontEndPage() override {
        if (!game_) return;
        float y = 50.0f;
        game_->SetFont(IRBRGame::FONT_BIG);
        game_->SetMenuColor(IRBRGame::MENU_HEADING);
        game_->WriteText(70.0f, y, "Trueforce Rumble Settings");
        y += 28.0f;

        game_->SetFont(IRBRGame::FONT_SMALL);
        game_->SetMenuColor(IRBRGame::MENU_TEXT);
        char buf[300];
        auto line = [&](const char* txt) { game_->WriteText(70.0f, y, txt); y += 19.0f; };

        snprintf(buf, sizeof(buf), "FFB devices: %d   (Left/Right = choose, Select = connect / toggle TEST)", (int)m_devices.size());
        line(buf);
        if (!m_devices.empty()) {
            snprintf(buf, sizeof(buf), "> [%d/%d] %s%s", m_sel + 1, (int)m_devices.size(), m_devices[m_sel].name,
                     m_sel == m_connectedIdx ? "  (connected)" : "");
            game_->SetMenuColor(IRBRGame::MENU_SELECTION);
            line(buf);
            game_->SetMenuColor(IRBRGame::MENU_TEXT);
        } else {
            line("> [no force feedback devices detected]");
        }

        std::string st;
        { std::lock_guard<std::mutex> lk(m_mtx); st = m_status; }
        snprintf(buf, sizeof(buf), "Status: %s", st.c_str());
        line(buf);

        static const char* const kPhase[] = {"engine sweep", "gravel", "rough / dirt", "hard braking + lock-up", "idle"};
        snprintf(buf, sizeof(buf), "UDP packets: %lld (port %d)   TEST: %s", (long long)m_packets.load(), m_cfg.udpPort,
                 m_test ? kPhase[std::max(0, std::min(m_testPhase.load(), 4))] : "off");
        line(buf);

        snprintf(buf, sizeof(buf), "rpm %.0f  speed %.1f m/s  throttle %.2f  brake %.2f",
                 m_dRpm.load(), m_dSpeed.load(), m_dThr.load(), m_dBrk.load());
        line(buf);
        snprintf(buf, sizeof(buf), "texture %.2f (front roughness %.2f)  bump %.2f  dive %.2f  lock %.2f",
                 m_dTex.load(), m_dRough.load(), m_dBump.load(), m_dDive.load(), m_dLock.load());
        line(buf);
        { std::string nm, pr; { std::lock_guard<std::mutex> lk(m_mtx); nm = m_carName; pr = m_carPreset; }
          snprintf(buf, sizeof(buf), "car %d: %s%s%s   static load %.0f   load L %.2f R %.2f   airborne %.2f",
                   m_dCar.load(), nm.empty() ? "waiting for telemetry" : nm.c_str(), pr.empty() ? "" : "  preset ", pr.c_str(),
                   m_dStat.load(), m_dLoadL.load(), m_dLoadR.load(), m_dAir.load());
          line(buf); }
        snprintf(buf, sizeof(buf), "live gain %.0f%%%s   thermal derate x%.2f   (Ctrl + Num+ / Num- / Num* in game)",
                 m_userGain.load() * 100.0f, m_muted ? " MUTED" : "", m_dHeat.load());
        line(buf);
        snprintf(buf, sizeof(buf), "engine %.0f@%.0fHz  lump %.0f@%.0fHz  road L %.0f R %.0f  brake %.0f@%.0fHz",
                 m_mag[S_ENG1].load(), m_hz[S_ENG1].load(), m_mag[S_ENG2].load(), m_hz[S_ENG2].load(),
                 m_mag[S_ROAD].load(), m_mag[S_ROADR].load(), m_mag[S_BRAKE].load(), m_hz[S_BRAKE].load());
        line(buf);
        y += 6.0f;

        uint8_t pkt[2048]; int len;
        { std::lock_guard<std::mutex> lk(m_mtx); len = m_pktLen; std::memcpy(pkt, m_pkt, sizeof(pkt)); }
        line("Raw packet floats (Up/Down scroll):");
        for (int row = 0; row < 4; ++row) {
            int base = m_tableBase + row * 16;
            char ln[200]; int p = snprintf(ln, sizeof(ln), "0x%03X ", base);
            for (int c = 0; c < 4; ++c) {
                int off = base + c * 4;
                if (off + 4 <= len) {
                    float v; std::memcpy(&v, pkt + off, 4);
                    p += snprintf(ln + p, sizeof(ln) - p, " %10.4g", std::isfinite(v) ? (double)v : 0.0);
                } else {
                    p += snprintf(ln + p, sizeof(ln) - p, " %10s", "-");
                }
            }
            game_->WriteText(70.0f, y, ln); y += 17.0f;
        }
    }

    void DrawResultsUI() override {}

    void HandleFrontEndEvents(char, bool bUp, bool bDown, bool bLeft, bool bRight, bool bSelect) override {
        if (bUp)   m_tableBase = std::max(0, m_tableBase - 64);
        if (bDown) m_tableBase = std::min(2048 - 64, m_tableBase + 64);
        if (m_devices.empty()) return;

        if (bLeft)  { m_sel = (m_sel - 1 + (int)m_devices.size()) % (int)m_devices.size(); }
        if (bRight) { m_sel = (m_sel + 1) % (int)m_devices.size(); }
        if (bSelect) {
            if (m_connectedIdx == m_sel && m_run) m_test = !m_test;   // already connected: toggle synthetic TEST vibration
            else Connect();
        }
    }

    void TickFrontEndPage(float) override {}
    void StageStarted(int, const char*, bool) override { m_stageReset = true; }
    void HandleResults(float, float, float, const char*) override {}
    void CheckPoint(float, int, const char*) override {}
};

} // namespace

extern "C" __declspec(dllexport) IPlugin* RBR_CreatePlugin(IRBRGame* pGame) {
    // Proof-of-life: if this file never appears next to the DLL, RBR did not call the plugin.
    try {
        g_dir = pluginDirectory();
        FILE* f = fopen((g_dir + "TrueforceRumble_loaded.txt").c_str(), "w");
        if (f) { fprintf(f, "RBR_CreatePlugin was called by RBR.\n"); fclose(f); }
    } catch (...) {}
    try { return new TrueforcePlugin(pGame); }
    catch (...) { return nullptr; }
}
