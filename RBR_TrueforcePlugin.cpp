// RBR_TrueforcePlugin.cpp  (v2)
//
// Layers extra "Trueforce-like" vibration (engine + road surface) on top of
// Richard Burns Rally's own force feedback.
//
// WHY v1 FAILED
//   RBR already holds the wheel with an exclusive DirectInput acquire. A second
//   device object in the plugin can never get exclusive access, and DirectInput
//   only lets you CreateEffect() on an exclusively acquired device. (Also: the
//   cooperative level used a NULL window handle, nothing was error-checked and
//   the effect struct was incomplete.)
//
// HOW v2 WORKS
//   1. We patch the IDirectInputDevice8 vtable (Acquire / GetDeviceState /
//      GetDeviceData / Poll). RBR polls its wheel every frame, so our hook sees
//      RBR's own device object, matches it by instance GUID and keeps a
//      reference to it.
//   2. On that device we create two extra periodic (sine) effects. DirectInput
//      sums them with RBR's own effects in the wheel firmware:
//        - ENGINE: frequency follows RPM (cylinders/2 pulses per rev, folded
//          into a range wheels can actually render), amplitude follows load.
//        - ROAD:   frequency follows speed (+ jitter), amplitude follows
//          suspension activity (fallback: speed only).
//   3. Telemetry comes from NGP's UDP stream (rbr.telemetry.data.TelemetryData.h,
//      NGP 7.4.778, packed, 664 bytes). Offsets used (all float32 unless noted):
//        throttle 28 | speed 60 (kph or mph) | accHeave 120 | engine rpm 136
//        suspension blocks LF/RF/LB/RB at 152 + n*128: strutForce +16, piston velocity +28
//      They can be overridden in the INI if a future NGP changes the layout.
//      NGP must send a SECOND UDP endpoint to this plugin, e.g. in NGP's ini:
//        udpTelemetry=1
//        udpTelemetryEndpoints=127.0.0.1:6776,127.0.0.1:6778   (6776 SimHub, 6777 RbrSurfaceAudio)
//
// BUILD: 32-bit (RBR is x86), link ws2_32 / dinput8 / dxguid (pragmas below).
//
// FILES (next to the DLL): TrueforceRumble.ini, TrueforceRumble.log,
//                           TrueforceRumble_packet.txt (first packet dump)

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

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float clamp01(float v) { return clampf(v, 0.0f, 1.0f); }

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
    int   udpPort = 6778;            // 6776 = SimHub, 6777 = RbrSurfaceAudio; do not share a port (UDP unicast goes to one socket only)
    int   minPacket = 664;           // NGP 7.4.778 packet length
    int   cylinders = 4;             // engine pulses per rev = cylinders / 2
    float masterGain = 1.0f, engineGain = 1.0f, roadGain = 1.0f;
    int   engineMaxMag = 3000;       // DirectInput units, 0..10000
    int   roadMaxMag = 3000;
    float engineMinHz = 18.0f, engineMaxHz = 110.0f;
    float roadMinHz = 28.0f, roadMaxHz = 95.0f;

    // Byte offsets of float32 fields inside the NGP UDP packet (defaults from TelemetryData.h, NGP 7.4.778).
    int   offRpm = 136, offSpeed = 60, offThrottle = 28, offAccHeave = 120;
    int   offSuspBase = 152, suspStride = 128, offPiston = 28, suspCount = 4;   // damper piston velocity per wheel
    int   speedIsKph = 1;            // 0 if RBR is set to mph
    float rpmScale = 1.0f;
    float suspScale = 1.0f;          // RMS piston velocity (m/s) -> 0..1 road texture; tune with the status page
    float bumpScale = 0.1f;          // high-passed heave acceleration -> 0..1 bump impulse; tune with the status page
    float maxSpeed = 40.0f;          // m/s that counts as "full speed" for road texture

    int         autoConnect = 0;
    std::string wheel;               // device name (substring match, case-insensitive)
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
    I("UdpPort", c.udpPort);            I("MinPacketBytes", c.minPacket);
    I("Cylinders", c.cylinders);
    F("MasterGain", c.masterGain);      F("EngineGain", c.engineGain);   F("RoadGain", c.roadGain);
    I("EngineMaxMagnitude", c.engineMaxMag); I("RoadMaxMagnitude", c.roadMaxMag);
    F("EngineMinHz", c.engineMinHz);    F("EngineMaxHz", c.engineMaxHz);
    F("RoadMinHz", c.roadMinHz);        F("RoadMaxHz", c.roadMaxHz);
    I("OffsetRpm", c.offRpm);           I("OffsetSpeed", c.offSpeed);    I("OffsetThrottle", c.offThrottle);
    I("OffsetAccHeave", c.offAccHeave); I("OffsetSuspBase", c.offSuspBase); I("SuspStride", c.suspStride);
    I("OffsetPistonVel", c.offPiston);  I("SuspCount", c.suspCount);     I("SpeedIsKph", c.speedIsKph);
    F("RpmScale", c.rpmScale);          F("SuspScale", c.suspScale);     F("BumpScale", c.bumpScale);
    F("MaxSpeed", c.maxSpeed);
    I("AutoConnect", c.autoConnect);
    char b[260];
    if (write) WritePrivateProfileStringA(S, "Wheel", c.wheel.c_str(), path.c_str());
    else { GetPrivateProfileStringA(S, "Wheel", "", b, sizeof(b), path.c_str()); c.wheel = b; }

    if (!write) {
        c.cylinders = std::max(1, std::min(c.cylinders, 16));
        c.suspCount = std::max(0, std::min(c.suspCount, 4));
        c.engineMaxMag = std::max(0, std::min(c.engineMaxMag, 10000));
        c.roadMaxMag = std::max(0, std::min(c.roadMaxMag, 10000));
        if (c.engineMaxHz < c.engineMinHz * 2.0f) c.engineMinHz = c.engineMaxHz * 0.5f; // fold loop needs 2x range
        if (c.maxSpeed < 1.0f) c.maxSpeed = 1.0f;
    }
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
    static void Unpatch() {
        if (!vtbl) return;
        const int idx[5] = { VT_ACQUIRE, VT_GETDEVICESTATE, VT_GETDEVICEDATA, VT_POLL, VT_UNACQUIRE };
        for (int i = 0; i < 5; ++i) {
            DWORD old;
            if (orig[i] && VirtualProtect(&vtbl[idx[i]], sizeof(void*), PAGE_READWRITE, &old)) {
                vtbl[idx[i]] = orig[i];
                VirtualProtect(&vtbl[idx[i]], sizeof(void*), old, &old);
            }
        }
        vtbl = nullptr;
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
    Tap<0>::Unpatch();
    Tap<1>::Unpatch();
    m_installed = false;
}

// ----------------------------------------------------------- effect layer
BOOL CALLBACK FindFFAxisCb(const DIDEVICEOBJECTINSTANCEA* o, VOID* ref) {
    // Only reads fields that sit before the name strings, so it is safe for the
    // ANSI and unicode variants alike.
    if (o->dwFlags & DIDOI_FFACTUATOR) {
        *static_cast<DWORD*>(ref) = o->dwType;
        return DIENUM_STOP;
    }
    return DIENUM_CONTINUE;
}

class FFLayer {
public:
    bool Ready() const { return m_engine && m_road; }

    HRESULT Create(IDirectInputDevice8A* dev) {
        Destroy();
        m_dev = dev;

        DWORD axisId = 0xFFFFFFFF;
        dev->EnumObjects(FindFFAxisCb, &axisId, DIDFT_AXIS);
        if (axisId == 0xFFFFFFFF) return E_NOTIMPL;   // no force feedback actuator on this device
        m_axisId = axisId;

        HRESULT hr = MakeSine(&m_engine);
        if (SUCCEEDED(hr)) hr = MakeSine(&m_road);
        if (FAILED(hr)) { Destroy(); return hr; }

        m_engine->Start(1, 0);
        m_road->Start(1, 0);
        m_lastEngMag = m_lastRoadMag = -1;
        m_fail = 0;
        return DI_OK;
    }

    void Destroy() {
        if (m_engine) { m_engine->Stop(); m_engine->Release(); m_engine = nullptr; }
        if (m_road)   { m_road->Stop();   m_road->Release();   m_road = nullptr; }
        m_dev = nullptr;
    }

    // RBR re-acquired the device (alt-tab, menu change): make sure we are playing.
    void Restart() {
        if (m_engine) m_engine->Start(1, 0);
        if (m_road)   m_road->Start(1, 0);
        m_lastEngMag = m_lastRoadMag = -1;
    }

    // Returns false if effects look dead and should be recreated.
    bool Push(int engMag, DWORD engPeriodUs, int roadMag, DWORD roadPeriodUs) {
        HRESULT h1 = Update(m_engine, engMag, engPeriodUs, m_lastEngMag, m_lastEngPer);
        HRESULT h2 = Update(m_road,   roadMag, roadPeriodUs, m_lastRoadMag, m_lastRoadPer);
        if (FAILED(h1) || FAILED(h2)) {
            m_lastHr = FAILED(h1) ? h1 : h2;
            // lost/unloaded effect: try to start it again, give up after a while
            if (m_engine) m_engine->Start(1, 0);
            if (m_road)   m_road->Start(1, 0);
            m_lastEngMag = m_lastRoadMag = -1;
            return ++m_fail < 100;
        }
        m_fail = 0;
        return true;
    }
    HRESULT LastHr() const { return m_lastHr; }

private:
    IDirectInputDevice8A* m_dev = nullptr;
    IDirectInputEffect* m_engine = nullptr;
    IDirectInputEffect* m_road = nullptr;
    DWORD m_axisId = 0;
    int m_lastEngMag = -1, m_lastRoadMag = -1;
    DWORD m_lastEngPer = 0, m_lastRoadPer = 0;
    int m_fail = 0;
    HRESULT m_lastHr = DI_OK;

    HRESULT MakeSine(IDirectInputEffect** out) {
        DIPERIODIC per{};
        per.dwMagnitude = 0;
        per.lOffset = 0;
        per.dwPhase = 0;
        per.dwPeriod = 20000;                 // 50 Hz placeholder

        LONG dir = 0;
        DIEFFECT e{};
        e.dwSize = sizeof(DIEFFECT);
        e.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTIDS;   // axis given as object id: independent of RBR's data format
        e.dwDuration = INFINITE;
        e.dwSamplePeriod = 0;
        e.dwGain = DI_FFNOMINALMAX;
        e.dwTriggerButton = DIEB_NOTRIGGER;              // v1 left this 0 = "button 0 triggers the effect"
        e.dwTriggerRepeatInterval = 0;
        e.cAxes = 1;
        e.rgdwAxes = &m_axisId;
        e.rglDirection = &dir;
        e.lpEnvelope = NULL;
        e.cbTypeSpecificParams = sizeof(per);
        e.lpvTypeSpecificParams = &per;
        e.dwStartDelay = 0;
        return m_dev->CreateEffect(GUID_Sine, &e, out, NULL);
    }

    static HRESULT Update(IDirectInputEffect* fx, int mag, DWORD period, int& lastMag, DWORD& lastPer) {
        if (!fx) return DI_OK;
        mag = std::max(0, std::min(mag, 10000));
        bool magChanged = (lastMag < 0) || std::abs(mag - lastMag) >= 20;
        bool perChanged = (lastPer == 0) || (period > lastPer ? period - lastPer : lastPer - period) * 100 > lastPer * 2;
        if (!magChanged && !perChanged) return DI_OK;      // spare the USB bus

        DIPERIODIC per{};
        per.dwMagnitude = (DWORD)mag;
        per.lOffset = 0;
        per.dwPhase = 0;
        per.dwPeriod = period;
        DIEFFECT e{};
        e.dwSize = sizeof(DIEFFECT);
        e.cbTypeSpecificParams = sizeof(per);
        e.lpvTypeSpecificParams = &per;
        HRESULT hr = fx->SetParameters(&e, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);
        if (SUCCEEDED(hr)) { lastMag = mag; lastPer = period; }
        return hr;
    }
};

// ---------------------------------------------------------------- signals
struct Signals {
    float rpm = 0, speed = 0 /* m/s */, throttle = 0, susp = 0, bump = 0;
};

// Per-packet analysis state: road texture = RMS of the high-passed damper piston velocity of each wheel
// (like the reference RbrSurfaceAudio analysis); bumps = high-passed heave acceleration with a decaying envelope.
struct RoadDsp {
    float lastPiston[4] = {0, 0, 0, 0}, hp[4] = {0, 0, 0, 0}, var[4] = {0, 0, 0, 0};
    bool havePiston = false;
    float lastAcc = 0, accHp = 0, bumpEnv = 0;
    bool haveAcc = false;
};

inline bool readF(const uint8_t* b, int len, int off, float& out) {
    if (off < 0 || off + 4 > len) return false;
    float v; std::memcpy(&v, b + off, 4);
    if (!std::isfinite(v)) return false;
    out = v;
    return true;
}

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

    // thread + shared state
    std::thread m_thread;
    std::atomic<bool> m_run{false};
    std::atomic<bool> m_test{false};
    std::atomic<long long> m_packets{0};
    std::atomic<float> m_engMag{0}, m_roadMag{0}, m_engHz{0}, m_roadHz{0};
    std::atomic<float> m_rRpm{0}, m_rSpeed{0}, m_rThr{0}, m_rSusp{0}, m_rBump{0};
    std::atomic<bool> m_ffActive{false};
    std::mutex m_mtx;
    std::string m_status;
    uint8_t m_pkt[2048];
    int m_pktLen = 0;
    int m_tableBase = 0;           // byte offset of the packet table shown on screen (UI thread only)

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

    // --------------------------------------------------------- DSP helpers
    void ParsePacket(const uint8_t* b, int n, Signals& s, RoadDsp& d, float dt) {
        float v;
        if (readF(b, n, m_cfg.offRpm, v))      s.rpm = std::max(0.0f, v * m_cfg.rpmScale);
        if (readF(b, n, m_cfg.offSpeed, v))    s.speed = std::fabs(v) * (m_cfg.speedIsKph ? (1.0f / 3.6f) : 0.44704f);
        if (readF(b, n, m_cfg.offThrottle, v)) s.throttle = clamp01(v);
        else                                    s.throttle = 0.6f;

        dt = clampf(dt, 0.002f, 0.05f);
        const float hpA = 0.04f / (0.04f + dt);                       // ~4 Hz high-pass: removes body motion, keeps texture
        const float emaA = 1.0f - std::exp(-dt / 0.12f);              // RMS window ~120 ms

        float sum = 0; int cnt = 0;
        for (int i = 0; i < m_cfg.suspCount && m_cfg.offSuspBase >= 0; ++i) {
            if (readF(b, n, m_cfg.offSuspBase + i * m_cfg.suspStride + m_cfg.offPiston, v)) {
                if (d.havePiston) d.hp[i] = hpA * (d.hp[i] + v - d.lastPiston[i]);
                d.lastPiston[i] = v;
                d.var[i] += (d.hp[i] * d.hp[i] - d.var[i]) * emaA;
                sum += std::sqrt(std::max(0.0f, d.var[i])); ++cnt;
            }
        }
        d.havePiston = d.havePiston || cnt > 0;
        s.susp = cnt ? clamp01(sum / cnt * m_cfg.suspScale) : 0.0f;

        if (readF(b, n, m_cfg.offAccHeave, v)) {
            if (d.haveAcc) d.accHp = (0.05f / (0.05f + dt)) * (d.accHp + v - d.lastAcc);
            d.lastAcc = v; d.haveAcc = true;
            d.bumpEnv = std::max(std::fabs(d.accHp) * m_cfg.bumpScale, d.bumpEnv * std::exp(-dt / 0.12f));
        }
        s.bump = clamp01(d.bumpEnv);
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

    // ------------------------------------------------------------ main loop
    void Loop() {
        WSADATA wsa;
        bool wsaOk = WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
        SOCKET sock = INVALID_SOCKET;
        if (wsaOk) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock != INVALID_SOCKET) {
                sockaddr_in a{};
                a.sin_family = AF_INET;
                a.sin_port = htons((u_short)m_cfg.udpPort);
                a.sin_addr.s_addr = htonl(INADDR_ANY);
                if (bind(sock, (sockaddr*)&a, sizeof(a)) == SOCKET_ERROR) {
                    Logf("UDP bind to port %d failed (WSA %d)", m_cfg.udpPort, WSAGetLastError());
                    closesocket(sock); sock = INVALID_SOCKET;
                } else {
                    DWORD to = 20;
                    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&to, sizeof(to));
                }
            }
        }
        bool haveSock = sock != INVALID_SOCKET;
        if (!haveSock) Logf("No UDP socket: telemetry unavailable, only TEST mode will work");

        std::vector<uint8_t> buf(2048);
        FFLayer ff;
        Signals sig, live;
        RoadDsp dsp;
        bool dumped = false;
        auto lastPkt = Clock::now();
        float engSm = 0, roadSm = 0, engHz = m_cfg.engineMinHz, roadHz = m_cfg.roadMinHz;

        auto t0 = Clock::now();
        auto lastRx = t0 - std::chrono::seconds(10);
        auto lastApply = t0;
        auto nextTry = t0;

        while (m_run) {
            auto now = Clock::now();
            bool got = false;

            if (haveSock) {
                int n = recvfrom(sock, (char*)buf.data(), (int)buf.size(), 0, nullptr, nullptr);
                if (n >= m_cfg.minPacket) {
                    m_packets++;
                    got = true;
                    lastRx = now;
                    float pdt = std::chrono::duration<float>(now - lastPkt).count();
                    lastPkt = now;
                    ParsePacket(buf.data(), n, live, dsp, pdt);
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

            double t = std::chrono::duration<double>(now - t0).count();
            if (m_test) {
                sig.rpm = 1500.0f + 5500.0f * (0.5f + 0.5f * (float)std::sin(t * 0.6));
                sig.speed = 25.0f;
                sig.throttle = 1.0f;
                sig.susp = 0.4f + 0.3f * (float)std::sin(t * 3.0);
                sig.bump = std::fmod((float)t, 1.5f) < 0.1f ? 0.8f : 0.0f;
            } else if (std::chrono::duration<double>(now - lastRx).count() < 0.4) {
                sig = live;
            } else {
                sig = Signals();       // stage not running / no telemetry -> fade to zero
            }

            if (std::chrono::duration<double>(now - lastApply).count() < 0.008) continue;
            lastApply = now;

            m_rRpm = sig.rpm; m_rSpeed = sig.speed; m_rThr = sig.throttle; m_rSusp = sig.susp; m_rBump = sig.bump;

            // ------------------------------------------------ engine layer
            float engT = 0;
            if (sig.rpm > 200.0f) {
                float hz = sig.rpm / 60.0f * (m_cfg.cylinders * 0.5f);
                for (int i = 0; i < 16 && hz > m_cfg.engineMaxHz; ++i) hz *= 0.5f;   // fold into renderable range
                for (int i = 0; i < 16 && hz < m_cfg.engineMinHz; ++i) hz *= 2.0f;
                engHz = hz;
                float load = 0.35f + 0.65f * clamp01(sig.throttle);
                float rf = 0.5f + 0.5f * clamp01(sig.rpm / 8000.0f);
                engT = m_cfg.engineMaxMag * m_cfg.engineGain * m_cfg.masterGain * load * rf;
            }

            // ------------------------------------------------- road layer
            float roadT = 0;
            if (sig.speed > 1.0f) {
                float spN = clamp01(sig.speed / m_cfg.maxSpeed);
                float act = clamp01(sig.susp) * clamp01(sig.speed / 5.0f);
                float norm = clamp01(0.15f * spN + 0.85f * act + 0.9f * clamp01(sig.bump));
                roadHz = m_cfg.roadMinHz + (m_cfg.roadMaxHz - m_cfg.roadMinHz) * spN;
                roadHz *= 1.0f + 0.08f * ((rand() % 2001 - 1000) / 1000.0f);        // grit
                roadT = m_cfg.roadMaxMag * m_cfg.roadGain * m_cfg.masterGain * norm;
            }

            // attack / release smoothing so effects never click
            engSm  += (engT  - engSm)  * (engT  > engSm  ? 0.5f : 0.15f);
            roadSm += (roadT - roadSm) * (roadT > roadSm ? 0.5f : 0.15f);

            // never exceed the device range once both layers are summed with RBR's own force
            float total = engSm + roadSm;
            float lim = (float)std::max(m_cfg.engineMaxMag, m_cfg.roadMaxMag);
            float scale = total > lim * 1.5f ? (lim * 1.5f) / total : 1.0f;
            int engOut = (int)(engSm * scale), roadOut = (int)(roadSm * scale);

            m_engMag = (float)engOut; m_roadMag = (float)roadOut;
            m_engHz = engHz; m_roadHz = roadHz;

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
                            Logf("Effects created");
                        } else {
                            m_ffActive = false;
                            Logf("CreateEffect failed: 0x%08X", (unsigned)hr);
                            if (hr == DIERR_NOTEXCLUSIVEACQUIRED || hr == DIERR_OTHERAPPHASPRIO) {
                                // That device object is not the one holding the wheel exclusively (RBR/NGP may
                                // use several). Let go of it; another object polling the same wheel can be captured.
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
                DWORD engPer = (DWORD)(1000000.0f / std::max(1.0f, engHz));
                DWORD roadPer = (DWORD)(1000000.0f / std::max(1.0f, roadHz));
                if (!ff.Push(engOut, engPer, roadOut, roadPer)) {
                    Logf("Effects lost (0x%08X), recreating", (unsigned)ff.LastHr());
                    ff.Destroy();
                    m_ffActive = false;
                    SetStatus("Effects lost (0x%08X), retrying", (unsigned)ff.LastHr());
                }
            }
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
        bool exists = GetFileAttributesA(m_iniPath.c_str()) != INVALID_FILE_ATTRIBUTES;
        if (!exists) SyncConfig(m_cfg, m_iniPath, true);     // write a template with every key
        SyncConfig(m_cfg, m_iniPath, false);

        SetStatus("Pick the wheel with Left/Right, press Select");
        EnumerateDevices();

        if (m_cfg.autoConnect && !m_cfg.wheel.empty()) {
            std::string want = lower(m_cfg.wheel);
            for (int i = 0; i < (int)m_devices.size(); ++i) {
                if (lower(m_devices[i].name).find(want) != std::string::npos) { m_sel = i; Connect(); break; }
            }
        } else if (!m_cfg.wheel.empty()) {
            std::string want = lower(m_cfg.wheel);
            for (int i = 0; i < (int)m_devices.size(); ++i)
                if (lower(m_devices[i].name).find(want) != std::string::npos) { m_sel = i; break; }
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
        float y = 60.0f;
        game_->SetFont(IRBRGame::FONT_BIG);
        game_->SetMenuColor(IRBRGame::MENU_HEADING);
        game_->WriteText(70.0f, y, "Trueforce Rumble Settings");
        y += 30.0f;

        game_->SetFont(IRBRGame::FONT_SMALL);
        game_->SetMenuColor(IRBRGame::MENU_TEXT);
        char buf[256];

        snprintf(buf, sizeof(buf), "FFB devices: %d   (Left/Right = choose, Select = connect / toggle TEST)", (int)m_devices.size());
        game_->WriteText(70.0f, y, buf); y += 20.0f;

        if (!m_devices.empty()) {
            snprintf(buf, sizeof(buf), "> [%d/%d] %s%s", m_sel + 1, (int)m_devices.size(), m_devices[m_sel].name,
                     m_sel == m_connectedIdx ? "  (connected)" : "");
            game_->SetMenuColor(IRBRGame::MENU_SELECTION);
            game_->WriteText(70.0f, y, buf);
            game_->SetMenuColor(IRBRGame::MENU_TEXT);
        } else {
            game_->WriteText(70.0f, y, "> [no force feedback devices detected]");
        }
        y += 22.0f;

        std::string st;
        { std::lock_guard<std::mutex> lk(m_mtx); st = m_status; }
        snprintf(buf, sizeof(buf), "Status: %s", st.c_str());
        game_->WriteText(70.0f, y, buf); y += 20.0f;

        snprintf(buf, sizeof(buf), "UDP packets: %lld (port %d)   TEST: %s", (long long)m_packets.load(), m_cfg.udpPort,
                 m_test ? "ON" : "off");
        game_->WriteText(70.0f, y, buf); y += 20.0f;

        snprintf(buf, sizeof(buf), "rpm %.0f  speed %.1f m/s  throttle %.2f  road texture %.2f  bump %.2f",
                 m_rRpm.load(), m_rSpeed.load(), m_rThr.load(), m_rSusp.load(), m_rBump.load());
        game_->WriteText(70.0f, y, buf); y += 20.0f;

        snprintf(buf, sizeof(buf), "Engine %.0f Hz / mag %.0f     Road %.0f Hz / mag %.0f",
                 m_engHz.load(), m_engMag.load(), m_roadHz.load(), m_roadMag.load());
        game_->WriteText(70.0f, y, buf); y += 26.0f;

        // calibration table: Up/Down scroll through the raw packet floats
        uint8_t pkt[2048]; int len;
        { std::lock_guard<std::mutex> lk(m_mtx); len = m_pktLen; std::memcpy(pkt, m_pkt, sizeof(pkt)); }
        snprintf(buf, sizeof(buf), "Raw packet floats (Up/Down scroll); expected: rpm @0x88, speed @0x3C, throttle @0x1C:");
        game_->WriteText(70.0f, y, buf); y += 18.0f;
        for (int row = 0; row < 6; ++row) {
            int base = m_tableBase + row * 16;
            char line[200]; int p = snprintf(line, sizeof(line), "0x%03X ", base);
            for (int c = 0; c < 4; ++c) {
                int off = base + c * 4;
                if (off + 4 <= len) {
                    float v; std::memcpy(&v, pkt + off, 4);
                    p += snprintf(line + p, sizeof(line) - p, " %10.4g", std::isfinite(v) ? (double)v : 0.0);
                } else {
                    p += snprintf(line + p, sizeof(line) - p, " %10s", "-");
                }
            }
            game_->WriteText(70.0f, y, line); y += 17.0f;
        }
    }

    void DrawResultsUI() override {}

    void HandleFrontEndEvents(char, bool bUp, bool bDown, bool bLeft, bool bRight, bool bSelect) override {
        if (bUp)   m_tableBase = std::max(0, m_tableBase - 96);
        if (bDown) m_tableBase = std::min(2048 - 96, m_tableBase + 96);
        if (m_devices.empty()) return;

        if (bLeft)  { m_sel = (m_sel - 1 + (int)m_devices.size()) % (int)m_devices.size(); }
        if (bRight) { m_sel = (m_sel + 1) % (int)m_devices.size(); }
        if (bSelect) {
            if (m_connectedIdx == m_sel && m_run) m_test = !m_test;   // already connected: toggle synthetic test vibration
            else Connect();
        }
    }

    void TickFrontEndPage(float) override {}
    void StageStarted(int, const char*, bool) override {}
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
