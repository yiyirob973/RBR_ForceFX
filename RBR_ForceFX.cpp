// RBR_ForceFX.cpp
//
// Extra force feedback for Richard Burns Rally (RSF with NGP7). It reads NGP's telemetry and adds engine,
// road and braking vibration on top of RBR's own steering force. When the car lands from a jump or takes a hard
// hit, it also makes RBR's own force pull harder for a moment.
//
// How it reaches the wheel
//   RBR already holds the wheel exclusively, so a second connection from us could never create effects.
//   Instead we patch a few IDirectInputDevice8 methods (Acquire, Unacquire, GetDeviceState, GetDeviceData, Poll),
//   spot RBR's own device object by its instance GUID, and create our effects on that same object. The wheel
//   adds them to RBR's force.
//
// The effects
//   Engine   A sine at the engine's firing frequency, squeezed into the range a wheel can actually reproduce so
//            the pitch never jumps. Strength follows rpm, throttle load and a few engine resonances, and the
//            rev limiter makes it stutter.
//   Lump     A second, lower sine at half the firing order. It gives you the lumpy idle and, at high revs, a
//            slightly detuned tone that beats against the main one so the engine doesn't feel like a pure sine.
//   Road     Three sine effects, each carrying one band (low, mid, high) of the road's spectrum, with a bit of
//            random pitch and strength jitter. Their pitch follows the car's speed, and the tyre's contact patch
//            takes the high bands away at low speed (road_bands.hpp). How strong the road is follows how rough the
//            tyres feel (high-passed strut force relative to the average load, the same idea RbrSurfaceAudio uses)
//            plus damper velocity. Smooth tarmac is silent and rough ground or kerbs turn violent and sharp. What
//            the road feels like comes from the ground under each tyre (how hard, loose, coarse and slippery it
//            is, from the game's material names) together with speed and wheel slip, not from a list of surfaces;
//            see road_feel.hpp. The road gain is the one setting for it.
//   Brake    A sine that grows with the brake pedal and with weight shifting onto the front tyres. Front wheel
//            lock-up (tyre segment counter vs car speed) makes it much harsher.
//   Landing  No effect of its own. When a landing or hard hit is detected, a hook on RBR's constant-force updates
//            scales RBR's own force up for a split second and adds a minimum push in the direction RBR is
//            already pulling.
//
//   Engine, lump, road and brake are all zero-mean vibrations, so they never pull the wheel to one side. The
//   landing boost is the only thing that touches RBR's own steering force.
//
// Building (RBR is 32-bit, so build x86 with the static runtime):
//   cl /std:c++17 /EHsc /LD /MT /O2 RBR_ForceFX.cpp /Fe:RBR_ForceFX.dll /link /DEF:RBR_ForceFX.def
//
// Telemetry: NGP needs UDP telemetry switched on, but the destination doesn't matter. ForceFX picks the packets up as
//   NGP sends them, by watching the Winsock send calls inside the RBR process (see PacketTap), so it needs no UDP port
//   of its own and won't fight SimHub or anything else for one. As a backup it also listens on UdpPort, so a setup where
//   the hook can't see NGP's packets still works if NGP is pointed at that port, for example
//   udpTelemetryEndpoints=127.0.0.1:6776,127.0.0.1:6778. TelemetrySource in ForceFX.ini chooses hook, UDP, or both (default).
//   The packet layout is NGP 7.4.778's TelemetryData (664 bytes). Offsets are configurable in ForceFX.ini.
//
// Live level while driving: Numpad + and - change the overall level in equal-loudness steps (x or / 0.85) and
//   Numpad * mutes or unmutes. The keys only work while RBR is the foreground window, and the level can only go
//   down from the configured ceilings, never above them. Keys and an optional modifier are set in ForceFX.ini.
//
// Safety: hard compile-time ceilings (kHardTotalMax and friends), a frequency floor, a slow thermal derate,
//   loopback-only telemetry (the send hook ignores anything that isn't a loopback UDP packet), a fade to zero on
//   shutdown, and an exception guard around the worker thread.
//
// Car profiles: the car is identified by its name (rallysimfans.ini slot -> Cars\Cars.ini CarName / RSFCarID)
//   and matched loosely against the Name= entries in ForceFXCars.ini. Each profile sets the engine character
//   (cylinders, rev limit, roughness) and a weight feel.
//
// Weight: each wheel's strut load is compared with the car's own static load, which drives road strength (the
//   more heavily loaded side transmits more), the compression thump, a quiet engine and road when airborne, and
//   the braking judder.
//
// Files next to the DLL: ForceFX.ini (settings, written with defaults on first run), ForceFXCars.ini (car
//   profiles), ForceFX.log, and ForceFX_packet.txt (a dump of the first telemetry packet, handy for checking offsets).

#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define DIRECTINPUT_VERSION 0x0800
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <tlhelp32.h>
#include <dinput.h>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <set>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <algorithm>

#define __SUPPORT_PLUGIN__ 1
#include "IRBRGame.h"
#include "IPlugin.h"
#include "materials.hpp"      // ground material table, shared by the surface feel and the audio
#include "audio_module.hpp"   // dynamic surface audio: only built in when FORCEFX_AUDIO is defined (off until the menu switch is turned on)
#include "camera_module.hpp"  // head movement and windscreen effects: only built in when FORCEFX_CAMERA is defined
#include "wheel_slip.hpp"   // wheel slip from the segment counter, shared with the audio module
#include "tyre_motion.hpp"  // drivetrain, per-tyre ground speed, grip, wheelspin vs slides (shared with the audio module)

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")

namespace {
#include "road_guard.hpp"   // keeps the road vibration from pushing or pulling the wheel (low cut, headroom, ceiling)

typedef std::chrono::steady_clock Clock;

// Hard ceilings: the .ini can never push past these, whatever it says.
constexpr int   kHardTotalMax = 5000;    // sum of all layer magnitudes (0..10000 DirectInput units)
constexpr float kHardMinHz    = 12.0f;   // below this a vibration becomes a visible wheel swing
constexpr float kHardMaxHz    = 120.0f;
constexpr int   kMagDeadband  = 60;      // skip USB updates for changes smaller than this
constexpr int   kMinUpdateGapMs = 15;    // an effect is updated at most this often (about every other 10 ms step), so the wheel has room for RBR's own force updates
constexpr int   kPushMinForce = 300;     // RBR force magnitude (of 10000) below which the landing push is not added
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

// Keeps our DLL loaded for the rest of the process. Used when another hook was chained after ours and we can't unhook cleanly.
void PinThisDll() {
    HMODULE h = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN, (LPCSTR)&pluginDirectory, &h);
}

void Logf(const char* fmt, ...) {
    char b[512];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    FILE* f = fopen((g_dir + "ForceFX.log").c_str(), "a");
    if (f) { fprintf(f, "%s\n", b); fclose(f); }
}

// ------------------------------------------------------------------- config
struct Config {
    // These defaults match the shipped ForceFX.ini, so "Reset" in the menu gives you a fresh-install setup.

    // telemetry connection
    int   telemetrySource = 0;       // 0 = auto (NGP's send hook, UDP port as backup), 1 = send hook only, 2 = UDP port only
    int   udpPort = 6778;            // backup listening port (0 = never open one). SimHub usually uses 6776 and RbrSurfaceAudio 6777: never share a port
    int   minPacket = 664;
    int   cylinders = 4;             // engine pulses per rev = cylinders / 2

    // effect switches (menu): 1 = on, 0 = off
    int   fxEngine = 1, fxRoad = 1, fxBrake = 1, fxLandBoost = 1, fxSurface = 1;
    int   roadViaRbr = 1;            // 1 = the road vibration is added to RBR's own force updates, so the wheel gets no extra road effect; 0 = one sine effect of its own
    int   probe = 0;                 // 1 = write ForceFX_probe.csv (ground ids and road signals ten times a second) for tuning surfaces; read at start only
#ifdef FORCEFX_AUDIO
    int   fxAudio = 0;               // dynamic surface audio (settings in the [Audio] section of ForceFX.ini); off until switched on
    float audioDebris = 1.0f;        // 0..4 live multiplier on the stone / debris sounds (on top of StoneLevel in [Audio] of ForceFX.ini)
    float audioImpact = 1.0f;        // 0..4 live multiplier on impacts and landings (on top of EventLevel)
    float audioSusp = 1.0f;          // 0..4 live multiplier on suspension creaks and bottoming out (on top of SqueakLevel / BottomOutLevel)
#endif
#ifdef FORCEFX_CAMERA
    int   fxCamera = 1;              // head movement in the cockpit camera (settings in the [Camera...] sections of ForceFX.ini)
    int   fxWindscreen = 0;          // rain, snow and mud on the windscreen; off by default. Its hooks are installed at startup, so switching it on needs a restart
#endif

    // per-layer ceilings (DirectInput units 0..10000), a combined ceiling, and one live level (userGain)
    int   engineMaxMag = 1200, roadMaxMag = 500, brakeMaxMag = 2000;
    int   totalMaxMag = 4000;        // all layers together are scaled down to this (never above kHardTotalMax)
    float priorityDuck = 0.85f;      // 0..1: how far lower layers back off while a higher one is active (landing > road > engine)
    float sustainedLimit = 0.10f;    // thermal derate: long-term mean-square level (0.10 is roughly 32% RMS)

    // landings and hard hits
    float impactGain = 0.7f;         // landing sensitivity 0..3 (0 = off)
    float airLevel = 0.2f;           // 0.2..0.8: total strut load (fraction of static) below which the car counts as airborne; raise it to catch smaller jumps
    float landingThresh = 3.0f;      // 0.5..3: how hard a hit without airtime must be to count as a landing (higher = fewer false triggers)
    float landingBoost = 1.1f;       // 0..4: extra RBR steering torque at the peak of a full-severity landing (2 = +200%)
    float landingAdd = 0.15f;        // 0..1: minimum extra push (share of full torque) in the direction RBR is already pushing
    float thumpGain = 1.4f;          // compression / landing thump added to the road layer

    // brake
    float brakeHzScale = 1.0f;       // judder pitch multiplier 0.5..2
    float tyreCircumference = 1.95f; // metres, until the plugin has measured it while coasting

    // engine
    float engineMinHz = 14.0f, engineMaxHz = 100.0f;
    float rpmMax = 0.0f;             // fallback rev limit for when telemetry has none (0 = learn it from the data, starting at rpmMaxInit)
    float rpmMaxInit = 7500.0f;
    float engineGrit = 0.5f;         // 0 = clean sine at high revs, 1 = rough, beating, irregular

    // road
    float roadMinHz = 25.0f, roadMaxHz = 100.0f;
    float roughLow = 0.03f, roughHigh = 0.35f, roughGamma = 1.0f;   // tyre roughness -> 0..1 level (tarmac sits below roughLow)
    float pistonScale = 1.0f;        // damper piston velocity -> 0..1 (hard hits, kerbs)
    float bumpScale = 0.25f;         // heave acceleration spikes -> 0..1
    float maxSpeed = 40.0f;          // m/s that counts as full speed

    // weight and suspension load (relative to the car's own static load, so units never matter)
    float loadInfluence = 0.6f;      // 0 = ignore tyre load, 1 = strongly favour the more heavily loaded side
    float airborneFade = 1.0f;       // 1 = road and engine go quiet when the wheels are off the ground

    // per-car character (these are the global values; a car profile can override them)
    float weightFeel = 1.2f;         // >1 heavier: stronger thump, slower decay, lower road pitch
    float lumpScale = 1.0f;          // idle lump / beat strength
    float unevenness = 0.0f;         // uneven firing (boxer, five-cylinder, cross-plane V8)
    int   carProfiles = 1;           // use ForceFXCars.ini
    // Which tyres the engine drives (Tyre::FWD / RWD / AWD). Only driven tyres can have wheelspin. RWD unless the car's profile in
    // ForceFXCars.ini says otherwise. Car geometry for the per-tyre slip angles (a profile can set it per car).
    int   drivetrain = Tyre::kDefaultDrivetrain;
    float wheelbase = 2.5f, cgToFront = 1.25f, trackWidth = 1.5f;   // metres

    // NGP packet offsets (float32 unless noted)
    int   offCarIndex = 56;
    int   offSteps = 0;              // totalSteps_: NGP's own step counter, a clean clock for counting wheel turns (-1 = not used)
    int   offVelSurge = 88, offVelSway = 92;   // the car's velocity over the ground, m/s: what wheel speed is compared with
    int   offThrottle = 28, offBrake = 32, offSpeed = 60, offAccHeave = 120, offRpm = 136;
    int   offHandbrake = 36, offSteering = 24, offYawRate = 108, offAccSurge = 112, offAccSway = 116;   // for the per-tyre motion (slip angle, lock-up, grip)
    int   offRpmMax = -1;            // float32 rev limit in the packet (-1 = this packet doesn't have one)
    int   offSuspBase = 152, suspStride = 128, offStrut = 16, offPiston = 28, offSegment = 60;   // segment = uint32
    int   speedIsKph = 0;
    float rpmScale = 1.0f;

    // live level and hotkeys (polled while RBR is the foreground window). Keys are VK codes, 0 = disabled.
    float userGain = 0.85f;          // saved live level 0..1 (it can only turn things down)
    float gainRatio = 0.85f;         // each key press multiplies or divides the level by this (about 1.4 dB per step)
    int   keyGainUp = 0x6B, keyGainDown = 0x6D, keyMute = 0x6A;   // Numpad + / - / *
    int   keyModifier = 0;           // 0 = no modifier (0x11 = Ctrl)
    int   gainBeep = 1;

    int         autoConnect = 0;
    std::string wheel;               // name of the wheel to use, saved by the menu
};

// Pulls every setting back into its safe range. Runs after the ini is read and after every menu edit.
void ClampConfig(Config& c) {
    c.telemetrySource = std::max(0, std::min(c.telemetrySource, 2));
    c.udpPort = std::max(0, std::min(c.udpPort, 65535));
    c.minPacket = std::max(64, std::min(c.minPacket, 1024));
    c.cylinders = std::max(1, std::min(c.cylinders, 16));
    c.engineMaxMag = std::max(0, std::min(c.engineMaxMag, 10000));
    c.roadMaxMag = std::max(0, std::min(c.roadMaxMag, 10000));
    c.brakeMaxMag = std::max(0, std::min(c.brakeMaxMag, 10000));
    c.impactGain = clampf(c.impactGain, 0.0f, 3.0f);
    c.brakeHzScale = clampf(c.brakeHzScale, 0.5f, 2.0f);
    c.drivetrain = std::max((int)Tyre::FWD, std::min(c.drivetrain, (int)Tyre::AWD));
    c.wheelbase = clampf(c.wheelbase, 1.5f, 4.0f); c.trackWidth = clampf(c.trackWidth, 1.0f, 2.2f); c.cgToFront = clampf(c.cgToFront, 0.4f * c.wheelbase, 0.6f * c.wheelbase);
    c.priorityDuck = clampf(c.priorityDuck, 0.0f, 1.0f);
    c.fxEngine = c.fxEngine ? 1 : 0; c.fxRoad = c.fxRoad ? 1 : 0; c.fxBrake = c.fxBrake ? 1 : 0; c.fxLandBoost = c.fxLandBoost ? 1 : 0; c.fxSurface = c.fxSurface ? 1 : 0; c.roadViaRbr = c.roadViaRbr ? 1 : 0;
#ifdef FORCEFX_AUDIO
    c.fxAudio = c.fxAudio ? 1 : 0;
    c.audioDebris = clampf(c.audioDebris, 0.0f, 4.0f); c.audioImpact = clampf(c.audioImpact, 0.0f, 4.0f); c.audioSusp = clampf(c.audioSusp, 0.0f, 4.0f);
#endif
#ifdef FORCEFX_CAMERA
    c.fxCamera = c.fxCamera ? 1 : 0; c.fxWindscreen = c.fxWindscreen ? 1 : 0;
#endif
    c.landingBoost = clampf(c.landingBoost, 0.0f, 4.0f);
    c.landingAdd = clampf(c.landingAdd, 0.0f, 1.0f);
    c.landingThresh = clampf(c.landingThresh, 0.5f, 3.0f);
    c.airLevel = clampf(c.airLevel, 0.2f, 0.8f);
    c.totalMaxMag = std::max(500, std::min(c.totalMaxMag, kHardTotalMax));
    c.userGain   = clampf(c.userGain,   0.0f, 1.0f);
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
    if (c.engineMaxHz < c.engineMinHz + 5.0f) { c.engineMaxHz = std::min(c.engineMinHz + 5.0f, kHardMaxHz); c.engineMinHz = c.engineMaxHz - 5.0f; }
    if (c.roughHigh < c.roughLow + 0.05f) c.roughHigh = c.roughLow + 0.05f;
    if (c.maxSpeed < 1.0f) c.maxSpeed = 1.0f;
    if (c.tyreCircumference < 0.5f) c.tyreCircumference = 1.95f;
}

void SyncConfig(Config& c, const std::string& path, bool write) {
    const char* S = "ForceFX";
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
    I("TelemetrySource", c.telemetrySource);  I("UdpPort", c.udpPort);  I("MinPacketBytes", c.minPacket);  I("Cylinders", c.cylinders);
    I("EngineMaxMagnitude", c.engineMaxMag); I("RoadMaxMagnitude", c.roadMaxMag);
    I("BrakeMaxMagnitude", c.brakeMaxMag); I("TotalMaxMagnitude", c.totalMaxMag);
    F("ImpactGain", c.impactGain);
    F("BrakeHzScale", c.brakeHzScale); F("PriorityDuck", c.priorityDuck);
    I("EnableEngine", c.fxEngine); I("EnableRoad", c.fxRoad); I("RoadViaRbr", c.roadViaRbr); I("EnableBrake", c.fxBrake);
    I("SurfaceProbe", c.probe);
#ifdef FORCEFX_AUDIO
    I("EnableDynamicAudio", c.fxAudio); F("AudioDebrisLevel", c.audioDebris); F("AudioImpactLevel", c.audioImpact); F("AudioSuspensionLevel", c.audioSusp);
#endif
#ifdef FORCEFX_CAMERA
    I("EnableCamera", c.fxCamera); I("EnableWindscreen", c.fxWindscreen);
#endif
    I("EnableSurfaceFeel", c.fxSurface); I("EnableLandingBoost", c.fxLandBoost); F("LandingBoost", c.landingBoost); F("LandingMinPush", c.landingAdd); F("LandingThreshold", c.landingThresh); F("LandingAirLevel", c.airLevel);
    F("EngineMinHz", c.engineMinHz); F("EngineMaxHz", c.engineMaxHz);
    F("RpmMax", c.rpmMax); F("RpmMaxInit", c.rpmMaxInit);
    F("RoadMinHz", c.roadMinHz); F("RoadMaxHz", c.roadMaxHz);
    F("RoughLow", c.roughLow); F("RoughHigh", c.roughHigh); F("RoughGamma", c.roughGamma);
    F("EngineGrit", c.engineGrit);
    F("LoadInfluence", c.loadInfluence); F("ThumpGain", c.thumpGain); F("AirborneFade", c.airborneFade);
    F("WeightFeel", c.weightFeel); F("LumpScale", c.lumpScale); F("Unevenness", c.unevenness);
    I("CarProfiles", c.carProfiles); I("OffsetCarIndex", c.offCarIndex); I("OffsetSteps", c.offSteps); I("OffsetVelSurge", c.offVelSurge); I("OffsetVelSway", c.offVelSway);
    F("PistonScale", c.pistonScale); F("BumpScale", c.bumpScale); F("MaxSpeed", c.maxSpeed);
    F("TyreCircumference", c.tyreCircumference);
    F("Wheelbase", c.wheelbase); F("CgToFront", c.cgToFront); F("TrackWidth", c.trackWidth);
    I("OffsetThrottle", c.offThrottle); I("OffsetBrake", c.offBrake); I("OffsetSpeed", c.offSpeed);
    I("OffsetAccHeave", c.offAccHeave); I("OffsetHandbrake", c.offHandbrake); I("OffsetSteering", c.offSteering); I("OffsetYawRate", c.offYawRate); I("OffsetAccSurge", c.offAccSurge); I("OffsetAccSway", c.offAccSway); I("OffsetRpm", c.offRpm); I("OffsetRpmMax", c.offRpmMax);
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

    if (!write) ClampConfig(c);
}

// ------------------------------------------------------------ per-car profiles
// The car is identified by its name, the way RSF and RBR store it: rallysimfans.ini [cars] picks the active slot, and
// Cars\Cars.ini [Car0<slot>] holds CarName and RSFCarID. ForceFXCars.ini then has any number of sections, each with
// Name= (and optionally Match=a,b and RSFCarID=) plus the settings for that car. Names are compared loosely (case,
// punctuation, "Grp"/"Group" spelling and missing manufacturer words don't matter), see MatchScore.
struct CarProfile {
    bool found = false;
    std::string section, name, preset;
    std::vector<std::string> alias;  // extra names that should also match this profile
    int   rsfId = -1;                // exact RSF car id match (strongest)
    int   cylinders = 0;             // 0 = keep global
    float rpmMax = 0;                // 0 = keep global / learn
    float engineScale = 1.0f, roadScale = 1.0f;
    float lumpScale = -1, grit = -1, unevenness = -1, weightFeel = -1;   // negative = keep global
    int   drivetrain = -1;           // Tyre::FWD / RWD / AWD; -1 = not given (rear-wheel drive)
    float wheelbase = -1, cgToFront = -1, trackWidth = -1;   // metres; negative = keep global
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

std::string CarsIniPath() { return g_dir + "ForceFXCars.ini"; }
std::string LowerStr(std::string v) { for (char& ch : v) ch = (char)tolower((unsigned char)ch); return v; }
std::string TrimStr(const std::string& v) {
    size_t a = 0, b = v.size();
    while (a < b && isspace((unsigned char)v[a])) ++a;
    while (b > a && isspace((unsigned char)v[b - 1])) --b;
    return v.substr(a, b - a);
}

// "Ford Escort RS Cosworth GrpA" -> {ford, escort, rs, cosworth, gra}
std::vector<std::string> NameTokens(const std::string& s) {
    std::vector<std::string> out; std::string cur;
    auto flush = [&]() {
        if (cur.empty()) return;
        if (cur.rfind("grp", 0) == 0 && cur.size() <= 5) cur = "gr" + cur.substr(3);          // grpa -> gra, grp4 -> gr4
        else if (cur.rfind("group", 0) == 0 && cur.size() <= 7) cur = "gr" + cur.substr(5);   // groupa -> gra
        out.push_back(cur); cur.clear();
    };
    for (unsigned char ch : s) { if (isalnum(ch)) cur.push_back((char)tolower(ch)); else flush(); }
    flush();
    return out;
}

// Tokens that tell two cars of the same model apart (years, model numbers, roman numerals): they must agree.
bool IsDiscriminator(const std::string& t) {
    for (char ch : t) if (isdigit((unsigned char)ch)) return true;
    static const char* roman[] = { "ii", "iii", "iv", "vi", "vii", "viii", "ix", "x" };
    for (const char* r : roman) if (t == r) return true;
    return false;
}

// Higher is better, -1 means "not the same car". Every token of the shorter name has to appear in the longer one (so the
// game's "Escort Cosworth GrA" matches "Ford Escort RS Cosworth GrpA"), and distinguishing tokens have to agree.
float MatchScore(const std::vector<std::string>& a, const std::vector<std::string>& b) {
    if (a.empty() || b.empty()) return -1.0f;
    const std::set<std::string> A(a.begin(), a.end()), B(b.begin(), b.end());
    const std::set<std::string>& shortSet = A.size() <= B.size() ? A : B;
    const std::set<std::string>& longSet   = A.size() <= B.size() ? B : A;
    int inter = 0;
    for (const std::string& t : shortSet) {
        if (longSet.count(t)) ++inter;
        else if (IsDiscriminator(t)) return -1.0f;
    }
    const float contain = (float)inter / (float)shortSet.size();
    if (contain < 0.8f) return -1.0f;
    const float jac = (float)inter / (float)(A.size() + B.size() - inter);
    return contain + 0.5f * jac;
}

struct CarDb {
    std::vector<CarProfile> list;
    FILETIME stamp{};
    bool loaded = false;

    bool Reload() {                                           // re-reads the file only when it changed; true = it was (re)loaded
        const std::string path = CarsIniPath();
        WIN32_FILE_ATTRIBUTE_DATA fa;
        if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &fa)) { const bool had = !list.empty(); list.clear(); loaded = false; return had; }
        if (loaded && CompareFileTime(&fa.ftLastWriteTime, &stamp) == 0) return false;
        stamp = fa.ftLastWriteTime; loaded = true; list.clear();
        FILE* f = fopen(path.c_str(), "rb");
        if (!f) { loaded = false; return true; }                // being written right now: try again on the next poll
        std::string text; char chunk[4096]; size_t n;
        while ((n = fread(chunk, 1, sizeof(chunk), f)) > 0) text.append(chunk, n);
        fclose(f);

        typedef std::vector<std::pair<std::string, std::string> > KV;
        std::vector<std::pair<std::string, KV> > secs;        // own parser: duplicate section names are allowed
        size_t pos = 0;
        while (pos <= text.size()) {
            size_t e = text.find('\n', pos); if (e == std::string::npos) e = text.size();
            std::string line = TrimStr(text.substr(pos, e - pos));
            pos = e + 1;
            if (line.empty() || line[0] == ';' || line[0] == '#') continue;
            if (line[0] == '[') { size_t r = line.find(']'); secs.push_back(std::make_pair(TrimStr(line.substr(1, r == std::string::npos ? std::string::npos : r - 1)), KV())); continue; }
            size_t eq = line.find_first_of("=:");               // "Drivetrain: AWD" works too
            if (eq == std::string::npos || secs.empty()) continue;
            std::string v = TrimStr(line.substr(eq + 1));
            if (v.size() >= 2 && v.front() == '"' && v.back() == '"') v = v.substr(1, v.size() - 2);
            secs.back().second.push_back(std::make_pair(LowerStr(TrimStr(line.substr(0, eq))), v));
        }
        for (auto& sec : secs) {
            CarProfile p; p.section = sec.first;
            bool identified = false;
            for (auto& kv : sec.second) {                       // identity + preset first, so explicit keys can override the preset
                if (kv.first == "name") { p.name = kv.second; identified = true; }
                else if (kv.first == "match") {
                    std::string cur;
                    for (char ch : kv.second + ",") { if (ch == ',' || ch == '|') { cur = TrimStr(cur); if (!cur.empty()) p.alias.push_back(cur); cur.clear(); } else cur.push_back(ch); }
                    identified = true;
                }
                else if (kv.first == "rsfcarid") { p.rsfId = atoi(kv.second.c_str()); identified = true; }
                else if (kv.first == "preset") {
                    p.preset = kv.second;
                    for (const Preset& pr : kPresets) if (LowerStr(kv.second) == pr.name) {
                        p.cylinders = pr.cyl; p.lumpScale = pr.lump; p.grit = pr.grit; p.unevenness = pr.uneven; p.engineScale = pr.engine;
                    }
                }
            }
            if (!identified) continue;
            for (auto& kv : sec.second) {
                const float fv = (float)atof(kv.second.c_str());
                if (kv.first == "cylinders") p.cylinders = atoi(kv.second.c_str());
                else if (kv.first == "rpmmax") p.rpmMax = fv;
                else if (kv.first == "enginescale") p.engineScale = fv;
                else if (kv.first == "roadscale") p.roadScale = fv;
                else if (kv.first == "lumpscale") p.lumpScale = fv;
                else if (kv.first == "grit") p.grit = fv;
                else if (kv.first == "unevenness") p.unevenness = fv;
                else if (kv.first == "weightfeel") p.weightFeel = fv;
                else if (kv.first == "drivetrain" || kv.first == "drivetrains" || kv.first == "drive" || kv.first == "drivetype") p.drivetrain = Tyre::ParseDrivetrain(kv.second, -1);
                else if (kv.first == "wheelbase") p.wheelbase = fv;
                else if (kv.first == "cgtofront") p.cgToFront = fv;
                else if (kv.first == "trackwidth") p.trackWidth = fv;
            }
            p.found = true;
            list.push_back(p);
        }
        Logf("ForceFXCars.ini: %d car profiles", (int)list.size());
        return true;
    }

    // An exact RSFCarID wins; otherwise the best name match.
    const CarProfile* Match(const std::string& name, int rsfId, float& score) const {
        score = 0;
        if (rsfId >= 0) for (const CarProfile& p : list) if (p.rsfId == rsfId) { score = 3.0f; return &p; }
        if (!name.empty()) {
            const std::vector<std::string> g = NameTokens(name);
            const CarProfile* best = nullptr; float bs = -1.0f;
            for (const CarProfile& p : list) {
                float sc = p.name.empty() ? -1.0f : MatchScore(g, NameTokens(p.name));
                for (const std::string& al : p.alias) sc = std::max(sc, MatchScore(g, NameTokens(al)));
                if (sc > bs) { bs = sc; best = &p; }
            }
            if (best && bs >= 0) { score = bs; return best; }
        }
        return nullptr;
    }
};

struct CarIdent { std::string name; int rsfId = -1; };

std::string GameDir() {
    char b[MAX_PATH]; DWORD n = GetModuleFileNameA(NULL, b, MAX_PATH);
    std::string d(b, n); size_t k = d.find_last_of("\\/");
    return k == std::string::npos ? std::string() : d.substr(0, k + 1);
}

// Looks the car up the way RSF does: rallysimfans.ini [cars] slot, then Cars\Cars.ini [Car0<slot>]. Installs without
// rallysimfans.ini (vanilla RBR, RBRCIT) use the telemetry slot instead, and the name falls back to the car's folder.
CarIdent ResolveCarIdentity(int telemSlot) {
    CarIdent id;
    const std::string gd = GameDir();
    if (gd.empty()) return id;
    int slot = (int)GetPrivateProfileIntA("cars", "slot", -1, (gd + "rallysimfans.ini").c_str());
    if (slot < 0 || slot > 7) slot = telemSlot;
    if (slot < 0 || slot > 7) return id;
    char sec[16]; snprintf(sec, sizeof(sec), "Car%02d", slot);
    const std::string ini = gd + "Cars\\Cars.ini";
    char b[260];
    id.rsfId = (int)GetPrivateProfileIntA(sec, "RSFCarID", -1, ini.c_str());
    GetPrivateProfileStringA(sec, "CarName", "", b, sizeof(b), ini.c_str());
    id.name = TrimStr(b);
    if (id.name.empty()) {                                    // no CarName: use the model file name, e.g. ...\Sunny_GTi-R\Sunny_GTi-R.ini
        GetPrivateProfileStringA(sec, "IniFile", "", b, sizeof(b), ini.c_str());
        std::string f = b; size_t k = f.find_last_of("\\/"); if (k != std::string::npos) f = f.substr(k + 1);
        size_t d = f.find_last_of('.'); if (d != std::string::npos) f = f.substr(0, d);
        for (char& ch : f) if (ch == '_') ch = ' ';
        id.name = TrimStr(f);
    }
    return id;
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
    if (p.drivetrain >= 0)  e.drivetrain = p.drivetrain;            // otherwise whatever the global config says (rear-wheel drive)
    if (p.wheelbase > 0)    e.wheelbase = p.wheelbase;
    if (p.cgToFront > 0)    e.cgToFront = p.cgToFront;
    if (p.trackWidth > 0)   e.trackWidth = p.trackWidth;
    ClampConfig(e);
}

// ------------------------------------------------ landing boost on RBR's own force
// RBR drives the wheel with its own DirectInput constant-force effect. Rather than stacking another effect on top, a
// detected landing scales that force on its way to the wheel: every SetParameters call RBR makes on a constant-force
// effect is multiplied by g_boostMul, plus a minimum extra push (g_boostAdd, a share of full torque) in the direction
// RBR is already pushing. Our own effects are periodic, so they never go through this path.
std::atomic<float> g_boostMul{1.0f}, g_boostAdd{0.0f};
std::atomic<int>   g_fxUpdates{0};                              // effect updates sent to the wheel; the Diagnostics page shows them per second
std::atomic<int>   g_rbrSeen{0}, g_rbrRaw{0}, g_rbrOut{0};      // for the diagnostics page: updates seen, last magnitude in and out
// Road vibration through RBR's own force: the sine bands are computed here and added to the magnitude RBR sends, so the wheel
// sees RBR's one constant-force effect and nothing extra for the road. RBR only updates its force every so often, so a small
// thread (RoadVib, below) repeats the last force plus the vibration on that same effect a few hundred times a second.
inline long long SteadyMs() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::atomic<IDirectInputEffect*> g_rbrFx{nullptr};              // RBR's constant-force effect, as last seen in SetParameters
std::atomic<long long> g_rbrFxMs{0};                            // when RBR last updated it
std::atomic<int>   g_rbrBase{0};                                // RBR's force after the landing boost, without the road vibration
std::atomic<int>   g_vibNow{0}, g_vibSends{0};                  // the road vibration added right now; sends of it to the wheel (counted for the diagnostics page)
std::atomic<bool>  g_vibOn{false};
typedef HRESULT (WINAPI *PFN_SetParams)(IDirectInputEffect*, LPCDIEFFECT, DWORD);
enum { VT_FX_SETPARAMETERS = 6 };

struct EffectTap {
    static void** vtbl;
    static void*  orig;
    static HRESULT WINAPI HSetParameters(IDirectInputEffect* self, LPCDIEFFECT peff, DWORD flags) {
        if (peff && (flags & DIEP_TYPESPECIFICPARAMS) && peff->cbTypeSpecificParams == sizeof(DICONSTANTFORCE) &&
            peff->lpvTypeSpecificParams && peff->dwSize >= 48 && peff->dwSize <= 256) {   // size check: we copy the struct into a local buffer below
            const LONG raw = static_cast<const DICONSTANTFORCE*>(peff->lpvTypeSpecificParams)->lMagnitude;
            if (g_rbrSeen.fetch_add(1) == 0) Logf("Hook sees RBR's constant-force updates (first magnitude %ld)", (long)raw);
            const float mul = g_boostMul.load(), add = g_boostAdd.load();
            LONG out = raw;
            if (mul != 1.0f || add > 0.0f) {
                float f = (float)raw * mul;
                if (raw > kPushMinForce) f += add * 10000.0f; else if (raw < -kPushMinForce) f -= add * 10000.0f;   // no push while RBR's force is about zero: its sign is noise there
                out = (LONG)clampf(f, -10000.0f, 10000.0f);
            }
            g_rbrBase = (int)out; g_rbrFx = self; g_rbrFxMs = SteadyMs();
            if (g_vibOn.load()) out = (LONG)clampf((float)out + (float)Road::FitHeadroom((int)out, g_vibNow.load()), -10000.0f, 10000.0f);
            g_rbrRaw = (int)raw; g_rbrOut = (int)out;
            if (out != raw) {
                DICONSTANTFORCE cf{}; cf.lMagnitude = out;
                uint8_t buf[256];
                std::memcpy(buf, peff, peff->dwSize);
                reinterpret_cast<DIEFFECT*>(buf)->lpvTypeSpecificParams = &cf;
                return ((PFN_SetParams)orig)(self, reinterpret_cast<LPCDIEFFECT>(buf), flags);
            }
        }
        return ((PFN_SetParams)orig)(self, peff, flags);
    }
    // vt = vtable of any effect created on RBR's device (RBR's own effects share the same class)
    static bool Patch(void** vt) {
        if (vtbl == vt) return true;
        if (vtbl) return false;
        DWORD old;
        if (!VirtualProtect(&vt[VT_FX_SETPARAMETERS], sizeof(void*), PAGE_READWRITE, &old)) return false;
        orig = vt[VT_FX_SETPARAMETERS];
        vt[VT_FX_SETPARAMETERS] = (void*)&HSetParameters;
        VirtualProtect(&vt[VT_FX_SETPARAMETERS], sizeof(void*), old, &old);
        vtbl = vt;
        Logf("Effect hook installed");
        return true;
    }
    static bool Unpatch() {
        g_boostMul = 1.0f; g_boostAdd = 0.0f;
        g_vibOn = false; g_vibNow = 0; g_rbrFx = nullptr;
        if (!vtbl) return true;
        bool clean = vtbl[VT_FX_SETPARAMETERS] == (void*)&HSetParameters;
        if (clean) {
            DWORD old;
            if (orig && VirtualProtect(&vtbl[VT_FX_SETPARAMETERS], sizeof(void*), PAGE_READWRITE, &old)) {
                vtbl[VT_FX_SETPARAMETERS] = orig;
                VirtualProtect(&vtbl[VT_FX_SETPARAMETERS], sizeof(void*), old, &old);
            }
            vtbl = nullptr;
        }
        return clean;
    }
};
void** EffectTap::vtbl = nullptr;
void*  EffectTap::orig = nullptr;

namespace RoadVib {
std::atomic<int>   g_mag[3];                                    // sine amplitude of each road band, wheel units (set by the worker thread)
std::atomic<float> g_hz[3];
std::atomic<bool>  g_run{false};
std::thread        g_thread;
constexpr int       kStreamMs = 3;                              // one send about every 3 ms
constexpr long long kFreshMs  = 100;                            // RBR must have updated its force this recently, or we leave it alone

// True while RBR is updating its force often enough for the road vibration to ride along on it.
bool Ready() { return g_rbrFx.load() != nullptr && SteadyMs() - g_rbrFxMs.load() < kFreshMs; }

// Changes only the magnitude of RBR's effect. Calls the original SetParameters, so our own send does not go through the hook.
bool Send(IDirectInputEffect* fx, int magnitude) {
    DICONSTANTFORCE cf{}; cf.lMagnitude = magnitude;
    DIEFFECT e{}; e.dwSize = sizeof(DIEFFECT); e.cbTypeSpecificParams = sizeof(cf); e.lpvTypeSpecificParams = &cf;
    HRESULT hr = E_FAIL;
    __try { hr = ((PFN_SetParams)EffectTap::orig)(fx, &e, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART); }
    __except (EXCEPTION_EXECUTE_HANDLER) { g_rbrFx = nullptr; return false; }       // RBR released the effect
    return SUCCEEDED(hr);
}

void Loop() {
    typedef UINT (WINAPI *PFN_Period)(UINT);
    HMODULE wm = LoadLibraryA("winmm.dll");                    // loaded by name so the build needs no extra library
    PFN_Period begin = wm ? (PFN_Period)GetProcAddress(wm, "timeBeginPeriod") : nullptr;
    PFN_Period end   = wm ? (PFN_Period)GetProcAddress(wm, "timeEndPeriod") : nullptr;
    if (begin) begin(1);                                        // 1 ms timer, otherwise Sleep(3) would take 15 ms
    float ph[3] = {0.0f, 0.0f, 0.0f};
    Road::LowCut lowCut;
    auto last = Clock::now();
    int lastSent = 0;
    while (g_run.load()) {
        Sleep(g_vibOn.load() ? kStreamMs : 10);                 // idle most of the time: nothing to send while the road is off
        const auto now = Clock::now();
        const float dt = clampf(std::chrono::duration<float>(now - last).count(), 0.0005f, 0.03f);
        last = now;
        IDirectInputEffect* fx = g_rbrFx.load();
        if (!g_vibOn.load() || !fx || !Ready()) {
            g_vibNow = 0; lowCut.Reset();
            if (lastSent != 0 && fx && Ready() && Send(fx, g_rbrBase.load())) g_vibSends++;   // take the vibration off RBR's force again
            lastSent = 0;
            continue;
        }
        float v = 0.0f;
        for (int b = 0; b < 3; ++b) {
            const float f = clampf(g_hz[b].load(), 0.0f, 200.0f), m = (float)g_mag[b].load();
            ph[b] = std::fmod(ph[b] + 6.2831853f * f * dt, 6.2831853f);
            v += m * std::sin(ph[b]);
        }
        // If RBR stops updating its force (menu, stage end), fade the vibration out before we stop sending, so the effect is never left
        // sitting at an arbitrary point of a vibration cycle.
        const long long age = SteadyMs() - g_rbrFxMs.load();
        if (age > kFreshMs / 2) v *= clampf((float)(kFreshMs - age) / (float)(kFreshMs / 2), 0.0f, 1.0f);
        v = lowCut.Step(v, dt);                                 // no slow push, only vibration
        const int base = g_rbrBase.load();
        const int vib = Road::FitHeadroom(base, (int)std::lround(clampf(v, -10000.0f, 10000.0f)));   // never cut off one side of the vibration
        g_vibNow = vib;
        if (vib == 0 && lastSent == 0) continue;
        if (Send(fx, (int)clampf((float)base + (float)vib, -10000.0f, 10000.0f))) { g_vibSends++; lastSent = vib; }
    }
    g_vibNow = 0;
    if (end) end(1);
}

void Start() { if (g_run.exchange(true)) return; g_thread = std::thread(Loop); }
void Stop()  { g_run = false; if (g_thread.joinable()) g_thread.join(); g_vibOn = false; g_vibNow = 0; }
}  // namespace RoadVib

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

    // Start watching for the game's device object with this instance GUID.
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
    // The game gave up the wheel (menu change, device reset). Our effects went with it, and if we kept holding a
    // reference the game's next exclusive acquire could be blocked, so we ask the worker thread to let go.
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
    // Returns false if another tool hooked on top of ours, in which case we have to stay loaded.
    static bool Unpatch() {
        if (!vtbl) return true;
        const int idx[5] = { VT_ACQUIRE, VT_GETDEVICESTATE, VT_GETDEVICEDATA, VT_POLL, VT_UNACQUIRE };
        void* const mine[5] = { (void*)&HAcquire, (void*)&HGetState, (void*)&HGetData, (void*)&HPoll, (void*)&HUnacquire };
        bool clean = true;
        for (int i = 0; i < 5; ++i) {
            if (vtbl[idx[i]] != mine[i]) { clean = false; continue; }   // someone else hooked after us: leave it alone
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
    // RBR might use the unicode interface, which has its own vtable.
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
    clean = EffectTap::Unpatch() && clean;
    if (!clean) {
        // Someone chained their hook after ours. Unloading now would leave them pointing into freed code, so pin the DLL.
        PinThisDll();
        Logf("Another hook was chained after ours, so the DLL stays pinned in memory");
    }
    m_installed = false;
}

// ------------------------------------------------ telemetry tap on NGP's UDP send
// NGP runs inside the RBR process and sends its telemetry packet from the game loop. Rather than opening a UDP port to
// receive that packet, we watch the send: sendto, send, WSASendTo and WSASend are redirected through small hooks that copy
// any loopback UDP packet of telemetry size into a queue and then let the call carry on untouched. NGP only needs UDP
// telemetry switched on. Where it sends the packet no longer matters.
//
// Each function is hooked one of two ways:
//   hot patch  ws2_32's 32-bit exports normally start with a 2-byte "mov edi, edi" and have 5 spare bytes in front of them.
//              We write a jump into the spare bytes and turn the 2 bytes into a short jump back to it. That catches every
//              caller, however it found the function. It's only used after checking the bytes really look like that.
//   IAT        otherwise every loaded module's import table is searched for the function's address and patched. The
//              search repeats now and then, so a module loaded later is picked up too.
//
// NGP sends one copy of the packet per configured endpoint. We lock on to the first destination we see and ignore the rest.
namespace PacketTap {

constexpr int kMaxPkt = 1024;     // NGP's packet is 664 bytes; anything bigger is not telemetry we know about
constexpr int kRing = 32;

struct Slot { int len; Clock::time_point ts; uint8_t d[kMaxPkt]; };

Slot g_ring[kRing];
std::atomic<uint32_t> g_head{0}, g_tail{0};
std::atomic<bool>     g_spin{false};          // only matters if NGP ever sends from more than one thread
std::atomic<bool>     g_consume{false};       // true while the worker thread wants packets
typedef void (*Sink)(const uint8_t*, int);
std::atomic<Sink>     g_sink{nullptr};        // gets every packet as NGP sends it, whether or not the worker thread is running (the camera module)
std::atomic<int>      g_minLen{664};
std::atomic<unsigned long long> g_destKey{0};
std::atomic<DWORD>    g_destSeen{0};
std::atomic<long long> g_calls{0}, g_taken{0}, g_otherDest{0}, g_dropped{0};
HANDLE g_event = nullptr;                     // set whenever a packet is queued
bool   g_installed = false;
thread_local bool t_busy = false;             // stops a hooked call that ends up in another hooked call from being counted twice

struct Busy {
    bool was;
    Busy() : was(t_busy) { t_busy = true; }
    ~Busy() { t_busy = was; }
};

// ---- queue
void Offer(const uint8_t* data, int len, uint32_t ipNet, uint16_t portNet) {
    g_calls++;
    if ((ipNet & 0xFF) != 127) return;                                  // loopback only (the first octet is the low byte)
    const unsigned long long key = ((unsigned long long)ipNet << 16) | portNet;
    const DWORD nowMs = GetTickCount();
    unsigned long long locked = g_destKey.load();
    if (locked == 0 || (locked != key && nowMs - g_destSeen.load() > 5000)) {   // first destination, or the old one went quiet
        g_destKey.store(key);
        locked = key;
    }
    if (locked != key) { g_otherDest++; return; }
    g_destSeen.store(nowMs);
    if (Sink sink = g_sink.load(std::memory_order_acquire)) sink(data, len);
    if (!g_consume.load()) return;

    while (g_spin.exchange(true, std::memory_order_acquire)) {}
    const uint32_t h = g_head.load(std::memory_order_relaxed), t = g_tail.load(std::memory_order_acquire);
    if (h - t >= (uint32_t)kRing) {
        g_dropped++;
    } else {
        Slot& sl = g_ring[h % kRing];
        sl.len = len;
        sl.ts = Clock::now();
        std::memcpy(sl.d, data, (size_t)len);
        g_head.store(h + 1, std::memory_order_release);
        g_taken++;
    }
    g_spin.store(false, std::memory_order_release);
    if (g_event) SetEvent(g_event);
}

bool Pop(Slot& out) {
    const uint32_t t = g_tail.load(std::memory_order_relaxed);
    if (t == g_head.load(std::memory_order_acquire)) return false;
    out = g_ring[t % kRing];
    g_tail.store(t + 1, std::memory_order_release);
    return true;
}

void SetSink(Sink s) { g_sink.store(s, std::memory_order_release); }
void Activate()   { g_tail.store(g_head.load()); g_consume.store(true); }    // starts with an empty queue
void Deactivate() { g_consume.store(false); }

// ---- the hooks
typedef int (WSAAPI *PFN_sendto)(SOCKET, const char*, int, int, const sockaddr*, int);
typedef int (WSAAPI *PFN_send)(SOCKET, const char*, int, int);
typedef int (WSAAPI *PFN_WSASendTo)(SOCKET, LPWSABUF, DWORD, LPDWORD, DWORD, const sockaddr*, int, LPWSAOVERLAPPED, LPWSAOVERLAPPED_COMPLETION_ROUTINE);
typedef int (WSAAPI *PFN_WSASend)(SOCKET, LPWSABUF, DWORD, LPDWORD, DWORD, LPWSAOVERLAPPED, LPWSAOVERLAPPED_COMPLETION_ROUTINE);

void* r_sendto = nullptr; void* r_send = nullptr; void* r_WSASendTo = nullptr; void* r_WSASend = nullptr;   // the real functions

bool WantLen(int len) { return len >= g_minLen.load(std::memory_order_relaxed) && len <= kMaxPkt; }

// The destination of a connected UDP socket (for send / WSASend).
bool PeerOf(SOCKET s, uint32_t& ip, uint16_t& port) {
    int type = 0, tl = sizeof(type);
    if (getsockopt(s, SOL_SOCKET, SO_TYPE, (char*)&type, &tl) != 0 || type != SOCK_DGRAM) return false;
    sockaddr_in a{}; int al = sizeof(a);
    if (getpeername(s, (sockaddr*)&a, &al) != 0 || a.sin_family != AF_INET) return false;
    ip = a.sin_addr.s_addr; port = a.sin_port;
    return true;
}

// Joins a WSABUF list into one contiguous block (almost always it is a single buffer already).
const uint8_t* Flatten(LPWSABUF b, DWORD cnt, uint8_t* tmp, int& len) {
    len = 0;
    if (!b || cnt == 0) return nullptr;
    if (cnt == 1) { len = (int)b[0].len; return (const uint8_t*)b[0].buf; }
    long long total = 0;
    for (DWORD i = 0; i < cnt; ++i) total += b[i].len;
    if (total > kMaxPkt) { len = (int)total; return nullptr; }
    for (DWORD i = 0; i < cnt; ++i) { std::memcpy(tmp + len, b[i].buf, b[i].len); len += (int)b[i].len; }
    return tmp;
}

int WSAAPI H_sendto(SOCKET s, const char* buf, int len, int flags, const sockaddr* to, int tolen) {
    if (!t_busy) {
        Busy guard;
        if (buf && to && WantLen(len) && tolen >= (int)sizeof(sockaddr_in) && to->sa_family == AF_INET) {
            const sockaddr_in* a = (const sockaddr_in*)to;
            Offer((const uint8_t*)buf, len, a->sin_addr.s_addr, a->sin_port);
        }
        return ((PFN_sendto)r_sendto)(s, buf, len, flags, to, tolen);
    }
    return ((PFN_sendto)r_sendto)(s, buf, len, flags, to, tolen);
}

int WSAAPI H_send(SOCKET s, const char* buf, int len, int flags) {
    if (!t_busy) {
        Busy guard;
        uint32_t ip; uint16_t port;
        if (buf && WantLen(len) && PeerOf(s, ip, port)) Offer((const uint8_t*)buf, len, ip, port);
        return ((PFN_send)r_send)(s, buf, len, flags);
    }
    return ((PFN_send)r_send)(s, buf, len, flags);
}

int WSAAPI H_WSASendTo(SOCKET s, LPWSABUF bufs, DWORD cnt, LPDWORD sent, DWORD flags, const sockaddr* to, int tolen,
                       LPWSAOVERLAPPED ov, LPWSAOVERLAPPED_COMPLETION_ROUTINE cr) {
    if (!t_busy) {
        Busy guard;
        uint8_t tmp[kMaxPkt]; int len;
        const uint8_t* p = Flatten(bufs, cnt, tmp, len);
        if (p && WantLen(len) && to && tolen >= (int)sizeof(sockaddr_in) && to->sa_family == AF_INET) {
            const sockaddr_in* a = (const sockaddr_in*)to;
            Offer(p, len, a->sin_addr.s_addr, a->sin_port);
        }
        return ((PFN_WSASendTo)r_WSASendTo)(s, bufs, cnt, sent, flags, to, tolen, ov, cr);
    }
    return ((PFN_WSASendTo)r_WSASendTo)(s, bufs, cnt, sent, flags, to, tolen, ov, cr);
}

int WSAAPI H_WSASend(SOCKET s, LPWSABUF bufs, DWORD cnt, LPDWORD sent, DWORD flags, LPWSAOVERLAPPED ov,
                     LPWSAOVERLAPPED_COMPLETION_ROUTINE cr) {
    if (!t_busy) {
        Busy guard;
        uint8_t tmp[kMaxPkt]; int len; uint32_t ip; uint16_t port;
        const uint8_t* p = Flatten(bufs, cnt, tmp, len);
        if (p && WantLen(len) && PeerOf(s, ip, port)) Offer(p, len, ip, port);
        return ((PFN_WSASend)r_WSASend)(s, bufs, cnt, sent, flags, ov, cr);
    }
    return ((PFN_WSASend)r_WSASend)(s, bufs, cnt, sent, flags, ov, cr);
}

// ---- installing the hooks
struct Target {
    const char* name;
    void*  hook;
    void** real;
    void*  addr;          // the function in ws2_32 (null if the export wasn't found)
    int    mode;          // 0 = not hooked, 1 = hot patch, 2 = import tables
    BYTE   saved[7];      // the 7 bytes a hot patch replaced
};
Target g_t[4] = {
    {"sendto",    (void*)&H_sendto,    &r_sendto,    nullptr, 0, {0}},
    {"send",      (void*)&H_send,      &r_send,      nullptr, 0, {0}},
    {"WSASendTo", (void*)&H_WSASendTo, &r_WSASendTo, nullptr, 0, {0}},
    {"WSASend",   (void*)&H_WSASend,   &r_WSASend,   nullptr, 0, {0}},
};

struct IatRecord { void** slot; void* orig; void* hook; };
std::vector<IatRecord> g_iat;
std::mutex g_mtx;
HMODULE g_self = nullptr;

bool TryHotPatch(Target& t) {
    BYTE* f = (BYTE*)t.addr;
    if (((uintptr_t)f & 1) != 0 || f[0] != 0x8B || f[1] != 0xFF) return false;              // not "mov edi, edi"
    for (int i = 1; i <= 5; ++i) if (f[-i] != 0xCC && f[-i] != 0x90) return false;          // no spare bytes in front
    DWORD old;
    if (!VirtualProtect(f - 5, 7, PAGE_EXECUTE_READWRITE, &old)) return false;
    std::memcpy(t.saved, f - 5, 7);
    *t.real = f + 2;                                           // the real function continues after the 2-byte nop
    BYTE jmp[5]; jmp[0] = 0xE9;
    const int32_t rel = (int32_t)((BYTE*)t.hook - f);          // from the end of the jump (f - 5 + 5) to the hook
    std::memcpy(jmp + 1, &rel, 4);
    std::memcpy(f - 5, jmp, 5);
    InterlockedExchange16((SHORT volatile*)f, (SHORT)0xF9EB);  // bytes EB F9: jmp short back 7, onto the jump above
    FlushInstructionCache(GetCurrentProcess(), f - 5, 7);
    VirtualProtect(f - 5, 7, old, &old);
    t.mode = 1;
    return true;
}

bool UnpatchHot(Target& t) {
    BYTE* f = (BYTE*)t.addr;
    BYTE jmp[5]; jmp[0] = 0xE9;
    const int32_t rel = (int32_t)((BYTE*)t.hook - f);
    std::memcpy(jmp + 1, &rel, 4);
    if (f[0] != 0xEB || f[1] != 0xF9 || std::memcmp(f - 5, jmp, 5) != 0) return false;   // someone changed it after us
    DWORD old;
    if (!VirtualProtect(f - 5, 7, PAGE_EXECUTE_READWRITE, &old)) return false;
    InterlockedExchange16((SHORT volatile*)f, (SHORT)(t.saved[5] | (t.saved[6] << 8)));       // the original "mov edi, edi"
    std::memcpy(f - 5, t.saved, 5);
    FlushInstructionCache(GetCurrentProcess(), f - 5, 7);
    VirtualProtect(f - 5, 7, old, &old);
    t.mode = 0;
    return true;
}

void RecordIat(void** slot, void* orig, void* hook) { g_iat.push_back(IatRecord{slot, orig, hook}); }

// Looks through one module's import table for the addresses of the functions we hook (by value, so it doesn't matter
// whether the module imported them by name, by ordinal, or through wsock32's forwarders). Plain C only: it runs under SEH.
void ScanModuleRaw(HMODULE m) {
    BYTE* base = (BYTE*)m;
    const IMAGE_DOS_HEADER* dos = (const IMAGE_DOS_HEADER*)base;
    if (dos->e_magic != IMAGE_DOS_SIGNATURE) return;
    const IMAGE_NT_HEADERS* nt = (const IMAGE_NT_HEADERS*)(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) return;
    const IMAGE_DATA_DIRECTORY& dd = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (!dd.VirtualAddress) return;
    const IMAGE_IMPORT_DESCRIPTOR* d = (const IMAGE_IMPORT_DESCRIPTOR*)(base + dd.VirtualAddress);
    for (; d->Name; ++d) {
        void** slot = (void**)(base + d->FirstThunk);
        for (; *slot; ++slot) {
            for (int i = 0; i < 4; ++i) {
                if (g_t[i].mode == 1 || !g_t[i].addr || *slot != g_t[i].addr) continue;
                DWORD old;
                if (!VirtualProtect(slot, sizeof(void*), PAGE_READWRITE, &old)) continue;
                *slot = g_t[i].hook;
                VirtualProtect(slot, sizeof(void*), old, &old);
                RecordIat(slot, g_t[i].addr, g_t[i].hook);
                g_t[i].mode = 2;
            }
        }
    }
}

void ScanModule(HMODULE m) {
    if (m == g_self) return;
    __try { ScanModuleRaw(m); } __except (EXCEPTION_EXECUTE_HANDLER) {}
}

void ScanAllModules() {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, GetCurrentProcessId());
    if (snap == INVALID_HANDLE_VALUE) return;
    MODULEENTRY32 me{}; me.dwSize = sizeof(me);
    if (Module32First(snap, &me)) do { ScanModule(me.hModule); } while (Module32Next(snap, &me));
    CloseHandle(snap);
}

bool Install(int minLen) {
    std::lock_guard<std::mutex> lk(g_mtx);
    if (g_installed) return true;
    g_minLen.store(minLen);
    if (!g_event) g_event = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)&pluginDirectory, &g_self);
    HMODULE ws = GetModuleHandleA("ws2_32.dll");
    if (!ws) ws = LoadLibraryA("ws2_32.dll");
    if (!ws || !g_event) return false;
    for (Target& t : g_t) {
        t.addr = (void*)GetProcAddress(ws, t.name);
        if (!t.addr) continue;
        *t.real = t.addr;
        TryHotPatch(t);
    }
    bool anyIat = false;
    for (const Target& t : g_t) if (t.addr && t.mode == 0) anyIat = true;
    if (anyIat) ScanAllModules();
    int n = 0;
    for (const Target& t : g_t) if (t.mode) ++n;
    g_installed = n > 0;
    return g_installed;
}

// Called now and then by the worker thread: picks up modules that were loaded after the first search.
void Rescan() {
    std::lock_guard<std::mutex> lk(g_mtx);
    if (!g_installed) return;
    bool anyIat = false;
    for (const Target& t : g_t) if (t.addr && t.mode != 1) anyIat = true;
    if (anyIat) ScanAllModules();
}

bool Installed() { return g_installed; }

std::string Describe() {
    std::string out;
    for (const Target& t : g_t) {
        if (!out.empty()) out += "  ";
        out += t.name;
        out += t.mode == 1 ? ":hot-patch" : t.mode == 2 ? ":import" : ":-";
    }
    return out;
}

bool RestoreSlot(const IatRecord& r) {      // plain C only: it runs under SEH
    __try {
        if (*r.slot != r.hook) return false;                    // someone patched it again after us
        DWORD old;
        if (!VirtualProtect(r.slot, sizeof(void*), PAGE_READWRITE, &old)) return false;
        *r.slot = r.orig;
        VirtualProtect(r.slot, sizeof(void*), old, &old);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

// Puts everything back. Returns false if something else changed our patches in the meantime.
bool Remove() {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_consume.store(false);
    if (!g_installed) return true;
    bool clean = true;
    for (Target& t : g_t) if (t.mode == 1 && !UnpatchHot(t)) clean = false;
    for (const IatRecord& r : g_iat) if (!RestoreSlot(r)) clean = false;
    g_iat.clear();
    g_installed = false;
    return clean;
}

}  // namespace PacketTap

// ----------------------------------------------------------- effect layer
BOOL CALLBACK FindFFAxisCb(const DIDEVICEOBJECTINSTANCEA* o, VOID* ref) {
    // Only reads fields that come before the name strings, so it works for both the ANSI and unicode structs.
    if (o->dwFlags & DIDOI_FFACTUATOR) {
        *static_cast<DWORD*>(ref) = o->dwType;
        return DIENUM_STOP;
    }
    return DIENUM_CONTINUE;
}

// The road is computed as three bands, low to high (road_bands.hpp), so the synth keeps three road slots. Only S_ROAD2 is ever a wheel
// effect: when the road is played as an effect of its own, the three bands are merged into that one sine (Road::Consolidate). Normally
// the bands are added to RBR's own force instead and the wheel gets no road effect at all.
enum Slot { S_ENG1 = 0, S_ENG2, S_ROAD1, S_ROAD2, S_ROAD3, S_BRAKE, S_COUNT };
const bool kSlotIsEffect[S_COUNT] = {true, true, false, true, false, true};
const char* const kSlotName[S_COUNT] = {"engine", "lump", "road-low", "road", "road-high", "brake"};

class FFLayer {
public:
    ~FFLayer() { Destroy(); }      // also runs if the worker thread unwinds because of an exception
    bool Ready() const {
        for (int i = 0; i < S_COUNT; ++i) if (m_fx[i]) return true;
        return false;
    }

    HRESULT Create(IDirectInputDevice8A* dev) {
        Destroy();
        m_dev = dev;

        DWORD axisId = 0xFFFFFFFF;
        dev->EnumObjects(FindFFAxisCb, &axisId, DIDFT_AXIS);
        if (axisId == 0xFFFFFFFF) return E_NOTIMPL;      // this device has no force feedback motor
        m_axisId = axisId;

        const GUID* guids[S_COUNT] = { &GUID_Sine, &GUID_Sine, &GUID_Sine, &GUID_Sine, &GUID_Sine, &GUID_Sine };
        HRESULT firstErr = DI_OK;
        int ok = 0;
        int wanted = 0;
        for (int i = 0; i < S_COUNT; ++i) {
            if (!kSlotIsEffect[i]) continue;                  // the road bands that only exist inside the synth
            ++wanted;
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
        for (int i = 0; i < S_COUNT; ++i) if (m_fx[i]) { EffectTap::Patch(*reinterpret_cast<void***>(m_fx[i])); break; }   // RBR's effects share this class
        Logf("Effects created: %d of %d", ok, wanted);
        return DI_OK;
    }

    void Destroy() {
        for (int i = 0; i < S_COUNT; ++i)
            if (m_fx[i]) { m_fx[i]->Stop(); m_fx[i]->Release(); m_fx[i] = nullptr; }
        m_dev = nullptr;
    }

    // RBR re-acquired the device, so make sure every effect is playing again.
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
    bool Has(int slot) const { return slot >= 0 && slot < S_COUNT && m_fx[slot] != nullptr; }
private:
    IDirectInputDevice8A* m_dev = nullptr;
    IDirectInputEffect* m_fx[S_COUNT] = {};
    int m_lastMag[S_COUNT];
    DWORD m_lastPer[S_COUNT];
    Clock::time_point m_lastT[S_COUNT];
    DWORD m_axisId = 0;
    int m_fail = 0;
    HRESULT m_lastHr = DI_OK;

    HRESULT Make(int slot, const GUID& g) {
        DIPERIODIC per{};
        per.dwPeriod = 20000;
        LONG dir = 0;
        DIEFFECT e{};
        e.dwSize = sizeof(DIEFFECT);
        e.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTIDS;   // address the axis by object id, so RBR's data format doesn't matter
        e.dwDuration = INFINITE;
        e.dwGain = DI_FFNOMINALMAX;
        e.dwTriggerButton = DIEB_NOTRIGGER;
        e.cAxes = 1;
        e.rgdwAxes = &m_axisId;
        e.rglDirection = &dir;
        e.cbTypeSpecificParams = sizeof(per); e.lpvTypeSpecificParams = &per;
        return m_dev->CreateEffect(g, &e, &m_fx[slot], NULL);
    }

    HRESULT Update(int slot, int mag, DWORD period) {
        DIEFFECT e{};
        e.dwSize = sizeof(DIEFFECT);
        mag = std::max(0, std::min(mag, 10000));
        const bool magChanged = std::abs(mag - m_lastMag[slot]) >= kMagDeadband || (mag == 0 && m_lastMag[slot] != 0);
        const bool perChanged = m_lastPer[slot] == 0 ||
            (period > m_lastPer[slot] ? period - m_lastPer[slot] : m_lastPer[slot] - period) * 100 > m_lastPer[slot] * 2;
        if (!magChanged && !perChanged) return DI_OK;
        if (mag == 0 && m_lastMag[slot] == 0) return DI_OK;                         // silent and staying silent: a pitch change is nothing to send
        const auto now = Clock::now();
        if (mag != 0 && now - m_lastT[slot] < std::chrono::milliseconds(kMinUpdateGapMs)) return DI_OK;   // silencing an effect is never held back
        DIPERIODIC p{};
        p.dwMagnitude = (DWORD)mag; p.lOffset = 0; p.dwPhase = 0; p.dwPeriod = period;
        e.cbTypeSpecificParams = sizeof(p); e.lpvTypeSpecificParams = &p;
        HRESULT hr = m_fx[slot]->SetParameters(&e, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);
        if (SUCCEEDED(hr)) { m_lastMag[slot] = mag; m_lastPer[slot] = period; m_lastT[slot] = now; g_fxUpdates++; }
        return hr;
    }
};

// ------------------------------------------------------------------ telemetry
// Car.Index is a uint32 in some NGP builds and an integral float in others, so accept either.
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

// The share of each side's ground and texture that comes from its front tyre (the rear tyre gets the rest). The front tyre is what the
// steering feels, so it dominates: 90:10. Overridden by FrontWeight in [Surface].
namespace Surf { float g_front = 0.90f; }

struct Signals {
    float rpm = 0, speed = 0 /* m/s */, throttle = 0, brake = 0;
    float rpmMaxTelem = 0;   // rev limit straight from the telemetry (0 = not available)
    float rough = 0;      // raw front-weighted tyre roughness
    float piston = 0;     // damper piston velocity level
    float texture = 0;    // combined road texture 0..1
    float texL = 0, texR = 0;   // the same, per side (left / right tyres)
    float bump = 0;       // heave acceleration spike envelope
    float dive = 0;       // front load transfer under braking 0..1
    float lock = 0;       // lock-up 0..1 (the synth works it out per tyre; only the test mode sets it here)
    float frontRev = 0;   // front wheel revolutions per second
    int   carIndex = -1;  // NGP Car.Index (-1 = unknown)
    // weight / suspension load, relative to this car's own static load (1 = at rest)
    float loadL = 1, loadR = 1, loadF = 1;
    float airborne = 0;   // 0 = wheels loaded, 1 = off the ground
    int   surf[4] = {0, 0, 0, 0};   // ground class under each tyre, in the order FL, FR, RL, RR (Surf::Cls); 0 = unknown
    int   mat[4] = {-1, -1, -1, -1};   // game material id under each tyre, same order (-1 = unknown); the ground's properties come from its name
    // Per tyre, same order (FL, FR, RL, RR): the raw material for wheelspin and slides. The synth judges them against the grip of the ground under each tyre.
    float ratio[4] = {0, 0, 0, 0};  // slip ratio past the uncertainty of the count: + the tyre turns faster than the ground along its own direction, - slower
    bool  ratioOk[4] = {false, false, false, false};
    float angle[4] = {0, 0, 0, 0};  // slip angle, rad (from body motion and steering)
    bool  angleOk = false;
    float handbrake = 0;
    float wload[4] = {0, 0, 0, 0};  // share of the car's weight on each tyre (sums to 1)
    float accMag = 0; bool accOk = false;   // the car's acceleration over the ground in m/s^2, if the units are known
    float ground = 0;               // the car's speed over the ground, m/s (0 = not known)
    float thumpL = 0, thumpR = 0;   // compression / landing envelope per side
    unsigned evtSeq = 0;  // goes up by one for every landing or hard hit; the synth starts one boost per event
    float evtSev = 0;     // severity of the current event 0..1 (refined upward while a landing is still compressing)
    float staticLoad = 0; // learned static strut load, all four wheels (units of the telemetry)
};

struct Hp  { float y = 0, x = 0; bool init = false;
             float step(float in, float a) { if (!init) { x = in; init = true; y = 0; return 0; } y = a * (y + in - x); x = in; return y; } };
struct Ema { float v = 0; bool init = false;
             float step(float in, float a) { if (!init) { v = in; init = true; } else v += (in - v) * a; return v; } };


// Analysis state that carries over from packet to packet. NGP wheel order: LF, RF, LB, RB.
struct Dsp {
    Hp  sfHp[4], pistonHp[4], accHp;
    Ema sfVar[4], pistonVar[4], loadShort[4];
    Ema loadRef, baseFront;
    // Wheel turns, counted over a window with enough steps, against the ground speed (wheel_slip.hpp)
    Slip::Timebase tbase; Slip::RevCounter revc[4]; Slip::WheelSlip wslip[4]; Slip::GroundSpeed gspeed;
    unsigned timeGen = ~0u; double tArr = 0; bool slipInit = false;
    Tyre::Kinematics kin;   // each tyre's own slip angle and ground speed
    float bumpEnv = 0, diveSm = 0;
    Ema statRef[4], rLsm, rRsm, rFsm;
    float thumpL = 0, thumpR = 0;
    // landing / impact detector (works on raw, unsmoothed load so the spike doesn't get averaged away)
    float peakRw = 0, totRatio = 1;
    float airT = 0, landWin = 0, landAir = 0, landPeak = 0, landBump = 0, landJump = 0, imRefr = 0;
    unsigned evtSeq = 0; float evtSev = 0;
    float totLow = 1.0f;   // running minimum of the total strut load (relative): a sudden rise above it means something hit the car
};

void ParsePacket(const Config& c, const uint8_t* b, int n, Signals& s, Dsp& d, float dt) {
    d.tArr += std::max(0.0f, dt);                    // arrival time of this packet, before the step is limited below
    dt = clampf(dt, 0.002f, 0.05f);
    float v;
    { int ci; if (readIndex(b, n, c.offCarIndex, ci)) s.carIndex = ci; }
    if (readF(b, n, c.offRpm, v))   s.rpm = std::max(0.0f, v * c.rpmScale);
    if (readF(b, n, c.offSpeed, v)) s.speed = std::fabs(v) * (c.speedIsKph ? (1.0f / 3.6f) : 0.44704f);
    if (readF(b, n, c.offRpmMax, v) && v * c.rpmScale >= 1500.0f && v * c.rpmScale <= 20000.0f) s.rpmMaxTelem = v * c.rpmScale;
    s.throttle = readF(b, n, c.offThrottle, v) ? clamp01(v) : 0.6f;
    s.brake    = readF(b, n, c.offBrake, v)    ? clamp01(v) : 0.0f;

    const float aHp = hpCoef(dt, 0.032f);          // ~5 Hz: removes weight transfer, keeps texture and hits
    const float aVar = alphaFor(dt, 0.10f);        // RMS window
    const float aLoad = alphaFor(dt, 0.3f);

    float rough[4] = {0, 0, 0, 0}, absSf[4] = {0, 0, 0, 0}, rawSf[4] = {0, 0, 0, 0}, pRms[4] = {0, 0, 0, 0};
    uint32_t segRaw[4] = {0, 0, 0, 0}; bool haveSeg[4] = {false, false, false, false};
    for (int i = 0; i < 4 && c.offSuspBase >= 0; ++i) {
        const int base = c.offSuspBase + i * c.suspStride;
        float sf = 0, pv = 0;
        if (readF(b, n, base + c.offStrut, sf)) {
            rawSf[i] = sf;
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
        if (so >= 0 && so + 4 <= n) { std::memcpy(&segRaw[i], b + so, 4); haveSeg[i] = true; }
    }
    const float meanLoad = 0.25f * (absSf[0] + absSf[1] + absSf[2] + absSf[3]);
    {   const float totAbs = absSf[0] + absSf[1] + absSf[2] + absSf[3];
        for (int i = 0; i < 4; ++i) s.wload[i] = totAbs > 1e-6f ? absSf[i] / totAbs : 0.25f; }
    if (s.speed > 3.0f && meanLoad > 0) d.loadRef.step(meanLoad, alphaFor(dt, 8.0f));

    // ---- weight / suspension load model. Everything is relative to this car's own static load (learned slowly and
    //      reset at the start of a stage while the car is standing still), so telemetry units and car mass don't matter.
    {
        const float tot = absSf[0] + absSf[1] + absSf[2] + absSf[3];
        float statTot = 0; bool seeded = true;
        for (int i = 0; i < 4; ++i) { statTot += d.statRef[i].v; seeded = seeded && d.statRef[i].init; }
        if (!seeded || tot > 0.5f * statTot)                       // don't learn the static load while airborne
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
            d.totRatio = tot / statTot;
            d.peakRw = std::max(std::max(rw[0], rw[1]), std::max(rw[2], rw[3]));
            const float eDecay = std::exp(-dt / 0.14f);               // compression / landing thump per side
            d.thumpL = std::max(clamp01((rl  - 1.7f) / 1.2f), d.thumpL * eDecay);
            d.thumpR = std::max(clamp01((rr2 - 1.7f) / 1.2f), d.thumpR * eDecay);
            s.thumpL = d.thumpL; s.thumpR = d.thumpR;
        }
    }

    // ---- road texture: the front tyres dominate, because they are what you feel through the steering rack
    const float rF = std::max(rough[0], rough[1]), rR = std::max(rough[2], rough[3]);
    s.rough = Surf::g_front * rF + (1.0f - Surf::g_front) * rR;
    const float roughLevel = std::pow(clamp01((s.rough - c.roughLow) / (c.roughHigh - c.roughLow)), c.roughGamma);
    s.piston = std::pow(clamp01(std::max(pRms[0], pRms[1]) * c.pistonScale), 1.2f);
    s.texture = clamp01(std::max(roughLevel, 0.8f * s.piston));

    // ---- the same thing per side (LF/LB on the left, RF/RB on the right); the fronts count most
    auto side = [&](int front, int back) {
        const float r = Surf::g_front * rough[front] + (1.0f - Surf::g_front) * rough[back];
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

    // ---- landings and hard hits. Three cues are combined with max(), so any one strong cue gives a full-strength hit:
    //      peak   the highest strut load on any wheel (relative to its static load)
    //      jump   how fast the TOTAL strut load rose from its recent minimum (works with no airtime at all: drops,
    //             crests, bottoming out)
    //      air    how long the wheels were unloaded before the hit, plus the heave acceleration spike
    //      Severity is continuous (no on/off threshold), so a small hop is a small tap and a big jump is violent.
    {
        d.imRefr = std::max(0.0f, d.imRefr - dt);
        const float rawTot = d.totRatio;
        d.totLow = std::min(rawTot, d.totLow + (rawTot - d.totLow) * alphaFor(dt, 0.12f));
        const float jump = std::max(0.0f, rawTot - d.totLow);
        const bool off = s.speed > 3.0f && rawTot < c.airLevel;

        // Two tiers: a landing after real airtime (air > 0) is scored sensitively so every jump is felt. The no-airtime
        // "hit" path (air == 0) is scored strictly, because weight transfer from braking while cornering also moves the
        // strut loads and must not be mistaken for a landing.
        auto severity = [&](float peak, float air, float bump, float jmp) {
            const bool sens = air > 0.0f;
            const float peakT = clamp01((peak - (sens ? 1.15f : 1.3f)) / (sens ? 1.3f : 1.7f));
            const float jumpT = clamp01((jmp - (sens ? 0.25f : 0.35f)) / (sens ? 1.0f : 1.4f));
            const float airT2 = clamp01(air / 0.5f);
            const float bumpT = clamp01(bump);
            const float top = std::max(std::max(peakT, jumpT), std::max(0.85f * airT2, 0.7f * bumpT));
            const float sum = peakT + jumpT + airT2 + bumpT;
            float sev = clamp01(top + 0.15f * (sum - top));
            if (air > 0.04f) sev = std::max(sev, 0.55f + 0.45f * airT2);    // anything that left the ground is always felt
            return sev;
        };

        // Every landing and hard hit fires ONE event with a severity. The synth turns it into a boost of RBR's own force.
        auto fire = [&](float sev) { ++d.evtSeq; d.evtSev = sev; };

        if (off) {
            d.airT += dt;
        } else {
            if (Road::LandingBoostWorthy(d.airT) && d.landWin <= 0.0f) {   // only a real jump boosts the wheel, a bump never does
                d.landWin = 0.10f; d.landAir = d.airT; d.landPeak = 0; d.landBump = 0; d.landJump = 0;
                fire(severity(0.0f, d.landAir, 0.0f, 0.0f));
            }
            d.airT = 0;
        }
        if (d.landWin > 0.0f) {                                   // refine the estimate while the suspension compresses
            d.landPeak = std::max(d.landPeak, d.peakRw);
            d.landBump = std::max(d.landBump, clamp01(d.bumpEnv));
            d.landJump = std::max(d.landJump, jump);
            const float sev = severity(d.landPeak, d.landAir, d.landBump, d.landJump);
            d.evtSev = std::max(d.evtSev, sev);
            d.landWin -= dt;
            if (d.landWin <= 0.0f) {
                d.landWin = 0.0f; d.imRefr = 0.15f;
                Logf("landing: severity %.2f  (air %.2fs, peak load %.2f, load jump %.2f, heave %.2f)", d.evtSev, d.landAir, d.landPeak, d.landJump, d.landBump);
            }
        } else if (!off && d.imRefr <= 0.0f && jump > 0.35f * c.landingThresh && d.peakRw > 1.0f + 0.5f * c.landingThresh) {   // hard hit / bottoming out with no airtime
            // Logged only. A hit without airtime is a bump or weight transfer; boosting RBR's force for it (and for the stream of
            // bumps on rough ground) made the wheel heavy to turn, so it no longer fires an event.
            const float sev = severity(d.peakRw, 0.0f, clamp01(d.bumpEnv), jump);
            if (sev > 0.12f) {
                d.imRefr = 0.15f;
                Logf("hit (no boost): severity %.2f  (peak load %.2f, load jump %.2f, heave %.2f)", sev, d.peakRw, jump, clamp01(d.bumpEnv));
            }
        }
        s.evtSeq = d.evtSeq;
        s.evtSev = clamp01(d.evtSev * c.impactGain);      // an impactGain above 1 lets mid-size hits reach full strength
    }

    // ---- braking: weight moving onto the front tyres ("dive") and front wheel lock-up
    const float tot = absSf[0] + absSf[1] + absSf[2] + absSf[3];
    if (tot > 1e-3f) {
        float share = (absSf[0] + absSf[1]) / tot;
        if (s.brake < 0.05f && s.speed > 3.0f) d.baseFront.step(share, alphaFor(dt, 2.0f));
        float target = (s.brake > 0.1f && d.baseFront.init) ? clamp01((share - d.baseFront.v) / 0.10f) : 0.0f;
        d.diveSm += (target - d.diveSm) * alphaFor(dt, 0.1f);
    }
    s.dive = d.diveSm;

    // ---- wheel spin and slip. NGP has no slip value, only a segment counter per wheel, so the spin is counted over a window with
    //      enough steps and compared with the car's real speed over the ground (wheel_slip.hpp). Slip only counts past the
    //      uncertainty of that count, so a wheel that is just rolling never reads as slipping.
    {
        if (!d.slipInit) { for (Slip::WheelSlip& w : d.wslip) w.reset(1.0 / c.tyreCircumference); d.slipInit = true; }
        uint32_t steps = 0; bool haveSteps = false;
        if (c.offSteps >= 0 && c.offSteps + 4 <= n) { std::memcpy(&steps, b + c.offSteps, 4); haveSteps = true; }
        float vs = 0, vw = 0;
        const bool haveVel = readF(b, n, c.offVelSurge, vs) && readF(b, n, c.offVelSway, vw);
        const double groundMs = d.gspeed.update(s.speed, haveVel ? vs : 0.0, haveVel ? vw : 0.0, s.throttle, s.brake, dt);
        s.ground = (float)groundMs;
        const Slip::Timebase::Stamp st = d.tbase.stamp(steps, haveSteps, d.tArr);
        if (d.tbase.generation() != d.timeGen) { for (Slip::RevCounter& r : d.revc) r.reset(); d.timeGen = d.tbase.generation(); }
        float steer = 0, yaw = 0, aSway = 0, aSurge = 0, hb = 0;
        readF(b, n, c.offSteering, steer); readF(b, n, c.offYawRate, yaw); readF(b, n, c.offAccSway, aSway);
        readF(b, n, c.offAccSurge, aSurge);
        s.handbrake = readF(b, n, c.offHandbrake, hb) ? clamp01(hb) : 0.0f;

        // Each tyre's own motion over the ground: its slip angle, and the ground speed along the direction it points. A tyre
        // sliding sideways only rolls at that speed, so its spin must be compared with it, not with the car's full speed.
        Tyre::Params kp;
        kp.wheelbase = c.wheelbase; kp.cgToFront = c.cgToFront; kp.trackWidth = c.trackWidth;
        Tyre::Input ki;
        ki.velSurge = haveVel ? vs : 0.0; ki.velSway = haveVel ? vw : 0.0; ki.yawRate = yaw; ki.accSway = aSway; ki.steering = steer;
        for (int i = 0; i < 4; ++i) ki.strut[i] = rawSf[i];
        d.kin.update(ki, dt, groundMs, kp, d.gspeed.fromVelocity());
        const double accScale = d.kin.accelScale(kp);
        s.accMag = (float)(std::sqrt((double)aSurge * aSurge + (double)aSway * aSway) * accScale);
        s.accOk = d.kin.unitsConfirmed();
        s.angleOk = d.kin.valid;

        const bool coasting = s.throttle < 0.12f && s.brake < 0.03f && s.handbrake < 0.03f &&
                              std::fabs(aSway * accScale) < kp.coastAccelLimit && std::fabs(aSurge * accScale) < kp.coastAccelLimit;
        float frontRev = 0; int frontN = 0;
        for (int i = 0; i < 4; ++i) {
            s.ratio[i] = 0.0f; s.ratioOk[i] = false; s.angle[i] = (float)d.kin.slipAngle[i];
            if (!haveSeg[i]) continue;
            d.revc[i].push(segRaw[i], st.tick, st.secPerTick);
            d.wslip[i].update(d.revc[i], st.secPerTick, st.timeErr, d.kin.vLong[i], coasting, dt);
            if (d.wslip[i].valid) { s.ratio[i] = (float)d.wslip[i].ratio; s.ratioOk[i] = true; }
            if (i < 2) { const Slip::RevCounter::Est e = d.revc[i].estimate(32.0, 0.5, st.secPerTick); if (e.valid) { frontRev += (float)e.revPerSec; ++frontN; } }
        }
        s.frontRev = frontN ? frontRev / frontN : 0.0f;
    }
}

// -------------------------------------------------------------- synthesis
// Turns the analysed signals into effect strengths and frequencies. Runs on the worker thread, about 100 times a second.
// ====================================================================================================================
// ROAD FEEL: how the road vibration is shaped by the ground and by what the car is doing.
//
// There is no list of "gravel feels like this, tarmac like that". The game's contact data names the material under each tyre
// (for example Gravel_Coarse_Deep_Wet), and road_feel.hpp turns that name into a few physical properties: how hard the ground
// is, how much loose material lies on it, how coarse it is and how slippery. The road effect then follows from those together
// with the telemetry: the measured roughness of the tyres sets how strong it is, speed and wheel slip decide how hard loose
// ground is worked (stones flung, grain), and tyres that shake in a regular pattern on hard ground are cobbles. Everything
// reshapes the existing road effect, so no new effects are added and the force stays zero-mean. Unknown ground is plain hard
// ground. The one setting for all of it is the road gain; the switch is in the menu (Plugins > ForceFX).
// ====================================================================================================================
// The surface classes and the id-to-class table live in surface_feel.hpp, the ground model in road_feel.hpp (both testable on
// their own). surface_feel.hpp is included here, inside the anonymous namespace, because the surface reader below extends the
// same Surf namespace.
#include "surface_feel.hpp"
#include "road_feel.hpp"   // how the ground and what the car is doing shape the road vibration
#include "road_bands.hpp"  // the road vibration as three sine bands: pitch from speed, strength spread by the ground
namespace Surf {
float g_fade = 0.25f;                // seconds to cross-fade between two surfaces
int   g_cobbleDetect = 1;            // 1 = hard ground that shakes in a regular pattern gets the cobble feel (detected from the telemetry)
float g_cobbleLo = 0.5f;             // rough-per-speed level (x100) where that blend starts ...
float g_cobbleHi = 1.0f;             // ... and where it is complete
}  // namespace Surf

// How far each side's road strength follows its own tyres (0 = both sides identical, 1 = left and right fully separate).
constexpr float kRoadDirectional = 0.7f;

struct Synth {
    float sm[S_COUNT] = {};
    Surf::CobbleDetector cob;                              // telemetry check for hard ground made of regular ridges
    Road::Ground gnd[4];                                   // the ground under each tyre (FL, FR, RL, RR), moved gradually towards what was read
    Tyre::GripLearner grip;                                // how hard the car can accelerate on each kind of ground (reset with the car)
    float spinSm[4] = {}, slideSm[4] = {}, lockSm[4] = {};   // wheelspin, sliding and lock-up of each tyre (FL, FR, RL, RR), 0..1, smoothed
    float lockAmt = 0;                                     // lock-up for the brake effect, 0..1
    float thrSm = 0, lope = 0, rpmPeak = 0, heat = 0;
    float jAge = 5.0f, evAmp = 0.0f;                       // age and severity of the current landing event
    unsigned seenEvt = 0;
    float evBusy = 0.0f, evGov = 1.0f;                     // how many landings and hits came lately, and how much the current one is held back because of them
    float boostMul = 1.0f, boostAdd = 0.0f;                // landing boost for RBR's own force (the worker thread hands it to the hook)
    float bandHzJit[Road::kBands] = {1.0f, 1.0f, 1.0f}, bandAmpJit[2][Road::kBands] = {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};   // random pitch per band, random strength per side and band
    float engJit = 1.0f, engHzJit = 1.0f, unevPhase = 0;
    double nextJit = 0;

    void Run(const Config& c, const Signals& s, double t, float dt, float userGain, int mag[S_COUNT], float hz[S_COUNT]) {
        thrSm += (s.throttle - thrSm) * alphaFor(dt, 0.12f);

        // ---------------- ENGINE: both strength and pitch follow the engine
        rpmPeak = std::max(rpmPeak, s.rpm);
        // rev limit: telemetry first, then the car profile or ini, then the highest rpm seen so far
        const float rmax = s.rpmMaxTelem > 0 ? s.rpmMaxTelem : (c.rpmMax > 0 ? c.rpmMax : std::max(rpmPeak, c.rpmMaxInit));
        const float x = clamp01(s.rpm / rmax);
        const float fire = s.rpm / 60.0f * c.cylinders * 0.5f;
        const float fireMax = rmax / 60.0f * c.cylinders * 0.5f;
        const float k = fireMax > c.engineMaxHz ? c.engineMaxHz / fireMax : 1.0f;     // squeeze the whole range linearly so the pitch never jumps
        const float buzzW = smooth01(0.50f, 1.0f, x);                                 // how far into the high-rev range we are
        const float grit = c.engineGrit * buzzW;
        hz[S_ENG1] = clampf(fire * k * (1.0f + 0.04f * grit * engHzJit), c.engineMinHz, c.engineMaxHz);

        const float idle = 1.0f - smooth01(0.08f, 0.45f, x);                          // lumpy idle
        const float res = gauss(x, 0.55f, 0.08f) + 0.8f * gauss(x, 0.80f, 0.07f);    // engine resonances
        const float buzz = smooth01(0.60f, 1.0f, x);                                  // high-rev buzz
        const float A1 = clamp01(0.22f + 0.45f * idle + 0.35f * res + 0.30f * buzz);
        const float loadF = 0.45f + 0.55f * thrSm;                                    // an engine under load shakes more
        const float comp = clampf(1.4f - 0.009f * (hz[S_ENG1] - c.engineMinHz), 0.55f, 1.4f);   // wheels are weaker at high frequencies, so compensate
        const float engAir  = 1.0f - 0.35f * c.airborneFade * s.airborne;                       // an engine revving in the air feels lighter
        const float unevMod = 1.0f - 0.35f * c.unevenness * (0.5f + 0.5f * std::sin(unevPhase)); // uneven firing makes the strength sway
        float t1 = s.rpm > 200 ? c.engineMaxMag * clamp01(A1 * loadF * comp * unevMod * engAir * (1.0f - 0.45f * grit * (0.5f - 0.5f * engJit))) : 0.0f;   // grit makes the strength irregular
        if (s.rpm > 200 && x > 0.985f && thrSm > 0.8f) t1 *= (std::fmod(t, 0.08) < 0.04 ? 1.0f : 0.25f);   // rev limiter

        lope += (rnd() - lope) * 0.05f;                                               // slow random wander for uneven low-rpm firing
        // At low revs this is a half-order lump. At high revs the same layer becomes a detuned second tone (x1.37)
        // whose beating against the main one makes a high-rev engine feel rough instead of like a clean sine.
        const float xover = smooth01(0.45f, 0.75f, x) * c.engineGrit;
        const float hzLump = clampf(fire * 0.5f, 7.0f, 42.0f);
        const float hzBeat = clampf(hz[S_ENG1] * 1.37f, c.engineMinHz, kHardMaxHz);
        hz[S_ENG2] = hzLump + (hzBeat - hzLump) * xover;
        unevPhase += 6.2831853f * hzLump * dt;
        if (unevPhase > 6.2831853f) unevPhase -= 6.2831853f;
        const float A2 = (0.15f + 0.85f * idle) * (0.5f + 0.5f * thrSm) * (1.0f + 0.25f * lope * (0.2f + idle))
                       + 0.55f * grit * (0.6f + 0.4f * thrSm);
        float t2 = s.rpm > 200 ? c.engineMaxMag * 0.8f * clamp01(A2 * c.lumpScale) * engAir : 0.0f;

        // ---------------- ROAD: three sine bands (low, mid, high) of the road's spectrum. The pitch of each band follows the car's
        // speed, the ground under each side decides how the strength is spread over the bands, and the tyre's contact patch takes
        // the high bands away at low speed (road_bands.hpp). The sines are zero-mean, so they never shift RBR's own force, and the
        // two sides add as energy, so they can not cancel each other. How the bands reach the wheel is decided in the worker loop.
        const float spN = clamp01(s.speed / c.maxSpeed);
        const float gSpd = smooth01(1.0f, 10.0f, s.speed);

        // ---------------- TYRE MOTION: wheelspin and slides, each tyre judged against the grip limit of the ground under it.
        //   Wheelspin: a driven tyre turning faster than the ground, throttle open. A rear-wheel-drive car only has it at the rear.
        //   Slide: a tyre moving the wrong way, sideways or locked up under braking or the handbrake. Any tyre can slide.
        // How much of the grip the car is using (learned per ground from how hard it has accelerated on it) confirms a slide.
        {
            double share[Tyre::GripLearner::kClasses] = {0, 0, 0, 0, 0, 0, 0, 0};
            for (int w = 0; w < 4; ++w) { const int cl = s.surf[w]; if (cl > 0 && cl < Tyre::GripLearner::kClasses) share[cl] += s.wload[w]; }
            double sum = 0; for (double v : share) sum += v;
            if (sum > 1e-6) for (double& v : share) v /= sum;
            grip.update(s.accMag, share, s.accOk && s.airborne < 0.5f && s.speed > 5.0f && sum > 1e-6, dt);
            const double gate = Tyre::GripGate(grip.use());
            for (int w = 0; w < 4; ++w) {
                Tyre::TyreIn in;
                in.driven = Tyre::Driven(c.drivetrain, w);
                in.ratioValid = s.ratioOk[w]; in.ratio = s.ratio[w];
                in.angleValid = s.angleOk && s.speed > 1.0f && std::fabs(s.angle[w]) < 0.7f; in.angle = s.angle[w];
                in.throttle = s.throttle; in.brake = s.brake; in.handbrake = s.handbrake; in.gate = gate;
                Tyre::PeaksFromGround(gnd[w].loose, gnd[w].slick, in.peakRatio, in.peakAngleDeg);   // the grip limit of the ground under this tyre
                const Tyre::TyreOut o = Tyre::Classify(in);
                const float aSp = alphaFor(dt, 0.05f);
                const float tSpin = (float)Tyre::SpinAmount(o.spin), tSlide = (float)Tyre::SlideAmount(o.slide), tLock = (float)Tyre::SlideAmount(o.lock, 1.0);
                spinSm[w]  += (tSpin  - spinSm[w])  * aSp;
                slideSm[w] += (tSlide - slideSm[w]) * aSp;
                lockSm[w]  += (tLock  - lockSm[w])  * alphaFor(dt, tLock > lockSm[w] ? 0.05f : 0.15f);
            }
            // what the steering feels: the front tyres first, the rear ones a little less (they are felt through the car)
            lockAmt = clamp01(0.5f * (lockSm[0] + lockSm[1]) + 0.25f * (lockSm[2] + lockSm[3]));
        }
        const auto sideDyn = [](const float* a, int k2) { return std::max(a[k2], 0.6f * a[2 + k2]); };   // front tyre of the side is k2, rear tyre 2 + k2
        const float texMono = clamp01(s.texture + 0.9f * s.bump);
        const float wf = clampf(c.weightFeel, 0.5f, 2.0f);
        const float thump[2] = { s.thumpL, s.thumpR };
        const float sideTex[2] = { s.texL, s.texR };
        const float sideLoad[2] = { s.loadL, s.loadR };
        // The ground under each tyre moves gradually towards what was read, so crossing from tarmac to gravel fades in over a
        // fraction of a second. Each side of the road effect mixes its front and rear tyre, the front weighing more because it is
        // what the steering feels, so left tyres on grass and right tyres on tarmac feel different. With the switch off, or with
        // nothing known about the ground, every value is neutral.
        Surf::Feel fes[2] = {Surf::kNeutral, Surf::kNeutral};                        // [0] = left, [1] = right
        if (c.fxSurface) {
            const float aw = alphaFor(dt, Surf::g_fade);
            const float cobScore = Surf::g_cobbleDetect ? cob.update(s.rough, s.speed, s.airborne > 0.5f, dt, Surf::g_cobbleLo, Surf::g_cobbleHi) : 0.0f;
            for (int w = 0; w < 4; ++w) gnd[w] = Road::Mix(gnd[w], Road::GroundForTyre(s.mat[w], s.surf[w]), aw);
            const float fw = Surf::g_front;
            for (int k2 = 0; k2 < 2; ++k2) {                                          // front tyre of this side is wheel k2, rear tyre is wheel 2 + k2
                Road::Drive dr;
                dr.spin = sideDyn(spinSm, k2); dr.slide = sideDyn(slideSm, k2);
                dr.activity = Road::Activity(spN, std::max(dr.spin, dr.slide));
                dr.ridge = cobScore;
                fes[k2] = Road::Derive(Road::Mix(gnd[2 + k2], gnd[k2], fw), dr);
            }
        }
        float texSide[2], loadGain[2];                                                // [0] = left, [1] = right
        for (int k2 = 0; k2 < 2; ++k2) {
            const float tm = texMono + (clamp01(sideTex[k2] + 0.9f * s.bump) - texMono) * kRoadDirectional;
            texSide[k2] = clamp01(tm + c.thumpGain * wf * thump[k2]);                 // a heavier car thumps harder on landing
            texSide[k2] = std::max(texSide[k2], fes[k2].floor);                            // smooth ground still gives a faint feel (snow, ice)
            loadGain[k2] = 1.0f + c.loadInfluence * (clampf(sideLoad[k2], 0.4f, 1.8f) - 1.0f);   // a more heavily loaded tyre transmits more
        }
        const float roadAir = 1.0f - c.airborneFade * s.airborne;                     // wheels off the ground: no road feel
        Road::SideIn rside[2];                                                        // [0] = left, [1] = right
        for (int k2 = 0; k2 < 2; ++k2) {
            rside[k2].fe = fes[k2];
            rside[k2].strength = c.roadMaxMag * 0.7f * texSide[k2] * gSpd * loadGain[k2] * roadAir * fes[k2].amp;
        }
        const Road::Targets plan = Road::Plan(rside, s.speed, c.roadMinHz, c.roadMaxHz);
        if (t >= nextJit) {                                                           // every ~12 ms, pick a new random strength per side and band, and a new pitch per band
            float grMix = 0.0f, txMax = 0.0f;
            for (int k2 = 0; k2 < 2; ++k2) {
                const Surf::Feel& fe = fes[k2];
                const float gr = clamp01(Road::kBaseGrain + fe.grainAdd);
                grMix += 0.5f * gr; txMax = std::max(txMax, texSide[k2]);
                const float lo = std::max(0.02f, 1.0f - 0.65f * gr - fe.dropout);     // strength varies between lo and 1
                const bool hit = fe.spark > 0.0f && rnd() * 0.5f + 0.5f < fe.spark;   // a stone, a crack: one hard hit, felt in every band
                for (int b = 0; b < Road::kBands; ++b)
                    bandAmpJit[k2][b] = hit ? fe.sparkMul : (lo + (1.0f - lo) * (rnd() * 0.5f + 0.5f)) * fe.base;
            }
            for (int b = 0; b < Road::kBands; ++b) bandHzJit[b] = 1.0f + (0.10f + 0.40f * grMix * (txMax > 0.3f ? 1.0f : 0.5f)) * rnd();
            engJit = rnd(); engHzJit = rnd();
            nextJit = t + 0.012;
        }
        float roadTarget[Road::kBands];
        Road::Combine(rside, plan.share, bandAmpJit, roadTarget);
        { const float cs = Road::CeilingScale(roadTarget, (float)c.roadMaxMag); for (int b = 0; b < Road::kBands; ++b) roadTarget[b] *= cs; }   // very rough ground can not make the road grow without limit
        for (int b = 0; b < Road::kBands; ++b)
            hz[S_ROAD1 + b] = clampf(plan.hz[b] * bandHzJit[b] / std::sqrt(wf), 20.0f, kHardMaxHz);   // heavier cars sound lower

        // ---------------- BRAKE: front-end judder, harsh on lock-up
        const float gB = smooth01(2.0f, 8.0f, s.speed);
        // Lock-up shakes the wheel according to the ground: soft ground swallows it, hard ground and cobbles pass it on.
        const float lockU = std::max(s.lock, lockAmt) * (c.fxSurface ? clampf(0.5f * (fes[0].amp + fes[1].amp), 0.4f, 1.3f) : 1.0f);
        const float Ab = clamp01(std::pow(s.brake, 1.2f) * (0.3f + 0.7f * s.dive) * 0.6f + lockU);
        hz[S_BRAKE] = lockU > 0.1f ? clampf(38.0f + 25.0f * rnd(), 25.0f, 70.0f) : clampf(s.frontRev * 2.0f, 12.0f, 60.0f);
        hz[S_BRAKE] *= c.brakeHzScale;
        float t4 = c.brakeMaxMag * Ab * gB * clampf(0.65f + 0.35f * s.loadF, 0.5f, 1.3f);   // more load on the fronts means more judder

        // ---------------- LANDING: no effect of its own. A detected landing boosts RBR's own steering force (see EffectTap):
        // a fast attack, a short hold, then a decay, scaled by how severe the landing was and by the live level.
        // A jump is rare, rough ground is not: bumps and washboard fire hit events one after another, and boosting RBR's force for each
        // of them makes the wheel heavy to turn. So the more events came in the last second or two, the less a new one counts. A single
        // jump is not held back at all; a second hit soon after is, and on rough ground the boost fades to almost nothing.
        evBusy *= (float)std::exp(-dt / 1.5f);
        if (s.evtSeq != seenEvt) { seenEvt = s.evtSeq; jAge = 0.0f; evGov = 1.0f / (1.0f + 1.2f * evBusy); evBusy += 1.0f; evAmp = s.evtSev * evGov; }
        if (jAge < 0.08f) evAmp = std::max(evAmp, s.evtSev * evGov);                 // the detector keeps refining a landing for its first ~100 ms
        jAge = std::min(jAge + dt, 5.0f);
        {
            const float ts = std::sqrt(wf);                                           // a heavier car holds the boost longer
            const float hold = 0.06f * ts, dec = 0.30f * ts;
            const float env = jAge < hold ? 1.0f : (float)std::exp(-(jAge - hold) / dec);
            const float lev = (c.fxLandBoost && jAge < 1.5f) ? clamp01(evAmp) * env * clampf(userGain, 0.0f, 1.0f) : 0.0f;
            boostMul = 1.0f + c.landingBoost * lev;
            boostAdd = c.landingAdd * lev;
        }

        // attack and release: road is deliberately quick (for sharpness), engine is rounder
        auto follow = [&](int i, float target, float up, float down) {
            sm[i] += (target - sm[i]) * (target > sm[i] ? up : down);
        };
        if (!c.fxEngine) { t1 = 0.0f; t2 = 0.0f; }
        follow(S_ENG1, t1, 0.35f, 0.15f);
        follow(S_ENG2, t2, 0.25f, 0.12f);
        const float roadDown = clampf(0.40f / wf, 0.15f, 0.70f);                     // heavier cars let the thump ring out longer
        // The road follows the ground under the side that is feeling more of it (equal weight when both are silent).
        const float sumSt = rside[0].strength + rside[1].strength, w0 = sumSt > 1e-6f ? rside[0].strength / sumSt : 0.5f;
        const float roadAtk = w0 * fes[0].atk + (1.0f - w0) * fes[1].atk;
        const float roadRel = clampf(roadDown * (w0 * fes[0].rel + (1.0f - w0) * fes[1].rel), 0.05f, 0.95f);   // gravel and ice let go fast, snow slowly
        for (int b = 0; b < Road::kBands; ++b) follow(S_ROAD1 + b, c.fxRoad ? roadTarget[b] : 0.0f, roadAtk, roadRel);
        follow(S_BRAKE, c.fxBrake ? t4 : 0.0f, 0.50f, 0.20f);
        // Priority: road beats engine. A lower layer backs off while a higher one is active.
        const float roadRms = std::sqrt(sm[S_ROAD1] * sm[S_ROAD1] + sm[S_ROAD2] * sm[S_ROAD2] + sm[S_ROAD3] * sm[S_ROAD3]);
        const float pR = c.roadMaxMag > 0 ? clamp01(roadRms / (float)c.roadMaxMag) : 0.0f;
        const float dk = clampf(c.priorityDuck, 0.0f, 1.0f);
        float duck[S_COUNT];
        duck[S_ROAD1] = duck[S_ROAD2] = duck[S_ROAD3] = duck[S_BRAKE] = 1.0f;
        duck[S_ENG1] = duck[S_ENG2] = 1.0f - dk * pR;
        float sumO = 0;
        for (int i = 0; i < S_COUNT; ++i) sumO += std::fabs(sm[i]) * duck[i];
        const float room = (float)c.totalMaxMag;
        const float scale = sumO > room && sumO > 0.0f ? room / sumO : 1.0f;
        const float sum = sumO * scale;

        // Slow thermal derate, based on the level actually delivered to the wheel (live gain included), so turning the
        // live gain down relieves the derate instead of stacking with it.
        const float ug = clampf(userGain, 0.0f, 1.0f);
        const float lvl = std::min(sum, (float)kHardTotalMax) / 10000.0f * ug;
        heat += (lvl * lvl - heat) * alphaFor(dt, 20.0f);
        const float derate = heat > c.sustainedLimit ? std::sqrt(c.sustainedLimit / heat) : 1.0f;

        const float g = ug * derate;
        for (int i = 0; i < S_COUNT; ++i) {
            float m = sm[i] * duck[i] * scale * g;
            if (!std::isfinite(m)) { m = 0.0f; sm[i] = 0.0f; }          // never let a NaN get stuck in the filters
            mag[i] = std::max(0, std::min((int)m, kHardTotalMax));
        }
    }
};

// ----------------------------------------------------------------- plugin
// ====================================================================================================================
// SURFACE READER: finds the tyre objects in RBR's memory and reports which kind of ground is under the car.
//
// Everything here only READS memory. The layout comes from the game's own code: a tyre is an
// object whose first dword is the vtable 0x74288C, and it holds three contact records at +0xC20, +0xC90 and +0xD00.
// A record is { +0x00 flag, +0x34 blend weight, +0x38 material A, +0x3C material B }; when the weight is under 0.5
// material A dominates. The objects are found once per stage (a scan of the game's heap, on this thread, at low
// priority), then read about 50 times a second, and only while telemetry is flowing (a stage is running). If anything is
// missing, the surface is "unknown" and nothing changes. Events go to ForceFX.log.
// ====================================================================================================================
namespace Surf {

class Reader {
public:
    std::atomic<int> clsWheel[4];            // ground class under each tyre: FL, FR, RL, RR
    std::atomic<int> ids[4];
    std::atomic<int> idWheel[4];             // material id of the ground each tyre settled on (-1 = none); the road feel takes the ground's properties from its name
    std::atomic<int> nTyres{0};
    std::atomic<int64_t> fed{0};
    std::atomic<bool> stage{false};
    std::atomic<bool> wanted{true};          // the menu switch: when off, nothing is scanned or read

    Reader() { for (auto& a : ids) a = -1; for (auto& a : idWheel) a = -1; for (auto& a : clsWheel) a = C_NONE; }

    void Begin(const std::string& iniPath) {
        ini_ = iniPath;
        for (int i = 0; i < 256; ++i) IdMap()[i] = 0;
        Load();
        run_ = true;
        th_ = std::thread(&Reader::Loop, this);
    }
    void End() { run_ = false; if (th_.joinable()) th_.join(); }
    std::string Status() const {
        std::lock_guard<std::mutex> lk(sm_);
        return status_;
    }

private:
    struct Rec { uint32_t flag = 0; float w = 0; uint32_t a = 0, b = 0; };
    struct Tyre { uintptr_t base = 0; Rec rec[3]; std::vector<uint8_t> blk; uint32_t blkChanges = 0; };
    struct Range { uintptr_t lo, hi; };

    static constexpr size_t kChunk = 4u << 20;
    static constexpr int kRecOff = 0xC20, kRecStride = 0x70, kRecCount = 3, kFlag = 0x00, kWeight = 0x34, kMatA = 0x38, kMatB = 0x3C;

    std::string ini_;
    std::thread th_;
    std::atomic<bool> run_{false};
    mutable std::mutex sm_;
    std::string status_ = "Surface: waiting for a stage";
    uint32_t vt_ = 0;
    int vtableCfg_ = 0x74288C;
    int readHz_ = 50, commitMs_ = 150;
    std::vector<Tyre> tyres_;
    void* chunk_ = nullptr;
    std::vector<Range> excl_;
    Clock::time_point nextScan_ = Clock::now();
    int retry_ = 0;
    struct Wheel { int committed = C_NONE, cand = C_NONE, candTicks = 0, noneTicks = 0; } wheel_[4];
    int wheelOf_[4] = {2, 3, 0, 1};          // which wheel (0 FL, 1 FR, 2 RL, 3 RR) each tyre object is, in the order the scan finds them
    uint32_t scans_ = 0;
    int failLogs_ = 0, classLogs_ = 0;

    void Log(const char* fmt, ...) {
        char b[300];
        va_list ap; va_start(ap, fmt);
        vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        Logf("Surface: %s", b);
    }
    void SetStatus(const char* fmt, ...) {
        char b[300];
        va_list ap; va_start(ap, fmt);
        vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        std::lock_guard<std::mutex> lk(sm_);
        status_ = b;
    }

    // ---------------------------------------------------------------- settings ([Surface] in ForceFX.ini)
    void Load() {
        const char* K = "Surface";
        char b[512];
        GetPrivateProfileStringA(K, "FadeSeconds", "", b, sizeof(b), ini_.c_str());
        const bool fresh = b[0] == 0;                                // no [Surface] section yet: write every key with its default so it can be edited
        auto Wr = [&](const char* k, const char* v) { WritePrivateProfileStringA(K, k, v, ini_.c_str()); };
        auto I = [&](const char* k, int& v, bool hex = false) {
            if (fresh) { char o[32]; snprintf(o, sizeof(o), hex ? "0x%X" : "%d", v); Wr(k, o); return; }
            GetPrivateProfileStringA(K, k, "", b, sizeof(b), ini_.c_str()); if (b[0]) v = (int)strtol(b, nullptr, 0);
        };
        auto F = [&](const char* k, float& v) {
            if (fresh) { char o[32]; snprintf(o, sizeof(o), "%g", v); Wr(k, o); return; }
            GetPrivateProfileStringA(K, k, "", b, sizeof(b), ini_.c_str()); if (b[0]) v = (float)atof(b);
        };
        F("FadeSeconds", g_fade); F("FrontWeight", g_front);
        I("CobbleDetect", g_cobbleDetect); F("CobbleDetectLow", g_cobbleLo); F("CobbleDetectHigh", g_cobbleHi);
        I("ReadHz", readHz_); I("CommitMs", commitMs_); I("TyreVtable", vtableCfg_, true);
        {   // which wheel each tyre object is, in the order the scan finds them (measured on test drives: rear pair first, then front pair)
            std::string order = "RL,RR,FL,FR";
            if (fresh) Wr("TyreOrder", order.c_str());
            else { GetPrivateProfileStringA(K, "TyreOrder", "", b, sizeof(b), ini_.c_str()); if (b[0]) order = b; }
            int map[4], n = 0;
            std::string tok;
            for (size_t i = 0; i <= order.size() && n < 4; ++i) {
                const char ch = i < order.size() ? (char)toupper((unsigned char)order[i]) : ',';
                if (ch == ',' || ch == ' ' || ch == ';') {
                    if (tok == "FL") map[n++] = 0; else if (tok == "FR") map[n++] = 1; else if (tok == "RL") map[n++] = 2; else if (tok == "RR") map[n++] = 3;
                    tok.clear();
                } else tok += ch;
            }
            bool ok = n == 4;
            for (int a = 0; ok && a < 4; ++a) for (int c2 = a + 1; c2 < 4; ++c2) if (map[a] == map[c2]) ok = false;
            if (ok) for (int a = 0; a < 4; ++a) wheelOf_[a] = map[a];
        }
        std::string map;
        if (fresh) Wr("Map", ""); else { GetPrivateProfileStringA(K, "Map", "", b, sizeof(b), ini_.c_str()); map = b; }
        ParseMap(map);
        g_fade = clampf(g_fade, 0.05f, 3.0f); g_front = clampf(g_front, 0.0f, 1.0f);
        g_cobbleDetect = g_cobbleDetect ? 1 : 0; g_cobbleLo = clampf(g_cobbleLo, 0.0f, 20.0f); g_cobbleHi = clampf(g_cobbleHi, g_cobbleLo + 0.05f, 40.0f);
        readHz_ = std::max(5, std::min(readHz_, 100)); commitMs_ = std::max(0, std::min(commitMs_, 2000));
    }
    // "88:snow,92:none,129:soft": correct the class of single ids
    void ParseMap(const std::string& m) {
        std::vector<std::string> bad;
        const int n = ParseIdMap(m, &bad);
        if (n > 0) Log("Map: %d surface override(s) applied", n);
        for (const std::string& b : bad) Log("Map: ignored '%s'. Types: tarmac, gravel, snow, ice, soft, cobble, wood, none", b.c_str());
    }

    // ---------------------------------------------------------------- memory
    static bool Rd(uintptr_t a, void* d, size_t n) {
        SIZE_T got = 0;
        return ReadProcessMemory(GetCurrentProcess(), (LPCVOID)a, d, n, &got) && got == n;
    }
    bool Excluded(uintptr_t a, size_t n) const {
        for (const Range& e : excl_) if (a < e.hi && a + n > e.lo) return true;
        return false;
    }
    bool ReadTyre(Tyre& t) {
        uint32_t v = 0;
        if (!Rd(t.base, &v, 4) || v != vt_) return false;                       // freed or reused
        uint8_t blk[0x1A0];
        const size_t need = (size_t)kRecStride * (kRecCount - 1) + 0x44;
        if (need > sizeof(blk) || !Rd(t.base + kRecOff, blk, need)) return false;
        if (t.blk.size() == need && std::memcmp(t.blk.data(), blk, need) != 0) ++t.blkChanges;
        t.blk.assign(blk, blk + need);
        for (int k = 0; k < kRecCount; ++k) {
            const uint8_t* r = blk + (size_t)k * kRecStride;
            std::memcpy(&t.rec[k].flag, r + kFlag, 4); std::memcpy(&t.rec[k].w, r + kWeight, 4);
            std::memcpy(&t.rec[k].a, r + kMatA, 4);    std::memcpy(&t.rec[k].b, r + kMatB, 4);
        }
        return true;
    }

    static bool RegionOk(const MEMORY_BASIC_INFORMATION& m) {
        if (m.State != MEM_COMMIT || m.Type == MEM_MAPPED) return false;
        if (m.Protect & (PAGE_GUARD | PAGE_NOACCESS | PAGE_NOCACHE | PAGE_WRITECOMBINE)) return false;   // never driver or GPU mappings
        const DWORD p = m.Protect & 0xFF;
        return p == PAGE_READWRITE || p == PAGE_WRITECOPY;
    }

    void FindTyres() {
        tyres_.clear(); ++scans_;
        const uintptr_t exe = (uintptr_t)GetModuleHandleA(nullptr);
        vt_ = (uint32_t)(exe + (uintptr_t)((uintptr_t)(unsigned)vtableCfg_ - 0x400000u));
        SYSTEM_INFO si; GetSystemInfo(&si);
        uintptr_t addr = (uintptr_t)si.lpMinimumApplicationAddress;
        const uintptr_t maxA = (uintptr_t)si.lpMaximumApplicationAddress;
        std::vector<uintptr_t> hits;
        size_t scanned = 0;
        while (addr < maxA && hits.size() < 64) {
            MEMORY_BASIC_INFORMATION m;
            if (VirtualQuery((LPCVOID)addr, &m, sizeof(m)) != sizeof(m)) break;
            const uintptr_t base = (uintptr_t)m.BaseAddress;
            const size_t size = (size_t)m.RegionSize;
            if (base + size <= addr) break;
            addr = base + size;
            if (!RegionOk(m) || size > ((size_t)64 << 20)) continue;
            for (size_t done = 0; done < size && hits.size() < 64;) {
                if (!run_ || stage.load()) return;
                const size_t n = std::min<size_t>((size_t)kChunk, size - done) & ~(size_t)3;
                if (!n) break;
                if (Rd(base + done, chunk_, n)) {
                    const uint32_t* p = static_cast<const uint32_t*>(chunk_);
                    for (size_t i = 0; i < n / 4; ++i)
                        if (p[i] == vt_ && !Excluded(base + done + i * 4, 4)) hits.push_back(base + done + i * 4);   // not our own copies
                    scanned += n;
                }
                done += n;
                Sleep(0);
            }
        }
        for (uintptr_t a : hits) { Tyre t; t.base = a; if (ReadTyre(t)) tyres_.push_back(t); }
        nTyres = (int)tyres_.size();
        if (!tyres_.empty()) { failLogs_ = 0; Log("found %u objects with the tyre vtable (scan %u, %u MB read); the ones that never change are ignored", (unsigned)tyres_.size(), scans_, (unsigned)(scanned >> 20)); }
        else if (failLogs_++ < 2) Log("no tyre objects yet (scan %u, %u objects with vtable 0x%X); will keep looking while the stage loads", scans_, (unsigned)hits.size(), (unsigned)vt_);
    }

    // The first time the car drives on a ground that has no class, say which id it is, so it can be assigned with Map=.
    // (The game's own table has no cobblestone or wooden bridge entry, so this is how those ids are found.)
    bool unmappedSeen_[256] = {};
    void NoteUnmapped(int id) {
        if (id < 0 || id > 255 || unmappedSeen_[id]) return;
        const rsa::MaterialInfo* known = nullptr;
        for (const rsa::MaterialInfo& m : rsa::kMaterials) if (m.id == id) { known = &m; break; }
        unmappedSeen_[id] = true;
        if (known && known->cls < 0) { Log("tyre contact id %d (%s) is an object or undefined, not ground; if this is a bridge or another special surface add Map=%d:wood (or cobble, gravel...) in [Surface]", id, known->name, id); return; }
        if (known) Log("ground material id %d (%s) has no surface class; add Map=%d:gravel (or tarmac, snow, ice, soft, cobble, wood) in [Surface] to give it one", id, known->name, id);
        else Log("ground material id %d is not in the game's table and has no surface class; if this is cobblestone or a wooden bridge add Map=%d:cobble (or wood) in [Surface]", id, id);
    }

    // Every ground id the tyres touch is listed once with the class it got, so a surface that is being read as the wrong class
    // (cobbles that share a tarmac id, say) shows up in the log as well as one that isn't known at all.
    bool idSeen_[256] = {};
    void NoteIdSeen(int id, int cls) {
        if (id < 0 || id > 255 || idSeen_[id]) return;
        idSeen_[id] = true;
        const rsa::MaterialInfo* known = nullptr;
        for (const rsa::MaterialInfo& m : rsa::kMaterials) if (m.id == id) { known = &m; break; }
        Log("ground id %d (%s) is read as %s", id, known ? known->name : "not in the game's table", ClsName(cls));
    }

    // A live tyre whose three contact records gave nothing usable. Logged once per distinct set of values (at most 10).
    std::vector<uint64_t> noRecSeen_;
    void NoteNoRecord(int w, const Rec* r) {
        if (noRecSeen_.size() >= 10) return;
        uint64_t sig = 1469598103934665603ULL;
        for (int k = 0; k < kRecCount; ++k) for (uint32_t v : {r[k].flag, r[k].a, r[k].b}) sig = (sig ^ v) * 1099511628211ULL;
        for (uint64_t s : noRecSeen_) if (s == sig) return;
        noRecSeen_.push_back(sig);
        static const char* const nm[4] = {"front left", "front right", "rear left", "rear right"};
        Log("%s: no usable contact record. records: [flag %u weight %.2f materials %u/%u] [flag %u weight %.2f materials %u/%u] [flag %u weight %.2f materials %u/%u]", nm[w],
            (unsigned)r[0].flag, (double)r[0].w, (unsigned)r[0].a, (unsigned)r[0].b, (unsigned)r[1].flag, (double)r[1].w, (unsigned)r[1].a, (unsigned)r[1].b,
            (unsigned)r[2].flag, (double)r[2].w, (unsigned)r[2].a, (unsigned)r[2].b);
    }
    bool deadLogged_ = false;

    // A contact record the plugin can't use (not flagged as touching, or a material number outside the table). Logged a few
    // times so a surface that reads as "unknown" can be explained instead of guessed at.
    int skippedLogs_ = 0;
    void NoteSkipped(int flag, uint32_t a, uint32_t b, float w) {
        if (skippedLogs_ >= 12) return;
        ++skippedLogs_;
        Log("tyre contact record skipped: flag %d, materials %u / %u, weight %.2f", flag, (unsigned)a, (unsigned)b, (double)w);
    }

    // ---------------------------------------------------------------- one reading
    void Tick() {
        int votes[4][C_COUNT] = {{0}};
        int voteId[4][C_COUNT];                                                  // the last id read for each class, so the winning class has an id to go with it
        for (auto& row : voteId) for (int& v : row) v = -1;
        int total[4] = {0, 0, 0, 0};
        int alive = 0, idx = 0;
        for (auto& a : ids) a = -1;
        for (Tyre& t : tyres_) {
            const int w = idx < 4 ? wheelOf_[idx] : -1;
            ++idx;
            if (!ReadTyre(t)) { t.blkChanges = 0; continue; }
            ++alive;
            if (w < 0 || t.blkChanges == 0) continue;                            // not proven live (a stale copy of an old stage never changes)
            int last = -1, usable = 0;
            for (int k = 0; k < kRecCount; ++k) {
                const Rec& r = t.rec[k];
                if (r.flag != 1 || r.a > 255 || r.b > 255) {
                    if (r.flag > 1 || (r.flag == 1 && (r.a > 255 || r.b > 255))) NoteSkipped((int)r.flag, r.a, r.b, r.w);   // flag 0 is just an unused record; anything else is worth knowing about
                    continue;
                }
                const int id = (int)(r.w < 0.5f ? r.a : r.b);
                last = id; ++usable;
                const int c = ClassOfId(id);
                NoteIdSeen(id, c);
                if (c != C_NONE) { ++votes[w][c]; ++total[w]; voteId[w][c] = id; }
                else NoteUnmapped(id);
            }
            if (usable == 0) NoteNoRecord(w, t.rec);
            ids[w] = last;
        }
        if (alive == 0) {
            if (!deadLogged_) { deadLogged_ = true; Log("the tyre objects can't be read any more (the stage ended or was reloaded): ground is unknown until they are found again"); }
            tyres_.clear(); nTyres = 0;
            retry_ = 0; nextScan_ = Clock::now() + std::chrono::seconds(3);
            ResetUnknown();
            return;
        }
        deadLogged_ = false;
        for (int w = 0; w < 4; ++w) {
            int best = C_NONE, bv = 0;
            for (int c = C_TARMAC; c < C_COUNT; ++c) if (votes[w][c] > bv) { bv = votes[w][c]; best = c; }
            Commit(w, total[w] >= 1 ? best : C_NONE, false, total[w] >= 1 ? voteId[w][best] : -1);
        }
    }

    // a new class has to hold for CommitMs before it is used, so a single stray record can't flicker the feel
    void Set(int w, int c, int id = -1) {
        Wheel& S = wheel_[w];
        static const char* const nm[4] = {"front left", "front right", "rear left", "rear right"};
        if (c != S.committed && classLogs_ < 300) {
            ++classLogs_;
            if (ids[w].load() >= 0) Log("%s ground is now %s (material id %d)", nm[w], ClsName(c), ids[w].load());
            else Log("%s ground is now %s (no material read: telemetry stopped, the stage ended or was reset)", nm[w], ClsName(c));
        }
        S.committed = c; clsWheel[w] = c; idWheel[w] = c == C_NONE ? -1 : id;
    }
    void Commit(int w, int c, bool now, int id = -1) {
        Wheel& S = wheel_[w];
        const int need = std::max(1, commitMs_ * readHz_ / 1000);
        if (now) { Set(w, c, id); S.cand = c; S.candTicks = 0; return; }
        if (c == C_NONE) {                                                       // nothing readable: hold the last class for a second
            if (++S.noneTicks >= readHz_) Set(w, C_NONE);
            return;
        }
        S.noneTicks = 0;
        if (c == S.committed) { S.cand = c; S.candTicks = 0; if (id >= 0) idWheel[w] = id; return; }   // same class, maybe a different depth or wetness: the id follows at once
        if (c != S.cand) { S.cand = c; S.candTicks = 1; } else ++S.candTicks;
        if (S.candTicks >= (S.committed == C_NONE ? std::max(1, need / 2) : need)) { Set(w, c, id); S.candTicks = 0; }
    }
    void Commit(int c, bool now) { for (int w = 0; w < 4; ++w) Commit(w, c, now); }
    // Telemetry stopped or the stage was reset: every tyre goes back to unknown. The ids are cleared first so the log doesn't
    // print the last ids it happened to read as if the car had just driven onto an unknown surface.
    void ResetUnknown() { for (auto& a : ids) a = -1; Commit(C_NONE, true); }
    bool AnyClass() const { for (const auto& a : clsWheel) if (a.load() != C_NONE) return true; return false; }

    void Loop() {
        try { LoopImpl(); } catch (...) { Log("the surface reader stopped after an exception"); }
    }
    void LoopImpl() {
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);   // never compete with the game for CPU
        chunk_ = VirtualAlloc(nullptr, kChunk, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        if (!chunk_) { Log("couldn't allocate the scan buffer"); return; }
        excl_.push_back(Range{(uintptr_t)chunk_, (uintptr_t)chunk_ + kChunk});                  // our own buffer holds copies of what we read
        excl_.push_back(Range{(uintptr_t)this, (uintptr_t)this + sizeof(*this)});               // and this object holds the vtable value
        typedef BOOL (WINAPI *LimitsFn)(PULONG_PTR, PULONG_PTR);
        LimitsFn fn = (LimitsFn)GetProcAddress(GetModuleHandleA("kernel32.dll"), "GetCurrentThreadStackLimits");
        ULONG_PTR slo = 0, shi = 0;
        if (fn) { fn(&slo, &shi); if (shi > slo) excl_.push_back(Range{(uintptr_t)slo, (uintptr_t)shi}); }
        while (run_) {
            Sleep((DWORD)(1000 / readHz_));
            const bool alive = NowMs() - fed.load() < 700;                       // telemetry only flows while a stage is running
            if (!wanted.load()) { if (AnyClass()) ResetUnknown(); SetStatus("Surface: off"); continue; }
            if (stage.exchange(false)) {
                tyres_.clear(); nTyres = 0; retry_ = 0; nextScan_ = Clock::now() + std::chrono::seconds(4);
                ResetUnknown();
            }
            if (!alive) { if (AnyClass()) ResetUnknown(); SetStatus("Surface: waiting for a stage"); continue; }
            if (tyres_.empty()) {
                if (Clock::now() >= nextScan_) {
                    FindTyres();
                    if (tyres_.empty()) { ++retry_; nextScan_ = Clock::now() + std::chrono::seconds(std::min(30, 5 * retry_)); }
                }
                SetStatus("Surface: looking for the tyres");
                continue;
            }
            Tick();
            SetStatus("Surface: FL %s, FR %s, RL %s, RR %s  (tyres %d, ids FL %d FR %d RL %d RR %d)", ClsName(clsWheel[0].load()), ClsName(clsWheel[1].load()), ClsName(clsWheel[2].load()), ClsName(clsWheel[3].load()), (int)tyres_.size(), ids[0].load(), ids[1].load(), ids[2].load(), ids[3].load());
        }
        VirtualFree(chunk_, 0, MEM_RELEASE); chunk_ = nullptr;
    }

    static int64_t NowMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
    }
public:
    static int64_t Now() { return NowMs(); }
};

Reader* g_reader = nullptr;

void Start(const std::string& ini) {
    if (g_reader) return;
    g_reader = new Reader();
    g_reader->Begin(ini);
    Logf("Surface feel: reader started (logging v3: every ground id the tyres touch is listed once, with its class)");
}
void Stop() { if (g_reader) { g_reader->End(); delete g_reader; g_reader = nullptr; } }
// Called while telemetry is flowing (a stage is running); 'want' is the menu switch.
void Feed(bool want) { if (g_reader) { g_reader->wanted = want; g_reader->fed = Reader::Now(); } }
void OnStage() { if (g_reader) g_reader->stage = true; }
int  Current(int wheel) { return g_reader ? g_reader->clsWheel[wheel & 3].load() : (int)C_NONE; }   // 0 FL, 1 FR, 2 RL, 3 RR
int  CurrentId(int wheel) { return g_reader ? g_reader->idWheel[wheel & 3].load() : -1; }          // the material id behind that class (-1 = none)
std::string Status() { return g_reader ? g_reader->Status() : std::string(); }
// The material id under each tyre (0 FL, 1 FR, 2 RL, 3 RR). False until all four are known; the dynamic audio reads these.
bool Ids(int out[4]) {
    if (!g_reader) return false;
    for (int i = 0; i < 4; ++i) { out[i] = g_reader->ids[i].load(); if (out[i] < 0) return false; }
    return true;
}
bool Found() { return g_reader && g_reader->nTyres.load() >= 4; }
// The raw ids and classes of the four tyres for the probe log (-1 = none).
void Raw(int id[4], int cls[4]) {
    for (int i = 0; i < 4; ++i) { id[i] = g_reader ? g_reader->ids[i].load() : -1; cls[i] = g_reader ? g_reader->clsWheel[i].load() : 0; }
}

}  // namespace Surf

struct DeviceInfo { GUID guid; char name[MAX_PATH]; };

class ForceFXPlugin : public IPlugin {
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
    std::atomic<int>       m_source{0};      // where the latest packet came from: 0 none yet, 1 NGP send hook, 2 UDP port
    std::atomic<float> m_mag[S_COUNT], m_hz[S_COUNT];
    std::atomic<float> m_dRpm{0}, m_dSpeed{0}, m_dThr{0}, m_dBrk{0}, m_dTex{0}, m_dRough{0}, m_dBump{0},
                       m_dDive{0}, m_dLock{0};
    std::atomic<bool> m_ffActive{false};
    std::atomic<float> m_userGain{0.85f};   // live level 0..1, changed by the hotkeys and the menu
    std::atomic<bool>  m_muted{false};
    std::atomic<float> m_dBoostMul{1.0f}, m_dBoostAdd{0.0f};   // current landing boost on RBR's force (shown in the menu)
    std::atomic<int>   m_dUpd{0}, m_dVib{0};   // effect updates sent to the wheel in the last second, and sends of the road vibration on RBR's force (shown in the menu)
    std::atomic<bool>  m_dViaRbr{false};   // the road is riding on RBR's force right now
    std::atomic<float> m_dHeat{1.0f};      // shown in the menu: 1 = no derating
    std::atomic<int>   m_dCar{-1};
    std::atomic<int>   m_cfgRev{0};         // bumped by the menu whenever m_cfg changes
    std::atomic<float> m_dStat{0}, m_dLoadL{1}, m_dLoadR{1}, m_dAir{0};
    std::atomic<bool>  m_stageReset{false};
    std::atomic<float> m_cobScore{0.0f}, m_cobLevel{0.0f};   // telemetry cobble detection, for the Diagnostics page, the log and the probe file
    FILE*              m_probe = nullptr;   // ForceFX_probe.csv, only while SurfaceProbe=1
    std::chrono::steady_clock::time_point m_probeT{};
    ffx::Audio         m_audio;            // sounds for the ground under each tyre; off until the menu switch is on (empty stand-in without FORCEFX_AUDIO)
    ffx::Camera        m_camera;           // head movement and windscreen effects (empty stand-in without FORCEFX_CAMERA)
    static ffx::Camera*& CameraSlot() { static ffx::Camera* p = nullptr; return p; }
    static void CameraSink(const uint8_t* d, int n) { if (ffx::Camera* c = CameraSlot()) c->feed(d, n); }   // called from NGP's send, so it works without a wheel connected
    std::atomic<bool>  m_carRescan{false};
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
        auto* self = static_cast<ForceFXPlugin*>(ref);
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
            SetStatus("Couldn't start DirectInput.");
            return;
        }
        // Prefer force feedback devices, but fall back to every game controller if none report FFB.
        m_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumCb, this, DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK);
        if (m_devices.empty())
            m_pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumCb, this, DIEDFL_ATTACHEDONLY);
        if (m_sel >= (int)m_devices.size()) m_sel = 0;
    }

    // ------------------------------------------------------- connect logic
    void Connect() {
        Disconnect();
        if (m_devices.empty()) { SetStatus("No force feedback wheel found."); return; }

        const DeviceInfo& dev = m_devices[m_sel];
        m_cfg.wheel = dev.name;
        WritePrivateProfileStringA("ForceFX", "Wheel", dev.name, m_iniPath.c_str());

        if (!GameDeviceTap::I().Install(m_pDI, dev.guid)) {
            SetStatus("Couldn't hook into DirectInput. See ForceFX.log for details.");
            return;
        }
        GameDeviceTap::I().Arm(dev.guid);
        m_connectedIdx = m_sel;
        m_packets = 0;
        m_source = 0;
        m_pktLen = 0;
        m_run = true;
        m_thread = std::thread(&ForceFXPlugin::Loop, this);
        SetStatus("Waiting for RBR to start using %s...", dev.name);
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
        FILE* f = fopen((g_dir + "ForceFX_packet.txt").c_str(), "w");
        if (!f) return;
        fprintf(f, "packet length = %d bytes\noffset  float32          raw hex     raw uint32\n", n);
        for (int i = 0; i + 4 <= n; i += 4) {
            float v; uint32_t u;
            std::memcpy(&v, b + i, 4); std::memcpy(&u, b + i, 4);
            fprintf(f, "0x%03X   %-16.6g 0x%08X  %u\n", i, (double)v, u, u);
        }
        fclose(f);
    }

    // Made-up signals for the test mode: a 13 second loop through the engine, road and brake effects, no telemetry needed.
    void TestSignals(double t, Signals& s) {
        s = Signals();
        s.speed = 25.0f; s.throttle = 1.0f;
        double ph = std::fmod(t, 13.0);
        if (ph < 4.0) {                                   // 0 engine sweep
            m_testPhase = 0; s.rpm = (float)(1000.0 + 6500.0 * (ph / 4.0));
        } else if (ph < 7.0) {                            // 1 gravel
            m_testPhase = 1; s.rpm = 3500; s.texture = 0.35f;
        } else if (ph < 10.0) {                           // 2 rough / dirt
            m_testPhase = 2; s.rpm = 4500; s.texture = 0.9f;
            const bool leftHit = std::fmod(ph - 7.0, 3.0) < 1.5;                      // 1.5 s left-side hits, then 1.5 s right-side
            s.texL = leftHit ? 0.9f : 0.15f; s.texR = leftHit ? 0.15f : 0.9f;
        } else {                                          // 3 hard braking, lock-up after 1.5 s
            m_testPhase = 3; s.rpm = 3000; s.throttle = 0; s.brake = 1.0f; s.dive = 1.0f; s.frontRev = 15.0f;
            if (ph > 11.5) s.lock = 0.9f;
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

    // Called from the worker thread every cycle. A key press counts once, then repeats quickly while it is held.
    void PollHotkeys(Clock::time_point now, const Config& cf) {
        static bool up = false, dn = false, mu = false;
        static Clock::time_point nextRepeat{};
        if (!RbrHasFocus() || (cf.keyModifier && !KeyDown(cf.keyModifier))) { up = dn = mu = false; return; }
        const bool bu = KeyDown(cf.keyGainUp), bd = KeyDown(cf.keyGainDown), bm = KeyDown(cf.keyMute);
        bool changed = false;
        float g = m_userGain.load();
        const float r = cf.gainRatio;
        if (bu && (!up || now >= nextRepeat)) { g = g < 0.02f ? 0.05f : g / r; changed = true; }
        if (bd && (!dn || now >= nextRepeat)) { g = g * r; if (g < 0.03f) g = 0.0f; changed = true; }
        if (changed) {
            nextRepeat = now + (((bu && up) || (bd && dn)) ? std::chrono::milliseconds(120) : std::chrono::milliseconds(350));
            if (g > 0.98f) g = 1.0f;
            g = clampf(g, 0.0f, 1.0f);
            m_userGain = g;
            m_muted = false;
            char b[32]; snprintf(b, sizeof(b), "%g", g);
            WritePrivateProfileStringA("ForceFX", "UserGain", b, m_iniPath.c_str());
            if (cf.gainBeep) BeepAsync(300 + (int)(g * 900.0f));          // the pitch of the beep tells you the level
            Logf("Live gain -> %.0f%%", g * 100.0f);
        }
        if (bm && !mu) {
            m_muted = !m_muted;
            if (cf.gainBeep) BeepAsync(m_muted ? 200 : 700);
            Logf("Vibration %s", m_muted ? "muted" : "unmuted");
        }
        up = bu; dn = bd; mu = bm;
    }

    // ------------------------------------------------------------ main loop
    void Loop() {
        RoadVib::Start();                      // the road vibration thread lives exactly as long as this one, whichever way it ends
        try { LoopImpl(); }
        catch (...) { Logf("The worker thread hit an exception; effects released"); m_ffActive = false; }
        RoadVib::Stop();
    }

    void LoopImpl() {
        Config base;                           // this thread's own copy of m_cfg, because the menu edits m_cfg on the UI thread
        { std::lock_guard<std::mutex> lk(m_mtx); base = m_cfg; }
        int seenRev = m_cfgRev.load();
        // Telemetry comes from NGP's send call (the hook in PacketTap), from our own UDP port, or from both. The hook wins
        // whenever it is delivering, so a setup that happens to feed both never counts a packet twice.
        const bool useTap = base.telemetrySource != 2 && PacketTap::Installed();
        const bool useUdp = base.telemetrySource != 1 && base.udpPort > 0;
        WSADATA wsa;
        bool wsaOk = WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
        SOCKET sock = INVALID_SOCKET;
        WSAEVENT sockEvent = WSA_INVALID_EVENT;
        if (wsaOk && useUdp) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock != INVALID_SOCKET) {
                sockaddr_in a{};
                a.sin_family = AF_INET;
                a.sin_port = htons((u_short)base.udpPort);
                a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   // telemetry is local only
                sockEvent = WSACreateEvent();
                if (bind(sock, (sockaddr*)&a, sizeof(a)) == SOCKET_ERROR || sockEvent == WSA_INVALID_EVENT ||
                    WSAEventSelect(sock, sockEvent, FD_READ) == SOCKET_ERROR) {
                    Logf("Couldn't listen on UDP port %d (WSA error %d)%s", base.udpPort, WSAGetLastError(),
                         useTap ? ". That's fine, the telemetry hook is in use" : ". Is another app using that port?");
                    if (sockEvent != WSA_INVALID_EVENT) { WSACloseEvent(sockEvent); sockEvent = WSA_INVALID_EVENT; }
                    closesocket(sock); sock = INVALID_SOCKET;
                }
            }
        }
        const bool haveSock = sock != INVALID_SOCKET;
        if (useTap) PacketTap::Activate();
        if (!useTap && !haveSock) Logf("No telemetry source is available, so only the test mode will work");
        else Logf("Telemetry source: %s%s%s", useTap ? "NGP send hook" : "", useTap && haveSock ? " + " : "",
                  haveSock ? "UDP port (backup)" : "");
        HANDLE waitOn[2]; DWORD nWait = 0;
        if (useTap) waitOn[nWait++] = PacketTap::g_event;
        if (haveSock) waitOn[nWait++] = sockEvent;

        std::vector<uint8_t> buf(2048);
        FFLayer ff;
        Dsp dsp;
        Synth synth;
        Signals live, sig;
        Config eff = base;                     // base settings plus the active car profile
        CarDb carDb;
        CarProfile prof;
        int curSlot = -2;                      // car slot from telemetry (Car.Index)
        std::string curKey;                    // name|rsfId|slot of the car the current profile was picked for
        Clock::time_point nextCarPoll{};       // the car is checked about once a second, with or without telemetry
        bool carForce = true;                  // check the car right away (stage start, telemetry came back, slot changed)
        bool dumped = false, prevTest = false;
        double testT0 = 0;

        const auto t0 = Clock::now();
        auto lastRx = t0 - std::chrono::seconds(10);
        bool cobOn = false; int cobLogs = 0;     // cobble detection state, for the log
        bool audioQuiet = false;               // the audio has been told that telemetry stopped
        auto lastPkt = t0, lastApply = t0, nextTry = t0, tapLastRx = t0 - std::chrono::seconds(10), nextRescan = t0, nextUpdCount = t0 + std::chrono::seconds(1);

        // Which car is this? The name comes from RSF / RBR (see ResolveCarIdentity). We keep polling rather than relying on
        // a few events, because the car is changed in the menus where no telemetry flows, and RSF may write its files a
        // moment after the stage loads. A different car, or an edited ForceFXCars.ini, reloads the profile.
        auto pollCar = [&]() {
            const bool reloaded = base.carProfiles ? carDb.Reload() : false;
            const CarIdent id = ResolveCarIdentity(curSlot);
            if (id.name.empty() && id.rsfId < 0 && (curSlot < 0 || !curKey.empty())) return;   // nothing readable right now, so keep the current car
            char kb[320]; snprintf(kb, sizeof(kb), "%s|%d|%d", id.name.c_str(), id.rsfId, curSlot);
            if (curKey == kb && !reloaded) return;
            const bool sameCar = (curKey == kb);
            curKey = kb;
            prof = CarProfile();
            float score = 0;
            if (base.carProfiles)
                if (const CarProfile* m = carDb.Match(id.name, id.rsfId, score)) prof = *m;
            ApplyProfile(base, prof, eff);
#ifdef FORCEFX_AUDIO
            m_audio.setCar(eff.drivetrain, eff.wheelbase, eff.cgToFront, eff.trackWidth);
#endif
            if (!sameCar) { dsp = Dsp(); synth.rpmPeak = 0; synth.grip = Tyre::GripLearner(); }           // a different car, so forget what the last one taught us
            char lab[200];
            if (prof.found) snprintf(lab, sizeof(lab), "-> %s%s%s (match %.2f)", prof.name.empty() ? prof.section.c_str() : prof.name.c_str(),
                                     prof.preset.empty() ? "" : ", ", prof.preset.c_str(), score);
            else snprintf(lab, sizeof(lab), "-> no matching profile in ForceFXCars.ini");
            { std::lock_guard<std::mutex> lk(m_mtx);
              m_carName = id.name.empty() ? "(name unknown)" : id.name;
              m_carPreset = lab; }
            m_dCar = curSlot;
            Logf("Car slot %d, RSFCarID %d, name \"%s\" %s  cyl=%d rpmMax=%.0f weightFeel=%.2f drivetrain=%s%s", curSlot, id.rsfId,
                 id.name.c_str(), lab, eff.cylinders, eff.rpmMax, eff.weightFeel, Tyre::DrivetrainName(eff.drivetrain), sameCar ? "  (profiles reloaded)" : "");
        };

        // One telemetry packet, from either source. ts is when it was sent (or arrived), so packets that pile up
        // in the queue still get the right time step between them.
        auto handlePacket = [&](const uint8_t* data, int n, Clock::time_point ts, int source) {
            m_packets++;
            m_source = source;
            const float pdt = std::max(0.0f, std::chrono::duration<float>(ts - lastPkt).count());
            lastPkt = ts; lastRx = ts;
            if (pdt > 1.0f) { carForce = true; m_audio.telemetryLost(); }   // telemetry is back after a pause: probably a new stage, maybe a new car
            if (m_stageReset.exchange(false)) {                 // the car is at the start line: learn its static load again
                m_audio.newStage();
                for (int i = 0; i < 4; ++i) dsp.statRef[i] = Ema();
                dsp.thumpL = dsp.thumpR = 0;
                dsp.airT = 0; dsp.landWin = 0; dsp.totLow = 1.0f;
            }
            ParsePacket(eff, data, n, live, dsp, pdt);
            m_audio.feed(data, n, std::chrono::duration<double>(ts.time_since_epoch()).count());
            if (source == 2) m_camera.feed(data, n);              // packets from the send hook already went to the camera
            if (live.carIndex >= 0 && live.carIndex != curSlot) { curSlot = live.carIndex; carForce = true; }
            {
                std::lock_guard<std::mutex> lk(m_mtx);
                m_pktLen = n;
                std::memcpy(m_pkt, data, n);
            }
            if (!dumped) { DumpPacket(data, n); dumped = true; }
        };

#ifdef FORCEFX_AUDIO
        m_audio.setActive(base.fxAudio != 0);
        m_audio.setLevels(base.audioDebris, base.audioImpact, base.audioSusp);
#endif
        while (m_run) {
            // Wait for a packet from either source (or 20 ms), then handle everything that has arrived.
            if (nWait) {
                WaitForMultipleObjects(nWait, waitOn, FALSE, 20);
                if (useTap) {
                    PacketTap::Slot sl;
                    while (PacketTap::Pop(sl)) {
                        if (sl.len < base.minPacket) continue;
                        tapLastRx = sl.ts;
                        handlePacket(sl.d, sl.len, sl.ts, 1);
                    }
                }
                if (haveSock) {
                    WSAResetEvent(sockEvent);
                    for (;;) {
                        const int n = recvfrom(sock, (char*)buf.data(), (int)buf.size(), 0, nullptr, nullptr);
                        if (n == SOCKET_ERROR) break;                          // nothing more waiting
                        const auto ts = Clock::now();
                        if (n < base.minPacket) continue;
                        if (useTap && std::chrono::duration<float>(ts - tapLastRx).count() < 1.0f) continue;   // the hook is delivering
                        handlePacket(buf.data(), n, ts, 2);
                    }
                }
            } else {
                Sleep(10);
            }
            const auto now = Clock::now();
            if (useTap && now >= nextRescan) { nextRescan = now + std::chrono::seconds(5); PacketTap::Rescan(); }

            if (m_cfgRev.load() != seenRev) {                  // the menu changed something: pick it up now
                seenRev = m_cfgRev.load();
                { std::lock_guard<std::mutex> lk(m_mtx); base = m_cfg; }
                ApplyProfile(base, prof, eff);
#ifdef FORCEFX_AUDIO
                m_audio.setCar(eff.drivetrain, eff.wheelbase, eff.cgToFront, eff.trackWidth);
                m_audio.setActive(base.fxAudio != 0);
                m_audio.setLevels(base.audioDebris, base.audioImpact, base.audioSusp);
#endif
            }
            {   // telemetry cobble detection: copy its state for the Diagnostics page and log when it switches on and off
                m_cobScore = synth.cob.score; m_cobLevel = synth.cob.ema;
                if (!cobOn && synth.cob.score > 0.6f) {
                    cobOn = true;
                    if (++cobLogs <= 200) Logf("Cobbles detected from the telemetry (roughness per speed %.2f, %.0f km/h): tarmac tyres now get the cobble feel", (double)synth.cob.ema, (double)(live.speed * 3.6f));
                } else if (cobOn && synth.cob.score < 0.3f) {
                    cobOn = false;
                    if (++cobLogs <= 200) Logf("Cobbles no longer detected (roughness per speed %.2f)", (double)synth.cob.ema);
                }
            }
            if (base.probe) WriteProbe(live, now);
            if (m_audio.msSinceFrame() > 500) { if (!audioQuiet) { audioQuiet = true; m_audio.telemetryLost(); } }   // menus, pause, stage end, or packets that are not real telemetry: let the loops fade out
            else audioQuiet = false;
            if (m_carRescan.exchange(false)) carForce = true;
            if (carForce || now >= nextCarPoll) {
                carForce = false;
                nextCarPoll = now + std::chrono::milliseconds(1000);
                pollCar();
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
                sig = Signals();                   // no telemetry (stage over or paused): fade to zero
            }

            m_dRpm = sig.rpm; m_dSpeed = sig.speed; m_dThr = sig.throttle; m_dBrk = sig.brake;
            m_dTex = sig.texture; m_dRough = sig.rough; m_dBump = sig.bump;
            m_dDive = sig.dive; m_dLock = std::max(sig.lock, synth.lockAmt);
            m_dStat = sig.staticLoad; m_dLoadL = sig.loadL; m_dLoadR = sig.loadR; m_dAir = sig.airborne;
            {   // the ground class is only read while telemetry is flowing (a stage is running)
                const bool telemLive = !m_test && std::chrono::duration<double>(now - lastRx).count() < 0.4;
                if (telemLive) Surf::Feed(base.fxSurface != 0
#ifdef FORCEFX_AUDIO
                                       || base.fxAudio != 0
#endif
                                      );
                for (int w = 0; w < 4; ++w) { sig.surf[w] = telemLive ? Surf::Current(w) : 0; sig.mat[w] = telemLive ? Surf::CurrentId(w) : -1; }
            }

            PollHotkeys(now, base);
            const bool testNow = m_test.load();
            if (testNow && !prevTest) testT0 = t;
            prevTest = testNow;
            // The test mode fades in over one second so it never starts with a jolt.
            const float testRamp = testNow ? clamp01((float)(t - testT0)) : 1.0f;
            const float live_gain = m_muted.load() ? 0.0f : m_userGain.load() * testRamp;

            int mag[S_COUNT]; float hz[S_COUNT];
            synth.Run(eff, sig, t, dt, live_gain, mag, hz);
            // Landing boost: EffectTap applies it to RBR's own constant-force updates (not in test mode, where RBR sends no force)
            g_boostMul = m_test ? 1.0f : synth.boostMul; g_boostAdd = m_test ? 0.0f : synth.boostAdd;
            m_dBoostMul = synth.boostMul; m_dBoostAdd = synth.boostAdd;
            if (now >= nextUpdCount) { nextUpdCount = now + std::chrono::seconds(1); m_dUpd = g_fxUpdates.exchange(0); m_dVib = g_vibSends.exchange(0); }
            m_dHeat = synth.heat > base.sustainedLimit ? std::sqrt(base.sustainedLimit / synth.heat) : 1.0f;
            DWORD per[S_COUNT];
            for (int i = 0; i < S_COUNT; ++i) {
                hz[i] = std::isfinite(hz[i]) ? clampf(hz[i], kHardMinHz, kHardMaxHz) : kHardMinHz;
                m_mag[i] = (float)mag[i]; m_hz[i] = hz[i];
                per[i] = (DWORD)(1000000.0f / hz[i]);
            }
            // Road vibration through RBR's own force: hand the three bands to the vibration thread and keep them off the wheel's own
            // effects. Without RBR's constant-force updates (no hook yet, test mode, menus), or when switched off in the settings,
            // the road is merged into one sine effect of its own.
            const bool viaRbr = eff.roadViaRbr != 0 && RoadVib::Ready();
            if (viaRbr) {
                for (int b = 0; b < Road::kBands; ++b) { RoadVib::g_mag[b] = mag[S_ROAD1 + b]; RoadVib::g_hz[b] = hz[S_ROAD1 + b]; mag[S_ROAD1 + b] = 0; }
            } else {                                          // the road as one effect of its own: all three bands merged into one sine
                float bm[Road::kBands], bh[Road::kBands], amp, pitch;
                for (int b = 0; b < Road::kBands; ++b) { bm[b] = (float)mag[S_ROAD1 + b]; bh[b] = hz[S_ROAD1 + b]; mag[S_ROAD1 + b] = 0; }
                Road::Consolidate(bm, bh, amp, pitch);
                mag[S_ROAD2] = (int)std::lround(clampf(amp, 0.0f, 10000.0f));
                hz[S_ROAD2] = clampf(pitch, kHardMinHz, kHardMaxHz);
                per[S_ROAD2] = (DWORD)(1000000.0f / hz[S_ROAD2]);
            }
            g_vibOn = viaRbr;
            m_dViaRbr = viaRbr;

            // ------------------------------------------------ send to the wheel
            if (!ff.Ready()) {
                if (now >= nextTry) {
                    nextTry = now + std::chrono::seconds(1);
                    IDirectInputDevice8A* d = GameDeviceTap::I().Captured();
                    if (!d) {
                        SetStatus("Waiting for RBR to use the wheel. Is force feedback enabled in RBR?");
                    } else {
                        HRESULT hr = ff.Create(d);
                        if (SUCCEEDED(hr)) {
                            m_ffActive = true;
                            SetStatus("Active, running alongside RBR's force feedback. Select on the Wheel row starts the test.");
                        } else {
                            m_ffActive = false;
                            Logf("Creating effects failed: 0x%08X", (unsigned)hr);
                            if (hr == DIERR_NOTEXCLUSIVEACQUIRED || hr == DIERR_OTHERAPPHASPRIO) {
                                // This isn't the device object that holds the wheel exclusively (RBR/NGP may use several).
                                GameDeviceTap::I().ReleaseCaptured();
                                SetStatus("That wasn't the device object RBR holds exclusively (0x%08X). Still looking.", (unsigned)hr);
                            } else {
                                SetStatus("Couldn't create effects: 0x%08X (E_NOTIMPL usually means the device has no force feedback)", (unsigned)hr);
                            }
                        }
                    }
                }
            } else {
                if (GameDeviceTap::I().ConsumeDropRequest()) {
                    ff.Destroy();
                    GameDeviceTap::I().ReleaseCaptured();
                    m_ffActive = false;
                    SetStatus("RBR let go of the wheel. Waiting for it to come back...");
                    continue;
                }
                if (GameDeviceTap::I().ConsumeReacquired()) ff.Restart();
                if (!ff.Push(mag, per)) {
                    Logf("Lost the effects (0x%08X), recreating them", (unsigned)ff.LastHr());
                    SetStatus("Lost the effects (0x%08X), trying again.", (unsigned)ff.LastHr());
                    ff.Destroy();
                    m_ffActive = false;
                }
            }
        }

        if (ff.Ready()) {                       // settle at zero before the effects are torn down
            int z[S_COUNT] = {};
            DWORD p[S_COUNT];
            for (int i = 0; i < S_COUNT; ++i) p[i] = 20000;
            ff.Push(z, p);
            Sleep(30);
        }
        ff.Destroy();
        m_audio.telemetryLost();
        if (useTap) PacketTap::Deactivate();
        if (sock != INVALID_SOCKET) {
            WSAEventSelect(sock, sockEvent, 0);
            closesocket(sock);
        }
        if (sockEvent != WSA_INVALID_EVENT) WSACloseEvent(sockEvent);
        if (wsaOk) WSACleanup();
    }

    // ------------------------------------------------------------ in-game settings menu (UI thread only)
    // One row per adjustable value. Edits go into m_cfg under the mutex, the worker thread picks them up through m_cfgRev
    // (so the wheel changes straight away, test mode included), and the changed keys are saved to ForceFX.ini.
    // Every edit goes through ClampConfig(), so the menu can never leave the safe ranges (hard ceilings included).
    struct MenuRow {
        const char* label; const char* key;
        int Config::* ip; float Config::* fp;     // exactly one of the two is set
        float lo, hi, step;
        char fmt;                                 // 'g' magnitude, '%' percent, 'H' Hz, 'x' multiplier, 'o' on/off switch
        bool live;                                // this is the live level, which is mirrored into m_userGain
    };
    std::vector<MenuRow> m_rows;
    int  m_cursor = 0, m_top = 0, m_view = 0;     // m_view 0 = settings, 1 = raw packet diagnostics
    bool m_resetArmed = false;

    void BuildRows() {
        const float M = (float)kHardTotalMax;
        m_rows = {
#ifdef FORCEFX_AUDIO
            {"Dynamic audio",                    "EnableDynamicAudio", &Config::fxAudio,      nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
            {"Audio: debris level",              "AudioDebrisLevel",   nullptr, &Config::audioDebris, 0.0f,   4.0f, 0.1f,  'x', false},
            {"Audio: impact / landing level",    "AudioImpactLevel",   nullptr, &Config::audioImpact, 0.0f,   4.0f, 0.1f,  'x', false},
            {"Audio: suspension level",          "AudioSuspensionLevel", nullptr, &Config::audioSusp, 0.0f,   4.0f, 0.1f,  'x', false},
#endif
#ifdef FORCEFX_CAMERA
            {"Camera movement",                  "EnableCamera",       &Config::fxCamera,     nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
            {"Windscreen effects (restart)",     "EnableWindscreen",   &Config::fxWindscreen, nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
#endif
            // effect switches (turn effects off to feel the others on their own)
            {"Effect: Engine",                   "EnableEngine",       &Config::fxEngine,     nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
            {"Effect: Road feel",                "EnableRoad",         &Config::fxRoad,       nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
            {"Road feel delivery",               "RoadViaRbr",         &Config::roadViaRbr,   nullptr,  0.0f,   1.0f, 1.0f, 'r', false},
            {"Effect: Surface feel",             "EnableSurfaceFeel",  &Config::fxSurface,    nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
            {"Effect: Braking",                  "EnableBrake",        &Config::fxBrake,      nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
            {"Effect: Landing boost",            "EnableLandingBoost", &Config::fxLandBoost,  nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
            // gains
            {"Live level (Numpad +/- in game)", "UserGain",           nullptr, &Config::userGain,      0.0f,  1.0f, 0.05f, '%', true },
            {"Total ceiling",                    "TotalMaxMagnitude",  &Config::totalMaxMag, nullptr,   500.0f, M,    100.0f, 'g', false},
            {"Engine gain",                      "EngineMaxMagnitude", &Config::engineMaxMag, nullptr,  0.0f,   M,    100.0f, 'g', false},
            {"Road gain",                        "RoadMaxMagnitude",   &Config::roadMaxMag,   nullptr,  0.0f,   M,    100.0f, 'g', false},
            {"Brake gain",                       "BrakeMaxMagnitude",  &Config::brakeMaxMag,  nullptr,  0.0f,   M,    100.0f, 'g', false},
            {"Landing boost (x RBR force)",      "LandingBoost",       nullptr, &Config::landingBoost,  0.0f,  4.0f, 0.1f,  '%', false},
            {"Landing minimum push",             "LandingMinPush",     nullptr, &Config::landingAdd,    0.0f,  1.0f, 0.05f, '%', false},
            {"Landing sensitivity",              "ImpactGain",         nullptr, &Config::impactGain,    0.0f,  3.0f, 0.05f, 'x', false},
            {"Airborne load level",              "LandingAirLevel",    nullptr, &Config::airLevel,      0.2f,  0.8f, 0.05f, 'x', false},
            {"Landing trigger level",            "LandingThreshold",   nullptr, &Config::landingThresh, 0.5f,  3.0f, 0.1f,  'x', false},
            {"Priority ducking",                 "PriorityDuck",       nullptr, &Config::priorityDuck,  0.0f,  1.0f, 0.05f, '%', false},
            // frequencies
            {"Engine min frequency",             "EngineMinHz",        nullptr, &Config::engineMinHz,   kHardMinHz, kHardMaxHz - 5.0f, 2.0f, 'H', false},
            {"Engine max frequency",             "EngineMaxHz",        nullptr, &Config::engineMaxHz,   kHardMinHz, kHardMaxHz, 2.0f, 'H', false},
            {"Brake pitch",                      "BrakeHzScale",       nullptr, &Config::brakeHzScale,  0.5f,  2.0f, 0.05f, 'x', false},
        };
    }

    float RowValue(const MenuRow& r) const {
        if (r.live) return m_userGain.load();
        return r.ip ? (float)(m_cfg.*r.ip) : m_cfg.*r.fp;
    }

    void RowText(const MenuRow& r, char* out, size_t n) const {
        const float v = RowValue(r);
        switch (r.fmt) {
            case 'g': snprintf(out, n, "%d", (int)std::lround(v)); break;
            case '%': snprintf(out, n, "%.0f%%", v * 100.0f); break;
            case 'H': snprintf(out, n, "%.0f Hz", v); break;
            case 'o': snprintf(out, n, "%s", v >= 0.5f ? "ON" : "OFF"); break;
            case 'r': snprintf(out, n, "%s", v >= 0.5f ? "Through RBR's force" : "Own effect"); break;
            default:  snprintf(out, n, "x%.2f", v); break;
        }
    }

    // The camera works with or without a wheel connected, so its switches are applied here and not in the telemetry loop.
    void ApplyCameraSwitches() {
#ifdef FORCEFX_CAMERA
        m_camera.setActive(m_cfg.fxCamera != 0);
        m_camera.setWindscreen(m_cfg.fxWindscreen != 0);
#endif
    }

    void SaveRow(const MenuRow& r) {
        char b[32]; snprintf(b, sizeof(b), "%g", (double)RowValue(r));
        WritePrivateProfileStringA("ForceFX", r.key, b, m_iniPath.c_str());
    }

    // dir is +1 or -1. Clamps the result, applies it live, and saves every row the edit changed (moving one frequency can push its partner).
    void AdjustRow(int ri, int dir) {
        std::vector<float> before;
        for (const MenuRow& r : m_rows) before.push_back(RowValue(r));
        const MenuRow& r = m_rows[ri];
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            float v = clampf(RowValue(r) + (float)dir * r.step, r.lo, r.hi);
            v = std::round(v * 1000.0f) / 1000.0f;                                  // stops float drift like 0.35000002 after many steps
            if (r.ip) m_cfg.*r.ip = (int)std::lround(v); else m_cfg.*r.fp = v;
            if (r.live) { m_userGain = v; m_muted = false; }
            ClampConfig(m_cfg);
        }
        m_cfgRev++;
        ApplyCameraSwitches();
        for (size_t i = 0; i < m_rows.size(); ++i)
            if (RowValue(m_rows[i]) != before[i]) SaveRow(m_rows[i]);
        Logf("Menu: %s -> %g", r.key, (double)RowValue(r));
    }

    void ResetRows() {
        const Config def{};
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            for (const MenuRow& r : m_rows) {
                if (r.ip) m_cfg.*r.ip = def.*r.ip; else m_cfg.*r.fp = def.*r.fp;
                if (r.live) { m_userGain = def.*r.fp; m_muted = false; }
            }
            ClampConfig(m_cfg);
        }
        m_cfgRev++;
        ApplyCameraSwitches();
        for (const MenuRow& r : m_rows) SaveRow(r);
        Logf("Menu: switches, gains and frequencies reset to the defaults");
    }

    // RBR's WriteText works like printf, so a literal % has to be doubled or it swallows the next argument
    void WriteSafe(float x, float y, const char* t) {
        std::string e;
        for (; *t; ++t) { e += *t; if (*t == '%') e += '%'; }
        game_->WriteText(x, y, e.c_str());
    }

    // A short plain-language description of each setting, shown under the list for the highlighted row.
    static const char* TipFor(const char* key) {
        struct T { const char* k; const char* t; };
        static const T tips[] = {
#ifdef FORCEFX_AUDIO
            {"EnableDynamicAudio", "Plays tyre and ground sounds that follow each wheel: rolling noise, squeal, sliding, flying stones, bumps and landings, with the sound of each surface. Needs sound files in ForceFXAudio\\sounds; the mix levels are in the [Audio] section of ForceFX.ini. Off by default; loading takes a moment when you switch it on."},
            {"AudioDebrisLevel",   "How loud the stones and debris thrown at the car are, on top of StoneLevel in [Audio] of ForceFX.ini. Changes apply while you drive."},
            {"AudioImpactLevel",   "How loud bump impacts and landings are, on top of EventLevel in [Audio] of ForceFX.ini."},
            {"AudioSuspensionLevel", "How loud suspension creaks and bottoming-out thumps are, on top of SqueakLevel and BottomOutLevel in [Audio] of ForceFX.ini."},
#endif
#ifdef FORCEFX_CAMERA
            {"EnableCamera",       "Moves your head in the cockpit view with the car: leaning into braking and corners, nodding, bouncing over bumps and shaking on crashes and landings. Strength and everything else is in the [Camera] sections of ForceFX.ini (Intensity is the main one). Changes apply at once."},
            {"EnableWindscreen",   "Rain, snow and mud on the windscreen of the cockpit view, with wipers. It is off by default and is read at startup, so after switching it on restart RBR. Settings are in the [Windscreen] section of ForceFX.ini."},
#endif
            {"EnableEngine",       "Turns the engine vibration on or off. Handy for feeling what each effect does on its own."},
            {"EnableRoad",         "Turns the road feel (texture, bumps and ground) on or off."},
            {"RoadViaRbr",         "How the road vibration reaches the wheel. Through RBR's force: it is added to RBR's own steering force, so the wheel gets no extra road effect. Own effect: the road is one sine effect of its own, less detailed but easy on the wheel. If the road makes the wheel feel heavy or sticky to turn, use Through RBR's force; it may need a higher road gain. Without RBR's force updates it falls back to the own effect by itself."},
            {"EnableBrake",        "Turns the braking judder and front-wheel lock-up shake on or off."},
            {"EnableSurfaceFeel",  "Lets the ground under each tyre shape the road feel, together with how fast the car goes and how much the wheels slip: loose ground is grainy and throws stones, soft ground is dull, slippery hard ground is quiet with sharp cracks, cobbles hammer in time with your speed. Needs the road feel effect. Off, the road feels the same on every surface."},
            {"EnableLandingBoost", "Turns the landing boost on or off. With it off, landings feel exactly as RBR sends them."},
            {"UserGain",           "Master volume for everything ForceFX adds. It only turns things down, never up past your other limits. Numpad + and - change it while you drive."},
            {"TotalMaxMagnitude",  "The most that engine, road and brake can add together. If the wheel feels too busy or too strong, start here."},
            {"EngineMaxMagnitude", "How strong the engine vibration is. It's the quietest layer and steps aside for road and landings."},
            {"RoadMaxMagnitude",   "How strong the road feel is. This is the one setting for it: the ground and what the car is doing decide what it feels like. Higher makes everything from gravel to kerbs feel rougher."},
            {"BrakeMaxMagnitude",  "How strong the braking judder and front-wheel lock-up shake are."},
            {"ImpactGain",         "How easily a landing or hard hit counts as a big one. Turn it up if small jumps feel too soft."},
            {"LandingAirLevel",    "How light the car has to get before it counts as airborne. Raise it to catch smaller hops and crests."},
            {"LandingThreshold",   "How hard a hit has to be to count when the car stays on the ground. Raise it if braking or weight shifts set it off."},
            {"LandingBoost",       "Extra pull added to RBR's own steering force at the peak of a full landing. At 200%, the wheel gets three times the force RBR was sending."},
            {"LandingMinPush",     "A minimum extra pull on landing, in the direction RBR is already pulling. It helps when RBR's force happens to be small at touchdown."},
            {"PriorityDuck",       "How far engine and road back off while something more important is playing. At 0% nothing backs off."},
            {"EngineMinHz",        "Pitch of the engine vibration at idle. Lower is deeper and slower."},
            {"EngineMaxHz",        "Pitch of the engine vibration at the rev limit."},
            {"BrakeHzScale",       "Pitch of the braking judder. Below x1.00 it's slower and coarser, above it's faster and finer."},
        };
        for (const T& t : tips) if (std::strcmp(t.k, key) == 0) return t.t;
        return "";
    }

    // Wraps text onto several screen lines, breaking at spaces.
    void DrawWrapped(const char* text, float x, float& y, size_t maxChars) {
        std::string rest(text);
        while (!rest.empty()) {
            std::string ln;
            if (rest.size() <= maxChars) { ln = rest; rest.clear(); }
            else {
                size_t cut = rest.rfind(' ', maxChars);
                if (cut == std::string::npos || cut == 0) cut = maxChars;
                ln = rest.substr(0, cut);
                rest = rest.substr(cut);
                while (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
            }
            WriteSafe(x, y, ln.c_str());
            y += 19.0f;
        }
    }

    void DrawRawPage() {
        float y = 50.0f;
        game_->SetFont(IRBRGame::FONT_BIG);
        game_->SetMenuColor(IRBRGame::MENU_HEADING);
        WriteSafe(70.0f, y, "ForceFX Diagnostics");
        y += 28.0f;
        game_->SetFont(IRBRGame::FONT_SMALL);
        game_->SetMenuColor(IRBRGame::MENU_TEXT);
        char buf[300];
        auto line = [&](const char* txt) { WriteSafe(70.0f, y, txt); y += 19.0f; };
        line("Up/Down scrolls the raw packet. Select, Left or Right goes back to the settings.");
        snprintf(buf, sizeof(buf), "rpm %.0f  speed %.1f m/s  throttle %.2f  brake %.2f",
                 m_dRpm.load(), m_dSpeed.load(), m_dThr.load(), m_dBrk.load());
        line(buf);
        snprintf(buf, sizeof(buf), "texture %.2f (front roughness %.2f)  bump %.2f  dive %.2f  lock %.2f",
                 m_dTex.load(), m_dRough.load(), m_dBump.load(), m_dDive.load(), m_dLock.load());
        line(buf);
        snprintf(buf, sizeof(buf), "static load %.0f   load L %.2f R %.2f   airborne %.2f   thermal derate x%.2f",
                 m_dStat.load(), m_dLoadL.load(), m_dLoadR.load(), m_dAir.load(), m_dHeat.load());
        line(buf);
        line(Surf::Status().c_str());
#ifdef FORCEFX_AUDIO
        line(m_audio.status().c_str());
#endif
#ifdef FORCEFX_CAMERA
        line(m_camera.status().c_str());
#endif
        { char cb[160]; snprintf(cb, sizeof(cb), "Cobble detection: %s, roughness per speed %.2f (blend starts %.1f, full at %.1f), cobble feel %d%%",
                                 Surf::g_cobbleDetect ? "on" : "off", (double)m_cobLevel.load(), (double)Surf::g_cobbleLo, (double)Surf::g_cobbleHi, (int)(m_cobScore.load() * 100.0f + 0.5f));
          line(cb); }
        y += 6.0f;

        uint8_t pkt[2048]; int len;
        { std::lock_guard<std::mutex> lk(m_mtx); len = m_pktLen; std::memcpy(pkt, m_pkt, sizeof(pkt)); }
        snprintf(buf, sizeof(buf), "RBR force updates seen %d   last in %d   out %d",
                 g_rbrSeen.load(), g_rbrRaw.load(), g_rbrOut.load());
        line(buf);
        snprintf(buf, sizeof(buf), "send hook: %s", PacketTap::Installed() ? PacketTap::Describe().c_str() : "not installed");
        line(buf);
        snprintf(buf, sizeof(buf), "sends seen %lld   taken %lld   other destination %lld   dropped %lld",
                 (long long)PacketTap::g_calls.load(), (long long)PacketTap::g_taken.load(),
                 (long long)PacketTap::g_otherDest.load(), (long long)PacketTap::g_dropped.load());
        line(buf);
        line("Raw packet floats:");
        for (int row = 0; row < 12; ++row) {
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
            WriteSafe(70.0f, y, ln); y += 17.0f;
        }
    }

public:
    explicit ForceFXPlugin(IRBRGame* game) : game_(game) {
        g_dir = pluginDirectory();
        m_iniPath = g_dir + "ForceFX.ini";
        std::memset(m_pkt, 0, sizeof(m_pkt));
        for (int i = 0; i < S_COUNT; ++i) { m_mag[i] = 0; m_hz[i] = 0; }
        auto fileExists = [](const std::string& f) { return GetFileAttributesA(f.c_str()) != INVALID_FILE_ATTRIBUTES; };
        bool exists = fileExists(m_iniPath);
        if (!exists) SyncConfig(m_cfg, m_iniPath, true);     // first run: write an ini with every key and its default
        SyncConfig(m_cfg, m_iniPath, false);
        m_userGain = m_cfg.userGain;
        Surf::Start(m_iniPath);               // reads which ground is under the tyres
#ifdef FORCEFX_AUDIO
        Logf("Build: with dynamic audio module");
#else
        Logf("Build: without dynamic audio module");
#endif
        {
            ffx::Audio::Hooks h;
            h.readIds = [](int ids[4]) { return Surf::Ids(ids); };
            h.idsAvailable = []() { return Surf::Found(); };
            h.idsStatus = []() { return Surf::Status(); };
            h.log = [](const std::string& s) { Logf("%s", s.c_str()); };
            m_audio.configure(g_dir, h);
            m_audio.setSpeedUnits(m_cfg.speedIsKph != 0);
        }
        {
            ffx::Camera::Hooks h;
            h.log = [](const std::string& s) { Logf("%s", s.c_str()); };
#ifdef FORCEFX_CAMERA
            Logf("Build: with camera module");
            m_camera.configure(g_dir, h, m_cfg.fxCamera != 0, m_cfg.fxWindscreen != 0);
#else
            Logf("Build: without camera module");
#endif
        }

        if (m_cfg.telemetrySource != 2) {
            const bool ok = PacketTap::Install(m_cfg.minPacket);
            Logf("Telemetry hook: %s (%s)", ok ? "installed" : "not available", PacketTap::Describe().c_str());
#ifdef FORCEFX_CAMERA
            CameraSlot() = &m_camera;
            PacketTap::SetSink(&ForceFXPlugin::CameraSink);
#endif
        }

        BuildRows();
        SetStatus("Choose your wheel with Left/Right, then press Select.");
        EnumerateDevices();

        if (!m_cfg.wheel.empty()) {
            std::string want = LowerStr(m_cfg.wheel);
            for (int i = 0; i < (int)m_devices.size(); ++i) {
                if (LowerStr(m_devices[i].name).find(want) != std::string::npos) {
                    m_sel = i;
                    if (m_cfg.autoConnect) Connect();
                    break;
                }
            }
        }
    }

    // One line ten times a second while SurfaceProbe=1: what the tyres touch and what the road signals do there.
    // Drive tarmac, then a cobbled stretch, then a bridge, and the columns show how they differ.
    void WriteProbe(const Signals& s, std::chrono::steady_clock::time_point now) {
        if (now - m_probeT < std::chrono::milliseconds(100)) return;
        m_probeT = now;
        if (!m_probe) {
            m_probe = fopen((g_dir + "ForceFX_probe.csv").c_str(), "w");
            if (!m_probe) return;
            fprintf(m_probe, "t_s,speed_ms,id_fl,id_fr,id_rl,id_rr,cls_fl,cls_fr,cls_rl,cls_rr,rough,piston,texture,texL,texR,bump,airborne,loadL,loadR,cobble_level,cobble_score\n");
        }
        int id[4], cls[4]; Surf::Raw(id, cls);
        fprintf(m_probe, "%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.3f,%.3f,%.3f,%.3f\n",
                std::chrono::duration<double>(now.time_since_epoch()).count(), (double)s.speed, id[0], id[1], id[2], id[3], cls[0], cls[1], cls[2], cls[3],
                (double)s.rough, (double)s.piston, (double)s.texture, (double)s.texL, (double)s.texR, (double)s.bump, (double)s.airborne, (double)s.loadL, (double)s.loadR, (double)m_cobLevel.load(), (double)m_cobScore.load());
        fflush(m_probe);
    }

    ~ForceFXPlugin() {
        if (m_probe) { fclose(m_probe); m_probe = nullptr; }
        PacketTap::SetSink(nullptr);       // no more packets for the camera
        Disconnect();                      // stops the telemetry thread, the only one that feeds the audio
        m_audio.shutdown();
        m_camera.shutdown();
        Surf::Stop();
        GameDeviceTap::I().Remove();       // put the original vtable entries back before our DLL goes away
        if (!PacketTap::Remove()) {
            PinThisDll();
            Logf("Another hook was chained onto the Winsock send calls after ours, so the DLL stays pinned in memory");
        }
        if (m_pDI) { m_pDI->Release(); m_pDI = nullptr; }
    }

    const char* GetName() override { return "ForceFX"; }

    void DrawFrontEndPage() override {
        if (!game_) return;
        if (m_view == 1) { DrawRawPage(); return; }

        const int nVal = (int)m_rows.size();
        const int total = nVal + 4;                 // wheel, test, the settings rows, diagnostics, reset
        const int iView = 2 + nVal, iReset = iView + 1;
        constexpr int kVisible = 10;
        if (m_cursor < m_top) m_top = m_cursor;
        if (m_cursor >= m_top + kVisible) m_top = m_cursor - kVisible + 1;
        m_top = std::max(0, std::min(m_top, total - kVisible));

        float y = 40.0f;
        game_->SetFont(IRBRGame::FONT_BIG);
        game_->SetMenuColor(IRBRGame::MENU_HEADING);
        WriteSafe(70.0f, y, "ForceFX Settings");
        y += 28.0f;

        game_->SetFont(IRBRGame::FONT_SMALL);
        game_->SetMenuColor(IRBRGame::MENU_TEXT);
        char buf[300];
        auto line = [&](const char* txt) { WriteSafe(70.0f, y, txt); y += 19.0f; };

        std::string st;
        { std::lock_guard<std::mutex> lk(m_mtx); st = m_status; }
        snprintf(buf, sizeof(buf), "Status: %s", st.c_str());
        line(buf);
        static const char* const kPhase[] = {"engine sweep", "gravel", "rough / dirt", "hard braking + lock-up"};
        const int src = m_source.load();
        char from[48];
        if (src == 1) snprintf(from, sizeof(from), "from NGP");
        else if (src == 2) snprintf(from, sizeof(from), "from UDP port %d", m_cfg.udpPort);
        else snprintf(from, sizeof(from), "(none yet)");
        snprintf(buf, sizeof(buf), "Telemetry: %lld packets %s   live gain %.0f%%%s   derate x%.2f",
                 (long long)m_packets.load(), from, m_userGain.load() * 100.0f, m_muted ? " MUTED" : "", m_dHeat.load());
        line(buf);
        { std::string nm, pr; { std::lock_guard<std::mutex> lk(m_mtx); nm = m_carName; pr = m_carPreset; }
          snprintf(buf, sizeof(buf), "car slot %d: %s %s", m_dCar.load(), nm.empty() ? "waiting for telemetry" : nm.c_str(), pr.c_str());
          line(buf); }
        snprintf(buf, sizeof(buf), "now: engine %.0f@%.0fHz  lump %.0f  road low %.0f mid %.0f high %.0f  brake %.0f@%.0fHz",
                 m_mag[S_ENG1].load(), m_hz[S_ENG1].load(), m_mag[S_ENG2].load(),
                 m_mag[S_ROAD1].load(), m_mag[S_ROAD2].load(), m_mag[S_ROAD3].load(), m_mag[S_BRAKE].load(), m_hz[S_BRAKE].load());
        line(buf);
        snprintf(buf, sizeof(buf), "landing boost on RBR's force: x%.2f  +%.0f     effect updates to the wheel: %d/s", m_dBoostMul.load(), m_dBoostAdd.load() * 10000.0f, m_dUpd.load());
        line(buf);
        if (m_dViaRbr.load()) snprintf(buf, sizeof(buf), "road: through RBR's force, %d sends/s (RBR updates seen: %d)", m_dVib.load(), g_rbrSeen.load());
        else snprintf(buf, sizeof(buf), "road: one wheel effect of its own%s", m_cfg.roadViaRbr ? " (waiting for RBR's force updates)" : "");
        line(buf);
        y += 6.0f;

        for (int i = m_top; i < std::min(total, m_top + kVisible); ++i) {
            const bool sel = (i == m_cursor);
            char label[120], value[160];
            if (i == 0) {
                snprintf(label, sizeof(label), "Wheel (Select = connect)");
                if (m_devices.empty()) snprintf(value, sizeof(value), "[no force feedback wheel found]");
                else snprintf(value, sizeof(value), "%s%s", m_sel == m_connectedIdx ? "[on] " : "", m_devices[m_sel].name);
            } else if (i == 1) {
                snprintf(label, sizeof(label), "Test vibration (Select)");
                snprintf(value, sizeof(value), "%s", m_test ? kPhase[std::max(0, std::min(m_testPhase.load(), 3))] : "off");
            } else if (i == iView) {
                snprintf(label, sizeof(label), "Diagnostics / raw packet");
                snprintf(value, sizeof(value), "Select to open");
            } else if (i == iReset) {
                snprintf(label, sizeof(label), "Reset gains, frequencies and switches");
                snprintf(value, sizeof(value), "%s", m_resetArmed ? "Select again to confirm" : "Select to reset");
            } else {
                const MenuRow& r = m_rows[i - 2];
                snprintf(label, sizeof(label), "%s", r.label);
                char v[64]; RowText(r, v, sizeof(v));
                snprintf(value, sizeof(value), sel ? "< %s >" : "%s", v);
            }
            game_->SetMenuColor(sel ? IRBRGame::MENU_SELECTION : IRBRGame::MENU_TEXT);
            snprintf(buf, sizeof(buf), "%s%s", sel ? "> " : "  ", label);
            WriteSafe(70.0f, y, buf);
            WriteSafe(330.0f, y, value);
            y += 19.0f;
        }
        game_->SetMenuColor(IRBRGame::MENU_TEXT);
        y += 4.0f;
        snprintf(buf, sizeof(buf), "Up/Down: move (%d/%d)   Left/Right: change   Changes apply live and are saved",
                 m_cursor + 1, total);
        line(buf);
        {   // tooltip for the highlighted row
            const char* tip = "";
            if (m_cursor == 0) tip = "Pick your wheel with Left/Right, then press Select to connect. It will be remembered for next time.";
            else if (m_cursor == 1) tip = "A built-in demo of engine, gravel, dirt and braking vibration, so you can tune without driving. Landings work through RBR's own force, so they aren't part of it.";
            else if (m_cursor == iView) tip = "Shows live telemetry values and the raw packet. Handy when something doesn't look right.";
            else if (m_cursor == iReset) tip = "Puts the switches, gains and frequencies on this page back to their defaults. Press Select twice to confirm.";
            else tip = TipFor(m_rows[m_cursor - 2].key);
            game_->SetMenuColor(IRBRGame::MENU_HEADING);
            y += 4.0f;
            DrawWrapped(tip, 70.0f, y, 88);
            game_->SetMenuColor(IRBRGame::MENU_TEXT);
        }
        line("Per-car profiles in ForceFXCars.ini adjust the engine and road gains for each car.");
    }

    void DrawResultsUI() override {}

    void HandleFrontEndEvents(char, bool bUp, bool bDown, bool bLeft, bool bRight, bool bSelect) override {
        if (m_view == 1) {
            if (bUp)   m_tableBase = std::max(0, m_tableBase - 64);
            if (bDown) m_tableBase = std::min(2048 - 64, m_tableBase + 64);
            if (bSelect || bLeft || bRight) m_view = 0;
            return;
        }
        const int nVal = (int)m_rows.size();
        const int total = nVal + 4;
        const int iView = 2 + nVal, iReset = iView + 1;
        if (bUp)   { m_cursor = (m_cursor + total - 1) % total; m_resetArmed = false; }
        if (bDown) { m_cursor = (m_cursor + 1) % total;         m_resetArmed = false; }
        const int dir = (bRight ? 1 : 0) - (bLeft ? 1 : 0);

        if (m_cursor == 0) {
            if (m_devices.empty()) return;
            if (dir) m_sel = (m_sel + dir + (int)m_devices.size()) % (int)m_devices.size();
            if (bSelect) {
                if (m_connectedIdx == m_sel && m_run) m_test = !m_test;   // already connected, so Select toggles the test vibration
                else Connect();
            }
        } else if (m_cursor == 1) {
            if (bSelect || dir) {
                if (m_connectedIdx >= 0 && m_run) m_test = !m_test;
                else SetStatus("Connect to your wheel first (press Select on the Wheel row).");
            }
        } else if (m_cursor == iView) {
            if (bSelect) m_view = 1;
        } else if (m_cursor == iReset) {
            if (bSelect) {
                if (m_resetArmed) { ResetRows(); m_resetArmed = false; }
                else m_resetArmed = true;
            }
        } else if (dir) {
            AdjustRow(m_cursor - 2, dir);
        } else if (bSelect && m_rows[m_cursor - 2].fmt == 'o') {           // Select flips an on/off switch
            AdjustRow(m_cursor - 2, RowValue(m_rows[m_cursor - 2]) >= 0.5f ? -1 : 1);
        }
    }

    void TickFrontEndPage(float) override {}
    void StageStarted(int iMap, const char*, bool) override { m_stageReset = true; m_carRescan = true; Surf::OnStage(); m_camera.stageStarted(iMap); }
    void HandleResults(float, float, float, const char*) override {}
    void CheckPoint(float, int, const char*) override {}
};

} // namespace

extern "C" __declspec(dllexport) IPlugin* RBR_CreatePlugin(IRBRGame* pGame) {
    try { return new ForceFXPlugin(pGame); }
    catch (...) { return nullptr; }
}
