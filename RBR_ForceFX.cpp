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
//   Road     Two sawtooth effects with opposite sharp edges and a bit of random pitch and strength jitter. They
//            follow how rough the front tyres feel (high-passed strut force relative to the average load, the same
//            idea RbrSurfaceAudio uses) plus damper velocity. Smooth tarmac is silent, gravel is gritty, and
//            rough ground or kerbs turn violent and sharp.
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
// Branch "memory": a temporary memory scanner (see the MemScan block) that reads the surface id under each tyre.
//   Everything it adds is marked MEMSCAN or sits inside namespace MemScan, so it can be deleted in one go.
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
    int   fxEngine = 1, fxRoad = 1, fxBrake = 1, fxLandBoost = 1;

    // per-layer ceilings (DirectInput units 0..10000), a combined ceiling, and one live level (userGain)
    int   engineMaxMag = 1200, roadMaxMag = 3000, brakeMaxMag = 2000;
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
    float lockThreshold = 0.12f;     // how far the front wheels must slow relative to the car before it counts as lock-up

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
    float roadGrain = 0.9f;          // 0 = smooth buzz, 1 = coarse, irregular grain (random strength and pitch)
    float roadDirectional = 0.7f;    // 0 = both sides identical, 1 = left and right impacts fully separate
    int   roadSideInvert = 0;        // 1 = swap which direction each side's impacts pull
    float maxSpeed = 40.0f;          // m/s that counts as full speed

    // weight and suspension load (relative to the car's own static load, so units never matter)
    float loadInfluence = 0.6f;      // 0 = ignore tyre load, 1 = strongly favour the more heavily loaded side
    float airborneFade = 1.0f;       // 1 = road and engine go quiet when the wheels are off the ground

    // per-car character (these are the global values; a car profile can override them)
    float weightFeel = 1.2f;         // >1 heavier: stronger thump, slower decay, lower road pitch
    float lumpScale = 1.0f;          // idle lump / beat strength
    float unevenness = 0.0f;         // uneven firing (boxer, five-cylinder, cross-plane V8)
    int   carProfiles = 1;           // use ForceFXCars.ini

    // NGP packet offsets (float32 unless noted)
    int   offCarIndex = 56;
    int   offThrottle = 28, offBrake = 32, offSpeed = 60, offAccHeave = 120, offRpm = 136;
    int   offRpmMax = -1;            // float32 rev limit in the packet (-1 = this packet doesn't have one)
    int   offSuspBase = 152, suspStride = 128, offStrut = 16, offPiston = 28, offSegment = 60;   // segment = uint32
    int   speedIsKph = 0;
    float rpmScale = 1.0f;

    // live level and hotkeys (polled while RBR is the foreground window). Keys are VK codes, 0 = disabled.
    float userGain = 1.0f;           // saved live level 0..1 (it can only turn things down)
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
    c.priorityDuck = clampf(c.priorityDuck, 0.0f, 1.0f);
    c.fxEngine = c.fxEngine ? 1 : 0; c.fxRoad = c.fxRoad ? 1 : 0; c.fxBrake = c.fxBrake ? 1 : 0; c.fxLandBoost = c.fxLandBoost ? 1 : 0;
    c.landingBoost = clampf(c.landingBoost, 0.0f, 4.0f);
    c.landingAdd = clampf(c.landingAdd, 0.0f, 1.0f);
    c.landingThresh = clampf(c.landingThresh, 0.5f, 3.0f);
    c.airLevel = clampf(c.airLevel, 0.2f, 0.8f);
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
    I("EnableEngine", c.fxEngine); I("EnableRoad", c.fxRoad); I("EnableBrake", c.fxBrake);
    I("EnableLandingBoost", c.fxLandBoost); F("LandingBoost", c.landingBoost); F("LandingMinPush", c.landingAdd); F("LandingThreshold", c.landingThresh); F("LandingAirLevel", c.airLevel);
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
    I("OffsetAccHeave", c.offAccHeave); I("OffsetRpm", c.offRpm); I("OffsetRpmMax", c.offRpmMax);
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
            size_t eq = line.find('=');
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
}

// ------------------------------------------------ landing boost on RBR's own force
// RBR drives the wheel with its own DirectInput constant-force effect. Rather than stacking another effect on top, a
// detected landing scales that force on its way to the wheel: every SetParameters call RBR makes on a constant-force
// effect is multiplied by g_boostMul, plus a minimum extra push (g_boostAdd, a share of full torque) in the direction
// RBR is already pushing. Our own effects are periodic, so they never go through this path.
std::atomic<float> g_boostMul{1.0f}, g_boostAdd{0.0f};
std::atomic<int>   g_rbrSeen{0}, g_rbrRaw{0}, g_rbrOut{0};      // for the diagnostics page: updates seen, last magnitude in and out
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
                if (raw > 0) f += add * 10000.0f; else if (raw < 0) f -= add * 10000.0f;
                out = (LONG)clampf(f, -10000.0f, 10000.0f);
            }
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

enum Slot { S_ENG1 = 0, S_ENG2, S_ROAD, S_ROADR, S_BRAKE, S_COUNT };   // S_ROAD is a sawtooth-up (sharp edge to the left), S_ROADR a sawtooth-down (sharp edge to the right)
const char* const kSlotName[S_COUNT] = {"engine", "lump", "road-A", "road-B", "brake"};

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
        for (int i = 0; i < S_COUNT; ++i) if (m_fx[i]) { EffectTap::Patch(*reinterpret_cast<void***>(m_fx[i])); break; }   // RBR's effects share this class
        Logf("Effects created: %d of %d", ok, (int)S_COUNT);
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
        DIPERIODIC p{};
        p.dwMagnitude = (DWORD)mag; p.lOffset = 0; p.dwPhase = 0; p.dwPeriod = period;
        e.cbTypeSpecificParams = sizeof(p); e.lpvTypeSpecificParams = &p;
        HRESULT hr = m_fx[slot]->SetParameters(&e, DIEP_TYPESPECIFICPARAMS | DIEP_NORESTART);
        if (SUCCEEDED(hr)) { m_lastMag[slot] = mag; m_lastPer[slot] = period; }
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

struct Signals {
    float rpm = 0, speed = 0 /* m/s */, throttle = 0, brake = 0;
    float rpmMaxTelem = 0;   // rev limit straight from the telemetry (0 = not available)
    float rough = 0;      // raw front-weighted tyre roughness
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
    Ema sfVar[4], pistonVar[4], loadShort[4], rev[4];
    Ema loadRef, rollRatio, baseFront;
    uint32_t lastSeg[4] = {0, 0, 0, 0};
    bool haveSeg[4] = {false, false, false, false};
    float bumpEnv = 0, diveSm = 0, lockSm = 0;
    Ema statRef[4], rLsm, rRsm, rFsm;
    float thumpL = 0, thumpR = 0;
    // landing / impact detector (works on raw, unsmoothed load so the spike doesn't get averaged away)
    float peakRw = 0, totRatio = 1;
    float airT = 0, landWin = 0, landAir = 0, landPeak = 0, landBump = 0, landJump = 0, imRefr = 0;
    unsigned evtSeq = 0; float evtSev = 0;
    float totLow = 1.0f;   // running minimum of the total strut load (relative): a sudden rise above it means something hit the car
};

void ParsePacket(const Config& c, const uint8_t* b, int n, Signals& s, Dsp& d, float dt) {
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
    s.rough = 0.75f * rF + 0.25f * rR;
    const float roughLevel = std::pow(clamp01((s.rough - c.roughLow) / (c.roughHigh - c.roughLow)), c.roughGamma);
    s.piston = std::pow(clamp01(std::max(pRms[0], pRms[1]) * c.pistonScale), 1.2f);
    s.texture = clamp01(std::max(roughLevel, 0.8f * s.piston));

    // ---- the same thing per side (LF/LB on the left, RF/RB on the right); the fronts count most
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
            if (d.airT > 0.04f && d.landWin <= 0.0f) {
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
            const float sev = severity(d.peakRw, 0.0f, clamp01(d.bumpEnv), jump);
            if (sev > 0.12f) {
                d.imRefr = 0.15f;
                fire(sev);
                Logf("hit: severity %.2f  (peak load %.2f, load jump %.2f, heave %.2f)", sev, d.peakRw, jump, clamp01(d.bumpEnv));
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

    const float rr = 0.5f * (d.rev[0].v + d.rev[1].v);
    s.frontRev = rr;
    if (s.throttle < 0.05f && s.brake < 0.02f && s.speed > 8.0f && rr > 0.5f)
        d.rollRatio.step(rr / s.speed, alphaFor(dt, 1.0f));      // while coasting, learn how many tyre revs per second one m/s is
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
// Turns the analysed signals into effect strengths and frequencies. Runs on the worker thread, about 100 times a second.
struct Synth {
    float sm[S_COUNT] = {};
    float thrSm = 0, lope = 0, rpmPeak = 0, heat = 0;
    float jAge = 5.0f, evAmp = 0.0f;                       // age and severity of the current landing event
    unsigned seenEvt = 0;
    float boostMul = 1.0f, boostAdd = 0.0f;                // landing boost for RBR's own force (the worker thread hands it to the hook)
    float roadJit[2] = {1.0f, 1.0f}, ampJit[2] = {1.0f, 1.0f}, engJit = 1.0f, engHzJit = 1.0f, unevPhase = 0;
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

        // ---------------- ROAD: two sawtooth effects with opposite sharp edges (zero mean, so they never shift RBR's
        // own force). S_ROAD snaps to the left and S_ROADR to the right, and each is fed by one side's tyre
        // roughness, so a hit on one side is felt as a snap in one direction.
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
            texSide[k2] = clamp01(tm + c.thumpGain * wf * thump[k2]);                 // a heavier car thumps harder on landing
            loadGain[k2] = 1.0f + c.loadInfluence * (clampf(sideLoad[k2], 0.4f, 1.8f) - 1.0f);   // a more heavily loaded tyre transmits more
        }
        const float roadAir = 1.0f - c.airborneFade * s.airborne;                     // wheels off the ground: no road feel
        if (t >= nextJit) {                                                           // every ~12 ms, pick a new random strength and pitch per side
            for (int k2 = 0; k2 < 2; ++k2) {
                const float tx = texSide[k2];
                roadJit[k2] = 1.0f + (0.10f + 0.40f * c.roadGrain * (tx > 0.3f ? 1.0f : 0.5f)) * rnd();
                const float lo = 1.0f - 0.65f * c.roadGrain;                          // strength varies between lo and 1
                ampJit[k2] = lo + (1.0f - lo) * (rnd() * 0.5f + 0.5f);
            }
            engJit = rnd(); engHzJit = rnd();
            nextJit = t + 0.012;
        }
        float t3[2];
        for (int k2 = 0; k2 < 2; ++k2) {
            const float spd = c.roadMinHz + (c.roadMaxHz - c.roadMinHz) * clamp01(0.35f * spN + 0.65f * texSide[k2]);
            hz[k2 == 0 ? S_ROAD : S_ROADR] = spd * roadJit[k2] * (k2 ? 1.31f : 1.0f) / std::sqrt(wf);   // the two sides are detuned; heavier cars sound lower
            t3[k2] = c.roadMaxMag * 0.7f * texSide[k2] * gSpd * ampJit[k2] * loadGain[k2] * roadAir;     // the two road effects share the road budget
        }
        // A has its sharp edge to the left, B to the right. By default a hit on the left tyre snaps left.
        const int sA = c.roadSideInvert ? 1 : 0, sB = 1 - sA;
        const float tRoadA = t3[sA], tRoadB = t3[sB];
        { const float hzA = hz[sA == 0 ? S_ROAD : S_ROADR], hzB = hz[sB == 0 ? S_ROAD : S_ROADR];
          hz[S_ROAD] = hzA; hz[S_ROADR] = hzB; }

        // ---------------- BRAKE: front-end judder, harsh on lock-up
        const float gB = smooth01(2.0f, 8.0f, s.speed);
        const float Ab = clamp01(std::pow(s.brake, 1.2f) * (0.3f + 0.7f * s.dive) * 0.6f + s.lock);
        hz[S_BRAKE] = s.lock > 0.1f ? clampf(38.0f + 25.0f * rnd(), 25.0f, 70.0f) : clampf(s.frontRev * 2.0f, 12.0f, 60.0f);
        hz[S_BRAKE] *= c.brakeHzScale;
        float t4 = c.brakeMaxMag * Ab * gB * clampf(0.65f + 0.35f * s.loadF, 0.5f, 1.3f);   // more load on the fronts means more judder

        // ---------------- LANDING: no effect of its own. A detected landing boosts RBR's own steering force (see EffectTap):
        // a fast attack, a short hold, then a decay, scaled by how severe the landing was and by the live level.
        if (s.evtSeq != seenEvt) { seenEvt = s.evtSeq; jAge = 0.0f; evAmp = s.evtSev; }
        if (jAge < 0.08f) evAmp = std::max(evAmp, s.evtSev);                         // the detector keeps refining a landing for its first ~100 ms
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
        follow(S_ROAD,  c.fxRoad ? tRoadA : 0.0f, 0.55f, roadDown);
        follow(S_ROADR, c.fxRoad ? tRoadB : 0.0f, 0.55f, roadDown);
        follow(S_BRAKE, c.fxBrake ? t4 : 0.0f, 0.50f, 0.20f);
        // Priority: road beats engine. A lower layer backs off while a higher one is active.
        const float pR = c.roadMaxMag > 0 ? clamp01(std::max(std::fabs(sm[S_ROAD]), std::fabs(sm[S_ROADR])) / (float)c.roadMaxMag) : 0.0f;
        const float dk = clampf(c.priorityDuck, 0.0f, 1.0f);
        float duck[S_COUNT];
        duck[S_ROAD] = duck[S_ROADR] = duck[S_BRAKE] = 1.0f;
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
// TEMPORARY: RBR memory scanner (branch "memory")
//
// Goal: find where RBR keeps the surface type under each tyre, so ForceFX can give gravel, snow and ice their own
// feel. This whole block is throwaway. Once the addresses are known, delete it, the lines marked MEMSCAN further
// down, and the [MemScan] section of ForceFX.ini.
//
// How it finds the value (v4: find the tyre objects, read the ids)
//   The game's own code (found by disassembling RichardBurnsRally_SSE.exe) gave the exact layout, so nothing has to be
//   guessed any more:
//     * every tyre is an object of one class whose first dword is the vtable pointer 0x74288C (a fixed address, the exe
//       has no ASLR). Scanning memory for that one value finds all tyre objects.
//     * each tyre object holds three surface-contact records (three probe points across the tread) at tyre+0xC20,
//       +0xC90 and +0xD00. In a record: +0x00 contact flag, +0x34 blend weight (float), +0x38 material A, +0x3C material B.
//     * a material id is below 256; the manager that owns the materials sits behind [0x1659B70] (see below).
//   The scanner finds the tyres (Numpad * or automatically a few seconds after a stage starts), then reads the records
//   about 20 times a second and logs every id change together with speed and roughness. ForceFX_surfaces.txt lists every
//   id that was seen, how often, and the average roughness/texture/speed while it was under the tyres. That is the table
//   needed to give each surface its own feel. Pressing a surface key (Numpad 1..8, names from Label1..8) while driving
//   only annotates the log and the table with what you say you're on; it is not needed to find anything.
//
//   The old drive-and-label search (Numpad 1..8 with snapshots) is still there behind Passive=1, but it assumed the
//   id stays constant on a surface, which is false in a real stage, so it is off by default.
//
//   Keys:  Numpad *  find the tyre objects now     Numpad .  write both reports now     Numpad 0  forget everything
//          Numpad 1..8  tell the log what surface you are on    Numpad 9  clear that annotation
//
//   What the game's own code tells us (RichardBurnsRally_SSE.exe, found by disassembly):
//     * a surface is a material id below 256. Each tyre contact keeps {blend weight (float), material A, material B} and
//       the tyre model blends the two materials by the weight.
//     * the material table is reachable through a fixed pointer at VA 0x1659B70 (the exe has no ASLR):
//       world = [0x1659B70], physics = [world+0x24], manager = physics+0x18, material i = manager+0xC+i*0x160, and an
//       id is valid when bit 0 of the byte at manager+0x164+i*0x160 is set. The scanner reads that table and only
//       accepts values that are valid ids (UseValid=1), which throws away about half of all false candidates.
//     * RBR also has a FORCE_MATERIAL debug override. A first version of this scanner used it as a probe; it crashed the
//       game (the physics thread died right after the id was forced), so every write to RBR's memory has been removed.
//       The scanner only READS memory now.
//     * Read-only structure hint: the tyre contact is {float weight in 0..1, material A, material B} (three dwords in a
//       row). PairFilter=1 only accepts addresses that look like that on the first snapshot.
//
//   Passive mode only (Passive=1): sampling only happens while the car is moving and all wheels are loaded (telemetry speed and airborne), so the
//   ForceFX wheel connection has to be up, or set RequireTelemetry=0 in [MemScan]. Don't restart the stage in the
//   middle of a scan: heap addresses move and the candidates are cleared on every stage start. NumLock must be on.
//   Beeps confirm what happened: rising pitch = label set, two tones = snapshot taken, three tones = diff done,
//   one low long tone = the step was cancelled (the car left the ground, slowed down, or you pressed another key).
//
//   Output: ForceFX_memscan.txt (the report, rewritten every ReportSeconds), ForceFX_memscan.log (events).
//   Watchlist: put Watch1..Watch8 in [MemScan] to follow addresses you already know, even across game restarts:
//     Watch1=RichardBurnsRally_SSE.exe+0x7E1234>0x20*4@0x80
//     module+offset (or exe+offset for the main program, or an absolute 0xADDRESS), each ">off" reads a pointer
//     there and adds off, "*4@0x80" shows 4 values at a 0x80 stride. Every change is written to the log with the
//     current label, so you can confirm an address by driving over known surfaces.
// ====================================================================================================================
namespace MemScan {

constexpr int     kLabels = 8;
constexpr uint8_t kNone   = 0xFF;          // "no value": out of range, unread or not set yet
constexpr size_t  kChunk  = 4u << 20;      // read granularity of a full memory pass
constexpr size_t  kChunkBuf = kChunk + 16;  // a little headroom beyond one chunk
// layout of RBR's physics object (see the header comment)
constexpr uintptr_t kPhysOff = 0x24, kMgrOff = 0x18, kValidOff = 0x164, kMatSize = 0x160;

struct Settings {
    int   enable = 1;
    int   width = 4;                       // bytes per value: 1, 2 or 4. A surface id is most likely a 32-bit int
    int   maxValue = 254;                  // only values 0..maxValue are candidates (material ids go up to 255)
    int   maxRegionMB = 64;                // skip single regions bigger than this (textures, audio, mesh data); 0 = no limit
    int   snapshotMB = 192;                // memory budget of the first snapshot (one byte per scanned value)
    int   includeReadOnly = 0, includeMapped = 0;
    int   requireTelemetry = 1;            // 0 = sample even when ForceFX has no telemetry (you must keep the car on the ground yourself)
    float minSpeed = 6.0f;                 // m/s; sampling only happens above this speed
    int   settleMs = 1500;                 // a label has to be active this long before anything is sampled
    int   gapMs = 700;                     // gap between the two reads that prove a value holds still
    float trackHz = 4.0f;
    int   maxCands = 3000000;
    int   missPercent = 30, minSamples = 8;   // a candidate is dropped when more than this share of its samples disagree
    int   reportSec = 10;
    int   keyLabel1 = 0x61;                // VK_NUMPAD1; the next seven labels use the following key codes
    int   keyPause = 0x69, keyReport = 0x6E, keyReset = 0x60;   // Numpad 9, Numpad ., Numpad 0
    int   beep = 1;
    int   useValid = 1;                    // only accept values that are valid material ids (needs the anchor below)
    int   anchorVA = 0x1659B70;            // RichardBurnsRally_SSE.exe: pointer to the physics world
    int   passive = 0;                     // 1 = the old drive-and-label search (Numpad 1..8 snapshots); off by default
    int   tyreScan = 1;                    // find the tyre objects by their vtable and read the surface ids from them
    int   autoTyres = 1;                   // look for the tyres by itself a few seconds after a stage starts
    int   tyreHz = 20;                     // how often the tyre records are read
    int   tyreVtable = 0x74288C;           // RichardBurnsRally_SSE.exe: vtable of the tyre class
    int   recOff = 0xC20, recStride = 0x70, recCount = 3;   // contact records inside a tyre object
    int   recFlag = 0x00, recWeight = 0x34, recMatA = 0x38, recMatB = 0x3C;
    int   keyTyres = 0x6A;                 // Numpad *
    int   logLines = 4000;                 // stop writing id changes to the log after this many lines
    int   pairFilter = 1;                  // first snapshot only keeps {float 0..1, valid id, valid id} dword triples (width 4 only)
    std::string label[kLabels] = {"Tarmac", "Gravel", "Snow", "Ice", "Dirt", "Sand", "Mud", "Other"};
    std::string watch[8];
};

// ---------------------------------------------------------------- memory we must never scan (our own buffers)
struct Range { uintptr_t lo, hi; };
std::mutex         g_exMtx;
std::vector<Range> g_ex;

void ExAdd(const void* p, size_t n) {
    std::lock_guard<std::mutex> lk(g_exMtx);
    g_ex.push_back(Range{(uintptr_t)p, (uintptr_t)p + n});
}
void ExRemove(const void* p) {
    std::lock_guard<std::mutex> lk(g_exMtx);
    for (size_t i = 0; i < g_ex.size(); ++i)
        if (g_ex[i].lo == (uintptr_t)p) { g_ex.erase(g_ex.begin() + (ptrdiff_t)i); break; }
}
bool IsExcluded(uintptr_t a, size_t n) {
    std::lock_guard<std::mutex> lk(g_exMtx);
    for (const Range& e : g_ex) if (a < e.hi && a + n > e.lo) return true;
    return false;
}

// Fixed-capacity buffer on its own VirtualAlloc pages. Because it is registered as excluded, a scan of RBR's memory
// never reads the scanner's own data (which would otherwise change under it and look like a "candidate").
template <class T>
class Buf {
    T* p_ = nullptr; size_t n_ = 0, cap_ = 0;
public:
    Buf() {}
    Buf(const Buf&) = delete;
    Buf& operator=(const Buf&) = delete;
    ~Buf() { Free(); }
    bool Alloc(size_t cap) {
        Free();
        if (!cap) return false;
        void* m = VirtualAlloc(nullptr, cap * sizeof(T), MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        if (!m) return false;
        p_ = static_cast<T*>(m); cap_ = cap; n_ = 0;
        ExAdd(m, cap * sizeof(T));
        return true;
    }
    void Free() {
        if (p_) { ExRemove(p_); VirtualFree(p_, 0, MEM_RELEASE); }
        p_ = nullptr; n_ = cap_ = 0;
    }
    bool   ok() const { return p_ != nullptr; }
    T*     data() { return p_; }
    size_t size() const { return n_; }
    size_t cap() const { return cap_; }
    void   resize(size_t n) { n_ = n < cap_ ? n : cap_; }
    bool   push(const T& v) { if (n_ >= cap_) return false; p_[n_++] = v; return true; }
    T&     operator[](size_t i) { return p_[i]; }
};

// Names of the ids we could verify against the exe's material table (ids 1..65 line up exactly; above that the order
// of the table and the ids drift apart, so those are shown as unknown). Dry/damp/wet is the low two bits (1/2/3).
const char* FamilyName(int id) {
    struct F { int lo, hi; const char* n; };
    static const F t[] = {
        {1, 3, "Gravel_Fine_Deeper"}, {5, 7, "Gravel_Medium_Deeper"}, {9, 11, "Gravel_Fine_Deep"}, {13, 15, "Gravel_Medium_Deep"},
        {17, 19, "Gravel_Fine_Shallow"}, {21, 23, "Gravel_Medium_Shallow"}, {24, 24, "Spret_Bush_Small"}, {25, 25, "Spret_Bush_Medium"},
        {26, 26, "Spret_Bush_Large"}, {27, 27, "Lov_Bush_Small"}, {28, 28, "Lov_Bush_Medium"}, {29, 29, "Lov_Bush_Large"},
        {30, 30, "Tree_Small"}, {31, 31, "Tree_Medium"}, {32, 32, "Tree_Large"}, {33, 35, "Gravel_Coarse_Deeper"},
        {37, 39, "Tarmac_Fine_Clean"}, {41, 43, "Gravel_Coarse_Deep"}, {45, 47, "Tarmac_Fine_Sprinkled"}, {49, 51, "Gravel_Coarse_Shallow"},
        {53, 55, "Tarmac_Fine_Covered"}, {56, 56, "Rock_Small"}, {57, 57, "Rock_Medium"}, {58, 58, "Rock_Large"}, {59, 59, "Trunk_Small"},
        {60, 60, "Trunk_Medium"}, {61, 61, "Trunk_Large"}, {62, 62, "Snowwall"}, {63, 63, "Metal_Pole"}, {65, 65, "Metal_Barrier"}};
    for (const F& f : t) if (id >= f.lo && id <= f.hi) return f.n;
    return nullptr;
}
std::string IdName(int id) {
    const char* n = FamilyName(id);
    if (!n) return id == 0 ? "default" : "(unverified)";
    char b[96];
    if (id <= 55 && (id & 3)) snprintf(b, sizeof(b), "%s_%s", n, (id & 3) == 1 ? "Dry" : (id & 3) == 2 ? "Damp" : "Wet");
    else snprintf(b, sizeof(b), "%s", n);
    return b;
}

// ---------------------------------------------------------------- small helpers
bool Rd(uintptr_t a, void* d, size_t n) {            // safe even on unmapped memory: it just fails
    SIZE_T got = 0;
    return ReadProcessMemory(GetCurrentProcess(), (LPCVOID)a, d, n, &got) && got == n;
}

struct Mod { std::string name; uintptr_t base, size; };
std::vector<Mod> Modules() {
    std::vector<Mod> v;
    HANDLE h = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, GetCurrentProcessId());
    if (h == INVALID_HANDLE_VALUE) return v;
    MODULEENTRY32 e; std::memset(&e, 0, sizeof(e)); e.dwSize = sizeof(e);
    if (Module32First(h, &e)) do {
        char path[MAX_PATH] = {0};
        GetModuleFileNameA(e.hModule, path, MAX_PATH);
        std::string nm(path);
        const size_t sl = nm.find_last_of("\\/");
        Mod m; m.name = sl == std::string::npos ? nm : nm.substr(sl + 1);
        m.base = (uintptr_t)e.modBaseAddr; m.size = (uintptr_t)e.modBaseSize;
        v.push_back(m);
    } while (Module32Next(h, &e));
    CloseHandle(h);
    return v;
}
bool InModule(const std::vector<Mod>& mods, uintptr_t a) {
    for (const Mod& m : mods) if (a >= m.base && a < m.base + m.size) return true;
    return false;
}
std::string WatchTok(const std::vector<Mod>& mods, uintptr_t a) {
    char b[256];
    for (const Mod& m : mods)
        if (a >= m.base && a < m.base + m.size) {
            snprintf(b, sizeof(b), "%s+0x%llX", m.name.c_str(), (unsigned long long)(a - m.base));
            return b;
        }
    snprintf(b, sizeof(b), "0x%llX", (unsigned long long)a);
    return b;
}
std::string Describe(const std::vector<Mod>& mods, uintptr_t a) {
    char b[256];
    for (const Mod& m : mods)
        if (a >= m.base && a < m.base + m.size) {
            snprintf(b, sizeof(b), "%s+0x%llX", m.name.c_str(), (unsigned long long)(a - m.base));
            return b;
        }
    snprintf(b, sizeof(b), "heap 0x%08llX", (unsigned long long)a);
    return b;
}

std::string g_mdir;
Clock::time_point g_t0 = Clock::now();
void MLog(const char* fmt, ...) {
    char b[600];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    FILE* f = fopen((g_mdir + "ForceFX_memscan.log").c_str(), "a");
    if (f) { fprintf(f, "[%8.2f] %s\n", std::chrono::duration<double>(Clock::now() - g_t0).count(), b); fclose(f); }
}

inline int64_t NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}

// ---------------------------------------------------------------- data
struct Cand { uintptr_t addr; uint8_t v[kLabels]; uint8_t hit, miss; };   // v[label] = the value seen on that surface
struct Region { uintptr_t base; size_t size; size_t snapOff; bool image; };

struct LabStat {
    bool   used = false;
    int    passes = 0;
    double sumRough = 0, sumTex = 0, sumSpeed = 0;
    float  maxRough = 0, maxTex = 0;
};

struct Watch {
    std::string text;
    bool valid = false, isAbs = false;
    std::string mod;
    uintptr_t base = 0;                  // absolute address, or offset into the module
    std::vector<uintptr_t> chain;        // each step: read a pointer at the current address, then add the offset
    int count = 1;
    uintptr_t stride = 4;
    uint32_t last[8] = {};
    bool haveLast = false;
};

class Scanner {
public:
    Settings S;
    std::string ini;

    // fed from the ForceFX worker thread
    std::atomic<float>   fSpeed{0}, fAir{0}, fRough{0}, fTex{0};
    std::atomic<int64_t> fLast{0};
    std::atomic<bool>    stage{false};

    void Begin(const std::string& dir, const std::string& iniPath) {
        g_mdir = dir; ini = iniPath;
        ExAdd(this, sizeof(*this));        // the scanner's own state (label index, phase...) must never look like a candidate
        Load();
        if (!S.enable) return;
        MLog("---- memory scanner started: width %d, values 0..%d, report every %d s", S.width, S.maxValue, S.reportSec);
        run_ = true;
        th_ = std::thread(&Scanner::Loop, this);
    }
    void End() {
        run_ = false;
        if (th_.joinable()) th_.join();
        ExRemove(this);
    }
    std::string Status(int k) {
        std::lock_guard<std::mutex> lk(sm_);
        return k == 0 ? s0_ : k == 1 ? s1_ : s2_;
    }

private:
    std::thread th_;
    std::atomic<bool> run_{false};
    std::mutex sm_;
    std::string s0_, s1_, s2_;

    enum Phase { P_IDLE, P_WAIT_DIFF, P_TRACK };
    Phase phase_ = P_IDLE;
    int   label_ = -1, passLabel_ = -1, snapLabel_ = -1;
    bool  paused_ = false, reportReq_ = false, resetReq_ = false;
    bool  keyPrev_[kLabels + 4] = {};
    Clock::time_point sticky_ = Clock::now();      // a message set before this time isn't overwritten by routine status text
    Clock::time_point labelSince_ = Clock::now(), nextTrack_ = Clock::now(), nextReport_ = Clock::now(), nextTry_ = Clock::now(),
                      nextWatch_ = Clock::now();
    int   trackPasses_ = 0;
    bool  overflow_ = false;
    LabStat stat_[kLabels];

    Buf<uint8_t> snap_, chunk_, page_;
    Buf<Cand>    cands_;
    std::vector<Region> regions_;
    size_t snapItems_ = 0, skippedRegions_ = 0, skippedMB_ = 0, overBudget_ = 0;
    std::vector<Watch> watches_;
    // RBR's material table (read through the anchor)
    bool anchorOk_ = false; uintptr_t world_ = 0, phys_ = 0, mgr_ = 0; bool valid_[256] = {}; int validCount_ = 0;
    const std::string& LabName(int l) const { return S.label[l]; }

    // tyre objects found by their vtable, and what they hold
    struct Rec { uint32_t flag = 0; float w = 0; uint32_t a = 0, b = 0; float depth = 0; };
    struct Tyre {
        uintptr_t base = 0; uint32_t car = 0; std::vector<Rec> rec, last; bool haveLast = false; bool dead = false;
        uint32_t samples = 0, changes = 0, contact = 0, blkChanges = 0;
        std::vector<uint8_t> blk;                         // last raw read: a live tyre keeps changing, a stale copy never does
    };
    struct IdStat { uint32_t n = 0, asA = 0, asB = 0; double rough = 0, tex = 0, speed = 0; uint32_t tyreMask = 0; uint32_t byLabel[kLabels] = {}; };
    std::vector<Tyre> tyres_;
    IdStat ids_[256];
    bool tyreReq_ = false;
    uint32_t tyreScans_ = 0, tyreLogged_ = 0;
    Clock::time_point nextTyre_ = Clock::now(), tyreFindAt_ = Clock::now(), nextSurf_ = Clock::now();
    int tyreRetry_ = 0;

    // ------------------------------------------------------------ settings
    void Load() {
        const char* K = "MemScan";
        char b[512];
        GetPrivateProfileStringA(K, "Enable", "", b, sizeof(b), ini.c_str());
        const bool fresh = b[0] == 0;
        auto I = [&](const char* k, int& v, bool hex = false) {
            if (fresh) { char o[32]; snprintf(o, sizeof(o), hex ? "0x%X" : "%d", v); WritePrivateProfileStringA(K, k, o, ini.c_str()); return; }
            GetPrivateProfileStringA(K, k, "", b, sizeof(b), ini.c_str()); if (b[0]) v = (int)strtol(b, nullptr, 0);
        };
        auto F = [&](const char* k, float& v) {
            if (fresh) { char o[32]; snprintf(o, sizeof(o), "%g", v); WritePrivateProfileStringA(K, k, o, ini.c_str()); return; }
            GetPrivateProfileStringA(K, k, "", b, sizeof(b), ini.c_str()); if (b[0]) v = (float)atof(b);
        };
        auto T = [&](const char* k, std::string& v) {
            if (fresh) { WritePrivateProfileStringA(K, k, v.c_str(), ini.c_str()); return; }
            GetPrivateProfileStringA(K, k, "", b, sizeof(b), ini.c_str()); if (b[0]) v = b;
        };
        I("Enable", S.enable); I("Width", S.width); I("MaxValue", S.maxValue); I("MaxRegionMB", S.maxRegionMB);
        I("SnapshotMB", S.snapshotMB); I("IncludeReadOnly", S.includeReadOnly); I("IncludeMapped", S.includeMapped);
        I("RequireTelemetry", S.requireTelemetry); F("MinSpeed", S.minSpeed); I("SettleMs", S.settleMs); I("GapMs", S.gapMs);
        F("TrackHz", S.trackHz); I("MaxCandidates", S.maxCands); I("MissPercent", S.missPercent); I("MinSamples", S.minSamples);
        I("ReportSeconds", S.reportSec); I("KeyLabel1", S.keyLabel1, true); I("KeyPause", S.keyPause, true);
        I("KeyReport", S.keyReport, true); I("KeyReset", S.keyReset, true); I("Beep", S.beep);
        I("UseValid", S.useValid); I("AnchorVA", S.anchorVA, true); I("PairFilter", S.pairFilter);
        I("Passive", S.passive); I("TyreScan", S.tyreScan); I("AutoTyres", S.autoTyres); I("TyreHz", S.tyreHz);
        I("TyreVtable", S.tyreVtable, true); I("RecOff", S.recOff, true); I("RecStride", S.recStride, true); I("RecCount", S.recCount);
        I("RecFlag", S.recFlag, true); I("RecWeight", S.recWeight, true); I("RecMatA", S.recMatA, true); I("RecMatB", S.recMatB, true);
        I("KeyTyres", S.keyTyres, true); I("LogLines", S.logLines);
        for (int i = 0; i < kLabels; ++i) { char k[16]; snprintf(k, sizeof(k), "Label%d", i + 1); T(k, S.label[i]); }
        for (int i = 0; i < 8; ++i) { char k[16]; snprintf(k, sizeof(k), "Watch%d", i + 1); T(k, S.watch[i]); }

        S.width = (S.width == 1 || S.width == 2) ? S.width : 4;
        S.maxValue = std::max(1, std::min(S.maxValue, 254));
        S.maxRegionMB = std::max(0, S.maxRegionMB);
        S.snapshotMB = std::max(16, std::min(S.snapshotMB, 1024));
        S.minSpeed = clampf(S.minSpeed, 0.0f, 60.0f);
        S.settleMs = std::max(200, S.settleMs); S.gapMs = std::max(100, S.gapMs);
        S.trackHz = clampf(S.trackHz, 0.5f, 20.0f);
        S.maxCands = std::max(10000, std::min(S.maxCands, 8000000));
        S.missPercent = std::max(0, std::min(S.missPercent, 90)); S.minSamples = std::max(2, S.minSamples);
        S.reportSec = std::max(2, S.reportSec);
        S.tyreHz = std::max(1, std::min(S.tyreHz, 100)); S.recCount = std::max(1, std::min(S.recCount, 6));
        for (int i = 0; i < 8; ++i) {
            Watch w; w.text = S.watch[i];
            ParseWatch(w);
            if (w.valid) watches_.push_back(w);
        }
    }

    static std::string Trim(const std::string& s) {
        size_t a = s.find_first_not_of(" \t\r\n"), z = s.find_last_not_of(" \t\r\n");
        return a == std::string::npos ? std::string() : s.substr(a, z - a + 1);
    }
    static uintptr_t Num(const std::string& s) { return (uintptr_t)strtoull(Trim(s).c_str(), nullptr, 0); }

    // "module+0xOFF>0xA>0xB*4@0x80", "exe+0xOFF" or "0xABSOLUTE"
    static void ParseWatch(Watch& w) {
        std::string t = Trim(w.text);
        if (t.empty()) return;
        size_t star = t.find('*');
        if (star != std::string::npos) {
            std::string tail = t.substr(star + 1);
            t = Trim(t.substr(0, star));
            size_t at = tail.find('@');
            w.count = std::max(1, std::min((int)Num(at == std::string::npos ? tail : tail.substr(0, at)), 8));
            if (at != std::string::npos) w.stride = std::max<uintptr_t>(1, Num(tail.substr(at + 1)));
        }
        std::vector<std::string> parts;
        size_t p = 0;
        for (;;) {
            size_t q = t.find('>', p);
            parts.push_back(Trim(t.substr(p, q == std::string::npos ? std::string::npos : q - p)));
            if (q == std::string::npos) break;
            p = q + 1;
        }
        const std::string& head = parts[0];
        size_t plus = head.find('+');
        if (plus == std::string::npos) { w.isAbs = true; w.base = Num(head); }
        else { w.mod = Trim(head.substr(0, plus)); w.base = Num(head.substr(plus + 1)); }
        for (size_t i = 1; i < parts.size(); ++i) w.chain.push_back(Num(parts[i]));
        w.valid = w.isAbs ? w.base != 0 : !w.mod.empty();
    }

    // ------------------------------------------------------------ keys, gating, feedback
    static bool FocusOk() {
        HWND w = GetForegroundWindow();
        if (!w) return false;
        DWORD pid = 0; GetWindowThreadProcessId(w, &pid);
        return pid == GetCurrentProcessId();
    }
    static bool Key(int vk) { return vk > 0 && (GetAsyncKeyState(vk) & 0x8000) != 0; }

    void Tune(int a, int b = 0, int c = 0, int ms = 60) {
        if (!S.beep) return;
        std::thread([=] { Beep((DWORD)a, (DWORD)ms); if (b) Beep((DWORD)b, (DWORD)ms); if (c) Beep((DWORD)c, (DWORD)ms); }).detach();
    }

    void SetLabel(int i) {
        label_ = i; labelSince_ = Clock::now(); paused_ = false; stat_[i].used = true;
        Tune(450 + 70 * i, 0, 0, 45);
        MLog("label -> %d (%s)", i + 1, S.label[i].c_str());
    }

    void PollKeys() {
        if (!FocusOk()) { std::memset(keyPrev_, 0, sizeof(keyPrev_)); return; }
        for (int i = 0; i < kLabels; ++i) {
            const bool d = Key(S.keyLabel1 + i);
            if (d && !keyPrev_[i]) SetLabel(i);
            keyPrev_[i] = d;
        }
        const bool p = Key(S.keyPause), r = Key(S.keyReport), z = Key(S.keyReset);
        if (p && !keyPrev_[kLabels]) { paused_ = true; if (!S.passive) label_ = -1; Tune(300, 0, 0, 45); MLog(S.passive ? "paused" : "label cleared"); }
        const bool ty = Key(S.keyTyres);
        if (ty && !keyPrev_[kLabels + 3]) tyreReq_ = true;
        keyPrev_[kLabels + 3] = ty;
        if (r && !keyPrev_[kLabels + 1]) reportReq_ = true;
        if (z && !keyPrev_[kLabels + 2]) resetReq_ = true;
        keyPrev_[kLabels] = p; keyPrev_[kLabels + 1] = r; keyPrev_[kLabels + 2] = z;
    }

    bool Fresh() const { return NowMs() - fLast.load() < 700; }
    // The car has to be rolling on loaded wheels: that is when the surface ids are meaningful and stable.
    bool GateOk() const {
        if (!Fresh()) return !S.requireTelemetry;
        return fSpeed.load() >= S.minSpeed && fAir.load() < 0.3f;
    }
    bool Aborted() {
        PollKeys();
        return !run_ || stage.load() || paused_ || passLabel_ != label_ || resetReq_ || !GateOk();
    }
    bool Wait(int ms) {
        const int64_t until = NowMs() + ms;
        while (NowMs() < until) { if (Aborted()) return false; Sleep(20); }
        return true;
    }
    void SetMsg(const char* fmt, ...) {
        if (Clock::now() < sticky_) return;
        char b[300];
        va_list ap; va_start(ap, fmt);
        vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        std::lock_guard<std::mutex> lk(sm_);
        s1_ = b;
    }
    void Fail(const char* what) {
        sticky_ = Clock::now();
        SetMsg("%s cancelled (left the ground, slowed down, or another key). Keep driving on the surface and it will retry.", what);
        MLog("%s cancelled", what);
        Tune(200, 0, 0, 160);
        nextTry_ = Clock::now() + std::chrono::milliseconds(1000);
    }

    // ------------------------------------------------------------ memory layout
    static bool RegionOk(const MEMORY_BASIC_INFORMATION& m, const Settings& s) {
        if (m.State != MEM_COMMIT) return false;
        if (m.Protect & (PAGE_GUARD | PAGE_NOACCESS | PAGE_NOCACHE | PAGE_WRITECOMBINE)) return false;   // no driver/GPU mappings
        const DWORD p = m.Protect & 0xFF;
        const bool rw = p == PAGE_READWRITE || p == PAGE_WRITECOPY;
        if (!(rw || (p == PAGE_READONLY && s.includeReadOnly))) return false;
        if (m.Type == MEM_MAPPED && !s.includeMapped) return false;
        return true;
    }
    static void AddSplit(std::vector<Region>& out, const std::vector<Range>& ex, uintptr_t lo, size_t size, bool image) {
        const uintptr_t hi = lo + size;
        uintptr_t cur = lo;
        for (const Range& e : ex) {
            if (e.hi <= cur || e.lo >= hi) continue;
            if (e.lo > cur) out.push_back(Region{cur, (size_t)(e.lo - cur), 0, image});
            cur = std::max(cur, e.hi);
            if (cur >= hi) break;
        }
        if (cur < hi) out.push_back(Region{cur, (size_t)(hi - cur), 0, image});
    }
    // All scannable regions: program images (static data, where game singletons live) and private heap, minus our own memory.
    void Enum(std::vector<Region>& img, std::vector<Region>& prv) {
        std::vector<Range> ex;
        { std::lock_guard<std::mutex> lk(g_exMtx); ex = g_ex; }
        std::sort(ex.begin(), ex.end(), [](const Range& a, const Range& b) { return a.lo < b.lo; });
        SYSTEM_INFO si; GetSystemInfo(&si);
        uintptr_t addr = (uintptr_t)si.lpMinimumApplicationAddress;
        const uintptr_t maxA = (uintptr_t)si.lpMaximumApplicationAddress;
        skippedRegions_ = 0; skippedMB_ = 0;
        while (addr < maxA) {
            MEMORY_BASIC_INFORMATION m;
            if (VirtualQuery((LPCVOID)addr, &m, sizeof(m)) != sizeof(m)) break;
            const uintptr_t base = (uintptr_t)m.BaseAddress;
            const size_t size = (size_t)m.RegionSize;
            if (base + size <= addr) break;
            addr = base + size;
            if (!RegionOk(m, S)) continue;
            if (S.maxRegionMB > 0 && size > (size_t)S.maxRegionMB * 1048576u) { ++skippedRegions_; skippedMB_ += size >> 20; continue; }
            AddSplit(m.Type == MEM_IMAGE ? img : prv, ex, base, size, m.Type == MEM_IMAGE);
        }
    }
    void BuildRegions() {
        regions_.clear();
        std::vector<Region> img, prv;
        Enum(img, prv);
        std::sort(prv.begin(), prv.end(), [](const Region& a, const Region& b) { return a.size < b.size; });
        const size_t W = (size_t)S.width, budget = (size_t)S.snapshotMB * 1048576u;
        size_t off = 0;
        overBudget_ = 0;
        auto take = [&](std::vector<Region>& v) {
            for (Region r : v) {
                const size_t items = r.size / W;
                if (!items) continue;
                if (off + items > budget) { ++overBudget_; continue; }
                r.size = items * W; r.snapOff = off; off += items;
                regions_.push_back(r);
            }
        };
        take(img); take(prv);          // program images first, then heap regions from small to large
        snapItems_ = off;
    }

    // ------------------------------------------------------------ passes over the whole memory
    template <int W> static inline uint32_t Ld(const uint8_t* p) {
        if (W == 1) return *p;
        if (W == 2) { uint16_t v; std::memcpy(&v, p, 2); return v; }
        uint32_t v; std::memcpy(&v, p, 4); return v;
    }
    inline bool IdOk(uint32_t v) const { return !(anchorOk_ && S.useValid) || valid_[v & 0xFF]; }
    inline uint8_t Clip(uint32_t v) const { return v <= (uint32_t)S.maxValue && IdOk(v) ? (uint8_t)v : kNone; }

    // mode 0: first read into the snapshot. mode 1: second read, anything that changed is thrown away.
    // mode 2: read on the second surface, keep numbers that held still on both surfaces and differ between them.
    template <int W>
    void Chunk(int mode, const Region& r, size_t done, size_t items, bool ok, uint8_t* sn, int labA, int labB) {
        const uint8_t* c = chunk_.data();
        if (mode == 0) {
            if (!ok) { std::memset(sn, kNone, items); return; }
            if (W == 4 && S.pairFilter) {      // {float weight 0..1, material A, material B}: keep the address of material A
                for (size_t i = 0; i < items; ++i) {
                    uint8_t v = kNone;
                    if (i >= 1 && i + 1 < items) {
                        const uint32_t w = Ld<W>(c + (i - 1) * W), a = Ld<W>(c + i * W), b = Ld<W>(c + (i + 1) * W);
                        if ((w == 0 || (w >= 0x00800000u && w <= 0x3F800000u)) && Clip(a) != kNone && Clip(b) != kNone) v = (uint8_t)a;
                    }
                    sn[i] = v;
                }
                return;
            }
            for (size_t i = 0; i < items; ++i) sn[i] = Clip(Ld<W>(c + i * W));
        } else if (mode == 1) {
            if (!ok) { std::memset(sn, kNone, items); return; }
            for (size_t i = 0; i < items; ++i) if (sn[i] != kNone && Clip(Ld<W>(c + i * W)) != sn[i]) sn[i] = kNone;
        } else if (ok) {
            for (size_t i = 0; i < items; ++i) {
                const uint8_t a = sn[i];
                if (a == kNone) continue;
                const uint8_t b = Clip(Ld<W>(c + i * W));
                if (b == kNone || b == a) continue;
                Cand cd; std::memset(&cd, 0, sizeof(cd));
                cd.addr = r.base + done + i * W;
                for (int l = 0; l < kLabels; ++l) cd.v[l] = kNone;
                cd.v[labA] = a; cd.v[labB] = b; cd.hit = 1;
                if (!cands_.push(cd)) { overflow_ = true; return; }
            }
        }
    }

    bool Pass(int mode, int labA, int labB) {
        for (const Region& r : regions_) {
            size_t done = 0;
            while (done < r.size) {
                if (Aborted()) return false;
                const size_t n = std::min(kChunk, r.size - done);
                const size_t items = n / (size_t)S.width;
                bool ok = !IsExcluded(r.base + done, n) && Rd(r.base + done, chunk_.data(), n);
                uint8_t* sn = snap_.data() + r.snapOff + done / (size_t)S.width;
                switch (S.width) {
                    case 1: Chunk<1>(mode, r, done, items, ok, sn, labA, labB); break;
                    case 2: Chunk<2>(mode, r, done, items, ok, sn, labA, labB); break;
                    default: Chunk<4>(mode, r, done, items, ok, sn, labA, labB); break;
                }
                done += n;
                Sleep(1);                    // let the game breathe between 4 MB chunks
            }
        }
        return true;
    }

    // ------------------------------------------------------------ steps
    void DoSnapshot() {
        passLabel_ = label_;
        SetMsg("Taking the first snapshot on %s... keep the car rolling on it", S.label[label_].c_str());
        if (!chunk_.ok() && !chunk_.Alloc(kChunkBuf)) { SetMsg("Out of memory (scan buffer)"); nextTry_ = Clock::now() + std::chrono::seconds(5); return; }
        if (!page_.ok() && !page_.Alloc(4096)) { SetMsg("Out of memory (page buffer)"); nextTry_ = Clock::now() + std::chrono::seconds(5); return; }
        if (!cands_.ok() && !cands_.Alloc((size_t)S.maxCands)) { SetMsg("Out of memory (candidate list)"); nextTry_ = Clock::now() + std::chrono::seconds(5); return; }
        cands_.resize(0);
        ReadAnchor();
        BuildRegions();
        if (!snap_.Alloc(snapItems_)) { SetMsg("Out of memory (snapshot of %u MB). Lower SnapshotMB.", (unsigned)(snapItems_ >> 20)); nextTry_ = Clock::now() + std::chrono::seconds(5); return; }
        size_t bytes = 0; for (const Region& r : regions_) bytes += r.size;
        MLog("snapshot on %s: %u regions, %u MB scanned (%u big regions skipped = %u MB, %u over budget)", S.label[label_].c_str(),
             (unsigned)regions_.size(), (unsigned)(bytes >> 20), (unsigned)skippedRegions_, (unsigned)skippedMB_, (unsigned)overBudget_);
        if (!Pass(0, 0, 0) || !Wait(S.gapMs) || !Pass(1, 0, 0)) { snap_.Free(); Fail("The snapshot"); return; }
        size_t keep = 0;
        for (size_t i = 0; i < snap_.cap(); ++i) if (snap_[i] != kNone) ++keep;
        snapLabel_ = label_; phase_ = P_WAIT_DIFF;
        MLog("snapshot done: %u steady small values", (unsigned)keep);
        SetMsg("Snapshot taken on %s (%u steady values). Now drive onto a different surface and press its key.", S.label[label_].c_str(), (unsigned)keep);
        Tune(700, 900);
    }

    void DoDiff() {
        passLabel_ = label_;
        const int a = snapLabel_, b = label_;
        SetMsg("Comparing %s with %s...", S.label[a].c_str(), S.label[b].c_str());
        cands_.resize(0); overflow_ = false;
        if (!Pass(2, a, b)) { cands_.resize(0); Fail("The comparison"); return; }
        std::sort(cands_.data(), cands_.data() + cands_.size(), [](const Cand& x, const Cand& y) { return x.addr < y.addr; });
        MLog("compare %s vs %s: %u values differ between the two surfaces%s", S.label[a].c_str(), S.label[b].c_str(),
             (unsigned)cands_.size(), overflow_ ? " (list full, raise MaxCandidates or lower MaxValue)" : "");
        if (!Wait(S.gapMs) || !Refresh(b, true)) { cands_.resize(0); Fail("The comparison"); return; }
        snap_.Free();
        regions_.clear();
        phase_ = P_TRACK; trackPasses_ = 0;
        stat_[a].passes++; stat_[b].passes++;
        MLog("%u candidates held still on both surfaces", (unsigned)cands_.size());
        SetMsg("%u candidates left. Keep driving; press a key whenever the surface changes (revisit surfaces).", (unsigned)cands_.size());
        Tune(700, 900, 1100);
        nextReport_ = Clock::now();
    }

    // Reads every candidate once (a page at a time) and updates it. stabilityOnly: just drop anything that isn't equal to what we saw.
    bool Refresh(int lab, bool stabilityOnly) {
        size_t n = cands_.size(), w = 0, i = 0;
        const size_t W = (size_t)S.width;
        auto finish = [&](bool okRet) {
            while (i < n) cands_[w++] = cands_[i++];
            cands_.resize(w);
            return okRet;
        };
        while (i < n) {
            if ((i & 0x3FFF) == 0 && Aborted()) return finish(false);
            const uintptr_t page = cands_[i].addr & ~(uintptr_t)0xFFF;
            const bool ok = Rd(page, page_.data(), 4096);
            size_t j = i;
            while (j < n && (cands_[j].addr & ~(uintptr_t)0xFFF) == page) {
                Cand c = cands_[j];
                uint8_t b = kNone;
                if (ok) {
                    const uint8_t* p = page_.data() + (c.addr - page);
                    const uint32_t v = W == 1 ? Ld<1>(p) : W == 2 ? Ld<2>(p) : Ld<4>(p);
                    b = Clip(v);
                }
                bool keep = true;
                if (stabilityOnly) {
                    keep = b == c.v[lab];
                } else {
                    if (c.v[lab] == kNone) { if (b == kNone) ++c.miss; else { c.v[lab] = b; ++c.hit; } }
                    else if (b == c.v[lab]) ++c.hit; else ++c.miss;
                    if (c.hit == 255 || c.miss == 255) { c.hit >>= 1; c.miss >>= 1; }
                    const int samples = c.hit + c.miss;
                    if (samples >= S.minSamples && (int)c.miss * 100 > samples * S.missPercent) keep = false;
                }
                if (keep) cands_[w++] = c;
                ++j;
            }
            i = j;
        }
        return finish(true);
    }

    void DoTrack() {
        passLabel_ = label_;
        const size_t before = cands_.size();
        if (!Refresh(label_, false)) return;
        LabStat& st = stat_[label_];
        st.passes++; ++trackPasses_;
        const float rough = fRough.load(), tex = fTex.load(), sp = fSpeed.load();
        st.sumRough += rough; st.sumTex += tex; st.sumSpeed += sp;
        st.maxRough = std::max(st.maxRough, rough); st.maxTex = std::max(st.maxTex, tex);
        if (cands_.size() != before) MLog("%s: %u -> %u candidates", S.label[label_].c_str(), (unsigned)before, (unsigned)cands_.size());
        SetMsg("Tracking on %s: %u candidates after %d samples", S.label[label_].c_str(), (unsigned)cands_.size(), trackPasses_);
        if (cands_.size() == 0) {
            phase_ = P_IDLE;
            MLog("every candidate was eliminated");
            SetMsg("Every candidate was eliminated. Pick a surface key to start again (the value may need Width=2 or 1, or a higher MaxValue).");
            Tune(200, 0, 0, 200);
        }
    }


    // ------------------------------------------------------------ RBR's material table (read-only)
    // world = [anchor], physics = [world+0x24], manager = physics+0x18; an id is valid when bit 0 of its flag byte is set
    bool ReadAnchor() {
        anchorOk_ = false; world_ = phys_ = mgr_ = 0; validCount_ = 0;
        std::memset(valid_, 0, sizeof(valid_));
        if (!S.useValid) return false;
        const uintptr_t exe = (uintptr_t)GetModuleHandleA(nullptr);
        const uintptr_t cell = exe + (uintptr_t)((uintptr_t)(unsigned)S.anchorVA - 0x400000u);
        uintptr_t w = 0, p = 0;
        if (!Rd(cell, &w, sizeof(w)) || w < 0x10000) { MLog("material table: the anchor at 0x%llX holds no world pointer yet", (unsigned long long)cell); return false; }
        if (!Rd(w + kPhysOff, &p, sizeof(p)) || p < 0x10000) { MLog("material table: world 0x%llX has no physics pointer", (unsigned long long)w); return false; }
        const uintptr_t mgr = p + kMgrOff;
        int cnt = 0;
        for (int id = 0; id < 256; ++id) {
            uint8_t fl = 0;
            if (Rd(mgr + kValidOff + (uintptr_t)id * kMatSize, &fl, 1) && (fl & 1)) { valid_[id] = true; ++cnt; }
        }
        world_ = w; phys_ = p; mgr_ = mgr; validCount_ = cnt;
        anchorOk_ = cnt >= 8;
        MLog("material table: world 0x%llX physics 0x%llX manager 0x%llX, %d valid ids%s", (unsigned long long)w, (unsigned long long)p,
             (unsigned long long)mgr, cnt, anchorOk_ ? "" : " (too few: this is probably not the exe the offsets were taken from)");
        return anchorOk_;
    }

    void ResetAll(const char* why, bool full = true) {
        snap_.Free(); cands_.resize(0); regions_.clear();
        phase_ = P_IDLE; label_ = -1; snapLabel_ = -1; paused_ = false; trackPasses_ = 0;
        for (LabStat& s : stat_) s = LabStat();
        tyres_.clear(); tyreRetry_ = 0;
        tyreFindAt_ = Clock::now() + std::chrono::seconds(full ? 1 : 4);
        if (full) { for (IdStat& st : ids_) st = IdStat(); tyreScans_ = 0; tyreLogged_ = 0; }
        MLog("reset (%s)", why);
        SetMsg("Reset (%s).", why);
    }

    // ------------------------------------------------------------ watches
    bool EvalWatch(const Watch& w, uintptr_t& addr) {
        if (w.isAbs) addr = w.base;
        else {
            HMODULE m = GetModuleHandleA(LowerStr(w.mod) == "exe" ? nullptr : w.mod.c_str());
            if (!m) return false;
            addr = (uintptr_t)m + w.base;
        }
        for (uintptr_t off : w.chain) {
            uintptr_t p = 0;
            if (!Rd(addr, &p, sizeof(p)) || p == 0) return false;
            addr = p + off;
        }
        return true;
    }
    void TickWatches() {
        std::string line;
        for (size_t k = 0; k < watches_.size(); ++k) {
            Watch& w = watches_[k];
            uintptr_t a = 0;
            uint32_t now[8] = {};
            bool ok = EvalWatch(w, a);
            for (int i = 0; ok && i < w.count; ++i) {
                uint32_t v = 0;
                if (!Rd(a + (uintptr_t)i * w.stride, &v, (size_t)S.width)) ok = false;
                if (S.width == 1) v &= 0xFF; else if (S.width == 2) v &= 0xFFFF;
                now[i] = v;
            }
            char b[160]; int p = snprintf(b, sizeof(b), "w%u:", (unsigned)(k + 1));
            if (!ok) p += snprintf(b + p, sizeof(b) - p, " n/a");
            else for (int i = 0; i < w.count; ++i) p += snprintf(b + p, sizeof(b) - p, " %u", (unsigned)now[i]);
            line += b; line += "  ";
            if (ok && (!w.haveLast || std::memcmp(now, w.last, sizeof(now)) != 0)) {
                MLog("watch %u at 0x%llX ->%s  (label %s, speed %.1f m/s)", (unsigned)(k + 1), (unsigned long long)a,
                     b + 3, label_ >= 0 ? S.label[label_].c_str() : "none", fSpeed.load());
                std::memcpy(w.last, now, sizeof(now)); w.haveLast = true;
            }
        }
        std::lock_guard<std::mutex> lk(sm_);
        s2_ = line;
    }

    // ------------------------------------------------------------ report
    static uint64_t KeyOf(const Cand& c) { uint64_t k; std::memcpy(&k, c.v, 8); return k; }

    std::string Vals(const Cand& c) const {
        std::string s;
        for (int l = 0; l < kLabels; ++l) {
            if (!stat_[l].used) continue;
            char b[64];
            if (c.v[l] == kNone) snprintf(b, sizeof(b), "%s=- ", LabName(l).c_str());
            else snprintf(b, sizeof(b), "%s=%u ", LabName(l).c_str(), (unsigned)c.v[l]);
            s += b;
        }
        return s;
    }
    static int Distinct(const Cand& c, int& setCount) {
        bool seen[256] = {};
        int d = 0; setCount = 0;
        for (int l = 0; l < kLabels; ++l) if (c.v[l] != kNone) { ++setCount; if (!seen[c.v[l]]) { seen[c.v[l]] = true; ++d; } }
        return d;
    }
    static long Score(const Cand& c) { int sc; return Distinct(c, sc) * 1000L + (long)c.hit - (long)c.miss * 10L; }

    struct Quad { uintptr_t base, stride; int distinct; int minHit; int maxMiss; size_t idx; };
    struct Holder { uintptr_t at, value; size_t quad; bool inMod; };

    static bool HasAt(const std::vector<const Cand*>& e, uintptr_t addr, uint64_t key) {
        size_t lo = 0, hi = e.size();
        while (lo < hi) { size_t m = (lo + hi) / 2; if (e[m]->addr < addr) lo = m + 1; else hi = m; }
        return lo < e.size() && e[lo]->addr == addr && KeyOf(*e[lo]) == key;
    }

    void FindQuads(const std::vector<const Cand*>& e, std::vector<Quad>& out) {
        const uintptr_t maxStride = 0x2000;
        uint64_t pairs = 0;
        for (size_t i = 0; i < e.size() && pairs < 60000000ull; ++i) {
            const uint64_t ki = KeyOf(*e[i]);
            for (size_t j = i + 1; j < e.size(); ++j) {
                const uintptr_t d = e[j]->addr - e[i]->addr;
                if (d > maxStride) break;
                ++pairs;
                if (KeyOf(*e[j]) != ki) continue;
                if (!HasAt(e, e[i]->addr + 2 * d, ki) || !HasAt(e, e[i]->addr + 3 * d, ki)) continue;
                bool dup = false;
                for (Quad& q : out) if (q.base == e[i]->addr) { dup = true; if (d < q.stride) q.stride = d; break; }
                if (dup) continue;
                Quad q; q.base = e[i]->addr; q.stride = d; q.idx = i;
                int sc; q.distinct = Distinct(*e[i], sc);
                q.minHit = 255; q.maxMiss = 0;
                for (int k = 0; k < 4; ++k) {
                    const Cand* c = e[i];
                    const uintptr_t at = e[i]->addr + (uintptr_t)k * d;
                    size_t lo = 0, hi = e.size();
                    while (lo < hi) { size_t m = (lo + hi) / 2; if (e[m]->addr < at) lo = m + 1; else hi = m; }
                    if (lo < e.size()) c = e[lo];
                    q.minHit = std::min<int>(q.minHit, c->hit); q.maxMiss = std::max<int>(q.maxMiss, c->miss);
                }
                out.push_back(q);
                if (out.size() >= 400) return;
            }
        }
    }

    // Looks through all writable memory for pointers that land within 0x4000 bytes before each quad. A pointer held in a
    // program image is static: it survives restarts, so "module+offset, then +delta" is the stable way to the tyre data.
    void FindHolders(const std::vector<Quad>& quads, size_t nQuads, const std::vector<Mod>& mods, std::vector<Holder>& out) {
        std::vector<Region> img, prv;
        Enum(img, prv);
        img.insert(img.end(), prv.begin(), prv.end());
        const uintptr_t span = 0x4000;
        const size_t PW = sizeof(uintptr_t);
        for (const Region& r : img) {
            size_t done = 0;
            while (done < r.size) {
                if (!run_ || out.size() > 6000) return;
                const size_t n = std::min(kChunk, r.size - done);
                if (!IsExcluded(r.base + done, n) && Rd(r.base + done, chunk_.data(), n)) {
                    for (size_t i = 0; i + PW <= n; i += PW) {
                        uintptr_t v; std::memcpy(&v, chunk_.data() + i, PW);
                        if (v < 0x10000) continue;
                        for (size_t q = 0; q < nQuads; ++q) {
                            if (v <= quads[q].base && quads[q].base - v <= span) {
                                const uintptr_t at = r.base + done + i;
                                out.push_back(Holder{at, v, q, InModule(mods, at)});
                                break;
                            }
                        }
                    }
                }
                done += n;
                Sleep(1);
            }
        }
    }

    void WriteReport(bool pointers) {
        // The automatic report keeps being refreshed; the full one (with the pointer search) gets its own file so it isn't overwritten.
        FILE* f = fopen((g_mdir + (pointers ? "ForceFX_memscan_full.txt" : "ForceFX_memscan.txt")).c_str(), "w");
        if (!f) return;
        const std::vector<Mod> mods = Modules();
        fprintf(f, "ForceFX memory scan report   (width %d, values 0..%d)\n", S.width, S.maxValue);
        fprintf(f, "Candidates: %u   (state: %s)\n\n", (unsigned)cands_.size(),
                phase_ == P_TRACK ? "tracking" : phase_ == P_WAIT_DIFF ? "snapshot taken, waiting for a second surface" : "idle");
        if (anchorOk_) {
            fprintf(f, "RBR material table: world 0x%llX, physics 0x%llX, manager 0x%llX (material i at manager+0xC+i*0x160), %d valid ids:\n  ",
                    (unsigned long long)world_, (unsigned long long)phys_, (unsigned long long)mgr_, validCount_);
            for (int id = 0; id < 256; ++id) if (valid_[id]) fprintf(f, "%d ", id);
            fprintf(f, "\n  %s\n\n", S.useValid ? "Only valid ids are accepted as candidates." : "UseValid=0: ids are not checked against this table.");
        } else fprintf(f, "RBR material table: not read (no stage loaded yet, a different exe, or UseValid=0), candidates are not validated.\n\n");
        fprintf(f, "Surfaces used (telemetry averages while sampling, to help tell them apart from the signal side):\n");
        for (int l = 0; l < kLabels; ++l) {
            if (!stat_[l].used) continue;
            const LabStat& s = stat_[l]; const int n = std::max(1, s.passes);
            fprintf(f, "  %d %-8s samples %-4d  roughness avg %.3f max %.3f   texture avg %.3f max %.3f   speed avg %.1f m/s\n",
                    l + 1, S.label[l].c_str(), s.passes, s.sumRough / n, s.maxRough, s.sumTex / n, s.maxTex, s.sumSpeed / n);
        }
        fprintf(f, "\n");

        std::vector<const Cand*> e;               // address order, those that already separate at least two surfaces
        for (size_t i = 0; i < cands_.size(); ++i) {
            int sc; if (Distinct(cands_[i], sc) >= 2) e.push_back(&cands_[i]);
        }
        std::vector<Quad> quads;
        FindQuads(e, quads);
        std::sort(quads.begin(), quads.end(), [&](const Quad& a, const Quad& b) {
            if (a.distinct != b.distinct) return a.distinct > b.distinct;
            if (a.minHit != b.minHit) return a.minHit > b.minHit;
            return a.maxMiss < b.maxMiss;
        });
        fprintf(f, "== Wheel sets: four equal values at a fixed stride (best first) ==\n");
        if (quads.empty()) fprintf(f, "  none yet (visit more surfaces, or the four tyres aren't stored at an even stride)\n");
        for (size_t i = 0; i < quads.size() && i < 40; ++i) {
            const Quad& q = quads[i];
            fprintf(f, "  %s  stride 0x%llX  %s hit>=%d miss<=%d\n", Describe(mods, q.base).c_str(), (unsigned long long)q.stride,
                    Vals(*e[q.idx]).c_str(), q.minHit, q.maxMiss);
        }

        std::vector<const Cand*> best;
        for (const Cand* c : e) best.push_back(c);
        std::sort(best.begin(), best.end(), [](const Cand* a, const Cand* b) { return Score(*a) > Score(*b); });
        fprintf(f, "\n== Best single addresses ==\n");
        for (size_t i = 0; i < best.size() && i < 80; ++i)
            fprintf(f, "  %-34s %s hit %d miss %d\n", Describe(mods, best[i]->addr).c_str(), Vals(*best[i]).c_str(), best[i]->hit, best[i]->miss);

        if (pointers && !quads.empty()) {
            const size_t nq = std::min<size_t>(quads.size(), 6);
            std::vector<Holder> hs;
            SetMsg("Searching for pointers to the best wheel sets...");
            FindHolders(quads, nq, mods, hs);
            fprintf(f, "\n== Pointers into the structures that hold the best wheel sets ==\n");
            fprintf(f, "  Read it as: the value stored at HOLDER points DELTA bytes before the wheel set. A holder inside a program\n"
                       "  image never moves, so  Watch=<holder>>DELTA*4@STRIDE  in [MemScan] follows it from run to run.\n");
            for (size_t q = 0; q < nq; ++q) {
                fprintf(f, "\n  wheel set %s (stride 0x%llX, %s)\n", Describe(mods, quads[q].base).c_str(), (unsigned long long)quads[q].stride, Vals(*e[quads[q].idx]).c_str());
                std::vector<Holder> mine;
                for (const Holder& h : hs) if (h.quad == q) mine.push_back(h);
                std::sort(mine.begin(), mine.end(), [&](const Holder& a, const Holder& b) {
                    if (a.inMod != b.inMod) return a.inMod;
                    return (quads[q].base - a.value) < (quads[q].base - b.value);
                });
                if (mine.empty()) fprintf(f, "    no pointers found\n");
                for (size_t i = 0; i < mine.size() && i < 16; ++i) {
                    const uintptr_t delta = quads[q].base - mine[i].value;
                    fprintf(f, "    %-34s -> +0x%llX    Watch=%s>0x%llX*4@0x%llX%s\n", Describe(mods, mine[i].at).c_str(), (unsigned long long)delta,
                            WatchTok(mods, mine[i].at).c_str(), (unsigned long long)delta, (unsigned long long)quads[q].stride,
                            mine[i].inMod ? "" : "   (heap: moves between runs)");
                }
            }
        }
        fclose(f);
    }


    // ------------------------------------------------------------ tyre objects (the direct way)
    uint32_t vt_ = 0;

    bool ReadTyre(Tyre& t) {
        uint32_t v = 0;
        if (!Rd(t.base, &v, 4) || v != vt_) return false;               // freed or reused: it no longer starts with the tyre vtable
        const size_t mx = (size_t)std::max(std::max(S.recFlag, S.recWeight), std::max(S.recMatA, S.recMatB));
        const size_t need = (size_t)S.recStride * (size_t)(S.recCount - 1) + std::max<size_t>(mx + 4, 0x44);
        uint8_t blk[1024];
        if (need > sizeof(blk) || !Rd(t.base + (uintptr_t)S.recOff, blk, need)) return false;
        if (!t.blk.empty() && t.blk.size() == need && std::memcmp(t.blk.data(), blk, need) != 0) ++t.blkChanges;
        t.blk.assign(blk, blk + need);
        t.rec.resize((size_t)S.recCount);
        for (int k = 0; k < S.recCount; ++k) {
            const uint8_t* r = blk + (size_t)k * (size_t)S.recStride;
            Rec& o = t.rec[(size_t)k];
            std::memcpy(&o.flag, r + S.recFlag, 4); std::memcpy(&o.w, r + S.recWeight, 4);
            std::memcpy(&o.a, r + S.recMatA, 4);    std::memcpy(&o.b, r + S.recMatB, 4);
            std::memcpy(&o.depth, r + 0x40, 4);
        }
        uint32_t car = 0;
        if (Rd(t.base + 0x7F4, &car, 4)) t.car = car;                   // the car object that owns the tyre
        return true;
    }

    // Looks through all writable memory for objects that start with the tyre vtable.
    void FindTyres() {
        tyreReq_ = false;
        tyres_.clear();
        ++tyreScans_;
        if (!chunk_.ok() && !chunk_.Alloc(kChunkBuf)) return;
        const uintptr_t exe = (uintptr_t)GetModuleHandleA(nullptr);
        vt_ = (uint32_t)(exe + (uintptr_t)((uintptr_t)(unsigned)S.tyreVtable - 0x400000u));
        std::vector<Region> img, prv;
        Enum(img, prv);
        img.insert(img.end(), prv.begin(), prv.end());
        std::vector<uintptr_t> hits;
        size_t scanned = 0;
        SetMsg("Looking for the tyre objects (vtable 0x%X)...", (unsigned)vt_);
        for (const Region& r : img) {
            size_t done = 0;
            while (done < r.size && hits.size() < 64) {
                if (!run_ || stage.load() || resetReq_) return;
                const size_t n = std::min(kChunk, r.size - done) & ~(size_t)3;
                if (!n) break;
                if (!IsExcluded(r.base + done, n) && Rd(r.base + done, chunk_.data(), n)) {
                    const uint32_t* p = reinterpret_cast<const uint32_t*>(chunk_.data());
                    for (size_t i = 0; i < n / 4; ++i) if (p[i] == vt_) hits.push_back(r.base + done + i * 4);
                    scanned += n;
                }
                done += n;
                Sleep(0);
            }
        }
        ReadAnchor();                                                  // for the 'valid' column of the report
        MLog("tyre scan %u: %u MB read, %u objects start with vtable 0x%X", tyreScans_, (unsigned)(scanned >> 20), (unsigned)hits.size(), (unsigned)vt_);
        for (uintptr_t a : hits) {
            Tyre t; t.base = a;
            if (!ReadTyre(t)) continue;
            MLog("  tyre object 0x%llX  car 0x%X  rec0 flag %u  A %u B %u  w %.2f", (unsigned long long)a, t.car, t.rec[0].flag, t.rec[0].a, t.rec[0].b, t.rec[0].w);
            tyres_.push_back(t);
        }
        if (tyres_.empty()) SetMsg("No tyre objects yet. They exist once a stage is loaded; it will keep looking.");
        else { SetMsg("Found %u tyre objects. Reading their surface ids...", (unsigned)tyres_.size()); Tune(700, 900); }
    }

    void TickTyres() {
        const float rough = fRough.load(), tex = fTex.load(), speed = fSpeed.load();
        int alive = 0;
        for (size_t ti = 0; ti < tyres_.size(); ++ti) {
            Tyre& t = tyres_[ti];
            if (t.dead) continue;
            if (!ReadTyre(t)) { t.dead = true; continue; }
            ++alive; ++t.samples;
            if (t.blkChanges == 0) { t.last = t.rec; t.haveLast = true; continue; }     // not proven live (could be a stale copy of an old stage)
            bool changed = false;
            for (size_t k = 0; k < t.rec.size(); ++k) {
                const Rec& r = t.rec[k];
                if (r.flag != 1 || r.a > 255 || r.b > 255) continue;
                ++t.contact;
                const uint32_t dom = r.w < 0.5f ? r.a : r.b;           // the material that dominates the blend
                IdStat& st = ids_[dom];
                ++st.n; st.rough += rough; st.tex += tex; st.speed += speed; st.tyreMask |= 1u << (ti & 31);
                if (label_ >= 0) ++st.byLabel[label_];
                ++ids_[r.a].asA; ++ids_[r.b].asB;
                if (t.haveLast && k < t.last.size() && t.last[k].flag == 1 && (t.last[k].a != r.a || t.last[k].b != r.b)) changed = true;
            }
            if (changed) {
                ++t.changes;
                if ((int)tyreLogged_ < S.logLines) {
                    ++tyreLogged_;
                    std::string line;
                    for (const Rec& r : t.rec) { char b[48]; snprintf(b, sizeof(b), " %u/%u(w%.2f)%s", r.a, r.b, r.w, r.flag == 1 ? "" : "-"); line += b; }
                    MLog("tyre %u:%s   label %s  speed %.1f  rough %.2f", (unsigned)ti, line.c_str(), label_ >= 0 ? S.label[label_].c_str() : "-", speed, rough);
                }
            }
            t.last = t.rec; t.haveLast = true;
        }
        if (!alive) {
            tyres_.clear();
            MLog("the tyre objects are gone (stage ended?); will look again");
            tyreFindAt_ = Clock::now() + std::chrono::seconds(3); tyreRetry_ = 0;
        }
    }

    void WriteSurfaceReport() {
        FILE* f = fopen((g_mdir + "ForceFX_surfaces.txt").c_str(), "w");
        if (!f) return;
        const std::vector<Mod> mods = Modules();
        fprintf(f, "ForceFX surface ids seen under the tyres\n");
        fprintf(f, "tyre vtable 0x%X, %u scans, %u tyre objects, records %d x 0x%X from +0x%X (flag +0x%X, weight +0x%X, A +0x%X, B +0x%X)\n\n",
                (unsigned)vt_, tyreScans_, (unsigned)tyres_.size(), S.recCount, S.recStride, S.recOff, S.recFlag, S.recWeight, S.recMatA, S.recMatB);
        fprintf(f, "== Tyre objects ==\n");
        if (tyres_.empty()) fprintf(f, "  none right now (no stage loaded, or the scan hasn't found them yet)\n");
        for (size_t i = 0; i < tyres_.size(); ++i) {
            const Tyre& t = tyres_[i];
            fprintf(f, "  #%u  %s  car 0x%X  %s  samples %u, contact reads %u, id changes %u\n", (unsigned)i, Describe(mods, t.base).c_str(), t.car,
                    t.dead ? "GONE" : t.blkChanges ? "live" : "static copy (never changes, ignored)", t.samples, t.contact, t.changes);
            if (t.haveLast) for (size_t k = 0; k < t.last.size(); ++k)
                fprintf(f, "       rec %u: flag %u  weight %.2f  A %u  B %u\n", (unsigned)k, t.last[k].flag, t.last[k].w, t.last[k].a, t.last[k].b);
        }
        if (tyres_.size() > 1) {
            fprintf(f, "  spacing:");
            for (size_t i = 1; i < tyres_.size(); ++i) fprintf(f, " +0x%llX", (unsigned long long)(tyres_[i].base - tyres_[i - 1].base));
            fprintf(f, "\n");
        }
        uint64_t total = 0; for (const IdStat& s : ids_) total += s.n;
        fprintf(f, "\n== Surface ids seen (dominant material of each contact record; %llu reads in total) ==\n", (unsigned long long)total);
        fprintf(f, "  id  valid  share   reads   roughness  texture  speed(m/s)  tyres  name\n");
        std::vector<int> order;
        for (int id = 0; id < 256; ++id) if (ids_[id].n || ids_[id].asA || ids_[id].asB) order.push_back(id);
        std::sort(order.begin(), order.end(), [&](int a, int b) { return ids_[a].n > ids_[b].n; });
        for (int id : order) {
            const IdStat& s = ids_[id];
            const double n = (double)std::max<uint32_t>(1, s.n);
            fprintf(f, "  %3d  %-5s %5.1f%%  %6u  %9.3f  %7.3f  %9.1f   %04X   %s", id, valid_[id] ? "yes" : (anchorOk_ ? "NO" : "?"),
                    total ? 100.0 * (double)s.n / (double)total : 0.0, s.n, s.rough / n, s.tex / n, s.speed / n, s.tyreMask, IdName(id).c_str());
            bool any = false;
            for (int l = 0; l < kLabels; ++l) if (s.byLabel[l]) { fprintf(f, "%s%s %u", any ? ", " : "   [", S.label[l].c_str(), s.byLabel[l]); any = true; }
            fprintf(f, "%s", any ? "]\n" : "\n");
        }
        fprintf(f, "\n  'reads' counts the dominant material (weight < 0.5 means material A, otherwise B). Ids 1..65 are named from the exe's\n"
                   "  material table (dry/damp/wet from the low two bits); above that the names are unverified, which is what this\n"
                   "  table is for: drive over a surface, see which id appears and how rough it is.\n");
        fclose(f);
    }

    // ------------------------------------------------------------ thread
    void Loop() {
        try { LoopImpl(); }
        catch (...) { MLog("the scanner thread hit an exception and stopped"); }
    }

    std::string WhyNot() {
        if (label_ < 0) return "Press a surface key (Numpad 1-8) while driving on that surface.";
        if (paused_) return "Paused. Press a surface key to continue.";
        const int64_t settle = (int64_t)S.settleMs - std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - labelSince_).count();
        if (!Fresh() && S.requireTelemetry) return "No telemetry yet: connect the wheel in the ForceFX menu first (or set RequireTelemetry=0).";
        if (Fresh() && fSpeed.load() < S.minSpeed) return "Too slow: sampling starts above the minimum speed.";
        if (Fresh() && fAir.load() >= 0.3f) return "Wheels are off the ground; waiting for the landing.";
        if (settle > 0) return "Settling on the new surface...";
        return std::string();
    }

    void LoopImpl() {
        // the scanner's own stack and the plugin's image are never scanned
        typedef BOOL (WINAPI *LimitsFn)(PULONG_PTR, PULONG_PTR);
        LimitsFn fn = (LimitsFn)GetProcAddress(GetModuleHandleA("kernel32.dll"), "GetCurrentThreadStackLimits");
        ULONG_PTR slo = 0, shi = 0;
        if (fn) { fn(&slo, &shi); if (shi > slo) ExAdd((const void*)slo, (size_t)(shi - slo)); }
        {
            HMODULE self = nullptr;
            GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)&pluginDirectory, &self);
            if (self) {
                uintptr_t a = (uintptr_t)self, lo = a, hi = a;
                for (;;) {
                    MEMORY_BASIC_INFORMATION m;
                    if (VirtualQuery((LPCVOID)a, &m, sizeof(m)) != sizeof(m) || (uintptr_t)m.AllocationBase != lo) break;
                    a = (uintptr_t)m.BaseAddress + m.RegionSize; hi = a;
                }
                if (hi > lo) ExAdd((const void*)lo, (size_t)(hi - lo));
            }
        }
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);     // never compete with the game for CPU
        if (!chunk_.Alloc(kChunkBuf) || !page_.Alloc(4096)) { MLog("couldn't allocate the scan buffers"); return; }
        { std::lock_guard<std::mutex> lk(sm_); s0_ = "MEMSCAN ready"; s1_ = S.tyreScan ? "Looking for the tyre objects once a stage is running (or press Numpad *)." : "Idle."; }

        while (run_) {
            Sleep(25);
            const auto now = Clock::now();
            PollKeys();
            if (stage.exchange(false)) ResetAll("a stage started", false);
            if (resetReq_) { resetReq_ = false; ResetAll("Numpad 0"); Tune(250, 250); }
            if (now >= nextWatch_) { nextWatch_ = now + std::chrono::milliseconds(100); TickWatches(); }

            if (S.tyreScan) {
                if (tyreReq_ || (S.autoTyres && tyres_.empty() && (Fresh() || !S.requireTelemetry) && now >= tyreFindAt_)) {
                    FindTyres();
                    if (tyres_.empty()) { ++tyreRetry_; tyreFindAt_ = Clock::now() + std::chrono::seconds(std::min(30, 5 * tyreRetry_)); }
                    else tyreRetry_ = 0;
                }
                if (!tyres_.empty() && now >= nextTyre_) { nextTyre_ = now + std::chrono::milliseconds(1000 / S.tyreHz); TickTyres(); }
                if (!S.passive) {
                    uint32_t seen = 0; for (const IdStat& st : ids_) if (st.n) ++seen;
                    if (!tyres_.empty()) SetMsg("Reading %u tyre objects, %u different surface ids seen so far. Numpad . writes ForceFX_surfaces.txt.", (unsigned)tyres_.size(), seen);
                }
            }
            const std::string why = S.passive ? WhyNot() : std::string("passive off");
            if (!S.passive) {
                // nothing else to do: the tyre scan above is the whole job
            } else if (why.empty() && now >= nextTry_) {
                if (phase_ == P_IDLE) DoSnapshot();
                else if (phase_ == P_WAIT_DIFF) { if (label_ != snapLabel_) DoDiff(); }
                else if (phase_ == P_TRACK && now >= nextTrack_) {
                    nextTrack_ = now + std::chrono::milliseconds((int)(1000.0f / S.trackHz));
                    DoTrack();
                }
            } else if (!why.empty() && phase_ != P_TRACK) {
                SetMsg("%s", why.c_str());
            } else if (!why.empty()) {
                SetMsg("Tracking is waiting: %s", why.c_str());
            }
            if (phase_ == P_WAIT_DIFF && why.empty() && label_ == snapLabel_)
                SetMsg("Snapshot is ready on %s. Drive onto a different surface and press its key.", S.label[snapLabel_].c_str());

            { char b[200];
              if (S.passive) snprintf(b, sizeof(b), "MEMSCAN %s | label %s | %u candidates",
                       phase_ == P_TRACK ? "tracking" : phase_ == P_WAIT_DIFF ? "snapshot ready" : "idle",
                       label_ >= 0 ? S.label[label_].c_str() : "none", (unsigned)cands_.size());
              else snprintf(b, sizeof(b), "MEMSCAN tyres %u | label %s | ids seen %u", (unsigned)tyres_.size(),
                       label_ >= 0 ? S.label[label_].c_str() : "none", (unsigned)std::count_if(ids_, ids_ + 256, [](const IdStat& x) { return x.n > 0; }));
              std::lock_guard<std::mutex> lk(sm_); s0_ = b; }

            if (reportReq_) {
                reportReq_ = false;
                sticky_ = Clock::now();
                if (S.tyreScan) WriteSurfaceReport();
                if (S.passive) { SetMsg("Writing the full report (this includes the pointer search)..."); WriteReport(true); }
                sticky_ = Clock::now();
                SetMsg(S.passive ? "Reports written (ForceFX_surfaces.txt and ForceFX_memscan_full.txt)" : "Report written to ForceFX_surfaces.txt");
                sticky_ = Clock::now() + std::chrono::seconds(6);
                MLog("full report written");
                Tune(900, 700);
                nextReport_ = Clock::now() + std::chrono::seconds(S.reportSec);
            } else if (now >= nextReport_) {
                nextReport_ = now + std::chrono::seconds(S.reportSec);
                if (S.tyreScan && (!tyres_.empty() || tyreScans_)) WriteSurfaceReport();
                if (S.passive && phase_ == P_TRACK) WriteReport(false);
            }
        }
        if (S.tyreScan && tyreScans_) WriteSurfaceReport();
        if (S.passive && phase_ == P_TRACK && cands_.size()) WriteReport(false);
        snap_.Free(); cands_.Free(); chunk_.Free(); page_.Free();
        if (fn && shi > slo) ExRemove((const void*)slo);
    }
};

Scanner* g_scan = nullptr;

void Start(const std::string& dir, const std::string& ini) {
    if (g_scan) return;
    g_scan = new Scanner();
    g_scan->Begin(dir, ini);
}
void Stop() {
    if (!g_scan) return;
    g_scan->End();
    delete g_scan;
    g_scan = nullptr;
}
void Feed(float speed, float airborne, float rough, float texture) {
    if (!g_scan) return;
    g_scan->fSpeed = speed; g_scan->fAir = airborne; g_scan->fRough = rough; g_scan->fTex = texture;
    g_scan->fLast = NowMs();
}
void OnStage() { if (g_scan) g_scan->stage = true; }
std::string Status(int k) { return g_scan ? g_scan->Status(k) : std::string(); }

} // namespace MemScan

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
    std::atomic<float> m_userGain{1.0f};   // live level 0..1, changed by the hotkeys and the menu
    std::atomic<bool>  m_muted{false};
    std::atomic<float> m_dBoostMul{1.0f}, m_dBoostAdd{0.0f};   // current landing boost on RBR's force (shown in the menu)
    std::atomic<float> m_dHeat{1.0f};      // shown in the menu: 1 = no derating
    std::atomic<int>   m_dCar{-1};
    std::atomic<int>   m_cfgRev{0};         // bumped by the menu whenever m_cfg changes
    std::atomic<float> m_dStat{0}, m_dLoadL{1}, m_dLoadR{1}, m_dAir{0};
    std::atomic<bool>  m_stageReset{false};
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
        try { LoopImpl(); }
        catch (...) { Logf("The worker thread hit an exception; effects released"); m_ffActive = false; }
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
        auto lastPkt = t0, lastApply = t0, nextTry = t0, tapLastRx = t0 - std::chrono::seconds(10), nextRescan = t0;

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
            if (!sameCar) { dsp = Dsp(); synth.rpmPeak = 0; }           // a different car, so forget what the last one taught us
            char lab[200];
            if (prof.found) snprintf(lab, sizeof(lab), "-> %s%s%s (match %.2f)", prof.name.empty() ? prof.section.c_str() : prof.name.c_str(),
                                     prof.preset.empty() ? "" : ", ", prof.preset.c_str(), score);
            else snprintf(lab, sizeof(lab), "-> no matching profile in ForceFXCars.ini");
            { std::lock_guard<std::mutex> lk(m_mtx);
              m_carName = id.name.empty() ? "(name unknown)" : id.name;
              m_carPreset = lab; }
            m_dCar = curSlot;
            Logf("Car slot %d, RSFCarID %d, name \"%s\" %s  cyl=%d rpmMax=%.0f weightFeel=%.2f%s", curSlot, id.rsfId,
                 id.name.c_str(), lab, eff.cylinders, eff.rpmMax, eff.weightFeel, sameCar ? "  (profiles reloaded)" : "");
        };

        // One telemetry packet, from either source. ts is when it was sent (or arrived), so packets that pile up
        // in the queue still get the right time step between them.
        auto handlePacket = [&](const uint8_t* data, int n, Clock::time_point ts, int source) {
            m_packets++;
            m_source = source;
            const float pdt = std::max(0.0f, std::chrono::duration<float>(ts - lastPkt).count());
            lastPkt = ts; lastRx = ts;
            if (pdt > 1.0f) carForce = true;                   // telemetry is back after a pause: probably a new stage, maybe a new car
            if (m_stageReset.exchange(false)) {                 // the car is at the start line: learn its static load again
                for (int i = 0; i < 4; ++i) dsp.statRef[i] = Ema();
                dsp.thumpL = dsp.thumpR = 0;
                dsp.airT = 0; dsp.landWin = 0; dsp.totLow = 1.0f;
            }
            ParsePacket(eff, data, n, live, dsp, pdt);
            if (live.carIndex >= 0 && live.carIndex != curSlot) { curSlot = live.carIndex; carForce = true; }
            {
                std::lock_guard<std::mutex> lk(m_mtx);
                m_pktLen = n;
                std::memcpy(m_pkt, data, n);
            }
            if (!dumped) { DumpPacket(data, n); dumped = true; }
        };

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
            }
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
            m_dDive = sig.dive; m_dLock = sig.lock;
            m_dStat = sig.staticLoad; m_dLoadL = sig.loadL; m_dLoadR = sig.loadR; m_dAir = sig.airborne;
            if (!m_test) MemScan::Feed(sig.speed, sig.airborne, sig.rough, sig.texture);   // MEMSCAN

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
            m_dHeat = synth.heat > base.sustainedLimit ? std::sqrt(base.sustainedLimit / synth.heat) : 1.0f;
            DWORD per[S_COUNT];
            for (int i = 0; i < S_COUNT; ++i) {
                hz[i] = std::isfinite(hz[i]) ? clampf(hz[i], kHardMinHz, kHardMaxHz) : kHardMinHz;
                m_mag[i] = (float)mag[i]; m_hz[i] = hz[i];
                per[i] = (DWORD)(1000000.0f / hz[i]);
            }

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
            // effect switches (turn effects off to feel the others on their own)
            {"Effect: Engine",                   "EnableEngine",       &Config::fxEngine,     nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
            {"Effect: Road texture",             "EnableRoad",         &Config::fxRoad,       nullptr,  0.0f,   1.0f, 1.0f, 'o', false},
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
            {"Landing thump (road)",             "ThumpGain",          nullptr, &Config::thumpGain,     0.0f,  2.0f, 0.05f, 'x', false},
            {"Priority ducking",                 "PriorityDuck",       nullptr, &Config::priorityDuck,  0.0f,  1.0f, 0.05f, '%', false},
            // frequencies
            {"Engine min frequency",             "EngineMinHz",        nullptr, &Config::engineMinHz,   kHardMinHz, kHardMaxHz - 5.0f, 2.0f, 'H', false},
            {"Engine max frequency",             "EngineMaxHz",        nullptr, &Config::engineMaxHz,   kHardMinHz, kHardMaxHz, 2.0f, 'H', false},
            {"Road min frequency",               "RoadMinHz",          nullptr, &Config::roadMinHz,     20.0f, kHardMaxHz - 5.0f, 2.0f, 'H', false},
            {"Road max frequency",               "RoadMaxHz",          nullptr, &Config::roadMaxHz,     25.0f, kHardMaxHz, 2.0f, 'H', false},
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
            default:  snprintf(out, n, "x%.2f", v); break;
        }
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
            {"EnableEngine",       "Turns the engine vibration on or off. Handy for feeling what each effect does on its own."},
            {"EnableRoad",         "Turns the road texture and bump vibration on or off."},
            {"EnableBrake",        "Turns the braking judder and front-wheel lock-up shake on or off."},
            {"EnableLandingBoost", "Turns the landing boost on or off. With it off, landings feel exactly as RBR sends them."},
            {"UserGain",           "Master volume for everything ForceFX adds. It only turns things down, never up past your other limits. Numpad + and - change it while you drive."},
            {"TotalMaxMagnitude",  "The most that engine, road and brake can add together. If the wheel feels too busy or too strong, start here."},
            {"EngineMaxMagnitude", "How strong the engine vibration is. It's the quietest layer and steps aside for road and landings."},
            {"RoadMaxMagnitude",   "How strong road texture and bumps are. Higher makes gravel, dirt and kerbs feel rougher."},
            {"BrakeMaxMagnitude",  "How strong the braking judder and front-wheel lock-up shake are."},
            {"ImpactGain",         "How easily a landing or hard hit counts as a big one. Turn it up if small jumps feel too soft."},
            {"LandingAirLevel",    "How light the car has to get before it counts as airborne. Raise it to catch smaller hops and crests."},
            {"LandingThreshold",   "How hard a hit has to be to count when the car stays on the ground. Raise it if braking or weight shifts set it off."},
            {"LandingBoost",       "Extra pull added to RBR's own steering force at the peak of a full landing. At 200%, the wheel gets three times the force RBR was sending."},
            {"LandingMinPush",     "A minimum extra pull on landing, in the direction RBR is already pulling. It helps when RBR's force happens to be small at touchdown."},
            {"ThumpGain",          "Adds a short thump to the road effect on landings and heavy compressions."},
            {"PriorityDuck",       "How far engine and road back off while something more important is playing. At 0% nothing backs off."},
            {"EngineMinHz",        "Pitch of the engine vibration at idle. Lower is deeper and slower."},
            {"EngineMaxHz",        "Pitch of the engine vibration at the rev limit."},
            {"RoadMinHz",          "Pitch of the road vibration at low speed on smooth ground."},
            {"RoadMaxHz",          "Pitch of the road vibration at high speed or on rough ground. Higher feels sharper and finer."},
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
        for (int k = 0; k < 3; ++k) {                                   // MEMSCAN
            const std::string ms = MemScan::Status(k);
            if (!ms.empty()) DrawWrapped(ms.c_str(), 70.0f, y, 90);
        }
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
        MemScan::Start(g_dir, m_iniPath);     // MEMSCAN (temporary, branch "memory")

        if (m_cfg.telemetrySource != 2) {
            const bool ok = PacketTap::Install(m_cfg.minPacket);
            Logf("Telemetry hook: %s (%s)", ok ? "installed" : "not available", PacketTap::Describe().c_str());
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

    ~ForceFXPlugin() {
        MemScan::Stop();                   // MEMSCAN
        Disconnect();
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
        snprintf(buf, sizeof(buf), "now: engine %.0f@%.0fHz  lump %.0f  road L %.0f R %.0f  brake %.0f@%.0fHz",
                 m_mag[S_ENG1].load(), m_hz[S_ENG1].load(), m_mag[S_ENG2].load(),
                 m_mag[S_ROAD].load(), m_mag[S_ROADR].load(), m_mag[S_BRAKE].load(), m_hz[S_BRAKE].load());
        line(buf);
        snprintf(buf, sizeof(buf), "landing boost on RBR's force: x%.2f  +%.0f", m_dBoostMul.load(), m_dBoostAdd.load() * 10000.0f);
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
    void StageStarted(int, const char*, bool) override { m_stageReset = true; m_carRescan = true; MemScan::OnStage(); }   // MEMSCAN
    void HandleResults(float, float, float, const char*) override {}
    void CheckPoint(float, int, const char*) override {}
};

} // namespace

extern "C" __declspec(dllexport) IPlugin* RBR_CreatePlugin(IRBRGame* pGame) {
    try { return new ForceFXPlugin(pGame); }
    catch (...) { return nullptr; }
}
