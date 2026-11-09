// RBR_ForceFX.cpp  (v6: renamed from RBR_TrueforcePlugin, in-game settings menu for gains and frequencies.
//   Earlier: safety limits, live gain, directional granular road, weight/load model, per-car profiles, landing impacts)
//
// Layers extra ForceFX vibration on top of Richard Burns Rally's own force feedback,
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
//   LANDING  no effect of its own: a detected landing / hard hit boosts RBR's own steering force (a hook on its
//            constant-force updates multiplies it and adds a minimum push in the direction RBR already pushes).
//   BRAKE    sine: front-end judder under braking, grows with brake pedal and front load transfer (dive);
//            front wheel lock-up (from the tyre segment counter vs car speed) makes it harsh.
//
// BUILD: 32-bit (RBR is x86), static CRT:
//   cl /std:c++17 /EHsc /LD /MT /O2 RBR_ForceFX.cpp /Fe:RBR_ForceFX.dll /link /DEF:RBR_ForceFX.def
// NGP: udpTelemetry=1, udpTelemetryEndpoints=127.0.0.1:6776,127.0.0.1:6777,127.0.0.1:6778
// LIVE GAIN (while driving): Ctrl + Numpad+ / Numpad- change the overall vibration level in equal-loudness steps (x or / 0.85),
//   Ctrl + Numpad* mutes / unmutes. Only active while RBR is the foreground window. The level can only go DOWN
//   from the configured ceilings (0..100%), never above them. Keys are configurable in the .ini.
// SAFETY: hard compile-time ceilings (kHardTotalMax etc.), frequency floor, slow thermal derate,
//   loopback-only UDP, fade-to-zero on shutdown, exception guard on the worker thread.
// CAR IDENTITY: by name text (rallysimfans.ini slot -> Cars\Cars.ini CarName / RSFCarID), matched loosely against
//   the Name= of each section in ForceFXCars.ini.
// WEIGHT: per-wheel strut load relative to the car's own static load drives road strength (loaded side transmits more),
//   a compression/landing thump, quiet when airborne, and braking judder. ForceFXCars.ini gives each car index its own
//   engine character (Preset=boxer4 ...) and WeightFeel.
// FILES (next to the DLL): ForceFX.ini (shipped defaults; written on first run if missing), ForceFXCars.ini
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
    int   udpPort = 6778;            // 6776 = SimHub, 6777 = RbrSurfaceAudio. Never share a port.
    int   minPacket = 664;
    int   cylinders = 4;             // engine pulses per rev = cylinders / 2

    // per-layer ceilings (DirectInput units 0..10000), a fixed total ceiling, and one live level (userGain)
    int   engineMaxMag = 1200, roadMaxMag = 3000, brakeMaxMag = 2000;
    float impactGain = 1.5f;         // landing sensitivity 0..3 (1 = old default, 0 = off)
    float brakeHzScale = 1.0f;       // brake judder pitch multiplier 0.5..2 (menu)
    float airLevel = 0.45f;          // 0.2..0.8: total strut load (fraction of static) below which the car counts as airborne; raise it to catch smaller jumps
    float landingThresh = 1.0f;      // 0.5..3: how hard a no-airtime hit must be to count as a landing (higher = fewer false triggers)
    int   fxEngine = 1, fxRoad = 1, fxBrake = 1, fxLandBoost = 1;   // 1 = effect on, 0 = off (menu switches, for diagnosing)
    float landingBoost = 2.0f;       // 0..4: extra RBR steering torque at the moment of a full-severity landing (2 = +200%)
    float landingAdd = 0.30f;        // 0..1: minimum extra push (share of full torque) in the direction RBR is already pushing
    float priorityDuck = 0.85f;      // 0..1: how far lower-priority layers fall back while a higher one is active (landing > road > engine)
    int   totalMaxMag = 4000;        // all layers together are scaled down to this (hard ceiling kHardTotalMax)
    float sustainedLimit = 0.10f;    // thermal derate: long-term mean-square level (0.10 ~ 32% RMS-equivalent)

    // engine
    float engineMinHz = 14.0f, engineMaxHz = 100.0f;
    float rpmMax = 0.0f;             // fallback only: used when the telemetry has no rev limit (0 = learn from the data, starts at RpmMaxInit)
    float rpmMaxInit = 7500.0f;
    float engineGrit = 0.5f;         // 0 = clean sine at high revs, 1 = rough, beating, irregular

    // road
    float roadMinHz = 25.0f, roadMaxHz = 100.0f;
    float roughLow = 0.03f, roughHigh = 0.35f, roughGamma = 1.0f;   // tyre roughness -> 0..1 level (tarmac below roughLow)
    float pistonScale = 1.0f;        // RMS damper piston velocity -> 0..1 (absolute hits, kerbs)
    float bumpScale = 0.25f;         // heave acceleration spike -> 0..1
    float roadGrain = 0.9f;          // 0 = smooth buzz, 1 = coarse, irregular grain (amplitude + frequency randomness)
    float roadDirectional = 0.7f;    // 0 = both sides identical, 1 = left/right impacts fully separate
    int   roadSideInvert = 0;        // 1 = swap which side pushes which way (wheel/driver polarity differs)
    float maxSpeed = 40.0f;          // m/s that counts as full speed

    // weight / suspension load (all relative to the car's own static load, so units never matter)
    float loadInfluence = 0.6f;      // 0 = ignore tyre load, 1 = strongly favour the more heavily loaded side
    float thumpGain = 1.4f;          // compression / landing thump strength
    float airborneFade = 1.0f;       // 1 = road and engine go quiet when the wheels are off the ground

    // per-car character (the values below are the globals; a car profile can override them)
    float weightFeel = 1.2f;         // >1 heavier: stronger thump, slower decay, lower road pitch
    float lumpScale = 1.0f;          // idle lump / beat strength
    float unevenness = 0.0f;         // uneven firing (boxer, five-cylinder, V8 cross-plane)
    int   carProfiles = 1;           // use ForceFXCars.ini
    int   offCarIndex = 56;          // NGP Car.Index

    // braking
    float tyreCircumference = 1.95f; // m, until the plugin has learned it while coasting
    float lockThreshold = 0.12f;     // relative wheel speed deficit where lock-up starts

    // NGP packet offsets (float32 unless noted)
    int   offThrottle = 28, offBrake = 32, offSpeed = 60, offAccHeave = 120, offRpm = 136;
    int   offRpmMax = -1;            // float32 rev limit / max rpm in the packet (-1 = not in this packet: fall back to profile / learned)
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

// Keeps every value inside its safe range. Used when the ini is read and after every menu edit.
void ClampConfig(Config& c) {
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

void SyncConfig(Config& c, const std::string& path, bool write, const char* section = "ForceFX") {
    const char* S = section;
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
// The car is identified by its NAME (plain text), the way RSF / RBR store it: rallysimfans.ini [cars] slot selects the
// active slot, Cars\Cars.ini [Car0<slot>] holds CarName and RSFCarID. ForceFXCars.ini then holds any number of
// sections, each with Name= (and optionally Match=a,b  RSFCarID=  Index=), plus the settings for that car.
// Names are compared loosely (case, punctuation, "Grp"/"Group" spelling, missing manufacturer words), see MatchScore.
struct CarProfile {
    bool found = false;
    std::string section, name, preset;
    std::vector<std::string> alias;  // extra names that should also match this profile
    int   rsfId = -1;                // exact RSF car id match (strongest)
    int   slotIndex = -1;            // legacy: NGP Car.Index, only used when no name could be read
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

// Higher = better, -1 = not the same car. Every token of the shorter name must appear in the longer one (so the
// game's "Escort Cosworth GrA" matches "Ford Escort RS Cosworth GrpA"), and distinguishing tokens must agree.
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
                else if (kv.first == "index")    { p.slotIndex = atoi(kv.second.c_str()); identified = true; }
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

    // 1) exact RSFCarID  2) best name match  3) legacy Index, only when no name could be read
    const CarProfile* Match(const std::string& name, int rsfId, int index, float& score) const {
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
            return nullptr;
        }
        if (index >= 0) for (const CarProfile& p : list) if (p.slotIndex == index) { score = 1.0f; return &p; }
        return nullptr;
    }
};

struct CarIdent { std::string name; int rsfId = -1; };

std::string GameDir() {
    char b[MAX_PATH]; DWORD n = GetModuleFileNameA(NULL, b, MAX_PATH);
    std::string d(b, n); size_t k = d.find_last_of("\\/");
    return k == std::string::npos ? std::string() : d.substr(0, k + 1);
}

// Same sources RSF itself uses (and the SimHub RBR reader): rallysimfans.ini [cars] slot -> Cars\Cars.ini [Car0<slot>].
// Vanilla/RBRCIT installs have no rallysimfans.ini: the telemetry slot is used, and the name falls back to the car's folder.
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
    if (id.name.empty()) {                                    // fall back to the model file name: ...\Sunny_GTi-R\Sunny_GTi-R.ini
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
// RBR drives the wheel with its own DirectInput constant-force effect. Instead of adding effects on top of it, a detected
// landing scales that force on its way to the wheel: every SetParameters call RBR makes on a constant-force effect is
// multiplied by g_boostMul, plus a minimum extra push (g_boostAdd, share of full torque) in the direction RBR is already
// pushing. Our own effects are periodic, so they never pass through this path.
std::atomic<float> g_boostMul{1.0f}, g_boostAdd{0.0f};
std::atomic<int>   g_rbrSeen{0}, g_rbrRaw{0}, g_rbrOut{0};      // diagnostics: RBR constant-force updates seen, last in / out
typedef HRESULT (WINAPI *PFN_SetParams)(IDirectInputEffect*, LPCDIEFFECT, DWORD);
enum { VT_FX_SETPARAMETERS = 6 };

struct EffectTap {
    static void** vtbl;
    static void*  orig;
    static HRESULT WINAPI HSetParameters(IDirectInputEffect* self, LPCDIEFFECT peff, DWORD flags) {
        if (peff && (flags & DIEP_TYPESPECIFICPARAMS) && peff->cbTypeSpecificParams == sizeof(DICONSTANTFORCE) &&
            peff->lpvTypeSpecificParams && peff->dwSize >= 48 && peff->dwSize <= 256) {
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
    // vt = vtable of any effect created on RBR's device (RBR's own effects share the class)
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
    clean = EffectTap::Unpatch() && clean;
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
        for (int i = 0; i < S_COUNT; ++i) if (m_fx[i]) { EffectTap::Patch(*reinterpret_cast<void***>(m_fx[i])); break; }   // RBR's effects share this class
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
    float rpmMaxTelem = 0;   // rev limit straight from the telemetry (0 = not available)
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
    float impact = 0;     // landing / hard-hit severity envelope 0..1 (instant attack, decays)
    unsigned evtSeq = 0;  // bumped for every landing / hit / bump event; the synth fires one left-right kick per event
    float evtSev = 0;     // severity of the current event 0..1 (refined upward while a landing is still compressing)
    int   evtBump = 0;    // 1 = the event is an ordinary bump, 0 = a landing / hard hit
    float evtDir = -1;    // first-stroke direction of the current event: -1 = left, +1 = right
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
    // landing / impact detector (works on raw, unsmoothed load so the spike is not averaged away)
    float peakRw = 0, prevPeakRw = 0, totRatio = 1;
    float airT = 0, landWin = 0, landAir = 0, landPeak = 0, landBump = 0, landJump = 0, imRefr = 0, impactEnv = 0;
    int evtBump = 0;
    unsigned evtSeq = 0; float evtSev = 0, evtDir = -1, rawL = 1, rawR = 1, bumpRefr = 0, bumpPrevSig = 0;
    float totLow = 1.0f;   // running minimum of total strut load (relative): a sudden rise above it = something hit the car
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
            d.totRatio = tot / statTot;
            d.peakRw = std::max(std::max(rw[0], rw[1]), std::max(rw[2], rw[3]));
            const float eDecay = std::exp(-dt / 0.14f);               // compression / landing thump per side
            d.thumpL = std::max(clamp01((rl  - 1.7f) / 1.2f), d.thumpL * eDecay);
            d.thumpR = std::max(clamp01((rr2 - 1.7f) / 1.2f), d.thumpR * eDecay);
            s.thumpL = d.thumpL; s.thumpR = d.thumpR;
            d.rawL = rl; d.rawR = rr2;
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

    // ---- landings and hard hits. Three cues, combined with max() so that ANY strong one gives a full-strength hit:
    //      peak   raw peak strut load of any wheel (relative to its static load)
    //      jump   how fast the TOTAL strut load rose from its recent minimum (works with no airtime at all: drops,
    //             crests, bottoming out)
    //      air    time the wheels were unloaded before the hit, and the heave acceleration spike
    //      The severity is continuous (no on/off threshold), so a small hop is a small tap and a big jump is violent.
    {
        d.imRefr = std::max(0.0f, d.imRefr - dt);
        const float rawTot = d.totRatio;
        d.totLow = std::min(rawTot, d.totLow + (rawTot - d.totLow) * alphaFor(dt, 0.12f));
        const float jump = std::max(0.0f, rawTot - d.totLow);
        const bool off = s.speed > 3.0f && rawTot < c.airLevel;

        // Two tiers: a landing after real airtime (air > 0) is scored sensitively so every jump is felt; the no-airtime
        // "hit" path (air == 0) keeps the strict scoring, because weight transfer from braking while cornering also
        // moves strut loads and must not read as a landing.
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
        auto fire = [&](float sev, int isBump) { ++d.evtSeq; d.evtSev = sev; d.evtBump = isBump; };
        d.bumpRefr = std::max(0.0f, d.bumpRefr - dt);

        if (off) {
            d.airT += dt;
        } else {
            if (d.airT > 0.04f && d.landWin <= 0.0f) {
                d.landWin = 0.10f; d.landAir = d.airT; d.landPeak = 0; d.landBump = 0; d.landJump = 0;
                fire(severity(0.0f, d.landAir, 0.0f, 0.0f), 0);
            }
            d.airT = 0;
        }
        if (d.landWin > 0.0f) {                                   // refine the estimate while the suspension compresses
            d.landPeak = std::max(d.landPeak, d.peakRw);
            d.landBump = std::max(d.landBump, clamp01(d.bumpEnv));
            d.landJump = std::max(d.landJump, jump);
            const float sev = severity(d.landPeak, d.landAir, d.landBump, d.landJump);
            d.impactEnv = std::max(d.impactEnv, sev);
            d.evtSev = std::max(d.evtSev, sev);
            d.landWin -= dt;
            if (d.landWin <= 0.0f) {
                d.landWin = 0.0f; d.imRefr = 0.15f;
                Logf("landing: severity %.2f  (air %.2fs, peak load %.2f, load jump %.2f, heave %.2f)", d.impactEnv, d.landAir, d.landPeak, d.landJump, d.landBump);
            }
        } else if (!off && d.imRefr <= 0.0f && jump > 0.35f * c.landingThresh && d.peakRw > 1.0f + 0.5f * c.landingThresh) {   // hard hit / bottoming out with no airtime
            const float sev = severity(d.peakRw, 0.0f, clamp01(d.bumpEnv), jump);
            if (sev > 0.12f) {
                d.impactEnv = std::max(d.impactEnv, sev);
                d.imRefr = 0.15f; d.bumpRefr = 0.10f;
                fire(sev, 0);
                Logf("hit: severity %.2f  (peak load %.2f, load jump %.2f, heave %.2f)", sev, d.peakRw, jump, clamp01(d.bumpEnv));
            }
        }
        d.prevPeakRw = d.peakRw;
        d.impactEnv *= std::exp(-dt / (0.30f * clampf(c.weightFeel, 0.5f, 2.0f)));
        if (d.impactEnv < 0.005f) d.impactEnv = 0.0f;
        s.impact = clamp01(d.impactEnv * c.impactGain);   // impactGain > 1 lets mid-size hits reach full strength
        s.evtSeq = d.evtSeq; s.evtSev = clamp01(d.evtSev * c.impactGain); s.evtDir = d.evtDir * (c.roadSideInvert ? -1.0f : 1.0f); s.evtBump = d.evtBump;
    }

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
    float jAge = 5.0f, evAmp = 0.0f;                       // age and severity of the current landing event
    unsigned seenEvt = 0;
    float boostMul = 1.0f, boostAdd = 0.0f;                // landing boost applied to RBR's own force (read by the worker)
    float lastDerate = 1.0f;
    float roadJit[2] = {1.0f, 1.0f}, ampJit[2] = {1.0f, 1.0f}, engJit = 1.0f, engHzJit = 1.0f, unevPhase = 0;
    double nextJit = 0;

    void Run(const Config& c, const Signals& s, double t, float dt, float userGain, int mag[S_COUNT], float hz[S_COUNT]) {
        thrSm += (s.throttle - thrSm) * alphaFor(dt, 0.12f);

        // ---------------- ENGINE: gain AND frequency both follow the engine
        rpmPeak = std::max(rpmPeak, s.rpm);
        // rev limit: telemetry first, then the car profile / ini, then learned from the highest rpm seen
        const float rmax = s.rpmMaxTelem > 0 ? s.rpmMaxTelem : (c.rpmMax > 0 ? c.rpmMax : std::max(rpmPeak, c.rpmMaxInit));
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
        hz[S_BRAKE] *= c.brakeHzScale;
        float t4 = c.brakeMaxMag * Ab * gB * clampf(0.65f + 0.35f * s.loadF, 0.5f, 1.3f);   // more load on the fronts = more judder

        // ---------------- LANDING: no effect of its own. A detected landing boosts RBR's own steering force (see EffectTap):
        // a fast attack, a short hold, then a decay, scaled by the landing's severity and by the live level.
        if (s.evtSeq != seenEvt) { seenEvt = s.evtSeq; jAge = 0.0f; evAmp = s.evtSev; }
        if (jAge < 0.08f) evAmp = std::max(evAmp, s.evtSev);                         // a landing is scored over its first ~100 ms
        jAge = std::min(jAge + dt, 5.0f);
        {
            const float ts = std::sqrt(wf);                                           // heavier car = longer boost
            const float hold = 0.06f * ts, dec = 0.30f * ts;
            const float env = jAge < hold ? 1.0f : (float)std::exp(-(jAge - hold) / dec);
            const float lev = (c.fxLandBoost && jAge < 1.5f) ? clamp01(evAmp) * env * clampf(userGain, 0.0f, 1.0f) : 0.0f;
            boostMul = 1.0f + c.landingBoost * lev;
            boostAdd = c.landingAdd * lev;
        }

        // attack / release: road is deliberately fast (sharpness), engine is rounder
        auto follow = [&](int i, float target, float up, float down) {
            sm[i] += (target - sm[i]) * (target > sm[i] ? up : down);
        };
        if (!c.fxEngine) { t1 = 0.0f; t2 = 0.0f; }
        follow(S_ENG1, t1, 0.35f, 0.15f);
        follow(S_ENG2, t2, 0.25f, 0.12f);
        const float roadDown = clampf(0.40f / wf, 0.15f, 0.70f);                     // heavier = the thump rings out longer
        follow(S_ROAD,  c.fxRoad ? tRoadA : 0.0f, 0.55f, roadDown);
        follow(S_ROADR, c.fxRoad ? tRoadB : 0.0f, 0.55f, roadDown);
        follow(S_BRAKE, c.fxBrake ? t4 : 0.0f, 0.50f, 0.20f);
        // Priority: road > engine. A lower layer falls back while a higher one is active.
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

        // Slow thermal derate on the level actually delivered to the wheel (live gain included): turning the
        // live gain down therefore relieves the derate instead of stacking with it.
        const float ug = clampf(userGain, 0.0f, 1.0f);
        const float lvl = std::min(sum, (float)kHardTotalMax) / 10000.0f * ug;
        heat += (lvl * lvl - heat) * alphaFor(dt, 20.0f);
        const float derate = heat > c.sustainedLimit ? std::sqrt(c.sustainedLimit / heat) : 1.0f;

        const float g = ug * derate;
        lastDerate = derate;
        for (int i = 0; i < S_COUNT; ++i) {
            float m = sm[i] * duck[i] * scale * g;
            if (!std::isfinite(m)) { m = 0.0f; sm[i] = 0.0f; }          // never let a NaN latch into the filters
            mag[i] = std::max(0, std::min((int)m, kHardTotalMax));
        }
    }
};

// ----------------------------------------------------------------- plugin
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
    std::atomic<float> m_mag[S_COUNT], m_hz[S_COUNT];
    std::atomic<float> m_dRpm{0}, m_dSpeed{0}, m_dThr{0}, m_dBrk{0}, m_dTex{0}, m_dRough{0}, m_dBump{0},
                       m_dDive{0}, m_dLock{0};
    std::atomic<bool> m_ffActive{false};
    std::atomic<float> m_userGain{1.0f};   // live level 0..1, changed by hotkeys
    std::atomic<bool>  m_muted{false};
    std::atomic<float> m_dBoostMul{1.0f}, m_dBoostAdd{0.0f};   // current landing boost on RBR's force (menu diagnostics)
    std::atomic<float> m_dHeat{1.0f};      // shown in the menu: 1 = no derate
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
        WritePrivateProfileStringA("ForceFX", "Wheel", dev.name, m_iniPath.c_str());

        if (!GameDeviceTap::I().Install(m_pDI, dev.guid)) {
            SetStatus("Could not hook DirectInput (see log)");
            return;
        }
        GameDeviceTap::I().Arm(dev.guid);
        m_connectedIdx = m_sel;
        m_packets = 0;
        m_pktLen = 0;
        m_run = true;
        m_thread = std::thread(&ForceFXPlugin::Loop, this);
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

    // Synthetic signals for the TEST mode: 16 s loop through every layer, no telemetry needed.
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

    // Called from the worker thread every cycle. Edge-triggered, with a shortSet auto-repeat for held keys.
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
            if (cf.gainBeep) BeepAsync(300 + (int)(g * 900.0f));          // pitch tells you the level
            Logf("Live gain -> %.0f%%", g * 100.0f);
        }
        if (bm && !mu) {
            m_muted = !m_muted;
            if (cf.gainBeep) BeepAsync(m_muted ? 200 : 700);
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
        Config base;                           // private copy of m_cfg: the menu edits m_cfg on the UI thread
        { std::lock_guard<std::mutex> lk(m_mtx); base = m_cfg; }
        int seenRev = m_cfgRev.load();
        WSADATA wsa;
        bool wsaOk = WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
        SOCKET sock = INVALID_SOCKET;
        if (wsaOk) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock != INVALID_SOCKET) {
                sockaddr_in a{};
                a.sin_family = AF_INET;
                a.sin_port = htons((u_short)base.udpPort);
                a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   // telemetry is local only
                if (bind(sock, (sockaddr*)&a, sizeof(a)) == SOCKET_ERROR) {
                    Logf("UDP bind to port %d failed (WSA %d). Another app on that port?", base.udpPort, WSAGetLastError());
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
        Config eff = base;                     // base + the active car profile
        CarDb carDb;
        CarProfile prof;
        int curSlot = -2;                      // telemetry Car.Index
        std::string curKey;                    // name|rsfId|slot of the car the profile was applied for
        Clock::time_point nextCarPoll{};       // identity is polled about once a second, with or without telemetry
        bool carForce = true;                  // look at the car right away (stage start, telemetry back, slot changed)
        bool dumped = false, prevTest = false;
        double testT0 = 0;

        const auto t0 = Clock::now();
        auto lastRx = t0 - std::chrono::seconds(10);
        auto lastPkt = t0, lastApply = t0, nextTry = t0;

        // Which car is this? The name text comes from RSF / RBR (see ResolveCarIdentity). Polled continuously instead of
        // only on a few events, because the car is changed in the menus, where no telemetry flows, and RSF may write its
        // files a moment after the stage loads. A changed identity, or an edited ForceFXCars.ini, reloads the profile.
        auto pollCar = [&]() {
            const bool reloaded = base.carProfiles ? carDb.Reload() : false;
            const CarIdent id = ResolveCarIdentity(curSlot);
            if (id.name.empty() && id.rsfId < 0 && (curSlot < 0 || !curKey.empty())) return;   // nothing readable right now: keep the current car
            char kb[320]; snprintf(kb, sizeof(kb), "%s|%d|%d", id.name.c_str(), id.rsfId, curSlot);
            if (curKey == kb && !reloaded) return;
            const bool sameCar = (curKey == kb);
            curKey = kb;
            prof = CarProfile();
            float score = 0;
            if (base.carProfiles)
                if (const CarProfile* m = carDb.Match(id.name, id.rsfId, curSlot, score)) prof = *m;
            ApplyProfile(base, prof, eff);
            if (!sameCar) { dsp = Dsp(); synth.rpmPeak = 0; }           // a different car: forget what the last one taught us
            char lab[200];
            if (prof.found) snprintf(lab, sizeof(lab), "-> %s%s%s (match %.2f)", prof.name.empty() ? prof.section.c_str() : prof.name.c_str(),
                                     prof.preset.empty() ? "" : ", ", prof.preset.c_str(), score);
            else snprintf(lab, sizeof(lab), "-> no profile in ForceFXCars.ini");
            { std::lock_guard<std::mutex> lk(m_mtx);
              m_carName = id.name.empty() ? "(name unknown)" : id.name;
              m_carPreset = lab; }
            m_dCar = curSlot;
            Logf("Car slot %d, RSFCarID %d, name \"%s\" %s  cyl=%d rpmMax=%.0f weightFeel=%.2f%s", curSlot, id.rsfId,
                 id.name.c_str(), lab, eff.cylinders, eff.rpmMax, eff.weightFeel, sameCar ? "  (profiles reloaded)" : "");
        };

        while (m_run) {
            const auto now = Clock::now();
            bool got = false;

            if (haveSock) {
                int n = recvfrom(sock, (char*)buf.data(), (int)buf.size(), 0, nullptr, nullptr);
                if (n >= base.minPacket) {
                    m_packets++;
                    got = true;
                    float pdt = std::chrono::duration<float>(now - lastPkt).count();
                    lastPkt = now; lastRx = now;
                    if (pdt > 1.0f) carForce = true;                   // telemetry is back after a pause: a new stage, maybe a new car
                    if (m_stageReset.exchange(false)) {                 // car is standing at the start line: re-learn static load
                        for (int i = 0; i < 4; ++i) dsp.statRef[i] = Ema();
                        dsp.thumpL = dsp.thumpR = 0;
                        dsp.impactEnv = 0; dsp.airT = 0; dsp.landWin = 0; dsp.totLow = 1.0f;
                    }
                    ParsePacket(eff, buf.data(), n, live, dsp, pdt);
                    if (live.carIndex >= 0 && live.carIndex != curSlot) { curSlot = live.carIndex; carForce = true; }
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

            if (m_cfgRev.load() != seenRev) {                  // the menu changed a gain or frequency: pick it up now
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
                sig = Signals();                   // no telemetry (stage over / paused) -> fade to zero
            }

            m_dRpm = sig.rpm; m_dSpeed = sig.speed; m_dThr = sig.throttle; m_dBrk = sig.brake;
            m_dTex = sig.texture; m_dRough = sig.rough; m_dBump = sig.bump;
            m_dDive = sig.dive; m_dLock = sig.lock;
            m_dStat = sig.staticLoad; m_dLoadL = sig.loadL; m_dLoadR = sig.loadR; m_dAir = sig.airborne;

            PollHotkeys(now, base);
            const bool testNow = m_test.load();
            if (testNow && !prevTest) testT0 = t;
            prevTest = testNow;
            // TEST mode fades in over one second so it never starts with a jolt.
            const float testRamp = testNow ? clamp01((float)(t - testT0)) : 1.0f;
            const float live_gain = m_muted.load() ? 0.0f : m_userGain.load() * testRamp;

            int mag[S_COUNT]; float hz[S_COUNT];
            synth.Run(eff, sig, t, dt, live_gain, mag, hz);
            // Landing boost: applied to RBR's own constant-force updates by EffectTap (not in test mode: no RBR force there)
            g_boostMul = m_test ? 1.0f : synth.boostMul; g_boostAdd = m_test ? 0.0f : synth.boostAdd;
            m_dBoostMul = synth.boostMul; m_dBoostAdd = synth.boostAdd;
            m_dHeat = synth.heat > base.sustainedLimit ? std::sqrt(base.sustainedLimit / synth.heat) : 1.0f;
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

    // ------------------------------------------------------------ in-game settings menu (UI thread only)
    // One row per adjustable value. Edits go to m_cfg under the mutex, the worker thread picks them up through
    // m_cfgRev (so the wheel changes immediately, also in TEST mode), and the changed keys are written to ForceFX.ini.
    // Every edit passes through ClampConfig(), so the menu can never leave the safe ranges (hard ceilings included).
    struct MenuRow {
        const char* label; const char* key;
        int Config::* ip; float Config::* fp;     // exactly one of the two is set
        float lo, hi, step;
        char fmt;                                 // 'g' magnitude, '%' percent, 'H' Hz, 'x' multiplier
        bool live;                                // the live level: mirrored into m_userGain
    };
    std::vector<MenuRow> m_rows;
    int  m_cursor = 0, m_top = 0, m_view = 0;     // m_view 0 = settings, 1 = raw packet diagnostics
    bool m_resetArmed = false;

    void BuildRows() {
        const float M = (float)kHardTotalMax;
        m_rows = {
            // effect switches (diagnosing: turn individual effects off to feel the rest on their own)
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

    // dir = +1 / -1. Clamps, applies live, and writes every row the edit changed (a frequency can push its partner).
    void AdjustRow(int ri, int dir) {
        std::vector<float> before;
        for (const MenuRow& r : m_rows) before.push_back(RowValue(r));
        const MenuRow& r = m_rows[ri];
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            float v = clampf(RowValue(r) + (float)dir * r.step, r.lo, r.hi);
            v = std::round(v * 1000.0f) / 1000.0f;                                  // no 0.35000002 drift from repeated steps
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
        Logf("Menu: gains and frequencies reset to the built-in defaults");
    }

    // RBR's WriteText is printf-like: a literal % must be doubled or it eats the next argument
    void WriteSafe(float x, float y, const char* t) {
        std::string e;
        for (; *t; ++t) { e += *t; if (*t == '%') e += '%'; }
        game_->WriteText(x, y, e.c_str());
    }

    // One-line description of what each setting does, shown under the list for the highlighted row.
    static const char* TipFor(const char* key) {
        struct T { const char* k; const char* t; };
        static const T tips[] = {
            {"EnableEngine",       "Switch the engine vibration on or off. Left/Right or Select toggles it. For diagnosing."},
            {"EnableRoad",         "Switch road texture and surface vibration on or off. For diagnosing."},
            {"EnableBrake",        "Switch braking judder and lock-up shake on or off. For diagnosing."},
            {"EnableLandingBoost", "Switch the landing boost on or off. For diagnosing."},
            {"UserGain",           "Overall level for every added effect. Only lowers output; also on Numpad +/- while driving."},
            {"TotalMaxMagnitude",  "Cap on engine, road and brake combined. Lower it if the wheel feels too busy or too strong."},
            {"EngineMaxMagnitude", "Strength of the engine vibration. It has the lowest priority and gives way to road and landings."},
            {"RoadMaxMagnitude",   "Strength of surface texture and bumps. Higher feels rougher on gravel, dirt and kerbs."},
            {"BrakeMaxMagnitude",  "Strength of the braking judder and front-wheel lock-up shake."},
            {"ImpactGain",         "How hard landings and big hits are scored. Higher = even small jumps get the full boost."},
            {"LandingAirLevel",    "How light the car must get to count as airborne. Higher catches smaller hops and crests."},
            {"LandingThreshold",   "How hard a hit without airtime must be to count. Raise it if braking or weight shifts trigger it."},
            {"LandingBoost",       "On a landing, RBR's own steering force is multiplied by this much extra. 200% = three times its force at the peak."},
            {"LandingMinPush",     "Minimum extra torque on a landing, in the direction RBR is already pushing. Helps when RBR's own force is small."},
            {"ThumpGain",          "Extra thump added to the road layer on landings and heavy compressions."},
            {"PriorityDuck",       "How much engine and road fade back while a higher-priority effect plays. 0 = no ducking."},
            {"EngineMinHz",        "Engine vibration pitch at idle. Lower is deeper and slower."},
            {"EngineMaxHz",        "Engine vibration pitch at the rev limit. The rev limit comes from telemetry."},
            {"RoadMinHz",          "Road vibration pitch at low speed on smooth ground."},
            {"RoadMaxHz",          "Road vibration pitch at high speed or on rough ground. Higher feels sharper and finer."},
            {"BrakeHzScale",       "Pitch of the braking judder. Below 1 = slower and coarser, above 1 = faster and finer."},
        };
        for (const T& t : tips) if (std::strcmp(t.k, key) == 0) return t.t;
        return "";
    }

    // Splits text over several screen lines at spaces.
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
        line("Up/Down = scroll the raw packet, Select / Left / Right = back to the settings");
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
        { // one-time migration from the pre-rename names (RBR_TrueforcePlugin): keep the user's tuning and car profiles
          const std::string oldIni = g_dir + "TrueforceRumble.ini", oldCars = g_dir + "TrueforceCars.ini";
          if (!exists && fileExists(oldIni)) {
              SyncConfig(m_cfg, oldIni, false, "Trueforce");
              Logf("Migrated settings from TrueforceRumble.ini");
          }
          if (!fileExists(CarsIniPath()) && fileExists(oldCars) && CopyFileA(oldCars.c_str(), CarsIniPath().c_str(), TRUE))
              Logf("Copied car profiles from TrueforceCars.ini");
        }
        if (!exists) SyncConfig(m_cfg, m_iniPath, true);     // write a template with every key
        SyncConfig(m_cfg, m_iniPath, false);
        m_userGain = m_cfg.userGain;

        BuildRows();
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

    ~ForceFXPlugin() {
        Disconnect();
        GameDeviceTap::I().Remove();       // restore the vtable before our DLL goes away
        if (m_pDI) { m_pDI->Release(); m_pDI = nullptr; }
    }

    const char* GetName() override { return "ForceFX"; }

    void DrawFrontEndPage() override {
        if (!game_) return;
        if (m_view == 1) { DrawRawPage(); return; }

        const int nVal = (int)m_rows.size();
        const int total = nVal + 4;                 // device, test, values..., diagnostics, reset
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
        snprintf(buf, sizeof(buf), "UDP packets: %lld (port %d)   live gain %.0f%%%s   derate x%.2f",
                 (long long)m_packets.load(), m_cfg.udpPort, m_userGain.load() * 100.0f, m_muted ? " MUTED" : "", m_dHeat.load());
        line(buf);
        { std::string nm, pr; { std::lock_guard<std::mutex> lk(m_mtx); nm = m_carName; pr = m_carPreset; }
          snprintf(buf, sizeof(buf), "car slot %d: %s %s", m_dCar.load(), nm.empty() ? "waiting for telemetry" : nm.c_str(), pr.c_str());
          line(buf); }
        snprintf(buf, sizeof(buf), "now: engine %.0f@%.0fHz  lump %.0f  road L %.0f R %.0f  brake %.0f@%.0fHz",
                 m_mag[S_ENG1].load(), m_hz[S_ENG1].load(), m_mag[S_ENG2].load(),
                 m_mag[S_ROAD].load(), m_mag[S_ROADR].load(), m_mag[S_BRAKE].load(), m_hz[S_BRAKE].load());
        line(buf);
        snprintf(buf, sizeof(buf), "landing boost: x%.2f  +%.0f   RBR force updates seen %d  last in %d out %d",
                 m_dBoostMul.load(), m_dBoostAdd.load() * 10000.0f, g_rbrSeen.load(), g_rbrRaw.load(), g_rbrOut.load());
        line(buf);
        y += 6.0f;

        for (int i = m_top; i < std::min(total, m_top + kVisible); ++i) {
            const bool sel = (i == m_cursor);
            char label[120], value[160];
            if (i == 0) {
                snprintf(label, sizeof(label), "Wheel (Select = connect)");
                if (m_devices.empty()) snprintf(value, sizeof(value), "[no force feedback devices]");
                else snprintf(value, sizeof(value), "%s%s", m_sel == m_connectedIdx ? "[on] " : "", m_devices[m_sel].name);
            } else if (i == 1) {
                snprintf(label, sizeof(label), "TEST vibration (Select)");
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
        snprintf(buf, sizeof(buf), "Up/Down choose (%d/%d)  Left/Right change  (live, saved to ini)",
                 m_cursor + 1, total);
        line(buf);
        {   // tooltip for the highlighted row
            const char* tip = "";
            if (m_cursor == 0) tip = "Choose your wheel with Left/Right, then Select to connect. The name is saved for next time.";
            else if (m_cursor == 1) tip = "Built-in demo of engine, gravel, dirt and braking vibrations. Landings boost RBR's own force, so they are not part of the demo.";
            else if (m_cursor == iView) tip = "Live telemetry values and raw packet bytes. Useful for troubleshooting with no effects.";
            else if (m_cursor == iReset) tip = "Restores the gains and frequencies on this page to the built-in defaults. Select twice to confirm.";
            else tip = TipFor(m_rows[m_cursor - 2].key);
            game_->SetMenuColor(IRBRGame::MENU_HEADING);
            y += 4.0f;
            DrawWrapped(tip, 70.0f, y, 88);
            game_->SetMenuColor(IRBRGame::MENU_TEXT);
        }
        line("Car profiles scale the engine and road gains on top of these values.");
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
                if (m_connectedIdx == m_sel && m_run) m_test = !m_test;   // already connected: toggle synthetic TEST vibration
                else Connect();
            }
        } else if (m_cursor == 1) {
            if (bSelect || dir) {
                if (m_connectedIdx >= 0 && m_run) m_test = !m_test;
                else SetStatus("Connect to the wheel first (Select on the Wheel row)");
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
        } else if (bSelect && (m_rows[m_cursor - 2].fmt == 'o' || m_rows[m_cursor - 2].fmt == 'k')) {                      // Select flips an ON/OFF switch
            AdjustRow(m_cursor - 2, m_rows[m_cursor - 2].fmt == 'k' ? (RowValue(m_rows[m_cursor - 2]) >= 2.5f ? -3 : 1) : (RowValue(m_rows[m_cursor - 2]) >= 0.5f ? -1 : 1));
        }
    }

    void TickFrontEndPage(float) override {}
    void StageStarted(int, const char*, bool) override { m_stageReset = true; m_carRescan = true; }
    void HandleResults(float, float, float, const char*) override {}
    void CheckPoint(float, int, const char*) override {}
};

} // namespace

extern "C" __declspec(dllexport) IPlugin* RBR_CreatePlugin(IRBRGame* pGame) {
    // Proof-of-life: if this file never appears next to the DLL, RBR did not call the plugin.
    try {
        g_dir = pluginDirectory();
        FILE* f = fopen((g_dir + "ForceFX_loaded.txt").c_str(), "w");
        if (f) { fprintf(f, "RBR_CreatePlugin was called by RBR.\n"); fclose(f); }
    } catch (...) {}
    try { return new ForceFXPlugin(pGame); }
    catch (...) { return nullptr; }
}
