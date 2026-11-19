// Camera module: head movement for the cockpit camera, and optional rain, snow and mud on the windscreen.
// Built in when FORCEFX_CAMERA is defined (see camera_module.hpp). Without the define none of this code, MinHook or Direct3D
// is pulled in.
//
// HOW IT FITS TOGETHER
//   Telemetry   ForceFX hands every NGP packet to feed(); the packet is turned into filtered inputs (accelerations, yaw,
//               steering, brake, gear, jumps and crashes).
//   Head model  Once per rendered frame a set of springs turns those inputs into a head pose.
//   Camera      Method 4 (default) rewrites the Direct3D matrices at draw time, the way openRBRVR does, so the game's own
//               camera and wipers keep running untouched and head roll is possible. Method 2 forces a camera change every
//               frame (used when openRBRVR is loaded or the Direct3D hooks do not come up). Method 3 is experimental.
//   Windscreen  Rain, snow and mud drawn on the glass from an EndScene hook; see the [Windscreen] section of the ini.
//
// HEAD MODEL (everything is tunable in ForceFXCamera.ini)
//   * Torso and neck: each translation axis is a slow, well damped torso spring plus a faster, slightly bouncy neck spring,
//     both driven by the car's acceleration. The head ends up opposite to the acceleration, lags behind it and overshoots a little.
//   * Brake boost: stronger forward lean while the brake is pressed.
//   * Bump channel: high-passed heave acceleration drives a stiff, fast vertical spring (road texture, kerbs, jumps).
//   * Head pitch: nods forward under braking and back under acceleration.
//   * Look into the corner: yaw rate and steering turn the view toward the apex.
//   * Gear-shift kick, crash and landing shakes, airborne lift.
//   * Speed fade (no movement when parked), soft limits (tanh), fixed 240 Hz sub-stepping so the springs are stable at any frame rate.
//   * Test mode: scripted inputs for checking every axis from the cockpit.
//   * Hot keys (RBR window focused, Ctrl held): F9 reloads ForceFXCamera.ini, F10 camera on/off, F8 windscreen on/off, F7 syncs the
//     emulated wipers.
//
// This is written for the RallySimFans RichardBurnsRally_SSE.exe: it hooks fixed addresses in that executable, so it stays
// off for any other executable unless AnyExe=1 is set in the ini.
//
// TELEMETRY LAYOUT (pack 1, NGP\sdk\rbr.telemetry.data.TelemetryData.h):
//   Control @24: steering 24, throttle 28, brake 32, handbrake 36, clutch 40, gear (int) 44
//   Car @56: speed 60, velocities (6 floats) @88 (surge, sway, heave, roll, pitch, yaw -> yaw rate @108),
//            accelerations (6 floats) @112: surge 112, sway 116, heave 120, roll 124, pitch 128, yaw 132
#pragma once
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <windows.h>
#include <d3d9.h>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <algorithm>
#include "../thirdparty/minhook/include/MinHook.h"

namespace ffx {
namespace cam {
namespace {

// ================================================================================================ config
struct Cfg {
    // [General]
    int   apply = 1, log = 0, anyExe = 0, method = 4, autoFallback = 1, maxApplyHz = 0, diag = 0, diagDeep = 0, diagAfter = 300, test = 0, toggleKey = 121, reloadKey = 120, needCtrl = 1;
    float intensity = 1.0f;
    // [Input]  (NGP accelerations look like g: peaks ~0.5-1)
    float smooth = 0.35f, accelClamp = 2.5f, deadZone = 0.06f, minSpeed = 5.f, fullSpeed = 25.f, yawFloor = 0.2f, steerFloor = 0.1f;
    int   invSurge = 1, invSway = 1, invHeave = 1, invYaw = 1, invSteer = 1;     // 1 or -1: flips an NGP signal
    // [Translation]  metres of static head offset per unit of acceleration
    float kSurge = 0.012f, kSway = 0.014f, kHeave = 0.005f, brakeBoost = 1.15f;
    float torsoShare = 0.70f, torsoFreq = 1.2f, torsoDamp = 0.85f, neckFreq = 3.0f, neckDamp = 0.60f;
    // [Bumps]
    float bumpGain = 0.0015f, bumpFreq = 6.0f, bumpDamp = 0.45f, bumpTau = 0.25f, bumpThreshold = 0.15f;
    // [Rotation]  degrees
    float pitchPerG = 0.6f, pitchFreq = 2.8f, pitchDamp = 0.60f;
    float rollPerG = 1.0f, rollFreq = 2.4f, rollDamp = 0.60f, horizonLock = 0.0f, carRollScale = 1.0f; int invCarRoll = 1;   // roll: Method 4 only
    float lookYaw = 1.8f, lookSteerMix = 0.40f, lookFreq = 1.0f, lookDamp = 0.90f;
    // [Shift]
    float shiftKick = 0.05f;
    // [Impact]  crashes, jumps and landings
    float impactScale = 1.0f, crashThr = 3.0f, crashIgnore = 15.f, crashKick = 0.25f, crashPitchKick = 25.f, crashRollKick = 25.f, crashCooldown = 0.4f, maxKick = 0.5f;
    float airFrac = 0.08f, landFrac = 0.30f, airMs = 120.f, airRef = 0.9f, landRefG = 2.5f, landMinS = 0.25f, landMaxS = 2.0f;
    float landKick = 0.20f, landPitchKick = 10.f, airLift = 0.004f;
    float shakePos = 0.010f, shakeAng = 1.2f, shakeFreq1 = 11.f, shakeFreq2 = 17.f, shakeDecay = 0.35f, shakeGain = 0.30f;
    // [Limits]
    float maxPos = 0.025f, maxAng = 3.0f, maxRoll = 3.0f;
    // [Windscreen]  proof-of-concept overlay
    int   wsRainSource = 1, wsProbe = 0, wsWetFromGame = 0, wsDrawAt = 1, wsDepthTest = 1, wsEnable = 0, wsMode = 3, wsDrops = 200, wsMud = 16, wsSignSway = 1, wsSignSurge = 1, wsToggleKey = 119,
          wsWiperSource = 1, wsWiperAxis = 0, wsWiperAxisSign = 1, wsHideGameWipers = 0, wsWiperDraw = 0, wsMicro = 3500, wsPools = 1, wsHideGameFx = 1, wsRefraction = 1, wsSnowMax = 140, wsMudSmear = 1, wsWipers = 0, wsWiperAuto = 1, wsWiperDebug = 0, wsWiperSyncKey = 118, wsFly = 1, wsWiperBow = 1, wsWiperBlur = 1;
    float wsRainLight = 0.45f, wsRainHeavy = 1.0f, wsMudDry = 0.15f, wsMudDamp = 0.7f, wsMudWet = 1.0f, wsSpeedRef = 120.f, wsRainLevel = 0.7f, wsRoughRef = 0.25f, wsRain = 20.f, wsMudRate = 6.f, wsTrail = 5.f, wsGlassDist = 0.9f, wsDropSize = 1.0f, wsMudSize = 1.0f, wsOpacity = 1.0f, wsWind = 1.0f, wsGravity = 1.0f,
          wsWiperLenM = 0.5f, wsMicroRate = 200.f, wsVeil = 0.8f, wsPoolMinPx = 8.f, wsRefractK = 2.0f, wsRefractMinPx = 4.0f, wsSnowLight = 0.5f, wsSnowHeavy = 1.0f, wsSnowLevel = 0.f, wsSnowRate = 18.f, wsSnowMelt = 14.f, wsSnowSize = 1.0f, wsMudSnow = 0.6f,
          wsWiperPeriod = 1.4f, wsWiperPause = 0.f,
          wsDirt = 1.0f, wsGlow = 0.10f, wsFlyTime = 0.28f, wsMudR = 0.52f, wsMudG = 0.40f, wsMudB = 0.28f, wsWiperThick = 1.0f, wsFilmDecay = 0.15f, wsSpray = 1.0f,
          wsW1X = 0.30f, wsW1Y = 1.0f, wsW1Len = 0.70f, wsW1A0 = 5.f, wsW1A1 = 100.f, wsW2X = 0.60f, wsW2Y = 1.0f, wsW2Len = 0.f, wsW2A0 = 5.f, wsW2A1 = 100.f;
    // [Axes]  camera frame (from your log: forward = -Z, up = +Y, right = +X)
    int   axRight = 0, axUp = 1, axFwd = 2;
    int   sgRight = 1, sgUp = 1, sgFwd = -1;
    // [Render]  Method 4 (openRBRVR-style matrix patch): signs in D3D view space (x right, y up, z forward)
    int   fixedFunction = 0, rSgRight = 1, rSgUp = 1, rSgFwd = 1, rSgPitch = 1, rSgYaw = 1, rSgRoll = 1;
} g_cfg;

std::atomic<bool> g_enabled{true};        // head movement live switch (ForceFX menu, Ctrl+F10)
std::atomic<bool> g_ready{false};         // the hooks are installed and packets are being processed
std::atomic<bool> g_wsOn{false};         // WindscreenFX live switch (ForceFX menu, Ctrl+F8). wsEnable only says whether its hooks were installed at startup
std::string g_dir;
FILE* g_log = nullptr;                                 // ForceFXCamera.log, only when Log=1
std::function<void(const std::string&)> g_hostLog;     // ForceFX.log

// Logf: detail for tuning and bug reports, written to ForceFXCamera.log only when Log=1.
// Note: what a user needs to know when something does not work (hooks, failures); also goes to ForceFX.log.
void Logf(const char* fmt, ...) {
    if (!g_log) return;
    va_list a; va_start(a, fmt); vfprintf(g_log, fmt, a); va_end(a); fputc('\n', g_log); fflush(g_log);
}
void Note(const char* fmt, ...) {
    char b[512];
    va_list a; va_start(a, fmt); vsnprintf(b, sizeof(b), fmt, a); va_end(a);
    if (g_log) { fputs(b, g_log); fputc('\n', g_log); fflush(g_log); }
    if (g_hostLog) g_hostLog(std::string("Camera: ") + b);
}

float iniF(const std::string& path, const char* sec, const char* key, float def) {
    char b[64]; GetPrivateProfileStringA(sec, key, "", b, sizeof(b), path.c_str());
    char* semi = strchr(b, ';'); if (semi) *semi = 0;                // allow trailing ; comments
    return b[0] ? (float)atof(b) : def;
}

struct Opt { const char* sec; const char* key; float* f; int* i; float lo, hi; };
std::vector<Opt> optTable(Cfg& c) {
    return {
        {"General", "Apply", nullptr, &c.apply, 0, 1},           {"General", "Log", nullptr, &c.log, 0, 1},
        {"General", "AnyExe", nullptr, &c.anyExe, 0, 1}, {"General", "Method", nullptr, &c.method, 2, 4}, {"General", "AutoFallback", nullptr, &c.autoFallback, 0, 1}, {"General", "MaxApplyHz", nullptr, &c.maxApplyHz, 0, 240},
        {"General", "Diag", nullptr, &c.diag, 0, 1}, {"General", "DiagDeep", nullptr, &c.diagDeep, 0, 1}, {"General", "DiagAfter", nullptr, &c.diagAfter, 1, 100000},
        {"General", "Test", nullptr, &c.test, 0, 1},             {"General", "Intensity", &c.intensity, nullptr, 0, 2},
        {"General", "ToggleKey", nullptr, &c.toggleKey, 0, 254}, {"General", "ReloadKey", nullptr, &c.reloadKey, 0, 254},
        {"General", "NeedCtrl", nullptr, &c.needCtrl, 0, 1},

        {"Input", "Smooth", &c.smooth, nullptr, 0, 0.95f},       {"Input", "AccelClamp", &c.accelClamp, nullptr, 0.5f, 5000}, {"Input", "DeadZone", &c.deadZone, nullptr, 0, 1},
        {"Input", "MinSpeed", &c.minSpeed, nullptr, 0, 200},     {"Input", "FullSpeed", &c.fullSpeed, nullptr, 1, 400},
        {"Input", "YawFloor", &c.yawFloor, nullptr, 0.001f, 1000}, {"Input", "SteerFloor", &c.steerFloor, nullptr, 0.001f, 1000},
        {"Input", "InvertSurge", nullptr, &c.invSurge, -1, 1},   {"Input", "InvertSway", nullptr, &c.invSway, -1, 1},
        {"Input", "InvertHeave", nullptr, &c.invHeave, -1, 1},   {"Input", "InvertYaw", nullptr, &c.invYaw, -1, 1},
        {"Input", "InvertSteer", nullptr, &c.invSteer, -1, 1},

        {"Translation", "Surge", &c.kSurge, nullptr, 0, 0.2f},   {"Translation", "Sway", &c.kSway, nullptr, 0, 0.2f},
        {"Translation", "Heave", &c.kHeave, nullptr, 0, 0.2f},   {"Translation", "BrakeBoost", &c.brakeBoost, nullptr, 0.5f, 3},
        {"Translation", "TorsoShare", &c.torsoShare, nullptr, 0, 1},
        {"Translation", "TorsoFreq", &c.torsoFreq, nullptr, 0.3f, 8}, {"Translation", "TorsoDamp", &c.torsoDamp, nullptr, 0.2f, 2},
        {"Translation", "NeckFreq", &c.neckFreq, nullptr, 0.5f, 12}, {"Translation", "NeckDamp", &c.neckDamp, nullptr, 0.1f, 2},

        {"Bumps", "Gain", &c.bumpGain, nullptr, 0, 0.1f},        {"Bumps", "Freq", &c.bumpFreq, nullptr, 1, 20},
        {"Bumps", "Damp", &c.bumpDamp, nullptr, 0.1f, 2},        {"Bumps", "HighPassTau", &c.bumpTau, nullptr, 0.05f, 2}, {"Bumps", "Threshold", &c.bumpThreshold, nullptr, 0, 3},

        {"Rotation", "PitchPerG", &c.pitchPerG, nullptr, 0, 20}, {"Rotation", "PitchFreq", &c.pitchFreq, nullptr, 0.3f, 10},
        {"Rotation", "PitchDamp", &c.pitchDamp, nullptr, 0.2f, 2}, {"Rotation", "LookYaw", &c.lookYaw, nullptr, 0, 30},
        {"Rotation", "LookSteerMix", &c.lookSteerMix, nullptr, 0, 1}, {"Rotation", "LookFreq", &c.lookFreq, nullptr, 0.2f, 6},
        {"Rotation", "LookDamp", &c.lookDamp, nullptr, 0.2f, 2},
        {"Rotation", "RollPerG", &c.rollPerG, nullptr, 0, 10}, {"Rotation", "RollFreq", &c.rollFreq, nullptr, 0.3f, 10},
        {"Rotation", "RollDamp", &c.rollDamp, nullptr, 0.2f, 2}, {"Rotation", "HorizonLock", &c.horizonLock, nullptr, 0, 1},
        {"Rotation", "CarRollScale", &c.carRollScale, nullptr, 0, 100}, {"Rotation", "InvertCarRoll", nullptr, &c.invCarRoll, -1, 1},

        {"Shift", "Kick", &c.shiftKick, nullptr, -2, 2},

        {"Impact", "Scale", &c.impactScale, nullptr, 0, 3}, {"Impact", "CrashThreshold", &c.crashThr, nullptr, 0.5f, 50},
        {"Impact", "CrashIgnoreAbove", &c.crashIgnore, nullptr, 2, 500}, {"Impact", "CrashKick", &c.crashKick, nullptr, 0, 2},
        {"Impact", "CrashPitchKick", &c.crashPitchKick, nullptr, 0, 200}, {"Impact", "CrashRollKick", &c.crashRollKick, nullptr, 0, 200},
        {"Impact", "CrashCooldown", &c.crashCooldown, nullptr, 0.1f, 5}, {"Impact", "MaxKick", &c.maxKick, nullptr, 0.05f, 2},
        {"Impact", "AirLoadFraction", &c.airFrac, nullptr, 0.01f, 0.5f}, {"Impact", "LandLoadFraction", &c.landFrac, nullptr, 0.05f, 0.9f},
        {"Impact", "AirborneMs", &c.airMs, nullptr, 30, 1000}, {"Impact", "AirTimeRef", &c.airRef, nullptr, 0.2f, 3},
        {"Impact", "LandRefG", &c.landRefG, nullptr, 0.5f, 20}, {"Impact", "LandMinStrength", &c.landMinS, nullptr, 0, 2},
        {"Impact", "LandMaxStrength", &c.landMaxS, nullptr, 0.5f, 5}, {"Impact", "LandKick", &c.landKick, nullptr, 0, 2},
        {"Impact", "LandPitchKick", &c.landPitchKick, nullptr, 0, 200}, {"Impact", "AirLift", &c.airLift, nullptr, 0, 0.05f},
        {"Impact", "ShakePos", &c.shakePos, nullptr, 0, 0.05f}, {"Impact", "ShakeAngleDeg", &c.shakeAng, nullptr, 0, 10},
        {"Impact", "ShakeFreq1", &c.shakeFreq1, nullptr, 2, 40}, {"Impact", "ShakeFreq2", &c.shakeFreq2, nullptr, 2, 40},
        {"Impact", "ShakeDecay", &c.shakeDecay, nullptr, 0.05f, 3}, {"Impact", "ShakeGain", &c.shakeGain, nullptr, 0, 2},

        {"Limits", "MaxPos", &c.maxPos, nullptr, 0.005f, 0.15f}, {"Limits", "MaxAngleDeg", &c.maxAng, nullptr, 0.5f, 25}, {"Limits", "MaxRollDeg", &c.maxRoll, nullptr, 0.5f, 15},

        {"Windscreen", "Mode", nullptr, &c.wsMode, 1, 3},
        {"Windscreen", "WetFromGame", nullptr, &c.wsWetFromGame, 0, 1},
        {"Windscreen", "DrawAt", nullptr, &c.wsDrawAt, 0, 2},    {"Windscreen", "DepthTest", nullptr, &c.wsDepthTest, 0, 1},
        {"Windscreen", "GlassDist", &c.wsGlassDist, nullptr, 0.2f, 5},
        {"Windscreen", "RainSource", nullptr, &c.wsRainSource, 0, 1},
        {"Windscreen", "RainLight", &c.wsRainLight, nullptr, 0, 1}, {"Windscreen", "RainHeavy", &c.wsRainHeavy, nullptr, 0, 1},
        {"Windscreen", "MudDry", &c.wsMudDry, nullptr, 0, 3}, {"Windscreen", "MudDamp", &c.wsMudDamp, nullptr, 0, 3}, {"Windscreen", "MudWet", &c.wsMudWet, nullptr, 0, 3},
        {"Windscreen", "RainLevel", &c.wsRainLevel, nullptr, 0, 1}, {"Windscreen", "SpeedRef", &c.wsSpeedRef, nullptr, 20, 300},
        {"Windscreen", "RoughRef", &c.wsRoughRef, nullptr, 0.01f, 5}, {"Windscreen", "Probe", nullptr, &c.wsProbe, 0, 1},
        {"Windscreen", "Rain", &c.wsRain, nullptr, 0, 100},      {"Windscreen", "MudRate", &c.wsMudRate, nullptr, 0, 60},
        {"Windscreen", "Trail", &c.wsTrail, nullptr, 0.5f, 30},
        {"Windscreen", "Drops", nullptr, &c.wsDrops, 0, 400},    {"Windscreen", "DropSize", &c.wsDropSize, nullptr, 0.2f, 4},
        {"Windscreen", "Mud", nullptr, &c.wsMud, 0, 120},         {"Windscreen", "MudSize", &c.wsMudSize, nullptr, 0.2f, 4},
        {"Windscreen", "Opacity", &c.wsOpacity, nullptr, 0, 1},  {"Windscreen", "Wind", &c.wsWind, nullptr, 0, 3},
        {"Windscreen", "Gravity", &c.wsGravity, nullptr, 0, 3},
        {"Windscreen", "SignSway", nullptr, &c.wsSignSway, -1, 1}, {"Windscreen", "SignSurge", nullptr, &c.wsSignSurge, -1, 1},
        {"Windscreen", "ToggleKey", nullptr, &c.wsToggleKey, 0, 254},
        {"Windscreen", "HideGameFx", nullptr, &c.wsHideGameFx, 0, 2},
        {"Windscreen", "Micro", nullptr, &c.wsMicro, 0, 8000}, {"Windscreen", "MicroRate", &c.wsMicroRate, nullptr, 0, 1000}, {"Windscreen", "Veil", &c.wsVeil, nullptr, 0, 1},
        {"Windscreen", "Pools", nullptr, &c.wsPools, 0, 1}, {"Windscreen", "PoolMinPx", &c.wsPoolMinPx, nullptr, 3, 80},
        {"Windscreen", "Refraction", nullptr, &c.wsRefraction, 0, 1}, {"Windscreen", "RefractK", &c.wsRefractK, nullptr, 0.2f, 6}, {"Windscreen", "RefractMinPx", &c.wsRefractMinPx, nullptr, 1, 40},
        {"Windscreen", "SnowLight", &c.wsSnowLight, nullptr, 0, 1}, {"Windscreen", "SnowHeavy", &c.wsSnowHeavy, nullptr, 0, 1}, {"Windscreen", "SnowLevel", &c.wsSnowLevel, nullptr, 0, 1},
        {"Windscreen", "SnowRate", &c.wsSnowRate, nullptr, 0, 100}, {"Windscreen", "SnowMelt", &c.wsSnowMelt, nullptr, 1, 300}, {"Windscreen", "SnowSize", &c.wsSnowSize, nullptr, 0.2f, 4},
        {"Windscreen", "SnowMax", nullptr, &c.wsSnowMax, 0, 400}, {"Windscreen", "MudSnow", &c.wsMudSnow, nullptr, 0, 3}, {"Windscreen", "MudSmear", nullptr, &c.wsMudSmear, 0, 1},
        {"Windscreen", "Fly", nullptr, &c.wsFly, 0, 1}, {"Windscreen", "FlyTime", &c.wsFlyTime, nullptr, 0.08f, 1.5f}, {"Windscreen", "Dirt", &c.wsDirt, nullptr, 0, 2}, {"Windscreen", "Glow", &c.wsGlow, nullptr, 0, 0.6f},
        {"Windscreen", "FilmDecay", &c.wsFilmDecay, nullptr, 0, 2}, {"Windscreen", "Spray", &c.wsSpray, nullptr, 0, 4},
        {"Windscreen", "MudR", &c.wsMudR, nullptr, 0, 1}, {"Windscreen", "MudG", &c.wsMudG, nullptr, 0, 1}, {"Windscreen", "MudB", &c.wsMudB, nullptr, 0, 1},
        {"Windscreen", "WiperThick", &c.wsWiperThick, nullptr, 0.3f, 3}, {"Windscreen", "WiperBow", nullptr, &c.wsWiperBow, 0, 1}, {"Windscreen", "WiperBlur", nullptr, &c.wsWiperBlur, 0, 1},
        {"Windscreen", "Wipers", nullptr, &c.wsWipers, 0, 1}, {"Windscreen", "WiperSource", nullptr, &c.wsWiperSource, 0, 1},
        {"Windscreen", "WiperAxis", nullptr, &c.wsWiperAxis, 0, 2}, {"Windscreen", "WiperAxisSign", nullptr, &c.wsWiperAxisSign, -1, 1}, {"Windscreen", "WiperLenM", &c.wsWiperLenM, nullptr, 0.1f, 2},
        {"Windscreen", "HideGameWipers", nullptr, &c.wsHideGameWipers, 0, 1}, {"Windscreen", "WiperDraw", nullptr, &c.wsWiperDraw, 0, 1}, {"Windscreen", "WiperAuto", nullptr, &c.wsWiperAuto, 0, 1}, {"Windscreen", "WiperDebug", nullptr, &c.wsWiperDebug, 0, 1},
        {"Windscreen", "WiperSyncKey", nullptr, &c.wsWiperSyncKey, 0, 254}, {"Windscreen", "WiperPeriod", &c.wsWiperPeriod, nullptr, 0.3f, 10}, {"Windscreen", "WiperPause", &c.wsWiperPause, nullptr, 0, 20},
        {"Windscreen", "W1X", &c.wsW1X, nullptr, -0.5f, 1.5f}, {"Windscreen", "W1Y", &c.wsW1Y, nullptr, -0.5f, 1.5f}, {"Windscreen", "W1Len", &c.wsW1Len, nullptr, 0, 3},
        {"Windscreen", "W1A0", &c.wsW1A0, nullptr, -360, 360}, {"Windscreen", "W1A1", &c.wsW1A1, nullptr, -360, 360},
        {"Windscreen", "W2X", &c.wsW2X, nullptr, -0.5f, 1.5f}, {"Windscreen", "W2Y", &c.wsW2Y, nullptr, -0.5f, 1.5f}, {"Windscreen", "W2Len", &c.wsW2Len, nullptr, 0, 3},
        {"Windscreen", "W2A0", &c.wsW2A0, nullptr, -360, 360}, {"Windscreen", "W2A1", &c.wsW2A1, nullptr, -360, 360},

        {"Axes", "Right", nullptr, &c.axRight, 0, 2}, {"Axes", "Up", nullptr, &c.axUp, 0, 2}, {"Axes", "Forward", nullptr, &c.axFwd, 0, 2},
        {"Axes", "SignRight", nullptr, &c.sgRight, -1, 1}, {"Axes", "SignUp", nullptr, &c.sgUp, -1, 1}, {"Axes", "SignForward", nullptr, &c.sgFwd, -1, 1},
        {"Render", "FixedFunction", nullptr, &c.fixedFunction, 0, 1},
        {"Render", "SignRight", nullptr, &c.rSgRight, -1, 1}, {"Render", "SignUp", nullptr, &c.rSgUp, -1, 1}, {"Render", "SignForward", nullptr, &c.rSgFwd, -1, 1},
        {"Render", "SignPitch", nullptr, &c.rSgPitch, -1, 1}, {"Render", "SignYaw", nullptr, &c.rSgYaw, -1, 1}, {"Render", "SignRoll", nullptr, &c.rSgRoll, -1, 1},
    };
}

// ---- per-stage rain / mud for WindscreenFX. RBR has no weather system: whether a stage is wet is a fixed property of the stage, and the
// telemetry does not say. The plugin is told the map id when a stage starts (not its name), so [WindscreenStages] maps id -> rain[,mud].
struct WsStageRule { int id; float rain, mud; };
std::vector<WsStageRule> g_wsStageRules;
int   g_wsStageId = -1;
float g_wsRainEff = 0.7f, g_wsMudMul = 1.f, g_wsRuleRain = -1.f;                         // rain level (0..1) and mud multiplier in force for the current stage
void wsLoadStageRules(const std::string& path) {
    g_wsStageRules.clear();
    static char buf[8192];
    const DWORD n = GetPrivateProfileSectionA("WindscreenStages", buf, sizeof(buf), path.c_str());
    for (const char* q = buf; q < buf + n && *q; q += strlen(q) + 1) {
        const char* eq = strchr(q, '='); if (!eq) continue;
        char* end = nullptr; const long id = strtol(q, &end, 10);
        if (end == q) continue;                                        // keys must start with the map id (comment lines never do)
        char* e2 = nullptr; const float rain = strtof(eq + 1, &e2);
        if (e2 == eq + 1) continue;
        while (*e2 == ' ' || *e2 == '\t') ++e2;
        float mud = 1.f; if (*e2 == ',') mud = strtof(e2 + 1, nullptr);
        g_wsStageRules.push_back({(int)id, std::max(0.f, std::min(1.f, rain)), std::max(0.f, std::min(5.f, mud))});
    }
}
void wsApplyStage() {                                                // an explicit [WindscreenStages] rule overrides what is read from the game
    g_wsRuleRain = -1.f; g_wsMudMul = 1.f;
    for (const WsStageRule& r : g_wsStageRules) if (r.id == g_wsStageId) {
        g_wsRuleRain = r.rain; g_wsMudMul = r.mud;
        Logf("windscreen: map id %d has an override rule: rain=%.2f mud x%.2f", g_wsStageId, g_wsRuleRain, g_wsMudMul); return;
    }
    if (!g_wsStageRules.empty()) Logf("windscreen: map id %d has no override rule (%d rules loaded)", g_wsStageId, (int)g_wsStageRules.size());
}

int g_rawMethod = 2;
void loadConfig() {
    const std::string p = g_dir + "ForceFXCamera.ini";     // a missing file or key simply uses the built-in default
    g_rawMethod = (int)std::lround(iniF(p, "General", "Method", 2));
    Cfg c; auto t = optTable(c);
    for (const Opt& o : t) {
        if (o.f) *o.f = std::max(o.lo, std::min(iniF(p, o.sec, o.key, *o.f), o.hi));
        else     *o.i = (int)std::lround(std::max(o.lo, std::min(iniF(p, o.sec, o.key, (float)*o.i), o.hi)));
    }
    auto sgn = [](int& v) { v = (v < 0) ? -1 : 1; };                 // 0 is not a valid sign
    sgn(c.invSurge); sgn(c.invSway); sgn(c.invHeave); sgn(c.invYaw); sgn(c.invSteer); sgn(c.sgRight); sgn(c.sgUp); sgn(c.sgFwd);
    sgn(c.wsSignSway); sgn(c.wsSignSurge); sgn(c.invCarRoll); sgn(c.rSgRight); sgn(c.rSgUp); sgn(c.rSgFwd); sgn(c.rSgPitch); sgn(c.rSgYaw); sgn(c.rSgRoll);
    if (c.axRight == c.axUp || c.axRight == c.axFwd || c.axUp == c.axFwd) { c.axRight = 0; c.axUp = 1; c.axFwd = 2; }
    if (c.fullSpeed <= c.minSpeed + 0.5f) c.fullSpeed = c.minSpeed + 0.5f;
    c.wsEnable = g_cfg.wsEnable;                                     // set once at startup from the ForceFX settings; not part of this file
    g_cfg = c;
    wsLoadStageRules(p); wsApplyStage();
}

// ================================================================================================ game memory
// The hooks below sit at fixed addresses of the RallySimFans executable, so they are only installed in that one.
bool isRsfExe() {
    char p[MAX_PATH]; const DWORD n = GetModuleFileNameA(nullptr, p, MAX_PATH);
    std::string f(p, n); const size_t k = f.find_last_of("\\/"); if (k != std::string::npos) f = f.substr(k + 1);
    for (char& ch : f) ch = (char)std::tolower((unsigned char)ch);
    return f == "richardburnsrally_sse.exe";
}
uintptr_t absAddr(uintptr_t a) {   // 0x400000-based absolute address -> real address (handles a relocated exe)
    return (uintptr_t)GetModuleHandleA(nullptr) + a - 0x400000;
}

// Camera type via the pointer chain openRBRVR uses. 0xFFFFFFFF = not valid (yet).
uint32_t readCameraType() {
    __try {
        uintptr_t carInfo = *(uintptr_t*)absAddr(0x165FC68);
        if (!carInfo) return 0xFFFFFFFF;
        uintptr_t camData = *(uintptr_t*)(carInfo + 0x758);
        if (!camData) return 0xFFFFFFFF;
        uintptr_t camInfo = *(uintptr_t*)(camData + 0x10);
        if (!camInfo) return 0xFFFFFFFF;
        return *(uint32_t*)camInfo;
    } __except (EXCEPTION_EXECUTE_HANDLER) { return 0xFFFFFFFF; }
}
bool isCockpit(uint32_t t) { return t >= 3 && t <= 6; }      // cockpit camera family in openRBRVR (5 = interior in your logs)
bool sane3(const float* v) {
    for (int i = 0; i < 3; ++i) if (!std::isfinite(v[i]) || std::fabs(v[i]) > 1.0e5f) return false;
    return true;
}

// ================================================================================================ telemetry
constexpr int kOffCarRoll = 76, kOffSteer = 24, kOffThrottle = 28, kOffBrake = 32, kOffGear = 44, kOffSpeed = 60, kOffYawRate = 108;
constexpr int kOffSusp = 152, kSuspStride = 128, kOffStrut = 16, kMinSuspPacket = 560;
constexpr int kOffSurge = 112, kOffSway = 116, kOffHeave = 120, kMinPacket = 136;

struct Inputs {                                                       // written by processPacket(), read by the render thread
    std::atomic<float> surge{0}, sway{0}, heave{0}, heaveHp{0}, yawN{0}, steerN{0}, speed{0}, brake{0}, carRoll{0};
} g_in;
std::atomic<float>    g_pkSurge{0}, g_pkSway{0}, g_pkHeave{0}, g_pkYaw{0}, g_pkSteer{0}, g_pkCarRoll{0};
char                  g_pktLast[1500]; std::atomic<int> g_pktLen{0};      // last telemetry packet (only kept when [Windscreen] Probe=1)
std::atomic<float>    g_wsSplash{0};                                       // landing / impact strength handed to WindscreenFX (mud splatter)
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
    if (g_cfg.wsProbe) { memcpy(g_pktLast, buf, (size_t)std::min(n, 1500)); g_pktLen = n; }
    LARGE_INTEGER qn; QueryPerformanceCounter(&qn);
    float dt = (float)(qn.QuadPart - qlast.QuadPart) / (float)qf.QuadPart; qlast = qn;
    if (!(dt > 0.0005f && dt < 0.25f)) dt = 0.01f;
    tsec += dt;

    const Cfg c = g_cfg;                                           // snapshot (config can be reloaded live)
    const float su = clean(rdf(buf, kOffSurge)), sw = clean(rdf(buf, kOffSway)), he = clean(rdf(buf, kOffHeave));
    const float yr = clean(rdf(buf, kOffYawRate)), st = clean(rdf(buf, kOffSteer));
    const float spd = clean(rdf(buf, kOffSpeed)), brk = clamp01(clean(rdf(buf, kOffBrake)));
    const int   gear = rdi(buf, kOffGear);
    const float cr = clean(rdf(buf, kOffCarRoll));

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
            for (int i = 0; i < 4; ++i) ls += std::fabs(clean(rdf(buf, kOffSusp + i * kSuspStride + kOffStrut)));
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

// ================================================================================================ head model
struct Spring {                                                        // critically-tunable 2nd-order follower
    float x = 0, v = 0;
    void step(float h, float xeq, float w, float z) { const float a = w * w * (xeq - x) - 2.f * z * w * v; v += a * h; x += v * h; }
    void reset() { x = v = 0; }
};
struct Head {
    Spring torso[3], neck[3];                                          // 0 = forward, 1 = right, 2 = up   (metres)
    Spring bump;                                                       // up (metres)
    Spring pitch, look, roll;                                          // degrees: pitch + = look down, look + = look right, roll + = tilt right
    void reset() { for (int i = 0; i < 3; ++i) { torso[i].reset(); neck[i].reset(); } bump.reset(); pitch.reset(); look.reset(); roll.reset(); }
} g_head;

struct Shake { float amp = 0, t = 0, ph[5][2] = {{0}}; } g_shake;                  // decaying multi-frequency shake for crashes / landings
unsigned g_rng = 12345u;
float rnd01() { g_rng = g_rng * 1664525u + 1013904223u; return (float)(g_rng >> 8) / 16777216.f; }
void startShake(float add) {
    g_shake.amp = std::min(1.f, g_shake.amp + add); g_shake.t = 0.f;
    for (int i = 0; i < 5; ++i) for (int j = 0; j < 2; ++j) g_shake.ph[i][j] = rnd01() * 6.2831853f;
}

struct Drive { float gS = 0, gSw = 0, gH = 0, gHp = 0, yawN = 0, steerN = 0, speed = 0, brake = 0, carRoll = 0; };

LARGE_INTEGER g_qf, g_qlast;
bool  g_dirty = false;                                                 // camera currently carries a non-zero offset
float g_offPos[3] = {0, 0, 0};                                         // camera-axis position offset (metres)
float g_angPitch = 0, g_angYaw = 0;                                    // radians (+ pitch = look down, + yaw = look right)
float g_hF = 0, g_hR = 0, g_hU = 0, g_hPitchDeg = 0, g_hYawDeg = 0, g_hRollDeg = 0;   // internal head pose (forward/right/up metres, degrees) for Method 4
void resetHead() { g_head.reset(); g_shake.amp = 0.f; QueryPerformanceCounter(&g_qlast); for (int i = 0; i < 3; ++i) g_offPos[i] = 0; g_angPitch = g_angYaw = 0; }
void zeroOffsets() { for (int i = 0; i < 3; ++i) g_offPos[i] = 0; g_angPitch = g_angYaw = 0; g_hF = g_hR = g_hU = g_hPitchDeg = g_hYawDeg = g_hRollDeg = 0; }

float softLimit(float x, float m) { return m * std::tanh(x / m); }
float speedFade(float spd) {
    const float t = clamp01((spd - g_cfg.minSpeed) / (g_cfg.fullSpeed - g_cfg.minSpeed));
    return t * t * (3.f - 2.f * t);                                    // smoothstep
}

void stepHead(float dt, const Drive& d) {
    const Cfg& c = g_cfg;
    const float TWO_PI = 6.2831853f, DEG2RAD = 0.017453293f;
    const float fade = speedFade(d.speed);
    const float boost = 1.f + (c.brakeBoost - 1.f) * clamp01(d.brake);

    const float imp = g_shiftImpulse.exchange(0.f);                    // gear-shift kick: forward impulse on the neck
    if (imp != 0.f) g_head.neck[0].v += imp * c.shiftKick;

    // impact events from the telemetry thread
    const float evS = g_evCrashSurge.exchange(0.f), evW = g_evCrashSway.exchange(0.f), evL = g_evLand.exchange(0.f);
    if (evS != 0.f || evW != 0.f || evL != 0.f) g_wsSplash = std::max(g_wsSplash.load(), std::max(evL, std::max(std::fabs(evS), std::fabs(evW)) * 0.3f));
    if (c.impactScale > 0.f && (evS != 0.f || evW != 0.f || evL != 0.f)) {
        const float sc = c.impactScale;
        g_head.neck[0].v += clampf(-c.crashKick * evS * sc, c.maxKick);            // frontal impact (evS < 0): head thrown forward
        g_head.neck[1].v += clampf(-c.crashKick * evW * sc, c.maxKick);            // side impact: head thrown the other way
        g_head.pitch.v   += clampf(-c.crashPitchKick * evS * sc, 60.f);
        g_head.roll.v    += clampf(c.crashRollKick * evW * sc, 60.f);
        if (evL > 0.f) {                                                           // landing: head dips, nods, jolts
            g_head.torso[2].v += clampf(-c.landKick * evL * sc, c.maxKick);
            g_head.bump.v     += clampf(-c.landKick * 0.5f * evL * sc, c.maxKick);
            g_head.pitch.v    += c.landPitchKick * evL * sc;
        }
        startShake(c.shakeGain * sc * (std::fabs(evS) + std::fabs(evW) + 0.6f * evL));
    }

    // static targets: the head ends up OPPOSITE to the car's acceleration (internal frame: + = forward / right / up)
    const float eqF = -c.kSurge * d.gS * boost * fade;
    const float eqR = -c.kSway * d.gSw * fade;
    const float eqU = -c.kHeave * d.gH * fade + (g_airborne.load() ? c.airLift : 0.f);   // airborne: the head floats up a little
    const float eqB = -c.bumpGain * d.gHp * fade;
    const float eqPitch = -c.pitchPerG * d.gS * boost * fade;                              // braking (gS < 0) -> look down
    const float eqLook = c.lookYaw * ((1.f - c.lookSteerMix) * d.yawN + c.lookSteerMix * d.steerN) * fade;
    // roll: lean the head into the corner (+ = tilt right when the car accelerates to the right) and optionally cancel the car's own roll
    const float carRollDeg = d.carRoll * c.carRollScale * 57.29578f * (float)c.invCarRoll;
    const float eqRoll = (c.rollPerG * d.gSw - c.horizonLock * carRollDeg) * fade;

    const float wT = TWO_PI * c.torsoFreq, wN = TWO_PI * c.neckFreq, wB = TWO_PI * c.bumpFreq,
                wP = TWO_PI * c.pitchFreq, wL = TWO_PI * c.lookFreq, wR = TWO_PI * c.rollFreq;
    const int   steps = std::max(1, (int)std::ceil(dt * 240.f));       // fixed ~240 Hz sub-stepping
    const float h = dt / (float)steps, sh = c.torsoShare, eq3[3] = {eqF, eqR, eqU};
    for (int s = 0; s < steps; ++s) {
        for (int i = 0; i < 3; ++i) {
            g_head.torso[i].step(h, eq3[i] * sh, wT, c.torsoDamp);
            g_head.neck[i].step(h, eq3[i] * (1.f - sh), wN, c.neckDamp);
        }
        g_head.bump.step(h, eqB, wB, c.bumpDamp);
        g_head.pitch.step(h, eqPitch, wP, c.pitchDamp);
        g_head.look.step(h, eqLook, wL, c.lookDamp);
        g_head.roll.step(h, eqRoll, wR, c.rollDamp);
    }

    float F = g_head.torso[0].x + g_head.neck[0].x;
    float R = g_head.torso[1].x + g_head.neck[1].x;
    float U = g_head.torso[2].x + g_head.neck[2].x + g_head.bump.x;
    if (!std::isfinite(F + R + U + g_head.pitch.x + g_head.look.x + g_head.roll.x)) { g_head.reset(); F = R = U = 0; }

    F = softLimit(F, c.maxPos) * c.intensity; R = softLimit(R, c.maxPos) * c.intensity; U = softLimit(U, c.maxPos) * c.intensity;
    float shF = 0, shR = 0, shU = 0, shPitch = 0, shRoll = 0;
    if (g_shake.amp > 1e-4f) {
        g_shake.t += dt; g_shake.amp *= std::exp(-dt / c.shakeDecay);
        float v[5];
        for (int i = 0; i < 5; ++i)
            v[i] = 0.6f * std::sin(TWO_PI * c.shakeFreq1 * g_shake.t + g_shake.ph[i][0]) + 0.4f * std::sin(TWO_PI * c.shakeFreq2 * g_shake.t + g_shake.ph[i][1]);
        const float a = g_shake.amp * c.impactScale * c.intensity;
        shF = a * c.shakePos * 0.5f * v[0]; shR = a * c.shakePos * 0.7f * v[1]; shU = a * c.shakePos * v[2];
        shPitch = a * c.shakeAng * 0.8f * v[3]; shRoll = a * c.shakeAng * 0.6f * v[4];
    }
    F += shF; R += shR; U += shU;
    zeroOffsets();
    g_offPos[c.axFwd] = c.sgFwd * F; g_offPos[c.axRight] = c.sgRight * R; g_offPos[c.axUp] = c.sgUp * U;
    g_angPitch = (softLimit(g_head.pitch.x, c.maxAng) * c.intensity + shPitch) * DEG2RAD;
    g_angYaw   = softLimit(g_head.look.x,  c.maxAng) * c.intensity * DEG2RAD;
    g_hF = F; g_hR = R; g_hU = U;
    g_hPitchDeg = g_angPitch / DEG2RAD; g_hYawDeg = g_angYaw / DEG2RAD;
    g_hRollDeg = softLimit(g_head.roll.x, c.maxRoll) * c.intensity + shRoll;
}

// Add the head offset to cam_pos and rotate the look direction (cam_target - cam_pos) by the head angles.
void applyHead(float* pos, float* tgt) {
    const Cfg& c = g_cfg;
    float d[3] = {tgt[0] - pos[0], tgt[1] - pos[1], tgt[2] - pos[2]};
    const float f = d[c.axFwd] * c.sgFwd, r = d[c.axRight] * c.sgRight, u = d[c.axUp] * c.sgUp;   // forward / right / up components
    const float cp = std::cos(g_angPitch), sp = std::sin(g_angPitch), cy = std::cos(g_angYaw), sy = std::sin(g_angYaw);
    const float f1 = f * cp + u * sp, u1 = u * cp - f * sp;            // pitch (+ = look down)
    const float f2 = f1 * cy - r * sy, r2 = r * cy + f1 * sy;          // yaw   (+ = look right)
    d[c.axFwd] = f2 * c.sgFwd; d[c.axRight] = r2 * c.sgRight; d[c.axUp] = u1 * c.sgUp;
    for (int i = 0; i < 3; ++i) { pos[i] += g_offPos[i]; tgt[i] = pos[i] + d[i]; }
}

// scripted inputs (internal sign convention) to verify every axis from the cockpit
const char* testDrive(DWORD now, Drive& d, int& phaseOut) {
    static DWORD t0 = 0; if (!t0) t0 = now;
    const float t = (float)(now - t0) * 0.001f, cyc = 1.6f;
    const int phase = (int)(t / cyc) % 8; const float local = std::fmod(t, cyc);
    phaseOut = phase;
    const bool on = local < 1.0f;
    d = Drive(); d.speed = 100.f; d.brake = 0.f;
    static int lastCycle = -1; const int cycleNo = (int)(t / cyc); const bool newCycle = cycleNo != lastCycle; lastCycle = cycleNo;
    switch (phase) {
    case 0: if (on) { d.gS = -2.f; d.brake = 1.f; } return "BRAKING: head should lean FORWARD and nod DOWN";
    case 1: if (on) d.gS = 1.5f;                    return "ACCELERATING: head should go BACK and nod UP";
    case 2: if (on) { d.gSw = 2.f; d.yawN = 1.f; d.steerN = 1.f; } return "RIGHT TURN: head moves LEFT (outside), view looks RIGHT, head tilts RIGHT (roll: Method 4)";
    case 3: if (on) { d.gSw = -2.f; d.yawN = -1.f; d.steerN = -1.f; } return "LEFT TURN: head moves RIGHT (outside), view looks LEFT, head tilts LEFT (roll: Method 4)";
    case 4: if (local < 0.15f) { d.gH = 3.f; d.gHp = 3.f; } return "BUMP: head should dip DOWN briefly, then bounce";
    case 5: if (local < 0.15f) { d.gH = -3.f; d.gHp = -3.f; } return "DROP: head should rise UP briefly, then settle";
    case 6: if (newCycle) g_evLand = 1.5f; return "LANDING: head dips, nods and jolts, then a short shake";
    default: if (newCycle) { g_evCrashSurge = -3.f; g_evCrashSway = 1.5f; } return "CRASH: head thrown FORWARD and slightly LEFT with a decaying shake";
    }
}

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
M4 g_lastMvp = m4Ident(); bool g_lastMvpOk = false;                    // the game's own MVP of the draw about to happen (before the head patch), for the WindscreenFX wiper hunter
M4 g_wsP = m4Ident(), g_wsPinv = m4Ident(); bool g_wsPok = false;      // full scene projection (+ inverse) captured during the 3D pass, for WindscreenFX
float g_wsPzs = 0.f, g_wsPc33 = 0.f, g_wsPc43 = 0.f;                   // scene projection terms for the glass depth (WindscreenFX), captured during the 3D pass
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

// ================================================================================================ WindscreenFX (v3)
// Rain, snow and mud on the glass of the cockpit view, drawn through the D3D9 device (pre-transformed vertices, procedural textures).
//  - Drops are refractive: a copy of the frame is sampled upside down and magnified inside each drop, like a real bead of water. Moving drops
//    become teardrops, leave a streak and swallow the drops they hit. Rain falls at a rate set by the game's own sky type and your speed.
//  - Snow flakes stick, melt slowly into water drops, and turn thrown mud into grey slush.
//  - Mud arrives as splatters (rough, wet, fast) and landings; wipers smear it and sweep drops away.
//  - Everything lives on the glass plane and follows the head transform, so it stays on the car.
//  - HideGameFx suppresses the game's own wet-windscreen drawing so only these effects remain.
struct WsVert  { float x, y, z, rhw; DWORD color; float u, v; };
struct WsVert2 { float x, y, z, rhw; DWORD color; float u0, v0, u1, v1; };
constexpr DWORD kWsFvf1 = D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX1;
constexpr DWORD kWsFvf2 = D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX2;
struct WsDrop  { float x, y, r, vx, vy, age, thr, tx, ty, tr, melt, melt0, rot, asp; int kind, var; bool dead, sliding; };   // x,y: 0..1 of the viewport; r: fraction of viewport height; kind 0 water, 1 snow flake, 2 micro droplet; var / asp: pool shape
struct WsMud   { float x, y, r, rot, age, fade, stretch, shade, ox, oy, dur; int variant, tint; bool dying; };   // age < 0: waiting; ox, oy, dur > 0: thrown outwards from (ox, oy) over dur seconds; tint 0 mud, 1 slush
// A particle on its way to the glass (outside it). kind 0 rain bead, 1 mud clod, 2 snow flake, 3 dirty road spray. Positions are viewport fractions; t runs 0..1
// over dur seconds (negative = not started yet). mode 0: perspective approach from a far point (rain, snow, mud thrown by a car ahead); mode 1: thrown up from
// below (mud off your own wheels), fast at first and slowing as it arrives.
struct WsFly   { float x0, y0, x1, y1, t, dur, r, arc, shade; int kind, mode, variant, tint; };
struct WsTrail { float x0, y0, x1, y1, w0, w1, age; };                                                   // positions as x,y fractions, half widths as fractions of height

using EndSceneFn = HRESULT(__stdcall*)(void* dev);
using ResetFn    = HRESULT(__stdcall*)(void* dev, void* pp);
using DrawPrimFn      = HRESULT(__stdcall*)(void* dev, DWORD type, UINT start, UINT prims);
using DrawIdxPrimFn   = HRESULT(__stdcall*)(void* dev, DWORD type, INT baseVtx, UINT minVtx, UINT numVtx, UINT startIdx, UINT prims);
using DrawPrimUPFn    = HRESULT(__stdcall*)(void* dev, DWORD type, UINT prims, const void* data, UINT stride);
using DrawIdxPrimUPFn = HRESULT(__stdcall*)(void* dev, DWORD type, UINT minVtx, UINT numVtx, UINT prims, const void* idx, DWORD idxFmt, const void* vtx, UINT stride);
EndSceneFn g_origEndScene = nullptr;
ResetFn    g_origReset = nullptr;
DrawPrimFn g_origDrawPrim = nullptr; DrawIdxPrimFn g_origDrawIdxPrim = nullptr; DrawPrimUPFn g_origDrawPrimUP = nullptr; DrawIdxPrimUPFn g_origDrawIdxPrimUP = nullptr;
volatile bool g_wsSuppress = false, g_wsDrawing = false;          // g_wsSuppress: dropping the game's own windscreen draws; g_wsDrawing: our own draws always pass
std::atomic<long> g_wsHidden{0}, g_wsWiped{0};      // draw calls dropped / things cleared by the wipers, counted per second in the log

IDirect3DDevice9*     g_wsDev = nullptr;
IDirect3DTexture9    *g_wsPoolMaskTex = nullptr, *g_wsPoolShadeTex = nullptr, *g_wsMicroTex = nullptr, *g_wsSpriteTex = nullptr, *g_wsMaskTex = nullptr, *g_wsShadeTex = nullptr, *g_wsMudTex = nullptr, *g_wsTrailTex = nullptr,
                     *g_wsSplashTex = nullptr, *g_wsFlakeTex = nullptr, *g_wsWhiteTex = nullptr, *g_wsStreakTex = nullptr;
IDirect3DTexture9*    g_wsSceneTex = nullptr; IDirect3DSurface9* g_wsSceneSurf = nullptr; UINT g_wsSceneW = 0, g_wsSceneH = 0; int g_wsSceneFmt = 0;
IDirect3DTexture9*    g_wsBlurTex[2] = {nullptr, nullptr}; IDirect3DSurface9* g_wsBlurSurf[2] = {nullptr, nullptr};   // 1/2 and 1/4 of the scene copy: the veil blur
bool                  g_wsRefrFailed = false, g_wsRefrUsed = false, g_wsVeilFailed = false, g_wsVeilUsed = false;
IDirect3DStateBlock9* g_wsSB = nullptr;
std::vector<WsDrop>   g_wsDrops;
std::vector<WsMud>    g_wsMud;
std::vector<WsTrail>  g_wsTrails;
std::vector<WsFly>    g_wsFly;
std::vector<WsVert>   g_wsVSprite, g_wsVShade, g_wsVShadePool, g_wsVMud, g_wsVTrail, g_wsVFlake, g_wsVSplash, g_wsVDbg, g_wsVMicro, g_wsVVeil, g_wsVDirt, g_wsVGlow, g_wsVFlyMud, g_wsVFlyFlake, g_wsVStreak, g_wsVWBlur;
std::vector<WsVert2>  g_wsVRefr, g_wsVRefrPool, g_wsVFrost;
volatile bool g_wsArmed = false, g_wsHooked = false;
bool          g_wsReset = true, g_wsTexFailed = false;
DWORD         g_wsStamp = 0;
long          g_wsFrameNo = 0, g_wsLastDrawn = -1;
std::atomic<long> g_wsDrawn{0}, g_wsSkipped{0};
UINT          g_wsVpW = 0, g_wsVpH = 0;
uint32_t      g_wsRng = 0x9E3779B9u;
float         g_wsSprayAcc = 0.f, g_wsRainAcc = 0.f, g_wsSnowAcc = 0.f, g_wsMudAcc = 0.f, g_wsMicroAcc = 0.f, g_wsAspect = 16.f / 9.f, g_wsSnowEff = 0.f;
std::atomic<long>  g_wsGameCalls{0};      // calls of the game's own wet-windscreen function (0x452020), counted per second in the log
std::atomic<DWORD> g_wsGameTick{0};       // tick of the last such call
float         g_wsRough = 0.f, g_wsLoose = 0.f, g_wsSpdN = 0.f, g_wsDbgRain = 0.f, g_wsDbgMud = 0.f;   // telemetry-derived factors (logged for calibration)
uintptr_t     g_wsProbeA = 0;             // object passed to the game's wet-windscreen function (Probe=1 dumps / scans it)
float         g_wsWet = 1.f;              // 0..1 rain rate scale; follows the game's wet-windscreen calls when [Windscreen] WetFromGame=1
// the car's real wiper blades, found among the 3D draw calls by the hunter (see wsHuntReport) and turned into a pose every frame
struct WsModelBlade { bool ok; float px, py, len, ang; };            // pivot (fraction of width / height), length (fraction of height), angle (rad, 0 = right, CCW with y up)
WsModelBlade g_wsModelBlade[2] = {{false, 0, 0, 0, 0}, {false, 0, 0, 0, 0}};
bool g_wsHuntLocked = false;
std::atomic<long> g_wsHiddenWipers{0};
void wsHuntPose(float W, float H);                                   // defined with the hunter
void wsHuntReset();
void wsHuntTick();
// wipers (emulated sweep, see wsWiperUpdate)
float g_wsBladeAng[2] = {0, 0}, g_wsBladePrev[2] = {0, 0}, g_wsWiperW = 0.f; bool g_wsBladeInit = false; DWORD g_wsWipeT0 = 0;

float wsRnd() { g_wsRng ^= g_wsRng << 13; g_wsRng ^= g_wsRng >> 17; g_wsRng ^= g_wsRng << 5; return (float)(g_wsRng & 0xFFFFFF) / 16777216.0f; }
float wsRange(float a, float b) { return a + (b - a) * wsRnd(); }
float wsSmooth(float a, float b, float x) { const float t = clamp01((x - a) / (b - a)); return t * t * (3.f - 2.f * t); }   // works for a > b too
float wsHash(int x, int y, int seed) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + (uint32_t)seed * 2147483647u + 1013904223u;
    h = (h ^ (h >> 13)) * 1274126177u; h ^= h >> 16; return (float)(h & 0xFFFFFF) / 16777216.f;
}
float wsVNoise(float x, float y, int seed) {
    const int xi = (int)std::floor(x), yi = (int)std::floor(y); float fx = x - (float)xi, fy = y - (float)yi;
    fx = fx * fx * (3.f - 2.f * fx); fy = fy * fy * (3.f - 2.f * fy);
    const float a = wsHash(xi, yi, seed), b = wsHash(xi + 1, yi, seed), c = wsHash(xi, yi + 1, seed), d = wsHash(xi + 1, yi + 1, seed);
    return (a + (b - a) * fx) * (1.f - fy) + (c + (d - c) * fx) * fy;
}
float wsFbm(float x, float y, int seed, int oct) {
    float s = 0.f, a = 0.5f, f = 1.f, n = 0.f;
    for (int i = 0; i < oct; ++i) { s += a * wsVNoise(x * f, y * f, seed + i * 17); n += a; a *= 0.5f; f *= 2.03f; }
    return s / n;
}

// Glass coverage grids (viewport fractions): film = water haze from rain / snow impacts, dirt = mud haze. They blur the view behind the glass in proportion,
// fill up while it rains, and the wipers clear them (dirt only partly, so it smears).
constexpr int kGX = 80, kGY = 45;
float g_wsFilm[kGX * kGY], g_wsDirt[kGX * kGY];
float g_wsBow[2] = {0.f, 0.f}, g_wsGhost[2][2] = {{0.f, 0.f}, {0.f, 0.f}};      // wiper: water ridge pushed ahead of each blade, motion-blur ghost angles
void wsGridAdd(float* g, float x, float y, float rc, float amount) {
    const float cx = x * (float)kGX - 0.5f, cy = y * (float)kGY - 0.5f;
    const int x0 = std::max(0, (int)std::floor(cx - rc)), x1 = std::min(kGX - 1, (int)std::ceil(cx + rc)), y0 = std::max(0, (int)std::floor(cy - rc)), y1 = std::min(kGY - 1, (int)std::ceil(cy + rc));
    for (int j = y0; j <= y1; ++j) for (int i = x0; i <= x1; ++i) {
        const float dx = (float)i - cx, dy = (float)j - cy, dd = std::sqrt(dx * dx + dy * dy);
        if (dd >= rc) continue;
        const float w = 1.f - dd / rc; float& c = g[j * kGX + i]; c = std::min(1.f, c + amount * w * w * (3.f - 2.f * w));
    }
}
float wsGridMax(const float* g) { float m = 0.f; for (int i = 0; i < kGX * kGY; ++i) m = std::max(m, g[i]); return m; }

// The game's resolved stage conditions. NGPCarMenu documents RBRMapSettingsEx at the fixed address 0x8938F8: the Good / Random / Bad weather preset
// has already been turned into a sky type (0 crisp, 1 hazy, 2 no rain, 3 light rain, 4 heavy rain, 5 no snow, 6 light snow, 7 heavy snow, 8 light fog,
// 9 heavy fog), a surface wetness (0 dry, 1 damp, 2 wet), a cloud type and a time of day. That is what the game itself keys rain and wipers on.
struct WsGameWeather { bool ok = false; int track = -1, cloud = -1, wet = -1, tod = -1, sky = -1; };
const char* wsSkyName(int s) {
    static const char* n[10] = {"crisp", "hazy", "no rain", "light rain", "heavy rain", "no snow", "light snow", "heavy snow", "light fog", "heavy fog"};
    return (s >= 0 && s < 10) ? n[s] : "?";
}
WsGameWeather wsReadGameWeather() {
    WsGameWeather g;
    const uintptr_t a = absAddr(0x8938F8);
    if (!readableRange(a, 0x40)) return g;
    const int32_t* w = (const int32_t*)a;
    g.track = w[2]; g.cloud = w[4]; g.wet = w[5]; g.tod = w[14]; g.sky = w[15];     // offsets 0x08, 0x10, 0x14, 0x38, 0x3C
    g.ok = (g.sky >= 0 && g.sky <= 9 && g.wet >= 0 && g.wet <= 2);
    return g;
}
WsGameWeather g_wsGW;                                              // refreshed a few times a second from wsStep

// ---- procedural textures ----
struct WsPx { float c[3] = {0, 0, 0}; float a = 0; };            // premultiplied accumulation
void wsOver(WsPx& p, float r, float g, float b, float a) {
    a = clamp01(a); const float col[3] = {r, g, b};
    for (int i = 0; i < 3; ++i) p.c[i] = col[i] * a + p.c[i] * (1.f - a);
    p.a = a + p.a * (1.f - a);
}
void wsScale(WsPx& p, float s) { p.a *= s; for (int i = 0; i < 3; ++i) p.c[i] *= s; }
DWORD wsPack(const WsPx& p) {
    if (p.a <= 1e-4f) return 0;
    auto ch = [&](float c) { return (DWORD)(clamp01(c / p.a) * 255.f + 0.5f); };
    return ((DWORD)(clamp01(p.a) * 255.f + 0.5f) << 24) | (ch(p.c[0]) << 16) | (ch(p.c[1]) << 8) | ch(p.c[2]);
}
template <class F> bool wsMakeTex(IDirect3DDevice9* d, IDirect3DTexture9** t, int w, int h, F fill) {
    if (FAILED(d->CreateTexture(w, h, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, t, nullptr)) || !*t) return false;
    D3DLOCKED_RECT lr;
    if (FAILED((*t)->LockRect(0, &lr, nullptr, 0))) return false;
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) { WsPx p; fill(x, y, p); *(DWORD*)((BYTE*)lr.pBits + y * lr.Pitch + x * 4) = wsPack(p); }
    (*t)->UnlockRect(0);
    return true;
}

// Water bead height fields on a unit cell (v grows downwards). The comet has a round head at the bottom and a tail that tapers to the top;
// moving drops are rotated so the head leads.
float wsHRound(float u, float v) { const float dx = u - 0.5f, dy = v - 0.5f, d = std::sqrt(dx * dx + dy * dy) / 0.46f; return d < 1.f ? std::sqrt(1.f - d * d) : 0.f; }
float wsHComet(float u, float v) {
    const float cy = 0.62f, R = 0.34f;
    if (v >= cy) { const float dx = u - 0.5f, dy = v - cy, d = std::sqrt(dx * dx + dy * dy) / R; return d < 1.f ? std::sqrt(1.f - d * d) : 0.f; }
    const float t = clamp01((v - 0.03f) / (cy - 0.03f)), hw = R * std::pow(t, 0.5f);
    if (hw < 1e-3f) return 0.f;
    const float s = (u - 0.5f) / hw; if (std::fabs(s) >= 1.f) return 0.f;
    return std::sqrt(1.f - s * s) * std::pow(t, 0.5f);
}
float wsBeadH(int cell, float u, float v) { return cell == 0 ? wsHRound(u, v) : wsHComet(u, v); }
void wsBeadShade(int cell, float u, float v, WsPx& p) {            // rim, specular, caustic: no interior fill, the refraction provides that
    const float h = wsBeadH(cell, u, v); if (h <= 0.f) return;
    const float e = 1.f / 64.f, S = 0.30f / (2.f * e);
    const float hx = wsBeadH(cell, u + e, v) - wsBeadH(cell, u - e, v), hy = wsBeadH(cell, u, v + e) - wsBeadH(cell, u, v - e);
    float nx = -hx * S, ny = -hy * S, nz = 1.f; const float nl = std::sqrt(nx * nx + ny * ny + nz * nz); nx /= nl; ny /= nl; nz /= nl;
    const float Lx = -0.45f, Ly = -0.55f, Lz = 0.70f;                // light from upper left, towards the viewer
    const float diff = nx * Lx + ny * Ly + nz * Lz;
    const float rim = 1.f - nz;
    wsOver(p, 0.06f, 0.08f, 0.09f, 0.06f);                              // slightly darker than the glass around it
    wsOver(p, 0.05f, 0.07f, 0.10f, std::pow(rim, 1.3f) * (0.25f + 0.55f * std::max(0.f, -diff)));
    wsOver(p, 0.88f, 0.94f, 1.00f, std::max(0.f, -diff) * (1.f - rim) * (1.f - rim) * 0.32f);          // caustic on the side away from the light
    wsOver(p, 1.f, 1.f, 1.f, std::pow(std::max(0.f, diff), 4.f) * 0.16f);
    float hxv = Lx, hyv = Ly, hzv = Lz + 1.f; const float hl = std::sqrt(hxv * hxv + hyv * hyv + hzv * hzv); hxv /= hl; hyv /= hl; hzv /= hl;
    wsOver(p, 1.f, 1.f, 1.f, std::pow(std::max(0.f, nx * hxv + ny * hyv + nz * hzv), 60.f) * 0.55f);   // specular (small: rain beads on a car windscreen are not glossy)
    wsScale(p, wsSmooth(0.f, 0.10f, h));
}
// Mud / slush atlas: 4 x 2 cells of 128 px (luminance in rgb, tinted by the vertex colour). Cells 0-3 are thrown-mud splats like the ones on a rally car's
// windscreen: ragged spiky outline, translucent lacy interior, darker rim, wet highlights, elongated satellite drops flung outwards. 4 smear (long, streaky),
// 5 and 7 spray clumps, 6 fine speckle of dirty road water.
constexpr int kMudSmear = 4, kMudSpray = 5, kMudSpeckle = 6, kMudSpray2 = 7;
inline bool wsIsSprayV(int v) { return v >= 5; }
float wsAngDiff(float a, float b) { return std::fmod(a - b + 9.42478f, 6.28318f) - 3.14159f; }
void wsMudPx(int variant, float u, float v, WsPx& p) {
    const int seed = 11 + variant * 37;
    const float d = std::sqrt(u * u + v * v), ang = std::atan2(v, u);
    auto dot = [&](float cx, float cy, float rad, float stretch, float a, float lum) {      // ellipse elongated along the direction from the cell centre
        const float dx = u - cx, dy = v - cy, cl = std::sqrt(cx * cx + cy * cy) + 1e-4f, ax = cx / cl, ay = cy / cl;
        const float al = dx * ax + dy * ay, pe = -dx * ay + dy * ax, dd = std::sqrt((al / stretch) * (al / stretch) + pe * pe);
        if (dd < rad) wsOver(p, lum, lum, lum, wsSmooth(rad, rad * 0.45f, dd) * a);
    };
    if (variant == kMudSpray || variant == kMudSpray2) {                // a loose clump of clods: centre-weighted, uneven sizes
        for (int k = 0; k < 10; ++k) {
            const float a = wsHash(k, 1, seed) * 6.2832f, rr = std::pow(wsHash(k, 2, seed), 1.4f) * 0.62f, rad = 0.05f + 0.20f * wsHash(k, 3, seed) * wsHash(k, 3, seed);
            const float cx = std::cos(a) * rr, cy = std::sin(a) * rr;
            const float dx = u - cx, dy = v - cy, dd = std::sqrt(dx * dx + dy * dy);
            if (dd < rad) { const float lum = 0.34f + 0.30f * wsHash(k, 4, seed) + 0.22f * std::exp(-((dx + 0.3f * rad) * (dx + 0.3f * rad) + (dy + 0.35f * rad) * (dy + 0.35f * rad)) / (rad * rad * 0.18f)); wsOver(p, lum, lum, lum, wsSmooth(rad, rad * 0.5f, dd) * 0.95f); }
        }
        return;
    }
    if (variant == kMudSpeckle) {                                      // fine dirty spray: many tiny droplets, some elongated
        for (int k = 0; k < 34; ++k) {
            const float a = wsHash(k, 1, seed) * 6.2832f, rr = std::pow(wsHash(k, 2, seed), 0.8f) * 0.85f, rad = 0.018f + 0.050f * std::pow(wsHash(k, 3, seed), 2.f);
            dot(std::cos(a) * rr, std::sin(a) * rr, rad, 1.f + 1.2f * wsHash(k, 8, seed), 0.55f + 0.4f * wsHash(k, 9, seed), 0.30f + 0.35f * wsHash(k, 4, seed));
        }
        return;
    }
    if (variant == kMudSmear) {                                        // smear: long along v, streaky, translucent
        const float uu = u * 1.25f, vv = v * 0.55f, dd = std::sqrt(uu * uu + vv * vv);
        const float edge = 0.58f + 0.2f * wsFbm(ang * 1.2f + 3.f, 1.f, seed, 3);
        const float n = wsFbm(u * 9.f + 4.f, v * 1.6f, seed, 4);        // striations run along v
        const float dens = (edge - dd) / 0.30f + (n - 0.5f) * 0.9f;
        const float a = wsSmooth(0.0f, 0.65f, dens) * (0.50f + 0.25f * n);
        const float lum = 0.55f + 0.30f * (n - 0.5f);
        wsOver(p, lum, lum, lum, a);
        return;
    }
    // ---- splat
    const int ns = 3 + variant % 3;                                    // major lobes: 3-5 short, wide ones
    float R = 0.36f + 0.09f * (wsFbm(std::cos(ang) * 2.2f + (float)seed, std::sin(ang) * 2.2f + (float)seed * 1.7f, seed, 3) - 0.5f) * 2.f;
    for (int k = 0; k < ns; ++k) {
        const float ak = wsHash(k, 1, seed) * 6.2832f - 3.14159f, w = 0.14f + 0.14f * wsHash(k, 2, seed), len = 0.10f + 0.20f * std::pow(wsHash(k, 3, seed), 1.3f);
        const float x = wsAngDiff(ang, ak) / w; R += len * std::exp(-x * x);
    }
    for (int k = 0; k < 9; ++k) {                                      // minor jagged points all around the edge
        const float ak = wsHash(k, 11, seed) * 6.2832f - 3.14159f, w = 0.06f + 0.07f * wsHash(k, 12, seed), len = 0.035f + 0.075f * wsHash(k, 13, seed);
        const float x = wsAngDiff(ang, ak) / w; R += len * std::exp(-x * x);
    }
    R = std::min(R, 0.84f);
    const float jit = (wsFbm(u * 9.f + (float)seed, v * 9.f, seed + 3, 3) - 0.5f) * 0.09f;
    const float sd = d - R + jit;                                      // negative inside
    const float inside = wsSmooth(0.025f, -0.035f, sd);
    const float grain = wsFbm(u * 6.5f + (float)seed * 0.3f, v * 6.5f + (float)seed * 0.9f, seed + 5, 4), speck = wsVNoise(u * 34.f, v * 34.f, seed + 9);
    if (inside > 0.f) {
        const float core = wsSmooth(R, 0.f, d);
        float a = inside * (0.42f + 0.32f * core + 0.34f * (grain - 0.5f));
        a *= 0.72f + 0.28f * wsSmooth(0.22f, 0.55f, grain);            // lacy, thinner patches
        float lum = 0.72f + 0.60f * (grain - 0.5f) - 0.34f * wsSmooth(-0.16f, 0.0f, sd) + 0.14f * (speck - 0.5f);
        const float hx = u + 0.15f, hy = v + 0.19f; lum += 0.32f * std::exp(-(hx * hx + hy * hy) / 0.012f);        // wet sheen
        const float h2 = (u - 0.10f) * 1.0f, h3 = v + 0.30f; lum += 0.16f * std::exp(-(h2 * h2 * 6.f + h3 * h3 * 90.f));
        wsOver(p, clamp01(lum), clamp01(lum), clamp01(lum), clamp01(a));
        const float kr = (sd + 0.02f) / 0.028f; wsOver(p, 0.16f, 0.16f, 0.16f, std::exp(-kr * kr) * 0.55f * inside);   // dark rim where the mud is thickest
    }
    for (int k = 0; k < 10; ++k) {                                     // satellites flung outwards
        const float ak = wsHash(k, 5, seed) * 6.2832f, rr = std::min(0.90f, R * 0.85f + 0.14f + 0.34f * wsHash(k, 6, seed)), rad = 0.022f + 0.060f * std::pow(wsHash(k, 7, seed), 1.8f);
        dot(std::cos(ak) * rr, std::sin(ak) * rr, rad, 1.f + 1.6f * wsHash(k, 8, seed), 0.88f, 0.34f + 0.22f * wsHash(k, 9, seed));
    }
}


// Flat irregular pools (the big drops in real photos of a rainy windscreen): low profile, so a darker flat interior, a thin dark contact line on the
// upper side and a thin bright arc just inside the lower side, no big highlight. Four shapes in a 2x2 atlas, cell coordinates in [-1,1].
float wsPoolSD(int variant, float u, float v, float& nx, float& ny) {      // signed distance (negative inside) and outward direction
    const float asp = 0.80f + 0.12f * (float)variant;
    const float uu = u / asp, vv = v * asp, d = std::sqrt(uu * uu + vv * vv) + 1e-5f, ang = std::atan2(vv, uu);
    const int seed = 101 + variant * 29;
    const float n = wsFbm(std::cos(ang) * 1.4f + (float)seed, std::sin(ang) * 1.4f + (float)seed * 1.3f, seed, 3);
    float R = 0.62f * (1.f + 0.11f * (n - 0.5f) * 2.f) + 0.05f * std::sin(2.f * ang + (float)variant) + 0.014f * std::sin(3.f * ang + 2.f * (float)variant);
    R = std::max(0.38f, std::min(0.86f, R));
    nx = uu / d; ny = vv / d;
    return d - R;
}
void wsPoolPx(int variant, float u, float v, bool shade, WsPx& p) {
    float nx, ny; const float sd = wsPoolSD(variant, u, v, nx, ny);
    if (!shade) { wsOver(p, 1.f, 1.f, 1.f, wsSmooth(0.04f, -0.04f, sd) * 0.97f); return; }
    if (sd > 0.20f) return;
    const float inside = wsSmooth(0.03f, -0.03f, sd);
    const float mott = wsFbm(u * 3.5f + (float)variant * 7.f, v * 3.5f, 61 + variant, 3);
    const float lowSide = std::max(0.f, nx * -0.30f + ny * 0.95f), upSide = std::max(0.f, ny * -0.95f + nx * 0.20f);
    wsOver(p, 0.08f, 0.11f, 0.10f, inside * (0.27f + 0.14f * mott));                              // flat, darker interior
    wsOver(p, 0.04f, 0.06f, 0.05f, inside * wsSmooth(-0.30f, 0.0f, sd) * upSide * 0.20f);        // the thick part of the film near the upper edge
    wsOver(p, 0.85f, 0.90f, 0.90f, inside * 0.10f * wsSmooth(0.55f, 0.85f, mott));               // faint lighter reflection patches
    const float kd = (sd - 0.005f) / 0.050f, kb = (sd + 0.050f) / 0.055f;
    wsOver(p, 0.05f, 0.07f, 0.06f, std::exp(-kd * kd) * (0.25f + 0.60f * upSide));             // dark contact line
    wsOver(p, 0.93f, 0.96f, 0.95f, std::exp(-kb * kb) * (0.08f + 0.80f * lowSide) * 0.62f);    // bright arc inside the lower edge
}
void wsMicroPx(float u, float v, WsPx& p) {                                  // a tiny drop: dark body, bright speck low on one side
    const float d = std::sqrt(u * u + v * v); if (d >= 1.f) return;
    wsOver(p, 0.06f, 0.08f, 0.08f, wsSmooth(1.0f, 0.55f, d) * 0.42f);
    const float kr = (d - 0.72f) / 0.16f;
    wsOver(p, 0.93f, 0.96f, 0.96f, std::exp(-kr * kr) * std::max(0.f, -u * 0.3f + v * 0.95f) * 0.45f);
    const float sx = u + 0.30f, sy = v - 0.35f;
    wsOver(p, 1.f, 1.f, 1.f, std::exp(-(sx * sx + sy * sy) / 0.06f) * 0.95f);
}

bool wsMakeTextures(IDirect3DDevice9* d) {
    const int N = 64;
    const bool a = wsMakeTex(d, &g_wsSpriteTex, N, N, [&](int x, int y, WsPx& p) {          // small drops: plain alpha sprite
        const float u = (x + 0.5f) / N * 2.f - 1.f, v = (y + 0.5f) / N * 2.f - 1.f, dd = std::sqrt(u * u + v * v);
        if (dd >= 1.f) return;
        const float z = std::sqrt(std::max(0.f, 1.f - dd * dd)), diff = u * -0.45f + v * -0.55f + z * 0.70f;
        wsOver(p, 0.80f, 0.86f, 0.94f, 0.07f);
        wsOver(p, 0.08f, 0.10f, 0.14f, wsSmooth(0.55f, 0.98f, dd) * (0.20f + 0.40f * std::max(0.f, -diff)));
        wsOver(p, 1.f, 1.f, 1.f, std::pow(std::max(0.f, diff), 30.f) * 0.9f);
        const float hx = u + 0.38f, hy = v + 0.42f;
        wsOver(p, 1.f, 1.f, 1.f, wsSmooth(0.22f, 0.04f, std::sqrt(hx * hx + hy * hy)) * 0.8f);
        wsScale(p, wsSmooth(1.0f, 0.88f, dd));
    });
    const bool b = wsMakeTex(d, &g_wsMaskTex, 2 * N, N, [&](int x, int y, WsPx& p) {         // refraction mask: cell 0 round, cell 1 comet
        const int cell = x / N; const float u = ((x % N) + 0.5f) / N, v = (y + 0.5f) / N;
        wsOver(p, 1.f, 1.f, 1.f, wsSmooth(0.f, 0.20f, wsBeadH(cell, u, v)) * 0.96f);
    });
    const bool c = wsMakeTex(d, &g_wsShadeTex, 2 * N, N, [&](int x, int y, WsPx& p) {        // rim / specular overlay for the same two cells
        const int cell = x / N; wsBeadShade(cell, ((x % N) + 0.5f) / N, (y + 0.5f) / N, p);
    });
    const bool e = wsMakeTex(d, &g_wsMudTex, 512, 256, [&](int x, int y, WsPx& p) {          // 4x2 atlas of mud cells
        const int cx = x / 128, cy = y / 128;
        wsMudPx(cy * 4 + cx, ((x % 128) + 0.5f) / 128.f * 2.f - 1.f, ((y % 128) + 0.5f) / 128.f * 2.f - 1.f, p);
    });
    const bool m = wsMakeTex(d, &g_wsStreakTex, 16, 16, [&](int x, int, WsPx& p) {           // soft streak: u across, uniform along v (rain / dirt in flight)
        const float t = std::fabs(((x + 0.5f) / 16.f) * 2.f - 1.f); wsOver(p, 1.f, 1.f, 1.f, std::exp(-t * t * 5.f) * wsSmooth(1.0f, 0.75f, t));
    });
    const bool f = wsMakeTex(d, &g_wsTrailTex, 16, 16, [&](int x, int, WsPx& p) {            // wet streak: u across, uniform along v
        const float u = (x + 0.5f) / 16.f, t = std::fabs(2.f * u - 1.f), k1 = (t - 0.80f) / 0.12f, k2 = (u - 0.30f) / 0.09f;
        wsOver(p, 0.75f, 0.85f, 0.95f, 0.10f);
        wsOver(p, 0.08f, 0.11f, 0.15f, 0.30f * std::exp(-k1 * k1));
        wsOver(p, 1.f, 1.f, 1.f, 0.40f * std::exp(-k2 * k2));
        wsScale(p, wsSmooth(1.0f, 0.82f, t));
    });
    const bool g = wsMakeTex(d, &g_wsSplashTex, N, N, [&](int x, int y, WsPx& p) {           // impact flash: thin ring with spikes
        const float u = (x + 0.5f) / N * 2.f - 1.f, v = (y + 0.5f) / N * 2.f - 1.f, dd = std::sqrt(u * u + v * v), ang = std::atan2(v, u);
        if (dd >= 1.f) return;
        const float k = (dd - 0.62f) / 0.07f, ring = std::exp(-k * k) * 0.65f;
        const float spike = std::pow(std::max(0.f, std::cos(8.f * ang + 0.4f)), 6.f) * wsSmooth(0.15f, 0.4f, dd) * wsSmooth(0.98f, 0.6f, dd) * 0.55f;
        wsOver(p, 0.92f, 0.96f, 1.f, ring + spike);
        wsScale(p, wsSmooth(1.0f, 0.9f, dd));
    });
    const bool h = wsMakeTex(d, &g_wsFlakeTex, N, N, [&](int x, int y, WsPx& p) {            // snow flake: soft core with six feathery arms
        const float u = (x + 0.5f) / N * 2.f - 1.f, v = (y + 0.5f) / N * 2.f - 1.f, dd = std::sqrt(u * u + v * v), ang = std::atan2(v, u);
        if (dd >= 1.f) return;
        const float edge = 0.46f + 0.30f * wsFbm(std::cos(ang) * 1.5f + 3.f, std::sin(ang) * 1.5f + 8.f, 43, 3);
        const float n = wsFbm(u * 3.0f + 5.f, v * 3.0f + 9.f, 41, 4);
        const float dens = (edge - dd) / 0.32f + (n - 0.5f) * 0.9f;
        float al = wsSmooth(0.0f, 0.75f, dens) * 0.96f;
        float lum = 0.84f + 0.16f * n - 0.16f * wsSmooth(0.0f, 0.9f, (u + v) * 0.5f + 0.25f);     // soft shadow towards the lower right
        const float hx = u + 0.22f, hy = v + 0.26f; lum += 0.12f * std::exp(-(hx * hx + hy * hy) / 0.05f);
        if (wsHash(x, y, 77) > 0.975f) lum = 1.f;                      // a few ice glints
        lum = clamp01(lum);
        wsOver(p, lum * 0.96f, lum * 0.985f, lum, al);
        wsScale(p, wsSmooth(1.0f, 0.86f, dd));
    });
    const bool i = wsMakeTex(d, &g_wsWhiteTex, 4, 4, [&](int, int, WsPx& p) { wsOver(p, 1.f, 1.f, 1.f, 1.f); });
    const bool j = wsMakeTex(d, &g_wsPoolMaskTex, 256, 256, [&](int x, int y, WsPx& p) { wsPoolPx((y / 128) * 2 + (x / 128), ((x % 128) + 0.5f) / 128.f * 2.f - 1.f, ((y % 128) + 0.5f) / 128.f * 2.f - 1.f, false, p); });
    const bool k = wsMakeTex(d, &g_wsPoolShadeTex, 256, 256, [&](int x, int y, WsPx& p) { wsPoolPx((y / 128) * 2 + (x / 128), ((x % 128) + 0.5f) / 128.f * 2.f - 1.f, ((y % 128) + 0.5f) / 128.f * 2.f - 1.f, true, p); });
    const bool l = wsMakeTex(d, &g_wsMicroTex, 16, 16, [&](int x, int y, WsPx& p) { wsMicroPx(((x + 0.5f) / 16.f) * 2.f - 1.f, ((y + 0.5f) / 16.f) * 2.f - 1.f, p); });
    return a && b && c && e && f && g && h && i && j && k && l && m;
}

void wsReleaseScene() {
    for (int i = 0; i < 2; ++i) {
        if (g_wsBlurSurf[i]) { g_wsBlurSurf[i]->Release(); g_wsBlurSurf[i] = nullptr; }
        if (g_wsBlurTex[i]) { g_wsBlurTex[i]->Release(); g_wsBlurTex[i] = nullptr; }
    }
    if (g_wsSceneSurf) { g_wsSceneSurf->Release(); g_wsSceneSurf = nullptr; }
    if (g_wsSceneTex) { g_wsSceneTex->Release(); g_wsSceneTex = nullptr; }
    g_wsSceneW = g_wsSceneH = 0;
}
bool wsEnsureResources(IDirect3DDevice9* d) {
    if (!g_wsSpriteTex && !wsMakeTextures(d)) { g_wsTexFailed = true; Note("WindscreenFX: texture creation failed, effect disabled"); return false; }
    if (!g_wsSB && FAILED(d->CreateStateBlock(D3DSBT_ALL, &g_wsSB))) { g_wsSB = nullptr; return false; }
    return true;
}
void wsReleaseDeviceObjects() { if (g_wsSB) { g_wsSB->Release(); g_wsSB = nullptr; } wsReleaseScene(); }   // the textures are managed: they survive Reset

bool wsOnBackbuffer(IDirect3DDevice9* d) {                             // skip draws that belong to render-to-texture passes
    IDirect3DSurface9 *rt = nullptr, *bb = nullptr; bool same = false;
    if (SUCCEEDED(d->GetRenderTarget(0, &rt)) && rt) {
        if (SUCCEEDED(d->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb)) && bb) { same = (rt == bb); bb->Release(); }
        rt->Release();
    }
    return same;
}

// Copy of the frame so far (half resolution, or full size when the back buffer is multisampled), sampled by the refractive drops.
bool wsCopyScene(IDirect3DDevice9* d, const D3DVIEWPORT9& vp, bool needBlur) {
    if (g_wsRefrFailed) return false;
    IDirect3DSurface9* rt = nullptr;
    if (FAILED(d->GetRenderTarget(0, &rt)) || !rt) return false;
    D3DSURFACE_DESC desc{}; rt->GetDesc(&desc);
    const bool msaa = desc.MultiSampleType != D3DMULTISAMPLE_NONE;
    const UINT tw = msaa ? (UINT)vp.Width : std::max<UINT>(64u, (UINT)(vp.Width / 2)), th = msaa ? (UINT)vp.Height : std::max<UINT>(64u, (UINT)(vp.Height / 2));
    if (!g_wsSceneTex || g_wsSceneW != tw || g_wsSceneH != th || g_wsSceneFmt != (int)desc.Format) {
        wsReleaseScene();
        if (FAILED(d->CreateTexture(tw, th, 1, D3DUSAGE_RENDERTARGET, desc.Format, D3DPOOL_DEFAULT, &g_wsSceneTex, nullptr)) || !g_wsSceneTex ||
            FAILED(g_wsSceneTex->GetSurfaceLevel(0, &g_wsSceneSurf)) || !g_wsSceneSurf) {
            wsReleaseScene(); rt->Release(); g_wsRefrFailed = true; Note("WindscreenFX: refraction disabled (cannot create the scene copy, format %d)", (int)desc.Format); return false;
        }
        g_wsSceneW = tw; g_wsSceneH = th; g_wsSceneFmt = (int)desc.Format;
    }
    RECT src = { (LONG)vp.X, (LONG)vp.Y, (LONG)(vp.X + vp.Width), (LONG)(vp.Y + vp.Height) };
    const HRESULT hr = d->StretchRect(rt, &src, g_wsSceneSurf, nullptr, msaa ? D3DTEXF_NONE : D3DTEXF_LINEAR);
    rt->Release();
    if (FAILED(hr)) { g_wsRefrFailed = true; Note("WindscreenFX: refraction disabled (StretchRect failed 0x%08X, multisampled=%d)", (unsigned)hr, msaa ? 1 : 0); return false; }
    if (needBlur && !g_wsVeilFailed) {                                 // two more halvings: the veil is a heavily blurred copy of the same frame
        if (!g_wsBlurTex[1]) {
            UINT bw = tw, bh = th; bool ok = true;
            for (int i = 0; i < 2 && ok; ++i) {
                bw = std::max(16u, bw / 2); bh = std::max(16u, bh / 2);
                ok = SUCCEEDED(d->CreateTexture(bw, bh, 1, D3DUSAGE_RENDERTARGET, desc.Format, D3DPOOL_DEFAULT, &g_wsBlurTex[i], nullptr)) && g_wsBlurTex[i] &&
                     SUCCEEDED(g_wsBlurTex[i]->GetSurfaceLevel(0, &g_wsBlurSurf[i])) && g_wsBlurSurf[i];
            }
            if (!ok) { for (int i = 0; i < 2; ++i) { if (g_wsBlurSurf[i]) { g_wsBlurSurf[i]->Release(); g_wsBlurSurf[i] = nullptr; } if (g_wsBlurTex[i]) { g_wsBlurTex[i]->Release(); g_wsBlurTex[i] = nullptr; } }
                       g_wsVeilFailed = true; Note("WindscreenFX: veil disabled (cannot create the blur copies)"); return true; }
        }
        if (FAILED(d->StretchRect(g_wsSceneSurf, nullptr, g_wsBlurSurf[0], nullptr, D3DTEXF_LINEAR)) || FAILED(d->StretchRect(g_wsBlurSurf[0], nullptr, g_wsBlurSurf[1], nullptr, D3DTEXF_LINEAR)))
        { g_wsVeilFailed = true; Note("WindscreenFX: veil disabled (StretchRect for the blur failed)"); }
    }
    return true;
}

// depth (0..1 z-buffer value) of the glass plane, from the scene projection: z = s*c33 + c43/d for a point d metres in front of the eye
float wsGlassZ() {
    float s = g_wsPzs, c33 = g_wsPc33, c43 = g_wsPc43;
    if (std::fabs(s) < 0.5f) {
        if (!g_pValid || std::fabs(g_P.m[3][2]) < 0.5f) return -1.f;
        s = g_P.m[3][2]; c33 = g_P.m[2][2]; c43 = g_P.m[2][3];
    }
    return clamp01(s * c33 + c43 / std::max(0.05f, g_cfg.wsGlassDist));
}

// ---- simulation ----
void wsInsert(const WsDrop& n, int cap) {
    int count = 0, oi = -1; float oa = -1.f;
    for (int i = 0; i < (int)g_wsDrops.size(); ++i) {
        const WsDrop& o = g_wsDrops[i]; if (o.kind != n.kind) continue;
        ++count;
        if (o.age > oa && std::fabs(o.vx) + std::fabs(o.vy) < 0.02f) { oa = o.age; oi = i; }
    }
    if (count >= cap) { if (oi >= 0) g_wsDrops[oi] = n; }              // glass is full: the oldest standing one gives way
    else g_wsDrops.push_back(n);
}
void wsAddWater(float x, float y, float r, float age, int cap) {
    WsDrop d{}; d.kind = 0; d.x = x; d.y = y; d.r = r; d.age = age; d.tx = x; d.ty = y; d.tr = r; d.rot = wsRange(0.f, 6.2832f);   // rot = meander phase, and the pool orientation
    d.asp = wsRange(0.8f, 1.3f); d.var = (int)(wsRnd() * 4.f) & 3;
    const float rn = r / std::max(0.2f, g_cfg.wsDropSize);                                        // grip: small beads cling, medium ones let go sooner, flat pools cling again
    d.thr = (rn < 0.0055f ? 0.52f : rn < 0.0115f ? 0.52f - (rn - 0.0055f) / 0.006f * 0.22f : 0.55f) + wsRange(-0.04f, 0.04f);
    wsInsert(d, cap);
}
void wsAddMicro(float x, float y, float r, int cap) {                    // tiny droplets: they stick, never run, and are cleaned by wipers and by bigger drops running over them
    WsDrop d{}; d.kind = 2; d.x = x; d.y = y; d.r = r; d.age = 1.f; d.melt0 = d.melt = wsRange(70.f, 160.f);
    wsInsert(d, cap);
}
void wsAddFlake(float x, float y, float r, int cap) {
    WsDrop d{}; d.kind = 1; d.x = x; d.y = y; d.r = r; d.age = 0.f; d.tx = x; d.ty = y; d.tr = r; d.rot = wsRange(0.f, 6.2832f);
    d.thr = wsRange(0.9f, 1.3f);                                       // snow sticks much harder than water
    d.melt0 = d.melt = std::max(1.f, g_cfg.wsSnowMelt * wsRange(0.7f, 1.3f));
    wsInsert(d, cap);
}
// Particles in flight (see WsFly). wsFlyPos gives the position and the size scale (0..1) at progress p.
float wsPersp(float p, float zFar) { const float z = zFar + (1.f - zFar) * p; return (1.f / z - 1.f / zFar) / (1.f - 1.f / zFar); }          // 0 far .. 1 at the glass; accelerates
void wsFlyPos(const WsFly& f, float p, float& x, float& y, float& sc) {
    p = clamp01(p);
    if (f.mode == 1) {                                                 // thrown up from below: quick, slowing, growing
        const float e = 1.f - (1.f - p) * (1.f - p), bend = 4.f * e * (1.f - e) * f.arc;
        x = f.x0 + (f.x1 - f.x0) * e + bend; y = f.y0 + (f.y1 - f.y0) * e; sc = 0.30f + 0.70f * e;
    } else {                                                           // perspective approach from far away: slow at first, fast at the end
        const float zFar = f.kind == 1 ? 7.f : 12.f, e = wsPersp(p, zFar);
        x = f.x0 + (f.x1 - f.x0) * e; y = f.y0 + (f.y1 - f.y0) * e; sc = 1.f / (zFar + (1.f - zFar) * p);
    }
}
void wsMudLand(const WsFly& f) {                                       // a clod (or spray droplet) hits the glass: it flattens into a splat, satellites fly out of it
    WsMud m{}; m.x = f.x1; m.y = f.y1; m.r = f.r; m.rot = wsRange(0.f, 6.2832f); m.fade = 1.f; m.stretch = 1.f; m.variant = f.variant; m.tint = f.tint; m.shade = f.shade; m.dur = 0.f;
    g_wsMud.push_back(m);
    if (f.kind == 3) { wsGridAdd(g_wsDirt, f.x1, f.y1, 2.2f, 0.10f); return; }
    const int n = std::min(14, 2 + (int)(f.r * 170.f));
    if (f.r > 0.014f) for (int i = 0; i < n; ++i) {
        const float a = wsRange(0.f, 6.2832f), dist = f.r * wsRange(1.5f, 3.4f);
        WsMud s{}; s.ox = f.x1; s.oy = f.y1; s.x = f.x1 + std::cos(a) * dist / g_wsAspect; s.y = f.y1 + std::sin(a) * dist;
        s.r = f.r * wsRange(0.10f, 0.30f); s.rot = wsRange(0.f, 6.2832f); s.fade = 1.f; s.stretch = 1.f; s.shade = f.shade * wsRange(0.9f, 1.1f);
        s.variant = (wsRnd() < 0.5f) ? kMudSpray : kMudSpray2; s.tint = f.tint; s.dur = wsRange(0.05f, 0.12f); g_wsMud.push_back(s);
    }
    wsGridAdd(g_wsDirt, f.x1, f.y1, 2.5f + f.r * 110.f, 0.35f + f.r * 12.f);        // the glass behind and around it gets hazy
}
void wsMudThrow(float power, int tint, bool spray) {                   // a burst of clods (or fine dirty spray) heads for the glass: off your own wheels, or thrown by a car ahead
    const float sz = g_cfg.wsMudSize;
    const bool own = wsRnd() < 0.6f;
    float cx, cy, ox, oy;
    if (own) { const float side = wsRnd() < 0.5f ? 0.27f : 0.73f; cx = std::max(0.05f, std::min(0.95f, side + wsRange(-0.22f, 0.22f))); cy = wsRange(0.42f, 0.84f); ox = side + wsRange(-0.05f, 0.05f); oy = 1.03f; }
    else     { cx = wsRange(0.12f, 0.88f); cy = wsRange(0.22f, 0.75f); ox = 0.5f + wsRange(-0.08f, 0.08f); oy = 0.42f + wsRange(-0.04f, 0.04f); }
    const int n = spray ? 7 + (int)(wsRnd() * 8.f * power) : 3 + (int)(wsRnd() * 5.f * power);
    const float baseDur = own ? 0.30f : 0.45f, spread = own ? 0.11f : 0.15f;
    for (int i = 0; i < n; ++i) {
        const bool lead = !spray && i == 0;
        WsFly f{}; f.kind = spray ? 3 : 1; f.mode = own ? 1 : 0; f.tint = tint; f.shade = wsRange(0.85f, 1.15f);
        f.r = (spray ? wsRange(0.0035f, 0.010f) : lead ? wsRange(0.032f, 0.062f) * std::sqrt(power) : wsRange(0.007f, 0.022f)) * sz;
        f.variant = lead ? ((int)(wsRnd() * 4.f) & 3) : spray ? kMudSpeckle : ((wsRnd() < 0.5f) ? kMudSpray : kMudSpray2);
        if (!lead && !spray && f.r > 0.016f) f.variant = (int)(wsRnd() * 4.f) & 3;                     // mid-size clods splat too
        f.x1 = std::max(0.02f, std::min(0.98f, cx + (wsRange(-1.f, 1.f) + wsRange(-1.f, 1.f)) * 0.5f * spread / g_wsAspect * 1.6f));
        f.y1 = std::max(0.05f, std::min(0.90f, cy + (wsRange(-1.f, 1.f) + wsRange(-1.f, 1.f)) * 0.5f * spread));
        f.x0 = ox + wsRange(-0.03f, 0.03f); f.y0 = oy + wsRange(-0.02f, 0.02f); f.arc = wsRange(-0.06f, 0.06f);
        f.dur = baseDur * wsRange(0.8f, 1.25f); f.t = -wsRange(0.f, 0.20f);
        g_wsFly.push_back(f);
    }
}
void wsFlyUpdate(float dt) {                                           // advance everything in flight, land what has arrived
    for (WsFly& f : g_wsFly) {
        f.t += dt / std::max(0.05f, f.dur);
        if (f.t < 1.f) continue;
        switch (f.kind) {
            case 0: wsAddWater(f.x1, f.y1, f.r, 0.f, g_cfg.wsDrops); wsGridAdd(g_wsFilm, f.x1, f.y1, 3.5f, f.r < 0.0055f ? 1.2f : 2.0f); break;
            case 2: wsAddFlake(f.x1, f.y1, f.r, g_cfg.wsSnowMax); wsGridAdd(g_wsFilm, f.x1, f.y1, 4.f, 1.0f); break;
            default: wsMudLand(f); break;
        }
        f.kind = -1;
    }
    g_wsFly.erase(std::remove_if(g_wsFly.begin(), g_wsFly.end(), [](const WsFly& f) { return f.kind < 0; }), g_wsFly.end());
    if (g_wsFly.size() > 3000) g_wsFly.erase(g_wsFly.begin(), g_wsFly.begin() + (g_wsFly.size() - 3000));
}

void wsSync() {                                                        // match the particle sets to the INI
    const bool wantD = (g_cfg.wsMode & 1) != 0, wantM = (g_cfg.wsMode & 2) != 0;
    if (g_wsReset) {                                                   // new stage: clean glass, rain and mud arrive from here on
        g_wsDrops.clear(); g_wsMud.clear(); g_wsTrails.clear(); g_wsFly.clear(); std::fill(g_wsFilm, g_wsFilm + kGX * kGY, 0.f); std::fill(g_wsDirt, g_wsDirt + kGX * kGY, 0.f);
        g_wsBow[0] = g_wsBow[1] = 0.f; g_wsRainAcc = g_wsSnowAcc = g_wsMudAcc = g_wsMicroAcc = 0.f; g_wsSprayAcc = 0.f; g_wsReset = false; g_wsWipeT0 = GetTickCount(); wsHuntReset();
        g_wsRng = (g_wsRng ^ (uint32_t)GetTickCount()) | 1u;
    }
    if (!wantD) { g_wsDrops.clear(); g_wsTrails.clear(); std::fill(g_wsFilm, g_wsFilm + kGX * kGY, 0.f); }
    if (!wantM) { g_wsMud.clear(); std::fill(g_wsDirt, g_wsDirt + kGX * kGY, 0.f); }
}

// Wipers (emulated): the game's wiper pose is not exposed anywhere known, so two blades sweep on a timed cycle that you line up with the real ones
// (geometry in the ini, Ctrl+F7 to sync the timing, WiperDebug=1 to see them). Pose = pivot + length + angle range in glass space.
struct WsBlade { float px, py, len, a0, a1; };
void wsBlades(WsBlade (&b)[2]) {
    b[0] = {g_cfg.wsW1X, g_cfg.wsW1Y, g_cfg.wsW1Len, g_cfg.wsW1A0, g_cfg.wsW1A1};
    b[1] = {g_cfg.wsW2X, g_cfg.wsW2Y, g_cfg.wsW2Len, g_cfg.wsW2A0, g_cfg.wsW2A1};
    for (int i = 0; i < 2; ++i)                                        // WiperSource=1 and the hunter has found the real blade: its pose replaces the ini geometry
        if (g_cfg.wsWiperSource && g_wsModelBlade[i].ok) { b[i].px = g_wsModelBlade[i].px; b[i].py = g_wsModelBlade[i].py; b[i].len = g_wsModelBlade[i].len; }
}
void wsWiperUpdate() {
    WsBlade bl[2]; wsBlades(bl);
    const bool wet = g_wsRainEff > 0.02f || g_wsSnowEff > 0.02f;
    const bool active = g_cfg.wsWipers && (!g_cfg.wsWiperAuto || wet);
    float w = 0.f;
    if (active) {
        const float period = std::max(0.3f, g_cfg.wsWiperPeriod), cyc = period + std::max(0.f, g_cfg.wsWiperPause);
        const float t = std::fmod((float)(GetTickCount() - g_wsWipeT0) / 1000.f, cyc);
        if (t < period) w = 0.5f - 0.5f * std::cos(6.2832f * t / period);       // up and back, smooth at both ends
    }
    g_wsWiperW = w;
    for (int i = 0; i < 2; ++i) {
        float ang = (bl[i].a0 + (bl[i].a1 - bl[i].a0) * w) * 0.0174533f;
        if (g_cfg.wsWiperSource && g_wsModelBlade[i].ok) {            // the real blade: follow its angle, kept continuous across +-180 degrees
            const float prev = g_wsBladeInit ? g_wsBladeAng[i] : g_wsModelBlade[i].ang, d = g_wsModelBlade[i].ang - prev;
            ang = prev + std::atan2(std::sin(d), std::cos(d));
        }
        g_wsBladePrev[i] = g_wsBladeInit ? g_wsBladeAng[i] : ang; g_wsBladeAng[i] = ang;
    }
    g_wsBladeInit = true;
}
void wsWiperGhost(float dt) {                                          // lagging copies of each blade angle: the motion-blur wedge is drawn between them and the blade
    for (int i = 0; i < 2; ++i) {
        const float a = g_wsBladeAng[i];
        for (int k = 0; k < 2; ++k) {
            const float tau = k == 0 ? 0.035f : 0.080f; float& g = g_wsGhost[i][k];
            if (std::fabs(g - a) > 1.0f) g = a; else g += (a - g) * (1.f - std::exp(-dt / tau));
        }
    }
}
void wsSyncWipers() { g_wsWipeT0 = GetTickCount(); g_wsBladeInit = false; Logf("wiper cycle synced to now (Ctrl+F7): the real wipers should be starting their upward stroke"); }

void wsWipe(float W, float H) {
    if (!g_cfg.wsWipers) return;
    WsBlade bl[2]; wsBlades(bl);
    std::vector<WsDrop> spawn; std::vector<WsMud> smears;
    const float life = std::max(0.5f, g_cfg.wsTrail);
    for (int bi = 0; bi < 2; ++bi) {
        if (bl[bi].len <= 0.f) continue;
        const float cur = g_wsBladeAng[bi], prev = g_wsBladePrev[bi], dA = cur - prev;
        if (std::fabs(dA) < 1e-5f || std::fabs(dA) > 0.6f) continue;   // parked, or a jump (settings reloaded): no sweep this frame
        const float pxv = bl[bi].px * W, pyv = bl[bi].py * H, L = bl[bi].len * H, lo = std::min(cur, prev), hi = std::max(cur, prev);
        const float half = 3.f + 0.004f * H;                           // blade half thickness in pixels
        float mass = 0.f;                                              // water gathered by the blade this frame (feeds the ridge in front of it)
        auto swept = [&](float X, float Y, float R, float& tx, float& ty) -> bool {   // is the point (glass px) inside the sector the blade crossed this frame?
            const float dx = X - pxv, dyUp = pyv - Y, r = std::sqrt(dx * dx + dyUp * dyUp);
            if (r > L + R || r < 0.24f * L - R) return false;          // the rubber only covers the outer part of the arm
            float a = std::atan2(dyUp, dx);
            while (a < lo - 3.14159f) a += 6.28318f;
            while (a > lo + 3.14159f) a -= 6.28318f;
            const float margin = std::atan2(R + half, std::max(r, 1.f));
            if (a < lo - margin || a > hi + margin) return false;
            const float s = dA > 0.f ? 1.f : -1.f; tx = s * (-dyUp); ty = s * (-dx);       // direction the blade moves there (screen px)
            return true;
        };
        for (WsDrop& d : g_wsDrops) {
            if (d.dead) continue;
            float tx, ty; if (!swept(d.x * W, d.y * H, d.r * H, tx, ty)) continue;
            d.dead = true; g_wsWiped++; mass += (d.kind == 2 ? 0.12f : 1.f) * std::min(1.f, d.r * 90.f);
            if (d.kind == 0 && d.r > 0.005f && wsRnd() < 0.35f) {      // a few tiny beads are left behind
                WsDrop n{}; n.kind = 0; n.x = d.x; n.y = d.y; n.r = wsRange(0.0016f, 0.0028f) * g_cfg.wsDropSize; n.age = 2.f; n.tx = n.x; n.ty = n.y; n.tr = n.r; n.thr = 0.55f;
                spawn.push_back(n);
            }
        }
        for (WsTrail& t : g_wsTrails) { float tx, ty; if (swept((t.x0 + t.x1) * 0.5f * W, (t.y0 + t.y1) * 0.5f * H, t.w0 * H, tx, ty)) { t.age = life + 1.f; g_wsWiped++; } }
        for (WsMud& m : g_wsMud) {
            if (m.age < 0.f || m.fade <= 0.f) continue;
            float tx, ty; if (!swept(m.x * W, m.y * H, m.r * H * m.stretch, tx, ty)) continue;
            m.fade *= 0.55f; m.r *= 0.94f; g_wsWiped++; mass += 0.4f;                // mostly cleared, the rest is dragged along the stroke
            if (m.variant != kMudSmear && !wsIsSprayV(m.variant) && m.r > 0.008f && m.fade > 0.15f && g_cfg.wsMudSmear) {
                WsMud s{}; s.x = m.x; s.y = m.y; s.r = m.r * 0.9f; s.variant = kMudSmear; s.stretch = 2.6f; s.tint = m.tint; s.fade = 0.85f * m.fade; s.age = 0.f; s.shade = m.shade;
                s.rot = std::atan2(-tx, ty); smears.push_back(s);
            }
        }
        const float cw = W / (float)kGX, ch = H / (float)kGY, cr = 0.75f * std::max(cw, ch);   // the haze grids: water is squeegeed away, dirt is mostly smeared (some stays)
        for (int j = 0; j < kGY; ++j) for (int i = 0; i < kGX; ++i) {
            float tx, ty; if (!swept(((float)i + 0.5f) * cw, ((float)j + 0.5f) * ch, cr, tx, ty)) continue;
            float& f = g_wsFilm[j * kGX + i]; float& dt = g_wsDirt[j * kGX + i];
            mass += f * 0.05f; f *= 0.03f; dt *= 0.62f;
        }
        if (g_cfg.wsWiperBow) g_wsBow[bi] = std::min(1.5f, g_wsBow[bi] + mass * 0.045f);
    }
    for (const WsDrop& n : spawn) wsInsert(n, g_cfg.wsDrops);
    for (const WsMud& s : smears) g_wsMud.push_back(s);
}

void wsStep(float dt, float W, float H) {
    const float aspect = H / W; g_wsAspect = W / H;
    // telemetry -> glass conditions: speed (km/h) is the airflow, the high-passed heave is road roughness (loose surface), sway = cornering
    const float spd = g_in.speed.load();
    const float spdN = clamp01(spd / std::max(10.f, g_cfg.wsSpeedRef));
    g_wsRough += (std::fabs(g_in.heaveHp.load()) - g_wsRough) * std::min(1.f, dt * 1.5f);
    const float loose = clamp01(g_wsRough / std::max(0.01f, g_cfg.wsRoughRef));
    g_wsSpdN = spdN; g_wsLoose = loose;
    const float wind = spdN * g_cfg.wsWind;
    static DWORD lastGW = 0;
    if (GetTickCount() - lastGW > 250) { lastGW = GetTickCount(); g_wsGW = wsReadGameWeather(); }
    const bool useGame = g_cfg.wsRainSource && g_wsGW.ok;
    if (g_wsRuleRain >= 0.f) g_wsRainEff = g_wsRuleRain;                       // explicit override for this stage
    else if (useGame) g_wsRainEff = (g_wsGW.sky == 3) ? g_cfg.wsRainLight : (g_wsGW.sky == 4) ? g_cfg.wsRainHeavy : 0.f;   // the game's own sky type
    else g_wsRainEff = g_cfg.wsRainLevel;
    g_wsSnowEff = useGame ? ((g_wsGW.sky == 6) ? g_cfg.wsSnowLight : (g_wsGW.sky == 7) ? g_cfg.wsSnowHeavy : 0.f) : g_cfg.wsSnowLevel;
    const bool snowy = g_wsSnowEff > 0.01f;
    const float wetMul = useGame ? (snowy ? g_cfg.wsMudSnow : g_wsGW.wet == 0 ? g_cfg.wsMudDry : g_wsGW.wet == 1 ? g_cfg.wsMudDamp : g_cfg.wsMudWet) : 1.f;   // mud needs a wet road
    if (g_cfg.wsWetFromGame) {                                         // experiment: is the game drawing its own wet overlay right now?
        const bool wetNow = (GetTickCount() - g_wsGameTick.load()) < 300;
        g_wsWet += ((wetNow ? 1.f : 0.f) - g_wsWet) * std::min(1.f, dt * 0.5f);
    } else g_wsWet = 1.f;
    wsHuntPose(W, H);
    wsWiperUpdate();

    // ---- arrivals ----
    if ((g_cfg.wsMode & 1) && g_cfg.wsDrops > 0) {                     // rain impacts, more of them the faster you drive
        const float rainRate = g_cfg.wsRain * g_wsRainEff * (0.2f + 0.8f * std::pow(spdN, 0.8f)) * g_wsWet;
        g_wsDbgRain = rainRate; g_wsRainAcc += rainRate * dt;
        for (int guard = 0; g_wsRainAcc >= 1.f && guard < 50; ++guard) {
            g_wsRainAcc -= 1.f;
            const float u = wsRnd();                                                         // many small beads, some medium, a few big flat pools
            const float pl = 0.06f + 0.16f * g_wsRainEff;                                  // heavier rain: more of the big flat pools
            float r = u < 0.55f ? wsRange(0.0016f, 0.0055f) : u < 1.f - pl ? wsRange(0.0055f, 0.0115f) : wsRange(0.0115f, 0.030f);
            const float tx = wsRange(0.03f, 0.97f), ty = 0.06f + 0.74f * std::pow(wsRnd(), 0.85f), rr = r * g_cfg.wsDropSize;
            if (g_cfg.wsFly && rr >= 0.0032f) {                                            // it comes from somewhere: from above when you are slow, from the vanishing point when you are fast
                WsFly f{}; f.kind = 0; f.mode = 0; f.r = rr; f.x1 = tx; f.y1 = ty; f.shade = 1.f;
                const float fast = clamp01(spdN * 1.4f);
                f.x0 = tx * (1.f - fast) + (0.5f + wsRange(-0.04f, 0.04f)) * fast; f.y0 = (-0.25f) * (1.f - fast) + 0.40f * fast;
                f.dur = g_cfg.wsFlyTime * wsRange(0.7f, 1.3f) * (1.15f - 0.4f * fast); f.t = -wsRange(0.f, 0.6f / std::max(1.f, rainRate) + 0.02f);
                g_wsFly.push_back(f);
            } else { wsAddWater(tx, ty, rr, 0.f, g_cfg.wsDrops); wsGridAdd(g_wsFilm, tx, ty, 3.f, 1.0f); }
        }
        if (g_cfg.wsMicro > 0) {                                       // the fine speckle of tiny droplets that makes wet glass look wet
            g_wsMicroAcc += g_cfg.wsMicroRate * g_wsRainEff * (0.5f + 0.5f * spdN) * g_wsWet * dt;
            for (int guard = 0; g_wsMicroAcc >= 1.f && guard < 80; ++guard) {
                g_wsMicroAcc -= 1.f;
                const float mx = wsRange(0.02f, 0.98f), my = wsRange(0.04f, 0.86f);
                wsAddMicro(mx, my, wsRange(0.0006f, 0.0020f) * g_cfg.wsDropSize, g_cfg.wsMicro); wsGridAdd(g_wsFilm, mx, my, 3.f, 1.0f);
            }
        }
    }
    if ((g_cfg.wsMode & 1) && snowy && g_cfg.wsSnowMax > 0) {          // snow flakes landing
        g_wsSnowAcc += g_cfg.wsSnowRate * g_wsSnowEff * (0.3f + 0.7f * std::pow(spdN, 0.7f)) * dt;
        for (int guard = 0; g_wsSnowAcc >= 1.f && guard < 30; ++guard) {
            g_wsSnowAcc -= 1.f;
            const float fx = wsRange(0.03f, 0.97f), fy = 0.06f + 0.74f * std::pow(wsRnd(), 0.85f), fr = (0.006f + 0.020f * wsRnd() * wsRnd()) * g_cfg.wsSnowSize;
            if (g_cfg.wsFly) {
                WsFly f{}; f.kind = 2; f.mode = 0; f.r = fr; f.x1 = fx; f.y1 = fy; f.shade = 1.f; f.x0 = 0.5f + wsRange(-0.05f, 0.05f) + (fx - 0.5f) * 0.1f; f.y0 = 0.38f + wsRange(-0.04f, 0.04f);
                f.dur = g_cfg.wsFlyTime * wsRange(1.1f, 2.0f); f.t = -wsRange(0.f, 0.3f); g_wsFly.push_back(f);
            } else wsAddFlake(fx, fy, fr, g_cfg.wsSnowMax);
        }
    }
    const float splash = g_wsSplash.exchange(0.f);
    if (g_cfg.wsMode & 2) {
        const float mudRate = g_cfg.wsMudRate * g_wsMudMul * wetMul / 60.f * spdN * std::pow(loose, 1.5f) * (1.f + 0.5f * std::min(1.f, std::fabs(g_in.sway.load())));   // speed on a rough, wet surface, more when cornering
        g_wsDbgMud = mudRate * 60.f; g_wsMudAcc += mudRate * dt;
        const int tint = snowy ? 1 : 0;                                // thrown snow is grey slush
        for (int guard = 0; g_wsMudAcc >= 1.f && guard < 8; ++guard) { g_wsMudAcc -= 1.f; wsMudThrow(1.f, tint, false); }
        if (splash > 0.05f) { wsMudThrow(std::min(1.8f, 0.6f + splash * 0.5f), tint, false); if (splash > 0.6f) wsMudThrow(1.f, tint, true); }   // landing / impact
        const float wetSpray = useGame ? (g_wsGW.wet == 2 ? 1.f : g_wsGW.wet == 1 ? 0.4f : 0.f) : 0.f;           // dirty water thrown up off a wet road
        const float sprayRate = g_cfg.wsSpray * std::max(wetSpray, g_wsRainEff * 0.7f) * g_wsMudMul * 0.9f * std::pow(spdN, 1.5f) * (0.4f + 0.6f * std::min(1.f, loose + 0.3f));
        g_wsSprayAcc += sprayRate * dt;
        for (int guard = 0; g_wsSprayAcc >= 1.f && guard < 4; ++guard) { g_wsSprayAcc -= 1.f; wsMudThrow(1.f, tint, true); }
    }

    // ---- drops: stick, slide, shed water, swallow neighbours; flakes melt ----
    const float sway = g_in.sway.load() * (float)g_cfg.wsSignSway, surge = g_in.surge.load() * (float)g_cfg.wsSignSurge;
    const float Fx = -sway * g_cfg.wsGravity * 0.9f;                   // cornering: drops lag behind sideways
    const float Fy = 0.12f * g_cfg.wsGravity - wind + surge * 0.5f * g_cfg.wsGravity;   // gravity down, wind up, accel / brake along the glass
    const float F = std::sqrt(Fx * Fx + Fy * Fy);
    const float leak = 0.010f * g_cfg.wsDropSize, minR = 0.0012f * g_cfg.wsDropSize;
    std::vector<WsDrop> melted;
    for (size_t i = 0; i < g_wsDrops.size(); ++i) {
        WsDrop& d = g_wsDrops[i]; d.age += dt;
        if (d.kind == 2) { d.melt -= dt; if (d.melt <= 0.f) d.dead = true; continue; }          // micro droplet: only evaporates
        if (d.kind == 1) {                                             // snow flake
            d.melt -= dt;
            if (d.melt <= 0.f) {                                       // melted: becomes a water bead
                WsDrop n{}; n.kind = 0; n.x = d.x; n.y = d.y; n.r = d.r * 0.55f; n.age = 2.f; n.tx = n.x; n.ty = n.y; n.tr = n.r; n.thr = 0.45f + wsRange(-0.05f, 0.05f);
                n.melt0 = n.melt = wsRange(10.f, 24.f);               // slush water evaporates / runs off after a while
                melted.push_back(n); d.dead = true; continue;
            }
        }
        if (d.kind == 0 && d.melt0 > 0.f) { d.melt -= dt; if (d.melt <= 0.f) { d.dead = true; continue; } }
        const float thr = d.thr * (1.f - 0.45f * loose);               // a rough road shakes drops loose
        if (!d.sliding) { if (F > thr && d.age > 0.25f) d.sliding = true; }    // a fresh impact sticks for a moment
        else if (F < thr * 0.65f) d.sliding = false;                           // static friction is higher than kinetic: once moving it keeps going
        const bool slide = d.sliding;
        if (slide) {
            const float k = (F - 0.65f * thr) / F * 2.0f * dt;                // kinetic friction is lower than static: a drop that lets go runs
            const float wob = std::sin(d.age * 3.2f + d.rot) * 0.9f * dt;                  // slow heading drift, proportional to speed: smooth meander, no kinks
            const float ovx = d.vx, ovy = d.vy;
            d.vx += Fx * k - ovy * wob; d.vy += Fy * k + ovx * wob;
            const float drag = std::exp(-2.5f * dt); d.vx *= drag; d.vy *= drag;
        } else { const float stick = std::exp(-14.f * dt); d.vx *= stick; d.vy *= stick; }
        const float dh = std::sqrt(d.vx * d.vx + d.vy * d.vy) * dt;    // distance travelled this step, in viewport heights
        if (dh > 0.f) {
            d.x += d.vx * dt * aspect; d.y += d.vy * dt;
            if (d.kind == 0 && d.r > 0.0035f) {                         // a running drop leaves a clear track in the haze
                const int gi = (int)(d.x * (float)kGX), gj = (int)(d.y * (float)kGY);
                if (gi >= 0 && gi < kGX && gj >= 0 && gj < kGY) { g_wsFilm[gj * kGX + gi] *= 0.80f; g_wsDirt[gj * kGX + gi] *= 0.93f; }
            }
            if (d.kind == 0) {
                d.r -= leak * dh;
                const float ex = (d.x - d.tx) / aspect, ey = d.y - d.ty;
                if (std::sqrt(ex * ex + ey * ey) > 0.016f) {           // leave a trail segment
                    g_wsTrails.push_back({d.tx, d.ty, d.x, d.y, std::min(d.tr * 0.40f, 0.0025f), std::min(d.r * 0.40f, 0.0025f), 0.f});
                    d.tx = d.x; d.ty = d.y; d.tr = d.r;
                }
                if (slide && d.age > 0.4f) for (size_t j = 0; j < g_wsDrops.size(); ++j) {   // merge with whatever it runs into
                    WsDrop& o = g_wsDrops[j];
                    if (j == i || o.dead || o.age < 0.15f || (o.kind != 0 && o.kind != 2)) continue;
                    const float ox = (o.x - d.x) / aspect, oy = o.y - d.y;
                    if (std::sqrt(ox * ox + oy * oy) < (d.r + o.r) * (o.kind == 2 ? 0.95f : 0.85f)) { d.r = std::sqrt(d.r * d.r + o.r * o.r * (o.kind == 2 ? 0.25f : 0.9f)); o.dead = true; }   // leaves a clean track behind
                }
            }
        }
        if ((d.kind == 0 && d.r < minR) || d.x < -0.05f || d.x > 1.05f || d.y < -0.06f || d.y > 0.95f) d.dead = true;
    }
    wsFlyUpdate(dt);
    wsWipe(W, H); wsWiperGhost(dt);
    for (int i = 0; i < 2; ++i) g_wsBow[i] *= std::exp(-dt * 1.6f);
    {   // haze: water film runs off / evaporates slowly, rain washes dirt away a little, and a clean stretch of glass stays clean
        const float fk = std::exp(-dt * g_cfg.wsFilmDecay * (snowy ? 0.4f : 1.f)), dk = std::exp(-dt * (0.004f + 0.03f * g_wsRainEff));
        for (int i = 0; i < kGX * kGY; ++i) { g_wsFilm[i] *= fk; g_wsDirt[i] *= dk; }
    }
    g_wsDrops.erase(std::remove_if(g_wsDrops.begin(), g_wsDrops.end(), [](const WsDrop& d) { return d.dead; }), g_wsDrops.end());
    for (const WsDrop& n : melted) wsInsert(n, g_cfg.wsDrops);

    // ---- trails and mud age ----
    const float life = std::max(0.5f, g_cfg.wsTrail);
    for (WsTrail& t : g_wsTrails) t.age += dt;
    g_wsTrails.erase(std::remove_if(g_wsTrails.begin(), g_wsTrails.end(), [&](const WsTrail& t) { return t.age > life; }), g_wsTrails.end());
    if (g_wsTrails.size() > 4000) g_wsTrails.erase(g_wsTrails.begin(), g_wsTrails.begin() + (g_wsTrails.size() - 4000));
    int alive = 0, aliveSpray = 0; for (const WsMud& m : g_wsMud) if (!m.dying) { if (wsIsSprayV(m.variant)) ++aliveSpray; else ++alive; }
    const int sprayCap = g_cfg.wsMud * 3;                              // spray dots are small and numerous: their own, larger cap
    for (size_t i = 0; i < g_wsMud.size() && (alive > g_cfg.wsMud || aliveSpray > sprayCap); ++i) {   // over a cap: the oldest fade away
        WsMud& m = g_wsMud[i]; if (m.dying) continue;
        if (wsIsSprayV(m.variant)) { if (aliveSpray > sprayCap) { m.dying = true; --aliveSpray; } }
        else if (alive > g_cfg.wsMud) { m.dying = true; --alive; }
    }
    for (WsMud& m : g_wsMud) {
        m.age += dt; if (m.dying) m.fade -= dt / 2.f;
        else if (m.age > 0.3f) m.fade -= dt * (g_wsRainEff * 0.010f + (m.variant == kMudSpeckle ? 0.03f : 0.f));   // rain slowly washes it off; dirty water dries
    }
    g_wsMud.erase(std::remove_if(g_wsMud.begin(), g_wsMud.end(), [](const WsMud& m) { return m.fade <= 0.06f; }), g_wsMud.end());
}

// ---- drawing ----
bool wsDraw(IDirect3DDevice9* d) {
    if (!d || g_wsTexFailed) return false;
    if (!wsOnBackbuffer(d)) { g_wsSkipped++; return false; }
    if (!wsEnsureResources(d)) return false;
    D3DVIEWPORT9 vp{};
    if (FAILED(d->GetViewport(&vp)) || vp.Width < 64 || vp.Height < 64) return false;
    const float W = (float)vp.Width, H = (float)vp.Height; g_wsVpW = vp.Width; g_wsVpH = vp.Height;

    static LARGE_INTEGER last{}; LARGE_INTEGER now; QueryPerformanceCounter(&now);
    float dt = last.QuadPart ? (float)(now.QuadPart - last.QuadPart) / (float)g_qf.QuadPart : 0.f; last = now;
    dt = std::max(0.f, std::min(dt, 0.05f));
    wsSync(); wsStep(dt, W, H); wsHuntTick();

    const float gz = wsGlassZ();                                       // glass depth in z-buffer units, < 0 = projection not seen yet
    const bool useZ = g_cfg.wsDepthTest && gz >= 0.f;
    M4 C = m4Ident(); bool haveC = false;
    if (gz >= 0.f && g_wsPok) { C = m4Mul(g_wsP, m4Mul(g_patchActive ? g_Hinv : m4Ident(), g_wsPinv)); haveC = true; }
    static bool logged = false;
    if (!logged) { logged = true; Logf("WindscreenFX: first draw, DrawAt=%d depthTest=%d glassZ=%.5f glassLock=%d refraction=%d hideGameFx=%d viewport=%ux%u", g_cfg.wsDrawAt, useZ ? 1 : 0, gz, haveC ? 1 : 0, g_cfg.wsRefraction, g_cfg.wsHideGameFx, vp.Width, vp.Height); }

    // Glass space: every position is where that spot of the glass appears with the camera at its base pose. Each vertex goes through the same head
    // transform (P * Hinv * P^-1) the game's 3D scene gets, at the glass depth, so everything stays fixed on the car when the head moves.
    auto xf = [&](float px, float py, float& ox, float& oy, float& oz) -> bool {
        oz = 0.f;
        if (!haveC) { ox = px; oy = py; return true; }
        const float c0[4] = {px / W * 2.f - 1.f, 1.f - py / H * 2.f, gz, 1.f};
        float c1[4];
        for (int i = 0; i < 4; ++i) c1[i] = C.m[i][0] * c0[0] + C.m[i][1] * c0[1] + C.m[i][2] * c0[2] + C.m[i][3] * c0[3];
        if (c1[3] < 1e-4f) return false;                               // behind the eye
        const float iw = 1.f / c1[3];
        ox = (c1[0] * iw * 0.5f + 0.5f) * W; oy = (0.5f - c1[1] * iw * 0.5f) * H; oz = clamp01(c1[2] * iw);
        return true;
    };
    static const float uq[4] = {0, 1, 1, 0}, vq[4] = {0, 0, 1, 1};
    auto emit = [&](std::vector<WsVert>& v, const float (&px)[4], const float (&py)[4], DWORD col, float u0, float v0, float u1, float v1) {
        WsVert q[4];
        for (int i = 0; i < 4; ++i) {
            float ox, oy, oz; if (!xf(px[i], py[i], ox, oy, oz)) return;
            q[i] = { (float)vp.X + ox - 0.5f, (float)vp.Y + oy - 0.5f, useZ ? oz : 0.f, 1.f, col, u0 + (u1 - u0) * uq[i], v0 + (v1 - v0) * vq[i] };
        }
        v.push_back(q[0]); v.push_back(q[1]); v.push_back(q[2]); v.push_back(q[0]); v.push_back(q[2]); v.push_back(q[3]);
    };
    auto corners = [&](float cx, float cy, float hw, float hh, float ang, float (&px)[4], float (&py)[4]) {
        static const float sx[4] = {-1, 1, 1, -1}, sy[4] = {-1, -1, 1, 1};
        const float c = std::cos(ang), s = std::sin(ang);
        for (int i = 0; i < 4; ++i) { const float lx = sx[i] * hw, ly = sy[i] * hh; px[i] = cx + lx * c - ly * s; py[i] = cy + lx * s + ly * c; }
    };
    auto quad = [&](std::vector<WsVert>& v, float cx, float cy, float hw, float hh, float ang, DWORD col, float u0, float v0, float u1, float v1) {
        float px[4], py[4]; corners(cx, cy, hw, hh, ang, px, py); emit(v, px, py, col, u0, v0, u1, v1);
    };
    auto strip = [&](std::vector<WsVert>& v, float x0, float y0, float x1, float y1, float w0, float w1, DWORD col) {   // trapezoid from p0 to p1, half widths in pixels
        const float dx = x1 - x0, dy = y1 - y0, len = std::sqrt(dx * dx + dy * dy); if (len < 0.01f) return;
        const float nx = -dy / len, ny = dx / len;
        const float px[4] = {x0 - nx * w0, x0 + nx * w0, x1 + nx * w1, x1 - nx * w1}, py[4] = {y0 - ny * w0, y0 + ny * w0, y1 + ny * w1, y1 - ny * w1};
        emit(v, px, py, col, 0.f, 0.f, 1.f, 1.f);
    };
    const float refK = g_cfg.wsRefractK;
    auto emitRefr = [&](std::vector<WsVert2>& out, float cx, float cy, float hw, float hh, float ang, DWORD col, float mu0, float mv0, float mu1, float mv1, float K) {   // refractive drop: scene sampled inverted and magnified
        float px[4], py[4], ox[4], oy[4], oz[4]; corners(cx, cy, hw, hh, ang, px, py);
        for (int i = 0; i < 4; ++i) if (!xf(px[i], py[i], ox[i], oy[i], oz[i])) return;
        const float mx = (ox[0] + ox[1] + ox[2] + ox[3]) * 0.25f, my = (oy[0] + oy[1] + oy[2] + oy[3]) * 0.25f;
        WsVert2 q[4];
        for (int i = 0; i < 4; ++i)
            q[i] = { (float)vp.X + ox[i] - 0.5f, (float)vp.Y + oy[i] - 0.5f, useZ ? oz[i] : 0.f, 1.f, col,
                     (mx + (mx - ox[i]) * K) / W, (my + (my - oy[i]) * K) / H, mu0 + (mu1 - mu0) * uq[i], mv0 + (mv1 - mv0) * vq[i] };
        out.push_back(q[0]); out.push_back(q[1]); out.push_back(q[2]); out.push_back(q[0]); out.push_back(q[2]); out.push_back(q[3]);
    };
    auto emitFrost = [&](std::vector<WsVert2>& out, float cx, float cy, float hw, float hh, float ang, DWORD col, float mu0, float mv0, float mu1, float mv1) {   // blurred view, masked by a texture
        float px[4], py[4], ox[4], oy[4], oz[4]; corners(cx, cy, hw, hh, ang, px, py);
        for (int i = 0; i < 4; ++i) if (!xf(px[i], py[i], ox[i], oy[i], oz[i])) return;
        WsVert2 q[4];
        for (int i = 0; i < 4; ++i) q[i] = { (float)vp.X + ox[i] - 0.5f, (float)vp.Y + oy[i] - 0.5f, useZ ? oz[i] : 0.f, 1.f, col, ox[i] / W, oy[i] / H, mu0 + (mu1 - mu0) * uq[i], mv0 + (mv1 - mv0) * vq[i] };
        out.push_back(q[0]); out.push_back(q[1]); out.push_back(q[2]); out.push_back(q[0]); out.push_back(q[2]); out.push_back(q[3]);
    };
    g_wsVSprite.clear(); g_wsVShade.clear(); g_wsVShadePool.clear(); g_wsVMud.clear(); g_wsVTrail.clear(); g_wsVFlake.clear(); g_wsVSplash.clear(); g_wsVDbg.clear(); g_wsVMicro.clear(); g_wsVVeil.clear(); g_wsVRefr.clear(); g_wsVRefrPool.clear(); g_wsVFrost.clear(); g_wsVDirt.clear(); g_wsVGlow.clear(); g_wsVFlyMud.clear(); g_wsVFlyFlake.clear(); g_wsVStreak.clear(); g_wsVWBlur.clear();
    const float op = clamp01(g_cfg.wsOpacity);
    auto rgba = [](float r, float g, float b, float a) { return D3DCOLOR_ARGB((int)(255.f * clamp01(a)), (int)(255.f * clamp01(r)), (int)(255.f * clamp01(g)), (int)(255.f * clamp01(b))); };
    const float life = std::max(0.5f, g_cfg.wsTrail);


    bool anyRefr = false;
    for (const WsDrop& dr : g_wsDrops) if (dr.kind == 0 && dr.r * H >= g_cfg.wsRefractMinPx) { anyRefr = true; break; }
    const float filmMax = wsGridMax(g_wsFilm), dirtMax = wsGridMax(g_wsDirt);
    bool anyFrost = false; for (const WsMud& m : g_wsMud) if (m.age >= 0.f && !wsIsSprayV(m.variant) && m.r * H > 9.f) { anyFrost = true; break; }
    const bool veilWanted = !g_wsVeilFailed && ((g_cfg.wsVeil > 0.001f && filmMax > 0.01f) || (g_cfg.wsDirt > 0.001f && (dirtMax > 0.01f || anyFrost)));
    const bool needScene = (g_cfg.wsRefraction && !g_wsRefrFailed && anyRefr) || veilWanted;
    const bool sceneOK = needScene && wsCopyScene(d, vp, veilWanted);
    const bool useRefr = sceneOK && g_cfg.wsRefraction && anyRefr;
    const bool useVeil = sceneOK && veilWanted && g_wsBlurTex[1] && !g_wsVeilFailed;
    g_wsRefrUsed = useRefr; g_wsVeilUsed = useVeil;
    const bool snowyNow = g_wsSnowEff > 0.01f;
    auto buildHaze = [&](const float* grid, std::vector<WsVert>& out, float maxA, float gamma, float cr, float cg, float cb, bool additive) {   // the grid as a smooth sheet of blurred view
        static float va[(kGX + 1) * (kGY + 1)];
        for (int gy = 0; gy <= kGY; ++gy) for (int gx = 0; gx <= kGX; ++gx) {
            float sum = 0.f; int n = 0;
            for (int dy = -1; dy <= 0; ++dy) for (int dx = -1; dx <= 0; ++dx) { const int cx = gx + dx, cy = gy + dy; if (cx >= 0 && cx < kGX && cy >= 0 && cy < kGY) { sum += grid[cy * kGX + cx]; ++n; } }
            va[gy * (kGX + 1) + gx] = n ? std::pow(clamp01(sum / (float)n), gamma) : 0.f;
        }
        for (int gy = 0; gy < kGY; ++gy) for (int gx = 0; gx < kGX; ++gx) {
            const float al[4] = {va[gy * (kGX + 1) + gx], va[gy * (kGX + 1) + gx + 1], va[(gy + 1) * (kGX + 1) + gx + 1], va[(gy + 1) * (kGX + 1) + gx]};
            if (std::max(std::max(al[0], al[1]), std::max(al[2], al[3])) < 0.006f) continue;
            const float gxs[4] = {(float)gx, (float)gx + 1.f, (float)gx + 1.f, (float)gx}, gys[4] = {(float)gy, (float)gy, (float)gy + 1.f, (float)gy + 1.f};
            WsVert q[4]; bool ok = true;
            for (int i = 0; i < 4 && ok; ++i) {
                float ox, oy, oz; ok = xf(gxs[i] / (float)kGX * W, gys[i] / (float)kGY * H, ox, oy, oz);
                const float k = al[i] * maxA;
                q[i] = { (float)vp.X + ox - 0.5f, (float)vp.Y + oy - 0.5f, useZ ? oz : 0.f, 1.f, additive ? rgba(cr * k, cg * k, cb * k, 1.f) : rgba(cr, cg, cb, k), ox / W, oy / H };
            }
            if (ok) { out.push_back(q[0]); out.push_back(q[1]); out.push_back(q[2]); out.push_back(q[0]); out.push_back(q[2]); out.push_back(q[3]); }
        }
    };
    if (useVeil) {
        if (g_cfg.wsVeil > 0.001f && filmMax > 0.01f) {
            buildHaze(g_wsFilm, g_wsVVeil, clamp01(g_cfg.wsVeil) * 0.85f, 0.85f, 0.90f, 0.94f, 0.96f, false);
            if (g_cfg.wsGlow > 0.001f) buildHaze(g_wsFilm, g_wsVGlow, g_cfg.wsGlow, 1.0f, 1.f, 1.f, 1.f, true);      // bright lights bloom through wet glass
        }
        if (g_cfg.wsDirt > 0.001f && dirtMax > 0.01f) {
            const float dr = snowyNow ? 0.86f : g_cfg.wsMudR * 1.35f + 0.04f, dg = snowyNow ? 0.88f : g_cfg.wsMudG * 1.35f + 0.04f, db = snowyNow ? 0.92f : g_cfg.wsMudB * 1.35f + 0.04f;
            buildHaze(g_wsDirt, g_wsVDirt, std::min(0.95f, 0.78f * g_cfg.wsDirt), 0.80f, std::min(1.f, dr), std::min(1.f, dg), std::min(1.f, db), false);
        }
    }
    const bool useFrost = sceneOK && g_wsBlurTex[0] && !g_wsVeilFailed && g_cfg.wsDirt > 0.001f;
    for (const WsMud& m : g_wsMud) {
        if (m.age < 0.f) continue;
        float x = m.x, y = m.y, ang = m.rot, stretch = m.stretch, pop;
        if (m.dur > 0.f) {                                             // thrown outwards from the impact point: quick, with a streak along the motion
            pop = 1.f;
            if (m.age < m.dur) { const float k = m.age / m.dur, e = 1.f - (1.f - k) * (1.f - k); x = m.ox + (m.x - m.ox) * e; y = m.oy + (m.y - m.oy) * e; ang = std::atan2(-(m.x - m.ox) * W, (m.y - m.oy) * H); stretch = 1.f + 2.6f * (1.f - k); }
        } else pop = (0.5f + 0.5f * wsSmooth(0.f, 0.09f, m.age)) * (1.f + 0.10f * std::sin(3.14159f * clamp01(m.age / 0.16f)));      // flattens on impact, a little overshoot
        const float hw = m.r * H * pop, a = op * clamp01(m.fade), sh = m.shade > 0.f ? m.shade : 1.f;
        const bool dirty = m.variant == kMudSpeckle;
        DWORD col;
        if (m.tint) col = rgba(0.88f * std::min(1.1f, sh), 0.90f * std::min(1.1f, sh), 0.94f * std::min(1.1f, sh), a);
        else { const float k = dirty ? 1.25f : 1.f; col = rgba(g_cfg.wsMudR * sh * k, g_cfg.wsMudG * sh * k, g_cfg.wsMudB * sh * k, a * (dirty ? 0.85f : 1.f)); }
        const float u0 = (float)(m.variant & 3) * 0.25f, v0 = (float)(m.variant >> 2) * 0.5f;
        if (useFrost && m.variant <= kMudSmear && hw >= 9.f && m.dur <= 0.f)      // mud is not clear: the view behind it is frosted and tinted
            emitFrost(g_wsVFrost, x * W, y * H, hw, hw * stretch, ang, m.tint ? rgba(0.90f, 0.92f, 0.96f, 0.75f * a) : rgba(std::min(1.f, 0.50f + g_cfg.wsMudR * 0.9f), std::min(1.f, 0.50f + g_cfg.wsMudG * 0.9f), std::min(1.f, 0.50f + g_cfg.wsMudB * 0.9f), 0.78f * a * std::min(1.f, g_cfg.wsDirt)), u0, v0, u0 + 0.25f, v0 + 0.5f);
        quad(g_wsVMud, x * W, y * H, hw, hw * stretch, ang, col, u0, v0, u0 + 0.25f, v0 + 0.5f);
    }
    for (const WsFly& f : g_wsFly) {                                   // particles on their way to the glass
        if (f.t <= 0.f || f.kind < 0) continue;
        const float p = std::min(1.f, f.t); float fx, fy, sc, bx, by, bsc; wsFlyPos(f, p, fx, fy, sc);
        const float rpx = f.r * H, X = fx * W, Y = fy * H;
        wsFlyPos(f, std::max(0.f, p - 0.20f), bx, by, bsc);
        const float vis = wsSmooth(0.f, 0.30f, p);
        if (f.kind == 0) {                                             // rain: a short bright streak along its path
            const float w1 = std::max(0.7f, rpx * 0.55f * (0.30f + 0.70f * sc));
            strip(g_wsVStreak, bx * W, by * H, X, Y, w1 * 0.25f, w1, rgba(0.88f, 0.92f, 0.97f, op * 0.60f * vis));
        } else if (f.kind == 2) {                                      // snow: a soft white fleck that grows as it comes
            const float sz = std::max(1.2f, rpx * (0.20f + 0.80f * sc));
            quad(g_wsVFlyFlake, X, Y, sz, sz, f.x1 * 7.f, rgba(1.f, 1.f, 1.f, op * 0.9f * vis), 0.f, 0.f, 1.f, 1.f);
        } else {                                                       // mud clod / dirty spray: a dark blob, stretched along its flight, with a faint streak behind it
            const float dx = (X - bx * W), dy = (Y - by * H), len = std::sqrt(dx * dx + dy * dy), ang = len > 0.5f ? std::atan2(-dx, dy) : 0.f;
            const float hw = std::max(1.5f, rpx * (f.mode == 1 ? (0.40f + 0.60f * sc) : (0.25f + 0.75f * sc))), st = 1.f + std::min(1.4f, len / std::max(4.f, hw) * 0.35f);
            const float sh = f.shade, al = op * (f.kind == 3 ? 0.80f : 0.95f);
            const DWORD col = f.tint ? rgba(0.88f, 0.90f, 0.94f, al) : rgba(g_cfg.wsMudR * sh * (f.kind == 3 ? 1.25f : 1.f), g_cfg.wsMudG * sh * (f.kind == 3 ? 1.25f : 1.f), g_cfg.wsMudB * sh * (f.kind == 3 ? 1.25f : 1.f), al);
            const float u0 = (float)(f.variant & 3) * 0.25f, v0 = (float)(f.variant >> 2) * 0.5f;
            if (f.kind == 1 && len > 2.f) strip(g_wsVStreak, bx * W, by * H, X, Y, hw * 0.15f, hw * 0.55f, f.tint ? rgba(0.85f, 0.88f, 0.92f, op * 0.28f) : rgba(g_cfg.wsMudR, g_cfg.wsMudG, g_cfg.wsMudB, op * 0.30f));
            quad(g_wsVFlyMud, X, Y, hw, hw * st, ang, col, u0, v0, u0 + 0.25f, v0 + 0.5f);
        }
    }
    for (const WsTrail& t : g_wsTrails) {
        const float a = std::pow(std::max(0.f, 1.f - t.age / life), 1.3f) * op * 0.7f;
        strip(g_wsVTrail, t.x0 * W, t.y0 * H, t.x1 * W, t.y1 * H, t.w0 * H, t.w1 * H, rgba(1.f, 1.f, 1.f, a));
    }
    for (const WsDrop& dr : g_wsDrops) {
        const float rpx = dr.r * H;
        if (dr.kind == 1) {                                            // snow flake
            const float prog = 1.f - clamp01(dr.melt / std::max(1.f, dr.melt0)), pop = 0.5f + 0.5f * wsSmooth(0.f, 0.15f, dr.age);
            const float s = rpx * pop * (1.f - 0.35f * prog), a = op * 0.95f * (1.f - 0.45f * prog);
            quad(g_wsVFlake, dr.x * W, dr.y * H, s, s, dr.rot, rgba(1.f, 1.f, 1.f, a), 0.f, 0.f, 1.f, 1.f);
            continue;
        }
        if (dr.kind == 2) {                                            // micro droplet
            const float s = std::max(rpx, 1.2f) * 1.25f, lf = clamp01(dr.melt / 3.f);
            quad(g_wsVMicro, dr.x * W, dr.y * H, s, s, 0.f, rgba(1.f, 1.f, 1.f, op * 0.9f * lf), 0.f, 0.f, 1.f, 1.f);
            continue;
        }
        const float vlen = std::sqrt(dr.vx * dr.vx + dr.vy * dr.vy);
        const bool moving = vlen > 0.02f;
        const float stretch = moving ? 1.15f * (1.f + std::min(1.3f, vlen * 2.4f)) : 1.f;
        const float ang = moving ? std::atan2(-dr.vx, dr.vy) : 0.f;    // local y axis follows the motion, the head leads
        const float sc = 0.4f + 0.6f * wsSmooth(0.f, 0.12f, dr.age);   // impact: the drop spreads out
        const float lf = dr.melt0 > 0.f ? clamp01(dr.melt / 3.f) : 1.f;   // melted snow fades out
        const bool pool = !moving && g_cfg.wsPools && rpx >= g_cfg.wsPoolMinPx;
        if (pool) {                                                    // big standing drop: a flat irregular pool
            const float sq = std::sqrt(dr.asp), hwp = rpx * 1.75f * sq * sc, hhp = rpx * 1.75f / sq * sc, cu = (float)(dr.var & 1) * 0.5f, cv = (float)(dr.var >> 1) * 0.5f;
            if (useRefr) emitRefr(g_wsVRefrPool, dr.x * W, dr.y * H, hwp, hhp, dr.rot, rgba(0.78f, 0.83f, 0.83f, op * lf), cu, cv, cu + 0.5f, cv + 0.5f, refK * 0.35f);
            quad(g_wsVShadePool, dr.x * W, dr.y * H, hwp, hhp, dr.rot, rgba(1.f, 1.f, 1.f, op * lf), cu, cv, cu + 0.5f, cv + 0.5f);
        } else {
            const float rh = std::min(rpx, 0.013f * H * std::max(0.5f, g_cfg.wsDropSize)), hw = rh * sc * 1.08f, hh = hw * stretch;
            const int cell = moving ? 1 : 0;
            if (useRefr && rpx >= g_cfg.wsRefractMinPx) {
                emitRefr(g_wsVRefr, dr.x * W, dr.y * H, hw, hh, ang, rgba(0.95f, 0.97f, 0.98f, op * lf), cell * 0.5f, 0.f, cell * 0.5f + 0.5f, 1.f, refK);
                quad(g_wsVShade, dr.x * W, dr.y * H, hw, hh, ang, rgba(1.f, 1.f, 1.f, op * lf), cell * 0.5f, 0.f, cell * 0.5f + 0.5f, 1.f);
            } else {
                quad(g_wsVSprite, dr.x * W, dr.y * H, hw, hh, ang, rgba(1.f, 1.f, 1.f, op * 0.9f * lf), 0.f, 0.f, 1.f, 1.f);
            }
        }
        if (dr.age < 0.14f) {                                          // impact flash
            const float k = dr.age / 0.14f, s = rpx * (1.3f + 2.6f * k);
            quad(g_wsVSplash, dr.x * W, dr.y * H, s, s, 0.f, rgba(1.f, 1.f, 1.f, (1.f - k) * 0.7f * op), 0.f, 0.f, 1.f, 1.f);
        }
    }
    if (g_cfg.wsWipers && g_wsBladeInit) {
        WsBlade bl[2]; wsBlades(bl);
        const float thk = std::max(0.3f, g_cfg.wsWiperThick), aw = 0.0058f * H * thk, bw = 0.0070f * H * thk;
        for (int i = 0; i < 2; ++i) {
            if (bl[i].len <= 0.f || (g_cfg.wsWiperSource && !g_wsModelBlade[i].ok)) continue;      // with WiperSource=1 nothing is drawn until the real blade pose is known (no doubled wipers)
            const float pxv = bl[i].px * W, pyv = bl[i].py * H, L = bl[i].len * H, a = g_wsBladeAng[i], ca = std::cos(a), sa = -std::sin(a);
            const float dir = (g_wsBladeAng[i] - g_wsBladePrev[i]) >= 0.f ? 1.f : -1.f;
            if (g_cfg.wsWiperBow && g_wsBow[i] > 0.03f) {              // the ridge of water the blade pushes ahead of it
                const float tx = -std::sin(a) * dir, ty = -std::cos(a) * dir, off = bw * 1.15f, k = std::min(1.f, g_wsBow[i]), wd = (1.6f + 3.6f * k) * std::max(0.6f, H / 1080.f);
                strip(g_wsVTrail, pxv + ca * L * 0.27f + tx * off, pyv + sa * L * 0.27f + ty * off, pxv + ca * L + tx * off, pyv + sa * L + ty * off, wd * 0.8f, wd, rgba(1.f, 1.f, 1.f, std::min(0.9f, 0.95f * k) * op));
            }
            if (!g_cfg.wsWiperDraw) continue;                          // the car's own blades are drawn by the game; the rest is for the plugin's blades
            if (g_cfg.wsWiperBlur) for (int k = 1; k >= 0; --k) {       // motion blur: a faint wedge between the blade and where it was a moment ago
                const float g = g_wsGhost[i][k]; if (std::fabs(g - a) < 0.004f) continue;
                const float r0 = L * 0.26f, r1 = L, cg = std::cos(g), sg = -std::sin(g);
                const float px4[4] = {pxv + cg * r0, pxv + cg * r1, pxv + ca * r1, pxv + ca * r0}, py4[4] = {pyv + sg * r0, pyv + sg * r1, pyv + sa * r1, pyv + sa * r0};
                emit(g_wsVWBlur, px4, py4, rgba(0.03f, 0.03f, 0.035f, (k == 0 ? 0.32f : 0.16f) * op), 0.f, 0.f, 1.f, 1.f);
            }
            const float nx = -sa * (ca > 0.f ? -1.f : 1.f), ny = ca * (ca > 0.f ? -1.f : 1.f);        // unit normal on the upper side of the blade (for the highlights)
            strip(g_wsVDbg, pxv, pyv, pxv + ca * L * 0.97f, pyv + sa * L * 0.97f, aw * 1.15f, aw * 0.80f, rgba(0.085f, 0.085f, 0.095f, 0.98f * op));                  // arm
            strip(g_wsVDbg, pxv + nx * aw * 0.55f, pyv + ny * aw * 0.55f, pxv + ca * L * 0.95f + nx * aw * 0.4f, pyv + sa * L * 0.95f + ny * aw * 0.4f, aw * 0.16f, aw * 0.12f, rgba(0.40f, 0.42f, 0.46f, 0.40f * op));   // arm highlight
            strip(g_wsVDbg, pxv + ca * L * 0.26f, pyv + sa * L * 0.26f, pxv + ca * L, pyv + sa * L, bw, bw * 0.92f, rgba(0.012f, 0.012f, 0.016f, 0.99f * op));          // rubber blade
            strip(g_wsVDbg, pxv + ca * L * 0.27f + nx * bw * 0.45f, pyv + sa * L * 0.27f + ny * bw * 0.45f, pxv + ca * L * 0.99f + nx * bw * 0.45f, pyv + sa * L * 0.99f + ny * bw * 0.45f, bw * 0.20f, bw * 0.18f, rgba(0.45f, 0.47f, 0.52f, 0.38f * op));   // glint on the blade
            quad(g_wsVDbg, pxv, pyv, aw * 1.9f, aw * 1.9f, 0.f, rgba(0.085f, 0.085f, 0.095f, 0.98f * op), 0.f, 0.f, 1.f, 1.f);                                           // pivot cap (octagon)
            quad(g_wsVDbg, pxv, pyv, aw * 1.9f, aw * 1.9f, 0.7854f, rgba(0.085f, 0.085f, 0.095f, 0.98f * op), 0.f, 0.f, 1.f, 1.f);
        }
    }
    if (g_cfg.wsWiperDebug) {                                          // calibration: where the emulated blades are right now
        WsBlade bl[2]; wsBlades(bl);
        for (int i = 0; i < 2; ++i) {
            if (bl[i].len <= 0.f) continue;
            const float pxv = bl[i].px * W, pyv = bl[i].py * H, L = bl[i].len * H, a = g_wsBladeAng[i];
            const DWORD col = i == 0 ? rgba(1.f, 0.1f, 0.8f, 0.9f) : rgba(0.1f, 0.9f, 1.f, 0.9f);
            strip(g_wsVDbg, pxv, pyv, pxv + std::cos(a) * L, pyv - std::sin(a) * L, 2.5f, 2.5f, col);
            quad(g_wsVDbg, pxv, pyv, 8.f, 8.f, 0.f, col, 0.f, 0.f, 1.f, 1.f);
            if (!(g_cfg.wsWiperSource && g_wsModelBlade[i].ok)) {                              // emulated: the two ends of the sweep, faint
                const float a0 = bl[i].a0 * 0.0174533f, a1 = bl[i].a1 * 0.0174533f;
                strip(g_wsVDbg, pxv, pyv, pxv + std::cos(a0) * L, pyv - std::sin(a0) * L, 1.2f, 1.2f, rgba(0.2f, 1.f, 0.2f, 0.45f));
                strip(g_wsVDbg, pxv, pyv, pxv + std::cos(a1) * L, pyv - std::sin(a1) * L, 1.2f, 1.2f, rgba(1.f, 0.9f, 0.1f, 0.45f));
            }
        }
    }
    if (g_wsVMud.empty() && g_wsVDirt.empty() && g_wsVGlow.empty() && g_wsVFrost.empty() && g_wsVFlyMud.empty() && g_wsVFlyFlake.empty() && g_wsVStreak.empty() && g_wsVWBlur.empty() && g_wsVTrail.empty() && g_wsVSprite.empty() && g_wsVRefr.empty() && g_wsVRefrPool.empty() && g_wsVShadePool.empty() && g_wsVMicro.empty() && g_wsVVeil.empty() && g_wsVFlake.empty() && g_wsVSplash.empty() && g_wsVDbg.empty()) { g_wsDrawn++; return true; }

    g_wsDrawing = true;
    g_wsSB->Capture();                                                 // save everything the game had set
    d->SetVertexShader(nullptr); d->SetPixelShader(nullptr);
    d->SetRenderState(D3DRS_ZENABLE, useZ ? TRUE : FALSE);  d->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);   d->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    d->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);    d->SetRenderState(D3DRS_SEPARATEALPHABLENDENABLE, FALSE);
    d->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA); d->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    d->SetRenderState(D3DRS_BLENDOP, D3DBLENDOP_ADD);
    d->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);    d->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    d->SetRenderState(D3DRS_LIGHTING, FALSE);           d->SetRenderState(D3DRS_FOGENABLE, FALSE);
    d->SetRenderState(D3DRS_STENCILENABLE, FALSE);      d->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
    d->SetRenderState(D3DRS_COLORWRITEENABLE, 0xF);     d->SetRenderState(D3DRS_SRGBWRITEENABLE, FALSE);
    d->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);   d->SetRenderState(D3DRS_SPECULARENABLE, FALSE);
    for (DWORD s = 0; s < 2; ++s) {
        d->SetSamplerState(s, D3DSAMP_MINFILTER, D3DTEXF_LINEAR); d->SetSamplerState(s, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR); d->SetSamplerState(s, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
        d->SetSamplerState(s, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP); d->SetSamplerState(s, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP); d->SetSamplerState(s, D3DSAMP_SRGBTEXTURE, 0);
    }
    d->SetTextureStageState(0, D3DTSS_TEXCOORDINDEX, 0);
    d->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);   d->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE); d->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
    d->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);   d->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);  d->SetTextureStageState(0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
    d->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_DISABLE);    d->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
    d->SetTextureStageState(2, D3DTSS_COLOROP, D3DTOP_DISABLE);    d->SetTextureStageState(2, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
    d->SetTexture(1, nullptr);
    auto batch = [&](std::vector<WsVert>& v, IDirect3DTexture9* tex) {
        if (v.empty()) return;
        d->SetFVF(kWsFvf1); d->SetTexture(0, tex); d->DrawPrimitiveUP(D3DPT_TRIANGLELIST, (UINT)(v.size() / 3), v.data(), sizeof(WsVert));
    };
    batch(g_wsVStreak, g_wsStreakTex); batch(g_wsVFlyFlake, g_wsFlakeTex);          // things still outside the glass: rain streaks, snow, the trails behind mud
    if (useVeil) {
        batch(g_wsVVeil, g_wsBlurTex[1]); batch(g_wsVDirt, g_wsBlurTex[1]);          // haze: a blurred copy of the frame where the glass is wet / dirty
        if (!g_wsVGlow.empty()) {                                                    // bloom: the same blurred copy added on top (bright lights glow)
            d->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_ONE); d->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_ONE);
            batch(g_wsVGlow, g_wsBlurTex[1]);
            d->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA); d->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
        }
    }
    auto drawRefr = [&](std::vector<WsVert2>& v, IDirect3DTexture9* maskTex, IDirect3DTexture9* sceneTex) {   // stage 0: a copy of the frame tinted by the vertex colour; stage 1: the mask supplies the alpha
        if (v.empty()) return;
        d->SetFVF(kWsFvf2); d->SetTexture(0, sceneTex); d->SetTexture(1, maskTex);
        d->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1); d->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
        d->SetTextureStageState(1, D3DTSS_TEXCOORDINDEX, 1);
        d->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_SELECTARG1); d->SetTextureStageState(1, D3DTSS_COLORARG1, D3DTA_CURRENT);
        d->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_MODULATE);   d->SetTextureStageState(1, D3DTSS_ALPHAARG1, D3DTA_TEXTURE); d->SetTextureStageState(1, D3DTSS_ALPHAARG2, D3DTA_CURRENT);
        d->DrawPrimitiveUP(D3DPT_TRIANGLELIST, (UINT)(v.size() / 3), v.data(), sizeof(WsVert2));
        d->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_DISABLE); d->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_DISABLE); d->SetTexture(1, nullptr);
        d->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE); d->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
    };
    if (useFrost) drawRefr(g_wsVFrost, g_wsMudTex, g_wsBlurTex[0]);                 // frosted view inside the mud splats
    batch(g_wsVMud, g_wsMudTex); batch(g_wsVFlyMud, g_wsMudTex); batch(g_wsVTrail, g_wsTrailTex); batch(g_wsVMicro, g_wsMicroTex);
    drawRefr(g_wsVRefrPool, g_wsPoolMaskTex, g_wsSceneTex); batch(g_wsVShadePool, g_wsPoolShadeTex);
    drawRefr(g_wsVRefr, g_wsMaskTex, g_wsSceneTex);
    batch(g_wsVShade, g_wsShadeTex); batch(g_wsVSprite, g_wsSpriteTex); batch(g_wsVFlake, g_wsFlakeTex); batch(g_wsVSplash, g_wsSplashTex); batch(g_wsVWBlur, g_wsWhiteTex); batch(g_wsVDbg, g_wsWhiteTex);
    g_wsSB->Apply();                                                   // restore the game's state
    g_wsDrawing = false;
    g_wsDrawn++;
    return true;
}

// ---- Probe=1 ----
// (1) Wiper-pose scanner: every frame it watches ~1700 floats (the object the game's wet-windscreen function gets, the car info block, the car physics
// block) and every 6 s logs the ones that swing back and forth like a wiper angle: clean, regular reversals every 0.25-4 s. Run it on a rain stage
// with the wipers going, then read the "WiperScan candidate" lines: offsets are bytes from the start of that block.
struct WsScanStat { float minV, maxV, hi, lo, hiT, loT, lastRevT, sumHP, sumHP2; int dir, n; bool init, bad; };
std::vector<WsScanStat> g_wsScan; float g_wsScanT0 = 0.f;
constexpr int kScanObj = 256, kScanCar = 256, kScanMove = 1152;
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
void wsScanReport() {
    struct Cand { int idx; float cv, mean; };
    std::vector<Cand> c;
    for (int i = 0; i < (int)g_wsScan.size(); ++i) {
        const WsScanStat& s = g_wsScan[i];
        if (!s.init || s.bad || s.n < 4 || s.maxV - s.minV < 0.05f) continue;
        const float mean = s.sumHP / (float)s.n, var = std::max(0.f, s.sumHP2 / (float)s.n - mean * mean);
        if (mean < 0.25f || mean > 4.f) continue;
        const float cv = std::sqrt(var) / mean;
        if (cv < 0.35f) c.push_back({i, cv, mean});
    }
    std::sort(c.begin(), c.end(), [](const Cand& a, const Cand& b) { return a.cv < b.cv; });
    Logf("WiperScan: %d candidate(s) in the last 6 s (an angle-like value with regular reversals; run it on a rain stage with the wipers going)", (int)c.size());
    for (size_t k = 0; k < c.size() && k < 8; ++k) {
        const int i = c[k].idx; const WsScanStat& s = g_wsScan[i];
        const char* blk = i < kScanObj ? "wet-fx object" : i < kScanObj + kScanCar ? "car info" : "car physics";
        const int off = (i < kScanObj ? i : i < kScanObj + kScanCar ? i - kScanObj : i - kScanObj - kScanCar) * 4;
        Logf("WiperScan candidate: %s +0x%X  range %.4g..%.4g  half-period %.2fs  irregularity %.2f  reversals %d", blk, off, s.minV, s.maxV, c[k].mean, c[k].cv, s.n);
    }
    for (WsScanStat& s : g_wsScan) s = WsScanStat{};
}
void wsScan(void* obj) {
    if (g_wsScan.empty()) g_wsScan.assign(kScanObj + kScanCar + kScanMove, WsScanStat{});
    const float t = (float)GetTickCount() / 1000.f;
    if (g_wsScanT0 == 0.f) g_wsScanT0 = t;
    int base = 0;
    auto region = [&](uintptr_t a, int n) {
        if (a && readableRange(a, (size_t)n * 4)) for (int i = 0; i < n; ++i) { float v; memcpy(&v, (const void*)(a + (uintptr_t)i * 4), 4); wsScanUpdate(g_wsScan[base + i], v, t); }
        base += n;
    };
    region((uintptr_t)obj, kScanObj);
    region(absAddr(0x165FC68), kScanCar);
    uintptr_t mv = 0; const uintptr_t pm = absAddr(0x8EF660);
    if (readableRange(pm, 4)) mv = (uintptr_t) * (const uint32_t*)pm;
    region(mv, kScanMove);
    if (t - g_wsScanT0 > 6.f) { wsScanReport(); g_wsScanT0 = t; }
}
// (2) raw dumps of the last telemetry packet and the game's wet-windscreen object every ~2 s.
void wsProbeLog() {
    static int tick = 0; if ((++tick & 1) != 0) return;
    char line[1100]; int len = 0;
    auto flush = [&]() { if (len > 0) { Logf("%s", line); len = 0; } };
    const int n = g_pktLen.load();
    if (n >= 136) {
        Logf("probe pkt: %d bytes; dword index (= byte offset / 4) = value, zeros skipped, iNNN = integer", n);
        for (int i = 0; i < n / 4; ++i) {
            float f; int iv; memcpy(&f, g_pktLast + i * 4, 4); memcpy(&iv, g_pktLast + i * 4, 4);
            if (iv == 0) continue;
            if (std::isfinite(f) && std::fabs(f) < 1.0e6f && std::fabs(f) > 1.0e-6f) len += snprintf(line + len, sizeof(line) - len, " %d=%.4g", i, f);
            else len += snprintf(line + len, sizeof(line) - len, " %d=i%d", i, iv);
            if (len > 900) flush();
        }
        flush();
    }
    if (g_wsProbeA && readableRange(g_wsProbeA, 0x100)) {
        Logf("probe wet-windscreen object @%p (dword index = value):", (void*)g_wsProbeA);
        const uint32_t* w = (const uint32_t*)g_wsProbeA;
        for (int i = 0; i < 64; ++i) {
            if (!w[i]) continue;
            float f; memcpy(&f, &w[i], 4);
            if (std::isfinite(f) && std::fabs(f) < 1.0e6f && std::fabs(f) > 1.0e-6f) len += snprintf(line + len, sizeof(line) - len, " %d=%.4g", i, f);
            else len += snprintf(line + len, sizeof(line) - len, " %d=0x%X", i, (unsigned)w[i]);
        }
        flush();
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
inline bool wsHuntActive() { return g_in3D && g_wsArmed && !g_wsDrawing && ((g_cfg.wsWipers && g_cfg.wsWiperSource) || g_cfg.wsProbe); }
float wsRotAngle(const float* Ra, const float* Rb) {                  // angle between two rotations: acos((tr(Ra^T Rb) - 1) / 2)
    float tr = 0.f; for (int i = 0; i < 9; ++i) tr += Ra[i] * Rb[i];
    return std::acos(std::max(-1.f, std::min(1.f, (tr - 1.f) * 0.5f)));
}
void wsHuntReset() {
    g_wsHunt.clear(); memset(g_wsSlots, 0, sizeof(g_wsSlots));
    g_wsLock[0].on = g_wsLock[1].on = false; g_wsHuntLocked = false; g_wsPrevSel[0] = g_wsPrevSel[1] = 0; g_wsHuntT0 = 0.f;
    g_wsModelBlade[0].ok = g_wsModelBlade[1].ok = false;
}
// Called for every 3D draw call while hunting or while a blade is locked. Returns true when the draw is a locked blade that should be hidden.
bool wsHuntDraw(uint32_t sig) {
    if (!g_lastMvpOk || !g_wsPok) return false;
    WsHuntSlot* sl = nullptr;                                          // occurrence index of this mesh in the frame (two wipers can share a mesh)
    for (int p = 0; p < 8 && !sl; ++p) { WsHuntSlot& s = g_wsSlots[(sig + (uint32_t)p) & 2047u]; if (s.frame == g_wsFrameNo && s.sig == sig) sl = &s; }
    for (int p = 0; p < 8 && !sl; ++p) { WsHuntSlot& s = g_wsSlots[(sig + (uint32_t)p) & 2047u]; if (s.frame != g_wsFrameNo) { sl = &s; s.sig = sig; s.cnt = 0; s.frame = g_wsFrameNo; } }
    if (!sl) return false;
    const int k = std::min(sl->cnt++, 255);
    const uint64_t key = ((uint64_t)sig << 8) | (uint64_t)k;
    const bool l0 = g_wsLock[0].on && g_wsLock[0].key == key, l1 = g_wsLock[1].on && g_wsLock[1].key == key;
    const bool hunting = !g_wsHuntLocked || g_cfg.wsProbe;
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
bool wsProjectCam(float x, float y, float z, float W, float H, float& ox, float& oy) {    // camera-space point -> pixels of the base-pose frame
    const float cx = g_wsP.m[0][0] * x + g_wsP.m[0][1] * y + g_wsP.m[0][2] * z + g_wsP.m[0][3];
    const float cy = g_wsP.m[1][0] * x + g_wsP.m[1][1] * y + g_wsP.m[1][2] * z + g_wsP.m[1][3];
    const float cw = g_wsP.m[3][0] * x + g_wsP.m[3][1] * y + g_wsP.m[3][2] * z + g_wsP.m[3][3];
    if (cw < 1e-4f) return false;
    ox = (cx / cw * 0.5f + 0.5f) * W; oy = (0.5f - cy / cw * 0.5f) * H; return true;
}
void wsHuntPose(float W, float H) {                                    // the locked blades' pivot, length and angle on the glass, from their current matrices
    for (int i = 0; i < 2; ++i) {
        WsModelBlade& mb = g_wsModelBlade[i]; mb.ok = false;
        if (!g_cfg.wsWiperSource || !g_wsLock[i].on || !g_wsPok) continue;
        const auto it = g_wsHunt.find(g_wsLock[i].key); if (it == g_wsHunt.end()) continue;
        const WsHuntEnt& e = it->second; if (g_wsFrameNo - e.lastFrame > 30) continue;      // not drawn recently
        const int ax = std::max(0, std::min(2, g_cfg.wsWiperAxis)); const float sgn = g_cfg.wsWiperAxisSign < 0 ? -1.f : 1.f, Lm = g_cfg.wsWiperLenM;
        float s0x, s0y, s1x, s1y;
        if (!wsProjectCam(e.T[9], e.T[10], e.T[11], W, H, s0x, s0y)) continue;
        if (!wsProjectCam(e.T[9] + sgn * e.T[0 * 3 + ax] * Lm, e.T[10] + sgn * e.T[1 * 3 + ax] * Lm, e.T[11] + sgn * e.T[2 * 3 + ax] * Lm, W, H, s1x, s1y)) continue;
        mb.ok = true; mb.px = s0x / W; mb.py = s0y / H; mb.len = std::sqrt((s1x - s0x) * (s1x - s0x) + (s1y - s0y) * (s1y - s0y)) / H; mb.ang = std::atan2(s0y - s1y, s1x - s0x);
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
                g_wsLock[i] = {true, sel[i].key, sel[i].theta, sel[i].hp};
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
    if (!((g_cfg.wsWipers && g_cfg.wsWiperSource) || g_cfg.wsProbe)) return;
    const float t = (float)GetTickCount() / 1000.f;
    if (g_wsHuntT0 == 0.f) g_wsHuntT0 = t;
    if (g_wsHuntLocked && !g_cfg.wsProbe) return;
    if (t - g_wsHuntT0 < 6.f) return;
    g_wsHuntT0 = t; wsHuntReport();
}

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
    if (wsHuntActive() && wsHuntDraw(wsSig(t, 0, 0, 0, s, p))) { g_wsHiddenWipers++; return 0; }
    return g_origDrawPrim(dev, t, s, p);
}
HRESULT __stdcall hkDrawIdxPrim(void* dev, DWORD t, INT b, UINT mn, UINT nv, UINT si, UINT p) {
    if (g_wsSuppress && !g_wsDrawing) { g_wsHidden++; return 0; }
    if (wsHuntActive() && wsHuntDraw(wsSig(t, (UINT)b, mn, nv, si, p))) { g_wsHiddenWipers++; return 0; }
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
        if (g_cfg.wsEnable) {                                          // WindscreenFX: EndScene = 42, Reset = 16
            const bool c = MH_CreateHook(vt[42], (void*)&hkEndScene, (void**)&g_origEndScene) == MH_OK && MH_EnableHook(vt[42]) == MH_OK;
            const bool r = MH_CreateHook(vt[16], (void*)&hkReset, (void**)&g_origReset) == MH_OK && MH_EnableHook(vt[16]) == MH_OK;
            g_wsHooked = c && r;
            Note("WindscreenFX: EndScene hook %s, Reset hook %s", c ? "OK" : "FAILED", r ? "OK" : "FAILED");
            if (g_cfg.wsHideGameFx == 1 || g_cfg.wsWiperSource || g_cfg.wsHideGameWipers || g_cfg.wsProbe) {   // DrawPrimitive = 81, DrawIndexedPrimitive = 82, DrawPrimitiveUP = 83, DrawIndexedPrimitiveUP = 84
                const bool h1 = MH_CreateHook(vt[81], (void*)&hkDrawPrim, (void**)&g_origDrawPrim) == MH_OK && MH_EnableHook(vt[81]) == MH_OK;
                const bool h2 = MH_CreateHook(vt[82], (void*)&hkDrawIdxPrim, (void**)&g_origDrawIdxPrim) == MH_OK && MH_EnableHook(vt[82]) == MH_OK;
                const bool h3 = MH_CreateHook(vt[83], (void*)&hkDrawPrimUP, (void**)&g_origDrawPrimUP) == MH_OK && MH_EnableHook(vt[83]) == MH_OK;
                const bool h4 = MH_CreateHook(vt[84], (void*)&hkDrawIdxPrimUP, (void**)&g_origDrawIdxPrimUP) == MH_OK && MH_EnableHook(vt[84]) == MH_OK;
                Note("WindscreenFX: draw-call hooks for HideGameFx: %s %s %s %s", h1 ? "OK" : "FAILED", h2 ? "OK" : "FAILED", h3 ? "OK" : "FAILED", h4 ? "OK" : "FAILED");
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
    g_wsGameCalls++; g_wsGameTick = GetTickCount(); g_wsProbeA = (uintptr_t)a;
    if (g_cfg.wsProbe) wsScan(a);                                      // Probe=1: look for the wiper angle in the game's memory
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
DWORD g_lastReport = 0; uint32_t g_lastType = 0xFFFFFFFE; bool g_loggedFirst = false; int g_lastTestPhase = -1;

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
    if (g_cfg.wsProbe) wsProbeLog();
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
    g_wsArmed = g_wsOn.load() && g_wsHooked && g_cfg.wsEnable && isCockpit(type);   // WindscreenFX draws from the EndScene hook
    g_wsFrameNo++; g_wsStamp = now;
    g_in3D = true;
    g_origRender(p);
    g_in3D = false; g_patchActive = false;
}

} // namespace

// ================================================================================================ the module
class CameraFx {
public:
    struct Hooks { std::function<void(const std::string&)> log; };

    // Reads ForceFXCamera.ini and installs the hooks. cameraOn / windscreenOn are the switches from ForceFX.ini. The windscreen
    // hooks are only installed when it is on here, so a restart is needed to switch it on later.
    void configure(const std::string& dir, Hooks h, bool cameraOn, bool windscreenOn) {
        if (m_started) return;
        m_started = true;
        g_dir = dir; g_hostLog = h.log;
        g_enabled = cameraOn; g_wsOn = windscreenOn; g_cfg.wsEnable = windscreenOn ? 1 : 0;
        loadConfig();
        if (g_cfg.log) g_log = fopen((g_dir + "ForceFXCamera.log").c_str(), "w");
        if (!g_cfg.anyExe && !isRsfExe()) {
            Note("not the RallySimFans executable (RichardBurnsRally_SSE.exe), so the camera and windscreen effects stay off. Set AnyExe=1 in ForceFXCamera.ini to try anyway");
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
        const bool wantD3D = (g_cfg.method == 4) || (g_cfg.wsEnable && !openRBRVRLoaded());   // WindscreenFX also needs the device hooks
        if (wantD3D && installD3DHooks()) Logf("D3D hooks requested (method %d, windscreen enable=%d)", g_cfg.method, g_cfg.wsEnable);
        if (wantD3D) {                                                  // also needed by WindscreenFX: counts the game's own wet-overlay calls
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

    // One NGP telemetry packet. Safe to call from any thread, one packet at a time.
    void feed(const uint8_t* data, int len) {
        if (g_ready && data && len > 0) processPacket((const char*)data, len);
    }

    // The stage is starting: forget the old head pose and the last stage's events.
    void stageStarted(int iMap) {
        if (!g_ready) return;
        const WsGameWeather gw = wsReadGameWeather();
        Logf("StageStarted: map id=%d | game conditions: sky=%d (%s) surfaceWetness=%d cloud=%d timeOfDay=%d valid=%d", iMap, gw.sky, wsSkyName(gw.sky), gw.wet, gw.cloud, gw.tod, gw.ok ? 1 : 0);
        g_wsGW = gw; g_wsStageId = iMap; wsApplyStage();
        g_stageStartTick = GetTickCount(); g_wsReset = true; g_evCrashSurge = 0; g_evCrashSway = 0; g_evLand = 0;
        resetHead(); g_haveBase = false; g_haveLast = false;
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

}  // namespace cam
}  // namespace ffx
