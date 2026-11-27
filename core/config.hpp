// Every setting of the plugin, its defaults (they match the shipped ForceFX.ini), the packet layout built from it and the clamp that keeps it in its
// safe range. Reading and writing the ini file is Windows-only and stays in src/RBR_ForceFX.cpp (SyncConfig).
#pragma once
#include <string>
#include "core/common.hpp"
#include "effects/force_scale.hpp"
#include "telemetry/telemetry_layout.hpp"
#include "telemetry/tyre_motion.hpp"

namespace ForceFX {

// ------------------------------------------------------------------- config
struct Config {
    // These defaults match the shipped ForceFX.ini, so "Reset" in the menu gives you a fresh-install setup.

    // telemetry connection
    int   telemetrySource = 0;       // 0 = auto (NGP's send hook, UDP port as backup), 1 = send hook only, 2 = UDP port only
    int   udpPort = 6778;            // backup listening port (0 = never open one). SimHub usually uses 6776: never share a port
    int   minPacket = 664;
    int   cylinders = 4;             // engine pulses per rev = cylinders / 2

    // effect switches (menu): 1 = on, 0 = off
    int   fxEngine = 1, fxRoad = 1, fxBrake = 1, fxLandBoost = 1, fxSurface = 1;
    int   fxWater = 1;               // splash, wading and (optionally) aquaplaning when the tyres are in standing water
    float waterGain = 1.0f;          // 0..2: strength of the splash thump, the wading rumble and the kick on entry
    int   fxAccelLight = 1;          // lighter steering under hard acceleration, from the measured front load and the measured acceleration
    float accelLight = 1.0f;         // 0..2: 1 = the steering loses force in proportion to the front load lost (never more than 60%); 0 = off
    int   fxWeight = 1;              // the weight swelling RBR's force in corners and swinging across the car in a direction change
    float weightGain = 1.0f;         // 0..2: 1 = up to 25% more force from cornering load and up to 35% more while the load swings across (never more than 50% together)
    float waterHeavy = 0.6f;         // 0..2: how much heavier the steering gets while the front tyres push through water (0 = off, 1 = double RBR's force)
    float lockLight = 1.0f;          // 0..2: how far RBR's force lightens while the front tyres are locked (0 = off, 1 = by half when both are fully locked)
    float brakeHeavy = 0.0f;         // 0..2: how much heavier the steering gets in hard braking (0 = off: RBR already makes it heavy)
    int   fxCrash = 1;               // a jolt on RBR's own force when the car hits something
    float crashBoost = 0.8f;         // 0..3: how much the force grows in a full-strength crash (0.8 = 1.8x)
    float crashAdd = 0.15f;          // 0..1: the minimum extra push of a full-strength crash, a share of full torque
    float maxForceMul = ForceScale::kDefaultCap;   // 1..4: no combination of boosts ever scales RBR's force by more than this
    int   fxAudio = 0;               // dynamic audio (settings in the [Audio] section of ForceFX.ini); off until switched on
    float audioDebris = 1.0f;        // 0..4 live multiplier on the stone / debris sounds (on top of StoneLevel in [Audio] of ForceFX.ini)
    float audioImpact = 1.0f;        // 0..4 live multiplier on impacts and landings (on top of EventLevel)
    float audioCabin = 1.0f;         // 0..2 live multiplier on the interior noises (fan, wind, panel buzz, rattles), heard only from inside the car
    float audioWet = 1.0f;           // 0..2 live multiplier on how much damp and wet ground changes the tyre sounds (on top of the Wet* settings in [Audio])
    float audioSusp = 1.0f;          // 0..4 live multiplier on suspension creaks and bottoming out (on top of SqueakLevel / BottomOutLevel)
    int   fxCamera = 0;              // head movement in the cockpit camera (settings in the [Camera...] sections of ForceFX.ini)
    int   fxWindscreen = 0;          // rain, snow and mud on the windscreen; off by default. Its hooks are installed at startup, so switching it on needs a restart

    // per-layer ceilings (DirectInput units 0..10000), a combined ceiling, and one live level (userGain)
    int   engineMaxMag = 1200, roadMaxMag = 500, brakeMaxMag = 2000;
    int   totalMaxMag = 4000;        // all layers together are scaled down to this (never above kHardTotalMax)
    float priorityDuck = 0.85f;      // 0..1: how far lower layers back off while a higher one is active (landing > road > engine)
    float sustainedLimit = 0.10f;    // thermal derate: long-term mean-square level (0.10 is roughly 32% RMS)

    // landings
    float impactGain = 0.7f;         // landing sensitivity 0..3 (0 = off)
    float airLevel = 0.2f;           // 0.2..0.8: total strut load (fraction of static) below which the car counts as airborne; raise it to catch smaller jumps
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
    int   offGear = 44, offCarRoll = 76, offVelHeave = 96;   // used by the audio and the camera, which read the packet through the same layout
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

// The packet layout the audio and the camera read through: the same offsets this file uses (the Offset* keys in [ForceFX] of ForceFX.ini).
inline Telem::Layout LayoutFromConfig(const Config& c) {
    Telem::Layout l;
    l.minLength = c.minPacket;
    l.steps = c.offSteps; l.carIndex = c.offCarIndex;
    l.steering = c.offSteering; l.throttle = c.offThrottle; l.brake = c.offBrake; l.handbrake = c.offHandbrake; l.gear = c.offGear; l.carRoll = c.offCarRoll;
    l.speed = c.offSpeed; l.velSurge = c.offVelSurge; l.velSway = c.offVelSway; l.velHeave = c.offVelHeave; l.yawRate = c.offYawRate;
    l.accSurge = c.offAccSurge; l.accSway = c.offAccSway; l.accHeave = c.offAccHeave;
    l.rpm = c.offRpm; l.rpmMax = c.offRpmMax; l.rpmScale = c.rpmScale;
    l.SetSuspension(c.offSuspBase, c.suspStride);
    l.strutForce = c.offStrut; l.pistonVelocity = c.offPiston; l.currentSegment = c.offSegment;
    return l;
}

// Pulls every setting back into its safe range. Runs after the ini is read and after every menu edit.
inline void ClampConfig(Config& c) {
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
    c.fxEngine = c.fxEngine ? 1 : 0; c.fxRoad = c.fxRoad ? 1 : 0; c.fxBrake = c.fxBrake ? 1 : 0; c.fxLandBoost = c.fxLandBoost ? 1 : 0; c.fxSurface = c.fxSurface ? 1 : 0;
    c.fxCrash = c.fxCrash ? 1 : 0;
    c.lockLight = clampf(c.lockLight, 0.0f, 2.0f); c.brakeHeavy = clampf(c.brakeHeavy, 0.0f, 2.0f);
    c.crashBoost = clampf(c.crashBoost, 0.0f, 3.0f); c.crashAdd = clampf(c.crashAdd, 0.0f, 1.0f);
    c.maxForceMul = ForceScale::Cap(c.maxForceMul);
    c.fxAudio = c.fxAudio ? 1 : 0;
    c.audioDebris = clampf(c.audioDebris, 0.0f, 4.0f); c.audioImpact = clampf(c.audioImpact, 0.0f, 4.0f); c.audioSusp = clampf(c.audioSusp, 0.0f, 4.0f); c.audioWet = clampf(c.audioWet, 0.0f, 2.0f); c.audioCabin = clampf(c.audioCabin, 0.0f, 2.0f);
    c.fxCamera = c.fxCamera ? 1 : 0; c.fxWindscreen = c.fxWindscreen ? 1 : 0;
    c.fxWater = c.fxWater ? 1 : 0; c.waterGain = clampf(c.waterGain, 0.0f, 2.0f); c.waterHeavy = clampf(c.waterHeavy, 0.0f, 2.0f);
    c.fxAccelLight = c.fxAccelLight ? 1 : 0; c.accelLight = clampf(c.accelLight, 0.0f, 2.0f);
    c.fxWeight = c.fxWeight ? 1 : 0; c.weightGain = clampf(c.weightGain, 0.0f, 2.0f);
    c.landingBoost = clampf(c.landingBoost, 0.0f, 4.0f);
    c.landingAdd = clampf(c.landingAdd, 0.0f, 1.0f);
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

}  // namespace ForceFX
