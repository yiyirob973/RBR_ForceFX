// The synthesis: turns the analysed telemetry (Signals) into the strength and pitch of each vibration and into the scalings of RBR's own
// steering force. Pure logic: no Windows, no game access, no files. tests/synth_golden_test.cpp drives it through scripted drives.
#pragma once
#include <algorithm>
#include <cmath>
#include "core/common.hpp"
#include "core/config.hpp"
#include "effects/accel_light.hpp"
#include "effects/brake_fx.hpp"
#include "effects/crash_fx.hpp"
#include "effects/force_scale.hpp"
#include "effects/rev_limit.hpp"
#include "effects/water_fx.hpp"
#include "effects/weight_transfer.hpp"
#include "road/road_bands.hpp"    // the road vibration as three sine bands: pitch from speed, strength spread by the ground
#include "road/road_feel.hpp"     // how the ground and what the car is doing shape the road vibration
#include "road/surface_feel.hpp"  // the surface classes and the id-to-class table (the plugin's surface reader extends the same Surf namespace)
#include "telemetry/tyre_motion.hpp"

namespace ForceFX {

// The slots the synthesis fills: the engine's two sines, the road's three bands and the brake judder.
enum Slot { S_ENG1 = 0, S_ENG2, S_ROAD1, S_ROAD2, S_ROAD3, S_BRAKE, S_COUNT };
const bool kSlotIsEffect[S_COUNT] = {true, true, false, true, false, true};
const char* const kSlotName[S_COUNT] = {"engine", "lump", "road-low", "road", "road-high", "brake"};

// What the analysis of one telemetry packet hands to the synthesis.
struct Signals {
    float rpm = 0, speed = 0 /* m/s */, throttle = 0, brake = 0;
    float rpmMaxTelem = 0;   // rev limit straight from the telemetry (0 = not available)
    float rough = 0;      // raw front-weighted tyre roughness
    float piston = 0;     // damper piston velocity level
    float texture = 0;    // combined road texture 0..1
    float texL = 0, texR = 0;   // the same, per side (left / right tyres)
    float bump = 0;       // heave acceleration spike envelope
    float dive = 0;       // front load transfer under braking 0..1 (the synth takes it from the shared front-load tracker; only the test mode sets it here)
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
    float latG = 0;                 // sideways acceleration in g, absolute (0 until the units are known)
    float ground = 0;               // the car's speed over the ground, m/s (0 = not known)
    float thumpL = 0, thumpR = 0;   // compression / landing envelope per side
    unsigned evtSeq = 0;  // goes up by one for every landing; the synth starts one boost per event
    float evtSev = 0;     // severity of the current event 0..1 (refined upward while a landing is still compressing)
    unsigned crashSeq = 0;   // goes up by one for every crash (the car hitting something); the synth starts one jolt per event
    float crashSev = 0;      // severity of the last crash 0..1
    float staticLoad = 0; // learned static strut load, all four wheels (units of the telemetry)
};

// -------------------------------------------------------------- synthesis
// Turns the analysed signals into effect strengths and frequencies. Runs on the worker thread, about 100 times a second.
// ====================================================================================================================
// ROAD FEEL: how the road vibration is shaped by the ground and by what the car is doing.
//
// There is no list of "gravel feels like this, tarmac like that". The game's contact data names the material under each tyre
// (for example Gravel_Coarse_Deep_Wet), and road_feel.hpp turns that name into a few physical properties: how hard the ground
// is, how much loose material lies on it, how coarse it is and how slippery. The road effect then follows from those together
// with the telemetry: the measured roughness of the tyres sets how strong it is, speed and wheel slip decide how hard loose
// ground is worked (stones flung, grain). Everything
// reshapes the existing road effect, so no new effects are added and the force stays zero-mean. Unknown ground is plain hard
// ground. The one setting for all of it is the road gain; the switch is in the menu (Plugins > ForceFX).
// ====================================================================================================================
static_assert(Water::kBands == Road::kBands, "the water effects ride on the same bands as the road");

// How far each side's road strength follows its own tyres (0 = both sides identical, 1 = left and right fully separate).
constexpr float kRoadDirectional = 0.7f;

struct Synth {
    float sm[S_COUNT] = {};
    Road::Ground gnd[4];                                   // the ground under each tyre (FL, FR, RL, RR), moved gradually towards what was read
    Tyre::GripLearner grip;                                // how hard the car can accelerate on each kind of ground (reset with the car)
    float spinSm[4] = {}, slideSm[4] = {}, lockSm[4] = {};   // wheelspin, sliding and lock-up of each tyre (FL, FR, RL, RR), 0..1, smoothed
    float lockAmt = 0;                                     // lock-up for the brake effect, 0..1
    float lockFront = 0;                                   // lock-up of the two front tyres, 0..1: it lightens RBR's force
    float thrSm = 0, lope = 0, rpmPeak = 0, heat = 0;
    float jAge = 5.0f, evAmp = 0.0f;                       // age and severity of the current landing event
    unsigned seenEvt = 0;
    float evBusy = 0.0f, evGov = 1.0f;                     // how many landings and hits came lately, and how much the current one is held back because of them
    float boostMul = 1.0f, boostAdd = 0.0f;                // all the scalings of RBR's own force together (the worker thread hands it to the hook)
    ForceScale::Limiter fscale;                            // keeps that product in bounds (force_scale.hpp)
    CrashFx::Envelope crashEnv;                            // the jolt of a crash
    RevLimit::Learner revLearn;                            // rev limit learned from the highest rpm held, when no telemetry or profile knows it
    float bandHzJit[Road::kBands] = {1.0f, 1.0f, 1.0f}, bandAmpJit[2][Road::kBands] = {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};   // random pitch per band, random strength per side and band
    float engJit = 1.0f, engHzJit = 1.0f, unevPhase = 0;
    double nextJit = 0;
    Water::Tracker water;                                  // splash, wading and aquaplaning from the standing water under each tyre
    Water::Out wout;                                       // its result for the current step
    unsigned waterSeenEvt = 0;                             // last landing event the water tracker has seen
    float waterJit[2][Water::kBands] = {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};   // random strength per side and band for the wading: slaps, not a hum
    AccelLight::Tracker accel; AccelLight::Out aout;       // front load under acceleration: how light the steering should go
    WeightTransfer::Lateral lateral; WeightTransfer::Out lout;   // side load: weight built up in a corner and swinging across the car
    double wtLogT = -10.0; int wtLogN = 0;                 // when the last weight-transfer line was logged, and how many were
    double accLogT = -10.0; int accLogN = 0;               // when the last light-steering line was logged, and how many were
    bool accWasRest = false; int accRestLogs = 0;          // the standing front share was known on the last step, and how many times it was logged
    unsigned waterEvents = 0; float waterEntry = 0.0f; int waterEntryTyre = -1; bool waterEntryIn = false;   // goes up for every splash; strength, tyre and whether it was going in (not out) for the last one (for sound and glass)

    // The targets of the two engine sines (the strength each should reach), before the smoothing in Finish.
    struct EngineOut { float low, lump; };
    // What the road step hands on: the three band targets (road, then water on top), how strongly each side feels the road, and the ground feel per side.
    struct RoadOut { float target[Road::kBands], water[Road::kBands], strength[2]; Surf::Feel fes[2]; };

    // One step, about 100 times a second. The steps run in this order on purpose: they share the random numbers (rnd) and the trackers
    // below, and a step may read what an earlier one left in the members (the water, the front load, the lock-up). Each step is one block
    // of the effect, with its own comment.
    void Run(const Config& c, const Signals& s, double t, float dt, float userGain, int mag[S_COUNT], float hz[S_COUNT]) {
        thrSm += (s.throttle - thrSm) * alphaFor(dt, 0.12f);
        const EngineOut eng = StepEngine(c, s, t, dt, hz);
        StepWater(c, s, dt);
        StepFrontLoad(c, s, t, dt);
        StepSideLoad(c, s, t, dt);
        StepTyres(c, s, dt);
        const RoadOut road = StepRoad(c, s, t, dt, hz);
        const float brake = StepBrake(c, s, road, hz);
        StepForce(c, s, dt, userGain);
        Finish(c, eng, road, brake, dt, userGain, mag);
    }

    static float WeightFeel(const Config& c) { return clampf(c.weightFeel, 0.5f, 2.0f); }   // how heavy the car feels: 1 = average

    EngineOut StepEngine(const Config& c, const Signals& s, double t, float dt, float hz[S_COUNT]) {
        // ---------------- ENGINE: both strength and pitch follow the engine
        rpmPeak = revLearn.Update(s.rpm, dt);
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
        return {t1, t2};
    }

    void StepWater(const Config& c, const Signals& s, float dt) {
        // ---------------- WATER: which tyres are in standing water decides the splash on entry, the wading rumble and the planing.
        // Only the ground the surface scan found counts (Water_Surface and the ids in WaterIds=), so wet tarmac is left to the road feel.
        {
            Water::In wi;
            for (int w = 0; w < 4; ++w) wi.depth[w] = c.fxWater ? Water::DepthOfId(s.mat[w]) : 0.0f;
            wi.speed = s.speed; wi.maxSpeed = c.maxSpeed; wi.airborne = s.airborne;
            if (s.evtSeq != 0) { wi.landed = s.evtSeq != waterSeenEvt; wi.landSev = s.evtSev; waterSeenEvt = s.evtSeq; }   // no telemetry reads as sequence 0: not a landing
            if (!c.fxWater) { water.Reset(); wout = Water::Out(); }   // off: forget everything, so switching it on in the middle of a ford is not a splash
            else wout = water.Update(wi, dt);
            if (wout.entry > 0.0f) { waterEntry = wout.entry; waterEntryTyre = wout.entryTyre; waterEntryIn = wout.entryTyre >= 0 && wi.depth[wout.entryTyre] >= 0.5f; ++waterEvents; }
        }
    }

    void StepFrontLoad(const Config& c, const Signals& s, double t, float dt) {
        // ---------------- LIGHT STEERING FROM THE FRONT LOAD: the share of the weight on the front tyres (from the four strut forces) against what it is standing still on the
        // start line. Hard acceleration moves weight to the rear, and the steering loses force in proportion. Scales RBR's own force down (see boost below).
        {
            AccelLight::In ai;
            ai.frontShare = s.wload[0] + s.wload[1]; ai.throttle = s.throttle; ai.brake = s.brake; ai.speed = s.ground > 0.0f ? s.ground : s.speed;
            ai.airborne = s.airborne; ai.accMag = s.accMag; ai.accOk = s.accOk;
            if (!c.fxAccelLight && !c.fxBrake) { accel.Reset(); aout = AccelLight::Out(); }   // it also measures the dive for the brake feel
            else aout = accel.Update(ai, dt);
            if (c.fxAccelLight && aout.fromRest && !accWasRest && accRestLogs < 10) {   // the car stood still long enough: say what it read, so a start on a slope shows in the log
                ++accRestLogs;
                Logf("Light steering: standing front share %.3f (acceleration reads %.2f m/s^2 standing%s)", (double)aout.base, (double)ai.accMag, ai.accOk ? "" : ", units not known yet");
            }
            accWasRest = aout.fromRest;
            if (c.fxAccelLight && aout.rawDrop > 0.04f && aout.drop > 0.02f && t - accLogT > 1.5 && accLogN < 60) {   // how much the front really unloads, to tune AccelLightness by
                accLogT = t; ++accLogN;
                Logf("Light steering: front load %.0f%% below standing (standing %.2f, now %.2f), accelerating %.2f g, %.0f km/h -> steering force x%.2f", (double)(100.0f * aout.rawDrop), (double)aout.base, (double)ai.frontShare, (double)aout.accelG, (double)(ai.speed * 3.6f), (double)AccelLight::Mul(aout, c.accelLight));
            }
        }
    }

    void StepSideLoad(const Config& c, const Signals& s, double t, float dt) {
        // ---------------- SIDE LOAD: the weight across the car, from the same strut-force shares. Scales RBR's force (swell in a corner, surge in a direction change).
        {
            WeightTransfer::In wi;
            wi.lateral = (s.wload[1] + s.wload[3]) - (s.wload[0] + s.wload[2]); wi.speed = s.ground > 0.0f ? s.ground : s.speed;
            wi.airborne = s.airborne; wi.latG = s.latG; wi.latOk = s.accOk;
            if (!c.fxWeight) { lateral.Reset(); lout = WeightTransfer::Out(); }
            else lout = lateral.Update(wi, dt);
            if (c.fxWeight && (lout.swell > 0.5f || lout.surge > 0.4f) && t - wtLogT > 1.5 && wtLogN < 60) {   // how far the weight really moves, to tune WeightTransferGain by
                wtLogT = t; ++wtLogN;
                Logf("Weight transfer: side load %+.0f%% of the weight, %.2f g sideways, swell %.2f, surge %.2f -> steering force x%.2f", (double)(100.0f * lout.load), (double)s.latG, (double)lout.swell, (double)lout.surge, (double)WeightTransfer::LateralMul(lout, c.weightGain));
            }
        }
    }

    void StepTyres(const Config& c, const Signals& s, float dt) {
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
            lockFront = BrakeFx::FrontLock(lockSm);       // the front tyres only: that is what the steering loses
        }
    }

    RoadOut StepRoad(const Config& c, const Signals& s, double t, float dt, float hz[S_COUNT]) {
        // ---------------- ROAD: three sine bands (low, mid, high) of the road's spectrum. The pitch of each band follows the car's
        // speed, the ground under each side decides how the strength is spread over the bands, and the tyre's contact patch takes
        // the high bands away at low speed (road_bands.hpp). The sines are zero-mean, so they never shift RBR's own force, and the
        // two sides add as energy, so they can not cancel each other. How the bands reach the wheel is decided in the worker loop.
        const float spN = clamp01(s.speed / c.maxSpeed);
        const float gSpd = smooth01(1.0f, 10.0f, s.speed);

        const auto sideDyn = [](const float* a, int k2) { return std::max(a[k2], 0.6f * a[2 + k2]); };   // front tyre of the side is k2, rear tyre 2 + k2
        const float texMono = clamp01(s.texture + 0.9f * s.bump);
        const float wf = WeightFeel(c);
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
            for (int w = 0; w < 4; ++w) gnd[w] = Road::Mix(gnd[w], Road::GroundForTyre(s.mat[w], s.surf[w]), aw);
            const float fw = Surf::g_front;
            for (int k2 = 0; k2 < 2; ++k2) {                                          // front tyre of this side is wheel k2, rear tyre is wheel 2 + k2
                Road::Drive dr;
                dr.spin = sideDyn(spinSm, k2); dr.slide = sideDyn(slideSm, k2);
                dr.activity = Road::Activity(spN, std::max(dr.spin, dr.slide));
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
            for (int k2 = 0; k2 < 2; ++k2) for (int b = 0; b < Water::kBands; ++b) waterJit[k2][b] = 0.35f + 0.65f * (rnd() * 0.5f + 0.5f);
            nextJit = t + 0.012;
        }
        float roadTarget[Road::kBands];
        Road::Combine(rside, plan.share, bandAmpJit, roadTarget);
        { const float cs = Road::CeilingScale(roadTarget, (float)c.roadMaxMag); for (int b = 0; b < Road::kBands; ++b) roadTarget[b] *= cs; }   // very rough ground can not make the road grow without limit
        // Water rides on the same three bands, after the road's ceiling: a splash is a hit and wading is capped well below one, and both add to the road as energy.
        float waterTarget[Road::kBands] = {0.0f, 0.0f, 0.0f};
        if (c.fxWater) Water::BandMagnitudes(wout, c.waterGain, waterJit, waterTarget);
        for (int b = 0; b < Road::kBands; ++b)
            hz[S_ROAD1 + b] = clampf(plan.hz[b] * bandHzJit[b] / std::sqrt(wf), 20.0f, kHardMaxHz);   // heavier cars sound lower
        RoadOut o;
        for (int b = 0; b < Road::kBands; ++b) { o.target[b] = roadTarget[b]; o.water[b] = waterTarget[b]; }
        for (int k2 = 0; k2 < 2; ++k2) { o.strength[k2] = rside[k2].strength; o.fes[k2] = fes[k2]; }
        return o;
    }

    float StepBrake(const Config& c, const Signals& s, const RoadOut& road, float hz[S_COUNT]) {
        // ---------------- BRAKE: front-end judder, harsh on lock-up. It rides on RBR's force as one more band of the road vibration (see the worker loop), and
        // lock-up of the front tyres also lightens RBR's force (below). The weight on the front comes from the shared front-load tracker.
        const float gB = smooth01(2.0f, 8.0f, s.speed);
        // Lock-up shakes the wheel according to the ground: soft ground swallows it, hard ground passes it on.
        const float lockU = std::max(s.lock, lockAmt) * (c.fxSurface ? clampf(0.5f * (road.fes[0].amp + road.fes[1].amp), 0.4f, 1.3f) : 1.0f);
        const float Ab = BrakeFx::JudderAmp(s.brake, std::max(s.dive, aout.dive), lockU);
        const float gHard = c.fxSurface ? 0.5f * (gnd[0].hard + gnd[1].hard) : 0.4f, gLoose = c.fxSurface ? 0.5f * (gnd[0].loose + gnd[1].loose) : 0.6f;   // the ground under the front tyres sets the pitch of a lock-up scrub
        hz[S_BRAKE] = BrakeFx::JudderHz(lockU, s.frontRev, gHard, gLoose, lockU > 0.1f ? rnd() : 0.0f, c.brakeHzScale);
        float t4 = c.brakeMaxMag * Ab * gB * clampf(0.65f + 0.35f * s.loadF, 0.5f, 1.3f);   // more load on the fronts means more judder
        return t4;
    }

    // Every scaling of RBR's own force: landing, water, front load, side load, lock-up, crash (combined and bounded by force_scale.hpp).
    void StepForce(const Config& c, const Signals& s, float dt, float userGain) {
        const float wf = WeightFeel(c);
        // ---------------- LANDING: no effect of its own. A detected landing boosts RBR's own steering force (see EffectTap):
        // a fast attack, a short hold, then a decay, scaled by how severe the landing was and by the live level.
        // A jump is rare, rough ground is not: bumps and washboard fire hit events one after another, and boosting RBR's force for each
        // of them makes the wheel heavy to turn. So the more events came in the last second or two, the less a new one counts. A single
        // jump is not held back at all; a second hit soon after is, and on rough ground the boost fades to almost nothing.
        evBusy *= (float)std::exp(-dt / 1.5f);
        if (s.evtSeq != seenEvt) { seenEvt = s.evtSeq; jAge = 0.0f; evGov = 1.0f / (1.0f + 1.2f * evBusy); evBusy += 1.0f; evAmp = s.evtSev * evGov; }
        if (jAge < 0.08f) evAmp = std::max(evAmp, s.evtSev * evGov);                 // the detector keeps refining a landing for its first ~100 ms
        jAge = std::min(jAge + dt, 5.0f);
        ForceScale::Parts fp;                                                       // every scaling of RBR's own force, combined and bounded below (force_scale.hpp)
        float addRaw = 0.0f;
        const float ugc = clampf(userGain, 0.0f, 1.0f);
        {
            const float ts = std::sqrt(wf);                                           // a heavier car holds the boost longer
            const float hold = 0.06f * ts, dec = 0.30f * ts;
            const float env = jAge < hold ? 1.0f : (float)std::exp(-(jAge - hold) / dec);
            const float lev = (c.fxLandBoost && jAge < 1.5f) ? clamp01(evAmp) * env * ugc : 0.0f;
            fp.landing = 1.0f + c.landingBoost * lev;
            addRaw = c.landingAdd * lev;
        }
        // Water scales RBR's own force the same way: a kick on a splash (a moment of more), and heavy steering for as long as the front tyres are in the water.
        if (c.fxWater) {
            fp.waterKick = Water::KickMul(wout, c.waterGain * ugc);
            fp.waterHeavy = Water::HeavyMul(wout, c.waterHeavy * ugc);
        }
        // The front load: lighter while hard acceleration unloads the front, heavier in hard braking (off unless wanted: RBR already does this).
        fp.frontLoad = WeightTransfer::FrontMul(aout, c.fxAccelLight ? c.accelLight * ugc : 0.0f, c.fxBrake ? c.brakeHeavy * ugc : 0.0f);
        if (c.fxBrake) fp.brakeLock = BrakeFx::LockMul(lockFront, c.lockLight * ugc);   // lighter while the front tyres are locked
        if (c.fxWeight) fp.lateral = WeightTransfer::LateralMul(lout, c.weightGain * ugc);               // heavier as the weight builds up on the outside and swings across the car
        if (c.fxCrash) {                                                              // a jolt when the car hits something
            const float lev = crashEnv.Step(s.crashSeq, s.crashSev, dt, ugc);
            fp.crash = CrashFx::Mul(lev, c.crashBoost);
            addRaw += CrashFx::Add(lev, c.crashAdd);
        } else crashEnv.Reset();
        boostMul = fscale.Step(ForceScale::Product(fp), dt, c.maxForceMul);
        boostAdd = ForceScale::Add(addRaw);
    }

    // Smooths the targets, lets the road take priority over the engine, applies the ceiling, the thermal derate and the live level.
    void Finish(const Config& c, const EngineOut& eng, const RoadOut& road, float t4, float dt, float userGain, int mag[S_COUNT]) {
        const float wf = WeightFeel(c);
        float t1 = eng.low, t2 = eng.lump;
        // attack and release: road is deliberately quick (for sharpness), engine is rounder
        auto follow = [&](int i, float target, float up, float down) {
            sm[i] += (target - sm[i]) * (target > sm[i] ? up : down);
        };
        if (!c.fxEngine) { t1 = 0.0f; t2 = 0.0f; }
        follow(S_ENG1, t1, 0.35f, 0.15f);
        follow(S_ENG2, t2, 0.25f, 0.12f);
        const float roadDown = clampf(0.40f / wf, 0.15f, 0.70f);                     // heavier cars let the thump ring out longer
        // The road follows the ground under the side that is feeling more of it (equal weight when both are silent).
        const float sumSt = road.strength[0] + road.strength[1], w0 = sumSt > 1e-6f ? road.strength[0] / sumSt : 0.5f;
        const float roadAtk = w0 * road.fes[0].atk + (1.0f - w0) * road.fes[1].atk;
        const float roadRel = clampf(roadDown * (w0 * road.fes[0].rel + (1.0f - w0) * road.fes[1].rel), 0.05f, 0.95f);   // gravel and ice let go fast, snow slowly
        const float waterAtk = std::max(roadAtk, 0.55f * std::max(wout.splash[0], wout.splash[1]));   // a splash comes in hard
        for (int b = 0; b < Road::kBands; ++b) {
            const float rt = c.fxRoad ? road.target[b] : 0.0f;
            follow(S_ROAD1 + b, std::sqrt(rt * rt + road.water[b] * road.water[b]), waterAtk, roadRel);
        }
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

}  // namespace ForceFX
