// FRONT LOAD TRACKER, shared by light steering (acceleration) and the brake feel (dive and heavy braking).
// Light steering under hard acceleration: the weight moves to the rear, the front tyres have less load to push against, and the steering force falls
// with the load on the front axle. This scales RBR's own steering force down in proportion to how much front load the telemetry says has gone, only
// while the telemetry says the car is accelerating forward, and never adds a force of its own. Pure logic, no Windows or game access, tested in
// tests/accel_light_test.cpp.
//
// Everything comes from the telemetry, nothing from the pedals:
//   - The front share of the car's weight is read from the four strut forces (the share on the two front tyres, 0..1).
//   - "Normal" is that share with the car standing still, which the car does on the start line before every stage, so it is known from the first
//     moment of the launch. It is refreshed whenever the car stands still at idle and ignored while the engine is being revved (the car squats),
//     so it is never taught by a launch. A car that is already moving when the plugin starts learns it from steady driving instead (about one
//     second without hard acceleration, braking or cornering).
//   - A slope moves the standing share a little (a 10% grade moves it by about a point or two, against several points for hard acceleration), so
//     a start on a hill is slightly off. It is corrected as soon as the car rolls steadily: while driving without hard acceleration, braking or
//     cornering, "normal" follows the share slowly (about 6 s), so it always means what the front carries at an even pace on the slope the car is
//     on, and it is frozen during hard acceleration so a launch never teaches it.
//   - The drop is how far the share has fallen below normal, relative: 0.2 means the front carries 20% less of the weight than it does standing.
//     The steering force is scaled by (1 - drop), so at gain 1 the steering loses force in proportion to the front load it has lost, as the tyres
//     would, and never more than kMaxCut of it.
//   - Only a drop counts. Braking and downhill put more weight on the front, which makes the steering heavier in a real car; that is left to RBR.
//   - Only while the car is accelerating forward, judged from the ground speed in the telemetry (its rate of change, smoothed over about a quarter of
//     a second): from 0.06 g the effect starts to come in and from 0.22 g it is fully on. The load share is noisy on a rough road (every bump and damper
//     stroke moves it), and without this the noise alone would flicker the steering light on the straights. Tested against a simulated stage, the
//     gate removes that false lightening at every noise level and costs nothing on a launch, and it needs only a coarse acceleration, so it works with
//     noisy speed telemetry.
//   - The share is smoothed over about 0.07 s so a bump does not flicker the steering, and the drop fades in over about the same and out over
//     a quarter of a second, so it never snaps. A few percent of drop is ignored (noise in the strut forces).
//   - The other direction is the same measurement the other way up: braking puts weight on the front. `dive` is how far the front share is above
//     normal while the brake is pressed (0.10 of share = 1.0), and `heavy` is the same, only while the car is really slowing down (judged from the
//     ground speed like the acceleration gate), which is what makes the steering heavier. The brake feel reads these, so there is one learner of the
//     front load in the plugin.
#pragma once
#include <algorithm>
#include <cmath>

namespace AccelLight {

constexpr float kMaxCut = 0.6f;        // the steering never loses more than this share of RBR's force
constexpr float kDeadband = 0.02f;     // relative drop below this is noise in the strut forces

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
inline float alphaFor(float dt, float tau) { return 1.0f - std::exp(-dt / tau); }
inline float smooth01(float a, float b, float x) { const float t = c01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
}  // namespace detail

struct In {
    float frontShare = 0.5f;   // share of the car's weight on the two front tyres, 0..1
    float throttle = 0.0f, brake = 0.0f;
    float speed = 0.0f;        // m/s over the ground
    float airborne = 0.0f;     // 0 on the ground .. 1 in the air
    float accMag = 0.0f;       // the car's acceleration over the ground in m/s^2 ...
    bool  accOk = false;       // ... when the units are known (used only to judge steady driving for a car that starts moving)
};
struct Out {
    bool  learned = false;     // the normal front share is known
    bool  fromRest = false;    // ... and it was measured with the car standing still
    float base = 0.0f;         // the normal front share
    float share = 0.0f;        // the front share now, smoothed
    float drop = 0.0f;         // the share of the steering force that goes at gain 1 (0 = nothing): the relative drop below normal, past the dead band and the acceleration gate, smoothed
    float accelG = 0.0f;       // the car's forward acceleration from the ground speed, in g, smoothed (negative when braking)
    float gate = 0.0f;         // 0..1: how far the acceleration lets the effect through
    float rawDrop = 0.0f;      // the same as measured right now: not smoothed, no dead band (0.2 = the front carries 20% less than normal). For the log
    float dive = 0.0f;         // 0..1: weight moving onto the front while the brake is pressed (0.10 of share above normal = 1), smoothed over about 0.1 s
    float heavy = 0.0f;        // 0..1: the same, only while the ground speed says the car is really slowing down (from 0.06 g, fully from 0.22 g)
};

class Tracker {
public:
    static constexpr float kRestSpeed = 0.5f;     // m/s: slower than this the car is standing
    static constexpr float kRestNeeded = 0.4f;    // seconds standing at idle before the standing share is trusted
    static constexpr float kSteadyNeeded = 1.0f;  // seconds of steady driving before it is trusted, for a car that was already moving

    void Reset() { *this = Tracker(); }

    Out Update(const In& in, float dt) {
        using namespace detail;
        Out o;
        if (!std::isfinite(dt) || dt <= 0.0f) dt = 0.01f;
        if (dt > 0.1f) dt = 0.1f;
        const float share = std::isfinite(in.frontShare) ? std::min(0.95f, std::max(0.05f, in.frontShare)) : 0.5f;
        const float thr = c01(in.throttle), brk = c01(in.brake), air = c01(in.airborne);
        const float spd = std::isfinite(in.speed) && in.speed > 0.0f ? in.speed : 0.0f;

        // Forward acceleration from the ground speed. A jump in the speed (a crash, a respawn) is limited so it cannot look like a launch.
        const float acc = haveSpeed_ ? std::max(-30.0f, std::min(30.0f, (spd - vPrev_) / dt)) : 0.0f;
        vPrev_ = spd; haveSpeed_ = true;
        accG_ += (acc / 9.81f - accG_) * alphaFor(dt, 0.25f);
        const float gate = smooth01(0.06f, 0.22f, accG_);
        o.accelG = accG_; o.gate = gate;

        if (!haveShare_) { sm_ = share; haveShare_ = true; }
        else sm_ += (share - sm_) * alphaFor(dt, 0.07f);

        // Normal: the front share standing still at idle (the engine being revved squats the car, so that is not learned), else steady driving.
        const bool standing = spd < kRestSpeed && air < 0.1f && thr < 0.30f;
        const bool steady = spd > 3.0f && air < 0.1f && thr < 0.35f && brk < 0.05f && (!in.accOk || (std::isfinite(in.accMag) && in.accMag < 1.5f));
        if (standing) {
            restT_ += dt;
            if (!haveRest_) { base_ = sm_; haveBase_ = true; }                    // first look at the standing car: take it at once
            else base_ += (sm_ - base_) * alphaFor(dt, 0.5f);
            if (restT_ >= kRestNeeded) haveRest_ = true;
        } else {
            restT_ = 0.0f;
            if (steady) {
                steadyT_ += dt;
                if (!haveBase_) { base_ = sm_; haveBase_ = true; }
                else base_ += (sm_ - base_) * alphaFor(dt, haveRest_ ? 6.0f : (steadyT_ < 0.6f ? 0.3f : 4.0f));   // a car that has stood follows slowly; one that has not catches up quickly
            }
        }
        o.fromRest = haveRest_;
        o.learned = haveRest_ || (haveBase_ && steadyT_ >= kSteadyNeeded);
        o.base = base_;
        o.share = sm_;

        float target = 0.0f;
        if (o.learned && base_ > 0.05f) {
            o.rawDrop = std::max(0.0f, (base_ - share) / base_);
            const float d = std::max(0.0f, (base_ - sm_) / base_ - kDeadband);
            target = std::min(kMaxCut, d) * gate * (1.0f - air);
        }
        drop_ += (target - drop_) * alphaFor(dt, target > drop_ ? 0.07f : 0.25f - 0.20f * air);   // in the air it lets go quickly
        if (!(drop_ > 0.0f)) drop_ = 0.0f;
        o.drop = drop_;

        // Braking: the share rising above normal while the brake is pressed. Measured against the raw share, as the weight really moves.
        const float diveT = (o.learned && base_ > 0.05f && brk > 0.1f) ? c01((share - base_) / 0.10f) : 0.0f;
        dive_ += (diveT - dive_) * alphaFor(dt, 0.1f);
        o.dive = dive_;
        o.heavy = dive_ * smooth01(0.06f, 0.22f, -accG_) * (1.0f - air);
        return o;
    }

private:
    bool haveShare_ = false, haveBase_ = false, haveRest_ = false, haveSpeed_ = false;
    float sm_ = 0.5f, base_ = 0.5f, restT_ = 0.0f, steadyT_ = 0.0f, drop_ = 0.0f, dive_ = 0.0f, vPrev_ = 0.0f, accG_ = 0.0f;
};

// RBR's force times this (<= 1). `gain` is the setting: 0 = off, 1 = in proportion to the front load lost, at most 2; never more than kMaxCut off.
inline float Mul(const Out& o, float gain) {
    const float g = std::isfinite(gain) && gain > 0.0f ? (gain > 2.0f ? 2.0f : gain) : 0.0f;
    const float cut = g * (std::isfinite(o.drop) && o.drop > 0.0f ? o.drop : 0.0f);
    return 1.0f - (cut > kMaxCut ? kMaxCut : cut);
}

}  // namespace AccelLight
