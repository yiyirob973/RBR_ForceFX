// WEIGHT TRANSFER: where the car's weight is, felt as a change in how heavy RBR's own steering force is. One tracker reads the four strut forces, one
// function turns what it found into one multiplier for RBR's force. Pure logic, no Windows or game access, tested in tests/weight_transfer_test.cpp.
// Nothing here adds a force of its own or pulls the wheel in a direction: RBR's force is only scaled, so it always points the way RBR sent it.
//
// The model follows the tyres. The steering force a tyre makes (its self-aligning torque) grows with the load it carries, so the steering is as heavy as
// the load on the front tyres is: lighter when the front has lost load, heavier when it has gained, in proportion (front load retention, Fz / Fz0). The
// load moves in two directions, and both are read from the same four numbers (the share of the weight on each tyre, from the strut forces):
//
//   ALONG THE CAR   the share on the front tyres against what it is standing still. Acceleration moves weight to the rear (the front share falls, the
//                   steering goes light), braking moves it forward (it rises, the steering goes heavy). One measure, one proportion, both ways; only
//                   the gate differs, and each way is gated by the ground speed saying the car is really accelerating or slowing down.
//   ACROSS THE CAR  (right share - left share). Which side does not matter, only how far it has moved.
//     swell         the weight built up on the outside tyres in a corner: the steering force gets a little heavier (mechanical trail grows with the lateral
//                   force, which grows with the load), following the body roll a moment after the steering and easing off as the corner ends.
//     surge         the weight swinging from one side to the other in a direction change (the pendulum of a flick): RBR's force swells while the load
//                   crosses the car. Detected as the quick average of the side load having the opposite sign to the slow one.
//   Trail braking is both at once: the front is loaded by the braking and the outside by the corner, and the two multiply.
//
// Everything is learned from the telemetry, nothing from the pedals:
//   - "Normal" is the front share with the car standing still, which it does on the start line before every stage, so it is known from the first moment
//     of the launch. It is refreshed whenever the car stands still at idle and ignored while the engine is being revved (the car squats), so it is never
//     taught by a launch. A car that is already moving when the plugin starts learns it from steady driving instead (about one second without hard
//     acceleration, braking or cornering).
//   - A slope moves the standing share a little (a 10% grade moves it by about a point or two, against several points for hard acceleration), so a start
//     on a hill is slightly off. While driving without hard acceleration, braking or cornering, "normal" follows the share slowly (about 6 s), so it always
//     means what the front carries at an even pace on the slope the car is on; it is frozen during hard acceleration so a launch never teaches it.
//   - The change is relative: 0.2 means the front carries 20% less (or more) of the weight than it does standing. The steering force changes by the same
//     share at gain 1, as the tyres would, never more than kMaxCut less or kMaxHeavy more.
//   - Only while the car really accelerates or slows down, judged from the ground speed (its rate of change, smoothed over about a quarter of a second):
//     from 0.06 g the effect starts to come in and from 0.22 g it is fully on. The load share is noisy on a rough road (every bump and damper stroke
//     moves it), and without this the noise alone would flicker the steering on the straights. A coarse acceleration is enough, so it works with noisy
//     speed telemetry. Downhill at a steady speed puts weight on the front too, but that is not braking, and it is left to RBR.
//   - The share is smoothed over about 0.07 s so a bump does not flicker the steering, and a change fades in over about the same and out over a quarter
//     of a second, so it never snaps (in the air it lets go quickly). A few percent is ignored (noise in the strut forces).
//   - The side effects need real lateral acceleration (from 0.08 g, fully at 0.25 g), because the strut forces are noisy on a rough road and the noise
//     alone would otherwise swell the steering on a straight. They fade in with speed.
//   - Landings and crests unload every tyre for a moment: the effects let go in the air, and the landing itself is handled by the landing logic.
#pragma once
#include <algorithm>
#include <cmath>

namespace WeightTransfer {

constexpr float kMaxCut = 0.6f;        // the steering never loses more than this share of RBR's force from the front load
constexpr float kMaxHeavy = 0.6f;      // ... and never gains more than this
constexpr float kDeadband = 0.02f;     // a relative front load change below this is noise in the strut forces
constexpr float kSwellAdd = 0.25f;     // at gain 1 and full cornering load, RBR's force grows by this share ...
constexpr float kSurgeAdd = 0.35f;     // ... and by this much while the load swings across the car
constexpr float kMaxCorner = 0.5f;     // never more than this share of RBR's force from the side load together, whatever the gain

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
inline float alphaFor(float dt, float tau) { return 1.0f - std::exp(-dt / tau); }
inline float smooth01(float a, float b, float x) { const float t = c01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
inline float gain(float g) { return std::isfinite(g) && g > 0.0f ? (g > 2.0f ? 2.0f : g) : 0.0f; }
inline float pos(float v) { return std::isfinite(v) && v > 0.0f ? v : 0.0f; }
}  // namespace detail

struct In {
    float wload[4] = {0.25f, 0.25f, 0.25f, 0.25f};   // the share of the car's weight on each tyre (FL, FR, RL, RR), summing to 1
    float throttle = 0.0f, brake = 0.0f;
    float speed = 0.0f;        // m/s over the ground
    float airborne = 0.0f;     // 0 on the ground .. 1 in the air
    float accMag = 0.0f;       // the car's acceleration over the ground in m/s^2 ...
    bool  accOk = false;       // ... when the units are known (used to judge steady driving for a car that starts moving)
    float latG = 0.0f;         // sideways acceleration in g (absolute) ...
    bool  latOk = false;       // ... when its units are known; without it the side load is not trusted and nothing is added for it
};
struct Out {
    // along the car
    bool  learned = false;     // the normal front share is known
    bool  fromRest = false;    // ... and it was measured with the car standing still
    float base = 0.0f;         // the normal front share
    float share = 0.0f;        // the front share now, smoothed
    float rawChange = 0.0f;    // the front load against normal right now, not smoothed, no dead band: -0.2 = 20% less, +0.2 = 20% more. For the log
    float light = 0.0f;        // the relative front load lost, past the dead band and the acceleration gate, smoothed (the steering force loses this share at gain 1)
    float heavy = 0.0f;        // the relative front load gained, past the dead band and the braking gate, smoothed
    float accelG = 0.0f;       // forward acceleration from the ground speed, in g, smoothed (negative when braking)
    float gate = 0.0f;         // 0..1: how far forward acceleration lets the front-load-lost effect through (the braking one has its own, the same the other way up)
    float dive = 0.0f;         // 0..1: weight moving onto the front while the brake is pressed (0.10 of share above normal = 1), smoothed over about 0.1 s. The brake judder reads it
    // across the car
    float side = 0.0f;         // the side load now, quick average, minus the standing offset (for the log)
    float swell = 0.0f;        // 0..1: weight built up on the outside tyres, smoothed
    float surge = 0.0f;        // 0..1: weight swinging across the car, smoothed
    float sideGate = 0.0f;     // 0..1: how far the lateral acceleration and the speed let the side effects through
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
        const float front = in.wload[0] + in.wload[1];
        const float sideRaw = (in.wload[1] + in.wload[3]) - (in.wload[0] + in.wload[2]);
        const float thr = c01(in.throttle), brk = c01(in.brake), air = c01(in.airborne);
        const float spd = std::isfinite(in.speed) && in.speed > 0.0f ? in.speed : 0.0f;

        // Forward acceleration from the ground speed. A jump in the speed (a crash, a respawn) is limited so it cannot look like a launch.
        const float acc = haveSpeed_ ? std::max(-30.0f, std::min(30.0f, (spd - vPrev_) / dt)) : 0.0f;
        vPrev_ = spd; haveSpeed_ = true;
        accG_ += (acc / 9.81f - accG_) * alphaFor(dt, 0.25f);
        const float gateAccel = smooth01(0.06f, 0.22f, accG_), gateBrake = smooth01(0.06f, 0.22f, -accG_);
        o.accelG = accG_; o.gate = gateAccel;

        // ---- along the car: the front share against normal
        const float share = std::isfinite(front) ? std::min(0.95f, std::max(0.05f, front)) : 0.5f;
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

        float tLight = 0.0f, tHeavy = 0.0f;
        if (o.learned && base_ > 0.05f) {
            o.rawChange = (share - base_) / base_;
            const float x = (sm_ - base_) / base_;                                           // + more load on the front than normal, - less
            tLight = std::min(kMaxCut, std::max(0.0f, -x - kDeadband)) * gateAccel * (1.0f - air);
            tHeavy = std::min(kMaxHeavy, std::max(0.0f, x - kDeadband)) * gateBrake * (1.0f - air);
        }
        light_ += (tLight - light_) * alphaFor(dt, tLight > light_ ? 0.07f : 0.25f - 0.20f * air);   // in the air it lets go quickly
        heavy_ += (tHeavy - heavy_) * alphaFor(dt, tHeavy > heavy_ ? 0.07f : 0.25f - 0.20f * air);
        if (!(light_ > 0.0f)) light_ = 0.0f;
        if (!(heavy_ > 0.0f)) heavy_ = 0.0f;
        o.light = light_; o.heavy = heavy_;

        // The weight on the front while the brake is pressed (for the brake judder). Measured against the raw share, as the weight really moves.
        const float diveT = (o.learned && base_ > 0.05f && brk > 0.1f) ? c01((share - base_) / 0.10f) : 0.0f;
        dive_ += (diveT - dive_) * alphaFor(dt, 0.1f);
        o.dive = dive_;

        // ---- across the car
        const float raw = std::isfinite(sideRaw) ? std::max(-1.0f, std::min(1.0f, sideRaw)) : 0.0f;
        // The standing car is not perfectly even (fuel, driver, a slope): that offset is taken off, learned only while the car stands.
        if (spd < kRestSpeed && air < 0.1f) { if (!haveOff_) { off_ = raw; haveOff_ = true; } else off_ += (raw - off_) * alphaFor(dt, 0.5f); }
        const float x = raw - off_;
        if (!haveSide_) { fast_ = slow_ = x; haveSide_ = true; }
        else { fast_ += (x - fast_) * alphaFor(dt, 0.10f); slow_ += (x - slow_) * alphaFor(dt, 0.35f); }

        // Lateral acceleration, held over about a third of a second so it stays on while it passes through zero in a direction change.
        latG_ += ((in.latOk && std::isfinite(in.latG) ? in.latG : 0.0f) - latG_) * alphaFor(dt, 0.3f);
        const float gate = smooth01(0.08f, 0.25f, latG_) * smooth01(3.0f, 10.0f, spd) * (1.0f - air);
        o.sideGate = gate; o.side = fast_;

        const float swellT = smooth01(0.05f, 0.45f, std::fabs(fast_)) * gate;
        const float cross = std::sqrt(std::max(0.0f, -fast_ * slow_));      // > 0 only while the quick and the slow average disagree in sign
        const float surgeT = smooth01(0.03f, 0.18f, cross) * gate;
        swell_ += (swellT - swell_) * alphaFor(dt, swellT > swell_ ? 0.10f : 0.20f);   // follows the body roll, a little behind the steering
        surge_ += (surgeT - surge_) * alphaFor(dt, surgeT > surge_ ? 0.05f : 0.15f);
        o.swell = swell_; o.surge = surge_;
        return o;
    }

private:
    bool haveShare_ = false, haveBase_ = false, haveRest_ = false, haveSpeed_ = false, haveOff_ = false, haveSide_ = false;
    float sm_ = 0.5f, base_ = 0.5f, restT_ = 0.0f, steadyT_ = 0.0f, light_ = 0.0f, heavy_ = 0.0f, dive_ = 0.0f, vPrev_ = 0.0f, accG_ = 0.0f;
    float off_ = 0.0f, fast_ = 0.0f, slow_ = 0.0f, latG_ = 0.0f, swell_ = 0.0f, surge_ = 0.0f;
};

// The three settings. Each is 0 = off, 1 = normal, at most 2.
struct Gains {
    float light = 1.0f;        // the front load lost under acceleration: at 1 the steering loses force in proportion
    float heavy = 0.0f;        // the front load gained under braking: at 1 the steering gains force in proportion (RBR already does this, so it is normally off)
    float corner = 1.0f;       // the side load: swell in a corner and surge in a direction change
};

// RBR's force times this: the one function of all the weight transfer, (1 - the front load lost + the front load gained) * (1 + the cornering weight).
// Below 1 while acceleration unloads the front, above 1 in hard braking and under cornering load, exactly 1 when nothing moves.
inline float Mul(const Out& o, const Gains& g) {
    using namespace detail;
    const float cut = std::min(kMaxCut, gain(g.light) * pos(o.light));
    const float add = std::min(kMaxHeavy, gain(g.heavy) * pos(o.heavy));
    const float corner = std::min(kMaxCorner, gain(g.corner) * (kSwellAdd * c01(o.swell) + kSurgeAdd * c01(o.surge)));
    return (1.0f - cut + add) * (1.0f + corner);
}

}  // namespace WeightTransfer
