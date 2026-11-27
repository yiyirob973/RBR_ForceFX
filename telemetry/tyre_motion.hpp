// Tyre motion: which tyres are driven, how each tyre moves over the ground, how much grip the surface gives, and what
// counts as wheelspin and what counts as a slide. Pure logic, no Windows or game access (tests: tests/tyre_motion_test.cpp).
// Shared by the road feel (RBR_ForceFX.cpp) and the dynamic audio (audio/core.hpp), so both react to the same thing.
//
// Wheelspin and slides are two different events and are kept apart:
//  - Wheelspin is a driven tyre turning faster than the ground under it while the throttle is open. Only the tyres the engine
//    drives can have it: a rear-wheel-drive car only spins its rear tyres, a front-wheel-drive car its front tyres.
//  - A slide is a tyre moving over the ground in a direction it is not pointing: sideways (slip angle), or locked up and
//    dragged along under braking or the handbrake. Any tyre can slide, driven or not.
// Both are measured against the grip limit of the ground under that tyre, so the same slip can be a squeal on tarmac and an
// ordinary slide on snow. How much grip the car really has on a surface is learned from how hard the car has accelerated on it.
#pragma once
#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace Tyre {

namespace detail {
inline double clampd(double v, double lo, double hi) { return !(v > lo) ? lo : (v > hi ? hi : v); }   // NaN comes out as lo
inline double smoothstep(double x) { x = clampd(x, 0.0, 1.0); return x * x * (3.0 - 2.0 * x); }
struct Lp {                                  // first-order low-pass with a time constant
    double tau, v = 0; bool has = false;
    explicit Lp(double t = 1.0) : tau(t) {}
    double update(double x, double dt) { if (!has) { v = x; has = true; } else v += (1.0 - std::exp(-dt / tau)) * (x - v); return v; }
    bool hasValue() const { return has; }
    double value() const { return v; }
};
}  // namespace detail

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------- drivetrain
enum Drivetrain { FWD = 0, RWD = 1, AWD = 2 };
constexpr int kDefaultDrivetrain = RWD;       // used for every car that does not say otherwise in ForceFXCars.ini

// wheel: 0 front left, 1 front right, 2 rear left, 3 rear right (NGP order LF, RF, LB, RB).
inline bool Driven(int drivetrain, int wheel) {
    if (drivetrain == AWD) return true;
    return drivetrain == FWD ? (wheel == 0 || wheel == 1) : (wheel == 2 || wheel == 3);
}
inline const char* DrivetrainName(int d) { return d == FWD ? "FWD" : (d == AWD ? "AWD" : "RWD"); }

// "RWD", "fwd", "AWD", "4WD", "rear", "front wheel drive" ...; anything else gives `fallback`.
inline int ParseDrivetrain(const std::string& text, int fallback) {
    std::string t;
    for (char ch : text) if (std::isalnum((unsigned char)ch)) t.push_back((char)std::tolower((unsigned char)ch));
    if (t == "rwd" || t == "rear" || t == "rearwheeldrive" || t == "rearwheel" || t == "fr" || t == "mr" || t == "rr") return RWD;
    if (t == "fwd" || t == "front" || t == "frontwheeldrive" || t == "frontwheel" || t == "ff") return FWD;
    if (t == "awd" || t == "4wd" || t == "4x4" || t == "all" || t == "allwheeldrive" || t == "allwheel" || t == "fourwheeldrive") return AWD;
    return fallback;
}

// ---------------------------------------------------------------- how each tyre moves
// Body motion in, per-tyre slip angle and per-tyre longitudinal ground speed out (rigid-body kinematics).
// Learned automatically: yaw sign, which side +sway points to, steering-to-wheel angle, and the units of acceleration and yaw rate.
//
// A tyre only rolls at the speed of the ground along the direction it points. A tyre sliding at slip angle a rolls at
// |v| * cos(a), so comparing its spin with the car's full speed would read every slide as a lock-up. vLong is the speed to
// compare the tyre's spin with.
struct Params {
    double velocityScale = 1.0;      // velocity vector to m/s
    double accelToMs2 = 1.0;         // 9.81 if accelerations are in g; 1 = work it out from the data
    double yawRateScale = 1.0;       // 0.01745 if yaw rate is in deg/s; 1 = work it out from the data
    double wheelbase = 2.5, cgToFront = 1.25, trackWidth = 1.5;
    double coastAccelLimit = 1.5;    // m/s^2
};

struct Input {
    double velSurge = 0, velSway = 0, yawRate = 0, accSway = 0, steering = 0;
    double strut[4] = {0, 0, 0, 0};  // strut force as reported (signed), LF, RF, LB, RB
};

class Kinematics {
public:
    double slipAngle[4] = {0, 0, 0, 0};   // rad, per tyre
    double vLong[4] = {0, 0, 0, 0};       // m/s: ground speed along the direction each tyre points (the car's speed when it is not known)
    bool valid = false;                   // slipAngle is usable
    double vx = 0, vy = 0, yaw = 0, latAccel = 0, latRatio = 0;
    double accelScale(const Params& s) const { return s.accelToMs2 != 1.0 ? s.accelToMs2 : autoAccel_; }
    double yawScale(const Params& s) const { return s.yawRateScale != 1.0 ? s.yawRateScale : autoYaw_; }
    // True once lateral acceleration and speed x yaw rate agree: acceleration is known in m/s^2 and the yaw rate unit is right.
    bool unitsConfirmed() const { return latRatioAvg_.hasValue() && latRatioAvg_.value() > 0.5 && latRatioAvg_.value() < 2.0; }

    // speedMs: the car's speed over the ground. velocityIsGround: speedMs comes from the velocity vector itself.
    void update(const Input& f, double dt, double speedMs, const Params& s, bool velocityIsGround) {
        using detail::clampd;
        valid = false;
        for (int i = 0; i < 4; ++i) { slipAngle[i] = 0.0; vLong[i] = speedMs; }
        double raw = std::sqrt(f.velSurge * f.velSurge + f.velSway * f.velSway);
        double k;
        if (velocityIsGround) k = s.velocityScale;     // the vector IS the ground speed: nothing to calibrate against
        else {                                         // fallback: size the vector to the speed field
            if (speedMs > 5.0 && raw > 0.5) velScale_.update(clampd(speedMs / raw, 0.1, 10.0), dt);
            k = velScale_.hasValue() ? velScale_.value() : 1.0;
            if (k > 0.85 && k < 1.18) k = 1.0;
        }
        vx = f.velSurge * k; vy = f.velSway * k;
        if (speedMs < 8.0 || std::fabs(vx) < 3.0) return;

        // Unit auto-detection (only when the settings leave AccelToMs2 / YawRateScale at 1): while cornering,
        // raw a_y / (v * yaw) equals scaleYaw / scaleAccel, so it shows whether accel is in g (x9.81) and/or yaw in deg/s.
        if (s.accelToMs2 == 1.0 && s.yawRateScale == 1.0 && std::fabs(f.accSway) > 0.15 && std::fabs(vx * f.yawRate) > 2.0) {
            rawRatioLog_.update(std::log10(clampd(std::fabs(f.accSway) / std::fabs(vx * f.yawRate), 1e-5, 1e5)), dt);
            if (++rawRatioN_ >= 120) {
                static const double cand[4][3] = {{0.0, 1, 1}, {-0.9917, 9.81, 1}, {-1.7581, 1, 0.0174533}, {-2.7498, 9.81, 0.0174533}};
                int best = -1; double bd = 0.35;
                for (int i = 0; i < 4; ++i) { double d = std::fabs(rawRatioLog_.value() - cand[i][0]); if (d < bd) { bd = d; best = i; } }
                if (best >= 0 && (cand[best][1] != autoAccel_ || cand[best][2] != autoYaw_)) {
                    autoAccel_ = cand[best][1]; autoYaw_ = cand[best][2];
                    latRatioAvg_ = detail::Lp(3.0);       // re-check the sanity test with the corrected scales
                }
            }
        }
        double ay = f.accSway * accelScale(s), r = f.yawRate * yawScale(s);
        yawSign_.update(ay * r, dt);
        double rc = yawSign_.value() >= 0 ? r : -r;
        yaw = rc; latAccel = ay;
        latRatio = std::fabs(vx * rc) > 0.05 ? ay / (vx * rc) : 0.0;

        // Outside wheels carry more load: tells which side +sway points to.
        double dl = f.strut[0] + f.strut[2] - f.strut[1] - f.strut[3];
        sideSign_.update(ay * dl, dt);
        double leftY = sideSign_.value() <= 0 ? 1.0 : -1.0;

        // Self-check of the yaw/acceleration units: while cornering, lateral accel should be about speed x yaw rate
        // (ratio ~1). If it is not, the per-tyre angles would be nonsense, so fall back to the car's body slip angle, which
        // needs only the two velocities.
        if (std::fabs(vx * rc) > 0.5 && std::fabs(ay) > 0.3) latRatioAvg_.update(clampd(ay / (vx * rc), 0.0, 50.0), dt);
        if (!unitsConfirmed()) {
            double beta = std::atan2(std::fabs(vy), std::fabs(vx));
            for (int i = 0; i < 4; ++i) { slipAngle[i] = beta; vLong[i] = std::fabs(vx); }
            valid = true;
            return;
        }

        double L = std::max(s.wheelbase, 0.5), a = s.cgToFront, b = L - a, half = 0.5 * s.trackWidth;
        if (std::fabs(f.steering) > 0.05 && std::fabs(ay) < s.coastAccelLimit && std::fabs(vx) > 8.0)
            steerGain_.update(rc * L / (vx * f.steering), dt);
        double delta = steerGain_.hasValue() ? f.steering * steerGain_.value() : rc * L / vx;
        delta = clampd(delta, -0.8, 0.8);

        static const int leftSign[4] = {1, -1, 1, -1};
        for (int i = 0; i < 4; ++i) {
            double x = i < 2 ? a : -b, y = leftY * leftSign[i] * half;
            double vxw = vx - rc * y, vyw = vy + rc * x;
            slipAngle[i] = std::atan2(vyw, std::fabs(vxw)) - (i < 2 ? delta : 0.0);
            vLong[i] = std::max(0.0, std::sqrt(vxw * vxw + vyw * vyw) * std::cos(slipAngle[i]));
        }
        valid = true;
    }

private:
    detail::Lp yawSign_{2.0}, sideSign_{2.0}, steerGain_{3.0}, velScale_{2.0}, latRatioAvg_{3.0}, rawRatioLog_{4.0};
    double autoAccel_ = 1.0, autoYaw_ = 1.0; int rawRatioN_ = 0;
};

// ---------------------------------------------------------------- grip of the ground
// How hard the car has really been able to accelerate on each kind of ground. Every class keeps the highest smoothed
// acceleration seen while the car was on it (slowly forgotten, so one crash spike does not stay for ever). The acceleration
// now, against that, says how much of its grip the car is using: a tyre only slides when the car is at its limit.
// Classes are the caller's own numbering (0..kClasses-1). Acceleration must be in m/s^2.
class GripLearner {
public:
    static constexpr int kClasses = 8;
    // aMag: the car's acceleration over the ground, m/s^2. share: how much of the car's load rests on each class (sums to 1).
    // usable: the wheels are on the ground and the car is moving, and the acceleration is known in m/s^2.
    void update(double aMag, const double* share, bool usable, double dt) {
        using detail::clampd;
        if (!usable || !(dt > 0.0)) { use_ = -1.0; return; }
        const double a = clampd(aMag, 0.0, 18.0);                          // a crash must not become the grip limit
        aSm_ = hasA_ ? aSm_ + (1.0 - std::exp(-dt / 0.25)) * (a - aSm_) : a; hasA_ = true;
        int best = -1; double bw = 0.6;
        for (int c = 0; c < kClasses; ++c) if (share[c] > bw) { bw = share[c]; best = c; }
        if (best >= 0) {                                                    // mostly one ground: this is what that ground allows
            peak_[best] *= std::exp(-dt / 300.0);
            if (aSm_ > peak_[best]) peak_[best] += (1.0 - std::exp(-dt / 0.4)) * (aSm_ - peak_[best]);
            if (aSm_ > 4.0) hard_[best] += dt;
        }
        const double cap = Capacity(share);
        use_ = cap > 0.0 ? clampd(aSm_ / cap, 0.0, 1.5) : -1.0;
    }
    bool learned(int c) const { return c >= 0 && c < kClasses && hard_[c] >= 4.0 && peak_[c] >= 3.0; }
    double peak(int c) const { return c >= 0 && c < kClasses ? peak_[c] : 0.0; }
    // The acceleration the car can reach on this mix of ground, m/s^2 (0 = not learned yet for any of it).
    double Capacity(const double* share) const {
        double s = 0, w = 0;
        for (int c = 0; c < kClasses; ++c) if (learned(c) && share[c] > 0.0) { s += share[c] * peak_[c]; w += share[c]; }
        return w > 0.0 ? s / w : 0.0;
    }
    // Share of the grip in use, 0..1.5; -1 = not known yet (nothing learned for this ground, or no usable acceleration).
    double use() const { return use_; }

private:
    double peak_[kClasses] = {0, 0, 0, 0, 0, 0, 0, 0}, hard_[kClasses] = {0, 0, 0, 0, 0, 0, 0, 0};
    double aSm_ = 0, use_ = -1.0; bool hasA_ = false;
};

// A slide only counts when the car is near what its grip allows: slip angles worked out from the body motion can be off,
// and this stops a wrong one from sounding like a slide. Not known yet = no change.
inline double GripGate(double use) { return use < 0.0 ? 1.0 : 0.3 + 0.7 * detail::smoothstep((use - 0.4) / 0.4); }

// ---------------------------------------------------------------- wheelspin and slides
// Where one tyre's grip limit lies on this ground: the slip ratio and slip angle at which it gives the most grip. Hard,
// clean ground has a sharp, early peak; loose ground a wide one, since the tyre digs in; slippery ground a gentle one.
// loose and slick are 0..1 (see Road::Ground).
inline void PeaksFromGround(double loose, double slick, double& peakRatio, double& peakAngleDeg) {
    using detail::clampd;
    loose = clampd(loose, 0.0, 1.0); slick = clampd(slick, 0.0, 1.0);
    double pr = 0.10 + 0.10 * loose, pa = 6.0 + 7.0 * loose;
    const double w = slick * (1.0 - 0.6 * loose);
    pr += (0.15 - pr) * w; pa += (10.0 - pa) * w;
    peakRatio = pr; peakAngleDeg = pa;
}

struct TyreIn {
    bool driven = true;                          // the engine drives this tyre
    bool ratioValid = false; double ratio = 0;   // slip ratio past its dead band: + the tyre turns faster than the ground (spin), - slower (lock-up)
    bool angleValid = false; double angle = 0;   // slip angle, rad
    double throttle = 0, brake = 0, handbrake = 0;
    double peakRatio = 0.15, peakAngleDeg = 10;  // the grip limit of the ground under this tyre
    double gate = 1;                             // GripGate()
};
// Each value is in units of the tyre's grip limit: 1 = at the limit.
struct TyreOut {
    double spin = 0;      // wheelspin: a driven tyre faster than the ground, throttle open
    double lock = 0;      // lock-up: a tyre slower than the ground under braking or the handbrake
    double lateral = 0;   // sideways slip
    double slide = 0;     // sliding: lateral and lock-up together, confirmed by the grip in use
};

inline TyreOut Classify(const TyreIn& t) {
    using detail::clampd;
    TyreOut o;
    const double pkx = std::max(t.peakRatio, 0.02);
    const double tanPk = std::tan(std::max(t.peakAngleDeg, 1.0) * kPi / 180.0);
    if (t.ratioValid && std::isfinite(t.ratio)) {
        if (t.ratio > 0.0 && t.driven && t.throttle > 0.05 && t.brake < 0.5) o.spin = t.ratio / pkx;
        else if (t.ratio < 0.0 && (t.brake > 0.1 || t.handbrake > 0.1)) o.lock = -t.ratio / pkx;
    }
    if (t.angleValid && std::isfinite(t.angle)) o.lateral = std::tan(std::min(std::fabs(t.angle), 1.2)) / tanPk;
    o.slide = std::sqrt(o.lateral * o.lateral + o.lock * o.lock) * clampd(t.gate, 0.0, 1.0);
    return o;
}

// Where the squeal and the slide begin, in units of the grip limit, and how they ramp up.
constexpr double kSpinStart = 0.8, kSlideStart = 1.3;
inline double SpinAmount(double n, double start = kSpinStart) { return detail::smoothstep((n - start) / 0.45); }     // 0..1
inline double SlideAmount(double n, double start = kSlideStart) { return detail::smoothstep((n - start) / 0.8); }   // 0..1

}  // namespace Tyre
