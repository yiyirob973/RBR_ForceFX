// Wheel slip from NGP's telemetry. Pure logic, no Windows or game access (tests: tests/wheel_slip_test.cpp).
//
// NGP's packet has no slip value. Each wheel only reports a segment counter (0..7) that steps once every eighth of a
// revolution, so the wheel's spin has to be measured from that and compared with the car's real ground speed.
// That measurement is coarse, and the old way of doing it (average the step count per packet, divide by the packet's
// arrival time) read 10 to 30 percent slip on a wheel that was not slipping at all, whenever the segments per packet were
// small or the packets arrived unevenly. Squeal and slide sounds, and anything else keyed to slip, fired all the time.
//
// What is done instead:
//  - Revolutions are counted over a window that holds enough steps (4 revolutions) instead of per packet, so the count
//    error of at most one step is a small part of what was counted.
//  - Every estimate carries that error, and slip only counts beyond it (a dead band equal to the uncertainty), so noise
//    cannot read as slip. A second, shorter window catches big events (lock-up, wheelspin) quickly, with its own dead band.
//  - Time comes from NGP's own step counter (totalSteps_) when it behaves, so late or bunched UDP arrivals do not matter.
//    Without it, arrival time is used and the dead band grows with how unevenly the packets arrive.
//  - The distance one revolution covers is learned while the car coasts, and a coast that disagrees with it is ignored.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>

namespace Slip {

inline double clampd(double v, double lo, double hi) { return !(v > lo) ? lo : (v > hi ? hi : v); }

// ---------------------------------------------------------------- time base
// Gives each packet a position on a clean time axis. With NGP's step counter, a tick is one physics step and
// secPerTick is learned over many packets, so uneven arrival does not disturb it. Without a usable counter, a tick is a
// second of arrival time.
class Timebase {
public:
    // timeErr: how far a time span measured on this axis can be off, in seconds (about three times the spread of the arrival
    // intervals without the step counter; a small constant with it).
    struct Stamp { double tick; double secPerTick; double timeErr; };

    Stamp stamp(uint32_t steps, bool haveSteps, double arrival) {
        trackArrival(arrival);
        const double arrivalErr = 3.0 * std::sqrt(var_) + 0.0005;
        if (!haveSteps) { useSteps_ = false; return {arrival, 1.0, arrivalErr}; }
        if (!anchored_ || steps < last_ || steps - last_ > 200 || arrival - lastArrival_ > 1.0) anchor(steps, arrival);
        last_ = steps; lastArrival_ = arrival;
        const double span = arrival - a0_;
        const uint32_t n = steps - s0_;
        if (!useSteps_) {
            if (span >= 1.0 && n >= 30) {
                const double dt = span / n;
                if (dt > 0.0005 && dt < 0.05) { secPerStep_ = dt; useSteps_ = true; ++gen_; }
            }
            return {arrival, 1.0, arrivalErr};
        }
        if (n >= 200) secPerStep_ = span / n;
        if (span > 120.0 && n >= 200) { a0_ = arrival; s0_ = steps; }      // follow slow clock drift without a jump
        return {(double)steps, secPerStep_, 0.0003};
    }
    // Changes whenever the time axis changes meaning (restart, switch to the step counter): counters must start over.
    unsigned generation() const { return gen_; }
    bool usingSteps() const { return useSteps_; }

private:
    // Spread of the gaps between arrivals, over the last couple of seconds.
    void trackArrival(double arrival) {
        if (haveArr_) {
            const double gap = arrival - prevArr_;
            if (gap > 0.0 && gap < 0.5) {
                if (!haveStat_) { mean_ = gap; var_ = 0; haveStat_ = true; }
                else { const double a = 0.01, d = gap - mean_; mean_ += a * d; var_ = (1 - a) * (var_ + a * d * d); }
            }
        }
        prevArr_ = arrival; haveArr_ = true;
    }
    bool haveArr_ = false, haveStat_ = false; double prevArr_ = 0, mean_ = 0, var_ = 0;
    void anchor(uint32_t steps, double arrival) { anchored_ = true; s0_ = steps; a0_ = arrival; useSteps_ = false; ++gen_; }
    bool anchored_ = false, useSteps_ = false;
    uint32_t s0_ = 0, last_ = 0; double a0_ = 0, lastArrival_ = 0, secPerStep_ = 0.01;
    unsigned gen_ = 0;
};

// ---------------------------------------------------------------- revolutions from the segment counter
class RevCounter {
public:
    struct Est {
        bool valid = false;
        double revPerSec = 0;      // revolutions per second over the window
        double segments = 0;       // steps counted inside the window
        double seconds = 0;        // length of the window
        double rateErr = 1e9;      // the most revPerSec can be off by: one step over the window
    };

    void reset() { hist_.clear(); count_ = 0; have_ = false; }

    void push(uint32_t segment, double tick, double secPerTick) {
        segment &= 7u;
        if (!have_) { have_ = true; lastSeg_ = segment; lastTick_ = tick; count_ = 0; hist_.clear(); hist_.push_back({tick, 0.0}); return; }
        const double gap = (tick - lastTick_) * secPerTick;
        if (gap <= 0.0) return;                                           // same packet twice
        if (gap > 0.25) { reset(); push(segment, tick, secPerTick); return; }   // packets went missing: the count cannot be trusted
        count_ += (double)((segment + 8u - lastSeg_) & 7u);
        lastSeg_ = segment; lastTick_ = tick;
        hist_.push_back({tick, count_});
        while (hist_.size() > 2 && (tick - hist_.front().tick) * secPerTick > 1.2) hist_.pop_front();
    }

    // The shortest window, no longer than maxSeconds, that holds at least minSegments steps (or the longest one allowed).
    Est estimate(double minSegments, double maxSeconds, double secPerTick) const {
        Est e;
        if (hist_.size() < 2) return e;
        const Pt& now = hist_.back();
        const Pt* best = nullptr; double age = 0;
        for (size_t i = hist_.size() - 1; i-- > 0;) {
            const double a = (now.tick - hist_[i].tick) * secPerTick;
            if (a > maxSeconds) break;
            best = &hist_[i]; age = a;
            if (now.count - hist_[i].count >= minSegments) break;
        }
        if (!best || age < 0.04) return e;
        e.valid = true; e.seconds = age; e.segments = now.count - best->count;
        e.revPerSec = e.segments / 8.0 / age;
        e.rateErr = 1.0 / (8.0 * age);
        return e;
    }

private:
    struct Pt { double tick; double count; };
    std::deque<Pt> hist_;
    double count_ = 0, lastTick_ = 0; uint32_t lastSeg_ = 0; bool have_ = false;
};

// ---------------------------------------------------------------- slip of one wheel
class WheelSlip {
public:
    explicit WheelSlip(double priorRevPerMetre = 1.0 / 1.95) : revPerMetre_(priorRevPerMetre) {}

    // Results of the last update().
    double ratio = 0;          // slip ratio past the dead band: positive = wheelspin, negative = lock-up; 0 = rolling or not sure
    bool valid = false;        // false below 3 m/s, or while the count is not usable
    double margin = 1;         // the dead band that was applied (how sure the reading is)
    bool turning() const { return seenTurning_; }     // the wheel has been seen turning while the car drove; false = the counter is probably not reported
    double wheelMs = 0;        // how fast the tyre's surface turns, as a road speed (m/s), from the learned tyre size
    double wheelErrMs = 0;     // how far wheelMs can be off (m/s): count and timing error, not the tyre size
    bool wheelValid = false;

    bool learned() const { return learned_; }
    double revPerMetre() const { return revPerMetre_; }

    void reset(double priorRevPerMetre) { *this = WheelSlip(priorRevPerMetre); }

    // groundMs: the car's real speed over the ground. coasting: no throttle, no brake, no hard cornering.
    void update(const RevCounter& rc, double secPerTick, double timeErrSec, double groundMs, bool coasting, double dt) {
        const RevCounter::Est slow = rc.estimate(32.0, 0.5, secPerTick), fast = rc.estimate(8.0, 0.15, secPerTick);
        wheelValid = slow.valid;
        wheelMs = slow.valid ? slow.revPerSec / revPerMetre_ : 0.0;
        wheelErrMs = slow.valid ? (slow.rateErr + slow.revPerSec * timeErrSec / slow.seconds) / revPerMetre_ : 0.0;

        // Learn how many revolutions one metre takes. Only a clean coast counts, and one that disagrees a lot with what is known
        // (a handbrake turn, a lock-up with the pedals off) is thrown away.
        if (coasting && groundMs > 8.0 && slow.valid && slow.segments >= 20.0) {
            const double sample = slow.revPerSec / groundMs;
            const double w = clampd(slow.segments / 32.0, 0.0, 1.0);
            if (!haveRatio_) { revPerMetre_ = sample; haveRatio_ = true; }
            else if (std::fabs(sample / revPerMetre_ - 1.0) < (learned_ ? 0.25 : 0.6))
                revPerMetre_ += (1.0 - std::exp(-dt * w / 4.0)) * (sample - revPerMetre_);
            learnedTime_ += dt * w;
            learned_ = learnedTime_ >= 1.5;
        }

        // A counter that has never moved while the car drove (a game build that does not report it) says nothing about slip.
        // A wheel that locks up later has been seen turning, so lock-up is still found.
        if (slow.valid && slow.segments >= 16.0 && groundMs > 5.0) seenTurning_ = true;

        ratio = 0; valid = false;
        if (groundMs < 3.0 || !slow.valid || !seenTurning_) return;
        const double expect = revPerMetre_ * groundMs;
        const double floorErr = learned_ ? 0.02 : 0.15;                  // how well the tyre size is known
        // The dead band: one step of count error, plus the timing error over the window, plus how well the tyre size is known.
        const double ms = slow.rateErr / expect + timeErrSec / slow.seconds + floorErr;
        const double ns = beyond(slow.revPerSec / expect - 1.0, ms);
        double nf = 0;
        if (fast.valid) nf = beyond(fast.revPerSec / expect - 1.0, fast.rateErr / expect + timeErrSec / fast.seconds + floorErr);
        ratio = std::fabs(nf) > std::fabs(ns) ? nf : ns;
        margin = ms; valid = true;
    }

private:
    static double beyond(double raw, double band) { return raw >= 0 ? std::max(0.0, raw - band) : std::min(0.0, raw + band); }
    double revPerMetre_;
    bool haveRatio_ = false, learned_ = false, seenTurning_ = false; double learnedTime_ = 0;
};

// ---------------------------------------------------------------- ground speed
// The car's real speed over the ground, in m/s. The speedometer field comes from the driven wheels, so under wheelspin and
// lock-up it follows the wheels instead of the road, and it reads in other units than m/s; the velocity vector does not. When
// the vector stays dead while the car clearly moves, the field is used instead, scaled by how it compared with the vector while
// that worked (0 = not known).
class GroundSpeed {
public:
    // Which speed counts as the ground speed. Auto (0) takes the velocity vector, and the Speed field only if the vector is dead; 1 = always the
    // vector; 2 = always the Speed field (as it is, unscaled).
    void setMode(int mode) { mode_ = mode < 0 || mode > 2 ? 0 : mode; }
    // What the Speed field reads against the true ground speed before it has been learned from a steady roll (RBR/NGP: about 1.6).
    void setInitialFieldScale(double s) { initScale_ = s > 0.2 && s < 5.0 ? s : 1.609; }

    // fieldRaw: the Speed field in m/s. velSurge/velSway: the velocity vector in m/s. Returns the ground speed in m/s, or 0 when it is not known
    // (the vector is dead and the field has never been compared with it).
    double update(double fieldRaw, double velSurge, double velSway, double throttle, double brake, double dt) {
        const double vel = std::sqrt(velSurge * velSurge + velSway * velSway);
        if (fieldRaw > 8.0 && throttle < 0.1 && brake < 0.1) { if (vel < 0.5) dead_ += dt; else dead_ = 0; }
        else if (vel > 2.0) dead_ = 0;
        fromVel_ = mode_ == 1 || (mode_ == 0 && dead_ < 2.0);
        if (fromVel_ && vel > 8.0 && fieldRaw > 8.0 && throttle < 0.12 && brake < 0.03) {   // a steady roll: how the field compares with the vector
            const double r = std::max(1.0, std::min(2.5, fieldRaw / vel));
            scale_ = haveScale_ ? scale_ + (1.0 - std::exp(-dt / 4.0)) * (r - scale_) : r; haveScale_ = true;
        }
        if (fromVel_) return vel;
        if (mode_ == 2) return fieldRaw;
        return haveScale_ ? fieldRaw / scale_ : 0.0;
    }
    bool fromVelocity() const { return fromVel_; }
    // The speed of the driven wheels as a road speed, from the Speed field: scaled down while the true ground speed comes from the vector.
    double drivenMs(double fieldRaw) const { return fromVel_ ? fieldRaw / (haveScale_ ? scale_ : initScale_) : fieldRaw; }
private:
    double dead_ = 0, scale_ = 1, initScale_ = 1.609; int mode_ = 0; bool fromVel_ = true, haveScale_ = false;
};

}  // namespace Slip
