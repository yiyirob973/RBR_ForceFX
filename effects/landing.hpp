// Landing detector: turns the strut loads into ONE event per real jump, with a severity from 0 to 1. Pure logic, no Windows or game access,
// tested in tests/landing_test.cpp. (It used to be a block inside ParsePacket in RBR_ForceFX.cpp, which nothing could test.)
//
// Only a real jump counts: the car has to be off the ground for Road::kBoostMinAir first, so bumps, washboard and weight transfer never start
// one. Three cues are combined, so any one strong cue gives a full-strength landing:
//   peak   the highest strut load on any wheel (relative to its static load)
//   jump   how fast the TOTAL strut load rose from its recent minimum
//   air    how long the wheels were unloaded before the landing, plus the heave acceleration spike
// Severity is continuous (no on/off threshold), so a small hop is a small tap and a big jump is violent. The detector keeps refining the severity
// for 0.1 s while the suspension compresses.
#pragma once
#include <algorithm>
#include <cmath>
#include "road/road_guard.hpp"

namespace Landing {

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
inline float alphaFor(float dt, float tau) { return 1.0f - std::exp(-dt / tau); }
}  // namespace detail

constexpr float kWindow = 0.10f;       // seconds a landing is refined after it starts

struct In {
    float totRatio = 1.0f;     // total strut load / the learned static load (1 = at rest, 0 = in the air)
    float peakRw = 0.0f;       // the highest single-wheel load relative to its own static load
    float bumpEnv = 0.0f;      // heave acceleration spike envelope, 0..1+
    float speed = 0.0f;        // m/s: nothing is "off the ground" while the car is hardly moving
    float airLevel = 0.15f;    // totRatio below this counts as off the ground
};

struct Out {
    unsigned evtSeq = 0;       // goes up by one for every landing
    float evtSev = 0.0f;       // severity of the current event 0..1 (refined upward while it is still compressing)
    bool fired = false;        // a landing started on this step
    bool finished = false;     // the refining window closed on this step (log it now)
    float air = 0.0f, peak = 0.0f, jump = 0.0f, bump = 0.0f;   // what the finished landing was made of, for the log
    float sinceEvent = 99.0f;  // seconds since the last landing started or the car was in the air (other detectors keep quiet after a landing)
};

class Detector {
public:
    void Reset() { *this = Detector(); }
    // A new stage: forget the air time, the window and the running minimum, but keep counting the events, so nothing downstream sees the counter fall back.
    void NewStage() { totLow_ = 1.0f; airT_ = 0; landWin_ = 0; since_ = 99.0f; }

    Out Step(const In& in, float dt) {
        using namespace detail;
        Out o;
        if (!std::isfinite(dt) || dt <= 0.0f) dt = 0.01f;
        const float rawTot = std::isfinite(in.totRatio) ? in.totRatio : 1.0f;
        totLow_ = std::min(rawTot, totLow_ + (rawTot - totLow_) * alphaFor(dt, 0.12f));
        const float jump = std::max(0.0f, rawTot - totLow_);
        const bool off = in.speed > 3.0f && rawTot < in.airLevel;

        if (off) {
            airT_ += dt; since_ = 0.0f;
        } else {
            if (Road::LandingBoostWorthy(airT_) && landWin_ <= 0.0f) {   // only a real jump boosts the wheel, a bump never does
                landWin_ = kWindow; landAir_ = airT_; landPeak_ = 0; landBump_ = 0; landJump_ = 0;
                ++seq_; sev_ = Severity(0.0f, landAir_, 0.0f, 0.0f);
                o.fired = true; since_ = 0.0f;
            }
            airT_ = 0;
        }
        if (landWin_ > 0.0f) {                                    // refine the estimate while the suspension compresses
            landPeak_ = std::max(landPeak_, in.peakRw);
            landBump_ = std::max(landBump_, c01(in.bumpEnv));
            landJump_ = std::max(landJump_, jump);
            sev_ = std::max(sev_, Severity(landPeak_, landAir_, landBump_, landJump_));
            landWin_ -= dt; since_ = 0.0f;
            if (landWin_ <= 0.0f) {
                landWin_ = 0.0f;
                o.finished = true; o.air = landAir_; o.peak = landPeak_; o.jump = landJump_; o.bump = landBump_;
            }
        }
        since_ = std::min(since_ + dt, 99.0f);
        o.evtSeq = seq_; o.evtSev = sev_; o.sinceEvent = since_;
        return o;
    }

    static float Severity(float peak, float air, float bump, float jmp) {
        using namespace detail;
        const float peakT = c01((peak - 1.15f) / 1.3f);
        const float jumpT = c01((jmp - 0.25f) / 1.0f);
        const float airT2 = c01(air / 0.5f);
        const float bumpT = c01(bump);
        const float top = std::max(std::max(peakT, jumpT), std::max(0.85f * airT2, 0.7f * bumpT));
        const float sum = peakT + jumpT + airT2 + bumpT;
        const float sev = c01(top + 0.15f * (sum - top));
        return std::max(sev, 0.55f + 0.45f * airT2);                    // anything that really left the ground is always felt
    }

private:
    float totLow_ = 1.0f;      // running minimum of the total strut load (relative): a sudden rise above it means something hit the car
    float airT_ = 0, landWin_ = 0, landAir_ = 0, landPeak_ = 0, landBump_ = 0, landJump_ = 0, since_ = 99.0f;
    unsigned seq_ = 0; float sev_ = 0;
};

}  // namespace Landing
