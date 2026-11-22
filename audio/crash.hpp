// Crash detection from the telemetry: the car hitting a tree, a big rock, a wall or a bank, as opposed to bumps, kerbs and landings.
//
// A crash is a sudden spike in the body's acceleration along the car (surge) or across it (sway) that ordinary driving never makes: hard
// braking and cornering are well under 2 g and build up over a second, a crash is several g in a few hundredths of a second. What is hit
// is not in the telemetry, so the detector reports only how hard (0..1) and from which direction (nose first, or the side).
//   - A spike over `strongG` is a crash at once.
//   - A smaller spike over `minG` only counts if the car also loses speed right after it (`minDv` within `window`): that separates a hit
//     from a jolt that leaves the speed alone (a kerb, a ditch, a hard bump). It waits at most `window` seconds for that.
//   - Nothing counts in the air or in the moments after landing (a hard landing shakes the body for a while).
//   - One crash makes one event; the car scraping along a wall does not make a machine gun (`cooldown`).
// Portable, no Windows APIs; tested in tests/crash_test.cpp.
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>

namespace rsa {
namespace crash {

constexpr double kG = 9.80665;

struct Params {
    double minG = 3.0;          // body acceleration beyond the slow part (g) that may be a crash, if the car loses speed
    double strongG = 7.0;       // ... and that is a crash whatever the speed does
    double baselineTau = 0.6;   // seconds: what counts as the slow part (braking, cornering, a long compression)
    double window = 0.15;       // seconds to wait for the speed to drop after a smaller spike
    double minDv = 1.5;         // m/s of speed lost within `speedSpan` seconds that confirms a smaller spike
    double speedSpan = 0.35;
    double cooldown = 0.6;      // seconds after a crash before another can be reported
    double minSpeed = 2.0;      // m/s: slower than this nothing is a crash (parked, creeping, a nudge)
    double fullG = 14.0;        // peak g that is a full-strength (1.0) crash
    double fullDv = 15.0;       // m/s lost that is a full-strength crash
};

struct Result {
    bool hit = false;
    double peakG = 0.0;         // the strongest spike of the crash, in g
    double dv = 0.0;            // speed lost around it, m/s
    double severity = 0.0;      // 0..1
    bool frontal = true;        // along the car (nose or tail) rather than from the side
};

class Detector {
public:
    void reset() { *this = Detector(); }
    double lastPeakG() const { return lastPeakG_; }

    // accSurge / accSway in m/s^2 (already scaled), speedMs = true ground speed, suppress = in the air or just landed.
    Result update(double dt, double accSurge, double accSway, double speedMs, bool suppress, const Params& p = Params()) {
        Result r;
        if (!(dt > 0.0) || !std::isfinite(dt)) dt = 0.016;
        if (dt > 0.25) dt = 0.25;
        if (!std::isfinite(accSurge)) accSurge = 0.0;
        if (!std::isfinite(accSway)) accSway = 0.0;
        if (!std::isfinite(speedMs) || speedMs < 0.0) speedMs = 0.0;

        if (!init_) { baseA_ = accSurge; baseB_ = accSway; init_ = true; }
        const double hpA = accSurge - baseA_, hpB = accSway - baseB_;
        // The slow part follows after the measurement, so a spike is measured against what came before. It barely follows while a spike is on (over 2 g
        // beyond it), or the slow part would be dragged along and the return to normal afterwards would look like a spike the other way.
        double a = 1.0 - std::exp(-dt / std::max(0.05, p.baselineTau));
        if (std::sqrt(hpA * hpA + hpB * hpB) > 2.0 * kG) a *= 0.1;
        baseA_ += (accSurge - baseA_) * a; baseB_ += (accSway - baseB_) * a;

        t_ += dt;
        hist_.push_back({t_, speedMs});
        while (!hist_.empty() && hist_.front().t < t_ - std::max(p.speedSpan, 0.05)) hist_.pop_front();
        double vMax = speedMs;
        for (const Sample& s : hist_) vMax = std::max(vMax, s.v);
        const double dv = vMax - speedMs;

        if (cool_ > 0.0) cool_ -= dt;
        if (suppress) { pending_ = false; return r; }
        if (cool_ > 0.0) { pending_ = false; return r; }

        const double g = std::sqrt(hpA * hpA + hpB * hpB) / kG;
        if (!pending_) {
            if (g < p.minG || vMax < p.minSpeed) return r;
            pending_ = true; pendT_ = 0.0; peak_ = g; peakFrontal_ = std::fabs(hpA) >= std::fabs(hpB);
            if (g >= p.strongG) return fire(p, dv);
        } else {
            pendT_ += dt;
            if (g > peak_) { peak_ = g; peakFrontal_ = std::fabs(hpA) >= std::fabs(hpB); }
            if (peak_ >= p.strongG || dv >= p.minDv) return fire(p, dv);
            if (pendT_ >= p.window) pending_ = false;      // the speed never dropped: a jolt, not a crash
        }
        return r;
    }

private:
    struct Sample { double t, v; };
    bool init_ = false, pending_ = false, peakFrontal_ = true;
    double baseA_ = 0.0, baseB_ = 0.0, t_ = 0.0, pendT_ = 0.0, peak_ = 0.0, cool_ = 0.0, lastPeakG_ = 0.0;
    std::deque<Sample> hist_;

    Result fire(const Params& p, double dv) {
        Result r;
        r.hit = true; r.peakG = peak_; r.dv = dv; r.frontal = peakFrontal_;
        const double fromG = (peak_ - p.minG) / std::max(1.0, p.fullG - p.minG), fromV = dv / std::max(1.0, p.fullDv);
        r.severity = std::min(1.0, std::max(0.0, std::max(fromG, fromV)));
        pending_ = false; cool_ = p.cooldown; lastPeakG_ = peak_;
        return r;
    }
};

// Which sounds a crash of this severity plays, as indices into SoundEvent-like layers. The base thump always plays; the rest add to it.
struct Layers {
    int thump = 0;        // 0 light, 1 medium, 2 heavy
    bool metal = false;   // crumpling bodywork
    bool crumple = false; // a long bend of metal, for the bigger ones
    bool branch = false;  // twigs and leaves: a nose-first hit that is not the biggest, which is what a tree usually is (the telemetry cannot say)
    bool glass = false;   // the biggest ones
};
inline Layers PlanLayers(double severity, bool frontal, double roll1, double roll2) {   // roll1, roll2: two random numbers 0..1
    Layers l;
    const double s = std::isfinite(severity) ? std::min(1.0, std::max(0.0, severity)) : 0.0;
    l.thump = s < 0.25 ? 0 : (s < 0.55 ? 1 : 2);
    l.metal = s >= 0.15;
    l.crumple = s >= 0.40;
    l.branch = frontal && s >= 0.08 && s < 0.75 && roll1 < 0.5;
    l.glass = s >= 0.70 && roll2 < 0.6;
    return l;
}

}  // namespace crash
}  // namespace rsa
