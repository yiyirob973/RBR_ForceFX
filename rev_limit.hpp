// The engine's rev limit when nothing tells the plugin what it is: the highest rpm the engine really reaches, learned while driving. Pure logic,
// tested in tests/rev_limit_test.cpp. (Every car in ForceFXCars.ini has its RpmMax, so this is only a fallback.)
//   - A new highest rpm only counts once the engine has stayed above the old one for kHold seconds, so a single spike (a bad packet, a
//     missed shift for one frame) does not raise the limit for the rest of the stage.
//   - FindCandidates looks through one packet for float fields that could be a rev limit, so it can be found in a real packet when the NGP
//     layout does not say where it is (the plugin logs them once; set OffsetRpmMax in ForceFX.ini to the right one).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace RevLimit {

constexpr float kHold = 0.25f;

class Learner {
public:
    void Reset() { *this = Learner(); }
    float Peak() const { return peak_; }
    float Update(float rpm, float dt) {
        if (!std::isfinite(rpm) || rpm < 0.0f) return peak_;
        if (!std::isfinite(dt) || dt <= 0.0f) dt = 0.01f;
        if (rpm <= peak_) { cand_ = 0.0f; candT_ = 0.0f; return peak_; }
        cand_ = std::max(cand_, rpm); candT_ += std::min(dt, 0.1f);
        if (candT_ >= kHold) { peak_ = cand_; cand_ = 0.0f; candT_ = 0.0f; }
        return peak_;
    }
private:
    float peak_ = 0.0f, cand_ = 0.0f, candT_ = 0.0f;
};

// Offsets (multiples of 4, other than rpmOff) of floats between minRpm and maxRpm (after scale) that are at least the current rpm.
inline int FindCandidates(const uint8_t* b, int n, int rpmOff, float rpmScale, float rpmNow, int* out, int maxOut,
                          float minRpm = 3000.0f, float maxRpm = 20000.0f) {
    int k = 0;
    if (!b || n < 4 || maxOut <= 0) return 0;
    for (int off = 0; off + 4 <= n && k < maxOut; off += 4) {
        if (off == rpmOff) continue;
        float v; std::memcpy(&v, b + off, 4);
        if (!std::isfinite(v)) continue;
        const float r = v * rpmScale;
        if (r >= minRpm && r <= maxRpm && r >= rpmNow) out[k++] = off;
    }
    return k;
}

}  // namespace RevLimit
