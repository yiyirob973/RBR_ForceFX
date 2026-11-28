// The inside of the car, as far as the plugin's own sounds go. Pure logic, no Windows or audio access, tested in audio/tests/cabin_test.cpp.
//
// Nothing here is a setting you switch: when the camera is inside the car the sounds are heard through the cabin (muffled, boomy, with
// short reflections); in a view outside the car that goes away. The camera view is read from the game; a view the plugin cannot read counts
// as inside, because the cockpit is where this is mostly played.
#pragma once
#include <algorithm>
#include <cmath>

namespace ffx {
namespace cabin {

// How the cabin changes the sound. A small saloon's cabin keeps little above a kilohertz or two out of the tyre noise and boosts the
// low end, with a boxy resonance around 140 Hz.
constexpr double kLowpassHz = 1200.0;    // 4th order
constexpr double kBassDb = 5.0;          // shelf under 250 Hz
constexpr double kBoomDb = 3.0;          // the cabin's own resonance, 140 Hz
constexpr double kReverb = 0.25;         // short reflections
constexpr double kTrim = 0.9;            // what the cabin takes off the overall level

// The camera types the game reports (read from its memory) that are inside the car. The cockpit family is 3 to 6 (5 is the interior view).
constexpr int kTypeLo = 3, kTypeHi = 6;

// -1 = the camera type is not known, 0 = outside the car, 1 = inside.
inline int ViewOf(int cameraType, int lo, int hi) {
    if (cameraType < 0) return -1;
    return cameraType >= lo && cameraType <= hi ? 1 : 0;
}

namespace detail {
inline float c01(float v) { return !(v > 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }          // NaN comes out as 0
}  // namespace detail

// How inside the car the listener is, 0..1: it moves between the two over a fraction of a second, so a camera switch is a transition and
// not a click. A view that is not known counts as inside.
class Inside {
public:
    float Step(int view, double dt) {
        const float target = view == 0 ? 0.0f : 1.0f;
        const float d = (float)(std::isfinite(dt) && dt > 0.0 ? std::min(dt, 0.1) : 0.01);
        const float tau = target > a_ ? 0.25f : 0.35f;
        a_ += (target - a_) * (1.0f - std::exp(-d / tau));
        a_ = detail::c01(a_);
        return a_;
    }
    float Value() const { return a_; }
    void Set(float a) { a_ = detail::c01(a); }
private:
    float a_ = 1.0f;
};

}  // namespace cabin
}  // namespace ffx
