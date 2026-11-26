// The inside of the car, as far as the plugin's own sounds go. Pure logic, no Windows or audio access, tested in audio/tests/cabin_test.cpp.
//
// Nothing here is a setting you switch: when the camera is inside the car the sounds are heard through the cabin (muffled, boomy, with
// short reflections) and the interior makes its own noise (the fan, wind round the glass, panels buzzing with the engine, things rattling
// over bumps); in a view outside the car all of that goes away. The camera view is read from the game; a view the plugin cannot read counts
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
inline float smooth(float a, float b, float x) { const float t = c01((x - a) / (b - a)); return t * t * (3.0f - 2.0f * t); }
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

struct In {
    float speedKph = 0, rpm = 0, throttle = 0;
    float rough = 0;         // 0..1: how rough the ground and the ride are
    float airborne = 0;      // 0..1
    float inside = 1;        // 0..1, from Inside
    float level = 1;         // the live multiplier from the menu, 0 (none) .. 2
};

struct Ambience {
    float fan = 0;           // the heater blower: steady
    float wind = 0;          // air rushing past the glass and the seals: grows with the speed
    float buzz = 0;          // panels and trim vibrating with the engine
    float buzzRate = 1;      // playback rate of the buzz loop: its pitch follows the engine speed
};

// The loops are the sounds normalised to the same loudness as the rest of the mix, so these are levels relative to a tyre loop at its quiet.
inline Ambience Make(const In& in) {
    using namespace detail;
    Ambience o;
    const float lv = std::isfinite(in.level) ? std::max(0.0f, std::min(2.0f, in.level)) : 1.0f;
    const float k = c01(in.inside) * lv;
    if (c01(in.inside) < 0.005f || k <= 0.0f) return o;                                // outside (the crossfade has finished): nothing
    const float kph = std::isfinite(in.speedKph) ? std::max(0.0f, in.speedKph) : 0.0f;
    const float rpm = std::isfinite(in.rpm) ? std::max(0.0f, in.rpm) : 0.0f;
    o.fan = 0.10f * k;
    o.wind = 0.24f * k * std::pow(c01(kph / 130.0f), 1.7f);
    o.buzz = 0.13f * k * smooth(2200.0f, 6500.0f, rpm) * (0.55f + 0.45f * c01(in.throttle)) * (0.6f + 0.4f * c01(in.rough)) * (1.0f - 0.5f * c01(in.airborne));
    o.buzzRate = std::max(0.3f, std::min(4.0f, rpm / 3000.0f));
    o.fan = std::min(o.fan, 0.5f); o.wind = std::min(o.wind, 0.5f); o.buzz = std::min(o.buzz, 0.5f);
    return o;
}

// Things in the cabin rattle when the car is hit or thrown about: a bump, a landing, a crash. eventGain: the loudness of the event that caused it.
// Returns the level of the rattle, 0 when it is not heard (outside the car).
inline float RattleGain(float eventGain, float inside, float level) {
    using namespace detail;
    const float lv = std::isfinite(level) ? std::max(0.0f, std::min(2.0f, level)) : 1.0f;
    const float in = c01(inside);
    if (in < 0.3f) return 0.0f;
    const float g = std::isfinite(eventGain) ? std::max(0.0f, eventGain) : 0.0f;
    return std::min(0.8f, 0.55f * g * in * lv);
}

}  // namespace cabin
}  // namespace ffx
