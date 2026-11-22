// RBR Surface Audio - portable core: telemetry decode, per-tyre analysis, debris triggers.
// No Windows or audio dependencies, so everything here is unit-tested off-platform.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "../materials.hpp"
#include "../wheel_slip.hpp"   // wheel slip from the segment counter (shared with the road feel)
#include "../tyre_motion.hpp"  // drivetrain, per-tyre ground speed, grip, wheelspin vs slide (shared with the road feel)

namespace rsa {

// ------------------------------------------------------------------ math / dsp
constexpr double kPi = 3.14159265358979323846;
inline double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double sigmoid(double x) { return 1.0 / (1.0 + std::exp(-x)); }
inline double smoothstep(double x) { x = clampd(x, 0, 1); return x * x * (3 - 2 * x); }

class HighPass {
public:
    explicit HighPass(double tau) : tau_(tau) {}
    double process(double x, double dt) {
        if (!init_) { lp_ = x; init_ = true; }
        lp_ += dt / (tau_ + dt) * (x - lp_);
        return x - lp_;
    }
private:
    double tau_, lp_ = 0; bool init_ = false;
};

class Ema {
public:
    explicit Ema(double tau) : tau_(tau) {}
    double update(double x, double dt) {
        if (!has_) { v_ = x; has_ = true; }
        else v_ += (1.0 - std::exp(-dt / tau_)) * (x - v_);
        return v_;
    }
    bool hasValue() const { return has_; }
    double value() const { return v_; }
private:
    double tau_, v_ = 0; bool has_ = false;
};

// -------------------------------------------------------------------- settings
enum StageHint { HintAuto = 0, HintTarmac = 1, HintGravel = 2, HintSnow = 3 };

// Byte offsets of the NGP UDP packet fields. Defaults follow rbr.telemetry.data.TelemetryData.h (NGP 7.4.778).
// Override with "Packet.<Name> = <offset>" in the ini if a future NGP changes the layout.
struct PacketLayout {
    int minLength = 664;
    int steps = 0;                                    // totalSteps_: NGP's own step counter, a clean clock for the wheel counters (-1 = not there)
    int steering = 24, throttle = 28, brake = 32, handbrake = 36, gear = 44;
    int speed = 60, velSurge = 88, velSway = 92, velHeave = 96, yawRate = 108;
    int accSurge = 112, accSway = 116, accHeave = 120;
    int suspBase[4] = {152, 280, 408, 536};   // LF, RF, LB, RB
    int springDeflection = 0, rollbarForce = 4, springForce = 8, damperForce = 12, strutForce = 16;
    int pistonVelocity = 28, brakeDiskTemp = 36, tirePressure = 44, tireTemp = 48, tireTreadTemp = 56, currentSegment = 60;
    int neededLength() const {
        int m = std::max({steps, steering, throttle, brake, handbrake, gear, speed, velSurge, velSway, velHeave, yawRate,
                          accSurge, accSway, accHeave});
        int inBlock = std::max({springDeflection, rollbarForce, springForce, damperForce, strutForce, pistonVelocity,
                                brakeDiskTemp, tirePressure, tireTemp, tireTreadTemp, currentSegment});
        for (int b : suspBase) m = std::max(m, b + inBlock);
        return std::max(minLength, m + 4);
    }
};

struct Settings {
    bool enabled = true;
    int udpPort = 6777;   // second NGP endpoint, so SimHub keeps 6776
    std::string soundsFolder;          // blank = "<plugin folder>\ForceFXAudio\sounds"
    int outputDevice = -1;             // -1 = default waveOut device
    float masterVolume = 1.0f;
    float totalGain = 1.0f;            // final output gain, applied after the soft limiter (linear, 1.0 = unchanged)
    bool normalizeSounds = true;       // bring all sound files to a common loudness when loading
    double eventLevel = 1.0, rumbleLevel = 1.0;   // impacts and landings, and road-edge rumble
    double squeakLevel = 0.6, bottomLevel = 0.8;   // suspension creaks and bump-stop thumps
    double stoneLevel = 2.0;           // stones and debris thrown at the car (separate from impacts: they are quick, light sounds that need more gain)
    double loopBoost = 2.0;            // extra rolling-noise gain at full slip / very rough ground (0 = flat level)
    double loopLevel = 0.25, squealLevel = 0.5, slideLevel = 0.5;   // mix levels per tyre (each of 4 tyres has its own)
    // Where the squeal and the slide begin, in units of the tyre's grip limit (1 = the limit, from PeakSlipRatio / PeakSlipAngleDeg).
    // Squeal is wheelspin: a driven tyre turning faster than the ground under power. A slide is a tyre moving the wrong way:
    // sideways, or locked under braking. Any tyre can slide; only the driven ones can spin.
    double squealStart = Tyre::kSpinStart, slideStart = Tyre::kSlideStart;
    int drivetrain = Tyre::kDefaultDrivetrain;   // which tyres the engine drives (FWD, RWD, AWD); the plugin sets it per car (ForceFXCars.ini), with the car geometry below
    bool cabinEffect = true;           // sound as heard from inside the car: muffled, bassy, with cabin reflections
    double cabinLowpassHz = 2600, cabinBassDb = 6, cabinReverb = 0.2;
    bool speedIsKph = true;
    // Two different speeds exist in the telemetry. The velocity vector (VelSurge/VelSway) is the car's true speed over the
    // ground, in m/s. The Speed field is the speedometer, a driven-wheel speed: it jumps up in wheelspin, drops in lock-up,
    // and in RBR/NGP it also reads about 1.6x the ground speed even while rolling (measured from real logs).
    int groundSource = 0;              // 0 = auto (velocity vector; the Speed field only if the vector is dead), 1 = velocity vector, 2 = Speed field
    double velocityScale = 1.0;        // VelSurge/VelSway to m/s
    double speedFieldScale = 1.609;    // Speed field / true ground speed while rolling (learned while coasting, this is the starting value)

    double looseThreshold = 0.15, roughThreshold = 0.45, slipThreshold = 0.15, softness = 0.35;
    int hint = HintAuto;
    double coastAccelLimit = 1.5;      // m/s^2
    double accelToMs2 = 1.0;           // 9.81 if accelerations are in g
    double yawRateScale = 1.0;         // 0.01745 if yaw rate is in deg/s
    double wheelbase = 2.5, cgToFront = 1.25, trackWidth = 1.5;
    double peakSlipRatio[4] = {0.10, 0.18, 0.20, 0.15};
    double peakSlipAngleDeg[4] = {6.0, 12.0, 13.0, 10.0};

    std::string surfaceNames[4] = {"tarmac", "gravel", "grass", "snow"};
    float classVolume[4] = {0.7f, 1.0f, 0.9f, 0.6f};

    bool enableWheelLoops = true, enableSlide = true, enableSqueal = true, enableStones = true,
         enableImpacts = true, enableLandings = true, enableRumble = true, enableSuspension = true;
    double impactMinVelocity = 0.35;
    bool debugLog = false;
    bool memCrossCheck = true;         // distrust the game surface when it says tarmac but the suspension is hammered. For a blind memory scanner; ForceFX turns it off because its reader is exact (cobbles are tarmac to the game)
    bool startupBeep = true;           // short test tone when the plugin starts: proves the audio output path

    int surfaceMode = 0;                 // 0 auto: game memory when healthy, else telemetry inference; 1 telemetry only
    PacketLayout packet;
};

namespace detail {
inline std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}
inline std::vector<std::string> split(const std::string& s, char c) {
    std::vector<std::string> out; std::string cur; std::stringstream ss(s);
    while (std::getline(ss, cur, c)) out.push_back(trim(cur));
    return out;
}
inline bool toBool(const std::string& v) { return v == "1" || v == "true" || v == "yes" || v == "on"; }
}

// Plain "key = value" lines; ';' or '#' start a comment. Lists are comma separated.
inline void applySetting(Settings& s, const std::string& key, const std::string& v) {
    using namespace detail;
    auto d = [&]() { return std::atof(v.c_str()); };
    auto dl = [&](double* arr) { auto p = split(v, ','); for (size_t i = 0; i < p.size() && i < 4; ++i) arr[i] = std::atof(p[i].c_str()); };
    if (key == "Enabled") s.enabled = toBool(v);
    else if (key == "UdpPort") s.udpPort = std::atoi(v.c_str());
    else if (key == "SoundsFolder") s.soundsFolder = v;
    else if (key == "OutputDevice") s.outputDevice = std::atoi(v.c_str());
    else if (key == "MasterVolume") s.masterVolume = (float)d();
    else if (key == "TotalGain") s.totalGain = (float)clampd(d(), 0.0, 8.0);
    else if (key == "NormalizeSounds") s.normalizeSounds = toBool(v);
    else if (key == "LoopLevel") s.loopLevel = d();
    else if (key == "EventLevel") s.eventLevel = clampd(d(), 0.0, 4.0);
    else if (key == "StoneLevel") s.stoneLevel = clampd(d(), 0.0, 6.0);
    else if (key == "RumbleLevel") s.rumbleLevel = clampd(d(), 0.0, 4.0);
    else if (key == "LoopRoughBoost") s.loopBoost = clampd(d(), 0.0, 10.0);
    else if (key == "SquealLevel") s.squealLevel = d();
    else if (key == "SlideLevel") s.slideLevel = d();
    else if (key == "SquealStart") s.squealStart = clampd(d(), 0.1, 3.0);
    else if (key == "SlideStart") s.slideStart = clampd(d(), 0.2, 4.0);
    else if (key == "CabinEffect") s.cabinEffect = toBool(v);
    else if (key == "CabinLowpassHz") s.cabinLowpassHz = d();
    else if (key == "CabinBassDb") s.cabinBassDb = d();
    else if (key == "CabinReverb") s.cabinReverb = d();
    else if (key == "SpeedIsKph") s.speedIsKph = toBool(v);
    else if (key == "GroundSpeedSource") { std::string lv = v; for (char& ch : lv) ch = (char)std::tolower((unsigned char)ch); s.groundSource = (lv == "velocity" || lv == "1") ? 1 : (lv == "speed" || lv == "speedfield" || lv == "2") ? 2 : 0; }
    else if (key == "VelocityScale") s.velocityScale = clampd(d(), 0.01, 100.0);
    else if (key == "SpeedFieldScale") s.speedFieldScale = clampd(d(), 0.2, 5.0);
    else if (key == "LooseThreshold") s.looseThreshold = d();
    else if (key == "RoughThreshold") s.roughThreshold = d();
    else if (key == "SlipThreshold") s.slipThreshold = d();
    else if (key == "Softness") s.softness = d();
    else if (key == "StageHint") s.hint = std::atoi(v.c_str());
    else if (key == "CoastAccelLimit") s.coastAccelLimit = d();
    else if (key == "AccelToMs2") s.accelToMs2 = d();
    else if (key == "YawRateScale") s.yawRateScale = d();
    else if (key == "PeakSlipRatio") dl(s.peakSlipRatio);
    else if (key == "PeakSlipAngleDeg") dl(s.peakSlipAngleDeg);
    else if (key == "SurfaceNames") { auto p = split(v, ','); for (size_t i = 0; i < p.size() && i < 4; ++i) s.surfaceNames[i] = p[i]; }
    else if (key == "ClassVolume") { auto p = split(v, ','); for (size_t i = 0; i < p.size() && i < 4; ++i) s.classVolume[i] = (float)std::atof(p[i].c_str()); }
    else if (key == "EnableWheelLoops") s.enableWheelLoops = toBool(v);
    else if (key == "EnableSlide") s.enableSlide = toBool(v);
    else if (key == "EnableSqueal") s.enableSqueal = toBool(v);
    else if (key == "EnableStones") s.enableStones = toBool(v);
    else if (key == "EnableImpacts") s.enableImpacts = toBool(v);
    else if (key == "EnableLandings") s.enableLandings = toBool(v);
    else if (key == "EnableRumble") s.enableRumble = toBool(v);
    else if (key == "EnableSuspension") s.enableSuspension = toBool(v);
    else if (key == "SqueakLevel") s.squeakLevel = clampd(d(), 0.0, 6.0);
    else if (key == "BottomOutLevel") s.bottomLevel = clampd(d(), 0.0, 6.0);
    else if (key == "ImpactMinVelocity") s.impactMinVelocity = d();
    else if (key == "DebugLog") s.debugLog = toBool(v);
    else if (key == "StartupBeep") s.startupBeep = toBool(v);
    else if (key == "SurfaceMode") s.surfaceMode = (v == "telemetry" || v == "1") ? 1 : 0;
    else if (key.compare(0, 7, "Packet.") == 0) {
        PacketLayout& p = s.packet;
        std::string k = key.substr(7);
        int val = (int)std::strtol(v.c_str(), nullptr, 0);
        if (k == "MinLength") p.minLength = val; else if (k == "Steering") p.steering = val;
        else if (k == "Throttle") p.throttle = val; else if (k == "Brake") p.brake = val;
        else if (k == "Handbrake") p.handbrake = val; else if (k == "Gear") p.gear = val;
        else if (k == "Speed") p.speed = val; else if (k == "VelSurge") p.velSurge = val;
        else if (k == "VelSway") p.velSway = val; else if (k == "VelHeave") p.velHeave = val;
        else if (k == "YawRate") p.yawRate = val; else if (k == "AccSurge") p.accSurge = val;
        else if (k == "AccSway") p.accSway = val; else if (k == "AccHeave") p.accHeave = val;
        else if (k == "SpringDeflection") p.springDeflection = val; else if (k == "RollbarForce") p.rollbarForce = val;
        else if (k == "SpringForce") p.springForce = val; else if (k == "DamperForce") p.damperForce = val;
        else if (k == "StrutForce") p.strutForce = val; else if (k == "PistonVelocity") p.pistonVelocity = val;
        else if (k == "BrakeDiskTemp") p.brakeDiskTemp = val; else if (k == "TirePressure") p.tirePressure = val;
        else if (k == "TireTemp") p.tireTemp = val; else if (k == "TireTreadTemp") p.tireTreadTemp = val;
        else if (k == "CurrentSegment") p.currentSegment = val;
        else if (k == "SuspBase") { auto q = split(v, ','); for (size_t i = 0; i < q.size() && i < 4; ++i) p.suspBase[i] = (int)std::strtol(q[i].c_str(), nullptr, 0); }
    }
}

// Reads key = value lines. With a section name, only that [section] is read (ForceFX.ini holds several); with an empty
// name every line is read and section headers are ignored.
inline bool loadSettings(const std::string& path, Settings& s, const std::string& section = std::string()) {
    std::ifstream f(path);
    if (!f) return false;
    auto lower = [](std::string v) { for (char& ch : v) ch = (char)std::tolower((unsigned char)ch); return v; };
    const std::string want = lower(section);
    bool inSection = want.empty();
    std::string line;
    while (std::getline(f, line)) {
        size_t c = line.find_first_of(";#");
        if (c != std::string::npos) line = line.substr(0, c);
        line = detail::trim(line);
        if (!line.empty() && line.front() == '[') {
            if (!want.empty()) { size_t r = line.find(']'); inSection = lower(detail::trim(line.substr(1, r == std::string::npos ? std::string::npos : r - 1))) == want; }
            continue;
        }
        if (!inSection) continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = detail::trim(line.substr(0, eq)), v = detail::trim(line.substr(eq + 1));
        applySetting(s, k, v);
    }
    return true;
}

// ------------------------------------------------------------------- telemetry
struct WheelSample {
    float springDeflection = 0, rollbarForce = 0, springForce = 0, damperForce = 0, strutForce = 0;
    float pistonVelocity = 0, brakeDiskTemp = 0, tirePressure = 0, tireTemp = 0, tireTreadTemp = 0;
    uint32_t currentSegment = 0;
};

// Wheel order everywhere: LF, RF, LB, RB.
struct TelemetryFrame {
    double time = 0;          // seconds, monotonic
    uint32_t steps = 0; bool haveSteps = false;   // NGP's step counter (see PacketLayout::steps)
    float speed = 0;          // raw game units (kph or mph)
    float steering = 0, throttle = 0, brake = 0, handbrake = 0;
    int gear = 0;
    float velSurge = 0, velSway = 0, velHeave = 0, yawRate = 0;
    float accSurge = 0, accSway = 0, accHeave = 0;
    WheelSample wheels[4];
};

// Decoder for the NGP UDP packet (rbr.telemetry.data.TelemetryData.h, packed, little-endian, 664 bytes by default).
// Offsets come from PacketLayout so a changed NGP layout can be handled from the ini.
inline bool parsePacket(const uint8_t* d, size_t length, double time, TelemetryFrame& f,
                        const PacketLayout& L = PacketLayout()) {
    if (!d || length < (size_t)L.neededLength()) return false;
    auto F = [&](int o) { float v; std::memcpy(&v, d + o, 4); return std::isfinite(v) ? v : 0.0f; };
    auto U = [&](int o) { uint32_t v; std::memcpy(&v, d + o, 4); return v; };
    f.time = time;
    f.haveSteps = L.steps >= 0; f.steps = f.haveSteps ? U(L.steps) : 0u;
    f.steering = F(L.steering); f.throttle = F(L.throttle); f.brake = F(L.brake); f.handbrake = F(L.handbrake);
    f.gear = (int)U(L.gear);
    f.speed = std::fabs(F(L.speed));
    f.velSurge = F(L.velSurge); f.velSway = F(L.velSway); f.velHeave = F(L.velHeave); f.yawRate = F(L.yawRate);
    f.accSurge = F(L.accSurge); f.accSway = F(L.accSway); f.accHeave = F(L.accHeave);
    for (int i = 0; i < 4; ++i) {
        int b = L.suspBase[i];
        WheelSample w;
        w.springDeflection = F(b + L.springDeflection); w.rollbarForce = F(b + L.rollbarForce);
        w.springForce = F(b + L.springForce); w.damperForce = F(b + L.damperForce); w.strutForce = F(b + L.strutForce);
        w.pistonVelocity = F(b + L.pistonVelocity); w.brakeDiskTemp = F(b + L.brakeDiskTemp);
        w.tirePressure = F(b + L.tirePressure); w.tireTemp = F(b + L.tireTemp); w.tireTreadTemp = F(b + L.tireTreadTemp);
        w.currentSegment = U(b + L.currentSegment);
        f.wheels[i] = w;
    }
    return true;
}

// Cheap plausibility test: values inside physically sensible bounds. A layout mismatch (shifted fields) shows up
// as absurd numbers, so a run of insane frames means "stay silent and say so" rather than play garbage.
inline bool frameLooksSane(const TelemetryFrame& f) {
    if (f.speed > 1000.0f) return false;
    if (std::fabs(f.velSurge) > 1e4f || std::fabs(f.velSway) > 1e4f || std::fabs(f.yawRate) > 1e4f) return false;
    for (const WheelSample& w : f.wheels) {
        if (std::fabs(w.strutForce) > 1e7f || std::fabs(w.pistonVelocity) > 1e3f) return false;
        if (std::fabs(w.springDeflection) > 1e4f) return false;
    }
    return true;
}

class TelemetryHealth {
public:
    void add(bool sane) {
        if (n_ == kN) { if (!ring_[pos_]) --bad_; } else ++n_;
        ring_[pos_] = sane; if (!sane) ++bad_;
        pos_ = (pos_ + 1) % kN;
    }
    bool ok() const { return n_ < 60 || bad_ * 2 <= n_; }
    double insaneFraction() const { return n_ ? (double)bad_ / n_ : 0.0; }
private:
    static constexpr int kN = 120;
    bool ring_[kN] = {}; int n_ = 0, bad_ = 0, pos_ = 0;
};

// -------------------------------------------------------------------- analysis
enum SurfaceClass { Tarmac = 0, Loose = 1, Rough = 2, Slippery = 3 };

// Soft classifier: roughness bands (smooth / loose / rough); smooth + slipping a lot = slippery.
inline void classify(double roughness, double absSlip, bool slipValid, const Settings& s, double p[4]) {
    static const double priors[4][4] = {
        {1.0, 1.0, 1.0, 1.0}, {1.0, 0.3, 0.1, 0.1}, {0.1, 1.0, 0.6, 0.1}, {0.1, 0.6, 0.3, 1.0}};
    double lr = std::log(std::max(roughness, 1e-6));
    double soft = std::max(s.softness, 0.05);
    double sLoose = sigmoid((lr - std::log(s.looseThreshold)) / soft);
    double sRough = sigmoid((lr - std::log(s.roughThreshold)) / soft);
    double tarmac = 1.0 - sLoose, loose = sLoose * (1.0 - sRough), rough = sRough, slippery = 0.0;
    if (slipValid) {
        slippery = tarmac * sigmoid((absSlip - s.slipThreshold) / 0.05);
        tarmac -= slippery;
    }
    const double* pr = priors[std::min(std::max(s.hint, 0), 3)];
    p[0] = tarmac * pr[0]; p[1] = loose * pr[1]; p[2] = rough * pr[2]; p[3] = slippery * pr[3];
    double sum = p[0] + p[1] + p[2] + p[3];
    if (sum < 1e-9) { p[0] = 1; p[1] = p[2] = p[3] = 0; return; }
    for (int i = 0; i < 4; ++i) p[i] /= sum;
}

// Per-tyre signal analysis. Roughness = RMS of high-passed strut force / mean strut force.
// Slip = tyre rotation (8-segment counter, counted over a window) vs the rate expected from the ground speed (learned while coasting).
class WheelAnalyzer {
public:
    double probs[4] = {1, 0, 0, 0};      // final class probabilities (game memory when healthy, else inference)
    double inferred[4] = {1, 0, 0, 0};   // from telemetry only
    int wetness = 0;                     // 0 dry, 1 damp, 2 wet (only known from game memory)
    double load = 0, loadRef = 0, roughness = 0, pistonHp = 0, pistonRms = 0, pistonRefRms = 0;
    double loadSmooth = 0;               // strut load smoothed over about 0.4 s: weight transfer from body roll, braking and crests, without the bumps
    double pistonRaw = 0;                // damper piston velocity as reported (m/s): slow strokes are what make suspension squeak
    double revRate = 0;                  // tyre revolutions per second
    double slip = 0;                     // slip ratio past its dead band: + wheelspin, - lock-up, 0 = rolling or not sure (see wheel_slip.hpp)
    double slipAngle = 0, slipNorm = 0;
    bool slipValid = false;
    double speed_ = 0;                  // ground speed (the car's speed over the ground), m/s
    // Wheel speed: how fast this tyre's surface is turning, as a road speed (m/s). Ground speed and wheel speed are
    // equal while rolling; they split apart under wheelspin (wheel > ground) and braking lock-up (wheel < ground).
    double wheelSpeedMs = 0; bool wheelSpeedValid = false, rollLearned = false;
    // Speed of the tyre's rolling surface. Before the real tyre size is learned the ratio is a guess, so only a clear
    // excess over ground speed (wheelspin) is trusted then; a lock-up is not read from a guessed ratio.
    double contactSpeedMs() const {
        if (!wheelSpeedValid) return speed_;
        if (rollLearned) return wheelSpeedMs;
        double band = 0.12 * std::max(speed_, 3.0);
        return speed_ + std::max(0.0, wheelSpeedMs - speed_ - band);
    }
    // Low-speed / not-yet-learned wheelspin estimate (launches): uses a typical rolling ratio until the real one is
    // learned from coasting, with a dead band so a wrong guess never reads as slip.
    double launchSlip = 0; bool launchValid = false;

    double loadFraction() const { return loadRef <= 1e-6 ? 1.0 : clampd(load / (0.7 * loadRef), 0.0, 1.0); }
    int dominant() const { int b = 0; for (int i = 1; i < 4; ++i) if (probs[i] > probs[b]) b = i; return b; }

    // st: this packet on the clean time axis (see Slip::Timebase); timeGen changes when that axis restarts.
    // tyreGroundMs: the ground speed along the direction this tyre points (Tyre::Kinematics::vLong): a tyre sliding sideways
    // only rolls at that, not at the car's full speed.
    void update(const WheelSample& w, double speedMs, double drivenMs, double dt, bool coasting,
                const Slip::Timebase::Stamp& st, unsigned timeGen, double tyreGroundMs) {
        speed_ = speedMs;   // true ground speed
        load = std::fabs(w.strutForce);
        double ls = loadShort_.update(load, dt);
        loadSmooth = loadQuick_.update(load, dt);
        if (load > 0) loadLong_.update(load, dt);
        loadRef = loadLong_.value();

        double hf = hpForce_.process(w.strutForce, dt);
        double fv = forceVar_.update(hf * hf, dt);
        double denom = std::max(std::max(ls, 0.3 * loadRef), 1e-6);
        roughness = std::sqrt(fv) / denom;

        pistonRaw = w.pistonVelocity;
        pistonHp = hpPiston_.process(w.pistonVelocity, dt);
        pistonRms = std::sqrt(pistonVar_.update(pistonHp * pistonHp, dt));
        pistonRefRms = std::sqrt(pistonRef_.update(pistonHp * pistonHp, dt));

        // Spin of the tyre from its segment counter, counted over a window with enough steps and compared with the real ground
        // speed. Slip only counts past the uncertainty of that count, so a wheel that is just rolling never reads as slipping.
        if (timeGen != timeGen_) { rev_.reset(); timeGen_ = timeGen; }
        rev_.push(w.currentSegment, st.tick, st.secPerTick);
        slip_.update(rev_, st.secPerTick, st.timeErr, tyreGroundMs, coasting, dt);
        const Slip::RevCounter::Est e = rev_.estimate(32.0, 0.5, st.secPerTick);
        revRate = e.valid ? e.revPerSec : 0.0;
        slipValid = slip_.valid;
        slip = slip_.valid ? slip_.ratio : 0.0;
        {
            // Wheel speed of this tyre as a road speed. With the real tyre size learned (from coasting) it comes from this
            // tyre's own rotation counter. Before that it is the driven-wheel speed from the Speed field, which is known from
            // the very first metre: a standing-start launch is detected without having coasted once.
            const bool have = slip_.learned();
            rollLearned = have;
            wheelSpeedValid = have ? slip_.wheelValid : true;
            wheelSpeedMs = have ? (slip_.wheelValid ? slip_.wheelMs : tyreGroundMs) : drivenMs;
            double ls = (wheelSpeedMs - tyreGroundMs) / std::max(tyreGroundMs, 3.0);
            // A wheel speed from a count has an error too: only what lies beyond it is wheelspin or lock-up.
            const double band = (have ? slip_.wheelErrMs / std::max(tyreGroundMs, 3.0) : 0.08) + (have ? 0.03 : 0.0);
            ls = ls >= 0 ? std::max(0.0, ls - band) : (have ? std::min(0.0, ls + band) : 0.0);   // before the size is learned: wheelspin only
            launchValid = wheelSpeedValid && !slipValid;
            launchSlip = launchSmooth_.update(launchValid ? ls : 0.0, dt);
        }
    }

    // Wheelspin and slides, kept apart and each measured against the grip limit of this tyre's ground.
    //  spinN: wheelspin (driven tyre faster than the ground, throttle open), in grip limits.
    //  slideN: sideways slip and lock-up under braking, in grip limits.
    //  spinAmt / slideAmt: the same as 0..1 sound amounts. slipNorm: the larger of the two, how hard the tyre is working the ground.
    double spinN = 0, slideN = 0, spinAmt = 0, slideAmt = 0;
    void updateMotion(double angle, bool angleValid, bool driven, double throttle, double brake, double handbrake, double gate,
                      const Settings& s, double dt) {
        double pkx = 0, pka = 0;
        for (int c = 0; c < 4; ++c) { pkx += probs[c] * s.peakSlipRatio[c]; pka += probs[c] * s.peakSlipAngleDeg[c]; }
        angleValid = angleValid && speed_ > 1.0 && std::fabs(angle) < 0.7;   // slip angle is meaningless at a standstill; > 40 deg is a unit/learning error
        slipAngle = angleValid ? angle : 0.0;
        Tyre::TyreIn in;
        in.driven = driven; in.throttle = throttle; in.brake = brake; in.handbrake = handbrake;
        in.ratioValid = slipValid || launchValid; in.ratio = slipValid ? slip : (launchValid ? launchSlip : 0.0);
        in.angleValid = angleValid; in.angle = angle;
        in.peakRatio = pkx; in.peakAngleDeg = pka; in.gate = gate;
        const Tyre::TyreOut o = Tyre::Classify(in);
        spinN = spinSm_.update(o.spin, dt);
        slideN = slideSm_.update(o.slide, dt);
        spinAmt = Tyre::SpinAmount(spinN, s.squealStart);
        slideAmt = Tyre::SlideAmount(slideN, s.slideStart);
        slipNorm = std::max(spinN, slideN);
    }

    // Smooths the telemetry-based classification into `inferred`.
    void applyProbabilities(const double raw[4], double dt, bool fast = false) { smooth(inferred, raw, fast ? 0.15 : 0.4, dt); }

    // Chooses the final probabilities. A valid game-memory class wins; when memory goes away the result eases back
    // to the inferred values instead of jumping.
    void finalizeProbabilities(bool memValid, int memCls, double dt) {
        if (memValid && memCls >= 0 && memCls < 4) {
            double target[4];
            for (int c = 0; c < 4; ++c) target[c] = (c == memCls) ? 0.97 : 0.01;
            smooth(probs, target, 0.12, dt);
            transition_ = 1.0;
            return;
        }
        if (transition_ > 0) { transition_ -= dt; smooth(probs, inferred, 0.4, dt); return; }
        for (int c = 0; c < 4; ++c) probs[c] = inferred[c];
    }

private:
    HighPass hpForce_{0.12}, hpPiston_{0.12};
    Ema loadShort_{2.0}, loadQuick_{0.4}, loadLong_{8.0}, forceVar_{0.35}, pistonVar_{0.35}, pistonRef_{4.0};
    Ema spinSm_{0.05}, slideSm_{0.05}, launchSmooth_{0.05};
    Slip::RevCounter rev_; Slip::WheelSlip slip_; unsigned timeGen_ = ~0u;
    double transition_ = 0;
    static void smooth(double* cur, const double* target, double tau, double dt) {
        double a = 1.0 - std::exp(-dt / tau), sum = 0;
        for (int i = 0; i < 4; ++i) { cur[i] += a * (target[i] - cur[i]); sum += cur[i]; }
        if (sum > 1e-9) for (int i = 0; i < 4; ++i) cur[i] /= sum;
    }
};

// Per-tyre slip angles and ground speeds from body velocity, yaw rate and steering: see Tyre::Kinematics (tyre_motion.hpp).
class CarDynamics : public Tyre::Kinematics {
public:
    void update(const TelemetryFrame& f, double dt, double speedMs, const Settings& s, bool velocityIsGround = false) {
        Tyre::Params p;
        p.velocityScale = s.velocityScale; p.accelToMs2 = s.accelToMs2; p.yawRateScale = s.yawRateScale;
        p.wheelbase = s.wheelbase; p.cgToFront = s.cgToFront; p.trackWidth = s.trackWidth; p.coastAccelLimit = s.coastAccelLimit;
        Tyre::Input in;
        in.velSurge = f.velSurge; in.velSway = f.velSway; in.yawRate = f.yawRate; in.accSway = f.accSway; in.steering = f.steering;
        for (int i = 0; i < 4; ++i) in.strut[i] = f.wheels[i].strutForce;
        Tyre::Kinematics::update(in, dt, speedMs, p, velocityIsGround);
    }
    double accelScale(const Settings& s) const { Tyre::Params p; p.accelToMs2 = s.accelToMs2; return Tyre::Kinematics::accelScale(p); }
    double yawScale(const Settings& s) const { Tyre::Params p; p.yawRateScale = s.yawRateScale; return Tyre::Kinematics::yawScale(p); }
};

class SurfaceAnalyzer {
public:
    explicit SurfaceAnalyzer(const Settings& s) : s_(s) {}
    WheelAnalyzer wheels[4];
    CarDynamics dynamics;
    Tyre::GripLearner grip;       // how hard the car can accelerate on each kind of ground
    double gripUse = -1.0;        // share of that grip in use now (-1 = not learned yet)
    // Optional game-memory input, set by the controller before update(): per tyre class (0..3) and wetness.
    bool memValid[4] = {false, false, false, false};
    int memClass[4] = {0, 0, 0, 0}, memWet[4] = {0, 0, 0, 0};
    double memLoose[4] = {-1, -1, -1, -1};   // how much loose material the game says is under each tyre (-1 = not known)
    double speedMs = 0, landingAirTime = 0;   // speedMs = true ground speed
    double fieldMs = 0, drivenMs = 0;         // the Speed field (speedometer) and the driven-wheel road speed derived from it
    bool groundFromVelocity = true;
    bool airborne = false, justLanded = false;
    bool fastSettle() const { return speedMs < 12.0; }
    // Wheelspin under power and lock-up under braking happen on every surface, so only slip while cruising says the ground is
    // slippery. Since slip became real (wheel speed against true ground speed), counting it all turned launches and braking on
    // tarmac and gravel into 'snow' and left the classifier unsure of everything.
    static double slipGate(double throttle, double brake, double speedMs) {
        return (1.0 - smoothstep((brake - 0.05) / 0.2)) * (1.0 - smoothstep((throttle - 0.45) / 0.3)) * (speedMs > 5.0 ? 1.0 : 0.0);
    }
    double velDead_ = 0; Ema fieldScale_{4.0};
    int falseAirborneResets = 0;   // times the 1.5 s airborne safety net fired (shown in the debug log)

    void update(const TelemetryFrame& f, double dt) {
        fieldMs = f.speed * (s_.speedIsKph ? (1.0 / 3.6) : 0.44704);
        double velMag = std::sqrt((double)f.velSurge * f.velSurge + (double)f.velSway * f.velSway) * s_.velocityScale;
        // A vector that stays at zero while the car is clearly rolling (steady, no pedals) is a dead channel: fall back to the field.
        if (fieldMs > 8.0 && f.throttle < 0.1f && f.brake < 0.1f) { if (velMag < 0.5) velDead_ += dt; else velDead_ = 0.0; }
        else if (velMag > 2.0) velDead_ = 0.0;
        groundFromVelocity = s_.groundSource == 1 || (s_.groundSource == 0 && velDead_ < 2.0);
        speedMs = groundFromVelocity ? velMag : fieldMs;
        justLanded = false;
        bool coasting = f.throttle < 0.12f && f.brake < 0.03f && f.handbrake < 0.03f &&
                        std::fabs(f.accSway * dynamics.accelScale(s_)) < s_.coastAccelLimit &&
                        std::fabs(f.accSurge * dynamics.accelScale(s_)) < s_.coastAccelLimit && speedMs > 8.0;
        if (groundFromVelocity && coasting && speedMs > 8.0) fieldScale_.update(clampd(fieldMs / speedMs, 1.0, 2.5), dt);
        drivenMs = groundFromVelocity ? fieldMs / (fieldScale_.hasValue() ? fieldScale_.value() : s_.speedFieldScale) : fieldMs;
        const Slip::Timebase::Stamp stamp = timebase_.stamp(f.steps, f.haveSteps, f.time);
        dynamics.update(f, dt, speedMs, s_, groundFromVelocity);   // first: each tyre's own ground speed is what its spin is compared with
        double total = 0, raw[4];
        for (int i = 0; i < 4; ++i) {
            wheels[i].update(f.wheels[i], speedMs, drivenMs, dt, coasting, stamp, timebase_.generation(), dynamics.vLong[i]);
            total += std::fabs(f.wheels[i].strutForce);   // NGP reports compression as negative force
            classify(wheels[i].roughness, std::fabs(wheels[i].slip) * slipGate((double)f.throttle, (double)f.brake, speedMs), wheels[i].slipValid, s_, raw);
            wheels[i].applyProbabilities(raw, dt, speedMs < 12.0);   // settle fast while pulling away from a standstill
            wheels[i].finalizeProbabilities(memValid[i], memClass[i], dt);
            wheels[i].wetness = memValid[i] ? memWet[i] : 0;
        }
        {   // grip: how much of what this ground allows the car is using. Learned only while the wheels are down and acceleration is in m/s^2.
            double share[Tyre::GripLearner::kClasses] = {0, 0, 0, 0, 0, 0, 0, 0}, ld = 0;
            for (int i = 0; i < 4; ++i) ld += std::fabs(f.wheels[i].strutForce);
            if (ld > 1e-6) for (int i = 0; i < 4; ++i) {
                const double wgt = std::fabs(f.wheels[i].strutForce) / ld;
                for (int c = 0; c < 4; ++c) share[c] += wgt * wheels[i].probs[c];
            }
            const double sc = dynamics.accelScale(s_);
            const double aMag = std::sqrt((double)f.accSurge * f.accSurge + (double)f.accSway * f.accSway) * sc;
            grip.update(aMag, share, !airborne && speedMs > 5.0 && dynamics.unitsConfirmed(), dt);
            gripUse = grip.use();
        }
        const double gate = Tyre::GripGate(gripUse);
        for (int i = 0; i < 4; ++i)
            wheels[i].updateMotion(dynamics.slipAngle[i], dynamics.valid, Tyre::Driven(s_.drivetrain, i), (double)f.throttle,
                                   (double)f.brake, (double)f.handbrake, gate, s_, dt);

        lastTotal_ = total;
        bool light = totalLoad_.hasValue() && speedMs > 3.0 && total < 0.12 * totalLoad_.value();
        // Safety net: a real jump never lasts this long, so "airborne" for over 1.5 s means the learned load baseline
        // is wrong for this car or packet (e.g. learned at the start line). Drop it, relearn from now, and stay
        // grounded for a few seconds so the baseline can settle.
        if (light && airTimer_ > 1.5) { totalLoad_ = Ema(3.0); airborne = false; airTimer_ = 0; groundHold_ = 8.0; light = false; falseAirborneResets++; }
        if (groundHold_ > 0) { groundHold_ -= dt; light = false; }
        if (light) { airTimer_ += dt; if (airTimer_ > 0.25) airborne = true; }
        else {
            if (airborne) { justLanded = true; landingAirTime = airTimer_; }
            airborne = false; airTimer_ = 0;
            totalLoad_.update(total, dt);
        }
    }
private:
    const Settings& s_;
    Slip::Timebase timebase_;
    Ema totalLoad_{3.0};
    double airTimer_ = 0, groundHold_ = 0, lastTotal_ = 0;
};

// Coarse surface labels derived from telemetry alone, used as weak ground truth by auto-discovery.
// coarse: 0 tarmac, 1 loose or rough (gravel, dirt, grass), 3 slippery (snow/ice).
struct Label { bool valid = false; int coarse = -1; };
inline int coarseOfClass(int cls) { return cls == 0 ? 0 : (cls == 1 || cls == 2) ? 1 : cls == 3 ? 3 : -1; }

// ---------------------------------------------------------------- game-memory surface
// Source of the game's own material id for each tyre (LF, RF, LB, RB). Implemented by memory.hpp; faked in tests.
class ISurfaceMemory {
public:
    virtual ~ISurfaceMemory() {}
    virtual bool readMaterials(int ids[4]) = 0;      // false = nothing usable this frame
    virtual bool configured() const = 0;             // false while it is only the empty placeholder
    virtual std::string statusText() const = 0;
};

// Decides whether the memory values can be trusted, and drops back to telemetry inference if not:
//  - ids that are not in the material table mean the address is wrong or the layout changed (garbage),
//  - reads that keep failing mean the pointer chain no longer resolves,
//  - sustained extreme disagreement with the suspension data ("tarmac" while the tyre is bouncing on rough ground).
class MemoryHealth {
public:
    enum State { Off, Healthy, Invalid, Unreadable, Disagrees };

    State state() const { return state_; }
    bool usable() const { return state_ == Healthy; }
    const char* text() const {
        switch (state_) {
            case Healthy: return "healthy";
            case Invalid: return "values are not valid material ids";
            case Unreadable: return "reads failing";
            case Disagrees: return "disagrees with telemetry";
            default: return "off";
        }
    }

    void update(bool readOk, const int ids[4], const MaterialTable& table, const SurfaceAnalyzer& a,
                const Settings& s, double dt) {
        bool recognised = false;
        if (readOk) {
            recognised = true;
            for (int w = 0; w < 4; ++w) if (table.cls(ids[w]) == -2) recognised = false;
            failStreak_ = 0;
        } else ++failStreak_;
        ring_[pos_] = readOk && recognised; pos_ = (pos_ + 1) % kN; if (n_ < kN) ++n_;

        // Cross-check against suspension data (only the clear-cut case: tarmac while the tyre is hammered).
        for (int w = 0; s.memCrossCheck && w < 4; ++w) {
            bool clash = readOk && table.cls(ids[w]) == Tarmac && a.speedMs > 8.0 &&
                         a.wheels[w].roughness > 3.0 * s.roughThreshold;
            mismatch_[w] = clash ? mismatch_[w] + dt : std::max(0.0, mismatch_[w] - 0.5 * dt);
            if (mismatch_[w] > 2.0) { holdoff_ = 15.0; mismatch_[w] = 0; }
        }
        if (holdoff_ > 0) holdoff_ -= dt;

        int good = 0; for (int i = 0; i < n_; ++i) if (ring_[i]) ++good;
        if (holdoff_ > 0) state_ = Disagrees;
        else if (failStreak_ > 30) state_ = Unreadable;
        else if (n_ >= 30 && good * 10 < n_ * 9) state_ = Invalid;
        else if (n_ >= 5 && readOk) state_ = Healthy;
    }

private:
    static constexpr int kN = 120;
    bool ring_[kN] = {}; int n_ = 0, pos_ = 0, failStreak_ = 0;
    double mismatch_[4] = {0, 0, 0, 0}, holdoff_ = 0;
    State state_ = Off;
};

// ------------------------------------------------------------------ audio sink
enum class SoundEvent { StoneHit, Impact, Landing, SuspSqueak, SuspBottom, Splash };

constexpr int kMaxSets = 64;

// A "sound set" is the bundle of loop / squeal / slide / hit samples played for a surface. There can be many more sets
// than the four coarse classes: with the game's material read from memory each of the 107 ground materials can have
// its own set, with automatic fall-back to broader ones (see SoundSetMap in audio.hpp).
class ISoundSets {
public:
    virtual ~ISoundSets() {}
    virtual int count() const = 0;                 // <= kMaxSets
    virtual int forMaterial(int id) const = 0;     // -1 if not a ground material
    virtual int forClass(int cls) const = 0;       // coarse class 0..3 (used with telemetry inference)
    virtual float volume(int set) const = 0;
};

// Default: one set per coarse class (what a plain tarmac / gravel / grass / snow pack provides).
class ClassSoundSets : public ISoundSets {
public:
    ClassSoundSets(const Settings& s, const MaterialTable& t) : s_(s), t_(t) {}
    int count() const override { return 4; }
    int forMaterial(int id) const override { int c = t_.cls(id); return c >= 0 ? c : -1; }
    int forClass(int cls) const override { return cls >= 0 && cls < 4 ? cls : 0; }
    float volume(int set) const override { return set >= 0 && set < 4 ? s_.classVolume[set] : 1.f; }
private:
    const Settings& s_; const MaterialTable& t_;
};

// Back-end used by the controller. `set` indexes the active ISoundSets. Gains linear 0..1.
class IAudioSink {
public:
    virtual ~IAudioSink() {}
    virtual void setWheelLoop(int wheel, int cls, float gl, float gr, float rate) = 0;
    virtual void setSlideLoop(int wheel, int cls, float gl, float gr, float rate) = 0;
    virtual void setSquealLoop(int wheel, int cls, float gl, float gr, float rate) = 0;
    virtual void setRumble(float gl, float gr) = 0;
    // Telemetry-driven tone for one tyre's sounds, -1 (deep, dull) .. +1 (bright, crisp). Optional.
    virtual void setWheelTone(int wheel, float tone) { (void)wheel; (void)tone; }
    virtual void trigger(SoundEvent kind, int cls, float gl, float gr, float rate) = 0;
    virtual void silence() = 0;
    // The sink decides which sets exist (it owns the sound bank); the controller follows it.
    virtual const ISoundSets* soundSets() const { return nullptr; }
};

inline void panGains(double pan, float gain, float& l, float& r) {
    double a = (clampd(pan, -1, 1) + 1.0) * kPi / 4.0;
    l = gain * (float)std::cos(a); r = gain * (float)std::sin(a);
}
static const double kWheelPan[4] = {-0.6, 0.6, -0.35, 0.35};
static const float kWheelScale[4] = {1.f, 1.f, 0.8f, 0.8f};

class Rng {
public:
    explicit Rng(uint32_t seed = 12345) : e_(seed) {}
    double next() { return std::uniform_real_distribution<double>(0.0, 1.0)(e_); }
private:
    std::mt19937 e_;
};

// Per-tyre weights over the active sound sets (what each tyre should sound like right now).
struct SetView {
    int count = 4;
    const std::vector<double>* w = nullptr;     // four vectors, one per tyre
    const ISoundSets* sets = nullptr;
    int dominant(int wheel) const { const std::vector<double>& v = w[wheel]; int b = 0; for (int i = 1; i < count; ++i) if (v[i] > v[b]) b = i; return b; }
    float volume(int set) const { return sets ? sets->volume(set) : 1.f; }
};

// Situation streams on top of the per-tyre loops: impacts, landings, stone spray, rumble, squeal/slide.
class DebrisTriggers {
public:
    explicit DebrisTriggers(Rng& rng) : rng_(rng) {}
    double rumbleGain = 0;

    void update(const TelemetryFrame& f, SurfaceAnalyzer& a, const Settings& s, double dt, IAudioSink& sink, const SetView& sv) {
        for (double& c : cooldown_) if (c > 0) c -= dt;
        if (stoneCooldown_ > 0) stoneCooldown_ -= dt;
        if (s.enableSuspension && !a.airborne) suspension(a, s, dt, sink, sv);     // before impacts: a bottom-out replaces the plain impact sound
        if (s.enableImpacts && !a.airborne) impacts(a, s, sink, sv);
        if (s.enableLandings && a.justLanded && a.landingAirTime > 0.15) landing(a, sink, sv);
        if (s.enableStones && !a.airborne) stones(f, a, s, dt, sink, sv);
        rumble(a, s, sink);
        tyres(a, s, sink, sv);
    }

private:
    Rng& rng_;
    double cooldown_[4] = {0, 0, 0, 0};
    double stoneCooldown_ = 0;
    double travel_[4] = {0, 0, 0, 0}, squeakCool_[4] = {0, 0, 0, 0}, bottomCool_[4] = {0, 0, 0, 0}, prevRatio_[4] = {0, 0, 0, 0};
    double squeakGap_ = 0, bottomGap_ = 0;     // the whole car: creaks and thumps are separate events, never a stream

    float rate(double spread) { return (float)(1.0 + (rng_.next() * 2.0 - 1.0) * spread); }

    void impacts(SurfaceAnalyzer& a, const Settings& s, IAudioSink& sink, const SetView& sv) {
        for (int w = 0; w < 4; ++w) {
            auto& wa = a.wheels[w];
            double thr = std::max(s.impactMinVelocity, 4.0 * wa.pistonRefRms), v = std::fabs(wa.pistonHp);
            if (v > thr && cooldown_[w] <= 0 && a.speedMs > 2.0) {
                float intensity = (float)clampd((v - thr) / (2.0 * thr) + 0.25, 0.15, 1.0), l, r;
                panGains(kWheelPan[w], intensity * kWheelScale[w], l, r);
                sink.trigger(SoundEvent::Impact, sv.dominant(w), l, r, rate(0.08));
                cooldown_[w] = 0.12;
            }
        }
    }

    void landing(SurfaceAnalyzer& a, IAudioSink& sink, const SetView& sv) {
        for (int w = 0; w < 4; ++w) {   // each tyre lands on its own surface, panned to its corner
            float intensity = (float)clampd(0.4 + a.landingAirTime, 0.4, 1.0) * (float)(0.7 + 0.3 * rng_.next()), l, r;
            panGains(kWheelPan[w], intensity * kWheelScale[w], l, r);
            sink.trigger(SoundEvent::Landing, sv.dominant(w), l, r, rate(0.05));
        }
    }

    // Stones fly when a tyre's surface moves over ground that has something loose to throw, and how many depends on how hard
    // the tyre works that ground. Surface and telemetry each decide one half:
    //  - the ground: how much loose material is there. The game's own material says it (clean tarmac: none, tarmac with gravel
    //    sprinkled or spread over it: some, gravel, dirt, rough ground and snow: plenty). Without the game's material the telemetry's
    //    own reading of the ground (rough strut force, slipping with little grip) stands in for it;
    //  - the tyre: how fast its surface moves (the faster of ground speed and the wheel's own speed, so wheelspin from a standing
    //    start throws stones too) and how hard it is pushed (slip and throttle). There is no speed cut-off: rate and strength grow
    //    with surface speed.
    // The rate is not a setting. On clean tarmac the ground half is zero, so nothing is thrown whatever the car does.
    void stones(const TelemetryFrame& f, SurfaceAnalyzer& a, const Settings&, double dt, IAudioSink& sink, const SetView& sv) {
        constexpr double kStonesPerSecond = 10.0;      // what a gravel road gives at 90 km/h with the tyres working: fixed, not a setting
        double weight[4], total = 0, spray[4];
        for (int w = 0; w < 4; ++w) {
            const WheelAnalyzer& wa = a.wheels[w];
            spray[w] = std::max(a.speedMs, wa.contactSpeedMs()) * 3.6;                   // km/h
            double ground = a.memValid[w] && a.memLoose[w] >= 0.0 ? a.memLoose[w]
                                                                   : 0.8 * clampd(1.0 - wa.inferred[Tarmac], 0.0, 1.0);
            double slip = std::min(wa.slipNorm, 1.0);                                  // wheelspin or sliding: the tyre is throwing material
            weight[w] = ground * (0.4 + 0.6 * std::max(slip, (double)f.throttle)) * clampd(spray[w] / 90.0, 0.0, 1.5);
            total += weight[w];
        }
        if (total < 0.005) return;
        double r = kStonesPerSecond * (total / 4.0);
        if (rng_.next() > 1.0 - std::exp(-r * dt)) return;
        if (stoneCooldown_ > 0) return;
        stoneCooldown_ = 0.03;
        double pick = rng_.next() * total; int wheel = 0;
        for (int w = 0; w < 4; ++w) { pick -= weight[w]; if (pick <= 0) { wheel = w; break; } }
        float gain = (float)(0.4 + 0.6 * rng_.next()) * (float)clampd(spray[wheel] / 100.0, 0.15, 1.0), l, rr;
        panGains(kWheelPan[wheel], gain * kWheelScale[wheel], l, rr);
        sink.trigger(SoundEvent::StoneHit, sv.dominant(wheel), l, rr, rate(0.12));
    }

    // Suspension sounds from the damper and strut telemetry of each tyre. Both are rare, deliberate events, not texture:
    //  - A creak is the suspension articulating slowly while it carries real weight transfer: body roll in a hard corner, the nose
    //    dropping under braking, a slow roll over a crest. That needs the tyre's smoothed load to be well away from its normal
    //    load (a quarter or more) AND a few centimetres of slow stroke. Normal driving, small bumps and fast strokes make none.
    //  - Bottoming out is the strut load jumping to several times its normal value in one hard compression: the bump stop taking
    //    a real hit. It replaces the plain impact sound for that tyre, and the car makes at most one at a time.
    // Nothing here depends on the surface or on a setting: it follows how the car moves.
    void suspension(SurfaceAnalyzer& a, const Settings& s, double dt, IAudioSink& sink, const SetView&) {
        constexpr double kStressMin = 0.25;       // smoothed load this far from normal (25%) before a creak is possible
        constexpr double kSlowMax = 0.25;         // m/s: faster strokes are knocks, not creaks
        constexpr double kTravelNeeded = 0.08;    // metres of slow, loaded stroke per creak
        constexpr double kBottomRatio = 3.0;      // instantaneous load as a multiple of the normal load
        if (squeakGap_ > 0) squeakGap_ -= dt;
        if (bottomGap_ > 0) bottomGap_ -= dt;
        for (int w = 0; w < 4; ++w) {
            const WheelAnalyzer& wa = a.wheels[w];
            if (squeakCool_[w] > 0) squeakCool_[w] -= dt;
            if (bottomCool_[w] > 0) bottomCool_[w] -= dt;
            const bool haveRef = wa.loadRef > 1e-6;
            const double stress = haveRef ? std::fabs(wa.loadSmooth / wa.loadRef - 1.0) : 0.0;

            const double v = std::fabs(wa.pistonRaw);
            if (a.speedMs > 1.0 && stress > kStressMin && v > 0.01 && v < kSlowMax && cooldown_[w] <= 0)
                travel_[w] += v * dt;
            else
                travel_[w] = std::max(0.0, travel_[w] - 0.05 * dt);
            if (travel_[w] > kTravelNeeded && squeakCool_[w] <= 0 && squeakGap_ <= 0) {
                float intensity = (float)clampd(0.25 + 0.6 * (stress - kStressMin) / 0.75 + 0.15 * rng_.next(), 0.2, 0.8), l, r;
                panGains(kWheelPan[w], intensity * kWheelScale[w], l, r);
                sink.trigger(SoundEvent::SuspSqueak, w, l, r, rate(0.12));
                travel_[w] = 0; squeakCool_[w] = 3.0; squeakGap_ = 1.5;
            }

            const double ratio = haveRef ? wa.load / wa.loadRef : 0.0;
            const double hit = std::fabs(wa.pistonHp);
            const bool hard = hit > std::max(2.0 * s.impactMinVelocity, 6.0 * wa.pistonRefRms);   // a real, fast compression too
            if (ratio > kBottomRatio && prevRatio_[w] <= kBottomRatio && hard && bottomCool_[w] <= 0 && bottomGap_ <= 0 && a.speedMs > 2.0) {
                float intensity = (float)clampd(0.4 + (ratio - kBottomRatio) / (2.0 * kBottomRatio), 0.4, 1.0), l, r;
                panGains(kWheelPan[w], intensity * kWheelScale[w], l, r);
                sink.trigger(SoundEvent::SuspBottom, w, l, r, rate(0.06));
                bottomCool_[w] = 1.0; bottomGap_ = 0.5;
                cooldown_[w] = std::max(cooldown_[w], 0.3);                    // the plain impact sound would double it
            }
            prevRatio_[w] = ratio;
        }
    }

    // Road-edge rumble deliberately compares the two sides of the car.
    void rumble(SurfaceAnalyzer& a, const Settings& s, IAudioSink& sink) {
        double gain = 0, pan = 0;
        if (s.enableRumble && a.speedMs > 8.0 && !a.airborne) {
            double left = std::max(0.5 * (a.wheels[0].roughness + a.wheels[2].roughness), 1e-6);
            double right = std::max(0.5 * (a.wheels[1].roughness + a.wheels[3].roughness), 1e-6);
            double hi = std::max(left, right), lo = std::min(left, right);
            if (hi > s.looseThreshold) {
                gain = smoothstep((std::log(hi / lo) - 0.7) / 0.8) * clampd(a.speedMs / 15.0, 0.0, 1.0);
                pan = left > right ? -0.8 : 0.8;
            }
        }
        rumbleGain = gain;
        float l, r; panGains(pan, (float)gain * 0.8f, l, r);
        sink.setRumble(l, r);
    }

    // Squeal is wheelspin: a driven tyre turning faster than the ground under power (a rear-wheel-drive car only has it at the
    // rear, front-wheel drive at the front). The slide is a tyre moving the wrong way: sideways, or locked under braking.
    // Both are measured against the grip limit of the ground under that tyre.
    void tyres(SurfaceAnalyzer& a, const Settings& s, IAudioSink& sink, const SetView& sv) {
        for (int w = 0; w < 4; ++w) {
            auto& wa = a.wheels[w];
            double load = std::sqrt(std::max(0.35, wa.loadFraction()));
            bool on = a.speedMs > 1.0 || wa.launchSlip > 0.05;   // standing wheelspin counts
            double air = a.airborne ? 0.0 : 1.0;   // in the air: no tyre noise
            double squeal = 0, slide = 0;
            if (on && s.enableSqueal) squeal = wa.spinAmt * load * air;
            if (on && s.enableSlide) slide = wa.slideAmt * load * air;
            const double n = std::max(wa.spinN, wa.slideN);
            float sqRate = (float)(0.85 + 0.5 * clampd((n - 0.7) / 1.0, 0.0, 1.0));
            float slRate = (float)(0.9 + 0.4 * clampd(std::max(a.speedMs, wa.contactSpeedMs()) / 40.0, 0.0, 1.0));   // sliding speed: ground or spinning surface, whichever is faster
            const std::vector<double>& weights = sv.w[w];
            double sq = 0; for (int i = 0; i < sv.count; ++i) sq += weights[i] * weights[i];
            if (sq < 1e-9) sq = 1;
            for (int i = 0; i < sv.count; ++i) {
                double share = std::sqrt(weights[i] * weights[i] / sq); float l, r;   // equal power: uncorrelated sounds crossfade without a dip
                panGains(kWheelPan[w], (float)(squeal * share * s.squealLevel) * sv.volume(i) * kWheelScale[w], l, r);
                sink.setSquealLoop(w, i, l, r, sqRate);
                panGains(kWheelPan[w], (float)(slide * share * s.slideLevel) * sv.volume(i) * kWheelScale[w], l, r);
                sink.setSlideLoop(w, i, l, r, slRate);
            }
        }
    }
};

// Telemetry frame in -> analysis -> gains / triggers on the sink.
class SurfaceAudioController {
    const Settings& s_;
    IAudioSink& sink_;
    Rng rng_;
    double lastTime_ = -1;
    ISurfaceMemory* mem_ = nullptr;
    MaterialTable table_;
    MemoryHealth health_;
    ClassSoundSets defaultSets_;
    const ISoundSets* sets_ = nullptr;
    std::vector<double> setW_[4], target_;
    TelemetryHealth telemetry_;
    Label label_;
    double labelStable_ = 0;

public:
    SurfaceAudioController(const Settings& s, IAudioSink& sink, uint32_t seed = 12345)
        : s_(s), sink_(sink), rng_(seed), defaultSets_(s, table_), analyzer(s), debris(rng_) { setSoundSets(sink.soundSets()); }
    SurfaceAnalyzer analyzer;
    DebrisTriggers debris;

    // Optional: the game-memory surface reader (null / unconfigured = telemetry inference only).
    void setSurfaceMemory(ISurfaceMemory* m) { mem_ = m; }
    // Normally taken from the sink; override only in tests. Null = one set per coarse class.
    void setSoundSets(const ISoundSets* sets) {
        sets_ = sets ? sets : &defaultSets_;
        int n = std::min(sets_->count(), kMaxSets);
        for (auto& v : setW_) { v.assign(n, 0.0); v[std::min(std::max(sets_->forClass(0), 0), n - 1)] = 1.0; }
        target_.assign(n, 0.0);
    }
    MaterialTable& materials() { return table_; }
    bool usingMemory() const { return mem_ && s_.surfaceMode == 0 && mem_->configured() && health_.usable(); }

    std::string surfaceSourceText() const {
        if (!telemetry_.ok()) return "Telemetry: packet layout looks wrong (values out of range); silent. Check Packet.* in the ini.";
        if (s_.surfaceMode != 0) return "Surface source: telemetry inference (memory disabled in ini)";
        if (!mem_ || !mem_->configured()) return "Surface source: telemetry inference (memory: " + (mem_ ? mem_->statusText() : std::string("no reader")) + ")";
        if (health_.usable()) return "Surface source: game memory (" + mem_->statusText() + ")";
        return std::string("Surface source: telemetry inference (memory ") + health_.text() + "; " + mem_->statusText() + ")";
    }

    void onFrame(const TelemetryFrame& f) {
        if (!s_.enabled) { sink_.silence(); return; }
        telemetry_.add(frameLooksSane(f));
        if (!telemetry_.ok()) { sink_.silence(); return; }
        bool firstFrame = lastTime_ < 0;
        double dt = lastTime_ < 0 ? 1.0 / 60.0 : f.time - lastTime_;
        lastTime_ = f.time;
        dt = clampd(dt, 0.001, 0.1);

        int ids[4] = {0, 0, 0, 0};
        bool readOk = false;
        bool useMem = mem_ && s_.surfaceMode == 0 && mem_->configured();
        if (useMem) readOk = mem_->readMaterials(ids);
        for (int w = 0; w < 4; ++w) {
            int cls = readOk ? table_.cls(ids[w]) : -2;
            analyzer.memValid[w] = useMem && health_.usable() && cls >= 0;   // objects / unknown ids: no info for this tyre
            analyzer.memClass[w] = cls >= 0 ? cls : 0;
            analyzer.memWet[w] = readOk ? table_.wet(ids[w]) : 0;
            analyzer.memLoose[w] = readOk ? table_.looseness(ids[w]) : -1.0;
        }

        analyzer.update(f, dt);
        updateLabel(dt);
        if (useMem) health_.update(readOk, ids, table_, analyzer, s_, dt);
        updateSetWeights(ids, dt, firstFrame);
        wheelLoops();
        SetView sv; sv.count = (int)target_.size(); sv.w = setW_; sv.sets = sets_;
        debris.update(f, analyzer, s_, dt, sink_, sv);
    }
    void onTelemetryLost() { lastTime_ = -1; sink_.silence(); }
    double insaneFraction() const { return telemetry_.insaneFraction(); }

private:
    // A label is only offered when all four tyres confidently and steadily show the same coarse class.
    void updateLabel(double dt) {
        int coarse = -2;
        bool ok = analyzer.speedMs > 8.0 && !analyzer.airborne;
        for (int w = 0; ok && w < 4; ++w) {
            const WheelAnalyzer& wa = analyzer.wheels[w];
            int best = 0; for (int c = 1; c < 4; ++c) if (wa.inferred[c] > wa.inferred[best]) best = c;
            int cc = wa.inferred[best] > 0.8 ? coarseOfClass(best) : -1;
            if (cc < 0 || (coarse != -2 && cc != coarse)) ok = false; else coarse = cc;
        }
        // A StageHint set by the user is ground truth: use it as the label (tarmac / loose / snow) while driving normally,
        // so discovery no longer depends on the plugin's own, possibly wrong, guess.
        if (s_.hint != HintAuto) {
            ok = analyzer.speedMs > 5.0 && !analyzer.airborne;
            coarse = s_.hint == HintTarmac ? 0 : s_.hint == HintGravel ? 1 : 3;
        }
        labelStable_ = ok ? labelStable_ + dt : 0.0;
        label_.valid = ok && labelStable_ >= 1.0;
        label_.coarse = label_.valid ? coarse : -1;
    }

    // Each tyre's weight over the sound sets: the exact material's set when game memory is valid for that tyre,
    // otherwise the telemetry-inferred classes mapped onto their sets. Smoothed so sets crossfade.
    void updateSetWeights(const int ids[4], double dt, bool snap = false) {
        int n = (int)target_.size();
        for (int w = 0; w < 4; ++w) {
            std::vector<double>& cur = setW_[w];
            std::fill(target_.begin(), target_.end(), 0.0);
            bool mem = analyzer.memValid[w];
            if (mem) {
                int st = sets_->forMaterial(ids[w]);
                if (st < 0 || st >= n) st = sets_->forClass(analyzer.memClass[w]);
                target_[std::min(std::max(st, 0), n - 1)] = 1.0;
            } else {
                for (int c = 0; c < 4; ++c) {
                    int st = std::min(std::max(sets_->forClass(c), 0), n - 1);
                    target_[st] += analyzer.wheels[w].inferred[c];
                }
            }
            double a = snap ? 1.0 : 1.0 - std::exp(-dt / (mem ? 0.12 : (analyzer.fastSettle() ? 0.15 : 0.4))), sum = 0;
            for (int i = 0; i < n; ++i) { cur[i] += a * (target_[i] - cur[i]); sum += cur[i]; }
            if (sum > 1e-9) for (int i = 0; i < n; ++i) cur[i] /= sum;
        }
    }

    void wheelLoops() {
        // Each tyre's rolling noise follows its OWN wheel speed (how fast its surface turns), not just the car's ground
        // speed: a spinning tyre sounds faster and throws more, a locked tyre stops rolling (the slide noise takes over).
        int n = (int)target_.size();
        for (int w = 0; w < 4; ++w) {
            auto& wa = analyzer.wheels[w];
            double kph = wa.contactSpeedMs() * 3.6;
            double speedGain = clampd((kph - 1.0) / 110.0, 0.0, 1.0);
            double shape = 0.2 + 0.8 * std::pow(speedGain, 0.7);
            if (kph < 0.8) shape = 0.0;                      // sounds whenever the tyre is turning at all
            if (analyzer.airborne) shape = 0.0;              // wheels in the air make no tyre noise (fades out smoothly in the mixer)
            float rate = (float)(0.8 + 0.6 * clampd(kph / 220.0, 0.0, 1.0));
            const std::vector<double>& weights = setW_[w];
            // Cruising is quiet; the rolling noise swells only with tyre slip or very rough ground (rocks, rubble).
            double slipAct = (wa.slipValid || wa.launchValid) ? smoothstep((wa.slipNorm - 0.4) / 0.8) : 0.0;
            double roughAct = smoothstep((wa.roughness / std::max(s_.roughThreshold, 1e-3) - 0.8) / 0.8);   // from 0.8x to 1.6x the 'rough' threshold
            double activity = std::max(slipAct, roughAct);
            double sq = 0; for (int i = 0; i < n; ++i) sq += weights[i] * weights[i];
            if (sq < 1e-9) sq = 1;
            for (int i = 0; i < n; ++i) {
                double share = std::sqrt(weights[i] * weights[i] / sq);   // equal power: two uncorrelated sounds crossfade without a dip
                double g = s_.enableWheelLoops ? share * s_.loopLevel * (1.0 + s_.loopBoost * activity) * sets_->volume(i) * shape * std::max(0.4, wa.loadFraction()) : 0.0;
                float l, r; panGains(kWheelPan[w], (float)g * kWheelScale[w], l, r);
                sink_.setWheelLoop(w, i, l, r, rate);
            }
            // Tone from the telemetry, on top of each sound set's own character (dry stones/rubble bright, wet/snow deep):
            // faster = brighter, more loose/rough ground = crisper, slippery (low grip, smooth) = duller and deeper,
            // a heavily loaded tyre = a little fuller.
            double tone = 0.5 * (clampd(kph / 150.0, 0.0, 1.0) - 0.4) + 0.35 * (wa.inferred[1] + wa.inferred[2]) - 0.7 * wa.inferred[3]
                          - 0.15 * clampd(wa.loadFraction() - 1.0, 0.0, 1.0);
            sink_.setWheelTone(w, (float)clampd(tone, -1.0, 1.0));
        }
    }
};

}  // namespace rsa
