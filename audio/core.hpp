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
#include "materials.hpp"

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

// Where to find the game's own surface (material id) value in memory. Everything is data, nothing is
// hard-coded, so a game or NGP update only needs a new profile, never a rebuild. See README.
struct MemoryProfile {
    std::string name;
    std::string module = "exe";        // "exe" = RichardBurnsRally_SSE.exe, or a loaded DLL name such as NGP.dll
    uint32_t imageSize = 0;            // build fingerprint of that module (PE SizeOfImage); 0 = any
    uint32_t peTimestamp = 0;          // build fingerprint (PE TimeDateStamp); 0 = any
    std::string signature;             // byte pattern with ?? wildcards, e.g. "A1 ?? ?? ?? ?? 8B 40 10"
    int sigOffset = 0;                 // offset inside the match of a 4-byte absolute address operand
    uint32_t baseOffset = 0;           // used when no signature: module base + this = address of the base pointer
    std::vector<int32_t> chain;        // for each: ptr = read32(addr); addr = ptr + offset
    int width = 4;                     // value size in bytes: 1, 2 or 4
    int wheelStride = 4;               // bytes between the four wheels' values (LF, RF, LB, RB)
    bool perWheel = true;              // false = a single value for the whole car
    // Filled in by auto-discovery (learned.ini). A profile you write by hand is verified by default.
    bool verified = true;
    int agree[4] = {0, 0, 0, 0}, disagree[4] = {0, 0, 0, 0};   // per coarse class: tarmac, loose/rough, (unused), slippery
};

// Byte offsets of the NGP UDP packet fields. Defaults follow rbr.telemetry.data.TelemetryData.h (NGP 7.4.778).
// Override with "Packet.<Name> = <offset>" in the ini if a future NGP changes the layout.
struct PacketLayout {
    int minLength = 664;
    int steering = 24, throttle = 28, brake = 32, handbrake = 36, gear = 44;
    int speed = 60, velSurge = 88, velSway = 92, velHeave = 96, yawRate = 108;
    int accSurge = 112, accSway = 116, accHeave = 120;
    int suspBase[4] = {152, 280, 408, 536};   // LF, RF, LB, RB
    int springDeflection = 0, rollbarForce = 4, springForce = 8, damperForce = 12, strutForce = 16;
    int pistonVelocity = 28, brakeDiskTemp = 36, tirePressure = 44, tireTemp = 48, tireTreadTemp = 56, currentSegment = 60;
    int neededLength() const {
        int m = std::max({steering, throttle, brake, handbrake, gear, speed, velSurge, velSway, velHeave, yawRate,
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
    double squeakLevel = 1.0, bottomLevel = 1.0;   // suspension creaks and bump-stop thumps
    double squeakSensitivity = 1.0;    // how readily slow, loaded suspension travel makes a creak (0 = never)
    double bottomRatio = 2.2;          // a tyre's strut load above this multiple of its normal load counts as bottoming out
    double stoneLevel = 2.0;           // stones and debris thrown at the car (separate from impacts: they are quick, light sounds that need more gain)
    double loopBoost = 2.0;            // extra rolling-noise gain at full slip / very rough ground (0 = flat level)
    double loopLevel = 0.25, squealLevel = 0.5, slideLevel = 0.5;   // mix levels per tyre (each of 4 tyres has its own)
    bool cabinEffect = true;           // sound as heard from inside the car: muffled, bassy, with cabin reflections
    double cabinLowpassHz = 2600, cabinBassDb = 6, cabinReverb = 0.2;
    bool speedIsKph = true;
    // Two different speeds exist in the telemetry. The velocity vector (VelSurge/VelSway) is the car's true speed over the
    // ground, in m/s. The Speed field is the speedometer, a driven-wheel speed: it jumps up in wheelspin, drops in lock-up,
    // and in RBR/NGP it also reads about 1.6x the ground speed even while rolling (measured from real logs).
    int groundSource = 0;              // 0 = auto (velocity vector; the Speed field only if the vector is dead), 1 = velocity vector, 2 = Speed field
    double velocityScale = 1.0;        // VelSurge/VelSway to m/s
    double speedFieldScale = 1.609;    // Speed field / true ground speed while rolling (learned while coasting, this is the starting value)

    double looseThreshold = 0.05, roughThreshold = 0.20, slipThreshold = 0.15, softness = 0.35;
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
    double stoneRate = 10.0;
    double impactMinVelocity = 0.35;
    bool debugLog = false;
    bool startupBeep = true;           // short test tone when the plugin starts: proves the audio output path

    bool autoDiscover = true;            // let the plugin find the surface value in memory by itself (read-only)
    int surfaceMode = 0;                 // 0 auto: game memory when healthy, else telemetry inference; 1 telemetry only
    std::vector<MemoryProfile> memProfiles;
    std::vector<MemoryProfile> rejectedProfiles;   // decoys auto-discovery already threw out; remembered so rescans do not bring them back
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
    else if (key == "Wheelbase") s.wheelbase = d();
    else if (key == "CgToFront") s.cgToFront = d();
    else if (key == "TrackWidth") s.trackWidth = d();
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
    else if (key == "SqueakSensitivity") s.squeakSensitivity = clampd(d(), 0.0, 4.0);
    else if (key == "BottomOutRatio") s.bottomRatio = clampd(d(), 1.2, 8.0);
    else if (key == "StoneRate") s.stoneRate = d();
    else if (key == "ImpactMinVelocity") s.impactMinVelocity = d();
    else if (key == "DebugLog") s.debugLog = toBool(v);
    else if (key == "StartupBeep") s.startupBeep = toBool(v);
    else if (key == "AutoDiscover") s.autoDiscover = toBool(v);
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

inline void applyProfileSetting(MemoryProfile& m, const std::string& key, const std::string& v) {
    using namespace detail;
    auto num = [&]() { return (uint32_t)std::strtoul(v.c_str(), nullptr, 0); };
    if (key == "Module") m.module = v;
    else if (key == "ImageSize") m.imageSize = num();
    else if (key == "PeTimestamp") m.peTimestamp = num();
    else if (key == "Signature") m.signature = v;
    else if (key == "SigOffset") m.sigOffset = (int)std::strtol(v.c_str(), nullptr, 0);
    else if (key == "BaseOffset") m.baseOffset = num();
    else if (key == "Chain") { m.chain.clear(); for (auto& t : split(v, ',')) if (!t.empty()) m.chain.push_back((int32_t)std::strtol(t.c_str(), nullptr, 0)); }
    else if (key == "Width") m.width = std::atoi(v.c_str());
    else if (key == "WheelStride") m.wheelStride = std::atoi(v.c_str());
    else if (key == "PerWheel") m.perWheel = toBool(v);
    else if (key == "Verified") m.verified = toBool(v);
    else if (key == "Agree" || key == "Disagree") {
        int* dst = key == "Agree" ? m.agree : m.disagree;
        auto p = split(v, ',');
        for (size_t i = 0; i < p.size() && i < 4; ++i) dst[i] = std::atoi(p[i].c_str());
    }
}

inline bool loadSettings(const std::string& path, Settings& s) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    MemoryProfile* cur = nullptr;
    while (std::getline(f, line)) {
        size_t c = line.find_first_of(";#");
        if (c != std::string::npos) line = line.substr(0, c);
        line = detail::trim(line);
        if (line.size() > 2 && line.front() == '[' && line.back() == ']') {   // "[Memory: name]" starts a profile
            std::string h = detail::trim(line.substr(1, line.size() - 2));
            if (h.compare(0, 7, "Memory:") == 0 || h.compare(0, 7, "Memory ") == 0) {
                s.memProfiles.emplace_back();
                s.memProfiles.back().name = detail::trim(h.substr(7));
                cur = &s.memProfiles.back();
            } else if (h.compare(0, 9, "Rejected:") == 0) {
                s.rejectedProfiles.emplace_back();
                s.rejectedProfiles.back().name = detail::trim(h.substr(9));
                cur = &s.rejectedProfiles.back();
            } else cur = nullptr;
            continue;
        }
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = detail::trim(line.substr(0, eq)), v = detail::trim(line.substr(eq + 1));
        if (cur) applyProfileSetting(*cur, k, v); else applySetting(s, k, v);
    }
    return true;
}

// Returns the ini text with its StageHint line set to `hint` (comment kept, line endings kept); the line is appended if absent.
inline std::string withStageHint(const std::string& text, int hint) {
    hint = hint < 0 ? 0 : hint > 3 ? 3 : hint;
    std::string out; bool done = false; size_t pos = 0;
    while (pos <= text.size()) {
        size_t nl = text.find('\n', pos), end = nl == std::string::npos ? text.size() : nl;
        std::string ln = text.substr(pos, end - pos);
        bool cr = !ln.empty() && ln.back() == '\r'; if (cr) ln.pop_back();
        size_t i = ln.find_first_not_of(" \t");
        if (!done && i != std::string::npos && ln.compare(i, 9, "StageHint") == 0 &&
            (i + 9 >= ln.size() || ln[i + 9] == ' ' || ln[i + 9] == '=' || ln[i + 9] == '\t')) {
            size_t semi = ln.find(';');
            ln = "StageHint = " + std::to_string(hint) + (semi == std::string::npos ? "" : "             " + ln.substr(semi));
            done = true;
        }
        out += ln; if (cr) out += '\r';
        if (nl == std::string::npos) break;
        out += '\n'; pos = nl + 1;
    }
    if (!done) { if (!out.empty() && out.back() != '\n') out += "\r\n"; out += "StageHint = " + std::to_string(hint) + "\r\n"; }
    return out;
}

inline bool saveProfiles(const std::string& path, const std::vector<MemoryProfile>& profiles, const std::vector<MemoryProfile>& rejected = {}) {
    std::ofstream f(path);
    if (!f) return false;
    f << "; Written by RBR Surface Audio auto-discovery. Safe to delete; it is rebuilt automatically.\n";
    auto list = [](const int* a) { std::ostringstream o; o << a[0] << "," << a[1] << "," << a[2] << "," << a[3]; return o.str(); };
    for (const MemoryProfile& p : profiles) {
        f << "\n[Memory: " << p.name << "]\nModule = " << p.module << "\nImageSize = 0x" << std::hex << p.imageSize
          << "\nPeTimestamp = 0x" << p.peTimestamp << "\nBaseOffset = 0x" << p.baseOffset << std::dec << "\nChain = ";
        for (size_t i = 0; i < p.chain.size(); ++i) f << (i ? "," : "") << p.chain[i];
        f << "\nWidth = " << p.width << "\nWheelStride = " << p.wheelStride << "\nPerWheel = " << (p.perWheel ? 1 : 0)
          << "\nVerified = " << (p.verified ? 1 : 0) << "\nAgree = " << list(p.agree) << "\nDisagree = " << list(p.disagree) << "\n";
        if (!p.signature.empty()) f << "Signature = " << p.signature << "\nSigOffset = " << p.sigOffset << "\n";
    }
    for (const MemoryProfile& p : rejected) {
        f << "\n[Rejected: " << p.name << "]\nModule = " << p.module << "\nImageSize = 0x" << std::hex << p.imageSize
          << "\nPeTimestamp = 0x" << p.peTimestamp << "\nBaseOffset = 0x" << p.baseOffset << std::dec << "\nChain = ";
        for (size_t i = 0; i < p.chain.size(); ++i) f << (i ? "," : "") << p.chain[i];
        f << "\nWidth = " << p.width << "\nWheelStride = " << p.wheelStride << "\nPerWheel = " << (p.perWheel ? 1 : 0) << "\nVerified = 0\n";
    }
    return (bool)f;
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
    float speed = 0;          // raw game units (kph or mph)
    float steering = 0, throttle = 0, brake = 0, handbrake = 0;
    int gear = 0;
    float velSurge = 0, velSway = 0, velHeave = 0, yawRate = 0;
    float accSurge = 0, accSway = 0, accHeave = 0;
    WheelSample wheels[4];
};

// Decoder for the NGP UDP packet (rbr.telemetry.data.TelemetryData.h, packed, little-endian, 664 bytes by default).
// Offsets come from PacketLayout so a changed NGP layout can be handled from the ini.
constexpr int kPacketMinLength = 664;

inline bool parsePacket(const uint8_t* d, size_t length, double time, TelemetryFrame& f,
                        const PacketLayout& L = PacketLayout()) {
    if (!d || length < (size_t)L.neededLength()) return false;
    auto F = [&](int o) { float v; std::memcpy(&v, d + o, 4); return std::isfinite(v) ? v : 0.0f; };
    auto U = [&](int o) { uint32_t v; std::memcpy(&v, d + o, 4); return v; };
    f.time = time;
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
// Slip = tyre rotation (8-segment counter) vs the rate expected from car speed (learned while coasting).
class WheelAnalyzer {
public:
    double probs[4] = {1, 0, 0, 0};      // final class probabilities (game memory when healthy, else inference)
    double inferred[4] = {1, 0, 0, 0};   // from telemetry only
    int wetness = 0;                     // 0 dry, 1 damp, 2 wet (only known from game memory)
    double load = 0, loadRef = 0, roughness = 0, pistonHp = 0, pistonRms = 0, pistonRefRms = 0;
    double pistonRaw = 0;                // damper piston velocity as reported (m/s): slow strokes are what make suspension squeak
    double revRate = 0, slip = 0, slipAngle = 0, slipNorm = 0;
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

    void update(const WheelSample& w, double speedMs, double drivenMs, double dt, bool coasting) {
        speed_ = speedMs;   // true ground speed
        load = std::fabs(w.strutForce);
        double ls = loadShort_.update(load, dt);
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

        uint32_t seg = w.currentSegment % 8u;
        if (haveSeg_ && dt > 1e-4) {
            int delta = (int)((seg + 8u - lastSeg_) % 8u);
            revRate_.update(delta / 8.0 / dt, dt);
        }
        lastSeg_ = seg; haveSeg_ = true;
        revRate = revRate_.value();

        if (coasting && speedMs > 8.0 && revRate_.hasValue()) rollRatio_.update(revRate_.value() / speedMs, dt);
        slipValid = rollRatio_.hasValue() && rollRatio_.value() > 1e-3 && speedMs > 3.0;
        double raw = slipValid ? revRate_.value() / (rollRatio_.value() * speedMs) - 1.0 : 0.0;
        slip = slipSmooth_.update(raw, dt);
        {
            // Wheel speed of this tyre as a road speed. With the real tyre size learned (from coasting) it comes from this
            // tyre's own rotation counter. Before that it is the driven-wheel speed from the Speed field, which is known from
            // the very first metre: a standing-start launch is detected without having coasted once.
            bool have = rollRatio_.hasValue() && rollRatio_.value() > 1e-3;
            rollLearned = have;
            wheelSpeedValid = have ? revRate_.hasValue() : true;
            wheelSpeedMs = have ? (revRate_.hasValue() ? revRate_.value() / rollRatio_.value() : speedMs) : drivenMs;
            double ls = (wheelSpeedMs - speedMs) / std::max(speedMs, 3.0);
            if (!have) ls = std::max(0.0, ls - 0.08);   // an estimate: wheelspin only (a lock-up is not read from the driven speed)
            launchValid = wheelSpeedValid && !slipValid;
            launchSlip = launchSmooth_.update(launchValid ? ls : 0.0, dt);
        }
    }

    // Combines longitudinal and lateral slip, each normalised by the peak for this tyre's surface.
    void updateCombinedSlip(double angle, bool angleValid, const Settings& s, double dt) {
        double pkx = 0, pka = 0;
        for (int c = 0; c < 4; ++c) { pkx += probs[c] * s.peakSlipRatio[c]; pka += probs[c] * s.peakSlipAngleDeg[c]; }
        pkx = std::max(pkx, 0.02);
        double tanPeak = std::tan(std::max(pka, 1.0) * kPi / 180.0);
        angleValid = angleValid && speed_ > 1.0 && std::fabs(angle) < 0.7;   // slip angle is meaningless at a standstill   // > 40 deg is a unit/learning error, not a real slip angle
        slipAngle = angleValid ? angle : 0.0;
        double nx = slipValid ? slip / pkx : (launchValid ? launchSlip / pkx : 0.0);
        double na = angleValid ? std::tan(std::min(std::fabs(angle), 1.2)) / tanPeak : 0.0;
        slipNorm = slipNorm_.update(std::sqrt(nx * nx + na * na), dt);
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
    Ema loadShort_{2.0}, loadLong_{8.0}, forceVar_{0.35}, pistonVar_{0.35}, pistonRef_{4.0};
    Ema revRate_{0.12}, rollRatio_{3.0}, slipSmooth_{0.25}, slipNorm_{0.05}, launchSmooth_{0.05};
    uint32_t lastSeg_ = 0; bool haveSeg_ = false;
    double transition_ = 0;
    static void smooth(double* cur, const double* target, double tau, double dt) {
        double a = 1.0 - std::exp(-dt / tau), sum = 0;
        for (int i = 0; i < 4; ++i) { cur[i] += a * (target[i] - cur[i]); sum += cur[i]; }
        if (sum > 1e-9) for (int i = 0; i < 4; ++i) cur[i] /= sum;
    }
};

// Per-tyre slip angles from body velocity, yaw rate and steering (rigid-body kinematics).
// Learned automatically: yaw sign, sway-axis side, steering->wheel angle, velocity units.
class CarDynamics {
public:
    double slipAngle[4] = {0, 0, 0, 0};
    bool valid = false;
    double vx = 0, vy = 0, yaw = 0, latAccel = 0, latRatio = 0;
    bool steerLearned() const { return steerGain_.hasValue(); }
    double steerGain() const { return steerGain_.value(); }
    double accelScale(const Settings& s) const { return s.accelToMs2 != 1.0 ? s.accelToMs2 : autoAccel_; }
    double yawScale(const Settings& s) const { return s.yawRateScale != 1.0 ? s.yawRateScale : autoYaw_; }
    bool unitsConfirmed() const { return latRatioAvg_.hasValue() && latRatioAvg_.value() > 0.5 && latRatioAvg_.value() < 2.0; }

    void update(const TelemetryFrame& f, double dt, double speedMs, const Settings& s, bool velocityIsGround = false) {
        valid = false;
        for (double& a : slipAngle) a = 0.0;
        double raw = std::sqrt((double)f.velSurge * f.velSurge + (double)f.velSway * f.velSway);
        double k;
        if (velocityIsGround) k = s.velocityScale;     // the vector IS the ground speed: nothing to calibrate against
        else {                                         // fallback: size the vector to the Speed field
            if (speedMs > 5.0 && raw > 0.5) velScale_.update(clampd(speedMs / raw, 0.1, 10.0), dt);
            k = velScale_.hasValue() ? velScale_.value() : 1.0;
            if (k > 0.85 && k < 1.18) k = 1.0;
        }
        vx = f.velSurge * k; vy = f.velSway * k;
        if (speedMs < 8.0 || std::fabs(vx) < 3.0) return;

        // Unit auto-detection (only when the ini leaves AccelToMs2 / YawRateScale at 1): while cornering,
        // raw a_y / (v * yaw) equals scaleYaw / scaleAccel, so it shows whether accel is in g (x9.81) and/or yaw in deg/s.
        if (s.accelToMs2 == 1.0 && s.yawRateScale == 1.0 && std::fabs(f.accSway) > 0.15 && std::fabs(vx * f.yawRate) > 2.0) {
            rawRatioLog_.update(std::log10(clampd(std::fabs(f.accSway) / std::fabs(vx * f.yawRate), 1e-5, 1e5)), dt);
            if (++rawRatioN_ >= 120) {
                static const double cand[4][3] = {{0.0, 1, 1}, {-0.9917, 9.81, 1}, {-1.7581, 1, 0.0174533}, {-2.7498, 9.81, 0.0174533}};
                int best = -1; double bd = 0.35;
                for (int i = 0; i < 4; ++i) { double d = std::fabs(rawRatioLog_.value() - cand[i][0]); if (d < bd) { bd = d; best = i; } }
                if (best >= 0 && (cand[best][1] != autoAccel_ || cand[best][2] != autoYaw_)) {
                    autoAccel_ = cand[best][1]; autoYaw_ = cand[best][2];
                    latRatioAvg_ = Ema(3.0);       // re-check the sanity test with the corrected scales
                }
            }
        }
        double ay = f.accSway * accelScale(s), r = f.yawRate * yawScale(s);
        yawSign_.update(ay * r, dt);
        double rc = yawSign_.value() >= 0 ? r : -r;
        yaw = rc; latAccel = ay;
        latRatio = std::fabs(vx * rc) > 0.05 ? ay / (vx * rc) : 0.0;

        // Outside wheels carry more load: tells which side +sway points to.
        double dl = f.wheels[0].strutForce + f.wheels[2].strutForce - f.wheels[1].strutForce - f.wheels[3].strutForce;
        sideSign_.update(ay * dl, dt);
        double leftY = sideSign_.value() <= 0 ? 1.0 : -1.0;

        // Self-check of the yaw/acceleration units: while cornering, lateral accel should be about speed x yaw rate
        // (ratio ~1). If it is not (yaw in deg/s, wrong field, ...), the per-tyre angles would be nonsense (we saw
        // 70-90 degrees), so fall back to the car's body slip angle, which needs only the two velocities.
        if (std::fabs(vx * rc) > 0.5 && std::fabs(ay) > 0.3) latRatioAvg_.update(clampd(ay / (vx * rc), 0.0, 50.0), dt);
        bool unitsOk = latRatioAvg_.hasValue() && latRatioAvg_.value() > 0.5 && latRatioAvg_.value() < 2.0;
        if (!unitsOk) {
            double beta = std::atan2(std::fabs(vy), std::fabs(vx));
            for (double& sa : slipAngle) sa = beta;
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
        }
        valid = true;
    }
private:
    Ema yawSign_{2.0}, sideSign_{2.0}, steerGain_{3.0}, velScale_{2.0}, latRatioAvg_{3.0}, rawRatioLog_{4.0};
    double autoAccel_ = 1.0, autoYaw_ = 1.0; int rawRatioN_ = 0;
};

class SurfaceAnalyzer {
public:
    explicit SurfaceAnalyzer(const Settings& s) : s_(s) {}
    WheelAnalyzer wheels[4];
    CarDynamics dynamics;
    // Optional game-memory input, set by the controller before update(): per tyre class (0..3) and wetness.
    bool memValid[4] = {false, false, false, false};
    int memClass[4] = {0, 0, 0, 0}, memWet[4] = {0, 0, 0, 0};
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
    double loadRatio() const { return totalLoad_.hasValue() && totalLoad_.value() > 1e-9 ? lastTotal_ / totalLoad_.value() : -1.0; }   // total strut load / learned normal; airborne below 0.12

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
        double total = 0, raw[4];
        for (int i = 0; i < 4; ++i) {
            wheels[i].update(f.wheels[i], speedMs, drivenMs, dt, coasting);
            total += std::fabs(f.wheels[i].strutForce);   // NGP reports compression as negative force
            classify(wheels[i].roughness, std::fabs(wheels[i].slip) * slipGate((double)f.throttle, (double)f.brake, speedMs), wheels[i].slipValid, s_, raw);
            wheels[i].applyProbabilities(raw, dt, speedMs < 12.0);   // settle fast while pulling away from a standstill
            wheels[i].finalizeProbabilities(memValid[i], memClass[i], dt);
            wheels[i].wetness = memValid[i] ? memWet[i] : 0;
        }
        dynamics.update(f, dt, speedMs, s_, groundFromVelocity);
        for (int i = 0; i < 4; ++i) wheels[i].updateCombinedSlip(dynamics.slipAngle[i], dynamics.valid, s_, dt);

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
        for (int w = 0; w < 4; ++w) {
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
enum class SoundEvent { StoneHit, Impact, Landing, SuspSqueak, SuspBottom };

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
        if (s.enableImpacts && !a.airborne) impacts(a, s, sink, sv);
        if (s.enableLandings && a.justLanded && a.landingAirTime > 0.15) landing(a, sink, sv);
        if (s.enableStones && !a.airborne) stones(f, a, s, dt, sink, sv);
        if (s.enableSuspension && !a.airborne) suspension(a, s, dt, sink, sv);
        rumble(a, s, sink);
        tyres(a, s, sink, sv);
    }

private:
    Rng& rng_;
    double cooldown_[4] = {0, 0, 0, 0};
    double stoneCooldown_ = 0;
    double travel_[4] = {0, 0, 0, 0}, squeakCool_[4] = {0, 0, 0, 0}, bottomCool_[4] = {0, 0, 0, 0}, prevRatio_[4] = {0, 0, 0, 0};

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

    // Stones fly when a tyre's surface moves over ground that gives way, so both inputs come straight from the telemetry:
    //  - how loose the ground is under this tyre: what its own signals say (rough strut force, or slipping with little grip),
    //    taken from the telemetry inference, never from a per-surface table, so snow, mud, gravel and verge all follow the physics;
    //  - how fast the tyre surface moves: the faster of ground speed and the wheel's own speed, so wheelspin throws stones
    //    from a standing start. There is no speed cut-off: the rate and the strength simply grow with surface speed.
    void stones(const TelemetryFrame& f, SurfaceAnalyzer& a, const Settings& s, double dt, IAudioSink& sink, const SetView& sv) {
        double weight[4], total = 0, spray[4];
        for (int w = 0; w < 4; ++w) {
            const WheelAnalyzer& wa = a.wheels[w];
            spray[w] = std::max(a.speedMs, wa.contactSpeedMs()) * 3.6;                   // km/h
            double loose = clampd(1.0 - wa.inferred[Tarmac], 0.0, 1.0);
            double slip = wa.slipValid ? std::min(std::fabs(wa.slip) * 4.0, 1.0)
                          : (wa.launchValid ? std::min(wa.launchSlip * 4.0, 1.0) : 0.0);
            weight[w] = loose * (0.4 + 0.6 * std::max(slip, (double)f.throttle)) * clampd(spray[w] / 90.0, 0.0, 1.5);
            total += weight[w];
        }
        if (total < 0.005) return;
        double r = s.stoneRate * (total / 4.0);
        if (rng_.next() > 1.0 - std::exp(-r * dt)) return;
        if (stoneCooldown_ > 0) return;
        stoneCooldown_ = 0.03;
        double pick = rng_.next() * total; int wheel = 0;
        for (int w = 0; w < 4; ++w) { pick -= weight[w]; if (pick <= 0) { wheel = w; break; } }
        float gain = (float)(0.4 + 0.6 * rng_.next()) * (float)clampd(spray[wheel] / 100.0, 0.15, 1.0), l, rr;
        panGains(kWheelPan[wheel], gain * kWheelScale[wheel], l, rr);
        sink.trigger(SoundEvent::StoneHit, sv.dominant(wheel), l, rr, rate(0.12));
    }

    // Suspension sounds, both from the damper and strut telemetry of each tyre:
    //  - A creak comes from slow travel under load (body roll, weight shifting, easing over a bump): the stroke the damper
    //    makes while moving gently, weighted by how loaded the tyre is. Fast strokes are impacts and make no creak.
    //  - Bottoming out is a sharp rise of the strut load far above the tyre's own normal load (the bump stop taking the hit).
    // Nothing here depends on the surface: it follows how the car moves.
    void suspension(SurfaceAnalyzer& a, const Settings& s, double dt, IAudioSink& sink, const SetView&) {
        for (int w = 0; w < 4; ++w) {
            const WheelAnalyzer& wa = a.wheels[w];
            if (squeakCool_[w] > 0) squeakCool_[w] -= dt;
            if (bottomCool_[w] > 0) bottomCool_[w] -= dt;

            const double v = std::fabs(wa.pistonRaw);
            const double slowLimit = 0.25;                       // m/s: faster than this is a knock, not a creak
            if (s.squeakSensitivity > 0 && a.speedMs > 1.0 && v > 0.01 && v < slowLimit)
                travel_[w] += v * dt * wa.loadFraction() * s.squeakSensitivity;
            else
                travel_[w] = std::max(0.0, travel_[w] - 0.02 * dt);          // gentle decay so odd moments don't pile up
            const double need = 0.012;                           // metres of loaded slow travel per creak
            if (travel_[w] > need && squeakCool_[w] <= 0) {
                float intensity = (float)clampd(0.3 + 0.7 * wa.loadFraction() * (0.5 + rng_.next()), 0.2, 1.0), l, r;
                panGains(kWheelPan[w], intensity * kWheelScale[w], l, r);
                sink.trigger(SoundEvent::SuspSqueak, w, l, r, rate(0.12));
                travel_[w] = 0; squeakCool_[w] = 0.5 + 0.5 * rng_.next();
            }

            const double ratio = wa.loadRef > 1e-6 ? wa.load / wa.loadRef : 0.0;
            if (ratio > s.bottomRatio && prevRatio_[w] <= s.bottomRatio && bottomCool_[w] <= 0 && a.speedMs > 2.0) {
                float intensity = (float)clampd(0.35 + (ratio - s.bottomRatio) / s.bottomRatio, 0.35, 1.0), l, r;
                panGains(kWheelPan[w], intensity * kWheelScale[w], l, r);
                sink.trigger(SoundEvent::SuspBottom, w, l, r, rate(0.06));
                bottomCool_[w] = 0.3;
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

    // Squeal (tonal, near the grip limit) and slide (broadband, beyond it), per tyre.
    void tyres(SurfaceAnalyzer& a, const Settings& s, IAudioSink& sink, const SetView& sv) {
        for (int w = 0; w < 4; ++w) {
            auto& wa = a.wheels[w];
            double n = wa.slipNorm, load = std::sqrt(std::max(0.35, wa.loadFraction()));
            bool on = a.speedMs > 1.0 || wa.launchSlip > 0.05;   // standing wheelspin counts
            double air = a.airborne ? 0.0 : 1.0;   // in the air: no tyre noise
            double squeal = 0, slide = 0;
            if (on && s.enableSqueal) squeal = smoothstep((n - 0.45) / 0.5) * (1.0 - 0.6 * smoothstep((n - 1.4) / 0.6)) * load * air;
            if (on && s.enableSlide) slide = smoothstep((n - 1.0) / 0.8) * load * air;
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
    const MemoryHealth& memoryHealth() const { return health_; }
    bool telemetryOk() const { return telemetry_.ok(); }
    Label discoveryLabel() const { return label_; }
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
