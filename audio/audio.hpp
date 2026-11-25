// Dynamic audio: WAV loading, sound bank, mixer and the sink that connects the controller to it.
// Portable (no Windows APIs); the waveOut device lives in win_platform.hpp.
#pragma once
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>
#include "core.hpp"

namespace ffx {

constexpr int kSampleRate = 44100;

// Loads a WAV (PCM 8/16/24/32-bit or float32, any channel count, any rate) as mono float at `rate`.
inline bool loadWav(const std::string& path, int rate, std::vector<float>& out, std::string& err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { err = "cannot open"; return false; }
    std::vector<uint8_t> d((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    auto u16 = [&](size_t o) { return (uint32_t)(d[o] | (d[o + 1] << 8)); };
    auto u32 = [&](size_t o) { return (uint32_t)(d[o] | (d[o + 1] << 8) | (d[o + 2] << 16) | ((uint32_t)d[o + 3] << 24)); };
    if (d.size() < 44 || std::memcmp(d.data(), "RIFF", 4) || std::memcmp(d.data() + 8, "WAVE", 4)) { err = "not a RIFF/WAVE file"; return false; }

    uint32_t tag = 0, channels = 0, srate = 0, bits = 0;
    size_t dataOff = 0, dataLen = 0;
    size_t p = 12;
    while (p + 8 <= d.size()) {
        uint32_t sz = u32(p + 4);
        size_t body = p + 8;
        if (!std::memcmp(d.data() + p, "fmt ", 4) && body + 16 <= d.size()) {
            tag = u16(body); channels = u16(body + 2); srate = u32(body + 4); bits = u16(body + 14);
            if (tag == 0xFFFE && sz >= 26 && body + 26 <= d.size()) tag = u16(body + 24);
        } else if (!std::memcmp(d.data() + p, "data", 4)) {
            dataOff = body; dataLen = std::min<size_t>(sz, d.size() - body);
        }
        p = body + sz + (sz & 1);
    }
    if (!channels || !srate || !dataOff) { err = "missing fmt or data chunk"; return false; }
    bool isFloat = tag == 3;
    if (!(tag == 1 || tag == 3) || (isFloat && bits != 32) || (!isFloat && bits != 8 && bits != 16 && bits != 24 && bits != 32)) {
        err = "unsupported format (use PCM 8/16/24/32-bit or float32)"; return false;
    }
    size_t bps = bits / 8, frames = dataLen / (bps * channels);
    std::vector<float> mono(frames);
    for (size_t i = 0; i < frames; ++i) {
        double acc = 0;
        for (uint32_t c = 0; c < channels; ++c) {
            const uint8_t* s = d.data() + dataOff + (i * channels + c) * bps;
            double v;
            if (isFloat) { float x; std::memcpy(&x, s, 4); v = x; }
            else if (bits == 8) v = (s[0] - 128) / 128.0;
            else if (bits == 16) v = (int16_t)(s[0] | (s[1] << 8)) / 32768.0;
            else if (bits == 24) v = ((int32_t)((s[0] << 8) | (s[1] << 16) | ((uint32_t)s[2] << 24)) >> 8) / 8388608.0;
            else v = (int32_t)(s[0] | (s[1] << 8) | (s[2] << 16) | ((uint32_t)s[3] << 24)) / 2147483648.0;
            acc += v;
        }
        mono[i] = (float)(acc / channels);
    }
    if ((int)srate == rate || frames < 2) { out = std::move(mono); return true; }
    double step = (double)srate / rate;
    size_t n = (size_t)(frames / step);
    out.resize(n);
    for (size_t i = 0; i < n; ++i) {
        double pos = i * step; size_t i0 = (size_t)pos, i1 = std::min(i0 + 1, frames - 1);
        out[i] = (float)(mono[i0] + (mono[i1] - mono[i0]) * (pos - i0));
    }
    return true;
}

// Every .wav under a folder. "gravel_loop_3.wav" -> key "gravel_loop" (the _N suffix is optional).
// Makes a loop wrap without a click or dip: the last `fade` frames are blended into the start (equal power) and dropped.
inline void makeSeamless(std::vector<float>& d, double seconds = 0.12) {
    size_t k = (size_t)(seconds * kSampleRate);
    if (d.size() < 4 * k || k < 16) return;
    size_t n = d.size() - k;
    for (size_t i = 0; i < k; ++i) {
        double t = (double)i / (double)k;
        d[i] = (float)(d[i] * std::sqrt(t) + d[n + i] * std::sqrt(1.0 - t));
    }
    d.resize(n);
}
inline bool isLoopKey(const std::string& key) {
    auto ends = [&](const char* suf) { size_t l = std::strlen(suf); return key.size() >= l && key.compare(key.size() - l, l, suf) == 0; };
    return ends("_loop") || ends("_slide") || ends("_squeal") || ends("_rumble");
}

// Source files differ wildly in level (RBR's tarmac skid is several times louder than its gravel skid). Bring every file
// to a common loudness at load so the mix is decided by the telemetry, not by how each file happened to be mastered.
inline void normalizeLoudness(std::vector<float>& d, double targetRms, double maxPeak = 0.95) {
    if (d.empty()) return;
    double sum = 0, pk = 0;
    for (float x : d) { sum += (double)x * x; pk = std::max(pk, (double)std::fabs(x)); }
    double rms = std::sqrt(sum / (double)d.size());
    if (rms < 1e-6 || pk < 1e-6) return;
    double g = std::min(targetRms / rms, maxPeak / pk);
    for (float& x : d) x = (float)(x * g);
}
inline double targetRmsFor(const std::string& key) {
    auto ends = [&](const char* suf) { size_t l = std::strlen(suf); return key.size() >= l && key.compare(key.size() - l, l, suf) == 0; };
    return (ends("_squeal") || ends("_slide")) ? 0.24 : 0.30;
}

// Character of a sound by its name: wet, muddy, watery, snowy or icy sounds are deep and dull; dry stones, gravel,
// rubble, dirt and cobbles are bright and crisp; tarmac is neutral. Returns -1 .. +1.
inline float toneOfName(std::string n) {
    for (char& c : n) c = (char)std::tolower((unsigned char)c);
    auto has = [&](const char* k) { return n.find(k) != std::string::npos; };
    if (has("wet") || has("mud") || has("water") || has("puddle") || has("slush") || has("snow") || has("ice")) return -0.8f;
    if (has("gravel") || has("stone") || has("rubble") || has("rock") || has("pebble") || has("dirt") || has("sand") || has("cobble") || has("rough")) return 0.7f;
    if (has("grass") || has("leaf") || has("forest")) return 0.15f;
    return 0.0f;
}

class SoundBank {
public:
    using Variants = std::vector<std::vector<float>>;
    std::vector<std::string> errors;
    int fileCount = 0;
    bool normalize = true;   // NormalizeSounds in the ini

    void load(const std::string& folder, int rate = kSampleRate) {
        namespace fs = std::filesystem;
        std::error_code ec;
        if (folder.empty() || !fs::is_directory(folder, ec)) { errors.push_back("Sounds folder not found: " + folder); return; }
        for (auto it = fs::recursive_directory_iterator(folder, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
            if (!it->is_regular_file()) continue;
            std::string ext = it->path().extension().string();
            for (char& c : ext) c = (char)std::tolower((unsigned char)c);
            if (ext != ".wav") continue;
            std::vector<float> data; std::string err;
            if (!loadWav(it->path().string(), rate, data, err)) { errors.push_back(it->path().filename().string() + ": " + err); continue; }
            std::string key = keyFor(it->path().stem().string());
            if (isLoopKey(key)) makeSeamless(data);
            if (normalize) { if (isLoopKey(key)) normalizeLoudness(data, targetRmsFor(key)); else normalizeLoudness(data, 1e9, 0.6); }   // loops to a common RMS, one-shots to a common peak
            map_[key].push_back(std::move(data));
            ++fileCount;
        }
    }
    const Variants* get(const std::string& key) const {
        auto it = map_.find(lower(key));
        return it == map_.end() ? nullptr : &it->second;
    }
    // Adds a sound made in code (see water_sounds.hpp) under the key a file of that name would have.
    void add(const std::string& key, std::vector<float> data) { map_[lower(key)].push_back(std::move(data)); }
    static std::string keyFor(const std::string& stem) {
        std::string s = lower(stem);
        size_t u = s.find_last_of('_');
        if (u != std::string::npos && u + 1 < s.size() && s.find_first_not_of("0123456789", u + 1) == std::string::npos) s.resize(u);
        return s;
    }
private:
    static std::string lower(std::string s) { for (char& c : s) c = (char)std::tolower((unsigned char)c); return s; }
    std::map<std::string, Variants> map_;
};

// What the car's interior does to the sound: tyre noise arrives muffled and bassy, through floor, seats and bodywork,
// with short reflections inside the cabin. Low-pass (4th order) + low shelf + a tiny damped comb/all-pass "room".
struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    float process(float x) { float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
    void set(double B0, double B1, double B2, double A0, double A1, double A2) {
        b0 = (float)(B0 / A0); b1 = (float)(B1 / A0); b2 = (float)(B2 / A0); a1 = (float)(A1 / A0); a2 = (float)(A2 / A0);
    }
    void lowpass(double fc, double q) {
        double w = 2 * 3.14159265358979 * fc / kSampleRate, c = std::cos(w), al = std::sin(w) / (2 * q);
        set((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
    }
    void lowShelf(double fc, double gainDb) {
        double A = std::pow(10.0, gainDb / 40), w = 2 * 3.14159265358979 * fc / kSampleRate, c = std::cos(w);
        double al = std::sin(w) / 2 * std::sqrt(2.0), sa = 2 * std::sqrt(A) * al;
        set(A * ((A + 1) - (A - 1) * c + sa), 2 * A * ((A - 1) - (A + 1) * c), A * ((A + 1) - (A - 1) * c - sa),
            (A + 1) + (A - 1) * c + sa, -2 * ((A - 1) + (A + 1) * c), (A + 1) + (A - 1) * c - sa);
    }
};
struct Comb {
    std::vector<float> buf; size_t i = 0; float fb = 0.34f, damp = 0.35f, lp = 0;
    void init(double ms, float feedback) { buf.assign((size_t)(ms * 0.001 * kSampleRate) + 1, 0.0f); i = 0; fb = feedback; lp = 0; }
    float process(float x) { float y = buf[i]; lp = y * (1 - damp) + lp * damp; buf[i] = x + lp * fb; if (++i >= buf.size()) i = 0; return y; }
};
struct Allpass {
    std::vector<float> buf; size_t i = 0;
    void init(double ms) { buf.assign((size_t)(ms * 0.001 * kSampleRate) + 1, 0.0f); i = 0; }
    float process(float x) { float b = buf[i]; buf[i] = x + b * 0.5f; if (++i >= buf.size()) i = 0; return b - x; }
};
class CabinFilter {
public:
    bool enabled = false;
    void configure(bool on, double lowpassHz, double bassDb, double reverb) {
        enabled = on; wet_ = (float)std::min(std::max(reverb, 0.0), 0.8);
        for (int c = 0; c < 2; ++c) {
            lp1_[c].lowpass(std::max(lowpassHz, 500.0), 0.5412); lp2_[c].lowpass(std::max(lowpassHz, 500.0), 1.3066);
            shelf_[c].lowShelf(220.0, bassDb);
            const double d[2][3] = {{9.7, 13.1, 16.9}, {10.7, 14.3, 18.1}};   // slightly different per ear: a little width
            for (int k = 0; k < 3; ++k) comb_[c][k].init(d[c][k], 0.34f);
            ap_[c].init(3.1 + c * 0.7);
        }
    }
    void process(float* out, int frames) {
        if (!enabled) return;
        for (int i = 0; i < frames; ++i) {
            float dry[2];
            for (int c = 0; c < 2; ++c) dry[c] = lp2_[c].process(lp1_[c].process(shelf_[c].process(out[2 * i + c])));
            float send = 0.5f * (dry[0] + dry[1]);
            for (int c = 0; c < 2; ++c) {
                float r = 0; for (int k = 0; k < 3; ++k) r += comb_[c][k].process(send);
                r = ap_[c].process(r * 0.33f);
                out[2 * i + c] = dry[c] + wet_ * r;
            }
        }
    }
private:
    float wet_ = 0.2f;
    Biquad lp1_[2], lp2_[2], shelf_[2];
    Comb comb_[2][3];
    Allpass ap_[2];
};

struct LoopVoice {
    const SoundBank::Variants* variants = nullptr;
    int variant = 0;
    double pos = 0;
    float curL = 0, curR = 0, rateCur = 1.0f, toneCur = 0.0f, lpState = 0.0f;
    std::atomic<float> toneSet{0.0f}, toneDyn{0.0f};   // the set's own character + the live telemetry tone
    std::atomic<float> tgtL{0}, tgtR{0}, rate{1.f};
};

// Real-time stereo mixer. Targets are written from the telemetry thread; the audio thread ramps
// toward them inside each buffer to avoid clicks. One-shots overlap freely.
class Mixer {
public:
    std::atomic<float> master{0.8f};   // drive into the soft limiter (changes the mix character when pushed)
    std::atomic<float> total{1.0f};    // final output gain after the limiter; the result is hard-clipped to +-1
    CabinFilter cabin;   // configure() before the audio thread starts

    LoopVoice* addVoice(const SoundBank::Variants* v) {
        voices_.emplace_back(new LoopVoice());
        voices_.back()->variants = v;
        return voices_.back().get();
    }
    void playOneShot(const std::vector<float>* data, float gl, float gr, float rate) {
        if (!data || data->size() < 2) return;
        std::lock_guard<std::mutex> lock(mu_);
        if (shots_.size() >= 32) shots_.erase(shots_.begin());
        shots_.push_back({data, 0.0, gl, gr, rate});
    }
    size_t activeShots() { std::lock_guard<std::mutex> lock(mu_); return shots_.size(); }

    // Diagnostics for the debug log: loudest output sample since the last call, and the summed loop target gains.
    float takePeak() { return peak_.exchange(0.0f); }
    float targetSum() const {
        float t = 0;
        for (const auto& v : voices_) if (v->variants && !v->variants->empty()) t += v->tgtL.load() + v->tgtR.load();
        return t;
    }

    // Interleaved stereo float, `frames` frames.
    void read(float* out, int frames) {
        std::fill(out, out + 2 * (size_t)frames, 0.0f);
        for (auto& v : voices_) mixLoop(*v, out, frames);
        mixShots(out, frames);
        cabin.process(out, frames);
        float m = master.load(), t = total.load();
        float pk = 0.0f;
        for (int i = 0; i < 2 * frames; ++i) { out[i] = std::max(-1.0f, std::min(1.0f, std::tanh(out[i] * m) * t)); pk = std::max(pk, std::fabs(out[i])); }
        float old = peak_.load();
        while (pk > old && !peak_.compare_exchange_weak(old, pk)) {}
    }

private:
    struct Shot { const std::vector<float>* data; double pos; float gl, gr, rate; };
    std::vector<std::unique_ptr<LoopVoice>> voices_;
    std::vector<Shot> shots_;
    std::mutex mu_;
    std::atomic<float> peak_{0.0f};
    std::mt19937 rng_{777};

    // Gain and pitch are smoothed per sample (fast attack, slower release), so the noisy telemetry never turns into
    // flutter or a robotic stutter; a real tyre's sound swells and fades smoothly.
    void mixLoop(LoopVoice& v, float* out, int frames) {
        if (!v.variants || v.variants->empty()) return;
        float tl = v.tgtL.load(), tr = v.tgtR.load();
        bool silentNow = v.curL < 1e-4f && v.curR < 1e-4f;
        if (silentNow && tl < 1e-4f && tr < 1e-4f) { v.curL = v.curR = 0; return; }
        if (silentNow) {   // fading in from silence: new random variant and start point
            v.variant = (int)(rng_() % v.variants->size());
            v.pos = (rng_() / (double)rng_.max()) * (*v.variants)[v.variant].size();
            v.rateCur = v.rate.load();
        }
        const std::vector<float>& data = (*v.variants)[v.variant];
        size_t n = data.size();
        if (n < 2) return;
        static const float aUp = 1.0f - std::exp(-1.0f / (0.020f * kSampleRate));     // 20 ms attack
        static const float aDown = 1.0f - std::exp(-1.0f / (0.110f * kSampleRate));   // 110 ms release
        static const float aRate = 1.0f - std::exp(-1.0f / (0.200f * kSampleRate));   // 200 ms pitch glide
        static const float aTone = 1.0f - std::exp(-1.0f / (0.300f * kSampleRate));   // 300 ms tone glide
        static const float aXover = 1.0f - std::exp(-2.0f * 3.14159265f * 1100.0f / kSampleRate);   // 1.1 kHz crossover
        float toneT = std::min(1.0f, std::max(-1.0f, v.toneSet.load() + v.toneDyn.load()));
        float tc = v.toneCur, lps = v.lpState;
        double pos = v.pos; float rateT = v.rate.load();
        float cl = v.curL, cr = v.curR, rc = v.rateCur;
        for (int i = 0; i < frames; ++i) {
            size_t i0 = (size_t)pos, i1 = i0 + 1 >= n ? 0 : i0 + 1;
            float s = data[i0] + (data[i1] - data[i0]) * (float)(pos - i0);
            tc += (toneT - tc) * aTone;
            lps += (s - lps) * aXover;                       // low band; the rest is the high band
            float gHigh = tc >= 0 ? 1.0f + 0.9f * tc : 1.0f + 0.85f * tc, gLow = tc >= 0 ? 1.0f - 0.12f * tc : 1.0f - 0.45f * tc;
            s = lps * gLow + (s - lps) * gHigh;
            cl += (tl - cl) * (tl > cl ? aUp : aDown);
            cr += (tr - cr) * (tr > cr ? aUp : aDown);
            rc += (rateT - rc) * aRate;
            out[2 * i] += s * cl; out[2 * i + 1] += s * cr;
            pos += rc; if (pos >= n) pos -= n;
        }
        if (tl < 1e-5f && cl < 1e-4f) cl = 0;
        if (tr < 1e-5f && cr < 1e-4f) cr = 0;
        v.pos = pos; v.curL = cl; v.curR = cr; v.rateCur = rc; v.toneCur = tc; v.lpState = lps;
    }

    void mixShots(float* out, int frames) {
        std::lock_guard<std::mutex> lock(mu_);
        for (size_t k = shots_.size(); k-- > 0;) {
            Shot& sh = shots_[k];
            size_t n = sh.data->size(); double pos = sh.pos;
            for (int i = 0; i < frames && pos < (double)n - 1; ++i) {
                size_t i0 = (size_t)pos;
                float s = (*sh.data)[i0] + ((*sh.data)[i0 + 1] - (*sh.data)[i0]) * (float)(pos - i0);
                out[2 * i] += s * sh.gl; out[2 * i + 1] += s * sh.gr;
                pos += sh.rate;
            }
            sh.pos = pos;
            if (pos >= (double)n - 1) shots_.erase(shots_.begin() + k);
        }
    }
};

// Decides which sound set plays for each game material, from the files that actually exist in the sound bank.
//
// For a material such as "Gravel_Coarse_Deeper_Wet" the sample names tried, most specific first, are
//   gravel_coarse_deeper_wet, gravel_coarse_deeper, gravel_coarse, gravel_wet, gravel, <class default, e.g. "gravel">
// (each followed by _loop / _slide / _squeal / _hit; the first that exists wins, separately per kind). Wet/damp
// materials try "<family>_wet" / "<family>_damp" before the plain family. US2_/BR2_ gravel variants try their own
// names first, then the ordinary gravel ones. Snow on ice also tries "ice", snow tries "snow". Materials that resolve
// to the same files share one set, so a pack with only tarmac / gravel / grass / snow files still works, and every
// extra file you add makes that surface more specific. audio/surfaces.md lists the search order for every material.
class SoundSetMap : public ISoundSets {
public:
    struct Info { std::string loop, slide, squeal, hit, label; float vol = 1.f; bool wetNative = false; };   // wetNative: the loop is a recording of damp or wet ground

    SoundSetMap(const SoundBank& bank, const MaterialTable& table, const Settings& s) : bank_(bank) {
        for (int& m : mat_) m = -1;
        for (int c = 0; c < 4; ++c) cls_[c] = add(resolve({s.surfaceNames[c]}), s.classVolume[c], -1);
        for (int id = 0; id < 256; ++id) {
            int c = table.cls(id);
            if (c >= 0) mat_[id] = add(resolve(chainFor(table.name(id), c, s)), s.classVolume[c], cls_[c]);
        }
    }

    int count() const override { return (int)sets_.size(); }
    int forMaterial(int id) const override { return id >= 0 && id < 256 ? mat_[id] : -1; }
    int forClass(int cls) const override { return cls_[cls >= 0 && cls < 4 ? cls : 0]; }
    float volume(int set) const override { return set >= 0 && set < (int)sets_.size() ? sets_[set].vol : 1.f; }
    bool wetNative(int set) const override { return set >= 0 && set < (int)sets_.size() && sets_[set].wetNative; }
    const Info& info(int set) const { return sets_[set]; }

    // Names searched for a material, most specific first (pure function; also used to generate audio/surfaces.md).
    static std::vector<std::string> chainFor(const std::string& materialName, int cls, const Settings& s) {
        std::vector<std::string> t;
        for (const std::string& tok : detail::split(materialName, '_')) {
            std::string l = tok; for (char& ch : l) ch = (char)std::tolower((unsigned char)ch);
            if (!l.empty() && l != "clone") t.push_back(l);
        }
        std::vector<std::string> out;
        auto push = [&](const std::string& n) { if (!n.empty() && std::find(out.begin(), out.end(), n) == out.end()) out.push_back(n); };
        auto addChain = [&](const std::vector<std::string>& v) {
            if (v.empty()) return;
            std::string wet = (v.back() == "damp" || v.back() == "wet") ? v.back() : "";
            for (size_t k = v.size(); k >= 1; --k) {
                if (k == 1 && !wet.empty() && v[0] != "us2" && v[0] != "br2") push(v[0] + "_" + wet);
                std::string name = v[0]; for (size_t i = 1; i < k; ++i) name += "_" + v[i];
                push(name);
            }
        };
        addChain(t);
        std::vector<std::string> stripped = t;
        if (t.size() > 1 && (t[0] == "us2" || t[0] == "br2")) { stripped.erase(stripped.begin()); addChain(stripped); }
        if (!stripped.empty()) {
            if (stripped[0] == "snowonice") push("ice");
            if (stripped[0] == "snowongravel" || stripped[0] == "snowonice") push("snow");
        }
        push(s.surfaceNames[cls >= 0 && cls < 4 ? cls : 0]);
        return out;
    }

private:
    const SoundBank& bank_;
    std::vector<Info> sets_;
    int cls_[4] = {0, 0, 0, 0};
    int mat_[256];

    Info resolve(const std::vector<std::string>& chain) const {
        Info in;
        auto first = [&](const char* kind) {
            for (const std::string& b : chain) if (bank_.get(b + "_" + kind)) return b + "_" + kind;
            return std::string();
        };
        in.loop = first("loop"); in.slide = first("slide"); in.squeal = first("squeal"); in.hit = first("hit");
        for (const std::string* k : {&in.loop, &in.squeal, &in.slide, &in.hit}) if (!k->empty()) { in.label = *k; break; }
        if (in.label.empty()) in.label = chain.empty() ? "(none)" : chain.back() + " (no files)";
        for (const std::string& tok : detail::split(in.loop, '_')) if (tok == "wet" || tok == "damp") in.wetNative = true;   // e.g. gravel_wet_loop
        return in;
    }
    int add(Info in, float vol, int fallback) {
        for (size_t i = 0; i < sets_.size(); ++i)
            if (sets_[i].loop == in.loop && sets_[i].slide == in.slide && sets_[i].squeal == in.squeal && sets_[i].hit == in.hit) return (int)i;
        if ((int)sets_.size() >= kMaxSets) return fallback >= 0 ? fallback : 0;
        in.vol = vol; sets_.push_back(in);
        return (int)sets_.size() - 1;
    }
};

// Maps controller output onto mixer voices using the sound bank and the sound sets.
class MixerSink : public IAudioSink {
public:
    // map = null: build one from the bank with the default material table.
    MixerSink(Mixer& mixer, const SoundBank& bank, const Settings& s, const SoundSetMap* map = nullptr) : mixer_(mixer), bank_(bank), eventLevel_((float)s.eventLevel), stoneLevel_((float)s.stoneLevel), squeakLevel_((float)s.squeakLevel), bottomLevel_((float)s.bottomLevel), rumbleLevel_((float)s.rumbleLevel), crashLevel_((float)s.crashLevel) {
        if (!map) { ownTable_.reset(new MaterialTable()); own_.reset(new SoundSetMap(bank, *ownTable_, s)); map = own_.get(); }
        map_ = map;
        int n = map_->count();
        for (int w = 0; w < 4; ++w)
            for (int i = 0; i < n; ++i) {
                const SoundSetMap::Info& in = map_->info(i);
                wheel_[w].push_back(mixer.addVoice(in.loop.empty() ? nullptr : bank.get(in.loop)));
                slide_[w].push_back(mixer.addVoice(in.slide.empty() ? nullptr : bank.get(in.slide)));
                squeal_[w].push_back(mixer.addVoice(in.squeal.empty() ? nullptr : bank.get(in.squeal)));
                float t = toneOfName(in.label.empty() ? in.loop : in.label);
                wheel_[w].back()->toneSet = t; slide_[w].back()->toneSet = t; squeal_[w].back()->toneSet = 0.3f * t;
            }
        rumble_ = mixer.addVoice(bank.get("debris_rumble"));
        for (int w = 0; w < 4; ++w) hiss_[w] = mixer.addVoice(bank.get("wet_hiss_loop"));      // the spray of wet ground: a recording of it if there is one, else made in code
    }
    const ISoundSets* soundSets() const override { return map_; }

    void setWheelLoop(int w, int i, float gl, float gr, float rate) override { set(wheel_[w], i, gl, gr, rate); }
    void setWheelTone(int w, float tone) override {
        for (LoopVoice* v : wheel_[w]) v->toneDyn = tone;
        for (LoopVoice* v : slide_[w]) v->toneDyn = tone;
        for (LoopVoice* v : squeal_[w]) v->toneDyn = 0.3f * tone;
    }
    void setWetHiss(int w, float gl, float gr, float rate) override { if (w >= 0 && w < 4 && hiss_[w]) { hiss_[w]->tgtL = gl; hiss_[w]->tgtR = gr; hiss_[w]->rate = rate; } }
    void setSlideLoop(int w, int i, float gl, float gr, float rate) override { set(slide_[w], i, gl, gr, rate); }
    void setSquealLoop(int w, int i, float gl, float gr, float rate) override { set(squeal_[w], i, gl, gr, rate); }
    void setRumble(float gl, float gr) override { rumble_->tgtL = gl * rumbleLevel_; rumble_->tgtR = gr * rumbleLevel_; rumble_->rate = 1.f; }
    void trigger(SoundEvent kind, int i, float gl, float gr, float rate) override {
        if (i < 0 || i >= map_->count()) return;
        const std::string& hit = map_->info(i).hit;
        const SoundBank::Variants* v = nullptr;
        switch (kind) {
            case SoundEvent::StoneHit: v = hit.empty() ? nullptr : bank_.get(hit); if (!v) v = bank_.get("debris_stone"); break;
            case SoundEvent::Impact: v = bank_.get("debris_impact"); if (!v && !hit.empty()) v = bank_.get(hit); break;
            case SoundEvent::Landing: v = bank_.get("debris_landing"); if (!v) v = bank_.get("debris_impact"); break;
            case SoundEvent::SuspSqueak: v = bank_.get("susp_squeak"); break;
            case SoundEvent::SuspBottom: v = bank_.get("susp_bottom"); break;
            case SoundEvent::Splash: v = bank_.get("water_splash"); if (!v) v = bank_.get("debris_splash"); break;
            case SoundEvent::WetStone: v = bank_.get("water_hit"); if (!v) v = bank_.get("water_surface_hit"); break;      // a stone landing in the water on wet ground
            // Crashes: each takes its own sound (the RBR Impact folders, see tools/Copy-RbrSounds.ps1), and the thumps fall back to the bump and landing sounds
            case SoundEvent::CrashLight: v = bank_.get("crash_light"); if (!v) v = bank_.get("debris_impact"); break;
            case SoundEvent::CrashMedium: v = bank_.get("crash_medium"); if (!v) v = bank_.get("debris_impact"); break;
            case SoundEvent::CrashHeavy: v = bank_.get("crash_heavy"); if (!v) v = bank_.get("debris_landing"); if (!v) v = bank_.get("debris_impact"); break;
            case SoundEvent::CrashMetal: v = bank_.get("crash_metal"); break;
            case SoundEvent::CrashCrumple: v = bank_.get("crash_crumple"); break;
            case SoundEvent::CrashBranch: v = bank_.get("crash_branch"); break;
            case SoundEvent::CrashGlass: v = bank_.get("crash_glass"); break;
        }
        if (!v || v->empty()) return;
        float g;
        switch (kind) {
            case SoundEvent::StoneHit: case SoundEvent::WetStone: g = stoneLevel_ * liveStone_.load(); break;
            case SoundEvent::SuspSqueak: g = squeakLevel_ * liveSusp_.load(); break;
            case SoundEvent::SuspBottom: g = bottomLevel_ * liveSusp_.load(); break;
            case SoundEvent::CrashLight: case SoundEvent::CrashMedium: case SoundEvent::CrashHeavy: case SoundEvent::CrashMetal:
            case SoundEvent::CrashCrumple: case SoundEvent::CrashBranch: case SoundEvent::CrashGlass: g = crashLevel_ * liveImpact_.load(); break;
            default: g = eventLevel_ * liveImpact_.load(); break;
        }
        mixer_.playOneShot(&(*v)[rng_() % v->size()], gl * g, gr * g, rate);
    }
    // Live multipliers from the in-game menu, applied on top of StoneLevel / EventLevel. Safe to call from any thread.
    void setLiveLevels(float stones, float impacts, float suspension = 1.0f) { liveStone_ = stones; liveImpact_ = impacts; liveSusp_ = suspension; }
    void silence() override {
        for (int w = 0; w < 4; ++w)
            for (size_t i = 0; i < wheel_[w].size(); ++i) { set(wheel_[w], (int)i, 0, 0, 1); set(slide_[w], (int)i, 0, 0, 1); set(squeal_[w], (int)i, 0, 0, 1); }
        rumble_->tgtL = rumble_->tgtR = 0.f;
        for (int w = 0; w < 4; ++w) if (hiss_[w]) hiss_[w]->tgtL = hiss_[w]->tgtR = 0.f;
    }
private:
    static void set(std::vector<LoopVoice*>& v, int i, float gl, float gr, float rate) {
        if (i < 0 || i >= (int)v.size()) return;
        v[i]->tgtL = gl; v[i]->tgtR = gr; v[i]->rate = rate;
    }
    Mixer& mixer_; const SoundBank& bank_;
    float eventLevel_ = 1.0f, stoneLevel_ = 2.0f, squeakLevel_ = 1.0f, bottomLevel_ = 1.0f, rumbleLevel_ = 1.0f, crashLevel_ = 1.0f;
    std::atomic<float> liveStone_{1.0f}, liveImpact_{1.0f}, liveSusp_{1.0f};
    std::unique_ptr<MaterialTable> ownTable_; std::unique_ptr<SoundSetMap> own_;
    const SoundSetMap* map_ = nullptr;
    std::vector<LoopVoice*> wheel_[4], slide_[4], squeal_[4];
    LoopVoice* rumble_ = nullptr;
    LoopVoice* hiss_[4] = {nullptr, nullptr, nullptr, nullptr};
    std::mt19937 rng_{4242};
};

}  // namespace ffx
