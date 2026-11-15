// Dynamic surface audio for ForceFX.
//
// ForceFX already receives NGP's telemetry packets and already knows which material is under each tyre, so this engine
// has no UDP listener and no memory scanner of its own. The plugin hands it the packets and a way to read the four
// material ids; everything else (per-tyre analysis, sound sets, mixer, waveOut) comes from the surface audio code.
//
// It is off until the menu switch turns it on. Switching on loads the sounds on a background thread, so the force
// feedback loop never waits for the disk. Switching off tears everything down again.
#pragma once
#include <atomic>
#include <chrono>
#include <exception>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "core.hpp"
#include "audio.hpp"
#include "waveout.hpp"

namespace rsa {

// Answers the audio code's "what is under each tyre" question from ForceFX's own surface reader.
// Wheel order is LF, RF, LB, RB on both sides, so the ids pass straight through.
class ForceFXSurfaceMemory : public ISurfaceMemory {
public:
    std::function<bool(int ids[4])> read;      // false = no usable ids right now
    std::function<bool()> available;           // true once the reader has found the tyres
    std::function<std::string()> status;

    bool readMaterials(int ids[4]) override { return read && read(ids); }
    bool configured() const override { return available && available(); }
    std::string statusText() const override { return status ? status() : std::string("no reader"); }
};

class DynamicAudio {
public:
    enum State { Off = 0, Loading, Running, Failed };

    struct Hooks {
        std::function<bool(int ids[4])> readIds;
        std::function<bool()> idsAvailable;
        std::function<std::string()> idsStatus;
        std::function<void(const std::string&)> log;
    };

    // pluginDir: the folder holding the DLL. Settings: ForceFXAudio.ini beside it. Sounds: ForceFXAudio\sounds.
    void configure(std::string pluginDir, Hooks hooks) {
        if (!pluginDir.empty() && pluginDir.back() != '\\' && pluginDir.back() != '/') pluginDir += '\\';
        dir_ = pluginDir;
        hooks_ = std::move(hooks);
        mem_.read = hooks_.readIds;
        mem_.available = hooks_.idsAvailable;
        mem_.status = hooks_.idsStatus;
    }

    // Safe to call from any thread, as often as you like. Turning on returns at once; the sounds load in the background.
    void setActive(bool on) {
        if (on == wantOn_.load()) return;
        wantOn_ = on;
        if (on) {
            if (loading_.load()) return;                         // a load is already under way and will see wantOn_
            if (loader_.joinable()) loader_.join();
            state_ = Loading;
            loading_ = true;
            loader_ = std::thread(&DynamicAudio::load, this);
        } else {
            teardown();
            if (!loading_.load()) state_ = Off;                  // a load in progress sets Off itself when it notices
        }
    }

    // Stops everything and waits for the loader. Call before the DLL can unload.
    void shutdown() {
        wantOn_ = false;
        if (loader_.joinable()) loader_.join();
        teardown();
        state_ = Off;
    }
    ~DynamicAudio() { shutdown(); }

    // One NGP packet. Called from the ForceFX telemetry thread; t is seconds on any steady clock.
    void feed(const uint8_t* d, int n, double t) {
        if (state_.load() != Running) return;
        std::lock_guard<std::mutex> lk(mu_);
        if (!s_) return;
        TelemetryFrame f;
        if (!parsePacket(d, (size_t)n, t, f, s_->settings.packet)) return;
        s_->controller->onFrame(f);
        ++frames_;
    }

    // Telemetry stopped for a while: let the loops fade out instead of droning on.
    void telemetryLost() {
        if (state_.load() != Running) return;
        std::lock_guard<std::mutex> lk(mu_);
        if (s_) s_->controller->onTelemetryLost();
    }

    // A new stage may mean a new car: forget the per-car learned state.
    void newStage() {
        if (state_.load() != Running) return;
        std::lock_guard<std::mutex> lk(mu_);
        if (s_) remakeController();
    }

    // Live debris and impact multipliers from the menu (1 = as set in ForceFXAudio.ini). Safe from any thread.
    void setLevels(float debris, float impacts) {
        liveDebris_ = debris; liveImpacts_ = impacts;
        std::lock_guard<std::mutex> lk(mu_);
        if (s_) s_->sink->setLiveLevels(debris, impacts);
    }

    State state() const { return state_.load(); }

    std::string status() {
        std::lock_guard<std::mutex> lk(mu_);
        switch (state_.load()) {
            case Off: return "Dynamic audio: off";
            case Loading: return "Dynamic audio: loading sounds...";
            case Failed: return "Dynamic audio: " + error_;
            default: break;
        }
        if (!s_) return "Dynamic audio: starting";
        return "Dynamic audio: on, " + setsText_ + ". " + s_->controller->surfaceSourceText();
    }

private:
    // Everything one running session owns. The members are declared in the order they must be built, so they are
    // destroyed in reverse: the audio thread stops first, then the controller, the sink, the sound sets, the mixer and the sounds.
    struct Session {
        Settings settings;
        std::unique_ptr<SoundBank> bank;
        std::unique_ptr<Mixer> mixer;
        MaterialTable matTable;
        std::unique_ptr<SoundSetMap> sets;
        std::unique_ptr<MixerSink> sink;
        std::unique_ptr<SurfaceAudioController> controller;
        std::vector<float> beep;
        WaveOutPlayer player;
    };

    std::string dir_;
    Hooks hooks_;
    ForceFXSurfaceMemory mem_;
    std::mutex mu_;                             // guards s_ between the telemetry thread and the loader / menu
    std::unique_ptr<Session> s_;
    std::thread loader_;
    std::atomic<bool> wantOn_{false}, loading_{false};
    std::atomic<float> liveDebris_{1.0f}, liveImpacts_{1.0f};
    std::atomic<State> state_{Off};
    std::string error_, setsText_;
    long long frames_ = 0;

    void log(const std::string& s) { if (hooks_.log) hooks_.log(s); }

    void teardown() {
        std::unique_ptr<Session> old;
        { std::lock_guard<std::mutex> lk(mu_); old = std::move(s_); }
        old.reset();                            // stops the audio thread; done outside the lock so the telemetry thread is never held up
    }

    void remakeController() {                   // caller holds mu_
        Session& s = *s_;
        s.controller.reset(new SurfaceAudioController(s.settings, *s.sink));
        s.controller->setSurfaceMemory(&mem_);
        s.controller->materials().loadOverrides(dir_ + "ForceFXAudio\\materials.csv");
        s.sink->silence();
    }

    void fail(const std::string& why) {
        log("Dynamic audio: " + why);
        { std::lock_guard<std::mutex> lk(mu_); error_ = why; }
        state_ = Failed;
        wantOn_ = false;
    }

    // Runs on the loader thread.
    void load() {
        try {
            std::unique_ptr<Session> s(new Session());
            Settings& st = s->settings;
            const std::string ini = dir_ + "ForceFXAudio.ini";
            if (!loadSettings(ini, st)) log("Dynamic audio: no ForceFXAudio.ini found, using defaults");
            st.enabled = true;
            st.surfaceMode = 0;                  // surface comes from ForceFX's reader; telemetry inference is the fallback
            st.autoDiscover = false;             // ForceFX finds the tyres itself, so the audio code never scans memory
            st.memProfiles.clear();
            st.rejectedProfiles.clear();

            const std::string dataDir = dir_ + "ForceFXAudio\\";
            CreateDirectoryA(dataDir.c_str(), nullptr);
            const std::string sounds = st.soundsFolder.empty() ? dataDir + "sounds" : st.soundsFolder;

            s->bank.reset(new SoundBank());
            s->bank->normalize = st.normalizeSounds;
            s->bank->load(sounds);
            log("Dynamic audio: " + std::to_string(s->bank->fileCount) + " sound files from " + sounds);
            log("Audio devices: " + WaveOutPlayer::describeDevices());
            for (const std::string& e : s->bank->errors) log("  sound problem: " + e);

            s->mixer.reset(new Mixer());
            s->mixer->master = st.masterVolume;
            s->mixer->total = st.totalGain;
            s->mixer->cabin.configure(st.cabinEffect, st.cabinLowpassHz, st.cabinBassDb, st.cabinReverb);
            s->matTable.loadOverrides(dataDir + "materials.csv");
            s->sets.reset(new SoundSetMap(*s->bank, s->matTable, st));
            s->sink.reset(new MixerSink(*s->mixer, *s->bank, st, s->sets.get()));
            s->sink->setLiveLevels(liveDebris_.load(), liveImpacts_.load());
            s->controller.reset(new SurfaceAudioController(st, *s->sink));
            s->controller->setSurfaceMemory(&mem_);
            s->controller->materials().loadOverrides(dataDir + "materials.csv");

            if (!wantOn_.load()) { state_ = Off; loading_ = false; return; }   // switched off again while loading

            if (!s->player.start(*s->mixer, st.outputDevice)) {
                const std::string why = s->player.error;
                s.reset();
                fail("audio output failed (" + why + ")");
                loading_ = false;
                return;
            }
            if (st.startupBeep) {                                      // half a second at 440 Hz: if you hear it, the output works
                s->beep.assign((size_t)(kSampleRate / 2), 0.0f);
                for (size_t i = 0; i < s->beep.size(); ++i) {
                    const double t = (double)i / kSampleRate, env = std::min(1.0, std::min(t / 0.02, (0.5 - t) / 0.05));
                    s->beep[i] = (float)(0.5 * env * std::sin(2.0 * 3.14159265358979 * 440.0 * t));
                }
                s->mixer->playOneShot(&s->beep, 1.0f, 1.0f, 1.0f);
            }
            {
                std::string labels;
                for (int i = 0; i < s->sets->count() && i < 8; ++i) labels += (i ? ", " : "") + s->sets->info(i).label;
                log("Dynamic audio: " + std::to_string(s->sets->count()) + " sound sets (" + labels + (s->sets->count() > 8 ? ", ..." : "") + ")");
                std::lock_guard<std::mutex> lk(mu_);
                setsText_ = std::to_string(s->sets->count()) + " sound sets from " + std::to_string(s->bank->fileCount) + " files";
                s->sink->setLiveLevels(liveDebris_.load(), liveImpacts_.load());
                s_ = std::move(s);
            }
            if (!wantOn_.load()) { teardown(); state_ = Off; loading_ = false; return; }
            state_ = Running;
            loading_ = false;
            log("Dynamic audio: running");
        } catch (const std::exception& e) {
            fail(std::string("could not start (") + e.what() + ")");
            loading_ = false;
        } catch (...) {
            fail("could not start");
            loading_ = false;
        }
    }
};

}  // namespace rsa
