// Optional dynamic audio module.
//
// Build with /DFORCEFX_AUDIO to include it. Without that define, ffx::Audio is an empty stand-in with the same
// interface, so the rest of the plugin compiles unchanged and none of the audio code (or winmm) is pulled in.
#pragma once
#include <cstdint>
#include <functional>
#include <string>

#ifdef FORCEFX_AUDIO

#include "audio/dynamic_audio.hpp"
namespace ffx { using Audio = ffx::DynamicAudio; }

#else

namespace ffx {

class Audio {
public:
    struct Hooks {
        std::function<bool(int ids[4])> readIds;
        std::function<bool()> idsAvailable;
        std::function<std::string()> idsStatus;
        std::function<void(const std::string&)> log;
    };

    void configure(const std::string&, Hooks) {}
    void setActive(bool) {}
    void shutdown() {}
    void feed(const uint8_t*, int, double) {}
    long long msSinceFrame() const { return 0; }
    void telemetryLost() {}
    void newStage() {}
    void setLevels(float, float, float) {}
    void splash(float, int) {}
    void setCar(int, double, double, double) {}
    void setSpeedUnits(bool) {}
    std::string status() { return std::string(); }
};

}  // namespace ffx

#endif
