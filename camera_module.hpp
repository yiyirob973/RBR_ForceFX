// Optional camera module: head movement for the cockpit camera, and the optional windscreen effects (rain, snow, mud).
//
// Build with /DFORCEFX_CAMERA to include it. Without that define, ffx::Camera is an empty stand-in with the same interface, so
// the rest of the plugin compiles unchanged and none of the camera code (or MinHook, or Direct3D) is pulled in.
#pragma once
#include <cstdint>
#include <functional>
#include <string>

#ifdef FORCEFX_CAMERA

#include "camera/camera_fx.hpp"
namespace ffx { using Camera = cam::CameraFx; }

#else

namespace ffx {

class Camera {
public:
    struct Hooks {
        std::function<void(const std::string&)> log;
        std::function<bool(int[4])> readIds;       // material id under each tyre (FL, FR, RL, RR); unused in this stand-in
    };

    void configure(const std::string&, Hooks, bool, bool) {}
    void setActive(bool) {}
    void setWindscreen(bool) {}
    void feed(const uint8_t*, int) {}
    void stageStarted(int) {}
    std::string status() { return std::string(); }
    void shutdown() {}
};

}  // namespace ffx

#endif
