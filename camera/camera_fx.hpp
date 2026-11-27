// Camera module: head movement for the cockpit camera, and optional rain, snow and mud on the windscreen.
//
// HOW IT FITS TOGETHER
//   Telemetry   ForceFX hands every NGP packet to feed(); the packet is turned into filtered inputs (accelerations, yaw,
//               steering, brake, gear, jumps and crashes).
//   Head model  Once per rendered frame a set of springs turns those inputs into a head pose.
//   Camera      Method 4 (default) rewrites the Direct3D matrices at draw time, the way openRBRVR does, so the game's own
//               camera and wipers keep running untouched and head roll is possible. Method 2 forces a camera change every
//               frame (used when openRBRVR is loaded or the Direct3D hooks do not come up). Method 3 is experimental.
//   Windscreen  Rain, snow and mud drawn on the glass from an EndScene hook; see the [Windscreen] section of the ini.
//
// HEAD MODEL (everything is tunable in the [Camera...] sections of ForceFX.ini)
//   * Torso and neck: each translation axis is a slow, well damped torso spring plus a faster, slightly bouncy neck spring,
//     both driven by the car's acceleration. The head ends up opposite to the acceleration, lags behind it and overshoots a little.
//   * Brake boost: stronger forward lean while the brake is pressed.
//   * Bump channel: high-passed heave acceleration drives a stiff, fast vertical spring (road texture, kerbs, jumps).
//   * Head pitch: nods forward under braking and back under acceleration.
//   * Look into the corner: yaw rate and steering turn the view toward the apex.
//   * Gear-shift kick, crash and landing shakes, airborne lift.
//   * Speed fade (no movement when parked), soft limits (tanh), fixed 240 Hz sub-stepping so the springs are stable at any frame rate.
//   * Test mode: scripted inputs for checking every axis from the cockpit.
//   * Hot keys (RBR window focused, Ctrl held): F9 reloads the camera settings from ForceFX.ini, F10 camera on/off, F8 windscreen on/off, F7 syncs the
//     emulated wipers.
//
// This is written for the RallySimFans RichardBurnsRally_SSE.exe: it hooks fixed addresses in that executable, so it stays
// off for any other executable unless AnyExe=1 is set in the ini.
//
// TELEMETRY LAYOUT (pack 1, NGP\sdk\rbr.telemetry.data.TelemetryData.h):
//   Control @24: steering 24, throttle 28, brake 32, handbrake 36, clutch 40, gear (int) 44
//   Car @56: speed 60, velocities (6 floats) @88 (surge, sway, heave, roll, pitch, yaw -> yaw rate @108),
//            accelerations (6 floats) @112: surge 112, sway 116, heave 120, roll 124, pitch 128, yaw 132
#pragma once
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <windows.h>
#include <d3d9.h>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cctype>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <algorithm>
#include "../thirdparty/minhook/include/MinHook.h"
#include "road/materials.hpp"
#include "telemetry/telemetry_layout.hpp"   // where the fields are in an NGP packet: one table for the force feedback, the audio and the camera
#include "ws_logic.hpp"
#include "ws_art.hpp"

namespace ffx {
namespace cam {
namespace {

// The parts, in the order they must appear (they share one anonymous namespace and its state, so each needs the ones above it):
#include "fx/config.inl"
#include "fx/game_memory.inl"
#include "fx/telemetry.inl"
#include "fx/head_model.inl"
#include "fx/camera_hooks.inl"
#include "fx/windscreen_textures.inl"
#include "fx/windscreen_simulation.inl"
#include "fx/windscreen_drawing.inl"
#include "fx/windscreen_wipers.inl"
#include "fx/windscreen_hooks.inl"
#include "fx/module.inl"
}  // namespace cam
}  // namespace ffx
