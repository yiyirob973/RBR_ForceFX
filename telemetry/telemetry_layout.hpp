// Where the fields are in an NGP telemetry packet. One table for the whole plugin: the force feedback, the audio and the camera all read the
// packet through this layout, so if a future NGP moves a field there is one place to fix (the Offset* keys in [ForceFX] of ForceFX.ini).
// Offsets are byte offsets, float32 unless noted; -1 means the packet does not have that field. Pure data, no Windows or game access.
// The defaults are the layout of NGP 7.4.778 (rbr.telemetry.data.TelemetryData.h).
#pragma once
#include <algorithm>

namespace Telem {

struct Layout {
    int minLength = 664;                       // packets shorter than this are not telemetry
    int steps = 0;                             // uint32: NGP's own step counter, a clean clock for the wheel counters (-1 = not there)
    int carIndex = 56;                         // NGP Car.Index
    int steering = 24, throttle = 28, brake = 32, handbrake = 36, gear = 44, carRoll = 76;
    int speed = 60;                            // the speedometer (a driven-wheel speed), not the ground speed
    int velSurge = 88, velSway = 92, velHeave = 96, yawRate = 108;
    int accSurge = 112, accSway = 116, accHeave = 120;
    int rpm = 136, rpmMax = -1;                // rpmMax: -1 = this packet has no rev limit
    float rpmScale = 1.0f;                     // multiplies the rpm field to get revs per minute
    int suspBase[4] = {152, 280, 408, 536};    // the four suspension blocks, LF, RF, LB, RB
    int suspStride = 128;                      // distance between two blocks (suspBase follows it when set with SetSuspension)
    // offsets inside one suspension block
    int springDeflection = 0, rollbarForce = 4, springForce = 8, damperForce = 12, strutForce = 16;
    int pistonVelocity = 28, brakeDiskTemp = 36, tirePressure = 44, tireTemp = 48, tireTreadTemp = 56, currentSegment = 60;   // currentSegment is uint32

    // The four suspension blocks start at `base` and are `stride` apart.
    void SetSuspension(int base, int stride) {
        suspStride = stride;
        for (int i = 0; i < 4; ++i) suspBase[i] = base + i * stride;
    }

    // The shortest packet that holds every field that is used.
    int NeededLength() const {
        int m = std::max({steps, carIndex, steering, throttle, brake, handbrake, gear, speed, velSurge, velSway, velHeave, yawRate,
                          accSurge, accSway, accHeave, rpm});
        const int inBlock = std::max({springDeflection, rollbarForce, springForce, damperForce, strutForce, pistonVelocity,
                                      brakeDiskTemp, tirePressure, tireTemp, tireTreadTemp, currentSegment});
        for (int b : suspBase) m = std::max(m, b + inBlock);
        return std::max(minLength, m + 4);
    }
};

}  // namespace Telem
