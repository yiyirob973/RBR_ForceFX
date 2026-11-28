// Part of camera_fx.hpp: included from inside its anonymous namespace (ffx::cam), in the order listed there. Not a header on its own.
// ================================================================================================ head model
struct Spring {                                                        // critically-tunable 2nd-order follower
    float x = 0, v = 0;
    void step(float h, float xeq, float w, float z) { const float a = w * w * (xeq - x) - 2.f * z * w * v; v += a * h; x += v * h; }
    void reset() { x = v = 0; }
};
struct Head {
    Spring torso[3], neck[3];                                          // 0 = forward, 1 = right, 2 = up   (metres)
    Spring bump;                                                       // up (metres)
    Spring pitch, look, roll;                                          // degrees: pitch + = look down, look + = look right, roll + = tilt right
    void reset() { for (int i = 0; i < 3; ++i) { torso[i].reset(); neck[i].reset(); } bump.reset(); pitch.reset(); look.reset(); roll.reset(); }
} g_head;

struct Shake { float amp = 0, t = 0, ph[5][2] = {{0}}; } g_shake;                  // decaying multi-frequency shake for crashes / landings
unsigned g_rng = 12345u;
float rnd01() { g_rng = g_rng * 1664525u + 1013904223u; return (float)(g_rng >> 8) / 16777216.f; }
void startShake(float add) {
    g_shake.amp = std::min(1.f, g_shake.amp + add); g_shake.t = 0.f;
    for (int i = 0; i < 5; ++i) for (int j = 0; j < 2; ++j) g_shake.ph[i][j] = rnd01() * 6.2831853f;
}

struct Drive { float gS = 0, gSw = 0, gH = 0, gHp = 0, yawN = 0, steerN = 0, speed = 0, brake = 0, carRoll = 0; };

LARGE_INTEGER g_qf, g_qlast;
bool  g_dirty = false;                                                 // camera currently carries a non-zero offset
float g_offPos[3] = {0, 0, 0};                                         // camera-axis position offset (metres)
float g_angPitch = 0, g_angYaw = 0;                                    // radians (+ pitch = look down, + yaw = look right)
float g_hF = 0, g_hR = 0, g_hU = 0, g_hPitchDeg = 0, g_hYawDeg = 0, g_hRollDeg = 0;   // internal head pose (forward/right/up metres, degrees) for Method 4
void resetHead() { g_head.reset(); g_shake.amp = 0.f; QueryPerformanceCounter(&g_qlast); for (int i = 0; i < 3; ++i) g_offPos[i] = 0; g_angPitch = g_angYaw = 0; }
void zeroOffsets() { for (int i = 0; i < 3; ++i) g_offPos[i] = 0; g_angPitch = g_angYaw = 0; g_hF = g_hR = g_hU = g_hPitchDeg = g_hYawDeg = g_hRollDeg = 0; }

float softLimit(float x, float m) { return m * std::tanh(x / m); }
float speedFade(float spd) {
    const float t = clamp01((spd - g_cfg.minSpeed) / (g_cfg.fullSpeed - g_cfg.minSpeed));
    return t * t * (3.f - 2.f * t);                                    // smoothstep
}

void stepHead(float dt, const Drive& d) {
    const Cfg& c = g_cfg;
    const float TWO_PI = 6.2831853f, DEG2RAD = 0.017453293f;
    const float fade = speedFade(d.speed);
    const float boost = 1.f + (c.brakeBoost - 1.f) * clamp01(d.brake);

    const float imp = g_shiftImpulse.exchange(0.f);                    // gear-shift kick: forward impulse on the neck
    if (imp != 0.f) g_head.neck[0].v += imp * c.shiftKick;

    // impact events from the telemetry thread
    const float evS = g_evCrashSurge.exchange(0.f), evW = g_evCrashSway.exchange(0.f), evL = g_evLand.exchange(0.f);
    if (evS != 0.f || evW != 0.f || evL != 0.f) g_wsSplash = std::max(g_wsSplash.load(), std::max(evL, std::max(std::fabs(evS), std::fabs(evW)) * 0.3f));
    if (c.impactScale > 0.f && (evS != 0.f || evW != 0.f || evL != 0.f)) {
        const float sc = c.impactScale;
        g_head.neck[0].v += clampf(-c.crashKick * evS * sc, c.maxKick);            // frontal impact (evS < 0): head thrown forward
        g_head.neck[1].v += clampf(-c.crashKick * evW * sc, c.maxKick);            // side impact: head thrown the other way
        g_head.pitch.v   += clampf(-c.crashPitchKick * evS * sc, 60.f);
        g_head.roll.v    += clampf(c.crashRollKick * evW * sc, 60.f);
        if (evL > 0.f) {                                                           // landing: head dips, nods, jolts
            g_head.torso[2].v += clampf(-c.landKick * evL * sc, c.maxKick);
            g_head.bump.v     += clampf(-c.landKick * 0.5f * evL * sc, c.maxKick);
            g_head.pitch.v    += c.landPitchKick * evL * sc;
        }
        startShake(c.shakeGain * sc * (std::fabs(evS) + std::fabs(evW) + 0.6f * evL));
    }

    // static targets: the head ends up OPPOSITE to the car's acceleration (internal frame: + = forward / right / up)
    const float eqF = -c.kSurge * d.gS * boost * fade;
    const float eqR = -c.kSway * d.gSw * fade;
    const float eqU = -c.kHeave * d.gH * fade + (g_airborne.load() ? c.airLift : 0.f);   // airborne: the head floats up a little
    const float eqB = -c.bumpGain * d.gHp * fade;
    const float eqPitch = -c.pitchPerG * d.gS * boost * fade;                              // braking (gS < 0) -> look down
    const float eqLook = c.lookYaw * ((1.f - c.lookSteerMix) * d.yawN + c.lookSteerMix * d.steerN) * fade;
    // roll: lean the head into the corner (+ = tilt right when the car accelerates to the right) and optionally cancel the car's own roll
    const float carRollDeg = d.carRoll * c.carRollScale * 57.29578f * (float)c.invCarRoll;
    const float eqRoll = (c.rollPerG * d.gSw - c.horizonLock * carRollDeg) * fade;

    const float wT = TWO_PI * c.torsoFreq, wN = TWO_PI * c.neckFreq, wB = TWO_PI * c.bumpFreq,
                wP = TWO_PI * c.pitchFreq, wL = TWO_PI * c.lookFreq, wR = TWO_PI * c.rollFreq;
    const int   steps = std::max(1, (int)std::ceil(dt * 240.f));       // fixed ~240 Hz sub-stepping
    const float h = dt / (float)steps, sh = c.torsoShare, eq3[3] = {eqF, eqR, eqU};
    for (int s = 0; s < steps; ++s) {
        for (int i = 0; i < 3; ++i) {
            g_head.torso[i].step(h, eq3[i] * sh, wT, c.torsoDamp);
            g_head.neck[i].step(h, eq3[i] * (1.f - sh), wN, c.neckDamp);
        }
        g_head.bump.step(h, eqB, wB, c.bumpDamp);
        g_head.pitch.step(h, eqPitch, wP, c.pitchDamp);
        g_head.look.step(h, eqLook, wL, c.lookDamp);
        g_head.roll.step(h, eqRoll, wR, c.rollDamp);
    }

    float F = g_head.torso[0].x + g_head.neck[0].x;
    float R = g_head.torso[1].x + g_head.neck[1].x;
    float U = g_head.torso[2].x + g_head.neck[2].x + g_head.bump.x;
    if (!std::isfinite(F + R + U + g_head.pitch.x + g_head.look.x + g_head.roll.x)) { g_head.reset(); F = R = U = 0; }

    F = softLimit(F, c.maxPos) * c.intensity; R = softLimit(R, c.maxPos) * c.intensity; U = softLimit(U, c.maxPos) * c.intensity;
    float shF = 0, shR = 0, shU = 0, shPitch = 0, shRoll = 0;
    if (g_shake.amp > 1e-4f) {
        g_shake.t += dt; g_shake.amp *= std::exp(-dt / c.shakeDecay);
        float v[5];
        for (int i = 0; i < 5; ++i)
            v[i] = 0.6f * std::sin(TWO_PI * c.shakeFreq1 * g_shake.t + g_shake.ph[i][0]) + 0.4f * std::sin(TWO_PI * c.shakeFreq2 * g_shake.t + g_shake.ph[i][1]);
        const float a = g_shake.amp * c.impactScale * c.intensity;
        shF = a * c.shakePos * 0.5f * v[0]; shR = a * c.shakePos * 0.7f * v[1]; shU = a * c.shakePos * v[2];
        shPitch = a * c.shakeAng * 0.8f * v[3]; shRoll = a * c.shakeAng * 0.6f * v[4];
    }
    F += shF; R += shR; U += shU;
    zeroOffsets();
    g_offPos[c.axFwd] = c.sgFwd * F; g_offPos[c.axRight] = c.sgRight * R; g_offPos[c.axUp] = c.sgUp * U;
    g_angPitch = (softLimit(g_head.pitch.x, c.maxAng) * c.intensity + shPitch) * DEG2RAD;
    g_angYaw   = softLimit(g_head.look.x,  c.maxAng) * c.intensity * DEG2RAD;
    g_hF = F; g_hR = R; g_hU = U;
    g_hPitchDeg = g_angPitch / DEG2RAD; g_hYawDeg = g_angYaw / DEG2RAD;
    g_hRollDeg = softLimit(g_head.roll.x, c.maxRoll) * c.intensity + shRoll;
}

// Add the head offset to cam_pos and rotate the look direction (cam_target - cam_pos) by the head angles.
void applyHead(float* pos, float* tgt) {
    const Cfg& c = g_cfg;
    float d[3] = {tgt[0] - pos[0], tgt[1] - pos[1], tgt[2] - pos[2]};
    const float f = d[c.axFwd] * c.sgFwd, r = d[c.axRight] * c.sgRight, u = d[c.axUp] * c.sgUp;   // forward / right / up components
    const float cp = std::cos(g_angPitch), sp = std::sin(g_angPitch), cy = std::cos(g_angYaw), sy = std::sin(g_angYaw);
    const float f1 = f * cp + u * sp, u1 = u * cp - f * sp;            // pitch (+ = look down)
    const float f2 = f1 * cy - r * sy, r2 = r * cy + f1 * sy;          // yaw   (+ = look right)
    d[c.axFwd] = f2 * c.sgFwd; d[c.axRight] = r2 * c.sgRight; d[c.axUp] = u1 * c.sgUp;
    for (int i = 0; i < 3; ++i) { pos[i] += g_offPos[i]; tgt[i] = pos[i] + d[i]; }
}

// scripted inputs (internal sign convention) to verify every axis from the cockpit
const char* testDrive(DWORD now, Drive& d, int& phaseOut) {
    static DWORD t0 = 0; if (!t0) t0 = now;
    const float t = (float)(now - t0) * 0.001f, cyc = 1.6f;
    const int phase = (int)(t / cyc) % 8; const float local = std::fmod(t, cyc);
    phaseOut = phase;
    const bool on = local < 1.0f;
    d = Drive(); d.speed = 100.f; d.brake = 0.f;
    static int lastCycle = -1; const int cycleNo = (int)(t / cyc); const bool newCycle = cycleNo != lastCycle; lastCycle = cycleNo;
    switch (phase) {
    case 0: if (on) { d.gS = -2.f; d.brake = 1.f; } return "BRAKING: head should lean FORWARD and nod DOWN";
    case 1: if (on) d.gS = 1.5f;                    return "ACCELERATING: head should go BACK and nod UP";
    case 2: if (on) { d.gSw = 2.f; d.yawN = 1.f; d.steerN = 1.f; } return "RIGHT TURN: head moves LEFT (outside), view looks RIGHT, head tilts RIGHT (roll: Method 4)";
    case 3: if (on) { d.gSw = -2.f; d.yawN = -1.f; d.steerN = -1.f; } return "LEFT TURN: head moves RIGHT (outside), view looks LEFT, head tilts LEFT (roll: Method 4)";
    case 4: if (local < 0.15f) { d.gH = 3.f; d.gHp = 3.f; } return "BUMP: head should dip DOWN briefly, then bounce";
    case 5: if (local < 0.15f) { d.gH = -3.f; d.gHp = -3.f; } return "DROP: head should rise UP briefly, then settle";
    case 6: if (newCycle) g_evLand = 1.5f; return "LANDING: head dips, nods and jolts, then a short shake";
    default: if (newCycle) { g_evCrashSurge = -3.f; g_evCrashSway = 1.5f; } return "CRASH: head thrown FORWARD and slightly LEFT with a decaying shake";
    }
}

