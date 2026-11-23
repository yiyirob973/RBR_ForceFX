// Compares ways of estimating the front load that acceleration takes off, on a simulated stage (noise, hill starts, mismatched cars).
// Build: g++ -std=c++17 -O2 accel_light_compare.cpp -o accel_light_compare && ./accel_light_compare [speedNoise_m_s] [telemetryEveryNSteps]
// A = share against standing (the plugin before the gate), B = acceleration model with a learned constant, D = A gated by acceleration (what the plugin does), E = A with a noise-sized dead band (rejected).
// Compares ways of estimating "how much front load has the acceleration taken off" from telemetry, on a simulated stage.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include "../../accel_light.hpp"
using namespace std;
static float alphaFor(float dt, float tau) { return 1.f - exp(-dt / tau); }
static const float G = 9.81f, HL = 0.20f, S0 = 0.52f; static float g_vNoise = 0.01f; static int g_hold = 1;     // car: CoG height / wheelbase, static front share

struct Method { virtual ~Method() {} float thr = 0, aTrue = 0; virtual float step(float share, float speed, float dt) = 0; virtual const char* name() = 0; };

// A: the tracker that is in the plugin now (front share against the standing share, smoothed 0.07 s)
struct MA : Method {
    AccelLight::Tracker t; float vPrev = 0; bool first = true;
    float step(float share, float speed, float dt) override {
        AccelLight::In in; in.frontShare = share; in.speed = speed;
        vPrev = speed; first = false;
        in.throttle = thr; in.accOk = true; in.accMag = fabs(aTrue);
        return t.Update(in, dt).drop;
    }
    const char* name() override { return "A share vs standing (now)"; }
};

// B: acceleration from the speed, the car's load-transfer constant learned from its own strut data (recursive least squares), prior until then
struct MB : Method {
    float vPrev = 0, af = 0, sf = S0; bool first = true;
    float th[2] = {S0, 0.20f}, P[2][2] = {{0.5f, 0}, {0, 2.0f}}; bool haveShare = false;
    float step(float share, float speed, float dt) override {
        const float a = first ? 0 : (speed - vPrev) / dt; vPrev = speed;
        if (first) { sf = share; th[0] = share; first = false; }
        af += (a / G - af) * alphaFor(dt, 0.12f);                              // acceleration in g, lagged like the suspension is
        sf += (share - sf) * alphaFor(dt, 0.12f);
        // RLS on  share = c0 - k * a   with forgetting
        const float lam = exp(-dt / 60.f), phi[2] = {1.f, -af};
        const float Pphi[2] = {P[0][0] * phi[0] + P[0][1] * phi[1], P[1][0] * phi[0] + P[1][1] * phi[1]};
        const float den = lam + phi[0] * Pphi[0] + phi[1] * Pphi[1];
        const float K[2] = {Pphi[0] / den, Pphi[1] / den}, err = sf - (th[0] * phi[0] + th[1] * phi[1]);
        th[0] += K[0] * err; th[1] += K[1] * err;
        float nP[2][2];
        for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) nP[i][j] = (P[i][j] - K[i] * (phi[0] * P[0][j] + phi[1] * P[1][j])) / lam;
        for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) P[i][j] = min(nP[i][j], 5.f);
        const float k = min(0.45f, max(0.05f, th[1])), c0 = min(0.8f, max(0.3f, th[0]));
        return min(0.6f, max(0.f, k * max(0.f, af) / c0));
    }
    const char* name() override { return "B accel from speed, k learned (RLS)"; }
};

// C: the same with a fixed constant (no learning): what a rally-car default would give
struct MC : Method {
    float vPrev = 0, af = 0; bool first = true;
    float step(float, float speed, float dt) override {
        const float a = first ? 0 : (speed - vPrev) / dt; vPrev = speed; first = false;
        af += (a / G - af) * alphaFor(dt, 0.12f);
        return min(0.6f, max(0.f, 0.20f * max(0.f, af) / S0));
    }
    const char* name() override { return "C accel from speed, fixed k=0.20"; }
};



static float sm01(float a, float b, float x) { float t = max(0.f, min(1.f, (x - a) / (b - a))); return t * t * (3 - 2 * t); }
// D: A, but it only counts while the car is really accelerating forward (acceleration from the ground speed, filtered), so noise in the load can not lighten the steering when it is not
struct MD : Method {
    MA a; float vPrev = 0, af = 0; bool first = true;
    float step(float share, float speed, float dt) override {
        a.thr = thr; a.aTrue = aTrue;
        const float d = a.step(share, speed, dt);
        const float acc = first ? 0 : (speed - vPrev) / dt; vPrev = speed; first = false;
        af += (acc / G - af) * alphaFor(dt, 0.25f);
        return d * sm01(0.06f, 0.22f, af);
    }
    const char* name() override { return "D A gated by measured acceleration"; }
};
// E: A with a deadband that grows with the measured noise of the share (what the load itself says it can resolve)
struct ME : Method {
    AccelLight::Tracker t; float sm = 0, var = 0, drop = 0; bool init = false;
    float step(float share, float speed, float dt) override {
        AccelLight::In in; in.frontShare = share; in.speed = speed; in.throttle = thr; in.accOk = true; in.accMag = fabs(aTrue);
        const AccelLight::Out o = t.Update(in, dt);
        if (!init) { sm = share; init = true; }
        const float hp = share - sm; sm += (share - sm) * alphaFor(dt, 0.6f);
        var += (hp * hp - var) * alphaFor(dt, 1.0f);
        const float sigmaRel = o.base > 0.05f ? sqrt(max(0.f, var)) / o.base : 0.f;
        const float target = o.learned ? min(0.6f, max(0.f, o.rawDrop - 2.f * sigmaRel)) : 0.f;       // rawDrop: unsmoothed; the deadband is twice the noise
        drop += (target - drop) * alphaFor(dt, target > drop ? 0.12f : 0.25f);
        return drop;
    }
    const char* name() override { return "E A with a noise-sized deadband"; }
};
struct Scenario { const char* name; float noiseAbs; float startGrade; float carHL; };
struct Result { float rms[4] = {0,0,0,0}, falseLight[4] = {0,0,0,0}, launchAbs[4] = {0,0,0,0}, t70[4] = {0,0,0,0}; };

static Result run(const Scenario& sc, unsigned seed) {
    mt19937 rng(seed); normal_distribution<float> N(0, 1); uniform_real_distribution<float> U(0, 1);
    vector<Method*> ms = {new MA, new MB, new MD, new ME};
    const float dt = 0.01f; const int steps = 12000;
    vector<float> aPlan(steps, 0.f);
    { int i = 500; bool first = true; while (i < steps) { const float r = first ? 0.f : U(rng); int len; float a;
        if (r < 0.45f) { len = (int)((2 + 4 * U(rng)) * 100); a = 0.25f + 0.6f * U(rng); }
        else if (r < 0.75f) { len = (int)((2 + 3 * U(rng)) * 100); a = 0.f; }
        else { len = (int)((1 + 2 * U(rng)) * 100); a = -(0.4f + 0.8f * U(rng)); }
        first = false; for (int k = 0; k < len && i < steps; ++k, ++i) aPlan[i] = a; } }
    float v = 0, grade = sc.startGrade, lag = 0;
    float ph[6], fr[6]; for (int i = 0; i < 6; ++i) { ph[i] = U(rng) * 6.28f; fr[i] = 3.f + 9.f * U(rng); }
    double se[4] = {0,0,0,0}, fl[4] = {0,0,0,0}; long nAll = 0, nFalse = 0;
    double lau[4] = {0,0,0,0}; int nLau = 0; int launchStart = -1; float t70[4] = {-1,-1,-1,-1};
    for (int i = 0; i < steps; ++i) {
        const float t = i * dt;
        float a = aPlan[i] * G; if (v <= 0.f && a < 0) a = 0.f;
        v = max(0.f, v + a * dt);
        if (i % 800 == 0 && i > 0 && sc.startGrade != 99) grade = max(-0.12f, min(0.12f, grade + 0.04f * N(rng)));
        lag += (a / G - lag) * alphaFor(dt, 0.12f);
        const float tdrop = sc.carHL * max(0.f, lag) / S0;
        float bump = 0; for (int k = 0; k < 6; ++k) bump += sin(6.28f * fr[k] * t + ph[k]);
        const float vf = min(1.f, v / 12.f);
        const float share = max(0.1f, min(0.9f, S0 - sc.carHL * lag - sc.carHL * grade + vf * sc.noiseAbs * (0.55f * bump / 2.45f * 1.7f + 0.8f * N(rng))));
        static float vmHold = 0; if (i % g_hold == 0) vmHold = v + g_vNoise * N(rng); const float vm = vmHold;
        if (launchStart < 0 && a > 0.2f * G) launchStart = i;
        for (int m = 0; m < 4; ++m) {
            ms[m]->thr = aPlan[i] > 0.05f ? 1.f : (aPlan[i] < -0.05f ? 0.f : 0.2f); ms[m]->aTrue = a;
            const float e = ms[m]->step(share, vm, dt);
            se[m] += (e - tdrop) * (e - tdrop);
            if (tdrop < 0.01f && v > 3.f) fl[m] += e;
            if (launchStart >= 0 && i < launchStart + 300) { lau[m] += fabs(e - tdrop); if (t70[m] < 0 && tdrop > 0.05f && e > 0.7f * tdrop) t70[m] = (i - launchStart) * dt; }
        }
        ++nAll; if (tdrop < 0.01f && v > 3.f) ++nFalse; if (launchStart >= 0 && i < launchStart + 300) ++nLau;
    }
    Result r;
    for (int m = 0; m < 4; ++m) { r.rms[m] = sqrt(se[m] / nAll); r.falseLight[m] = nFalse ? fl[m] / nFalse : 0; r.launchAbs[m] = nLau ? lau[m] / nLau : 0; r.t70[m] = t70[m] < 0 ? 3.f : t70[m]; delete ms[m]; }
    return r;
}
int main(int argc, char** argv) {
    if (argc > 1) g_vNoise = atof(argv[1]); if (argc > 2) g_hold = atoi(argv[2]);
    printf("speed noise %.3f m/s, telemetry every %d loop step(s)\n", g_vNoise, g_hold);
    const Scenario sc[] = {
        {"share noise 2%  (calm gravel)", 0.02f, 0.f, 0.20f},
        {"share noise 5%  (rough gravel)", 0.05f, 0.f, 0.20f},
        {"share noise 10% (very rough, jumps)", 0.10f, 0.f, 0.20f},
        {"share noise 20% (damper spikes dominate)", 0.20f, 0.f, 0.20f},
        {"share noise 10%, h/L 0.27", 0.10f, 0.f, 0.27f},
        {"share noise 20%, downhill start, h/L 0.14", 0.20f, -0.10f, 0.14f},
    };
    printf("%-42s | RMS error (fraction of force)  | false light while not accelerating | first-launch mean |err| | time to 70%% of truth (s)\n", "scenario (6 seeds)");
    printf("%-42s | A now  B model D gated E dbnd | A      B      D      E          | A     B     D     E     | A     B     D     E\n", "");
    for (const Scenario& s : sc) {
        Result avg; const int N = 6;
        for (int k = 0; k < N; ++k) { Result r = run(s, 100 + k); for (int m = 0; m < 4; ++m) { avg.rms[m] += r.rms[m] / N; avg.falseLight[m] += r.falseLight[m] / N; avg.launchAbs[m] += r.launchAbs[m] / N; avg.t70[m] += r.t70[m] / N; } }
        printf("%-42s | %.3f  %.3f   %.3f   %.3f  | %.3f  %.3f  %.3f  %.3f      | %.3f %.3f %.3f %.3f | %.2f  %.2f  %.2f  %.2f\n", s.name, avg.rms[0], avg.rms[1], avg.rms[2], avg.rms[3], avg.falseLight[0], avg.falseLight[1], avg.falseLight[2], avg.falseLight[3], avg.launchAbs[0], avg.launchAbs[1], avg.launchAbs[2], avg.launchAbs[3], avg.t70[0], avg.t70[1], avg.t70[2], avg.t70[3]);
    }
}
