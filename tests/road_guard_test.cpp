// Tests for the road guard (road_guard.hpp). Self-contained:
//   g++ -std=c++17 -O1 road_guard_test.cpp -o road_guard_test && ./road_guard_test
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "../road_guard.hpp"

static int g_fail = 0, g_checks = 0;
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)

static double rmsAt(float hz, bool cut) {
    Road::LowCut lc; double t = 0, ph = 0, e = 0, n = 0;
    for (int i = 0; i < 60000; ++i) {
        const float dt = (3.0f + 1.5f * (std::rand() % 1000) / 1000.0f) / 1000.0f;
        t += dt; ph += 6.2831853 * hz * dt;
        float v = 1000.0f * (float)std::sin(ph);
        if (cut) v = lc.Step(v, dt);
        if (t > 1.0) { e += (double)v * v; n += 1; }
    }
    return std::sqrt(e / n);
}

static void lowcut_passes_the_road_and_removes_slow_movement() {
    CHECK(rmsAt(40.0f, true) > 0.90 * rmsAt(40.0f, false));     // the road range is nearly untouched
    CHECK(rmsAt(25.0f, true) > 0.85 * rmsAt(25.0f, false));
    CHECK(rmsAt(2.0f, true) < 0.15 * rmsAt(2.0f, false));       // a slow push is taken away
    CHECK(rmsAt(0.5f, true) < 0.02 * rmsAt(0.5f, false));
}
static void lowcut_removes_a_steady_offset() {
    Road::LowCut lc; float v = 0;
    for (int i = 0; i < 4000; ++i) v = lc.Step(800.0f, 0.004f);
    CHECK(std::fabs(v) < 1.0f);
}
static void lowcut_survives_bad_input() {
    Road::LowCut lc;
    CHECK(std::isfinite(lc.Step(NAN, 0.004f)));
    CHECK(std::isfinite(lc.Step(INFINITY, 0.004f)));
    CHECK(std::isfinite(lc.Step(100.0f, NAN)));
    CHECK(std::isfinite(lc.Step(100.0f, 5.0f)));
    lc.Reset(); CHECK(lc.Step(500.0f, 0.004f) == 0.0f);          // starts quiet, no jump
}
static void headroom_keeps_vibration_centred() {
    CHECK(Road::FitHeadroom(0, 3000) == 3000);
    CHECK(Road::FitHeadroom(6000, 3000) == 3000);
    CHECK(Road::FitHeadroom(9000, 3000) == 1000);
    CHECK(Road::FitHeadroom(9000, -3000) == -1000);              // same limit both ways: no shift of the average
    CHECK(Road::FitHeadroom(-9000, 3000) == 1000);
    CHECK(Road::FitHeadroom(10000, 500) == 0);
    CHECK(Road::FitHeadroom(12000, 500) == 0);
    double sum = 0; for (int i = 0; i < 3600; ++i) sum += Road::FitHeadroom(9200, (int)std::lround(2000 * std::sin(i * 6.2831853 / 360.0)));   // ten whole cycles
    CHECK(std::fabs(sum) < 1.0);                                  // a clipped sine still averages to zero
}
static void ceiling_leaves_normal_road_and_bounds_extreme_road() {
    const float R = 500.0f;
    float calm[3] = {0.1f * R, 0.1f * R, 0.1f * R};
    CHECK(Road::CeilingScale(calm, R) == 1.0f);
    float big[3] = {3 * R, 3 * R, 3 * R};
    const float s = Road::CeilingScale(big, R);
    float rms = 0; for (int b = 0; b < 3; ++b) rms += (big[b] * s) * (big[b] * s);
    CHECK(std::sqrt(rms) <= Road::kCeilMax * R * 1.0001f && std::sqrt(rms) > Road::kCeilKnee * R);
    float prev = 0;                                               // smooth and rising
    for (int i = 1; i <= 60; ++i) { float m[3] = {i * 0.05f * R, 0, 0}; const float o = m[0] * Road::CeilingScale(m, R); CHECK(o >= prev - 1e-3f); prev = o; }
    float one[3] = {2 * R, 0, 0}; CHECK(Road::CeilingScale(one, 0.0f) == 1.0f && Road::CeilingScale(one, NAN) == 1.0f);
    float nan3[3] = {NAN, 100.0f, 0}; CHECK(std::isfinite(Road::CeilingScale(nan3, R)));
}
int main() {
    lowcut_passes_the_road_and_removes_slow_movement(); lowcut_removes_a_steady_offset(); lowcut_survives_bad_input();
    headroom_keeps_vibration_centred(); ceiling_leaves_normal_road_and_bounds_extreme_road();
    std::printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
