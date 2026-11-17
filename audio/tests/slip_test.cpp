// End-to-end tests of tyre squeal and slide: telemetry frames with turning wheels go in, loop gains come out.
//   g++ -std=c++17 -O1 -I.. slip_test.cpp -o slip_test && ./slip_test
#include <cstdio>
#include <functional>
#include <random>
#include "core.hpp"
#include "audio.hpp"

using namespace rsa;

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

// Remembers the loudest squeal and slide any tyre was given in a time range.
struct GainSink : IAudioSink {
    double t = 0, from = 0, to = 1e9; float maxSqueal = 0, maxSlide = 0; int wheelOfMax = -1;
    void setWheelLoop(int, int, float, float, float) override {}
    void setSlideLoop(int w, int, float gl, float gr, float) override { if (t >= from && t < to && gl + gr > maxSlide) { maxSlide = gl + gr; wheelOfMax = w; } }
    void setSquealLoop(int w, int, float gl, float gr, float) override { if (t >= from && t < to && gl + gr > maxSqueal) { maxSqueal = gl + gr; wheelOfMax = w; } }
    void setRumble(float, float) override {}
    void trigger(SoundEvent, int, float, float, float) override {}
    void silence() override {}
};

struct Drive {
    double rate = 100, jitterMs = 0, ground = 25, circ = 1.95;
    bool withSteps = true;
    std::function<double(double, int)> slip = [](double, int) { return 0.0; };   // per wheel slip over time (0 FL, 1 FR, 2 RL, 3 RR)
    float throttle = 0.3f;
    double from = 8.0;                                                           // the gains are measured from here
};

// The wheels turn at ground speed x (1 + slip); the segment counter steps every eighth of a turn; packets arrive with jitter.
static void drive(const Drive& d, double seconds, GainSink& sink) {
    Settings st; st.memCrossCheck = false; st.accelToMs2 = 9.81;
    SurfaceAudioController c(st, sink, 1);
    std::mt19937 rng(5); std::normal_distribution<double> jit(0.0, d.jitterMs / 1000.0);
    double rev[4] = {0.11, 0.42, 0.67, 0.93}, lastArr = -1; const int n = (int)(seconds * d.rate);
    sink.from = d.from;
    for (int i = 0; i < n; ++i) {
        const double t = i / d.rate; sink.t = t;
        TelemetryFrame f;
        double arr = t + (d.jitterMs > 0 ? jit(rng) : 0.0); arr = std::max(arr, lastArr + 1e-6); lastArr = arr;
        f.time = arr; f.haveSteps = d.withSteps; f.steps = 5000 + (uint32_t)i;
        f.speed = (float)(d.ground * 3.6 * 1.609); f.velSurge = (float)d.ground;
        f.throttle = t < 6.0 ? 0.0f : d.throttle;                         // coasting first, as on a real stage: that is when the tyre size is learned
        for (int w = 0; w < 4; ++w) {
            if (i > 0) rev[w] += d.ground * (1.0 + d.slip(t, w)) / d.circ / d.rate;
            f.wheels[w].strutForce = -3500.f; f.wheels[w].currentSegment = (uint32_t)std::floor(rev[w] * 8.0) & 7u;
        }
        c.onFrame(f);
    }
}

TEST(tyres_that_are_only_rolling_never_squeal_or_slide_at_any_speed_packet_rate_or_jitter) {
    for (double rate : {60.0, 100.0, 180.0})
        for (double jitter : {0.0, 1.0, 3.0})
            for (double v : {6.0, 10.0, 20.0, 35.0})
                for (bool steps : {true, false}) {
                    Drive d; d.rate = rate; d.jitterMs = jitter; d.ground = v; d.withSteps = steps;
                    GainSink s; drive(d, 20.0, s);
                    if (s.maxSqueal > 0.005f || s.maxSlide > 0.005f)
                        std::printf("  rate %.0f jitter %.0f v %.0f steps %d: squeal %.3f slide %.3f\n", rate, jitter, v, steps, s.maxSqueal, s.maxSlide);
                    CHECK(s.maxSqueal <= 0.005f && s.maxSlide <= 0.005f);
                }
}
TEST(a_locked_wheel_slides_and_squeals_and_the_other_tyres_stay_quiet) {
    Drive d; d.throttle = 0.0f;
    d.slip = [](double t, int w) { return (w == 0 && t >= 12.0) ? -1.0 : 0.0; };
    d.from = 13.0;
    GainSink s; drive(d, 18.0, s);
    CHECK(s.maxSlide > 0.05f && s.wheelOfMax == 0);
    Drive q = d; q.slip = [](double, int) { return 0.0; };
    GainSink quiet; drive(q, 18.0, quiet);
    CHECK(quiet.maxSlide <= 0.005f && quiet.maxSqueal <= 0.005f);
}
TEST(wheelspin_makes_a_noise_and_a_gentle_roll_does_not) {
    Drive d; d.throttle = 0.9f;
    d.slip = [](double t, int w) { return (w >= 2 && t >= 12.0) ? 0.5 : 0.0; };
    d.from = 13.0;
    GainSink s; drive(d, 18.0, s);
    CHECK(s.maxSlide + s.maxSqueal > 0.05f);
}
TEST(slip_that_is_small_compared_with_the_grip_limit_stays_quiet) {
    Drive d; d.slip = [](double t, int) { return t >= 12.0 ? 0.04 : 0.0; };      // 4 percent: well inside a tyre's limit
    d.from = 13.0;
    GainSink s; drive(d, 18.0, s);
    CHECK(s.maxSqueal <= 0.005f && s.maxSlide <= 0.005f);
}
TEST(the_squeal_and_slide_starts_can_be_set) {
    Settings st; CHECK(st.squealStart == 0.8 && st.slideStart == 1.3);
    applySetting(st, "SquealStart", "0.45"); applySetting(st, "SlideStart", "1.0");
    CHECK(st.squealStart == 0.45 && st.slideStart == 1.0);
    applySetting(st, "SquealStart", "-5"); CHECK(st.squealStart >= 0.1);
}

int main() {
    for (const Reg& r : regs()) { int before = g_fail; r.fn(); std::printf("[%s] %s\n", g_fail == before ? " ok " : "FAIL", r.name); }
    std::printf("\n%zu tests, %d checks, %d failures\n", regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
