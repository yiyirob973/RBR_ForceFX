// Tests for how the camera module turns NGP telemetry packets into head-model inputs (camera/camera_fx.hpp, processPacket).
// It needs the Windows and Direct3D 9 headers, so build it with MSVC from an x86 Visual Studio command prompt:
//   cl /std:c++17 /EHsc /MD /O2 camera_packet_test.cpp ..\thirdparty\lib\libminhook.x86.lib && camera_packet_test.exe
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include "../camera/camera_fx.hpp"
#include "tests/mini_test.hpp"

using namespace ffx::cam;

static bool near(float a, float b, float e = 0.01f) { return std::fabs(a - b) <= e; }

// NGP packet offsets (see the header comment of camera_fx.hpp)
enum { oSteer = 24, oBrake = 32, oGear = 44, oSpeed = 60, oYaw = 108, oSurge = 112, oSway = 116, oHeave = 120, oSusp = 152, oSuspStride = 128, oStrut = 16 };

struct Pkt {
    std::vector<char> b;
    explicit Pkt(int n = 664) : b((size_t)n, 0) {}
    Pkt& f(int off, float v) { std::memcpy(&b[(size_t)off], &v, 4); return *this; }
    Pkt& i(int off, int v) { std::memcpy(&b[(size_t)off], &v, 4); return *this; }
    Pkt& struts(float v) { for (int w = 0; w < 4; ++w) f(oSusp + w * oSuspStride + oStrut, v); return *this; }
    void send() const { processPacket(b.data(), (int)b.size()); }
};

static void fresh() {                                   // a clean module, with the stage start long ago
    g_ps = PktState();
    g_cfg = Cfg();
    g_in.surge = 0; g_in.sway = 0; g_in.heave = 0; g_in.heaveHp = 0; g_in.yawN = 0; g_in.steerN = 0; g_in.speed = 0; g_in.brake = 0; g_in.carRoll = 0;
    g_evCrashSurge = 0; g_evCrashSway = 0; g_evLand = 0; g_shiftImpulse = 0; g_airborne = false; g_gear = 0;
    g_stageStartTick = 0;
}

TEST(packets_that_are_too_short_are_ignored) {
    fresh();
    Pkt p(100); p.f(oSpeed, 40.0f); p.send();
    CHECK(g_in.speed.load() == 0.0f);
    Pkt q(664); q.f(oSpeed, 40.0f); q.send();
    CHECK(near(g_in.speed.load(), 40.0f));
}
TEST(speed_and_brake_are_read_and_the_brake_is_kept_between_0_and_1) {
    fresh();
    Pkt p; p.f(oSpeed, 33.3f).f(oBrake, 1.7f); p.send();
    CHECK(near(g_in.speed.load(), 33.3f) && near(g_in.brake.load(), 1.0f));
    Pkt q; q.f(oSpeed, 10.0f).f(oBrake, -0.5f); q.send();
    CHECK(near(g_in.brake.load(), 0.0f));
}
TEST(a_spike_is_clamped_and_the_dead_zone_is_taken_off) {
    fresh();
    Pkt p; p.f(oSurge, 10.0f); p.send();                       // clamped to AccelClamp (2.5), then 0.06 dead zone removed
    CHECK(near(g_in.surge.load(), 2.44f));
    fresh();
    Pkt q; q.f(oSurge, -10.0f); q.send();
    CHECK(near(g_in.surge.load(), -2.44f));
}
TEST(small_accelerations_inside_the_dead_zone_do_nothing) {
    fresh();
    Pkt p; p.f(oSurge, 0.05f).f(oSway, -0.05f).f(oHeave, 0.02f); p.send();
    CHECK(g_in.surge.load() == 0.0f && g_in.sway.load() == 0.0f && g_in.heave.load() == 0.0f);
}
TEST(garbage_values_are_treated_as_zero) {
    fresh();
    Pkt p; p.f(oSurge, NAN).f(oSway, INFINITY).f(oSpeed, 1.0e9f).f(oYaw, -INFINITY); p.send();
    CHECK(g_in.surge.load() == 0.0f && g_in.sway.load() == 0.0f && g_in.speed.load() == 0.0f && g_in.yawN.load() == 0.0f);
}
TEST(the_invert_settings_flip_the_signals) {
    fresh();
    g_cfg.invSurge = -1;
    Pkt p; p.f(oSurge, 1.0f); p.send();
    CHECK(g_in.surge.load() < -0.9f);
}
TEST(gear_changes_make_a_shift_kick_up_or_down) {
    fresh();
    Pkt a; a.i(oGear, 2); a.send();
    CHECK(g_shiftImpulse.load() == 0.0f);                       // the first packet has nothing to compare with
    Pkt b; b.i(oGear, 3); b.send();
    CHECK(near(g_shiftImpulse.load(), 1.0f));
    fresh();
    Pkt c; c.i(oGear, 3); c.send();
    Pkt d; d.i(oGear, 2); d.send();
    CHECK(near(g_shiftImpulse.load(), -1.0f));
    fresh();
    Pkt e; e.i(oGear, 0); e.send();                             // neutral to first is not a shift
    Pkt g; g.i(oGear, 1); g.send();
    CHECK(g_shiftImpulse.load() == 0.0f);
}
TEST(a_hard_hit_is_a_crash_once_per_cooldown) {
    fresh();
    Pkt p; p.f(oSpeed, 20.0f).f(oSurge, -4.5f); p.send();
    CHECK(near(g_evCrashSurge.load(), -1.5f));                  // 4.5 g with a 3 g threshold
    p.send();
    CHECK(near(g_evCrashSurge.load(), -1.5f));                  // the same hit a moment later is the same crash
}
TEST(readings_above_the_glitch_limit_and_the_first_seconds_of_a_stage_are_not_crashes) {
    fresh();
    Pkt p; p.f(oSpeed, 20.0f).f(oSurge, 20.0f); p.send();      // above CrashIgnoreAbove
    CHECK(g_evCrashSurge.load() == 0.0f);
    fresh();
    g_stageStartTick = GetTickCount();                          // the stage has just started
    Pkt q; q.f(oSpeed, 20.0f).f(oSurge, -4.5f); q.send();
    CHECK(g_evCrashSurge.load() == 0.0f);
    fresh();
    Pkt r; r.f(oSpeed, 1.0f).f(oSurge, -4.5f); r.send();       // parked
    CHECK(g_evCrashSurge.load() == 0.0f);
}
TEST(a_jump_is_airborne_while_the_struts_are_unloaded_and_a_landing_when_they_load_again) {
    fresh();
    Pkt loaded; loaded.f(oSpeed, 30.0f).struts(1000.0f);
    for (int i = 0; i < 20; ++i) { loaded.send(); std::this_thread::sleep_for(std::chrono::milliseconds(3)); }
    CHECK(!g_airborne.load());
    Pkt air; air.f(oSpeed, 30.0f).struts(0.0f);
    for (int i = 0; i < 60 && !g_airborne.load(); ++i) { air.send(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    CHECK(g_airborne.load());                                    // unloaded for at least AirborneMs
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    Pkt land = loaded; land.f(oHeave, 3.0f);
    land.send();
    CHECK(!g_airborne.load());
    CHECK(g_evLand.load() > 0.0f);                               // strength from the air time and the heave
}
TEST(a_packet_without_suspension_data_never_flags_a_jump) {
    fresh();
    Pkt p(300); p.f(oSpeed, 30.0f);                              // long enough to be read, too short for the struts
    for (int i = 0; i < 40; ++i) { p.send(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    CHECK(!g_airborne.load() && g_evLand.load() == 0.0f);
}

int main() { return mini::RunAll(); }
