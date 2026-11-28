// Tests for the landing detector (landing.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. landing_test.cpp -o landing_test && ./landing_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>
#include "effects/landing.hpp"
#include "tests/mini_test.hpp"

// A car driving at 20 m/s whose total strut load is replayed at 100 Hz.
struct Rig {
    Landing::Detector d; Landing::Out last; int fires = 0, finishes = 0;
    void run(float tot, float peak, float bump, float secs, float speed = 20.0f) {
        for (int i = 0; i < (int)(secs * 100 + 0.5f); ++i) {
            Landing::In in; in.totRatio = tot; in.peakRw = peak; in.bumpEnv = bump; in.speed = speed;
            last = d.Step(in, 0.01f);
            fires += last.fired; finishes += last.finished;
        }
    }
};

TEST(driving_on_the_ground_never_fires) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 5.0f);
    CHECK(r.fires == 0); CHECK(r.last.evtSeq == 0); CHECK(r.last.evtSev == 0.0f);
}
TEST(a_big_jump_fires_once_with_high_severity) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 1.0f);
    r.run(0.02f, 0.05f, 0.0f, 1.0f);                  // a second in the air
    r.run(2.6f, 3.2f, 0.9f, 0.3f);                    // hard landing
    r.run(1.0f, 1.1f, 0.0f, 1.0f);
    CHECK(r.fires == 1); CHECK(r.last.evtSeq == 1); CHECK(r.last.evtSev > 0.9f); CHECK(r.finishes == 1);
}
TEST(a_short_hop_is_felt_but_never_stronger_than_a_long_jump) {
    Rig hop; hop.run(1.0f, 1.1f, 0.0f, 1.0f);
    hop.run(0.05f, 0.1f, 0.0f, 0.2f);                 // 0.2 s off the ground
    hop.run(1.1f, 1.2f, 0.0f, 0.3f);                  // lands softly
    Rig big; big.run(1.0f, 1.1f, 0.0f, 1.0f);
    big.run(0.05f, 0.1f, 0.0f, 1.2f);
    big.run(1.1f, 1.2f, 0.0f, 0.3f);                  // same landing, much more air
    CHECK(hop.fires == 1); CHECK(hop.last.evtSev >= 0.55f);
    CHECK(hop.last.evtSev <= big.last.evtSev + 1e-6f);
}
TEST(a_bump_without_air_time_never_fires) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 1.0f);
    for (int i = 0; i < 20; ++i) { r.run(0.1f, 0.2f, 0.0f, 0.1f); r.run(2.0f, 2.5f, 0.8f, 0.1f); }   // washboard: unloads briefly each time
    CHECK(r.fires == 0); CHECK(r.last.evtSeq == 0);
}
TEST(a_stationary_car_is_never_airborne) {
    Rig r; r.run(0.0f, 0.0f, 0.0f, 2.0f, 0.5f); r.run(2.0f, 2.0f, 0.5f, 0.5f, 0.5f);
    CHECK(r.fires == 0);
}
TEST(severity_is_refined_upward_during_the_window_only) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 1.0f); r.run(0.02f, 0.05f, 0.0f, 0.3f);
    Landing::In in; in.speed = 20.0f; in.totRatio = 1.0f; in.peakRw = 1.0f;
    Landing::Out first = r.d.Step(in, 0.01f);
    in.peakRw = 3.0f; in.bumpEnv = 1.0f; in.totRatio = 2.5f;
    Landing::Out second = r.d.Step(in, 0.01f);
    CHECK(second.evtSev >= first.evtSev);
    r.run(2.5f, 3.5f, 1.0f, 0.3f);                    // window has closed
    const float after = r.last.evtSev;
    r.run(2.5f, 5.0f, 1.0f, 0.3f);
    CHECK(r.last.evtSev == after);
}
TEST(two_separate_jumps_are_two_events) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 1.0f);
    for (int k = 0; k < 2; ++k) { r.run(0.02f, 0.05f, 0.0f, 0.5f); r.run(2.0f, 2.5f, 0.7f, 0.3f); r.run(1.0f, 1.1f, 0.0f, 1.0f); }
    CHECK(r.fires == 2); CHECK(r.last.evtSeq == 2); CHECK(r.finishes == 2);
}
TEST(finished_reports_what_the_landing_was_made_of) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 1.0f); r.run(0.02f, 0.05f, 0.0f, 0.6f);
    Landing::Out fin; bool got = false;
    for (int i = 0; i < 30; ++i) { Landing::In in; in.speed = 20; in.totRatio = 2.0f; in.peakRw = 2.2f; in.bumpEnv = 0.5f; Landing::Out o = r.d.Step(in, 0.01f); if (o.finished) { fin = o; got = true; } }
    CHECK(got); CHECK(std::fabs(fin.air - 0.6f) < 0.03f); CHECK(fin.peak > 2.0f);
}
TEST(sinceEvent_resets_in_the_air_and_after_a_landing_then_grows) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 3.0f);
    CHECK(r.last.sinceEvent > 2.5f);
    r.run(0.02f, 0.05f, 0.0f, 0.5f); CHECK(r.last.sinceEvent < 0.02f);
    r.run(2.0f, 2.5f, 0.7f, 0.2f); r.run(1.0f, 1.1f, 0.0f, 1.0f);
    CHECK(r.last.sinceEvent > 0.8f && r.last.sinceEvent < 1.1f);
}
TEST(a_new_stage_forgets_the_air_time_but_keeps_the_event_count) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 1.0f); r.run(0.02f, 0.05f, 0.0f, 0.5f); r.run(2.0f, 2.5f, 0.7f, 0.3f); r.run(1.0f, 1.1f, 0.0f, 0.5f);
    CHECK(r.last.evtSeq == 1);
    r.run(0.02f, 0.05f, 0.0f, 0.5f);                  // in the air when the stage restarts
    r.d.NewStage();
    r.run(1.0f, 1.1f, 0.0f, 0.5f);                    // lands normally: no jump counted for the air before the restart
    CHECK(r.fires == 1); CHECK(r.last.evtSeq == 1);
}
TEST(bad_numbers_do_not_poison_it) {
    Rig r; r.run(1.0f, 1.1f, 0.0f, 0.5f);
    Landing::In in; in.speed = 20; in.totRatio = std::numeric_limits<float>::quiet_NaN(); in.peakRw = std::numeric_limits<float>::infinity();
    Landing::Out o = r.d.Step(in, std::numeric_limits<float>::quiet_NaN());
    CHECK(std::isfinite(o.evtSev)); CHECK(o.fired == false);
    r.run(1.0f, 1.1f, 0.0f, 0.5f); CHECK(std::isfinite(r.last.evtSev));
}
TEST(severity_function_is_monotonic_and_bounded) {
    float prev = 0; for (int i = 0; i <= 20; ++i) { const float s = Landing::Detector::Severity(1.0f + 0.2f * i, 0.0f, 0.0f, 0.0f); CHECK(s >= prev - 1e-6f); CHECK(s >= 0.0f && s <= 1.0f); prev = s; }
    CHECK(Landing::Detector::Severity(0, 0.5f, 0, 0) >= 0.99f);
}

int main() { return mini::RunAll(); }
