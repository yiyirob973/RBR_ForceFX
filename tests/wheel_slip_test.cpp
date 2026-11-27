// Tests for the wheel slip estimator (wheel_slip.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. wheel_slip_test.cpp -o wheel_slip_test && ./wheel_slip_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <vector>
#include "telemetry/wheel_slip.hpp"
#include "tests/mini_test.hpp"

static bool within(double a, double b, double e) { return std::fabs(a - b) <= e; }

// A wheel on a road: packets leave NGP at `rate` Hz and reach us with some timing jitter. The wheel turns at
// ground speed x (1 + slip(t)) and its segment counter steps once per eighth of a revolution.
struct Sim {
    double rate = 100, jitterMs = 0, circ = 1.95, ground = 20;
    bool useSteps = true;
    std::function<double(double)> slip = [](double) { return 0.0; };       // wheel slip over time
    std::function<bool(double)> coasting = [](double t) { return t < 6.0; };
    std::function<double(double)> groundAt;                                  // optional: ground speed over time
    unsigned seed = 1;
    // Result of each packet.
    struct Step { double t; double ratio; bool valid; double margin; };
    std::vector<Step> out;
    Slip::WheelSlip ws{1.0 / 1.95};
    Slip::Timebase tb; Slip::RevCounter rc;

    void run(double seconds, double startPhase = 0.37, uint32_t steps0 = 1000) {
        std::mt19937 rng(seed); std::normal_distribution<double> jit(0.0, jitterMs / 1000.0);
        double rev = startPhase, lastArr = -1;
        const int n = (int)(seconds * rate);
        unsigned gen = ~0u;
        for (int i = 0; i < n; ++i) {
            const double t = i / rate, g = groundAt ? groundAt(t) : ground;
            if (i > 0) rev += g * (1.0 + slip(t)) / circ / rate;
            const uint32_t seg = (uint32_t)std::floor(rev * 8.0) & 7u;
            double arr = t + (jitterMs > 0 ? jit(rng) : 0.0);
            arr = std::max(arr, lastArr + 1e-6); lastArr = arr;
            const Slip::Timebase::Stamp st = tb.stamp(steps0 + (uint32_t)i, useSteps, arr);
            if (tb.generation() != gen) { rc.reset(); gen = tb.generation(); }
            rc.push(seg, st.tick, st.secPerTick);
            ws.update(rc, st.secPerTick, st.timeErr, g, coasting(t), 1.0 / rate);
            out.push_back({t, ws.ratio, ws.valid, ws.margin});
        }
    }
    double maxAbs(double from, double to) const { double m = 0; for (const Step& s : out) if (s.t >= from && s.t < to) m = std::max(m, std::fabs(s.ratio)); return m; }
    double extreme(double from, double to, int sign) const { double m = 0; for (const Step& s : out) if (s.t >= from && s.t < to) m = sign > 0 ? std::max(m, s.ratio) : std::min(m, s.ratio); return m; }
};

// The way the plugin measured it before: average the steps per packet over the packet's arrival time.
static double oldMaxSlip(double rate, double ground, double circ, double startPhase) {
    double rev = startPhase, revRate = 0, rollRatio = 0, slipS = 0, worst = 0; bool have = false; int last = (int)std::floor(rev * 8) & 7;
    const int n = (int)(30 * rate);
    for (int i = 1; i < n; ++i) {
        const double dt = 1.0 / rate;
        rev += ground / circ * dt;
        const int s8 = (int)std::floor(rev * 8) & 7, d = (s8 + 8 - last) & 7; last = s8;
        const double x = d / 8.0 / dt;
        if (!have) { revRate = x; rollRatio = x / ground; have = true; }
        revRate += (1 - std::exp(-dt / 0.12)) * (x - revRate);
        rollRatio += (1 - std::exp(-dt / 3.0)) * (revRate / ground - rollRatio);
        if (rollRatio < 1e-3) continue;
        slipS += (1 - std::exp(-dt / 0.25)) * (revRate / (rollRatio * ground) - 1.0 - slipS);
        if (i > 5 * rate) worst = std::max(worst, std::fabs(slipS));
    }
    return worst;
}

// ---------------------------------------------------------------- a rolling wheel is never read as slipping
TEST(the_old_way_read_big_slip_on_a_rolling_wheel) {
    CHECK(oldMaxSlip(100, 8, 1.95, 0.37) > 0.15);       // 8 m/s at 100 packets/s: a third of a step per packet
    CHECK(oldMaxSlip(180, 20, 1.95, 0.37) > 0.10);
}
TEST(a_rolling_wheel_reads_as_no_slip_at_any_speed_packet_rate_and_jitter) {
    for (double rate : {30.0, 60.0, 100.0, 180.0, 250.0})
        for (double jitter : {0.0, 1.0, 4.0, 8.0})
            for (double v : {5.0, 8.0, 12.0, 20.0, 30.0, 45.0})
                for (bool steps : {true, false})
                    for (double phase : {0.0, 0.37, 0.8}) {
                        Sim s; s.rate = rate; s.jitterMs = jitter; s.ground = v; s.useSteps = steps; s.seed = (unsigned)(rate + v + phase * 100);
                        s.run(20.0, phase);
                        const double m = s.maxAbs(6.0, 20.0);
                        // With the step counter the clock is clean, so up to a few ms of jitter changes nothing. Without it, the
                        // dead band widens with the jitter it sees, so a wheel that is not slipping is still never read as slipping.
                        const double allowed = steps ? (jitter <= 4.0 ? 0.0 : 0.04) : (jitter <= 1.0 ? 0.0 : (jitter <= 4.0 ? 0.04 : 0.2));
                        if (m > allowed) std::printf("  rate %.0f jitter %.0f v %.0f steps %d phase %.2f: max slip %.3f\n", rate, jitter, v, steps, phase, m);
                        CHECK(m <= allowed);
                    }
}
TEST(a_rolling_wheel_is_never_read_as_slip_even_with_the_wrong_tyre_size_assumed_before_learning) {
    for (double circ : {1.7, 2.1, 2.3}) {
        Sim s; s.circ = circ; s.ground = 25; s.run(20.0);
        CHECK(s.maxAbs(0.0, 20.0) < 0.2);                 // the first moments use the guess: only a clear excess counts
        CHECK(s.maxAbs(8.0, 20.0) <= 0.0);
    }
}
TEST(slip_is_zero_not_noise_at_a_walking_pace) {
    Sim s; s.ground = 2.0; s.run(10.0);
    CHECK(s.maxAbs(0.0, 10.0) == 0.0);
    CHECK(!s.out.back().valid);                           // below 3 m/s the ratio means nothing
}

// ---------------------------------------------------------------- real slip is found
TEST(wheelspin_is_found_quickly_and_goes_away_when_it_stops) {
    Sim s; s.ground = 25; s.slip = [](double t) { return t >= 10.0 && t < 12.0 ? 0.3 : 0.0; }; s.run(16.0);
    CHECK(s.extreme(10.0, 10.4, +1) > 0.15);              // within 0.4 s
    CHECK(s.extreme(10.6, 12.0, +1) > 0.2 && s.extreme(10.0, 12.0, +1) < 0.32);
    CHECK(s.maxAbs(12.6, 16.0) <= 0.0);
    CHECK(s.extreme(6.0, 10.0, -1) == 0.0);
}
TEST(lock_up_is_found_quickly_and_a_fully_locked_wheel_reads_as_almost_minus_one) {
    for (double lock : {-0.3, -0.6, -1.0}) {
        Sim s; s.ground = 25; s.slip = [lock](double t) { return t >= 10.0 && t < 12.0 ? lock : 0.0; }; s.run(16.0);
        CHECK(s.extreme(10.0, 10.4, -1) < 0.45 * lock);
        CHECK(s.extreme(10.6, 12.0, -1) < 0.8 * lock + 0.05);
        CHECK(s.maxAbs(12.6, 16.0) <= 0.0);
    }
}
TEST(slip_only_counts_past_its_uncertainty_so_a_small_true_slip_is_not_reported) {
    Sim s; s.ground = 25; s.slip = [](double t) { return t >= 10.0 ? 0.02 : 0.0; }; s.run(14.0);
    CHECK(s.maxAbs(10.5, 14.0) <= 0.0);
}
TEST(a_clear_slip_is_reported_close_to_its_real_size) {
    Sim s; s.ground = 25; s.slip = [](double t) { return t >= 10.0 ? 0.2 : 0.0; }; s.run(14.0);
    const Sim::Step& e = s.out.back();
    CHECK(e.ratio > 0.1 && e.ratio <= 0.2 + 1e-9);
}
TEST(wheelspin_while_the_car_is_barely_moving_is_seen_through_the_wheel_speed) {
    Sim s; s.ground = 2.0; s.slip = [](double) { return 4.0; }; s.run(4.0);     // wheel at 10 m/s, car at 2
    CHECK(!s.out.back().valid && s.ws.wheelValid);
    CHECK(s.ws.wheelMs > 8.0);
}

TEST(a_counter_that_never_moves_while_the_car_drives_says_nothing_about_slip) {
    Sim s; s.ground = 25; s.slip = [](double) { return -1.0; };         // the wheel never turns: no step is ever counted
    s.run(10.0);
    CHECK(!s.ws.turning() && !s.out.back().valid && s.maxAbs(0.0, 10.0) == 0.0);
}
TEST(a_wheel_that_locks_after_turning_is_still_a_lock_up) {
    Sim s; s.ground = 25; s.slip = [](double t) { return t >= 5.0 ? -1.0 : 0.0; };
    s.run(9.0);
    CHECK(s.ws.turning() && s.out.back().valid && s.extreme(6.0, 9.0, -1) < -0.7);
}

// ---------------------------------------------------------------- learning the tyre size
TEST(the_tyre_size_is_learned_from_coasting_within_two_percent) {
    for (double circ : {1.75, 1.95, 2.15}) {
        Sim s; s.circ = circ; s.ground = 25; s.run(12.0);
        CHECK(s.ws.learned());
        CHECK(within(s.ws.revPerMetre(), 1.0 / circ, 0.02 / circ));
    }
}
TEST(a_coast_with_a_locked_wheel_is_not_learned) {
    Sim s; s.ground = 25; s.coasting = [](double) { return true; };
    s.slip = [](double t) { return t >= 8.0 ? -0.5 : 0.0; };
    s.run(14.0);
    CHECK(within(s.ws.revPerMetre(), 1.0 / 1.95, 0.02 / 1.95));
    CHECK(s.extreme(9.0, 14.0, -1) < -0.3);               // and the lock-up is still seen
}
TEST(nothing_is_learned_without_coasting_but_the_prior_still_works_for_clear_slip) {
    Sim s; s.ground = 25; s.coasting = [](double) { return false; }; s.slip = [](double t) { return t >= 6.0 ? 0.5 : 0.0; }; s.run(10.0);
    CHECK(!s.ws.learned());
    CHECK(s.extreme(7.0, 10.0, +1) > 0.2);
    CHECK(s.maxAbs(0.0, 6.0) <= 0.0);
}
TEST(the_tyre_size_follows_a_slow_change_like_a_growing_tyre_at_speed) {
    Sim s; s.coasting = [](double) { return true; };
    s.groundAt = [](double t) { return 15.0 + 1.5 * t; };
    // the tyre grows by 3 percent while the speed climbs: revolutions per metre fall
    s.slip = [](double t) { return 1.0 / (1.0 + 0.03 * std::min(1.0, t / 20.0)) - 1.0; };
    s.run(20.0);
    CHECK(s.maxAbs(8.0, 20.0) <= 0.0);
}

// ---------------------------------------------------------------- the counter itself
TEST(the_segment_counter_wraps_and_a_start_at_segment_seven_works) {
    Slip::RevCounter rc;
    for (int i = 0; i < 200; ++i) rc.push((uint32_t)((7 + i) & 7), i * 0.01, 1.0);     // one step per packet, from 7
    const Slip::RevCounter::Est e = rc.estimate(32, 0.5, 1.0);
    CHECK(e.valid && within(e.revPerSec, 100.0 / 8.0, 0.5));
}
TEST(a_repeated_packet_is_ignored) {
    Slip::RevCounter a, b;
    for (int i = 0; i < 100; ++i) { a.push((uint32_t)(i & 7), i * 0.01, 1.0); b.push((uint32_t)(i & 7), i * 0.01, 1.0); b.push((uint32_t)(i & 7), i * 0.01, 1.0); }
    CHECK(within(a.estimate(32, 0.5, 1.0).revPerSec, b.estimate(32, 0.5, 1.0).revPerSec, 1e-9));
}
TEST(a_gap_in_the_packets_restarts_the_count_instead_of_guessing) {
    Slip::RevCounter rc;
    for (int i = 0; i < 100; ++i) rc.push((uint32_t)(i & 7), i * 0.01, 1.0);
    rc.push(3, 1.0 + 0.4, 1.0);                           // 0.4 s of packets never arrived: the wheel may have turned many times
    CHECK(!rc.estimate(8, 0.15, 1.0).valid);
}
TEST(a_stopped_wheel_has_zero_speed_with_an_error_of_one_step) {
    Slip::RevCounter rc;
    for (int i = 0; i < 100; ++i) rc.push(5, i * 0.01, 1.0);
    const Slip::RevCounter::Est e = rc.estimate(32, 0.5, 1.0);
    CHECK(e.valid && e.revPerSec == 0.0 && within(e.rateErr, 1.0 / (8.0 * e.seconds), 1e-9));
}
TEST(estimates_use_the_shortest_window_with_enough_steps_and_never_a_longer_one_than_allowed) {
    Slip::RevCounter rc;
    for (int i = 0; i < 300; ++i) rc.push((uint32_t)((i * 2) & 7), i * 0.005, 1.0);    // 2 steps per 5 ms = 50 rev/s
    const Slip::RevCounter::Est e = rc.estimate(32, 0.5, 1.0);
    CHECK(e.segments >= 32 && e.seconds < 0.2);
    const Slip::RevCounter::Est cap = rc.estimate(1e6, 0.25, 1.0);
    CHECK(cap.seconds <= 0.25 + 1e-9);
}

// ---------------------------------------------------------------- time base
TEST(the_step_counter_gives_a_clean_clock_even_when_packets_arrive_unevenly) {
    Slip::Timebase tb; std::mt19937 rng(7); std::normal_distribution<double> jit(0.0, 0.004);
    double last = -1; Slip::Timebase::Stamp st{0, 1, 0};
    for (int i = 0; i < 1000; ++i) { double arr = std::max(i * 0.01 + jit(rng), last + 1e-6); last = arr; st = tb.stamp(500 + i, true, arr); }
    CHECK(tb.usingSteps());
    CHECK(within(st.secPerTick, 0.01, 0.0002));
    CHECK(st.tick == 500.0 + 999.0);
}
TEST(without_a_usable_step_counter_arrival_time_is_used) {
    Slip::Timebase a, b;
    Slip::Timebase::Stamp sa{0, 1, 0}, sb{0, 1, 0};
    for (int i = 0; i < 500; ++i) { sa = a.stamp(0, false, i * 0.01); sb = b.stamp(42, true, i * 0.01); }   // no counter / a counter that never moves
    CHECK(!a.usingSteps() && !b.usingSteps());
    CHECK(sa.secPerTick == 1.0 && sb.secPerTick == 1.0 && within(sa.tick, 4.99, 1e-9) && within(sb.tick, 4.99, 1e-9));
}
TEST(a_counter_that_restarts_or_jumps_starts_a_new_time_axis) {
    Slip::Timebase tb;
    for (int i = 0; i < 300; ++i) tb.stamp(1000 + i, true, i * 0.01);
    const unsigned g = tb.generation();
    tb.stamp(5, true, 3.01);                              // a new stage: the counter starts again
    CHECK(tb.generation() != g && !tb.usingSteps());
}

// ---------------------------------------------------------------- ground speed
TEST(ground_speed_comes_from_the_velocity_vector) {
    Slip::GroundSpeed g;
    CHECK(within(g.update(30.0, 24.0, 7.0, 0.5, 0.0, 0.01), 25.0, 1e-9) && g.fromVelocity());
    CHECK(within(g.update(30.0, 0.0, -10.0, 0.5, 0.0, 0.01), 10.0, 1e-9));
}
TEST(a_dead_velocity_vector_falls_back_to_the_speed_field_scaled_by_how_it_compared_and_recovers) {
    Slip::GroundSpeed g; double v = 0;
    for (int i = 0; i < 600; ++i) v = g.update(40.0, 25.0, 0.0, 0.0, 0.0, 0.01);    // the field reads 1.6x the ground speed while the vector works
    CHECK(g.fromVelocity() && within(v, 25.0, 1e-9));
    for (int i = 0; i < 300; ++i) v = g.update(48.0, 0.0, 0.0, 0.0, 0.0, 0.01);      // the vector dies while the car speeds up: 3 s
    CHECK(!g.fromVelocity() && within(v, 48.0 / 1.6, 0.5));
    v = g.update(48.0, 30.0, 0.0, 0.0, 0.0, 0.01);
    CHECK(g.fromVelocity() && v == 30.0);
}
TEST(a_dead_vector_with_no_known_scale_gives_no_ground_speed_rather_than_a_wrong_one) {
    Slip::GroundSpeed g; double v = 1;
    for (int i = 0; i < 300; ++i) v = g.update(40.0, 0.0, 0.0, 0.0, 0.0, 0.01);
    CHECK(!g.fromVelocity() && v == 0.0);
}
TEST(standing_still_is_not_a_dead_vector) {
    Slip::GroundSpeed g; double v = 1;
    for (int i = 0; i < 500; ++i) v = g.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.01);
    CHECK(g.fromVelocity() && v == 0.0);
}

// ---- the options the audio module needs from the shared ground-speed logic
TEST(mode_velocity_always_takes_the_vector_even_when_it_looks_dead) {
    Slip::GroundSpeed g; g.setMode(1); double v = 1;
    for (int i = 0; i < 400; ++i) v = g.update(40.0, 0.0, 0.0, 0.0, 0.0, 0.01);
    CHECK(g.fromVelocity() && v == 0.0);
}
TEST(mode_field_always_takes_the_speed_field_as_it_is) {
    Slip::GroundSpeed g; g.setMode(2); double v = 0;
    for (int i = 0; i < 100; ++i) v = g.update(40.0, 25.0, 0.0, 0.0, 0.0, 0.01);
    CHECK(!g.fromVelocity() && v == 40.0);
}
TEST(a_bad_mode_is_auto) {
    Slip::GroundSpeed g; g.setMode(9); double v = 0;
    for (int i = 0; i < 100; ++i) v = g.update(40.0, 25.0, 0.0, 0.0, 0.0, 0.01);
    CHECK(g.fromVelocity() && v == 25.0);
}
TEST(the_field_scale_is_learned_from_a_steady_roll_and_clamped) {
    Slip::GroundSpeed g; double v = 0;
    for (int i = 0; i < 3000; ++i) v = g.update(40.0, 25.0, 0.0, 0.0, 0.0, 0.01);       // field 1.6 x the vector
    CHECK(std::fabs(g.drivenMs(40.0) - 25.0) < 0.3);
    Slip::GroundSpeed h;
    for (int i = 0; i < 3000; ++i) h.update(100.0, 20.0, 0.0, 0.0, 0.0, 0.01);          // 5x: clamped to 2.5
    CHECK(std::fabs(h.drivenMs(100.0) - 40.0) < 0.5);
    (void)v;
}
TEST(before_anything_is_learned_the_initial_scale_is_used) {
    Slip::GroundSpeed g; g.setInitialFieldScale(2.0); g.update(40.0, 25.0, 0.0, 0.5, 0.0, 0.01);
    CHECK(std::fabs(g.drivenMs(40.0) - 20.0) < 1e-9);
    g.setInitialFieldScale(-1.0); CHECK(std::fabs(g.drivenMs(40.0) - 40.0 / 1.609) < 1e-9);   // a bad value falls back to the default
}
TEST(with_a_dead_vector_the_driven_speed_is_the_field_itself) {
    Slip::GroundSpeed g;
    for (int i = 0; i < 300; ++i) g.update(40.0, 0.0, 0.0, 0.0, 0.0, 0.01);
    CHECK(!g.fromVelocity() && g.drivenMs(40.0) == 40.0);
}
TEST(a_dead_vector_after_a_learned_scale_gives_the_scaled_field) {
    Slip::GroundSpeed g; for (int i = 0; i < 3000; ++i) g.update(40.0, 25.0, 0.0, 0.0, 0.0, 0.01);
    double v = 0; for (int i = 0; i < 300; ++i) v = g.update(40.0, 0.0, 0.0, 0.0, 0.0, 0.01);
    CHECK(!g.fromVelocity() && std::fabs(v - 25.0) < 0.3);
}

int main() { return mini::RunAll(); }
