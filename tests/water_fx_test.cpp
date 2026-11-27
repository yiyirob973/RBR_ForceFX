// Tests for the water feel (water_fx.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. water_fx_test.cpp -o water_fx_test && ./water_fx_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include "effects/water_fx.hpp"
#include "tests/mini_test.hpp"

// Runs the tracker for `secs` seconds at 100 Hz with the given tyres in water; returns the hardest output seen of each kind.
struct Run { float maxSplash = 0, maxSlosh = 0, maxKick = 0, maxHeavy = 0; int events = 0; float lastEntry = 0; Water::Out last; };
static Run run(Water::Tracker& t, Water::In in, float secs, Run r = Run()) {
    for (int i = 0; i < (int)(secs * 100); ++i) {
        const Water::Out o = t.Update(in, 0.01f);
        r.maxSplash = std::max(r.maxSplash, std::max(o.splash[0], o.splash[1]));
        r.maxSlosh = std::max(r.maxSlosh, std::max(o.slosh[0], o.slosh[1]));
        r.maxKick = std::max(r.maxKick, o.kick); r.maxHeavy = std::max(r.maxHeavy, o.heavy);
        if (o.entry > 0) { ++r.events; r.lastEntry = o.entry; }
        r.last = o;
    }
    return r;
}
static Water::In dry(float speed) { Water::In in; in.speed = speed; return in; }
static Water::In wet(float speed, bool l, bool r, bool rl, bool rr) { Water::In in; in.speed = speed; in.depth[0] = l; in.depth[1] = r; in.depth[2] = rl; in.depth[3] = rr; return in; }

TEST(only_water_surface_and_listed_ids_are_water) {
    Water::ClearIds();
    CHECK(Water::DepthOfId(157) == 1.0f);
    CHECK(Water::DepthOfId(37) == 0.0f && Water::DepthOfId(39) == 0.0f && Water::DepthOfId(3) == 0.0f);   // wet tarmac and wet gravel are not standing water
    CHECK(Water::DepthOfId(-1) == 0.0f && Water::DepthOfId(256) == 0.0f && Water::DepthOfId(0) == 0.0f);
    CHECK(Water::ParseIds("200, 201,210") == 3);
    CHECK(Water::DepthOfId(200) == 1.0f && Water::DepthOfId(201) == 1.0f && Water::DepthOfId(210) == 1.0f && Water::DepthOfId(202) == 0.0f);
    Water::ClearIds();
    CHECK(Water::DepthOfId(200) == 0.0f);
}

TEST(id_list_skips_bad_entries_and_says_so) {
    Water::ClearIds();
    std::vector<std::string> rej;
    CHECK(Water::ParseIds("12,abc,300,,-4,99", &rej) == 2);
    CHECK(rej.size() == 3);
    CHECK(Water::DepthOfId(12) == 1.0f && Water::DepthOfId(99) == 1.0f && Water::DepthOfId(44) == 0.0f);
    CHECK(Water::ParseIds("") == 0 && Water::ParseIds(",,,") == 0);
    Water::ClearIds();
}

TEST(speed_factor_grows_with_speed_and_is_bounded) {
    CHECK(Water::SpeedFactor(0) == 0.0f && Water::SpeedFactor(3) == 0.0f && Water::SpeedFactor(-5) == 0.0f);
    float prev = 0;
    for (float v = 4; v <= 60; v += 2) { const float f = Water::SpeedFactor(v); CHECK(f >= prev && f <= 1.0f); prev = f; }
    CHECK(Water::SpeedFactor(32) > 0.99f);
    CHECK(Water::SpeedFactor(10) < 0.25f && Water::SpeedFactor(20) > 0.35f && Water::SpeedFactor(20) < 0.65f);
    CHECK(Water::SpeedFactor(std::numeric_limits<float>::quiet_NaN()) == 0.0f);
}

TEST(no_water_means_nothing) {
    Water::Tracker t;
    const Run r = run(t, dry(35), 3.0f);
    CHECK(r.maxSplash == 0 && r.maxSlosh == 0 && r.maxKick == 0 && r.maxHeavy == 0 && r.events == 0);
}

TEST(a_fast_entry_makes_one_hard_splash_that_dies_away) {
    Water::Tracker t;
    Run r = run(t, dry(30), 0.5f);
    r = run(t, wet(30, true, true, false, false), 0.02f, r);        // both front tyres go in at 108 km/h
    CHECK(r.events == 1 && r.lastEntry > 0.7f);
    CHECK(r.maxSplash > 0.7f && r.maxKick > 0.7f);
    r = run(t, wet(30, true, true, true, true), 1.5f, r);
    CHECK(r.last.splash[0] < 0.05f && r.last.splash[1] < 0.05f && r.last.kick < 0.05f);   // gone after a second and a half
    CHECK(r.events == 1 || r.events == 2);                           // the rear tyres going in make at most their own, smaller one
}

TEST(a_slow_entry_is_gentle_and_a_standing_start_in_water_is_silent) {
    Water::Tracker t;
    Run slow = run(t, dry(6), 0.3f);
    slow = run(t, wet(6, true, true, false, false), 0.3f, slow);
    Water::Tracker t2;
    Run fast = run(t2, dry(30), 0.3f);
    fast = run(t2, wet(30, true, true, false, false), 0.3f, fast);
    CHECK(slow.maxSplash < 0.2f && fast.maxSplash > 3.0f * slow.maxSplash);
    Water::Tracker t3;
    const Run still = run(t3, wet(0, true, true, true, true), 2.0f);
    CHECK(still.maxSplash == 0 && still.maxSlosh == 0 && still.events == 0);
}

TEST(starting_in_the_water_is_not_an_entry) {
    Water::Tracker t;
    const Run r = run(t, wet(30, true, true, true, true), 0.5f);     // the very first reading already shows water (stage start, plugin loaded late)
    CHECK(r.events == 0 && r.maxSplash == 0);
    CHECK(r.maxSlosh > 0.0f);                                        // but wading is felt
}

TEST(splash_is_felt_on_the_side_that_got_wet) {
    Water::Tracker t;
    Run r = run(t, dry(30), 0.3f);
    for (int i = 0; i < 3; ++i) { const Water::Out o = t.Update(wet(30, true, false, false, false), 0.01f); r.last = o; r.maxSplash = std::max(r.maxSplash, o.splash[0]); }
    CHECK(r.last.splash[0] > 0.7f && r.last.splash[1] == 0.0f);
    CHECK(r.last.entryTyre == -1 || r.last.entryTyre == 0);
    Water::Tracker t2;
    run(t2, dry(30), 0.3f);
    const Water::Out o2 = t2.Update(wet(30, false, true, false, false), 0.01f);
    CHECK(o2.splash[1] > 0.7f && o2.splash[0] == 0.0f && o2.entryTyre == 1);
}

TEST(leaving_the_water_gives_a_smaller_echo) {
    Water::Tracker t;
    Run in = run(t, dry(30), 0.3f);
    in = run(t, wet(30, true, true, true, true), 2.0f, in);
    const Run out = run(t, dry(30), 0.02f);
    Water::Tracker t2;
    Run in2 = run(t2, dry(30), 0.3f);
    const Run in3 = run(t2, wet(30, true, true, true, true), 0.02f, in2);
    CHECK(out.maxSplash > 0.1f && out.maxSplash < 0.5f * in3.maxSplash);
}

TEST(a_flickering_puddle_edge_does_not_machine_gun) {
    Water::Tracker t;
    Run r = run(t, dry(30), 0.3f);
    for (int i = 0; i < 40; ++i) r = run(t, wet(30, i % 2 == 0, i % 2 == 0, false, false), 0.02f, r);   // in, out, in, out every 20 ms
    CHECK(r.events <= 5);
}

TEST(landing_in_water_splashes_by_how_hard_it_landed) {
    Water::Tracker t;
    run(t, wet(8, true, true, true, true), 0.5f);
    Water::In in = wet(8, true, true, true, true); in.landed = true; in.landSev = 1.0f;
    const Water::Out o = t.Update(in, 0.01f);
    CHECK(o.entry > 0.5f && o.splash[0] > 0.5f);
    Water::Tracker t2;
    run(t2, wet(8, true, true, true, true), 0.5f);
    Water::In soft = wet(8, true, true, true, true); soft.landed = true; soft.landSev = 0.0f;
    CHECK(t2.Update(soft, 0.01f).entry < o.entry);
    Water::Tracker t3;
    run(t3, dry(8), 0.5f);
    Water::In onLand = dry(8); onLand.landed = true; onLand.landSev = 1.0f;
    CHECK(t3.Update(onLand, 0.01f).entry == 0.0f);                    // a landing on dry ground is not a splash
}

TEST(wading_grows_with_speed_and_with_tyres_in_water) {
    Water::Tracker slow, fast, one, four;
    const Run a = run(slow, wet(5, true, true, true, true), 2.0f);
    const Run b = run(fast, wet(30, true, true, true, true), 2.0f);
    const Run c = run(one, wet(30, true, false, false, false), 2.0f);
    const Run d = run(four, wet(30, true, true, true, true), 2.0f);
    CHECK(a.last.slosh[0] > 0.0f && b.last.slosh[0] > 3.0f * a.last.slosh[0]);
    CHECK(c.last.slosh[0] > 0.0f && c.last.slosh[1] < 0.05f * c.last.slosh[0] + 1e-6f);
    CHECK(c.last.slosh[0] < d.last.slosh[0]);
    CHECK(b.last.slosh[0] <= Water::kSloshMax + 1e-6f && b.last.slosh[1] <= Water::kSloshMax + 1e-6f);
}

TEST(wading_stops_after_the_tyres_leave_and_in_the_air) {
    Water::Tracker t;
    Run r = run(t, wet(30, true, true, true, true), 2.0f);
    CHECK(r.last.slosh[0] > 0.05f);
    r = run(t, dry(30), 2.0f, r);
    CHECK(r.last.slosh[0] < 0.01f && r.last.slosh[1] < 0.01f);
    Water::Tracker t2;
    Water::In air = wet(30, true, true, true, true); air.airborne = 1.0f;
    const Run a = run(t2, air, 1.0f);
    CHECK(a.last.slosh[0] == 0.0f && a.last.heavy == 0.0f);
}

TEST(steering_is_heavy_in_water_from_a_crawl_up_and_only_for_the_front_tyres) {
    Water::Tracker still, crawl, slow, fast, rear, one;
    CHECK(run(still, wet(0, true, true, true, true), 2.0f).last.heavy == 0.0f);          // stopped: nothing to push against
    const Run c = run(crawl, wet(2.5f, true, true, false, false), 2.0f);
    const Run s = run(slow, wet(10, true, true, false, false), 2.0f);
    const Run f = run(fast, wet(40, true, true, false, false), 2.0f);
    CHECK(c.last.heavy > 0.05f && c.last.heavy < s.last.heavy);
    CHECK(s.last.heavy > 0.8f);                                                           // already nearly full at 36 km/h, a normal ford speed
    CHECK(f.last.heavy > 0.97f && f.last.heavy <= 1.0f);
    const Run r = run(rear, wet(30, false, false, true, true), 2.0f);
    CHECK(r.last.heavy == 0.0f);                                                          // the rear tyres do not steer
    const Run o = run(one, wet(30, true, false, false, false), 2.0f);
    CHECK(o.last.heavy > 0.4f && o.last.heavy < 0.6f);                                    // one front tyre: half
}

TEST(heaviness_eases_off_after_the_water_and_not_in_the_air) {
    Water::Tracker t;
    Run r = run(t, wet(30, true, true, true, true), 2.0f);
    CHECK(r.last.heavy > 0.95f);
    r = run(t, dry(30), 2.0f, r);
    CHECK(r.last.heavy < 0.01f);
    Water::Tracker t2;
    Water::In air = wet(30, true, true, true, true); air.airborne = 1.0f;
    CHECK(run(t2, air, 1.0f).last.heavy == 0.0f);
}

TEST(multipliers_stay_inside_their_limits) {
    Water::Out o; o.kick = 1.0f; o.heavy = 1.0f;
    CHECK(std::fabs(Water::KickMul(o, 1.0f) - 1.45f) < 1e-4f);
    CHECK(Water::KickMul(o, 0.0f) == 1.0f && Water::KickMul(o, -3.0f) == 1.0f);
    CHECK(Water::KickMul(o, 99.0f) <= 1.0f + Water::kKick * 2.0f + 1e-4f);
    CHECK(Water::HeavyMul(o, 0.0f) == 1.0f && Water::HeavyMul(o, -1.0f) == 1.0f);        // off: exactly RBR's force
    CHECK(std::fabs(Water::HeavyMul(o, 0.6f) - 1.6f) < 1e-4f);
    CHECK(Water::HeavyMul(o, 99.0f) <= 3.0f + 1e-4f);                                      // never more than three times
    o.heavy = 0.5f;
    CHECK(std::fabs(Water::HeavyMul(o, 0.6f) - 1.3f) < 1e-4f);
    o.heavy = 0.0f; o.kick = 0.0f;
    CHECK(Water::HeavyMul(o, 2.0f) == 1.0f && Water::KickMul(o, 2.0f) == 1.0f);
    o.kick = std::numeric_limits<float>::quiet_NaN(); o.heavy = std::numeric_limits<float>::quiet_NaN();
    CHECK(Water::KickMul(o, 1.0f) == 1.0f && Water::HeavyMul(o, 0.5f) == 1.0f);
}

TEST(band_magnitudes_are_bounded_finite_and_low_heavy) {
    const float one[2][3] = {{1, 1, 1}, {1, 1, 1}};
    Water::Out o; o.splash[0] = 1.0f; o.splash[1] = 1.0f;
    float m[3];
    Water::BandMagnitudes(o, 1.0f, one, m);
    for (int b = 0; b < 3; ++b) CHECK(std::isfinite(m[b]) && m[b] >= 0.0f && m[b] <= Water::kRef + 1e-3f);
    CHECK(m[0] > m[1] && m[1] > m[2]);                                // a thump: the low band leads
    Water::BandMagnitudes(o, 0.0f, one, m);
    CHECK(m[0] == 0.0f && m[1] == 0.0f && m[2] == 0.0f);
    Water::BandMagnitudes(o, 50.0f, one, m);
    for (int b = 0; b < 3; ++b) CHECK(m[b] <= 2.0f * Water::kRef + 1e-3f);   // the gain is capped
    Water::Out none;
    Water::BandMagnitudes(none, 1.0f, one, m);
    CHECK(m[0] == 0.0f && m[1] == 0.0f && m[2] == 0.0f);
}

TEST(two_sides_add_as_energy_so_they_cannot_cancel) {
    const float one[2][3] = {{1, 1, 1}, {1, 1, 1}};
    Water::Out left, both; left.splash[0] = 0.5f; both.splash[0] = 0.5f; both.splash[1] = 0.5f;
    float a[3], b[3];
    Water::BandMagnitudes(left, 1.0f, one, a); Water::BandMagnitudes(both, 1.0f, one, b);
    CHECK(b[0] > a[0] && std::fabs(b[0] - a[0] * std::sqrt(2.0f)) < 1e-2f);
}

TEST(wading_alone_stays_well_under_a_full_splash) {
    const float one[2][3] = {{1, 1, 1}, {1, 1, 1}};
    Water::Tracker t;
    const Run r = run(t, wet(40, true, true, true, true), 3.0f);
    float m[3];
    Water::BandMagnitudes(r.last, 1.0f, one, m);
    for (int b = 0; b < 3; ++b) CHECK(m[b] < 0.45f * Water::kRef);
    CHECK(m[0] > 0.0f);
}

TEST(bad_input_never_makes_nan) {
    Water::Tracker t;
    Water::In in = wet(std::numeric_limits<float>::quiet_NaN(), true, true, true, true);
    in.maxSpeed = std::numeric_limits<float>::infinity(); in.airborne = std::numeric_limits<float>::quiet_NaN();
    for (int i = 0; i < 50; ++i) {
        const Water::Out o = t.Update(in, i % 2 ? -1.0f : std::numeric_limits<float>::quiet_NaN());
        CHECK(std::isfinite(o.splash[0]) && std::isfinite(o.slosh[0]) && std::isfinite(o.kick) && std::isfinite(o.heavy) && std::isfinite(o.entry));
    }
}

TEST(reset_forgets_everything) {
    Water::Tracker t;
    run(t, dry(30), 0.3f);
    run(t, wet(30, true, true, true, true), 1.0f);
    t.Reset();
    const Run r = run(t, wet(30, true, true, true, true), 0.5f);      // looks like the first reading again: no entry event
    CHECK(r.events == 0);
}

int main() { return mini::RunAll(); }
