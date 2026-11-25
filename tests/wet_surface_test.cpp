// Tests for what damp and wet ground do to the tyre sounds (wet_surface.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. wet_surface_test.cpp -o wet_surface_test && ./wet_surface_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>
#include "../wet_surface.hpp"

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)
static const float kNaN = std::numeric_limits<float>::quiet_NaN();
using WetSurface::Make; using WetSurface::Params; using WetSurface::Plan;

TEST(dry_ground_is_left_exactly_alone) {
    const Plan p = Make(0, 1.0f, 100.0f, 0.0f, 1.0f, Params());
    CHECK(p.tone == 0.0f && p.gain == 1.0f && p.hiss == 0.0f && p.stones == 1.0f && p.wetStone == 0.0f);
}
TEST(wet_gravel_is_duller_quieter_hissy_and_throws_fewer_stones) {
    const Plan p = Make(2, 0.9f, 80.0f, 0.0f, 1.0f, Params());
    CHECK(p.tone < -0.3f); CHECK(p.gain < 0.85f && p.gain > 0.4f); CHECK(p.hiss > 0.1f); CHECK(p.stones < 0.8f); CHECK(p.wetStone > 0.3f);
}
TEST(damp_is_between_dry_and_wet) {
    const Plan d = Make(1, 0.9f, 80.0f, 0.0f, 1.0f, Params()), w = Make(2, 0.9f, 80.0f, 0.0f, 1.0f, Params());
    CHECK(d.tone < 0.0f && d.tone > w.tone); CHECK(d.gain < 1.0f && d.gain > w.gain); CHECK(d.hiss > 0.0f && d.hiss < w.hiss); CHECK(d.stones > w.stones);
}
TEST(wet_tarmac_gets_the_hiss_but_not_the_dulling) {
    const Plan t = Make(2, 0.0f, 100.0f, 0.0f, 1.0f, Params()), g = Make(2, 0.9f, 100.0f, 0.0f, 1.0f, Params());
    CHECK(t.tone == 0.0f && t.gain == 1.0f && t.stones == 1.0f && t.wetStone == 0.0f);
    CHECK(t.hiss > g.hiss);                                  // hard ground sprays more clearly
}
TEST(the_hiss_follows_the_speed) {
    const Plan slow = Make(2, 0.5f, 5.0f, 0.0f, 1.0f, Params()), mid = Make(2, 0.5f, 50.0f, 0.0f, 1.0f, Params()), fast = Make(2, 0.5f, 120.0f, 0.0f, 1.0f, Params());
    CHECK(slow.hiss == 0.0f); CHECK(mid.hiss > 0.0f && mid.hiss < fast.hiss);
}
TEST(a_real_wet_recording_gets_no_treatment_but_stones_still_thin_out) {
    const Plan n = Make(2, 0.9f, 80.0f, 1.0f, 1.0f, Params());
    CHECK(n.tone == 0.0f && n.gain == 1.0f && n.hiss == 0.0f && n.wetStone == 0.0f);
    CHECK(n.stones < 0.8f);
    const Plan half = Make(2, 0.9f, 80.0f, 0.5f, 1.0f, Params()), none = Make(2, 0.9f, 80.0f, 0.0f, 1.0f, Params());
    CHECK(half.tone > none.tone && half.tone < 0.0f);
}
TEST(the_menu_level_scales_it_and_zero_turns_it_off) {
    const Plan lo = Make(2, 0.9f, 80.0f, 0.0f, 0.5f, Params()), one = Make(2, 0.9f, 80.0f, 0.0f, 1.0f, Params()), hi = Make(2, 0.9f, 80.0f, 0.0f, 2.0f, Params());
    CHECK(lo.tone > one.tone && one.tone >= hi.tone);
    const Plan off = Make(2, 0.9f, 80.0f, 0.0f, 0.0f, Params());
    CHECK(off.tone == 0.0f && off.gain == 1.0f && off.hiss == 0.0f && off.stones == 1.0f);
}
TEST(the_setting_can_turn_it_off_and_each_amount_is_its_own) {
    Params p; p.on = false; CHECK(Make(2, 0.9f, 80.0f, 0.0f, 1.0f, p).tone == 0.0f);
    Params q; q.tone = 0.0f; q.crunch = 0.0f; q.hiss = 0.0f; q.stones = 0.0f;
    const Plan z = Make(2, 0.9f, 80.0f, 0.0f, 1.0f, q);
    CHECK(z.tone == 0.0f && z.gain == 1.0f && z.hiss == 0.0f && z.stones == 1.0f);
}
TEST(nothing_goes_out_of_range_whatever_the_input) {
    Params big; big.tone = 9.0f; big.crunch = 9.0f; big.hiss = 9.0f; big.stones = 9.0f;
    for (int wet = -1; wet <= 3; ++wet) for (float l = -1; l <= 2; l += 0.5f) for (float s = -10; s <= 400; s += 70) for (float lv = -1; lv <= 5; lv += 1.5f) {
        const Plan p = Make(wet, l, s, 0.3f, lv, big);
        CHECK(p.tone >= -1.0f && p.tone <= 0.0f); CHECK(p.gain >= 0.399f && p.gain <= 1.0f); CHECK(p.hiss >= 0.0f && p.hiss <= 1.0f);
        CHECK(p.stones >= 0.199f && p.stones <= 1.0f); CHECK(p.wetStone >= 0.0f && p.wetStone <= 1.0f);
    }
}
TEST(bad_numbers_give_a_finite_plan) {
    const Plan p = Make(2, kNaN, kNaN, kNaN, kNaN, Params());
    CHECK(std::isfinite(p.tone) && std::isfinite(p.gain) && std::isfinite(p.hiss) && std::isfinite(p.stones) && std::isfinite(p.wetStone));
}

int main() {
    for (auto& r : regs()) { int before = g_fail; r.fn(); std::printf("%s %s\n", before == g_fail ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
