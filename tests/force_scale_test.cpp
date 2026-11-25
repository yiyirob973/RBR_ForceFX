// Tests for how the force scalings are combined and bounded (force_scale.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. force_scale_test.cpp -o force_scale_test && ./force_scale_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>

#include "../force_scale.hpp"

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)

static const float kNaN = std::numeric_limits<float>::quiet_NaN();
using namespace ForceScale;

TEST(no_scaling_is_exactly_one) { Parts p; CHECK(Product(p) == 1.0f); Limiter l; CHECK(l.Step(Product(p), 0.01f, 3.0f) == 1.0f); }
TEST(the_parts_multiply) { Parts p; p.landing = 2.0f; p.waterHeavy = 1.5f; p.accelLight = 0.8f; CHECK(std::fabs(Product(p) - 2.4f) < 1e-5f); }
TEST(a_broken_part_counts_as_no_change) { Parts p; p.landing = kNaN; p.crash = -4.0f; p.waterKick = 0.0f; p.brakeLock = 0.5f; CHECK(Product(p) == 0.5f); }
TEST(the_cap_stops_a_stack_of_boosts) {
    Parts p; p.landing = 5.0f; p.waterKick = 2.0f; p.waterHeavy = 3.0f;          // 30x
    Limiter l; float m = 1.0f; for (int i = 0; i < 100; ++i) m = l.Step(Product(p), 0.01f, 3.0f);
    CHECK(std::fabs(m - 3.0f) < 1e-5f);
}
TEST(the_cap_itself_is_never_above_the_hard_limit) {
    Limiter l; float m = 1.0f; for (int i = 0; i < 200; ++i) m = l.Step(50.0f, 0.01f, 100.0f);
    CHECK(m <= kHardMulMax + 1e-6f);
    CHECK(Cap(kNaN) == kDefaultCap); CHECK(Cap(0.2f) == 1.0f);
}
TEST(the_force_is_never_scaled_below_the_floor) {
    Limiter l; float m = 1.0f; for (int i = 0; i < 100; ++i) m = l.Step(0.01f, 0.01f, 3.0f);
    CHECK(std::fabs(m - kMinMul) < 1e-5f);
}
TEST(it_cannot_jump_in_one_step_but_gets_there_quickly) {
    Limiter l; const float a = l.Step(4.0f, 0.01f, 4.0f);
    CHECK(a <= 1.0f + kRiseRate * 0.01f + 1e-5f); CHECK(a < 4.0f);
    float m = a; int steps = 1; while (m < 3.99f && steps < 100) { m = l.Step(4.0f, 0.01f, 4.0f); ++steps; }
    CHECK(steps <= 3);                                                           // 20-30 ms
}
TEST(a_landing_at_the_default_strength_is_not_held_back) {
    Limiter l; const float m = l.Step(2.1f, 0.01f, 3.0f);                       // landing 1.1 boost at full severity
    CHECK(std::fabs(m - 2.1f) < 1e-5f);
}
TEST(falling_is_limited_too_but_quicker_than_any_real_decay) {
    Limiter l; for (int i = 0; i < 10; ++i) l.Step(3.0f, 0.01f, 3.0f);
    const float m = l.Step(1.0f, 0.01f, 3.0f); CHECK(m >= 3.0f - kFallRate * 0.01f - 1e-5f); CHECK(m > 1.0f);
}
TEST(a_lower_cap_takes_hold_at_once) {
    Limiter l; for (int i = 0; i < 50; ++i) l.Step(3.0f, 0.01f, 3.0f);
    CHECK(l.Step(3.0f, 0.01f, 1.5f) <= 1.5f + 1e-6f);
}
TEST(bad_dt_and_target_stay_finite) {
    Limiter l; CHECK(std::isfinite(l.Step(kNaN, kNaN, kNaN))); CHECK(std::isfinite(l.Step(2.0f, -1.0f, 3.0f))); CHECK(std::isfinite(l.Step(2.0f, 1000.0f, 3.0f)));
}
TEST(extra_push_is_capped) { CHECK(Add(0.2f) == 0.2f); CHECK(Add(5.0f) == kHardAddMax); CHECK(Add(-1.0f) == 0.0f); CHECK(Add(kNaN) == 0.0f); }

int main() {
    for (auto& r : regs()) { int before = g_fail; r.fn(); std::printf("%s %s\n", before == g_fail ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

