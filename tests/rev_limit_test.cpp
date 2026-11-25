// Tests for the learned rev limit (rev_limit.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. rev_limit_test.cpp -o rev_limit_test && ./rev_limit_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>

#include "../rev_limit.hpp"
#include <cstring>

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)

static const float kNaN = std::numeric_limits<float>::quiet_NaN();
using RevLimit::Learner;

TEST(it_learns_the_highest_rpm_the_engine_holds) {
    Learner l; for (int i = 0; i < 100; ++i) l.Update(5000.0f + 30.0f * i, 0.01f);
    CHECK(l.Peak() > 7500.0f && l.Peak() <= 7970.0f);
}
TEST(a_single_spike_does_not_raise_it) {
    Learner l; for (int i = 0; i < 100; ++i) l.Update(6500.0f, 0.01f);
    CHECK(l.Peak() == 6500.0f);
    l.Update(15000.0f, 0.01f);                            // one bad frame
    for (int i = 0; i < 100; ++i) l.Update(6500.0f, 0.01f);
    CHECK(l.Peak() == 6500.0f);
}
TEST(a_new_limit_held_for_a_moment_is_accepted) {
    Learner l; for (int i = 0; i < 100; ++i) l.Update(6500.0f, 0.01f);
    for (int i = 0; i < 40; ++i) l.Update(7200.0f, 0.01f);
    CHECK(l.Peak() == 7200.0f);
}
TEST(a_short_burst_above_the_old_peak_is_ignored_and_restarts_the_count) {
    Learner l; for (int i = 0; i < 100; ++i) l.Update(6500.0f, 0.01f);
    for (int k = 0; k < 10; ++k) { for (int i = 0; i < 15; ++i) l.Update(7000.0f, 0.01f); l.Update(6000.0f, 0.01f); }   // never 0.25 s in a row
    CHECK(l.Peak() == 6500.0f);
}
TEST(bad_input_is_ignored_and_reset_works) {
    Learner l; l.Update(kNaN, 0.01f); l.Update(-5.0f, 0.01f); l.Update(6000.0f, kNaN); CHECK(std::isfinite(l.Peak()));
    for (int i = 0; i < 100; ++i) l.Update(6000.0f, 0.01f);
    CHECK(l.Peak() == 6000.0f); l.Reset(); CHECK(l.Peak() == 0.0f);
}
TEST(candidates_are_floats_in_the_rev_range_above_the_current_rpm) {
    uint8_t pk[64] = {0}; auto put = [&](int off, float v) { std::memcpy(pk + off, &v, 4); };
    put(0, 1.0f); put(4, 6200.0f); put(8, 7000.0f); put(12, 800.0f); put(16, 25000.0f); put(20, 7000.0f); put(24, kNaN);
    int out[8]; const int k = RevLimit::FindCandidates(pk, 64, 20, 1.0f, 5000.0f, out, 8);
    CHECK(k == 2); CHECK(out[0] == 4 && out[1] == 8);                 // 20 is the rpm field itself; 12 is too low; 16 too high
}
TEST(candidates_respect_the_scale_and_the_output_size) {
    uint8_t pk[40] = {0}; for (int off = 0; off < 40; off += 4) { float v = 700.0f; std::memcpy(pk + off, &v, 4); }
    int out[3]; CHECK(RevLimit::FindCandidates(pk, 40, -1, 10.0f, 5000.0f, out, 3) == 3);   // 700 x 10 = 7000
    CHECK(RevLimit::FindCandidates(pk, 40, -1, 1.0f, 0.0f, out, 3) == 0);
    CHECK(RevLimit::FindCandidates(nullptr, 40, -1, 1.0f, 0.0f, out, 3) == 0); CHECK(RevLimit::FindCandidates(pk, 40, -1, 10.0f, 0.0f, out, 0) == 0);
}

int main() {
    for (auto& r : regs()) { int before = g_fail; r.fn(); std::printf("%s %s\n", before == g_fail ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

