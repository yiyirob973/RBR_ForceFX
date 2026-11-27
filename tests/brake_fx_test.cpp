// Tests for the brake feel (brake_fx.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. brake_fx_test.cpp -o brake_fx_test && ./brake_fx_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <vector>

#include "effects/brake_fx.hpp"
#include "tests/mini_test.hpp"

static const float kNaN = std::numeric_limits<float>::quiet_NaN();

TEST(no_lock_up_leaves_rbrs_force_alone) {
    const float l[4] = {0, 0, 0, 0};
    CHECK(BrakeFx::LockMul(BrakeFx::FrontLock(l), 1.0f) == 1.0f);
}
TEST(front_lock_up_lightens_the_force_in_proportion) {
    const float half[4] = {0.5f, 0.5f, 0, 0}, full[4] = {1, 1, 0, 0};
    const float m1 = BrakeFx::LockMul(BrakeFx::FrontLock(half), 1.0f), m2 = BrakeFx::LockMul(BrakeFx::FrontLock(full), 1.0f);
    CHECK(m1 < 1.0f && m2 < m1); CHECK(std::fabs(m2 - 0.5f) < 1e-5f);
}
TEST(only_the_front_tyres_count_for_the_steering) {
    const float rear[4] = {0, 0, 1, 1};
    CHECK(BrakeFx::FrontLock(rear) == 0.0f); CHECK(BrakeFx::LockMul(BrakeFx::FrontLock(rear), 2.0f) == 1.0f);
}
TEST(lock_lightening_is_bounded_whatever_the_gain) {
    CHECK(BrakeFx::LockMul(1.0f, 2.0f) >= 1.0f - BrakeFx::kLockCutMax - 1e-6f);
    CHECK(BrakeFx::LockMul(1.0f, 100.0f) == BrakeFx::LockMul(1.0f, 2.0f));
    CHECK(BrakeFx::LockMul(5.0f, 1.0f) == BrakeFx::LockMul(1.0f, 1.0f));
}
TEST(gain_zero_and_bad_numbers_turn_it_off) {
    CHECK(BrakeFx::LockMul(1.0f, 0.0f) == 1.0f); CHECK(BrakeFx::LockMul(1.0f, -3.0f) == 1.0f);
    CHECK(BrakeFx::LockMul(1.0f, kNaN) == 1.0f); CHECK(BrakeFx::LockMul(kNaN, 1.0f) == 1.0f);
    CHECK(BrakeFx::HeavyMul(1.0f, 0.0f) == 1.0f); CHECK(BrakeFx::HeavyMul(kNaN, 1.0f) == 1.0f);
}
TEST(heavy_braking_adds_force_and_is_bounded) {
    CHECK(BrakeFx::HeavyMul(0.0f, 1.0f) == 1.0f);
    CHECK(std::fabs(BrakeFx::HeavyMul(1.0f, 1.0f) - 1.3f) < 1e-5f);
    CHECK(BrakeFx::HeavyMul(1.0f, 2.0f) <= 1.0f + BrakeFx::kHeavyAddMax + 1e-6f);
    CHECK(BrakeFx::HeavyMul(0.5f, 1.0f) < BrakeFx::HeavyMul(1.0f, 1.0f));
}
TEST(lock_up_pitch_follows_the_ground) {
    const float tarmac = BrakeFx::LockHz(1.0f, 0.0f, 0.0f), gravel = BrakeFx::LockHz(0.4f, 0.6f, 0.0f), snow = BrakeFx::LockHz(0.15f, 0.9f, 0.0f);
    CHECK(tarmac > gravel && gravel > snow);
    CHECK(std::fabs(gravel - 38.0f) < 5.0f);            // where the old fixed pitch sat
}
TEST(lock_up_pitch_stays_in_range_for_any_input) {
    for (float h = -1; h <= 2; h += 0.5f) for (float l = -1; l <= 2; l += 0.5f) for (float r = -3; r <= 3; r += 0.5f) {
        const float f = BrakeFx::LockHz(h, l, r); CHECK(f >= 25.0f && f <= 70.0f);
    }
    CHECK(BrakeFx::LockHz(kNaN, kNaN, kNaN) >= 25.0f);
}
TEST(judder_strength_matches_the_old_formula) {
    // pow(brake,1.2) * (0.3 + 0.7 dive) * 0.6 + lock, clamped
    CHECK(std::fabs(BrakeFx::JudderAmp(1.0f, 1.0f, 0.0f) - 0.6f) < 1e-5f);
    CHECK(std::fabs(BrakeFx::JudderAmp(1.0f, 0.0f, 0.0f) - 0.18f) < 1e-5f);
    CHECK(BrakeFx::JudderAmp(0.0f, 0.0f, 0.0f) == 0.0f);
    CHECK(BrakeFx::JudderAmp(1.0f, 1.0f, 1.0f) == 1.0f);
    CHECK(BrakeFx::JudderAmp(kNaN, kNaN, kNaN) == 0.0f);
}
TEST(judder_pitch_is_the_wheel_rate_until_lock_up_then_the_scrub) {
    CHECK(std::fabs(BrakeFx::JudderHz(0.0f, 10.0f, 1, 0, 0, 1.0f) - 20.0f) < 1e-4f);
    CHECK(BrakeFx::JudderHz(0.0f, 1.0f, 1, 0, 0, 1.0f) == 12.0f);          // never below 12
    CHECK(BrakeFx::JudderHz(0.0f, 100.0f, 1, 0, 0, 1.0f) == 60.0f);        // never above 60
    CHECK(std::fabs(BrakeFx::JudderHz(0.5f, 10.0f, 1, 0, 0, 1.0f) - BrakeFx::LockHz(1, 0, 0)) < 1e-4f);
    CHECK(std::fabs(BrakeFx::JudderHz(0.0f, 10.0f, 1, 0, 0, 1.5f) - 30.0f) < 1e-4f);   // the pitch setting scales it
    CHECK(BrakeFx::JudderHz(0.0f, kNaN, 1, 0, 0, kNaN) == 12.0f);
}

int main() { return mini::RunAll(); }

