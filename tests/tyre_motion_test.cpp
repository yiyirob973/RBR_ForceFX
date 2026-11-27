// Tests of the tyre motion logic: drivetrain, per-tyre ground speed, grip, wheelspin vs slide.
//   g++ -std=c++17 -O1 -I.. tyre_motion_test.cpp -o tyre_motion_test && ./tyre_motion_test
#include <cstdio>
#include <functional>
#include <vector>
#include "telemetry/tyre_motion.hpp"
#include "tests/mini_test.hpp"

using namespace Tyre;

TEST(the_default_drivetrain_is_rear_wheel_drive) { CHECK(kDefaultDrivetrain == RWD); }
TEST(only_the_driven_tyres_are_driven) {
    for (int w = 0; w < 4; ++w) {
        CHECK(Driven(RWD, w) == (w >= 2));
        CHECK(Driven(FWD, w) == (w < 2));
        CHECK(Driven(AWD, w));
    }
}
TEST(drivetrain_names_are_read_loosely) {
    CHECK(ParseDrivetrain("RWD", FWD) == RWD && ParseDrivetrain(" fwd ", RWD) == FWD && ParseDrivetrain("AWD", RWD) == AWD);
    CHECK(ParseDrivetrain("4WD", RWD) == AWD && ParseDrivetrain("Rear-Wheel Drive", FWD) == RWD && ParseDrivetrain("front wheel drive", RWD) == FWD);
    CHECK(ParseDrivetrain("", AWD) == AWD && ParseDrivetrain("what", FWD) == FWD);
}

static TyreIn spinning(bool driven, double ratio) {
    TyreIn t; t.driven = driven; t.ratioValid = true; t.ratio = ratio; t.throttle = 0.9; return t;
}
TEST(wheelspin_needs_a_driven_tyre_and_the_throttle) {
    CHECK(Classify(spinning(true, 0.3)).spin > 1.5);
    CHECK(Classify(spinning(false, 0.3)).spin == 0.0);
    TyreIn off = spinning(true, 0.3); off.throttle = 0.0; CHECK(Classify(off).spin == 0.0);
    TyreIn braking = spinning(true, 0.3); braking.brake = 0.9; CHECK(Classify(braking).spin == 0.0);
}
TEST(a_slide_is_sideways_slip_or_lock_up_and_wheelspin_is_not_a_slide) {
    CHECK(Classify(spinning(true, 0.5)).slide == 0.0);
    TyreIn lat; lat.angleValid = true; lat.angle = 0.35; lat.peakAngleDeg = 8;
    CHECK(Classify(lat).slide > 1.5 && Classify(lat).spin == 0.0);
    TyreIn lock; lock.ratioValid = true; lock.ratio = -0.4; lock.brake = 0.9; lock.driven = false;
    CHECK(Classify(lock).slide > 1.5 && Classify(lock).spin == 0.0);
    lock.brake = 0.0; CHECK(Classify(lock).slide == 0.0);               // slow with no brake: not a lock-up
    lock.handbrake = 1.0; CHECK(Classify(lock).slide > 1.5);            // handbrake locks the rears
}
TEST(slides_are_measured_against_the_grip_of_the_ground) {
    TyreIn t; t.angleValid = true; t.angle = 0.2;
    double a, b, c, d; PeaksFromGround(0, 0, a, b); PeaksFromGround(0.9, 0.1, c, d);
    CHECK(c > a && d > b);                                              // loose ground: wider peak
    t.peakAngleDeg = b; double onTarmac = Classify(t).lateral;
    t.peakAngleDeg = d; double onGravel = Classify(t).lateral;
    CHECK(onTarmac > onGravel);
    double sr, sa; PeaksFromGround(0.0, 1.0, sr, sa); CHECK(sr > a && sa > b);   // ice: gentler than tarmac
}
TEST(the_grip_gate_only_softens_when_the_car_is_far_from_its_limit) {
    CHECK(GripGate(-1.0) == 1.0 && GripGate(1.0) > 0.99 && GripGate(0.1) < 0.4 && GripGate(0.1) >= 0.3);
}
TEST(spin_and_slide_amounts_start_at_their_thresholds) {
    CHECK(SpinAmount(kSpinStart) == 0.0 && SpinAmount(kSpinStart + 0.45) > 0.99);
    CHECK(SlideAmount(kSlideStart) == 0.0 && SlideAmount(kSlideStart + 0.8) > 0.99);
}

// ----------------------------------------------------------------------- kinematics
static Input rolling(double v, double sway) { Input i; i.velSurge = v; i.velSway = sway; return i; }

TEST(a_tyre_in_a_sideways_slide_rolls_at_the_speed_along_it_not_the_full_speed) {
    Kinematics k; Params p;
    for (int i = 0; i < 200; ++i) k.update(rolling(25, 12), 0.01, std::sqrt(25.0 * 25 + 12 * 12), p, true);
    for (int w = 0; w < 4; ++w) CHECK(std::fabs(k.vLong[w] - 25.0) < 1e-6);   // not 27.7
    CHECK(k.valid && k.slipAngle[0] > 0.4);
}
TEST(without_enough_speed_the_ground_speed_is_passed_through) {
    Kinematics k; Params p; k.update(rolling(4, 0), 0.01, 4.0, p, true);
    for (int w = 0; w < 4; ++w) CHECK(k.vLong[w] == 4.0);
    CHECK(!k.valid);
}
TEST(cornering_with_known_units_gives_each_tyre_its_own_speed) {
    Kinematics k; Params p; p.accelToMs2 = 1.0;
    Input in = rolling(20, 0); in.yawRate = 0.5; in.accSway = 10.0; in.steering = 0.1;
    in.strut[0] = 4000; in.strut[2] = 4000; in.strut[1] = 2000; in.strut[3] = 2000;
    for (int i = 0; i < 600; ++i) k.update(in, 0.01, 20.0, p, true);
    CHECK(k.unitsConfirmed() && k.valid);
    // the tyres on the outside of the turn cover more ground per second than the ones on the inside
    double lo = 1e9, hi = 0; for (int w = 0; w < 4; ++w) { lo = std::min(lo, k.vLong[w]); hi = std::max(hi, k.vLong[w]); }
    CHECK(hi - lo > 0.3 && hi - lo < 1.5);
}

// ----------------------------------------------------------------------- grip
TEST(grip_is_learned_per_ground_and_tells_how_much_is_in_use) {
    GripLearner g; double gravel[8] = {0, 1, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 1000; ++i) g.update(i % 100 < 50 ? 9.0 : 3.0, gravel, true, 0.02);   // pushes to 9 m/s2 now and then
    CHECK(g.learned(1) && !g.learned(0));
    CHECK(g.peak(1) > 6.0 && g.peak(1) < 9.5);
    for (int i = 0; i < 60; ++i) g.update(1.0, gravel, true, 0.02);
    CHECK(g.use() >= 0.0 && g.use() < 0.4);                    // cruising: little of the grip in use
    for (int i = 0; i < 100; ++i) g.update(9.0, gravel, true, 0.02);
    CHECK(g.use() > 0.85);
}
TEST(grip_is_not_known_before_it_is_learned_or_when_acceleration_is_unusable) {
    GripLearner g; double tar[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    g.update(8.0, tar, true, 0.02); CHECK(g.use() < 0.0);
    for (int i = 0; i < 500; ++i) g.update(8.0, tar, false, 0.02);
    CHECK(g.use() < 0.0 && !g.learned(0));
}
TEST(a_crash_spike_does_not_become_the_grip_limit) {
    GripLearner g; double tar[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 1000; ++i) g.update(8.0, tar, true, 0.02);
    for (int i = 0; i < 5; ++i) g.update(400.0, tar, true, 0.02);        // 0.1 s of impact
    CHECK(g.peak(0) < 11.0);
}
TEST(mixed_ground_neither_teaches_nor_judges_a_class) {
    GripLearner g; double mix[8] = {0.5, 0.5, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 1000; ++i) g.update(9.0, mix, true, 0.02);
    CHECK(!g.learned(0) && !g.learned(1));
}

int main() { return mini::RunAll(); }
