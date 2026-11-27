// Tests for the shared packet layout (telemetry_layout.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. telemetry_layout_test.cpp -o telemetry_layout_test && ./telemetry_layout_test
#include <cstdio>
#include <functional>
#include <vector>
#include "telemetry/telemetry_layout.hpp"
#include "tests/mini_test.hpp"

TEST(the_defaults_are_the_ngp_7_4_778_layout) {
    Telem::Layout l;
    CHECK(l.steering == 24 && l.throttle == 28 && l.brake == 32 && l.handbrake == 36 && l.gear == 44 && l.carIndex == 56);
    CHECK(l.speed == 60 && l.carRoll == 76 && l.velSurge == 88 && l.velSway == 92 && l.velHeave == 96 && l.yawRate == 108);
    CHECK(l.accSurge == 112 && l.accSway == 116 && l.accHeave == 120 && l.rpm == 136 && l.rpmMax == -1);
    CHECK(l.suspBase[0] == 152 && l.suspBase[1] == 280 && l.suspBase[2] == 408 && l.suspBase[3] == 536 && l.suspStride == 128);
    CHECK(l.strutForce == 16 && l.pistonVelocity == 28 && l.currentSegment == 60);
}
TEST(setting_the_suspension_moves_all_four_blocks) {
    Telem::Layout l; l.SetSuspension(200, 100);
    CHECK(l.suspBase[0] == 200 && l.suspBase[1] == 300 && l.suspBase[2] == 400 && l.suspBase[3] == 500 && l.suspStride == 100);
}
TEST(the_needed_length_covers_the_last_field_that_is_used) {
    Telem::Layout l; CHECK(l.NeededLength() == 664);                  // the default packet minimum is the larger
    l.SetSuspension(152, 400); CHECK(l.NeededLength() >= 152 + 3 * 400 + 60 + 4);
    Telem::Layout m; m.minLength = 100; CHECK(m.NeededLength() >= 536 + 60 + 4);
}
TEST(a_field_moved_far_out_grows_the_needed_length) {
    Telem::Layout l; l.rpm = 2000; CHECK(l.NeededLength() == 2004);
}

int main() { return mini::RunAll(); }
