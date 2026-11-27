// Tests for the materials.csv overrides reaching the road feel (Road::ApplyMaterialOverrides). Self-contained:
//   g++ -std=c++17 -O1 -I.. material_overrides_test.cpp -o material_overrides_test && ./material_overrides_test
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <string>
#include <vector>
#include "road/materials.hpp"
#include "road/surface_feel.hpp"
#include "road/road_feel.hpp"
#include "tests/mini_test.hpp"

static int apply(const char* csv) {
    const std::string path = "material_overrides_test.csv";
    { std::ofstream f(path); f << csv; }
    ffx::MaterialTable base, ovr;
    CHECK(ovr.loadOverrides(path));
    std::remove(path.c_str());
    return Road::ApplyMaterialOverrides(base, ovr);
}

// Each test works on its own ids, because the table and the class map are process-wide.
TEST(the_wetness_of_a_known_material_is_the_games_until_overridden) {
    CHECK(Road::WetOfId(1) == 0); CHECK(Road::WetOfId(2) == 1); CHECK(Road::WetOfId(3) == 2);
    CHECK(Road::WetOfId(-1) == 0); CHECK(Road::WetOfId(999) == 0);
}
TEST(an_override_that_makes_a_known_material_wetter_changes_its_ground_too) {
    Road::Ground before; CHECK(Road::GroundOfId(9, before));                      // Gravel_Fine_Deep_Dry
    CHECK(Road::WetOfId(9) == 0);
    CHECK(apply("9, 1, 2   ; wet\n") == 1);
    Road::Ground after; CHECK(Road::GroundOfId(9, after));
    CHECK(Road::WetOfId(9) == 2);
    CHECK(after.slick > before.slick);                                            // wet gravel is slicker
    CHECK(std::fabs(after.hard - before.hard) < 1e-6f);                           // and nothing else moved
}
TEST(a_new_id_gets_its_wetness_and_its_ground_comes_from_its_class) {
    Road::Ground g; CHECK(!Road::GroundOfId(200, g));                              // not in the game's table
    CHECK(apply("200, 1, 1\n") == 1);
    CHECK(Road::WetOfId(200) == 1);
    CHECK(!Road::GroundOfId(200, g));                                              // no name to work the ground from: GroundForTyre uses the class
    CHECK(Surf::ClassOfId(200) == Surf::C_GRAVEL);                                 // and the reader's class follows the override (audio class 1)
}
TEST(a_class_override_moves_the_readers_class_unless_map_set_one) {
    Surf::IdMap()[201] = Surf::C_ICE + 1;                                          // [Surface] Map= already says ice
    CHECK(apply("201, 0, 0\n202, 3, 0\n") == 2);
    CHECK(Surf::ClassOfId(201) == Surf::C_ICE);                                    // Map= wins
    CHECK(Surf::ClassOfId(202) == Surf::C_SNOW);
}
TEST(an_override_that_says_not_ground_removes_the_ground) {
    Road::Ground g; CHECK(Road::GroundOfId(13, g));
    CHECK(apply("13, -1, 0\n") == 1);
    CHECK(!Road::GroundOfId(13, g));
}
TEST(an_override_that_changes_nothing_changes_nothing) {
    CHECK(apply("17, 1, 0\n") == 0);                                               // id 17 is Gravel ..._Dry in the built-in table: same class, same wetness
    Road::Ground g; CHECK(Road::GroundOfId(17, g));
}

int main() { return mini::RunAll(); }
