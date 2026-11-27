// Tests for the surface classes, the Map= overrides and the class names (surface_feel.hpp). Self-contained:
//   g++ -std=c++17 -O1 -I.. surface_feel_test.cpp -o surface_feel_test && ./surface_feel_test
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "road/surface_feel.hpp"
#include "tests/mini_test.hpp"

using namespace Surf;

static bool near(float a, float b, float e = 1e-4f) { return std::fabs(a - b) <= e; }
static void resetMap() { for (int i = 0; i < 256; ++i) IdMap()[i] = 0; }

// ---------------------------------------------------------------- classes by id
TEST(verified_material_families_keep_their_classes) {
    resetMap();
    CHECK(ClassOfId(37) == C_TARMAC && ClassOfId(87) == C_TARMAC);      // clean and covered tarmac
    CHECK(ClassOfId(1) == C_GRAVEL && ClassOfId(199) == C_GRAVEL);       // gravel, including the BR2 set
    CHECK(ClassOfId(133) == C_GRAVEL);                                   // dirt
    CHECK(ClassOfId(101) == C_SNOW && ClassOfId(88) == C_SNOW);
    CHECK(ClassOfId(117) == C_ICE);
    CHECK(ClassOfId(97) == C_SOFT && ClassOfId(129) == C_SOFT);          // grass, rough
}
TEST(objects_and_out_of_range_ids_stay_unknown) {
    resetMap();
    CHECK(ClassOfId(30) == C_NONE);      // tree
    CHECK(ClassOfId(63) == C_NONE);      // metal pole
    CHECK(ClassOfId(157) == C_SOFT);     // water surface: it has its own ground in road_feel.hpp, the class is only for the status line
    CHECK(ClassOfId(255) == C_NONE);     // spectator
    CHECK(ClassOfId(-1) == C_NONE && ClassOfId(256) == C_NONE && ClassOfId(100000) == C_NONE);
}
// ---------------------------------------------------------------- Map= parsing
TEST(map_assigns_a_class_by_id) {
    resetMap();
    CHECK(ParseIdMap("200:snow,201:soft") == 2);
    CHECK(ClassOfId(200) == C_SNOW && ClassOfId(201) == C_SOFT);
    CHECK(ClassOfId(202) == C_NONE);                                      // neighbours untouched
}
TEST(map_tolerates_spaces_case_and_the_old_names) {
    resetMap();
    CHECK(ParseIdMap(" 210 : Ice , 211:GRAVEL,88:snow,92:none") == 4);
    CHECK(ClassOfId(210) == C_ICE && ClassOfId(211) == C_GRAVEL && ClassOfId(88) == C_SNOW && ClassOfId(92) == C_NONE);
}
TEST(map_can_override_a_built_in_class_and_none_switches_one_off) {
    resetMap();
    CHECK(ClassOfId(69) == C_TARMAC);                                     // Tarmac_Coarse_Clean_Dry
    ParseIdMap("69:gravel");
    CHECK(ClassOfId(69) == C_GRAVEL);                                     // a tarmac id corrected to gravel
    ParseIdMap("69:none");
    CHECK(ClassOfId(69) == C_NONE);
}
TEST(map_ignores_bad_entries_and_keeps_the_good_ones) {
    resetMap();
    const int n = ParseIdMap("abc,300:gravel,-4:soft,220:marble,221 gravel,,222:gravel,:soft,x7:soft,1e2:soft");
    CHECK(n == 1);                                                        // only 222:gravel is valid
    CHECK(ClassOfId(222) == C_GRAVEL);
    CHECK(ClassOfId(0) == C_NONE);                                        // ":soft" and "abc" did not touch id 0
    CHECK(ParseIdMap("") == 0);
}

// ---------------------------------------------------------------- blending
// ---------------------------------------------------------------- ini names
TEST(map_accepts_everyday_names_for_the_types) {
    resetMap();
    CHECK(ParseIdMap("300:gravel,94:asphalt,95:dirt,96:grass,97:road") == 4);   // four valid, 300 is out of range
    CHECK(ClassOfId(94) == C_TARMAC && ClassOfId(95) == C_GRAVEL && ClassOfId(96) == C_SOFT && ClassOfId(97) == C_TARMAC);
}
TEST(class_from_name_rejects_unknown_types) {
    CHECK(ClassFromName("snow") == C_SNOW && ClassFromName("none") == C_NONE);
    CHECK(ClassFromName("marble") == -1 && ClassFromName("") == -1 && ClassFromName("Snow") == -1);   // callers lower-case first
}
TEST(map_reports_every_entry_it_could_not_use) {
    resetMap();
    std::vector<std::string> bad;
    const int n = ParseIdMap("89:gravel, 90:marble, 400:soft, x:soft, 91 soft, 92:soft", &bad);
    CHECK(n == 2);                                                         // 89 and 92
    CHECK(bad.size() == 4);
    bool sawMarble = false, sawRange = false, sawNumber = false, sawFormat = false;
    for (const std::string& b : bad) {
        if (b.find("marble") != std::string::npos) sawMarble = true;
        if (b.find("400") != std::string::npos) sawRange = true;
        if (b.find("x:soft") != std::string::npos) sawNumber = true;
        if (b.find("91 soft") != std::string::npos && b.find("id:type") != std::string::npos) sawFormat = true;
    }
    CHECK(sawMarble && sawRange && sawNumber && sawFormat);
    CHECK(ClassOfId(89) == C_GRAVEL && ClassOfId(92) == C_SOFT && ClassOfId(90) == C_NONE);
}
TEST(map_without_a_report_list_still_works_and_blank_entries_are_not_errors) {
    resetMap();
    std::vector<std::string> bad;
    CHECK(ParseIdMap(" , ,89:gravel,", &bad) == 1);
    CHECK(bad.empty());
    CHECK(ParseIdMap("90:soft") == 1);
}

int main() { return mini::RunAll(); }
