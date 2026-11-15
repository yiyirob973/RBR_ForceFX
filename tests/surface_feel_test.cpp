// Tests for the surface classes and their feel values (surface_feel.hpp). Self-contained:
//   g++ -std=c++17 -O1 surface_feel_test.cpp -o surface_feel_test && ./surface_feel_test
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "../surface_feel.hpp"

using namespace Surf;

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)
static bool near(float a, float b, float e = 1e-4f) { return std::fabs(a - b) <= e; }
static void resetMap() { for (int i = 0; i < 256; ++i) IdMap()[i] = 0; }
static void weights(float w[C_COUNT], int a, float wa, int b = -1, float wb = 0.0f) {
    for (int i = 0; i < C_COUNT; ++i) w[i] = 0;
    w[a] = wa;
    if (b >= 0) w[b] = wb;
}

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
TEST(objects_water_and_out_of_range_ids_stay_unknown) {
    resetMap();
    CHECK(ClassOfId(30) == C_NONE);      // tree
    CHECK(ClassOfId(63) == C_NONE);      // metal pole
    CHECK(ClassOfId(157) == C_NONE);     // water surface
    CHECK(ClassOfId(255) == C_NONE);     // spectator
    CHECK(ClassOfId(-1) == C_NONE && ClassOfId(256) == C_NONE && ClassOfId(100000) == C_NONE);
}
TEST(no_id_maps_to_cobble_or_wood_until_the_user_assigns_it) {
    resetMap();
    for (int id = -2; id < 300; ++id) CHECK(ClassOfId(id) != C_COBBLE && ClassOfId(id) != C_WOOD);
}

// ---------------------------------------------------------------- Map= parsing
TEST(map_assigns_cobble_and_wood_by_id) {
    resetMap();
    CHECK(ParseIdMap("200:cobble,201:wood") == 2);
    CHECK(ClassOfId(200) == C_COBBLE && ClassOfId(201) == C_WOOD);
    CHECK(ClassOfId(202) == C_NONE);                                      // neighbours untouched
}
TEST(map_tolerates_spaces_case_and_the_old_names) {
    resetMap();
    CHECK(ParseIdMap(" 210 : Cobble , 211:WOOD,88:snow,92:none") == 4);
    CHECK(ClassOfId(210) == C_COBBLE && ClassOfId(211) == C_WOOD && ClassOfId(88) == C_SNOW && ClassOfId(92) == C_NONE);
}
TEST(map_can_override_a_built_in_class_and_none_switches_one_off) {
    resetMap();
    CHECK(ClassOfId(69) == C_TARMAC);                                     // Tarmac_Coarse_Clean_Dry
    ParseIdMap("69:cobble");
    CHECK(ClassOfId(69) == C_COBBLE);                                     // cobbles that reuse a tarmac id
    ParseIdMap("69:none");
    CHECK(ClassOfId(69) == C_NONE);
}
TEST(map_ignores_bad_entries_and_keeps_the_good_ones) {
    resetMap();
    const int n = ParseIdMap("abc,300:cobble,-4:wood,220:marble,221 cobble,,222:cobble,:wood,x7:wood,1e2:wood");
    CHECK(n == 1);                                                        // only 222:cobble is valid
    CHECK(ClassOfId(222) == C_COBBLE);
    CHECK(ClassOfId(0) == C_NONE);                                        // ":wood" and "abc" did not touch id 0
    CHECK(ParseIdMap("") == 0);
}

// ---------------------------------------------------------------- blending
TEST(plain_tarmac_blends_to_the_neutral_feel_with_no_ridge) {
    float w[C_COUNT]; weights(w, C_TARMAC, 1.0f);
    const Feel f = Blend(w, 1.0f);
    CHECK(near(f.amp, 1.0f) && near(f.floor, 0.0f) && near(f.spark, 0.0f) && f.ridge == 0.0f && f.ridgeMix == 0.0f);
}
TEST(pure_cobble_has_a_ridge_and_a_regular_strong_feel) {
    float w[C_COUNT]; weights(w, C_COBBLE, 1.0f);
    const Feel f = Blend(w, 1.0f);
    CHECK(near(f.ridge, 0.12f) && near(f.ridgeMix, 1.0f));
    CHECK(f.grainAdd < 0.0f && f.dropout == 0.0f);                        // regular, no gaps (gravel is the opposite)
    CHECK(f.floor >= 0.3f);                                               // felt even when the suspension barely moves
    CHECK(f.atk >= 0.8f && f.rel >= 2.0f);                                // crisp: each ridge is a separate hit
}
TEST(wood_ridges_are_wider_and_rounder_than_cobbles) {
    float wc[C_COUNT], ww[C_COUNT]; weights(wc, C_COBBLE, 1.0f); weights(ww, C_WOOD, 1.0f);
    const Feel c = Blend(wc, 1.0f), d = Blend(ww, 1.0f);
    CHECK(d.ridge > c.ridge);
    CHECK(d.rel < c.rel && d.atk < c.atk);
}
TEST(half_cobble_half_tarmac_keeps_the_real_ridge_and_half_the_mix) {
    float w[C_COUNT]; weights(w, C_COBBLE, 0.5f, C_TARMAC, 0.5f);
    const Feel f = Blend(w, 1.0f);
    CHECK(near(f.ridge, 0.12f));                                          // not averaged down to 6 cm
    CHECK(near(f.ridgeMix, 0.5f));
}
TEST(cobble_and_wood_mix_averages_the_spacing) {
    float w[C_COUNT]; weights(w, C_COBBLE, 0.5f, C_WOOD, 0.5f);
    const Feel f = Blend(w, 1.0f);
    CHECK(near(f.ridge, 0.185f, 1e-3f) && near(f.ridgeMix, 1.0f));
}
TEST(zero_strength_means_no_change_at_all_and_high_strength_stays_in_range) {
    float w[C_COUNT]; weights(w, C_COBBLE, 1.0f);
    const Feel off = Blend(w, 0.0f);
    CHECK(near(off.amp, 1.0f) && near(off.floor, 0.0f) && off.ridgeMix == 0.0f);
    const Feel hi = Blend(w, 1.5f);
    CHECK(hi.ridgeMix <= 1.0f && hi.floor <= 0.6f && hi.atk <= 1.0f && hi.rel <= 4.0f && hi.sparkMul <= 3.0f);
}
TEST(existing_surfaces_keep_their_tuned_values) {
    float g[C_COUNT], s[C_COUNT]; weights(g, C_GRAVEL, 1.0f); weights(s, C_SNOW, 1.0f);
    const Feel fg = Blend(g, 1.0f), fs = Blend(s, 1.0f);
    CHECK(near(fg.amp, 1.0f) && near(fg.grainAdd, 0.10f) && near(fg.dropout, 0.25f) && near(fg.hz, 1.10f) && near(fg.spark, 0.07f));
    CHECK(near(fs.amp, 0.55f) && near(fs.grainAdd, -0.70f) && near(fs.atk, 0.12f) && near(fs.floor, 0.22f));
    CHECK(fg.ridge == 0.0f && fs.ridge == 0.0f && fg.ridgeMix == 0.0f);
}

// ---------------------------------------------------------------- ridge pitch
TEST(ridge_pitch_is_speed_over_spacing_when_it_fits) {
    CHECK(near(RidgeHz(5.0f, 0.12f, 25.0f, 100.0f), 5.0f / 0.12f, 1e-3f));
    CHECK(near(RidgeHz(10.0f, 0.25f, 25.0f, 100.0f), 40.0f, 1e-3f));
}
TEST(ridge_pitch_rises_with_speed_inside_an_octave) {
    float last = 0;
    for (float v = 4.0f; v <= 12.0f; v += 1.0f) { const float h = RidgeHz(v, 0.12f, 25.0f, 100.0f); CHECK(h > last); last = h; }
}
TEST(ridge_pitch_folds_by_octaves_instead_of_pinning_at_the_ceiling) {
    CHECK(near(RidgeHz(30.0f, 0.12f, 25.0f, 100.0f), 62.5f, 1e-3f));    // 250 Hz -> 125 -> 62.5
    CHECK(RidgeHz(20.0f, 0.12f, 25.0f, 100.0f) != RidgeHz(30.0f, 0.12f, 25.0f, 100.0f));   // still follows speed at the top
    for (float v = 0.0f; v < 80.0f; v += 0.5f) { const float h = RidgeHz(v, 0.12f, 25.0f, 100.0f); CHECK(h >= 25.0f && h <= 100.0f); }
}
TEST(ridge_pitch_edges_are_safe) {
    CHECK(near(RidgeHz(0.0f, 0.12f, 25.0f, 100.0f), 25.0f));             // standing still
    CHECK(near(RidgeHz(-3.0f, 0.12f, 25.0f, 100.0f), 25.0f));            // reversing telemetry
    CHECK(near(RidgeHz(10.0f, 0.0f, 25.0f, 100.0f), 25.0f));             // no ridge
    CHECK(near(RidgeHz(1.0f, 0.12f, 25.0f, 100.0f), 25.0f));             // too slow: clamped up to the floor
    CHECK(RidgeHz(1e9f, 0.03f, 25.0f, 100.0f) <= 100.0f);                // absurd input
}

// ---------------------------------------------------------------- ini names
TEST(ini_keys_cover_every_value_including_ridge) {
    CHECK(std::string(FeelKeys()[0]) == "Amp" && std::string(FeelKeys()[kFeelParams - 1]) == "Ridge");
    Feel f = kNeutral; f.*FeelMembers()[kFeelParams - 1] = 0.5f;
    CHECK(near(f.ridge, 0.5f));
    CHECK(std::string(ClsIniName(C_COBBLE)) + FeelKeys()[kFeelParams - 1] == "CobbleRidge");
    CHECK(std::string(ClsIniName(C_WOOD)) + FeelKeys()[0] == "WoodAmp");
    CHECK(std::string(ClsName(C_COBBLE)) == "cobble" && std::string(ClsName(C_WOOD)) == "wood" && std::string(ClsName(99)) == "?");
}

// ---------------------------------------------------------------- Map= leniency and reporting
TEST(map_accepts_everyday_names_for_the_types) {
    resetMap();
    CHECK(ParseIdMap("300:cobble,89:cobblestone,90:Cobbles,91:wooden,92:bridge,93:planks,94:asphalt,95:dirt,96:grass") == 8);   // eight valid, 300 is out of range
    CHECK(ClassOfId(89) == C_COBBLE && ClassOfId(90) == C_COBBLE);
    CHECK(ClassOfId(91) == C_WOOD && ClassOfId(92) == C_WOOD && ClassOfId(93) == C_WOOD);
    CHECK(ClassOfId(94) == C_TARMAC && ClassOfId(95) == C_GRAVEL && ClassOfId(96) == C_SOFT);
}
TEST(class_from_name_rejects_unknown_types) {
    CHECK(ClassFromName("cobble") == C_COBBLE && ClassFromName("none") == C_NONE);
    CHECK(ClassFromName("marble") == -1 && ClassFromName("") == -1 && ClassFromName("Cobble") == -1);   // callers lower-case first
}
TEST(map_reports_every_entry_it_could_not_use) {
    resetMap();
    std::vector<std::string> bad;
    const int n = ParseIdMap("89:cobble, 90:marble, 400:wood, x:wood, 91 wood, 92:wood", &bad);
    CHECK(n == 2);                                                         // 89 and 92
    CHECK(bad.size() == 4);
    bool sawMarble = false, sawRange = false, sawNumber = false, sawFormat = false;
    for (const std::string& b : bad) {
        if (b.find("marble") != std::string::npos) sawMarble = true;
        if (b.find("400") != std::string::npos) sawRange = true;
        if (b.find("x:wood") != std::string::npos) sawNumber = true;
        if (b.find("91 wood") != std::string::npos && b.find("id:type") != std::string::npos) sawFormat = true;
    }
    CHECK(sawMarble && sawRange && sawNumber && sawFormat);
    CHECK(ClassOfId(89) == C_COBBLE && ClassOfId(92) == C_WOOD && ClassOfId(90) == C_NONE);
}
TEST(map_without_a_report_list_still_works_and_blank_entries_are_not_errors) {
    resetMap();
    std::vector<std::string> bad;
    CHECK(ParseIdMap(" , ,89:cobble,", &bad) == 1);
    CHECK(bad.empty());
    CHECK(ParseIdMap("90:wood") == 1);
}

int main() {
    for (const Reg& r : regs()) { int before = g_fail; r.fn(); std::printf("[%s] %s\n", g_fail == before ? " ok " : "FAIL", r.name); }
    std::printf("\n%zu tests, %d checks, %d failures\n", regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
