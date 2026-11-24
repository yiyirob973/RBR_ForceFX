// Tests for the road feel model (road_feel.hpp). Self-contained:
//   g++ -std=c++17 -O1 road_feel_test.cpp -o road_feel_test && ./road_feel_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "../materials.hpp"
#include "../surface_feel.hpp"
#include "../road_feel.hpp"

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)
static bool near(float a, float b, float e = 1e-4f) { return std::fabs(a - b) <= e; }
static void resetMap() { for (int i = 0; i < 256; ++i) Surf::IdMap()[i] = 0; }
static int idOf(const char* name) { for (const ffx::MaterialInfo& m : ffx::kMaterials) if (std::string(m.name) == name) return m.id; return -1; }
static Road::Ground gnd(const char* name) { Road::Ground g; const bool ok = Road::GroundOfId(idOf(name), g); CHECK(ok); return g; }
static Surf::Feel feel(const char* name, float activity = 0.6f, float ridge = 0.0f) { Road::Drive d; d.activity = activity; d.ridge = ridge; return Road::Derive(gnd(name), d); }
static bool finite(const Surf::Feel& f) {
    const float v[] = {f.amp, f.grainAdd, f.dropout, f.hz, f.atk, f.rel, f.base, f.spark, f.sparkMul, f.floor, f.ridge, f.ridgeMix};
    for (float x : v) if (!std::isfinite(x)) return false;
    return true;
}

// ---------------------------------------------------------------- ground from the material names
TEST(every_ground_material_has_a_ground_and_objects_have_none) {
    int grounds = 0;
    for (const ffx::MaterialInfo& m : ffx::kMaterials) {
        Road::Ground g;
        const bool ok = Road::GroundOfId(m.id, g);
        if (m.cls >= 0) {
            CHECK(ok); ++grounds;
            CHECK(g.hard >= 0 && g.hard <= 1 && g.loose >= 0 && g.loose <= 1 && g.coarse >= 0 && g.coarse <= 1 && g.slick >= 0 && g.slick <= 1);
        } else CHECK(!ok);
    }
    CHECK(grounds > 90);
}
TEST(ids_outside_the_table_have_no_ground) {
    Road::Ground g;
    CHECK(!Road::GroundOfId(-1, g) && !Road::GroundOfId(256, g) && !Road::GroundOfId(4, g) && !Road::GroundOfId(100000, g));
}
TEST(snow_materials_are_not_read_as_gravel_or_tarmac) {
    const Road::Ground s = gnd("SnowOnGravel_Medium"), gr = gnd("Gravel_Medium_Deep_Dry");
    CHECK(s.hard < 0.3f && s.slick > 0.4f);          // soft and slippery, unlike gravel
    CHECK(gr.hard > 0.5f && gr.slick == 0.0f);
    CHECK(gnd("SnowOnIce_Shallow").slick > 0.95f);
}
TEST(tarmac_is_hard_and_only_sprinkled_or_covered_tarmac_has_loose_stuff) {
    CHECK(gnd("Tarmac_Fine_Clean_Dry").hard == 1.0f && gnd("Tarmac_Fine_Clean_Dry").loose == 0.0f);
    CHECK(gnd("Tarmac_Medium_Sprinkled_Dry").loose > 0.2f);
    CHECK(gnd("Tarmac_Medium_Covered_Dry").loose > gnd("Tarmac_Medium_Sprinkled_Dry").loose);
}
TEST(deeper_gravel_is_softer_and_looser_and_coarser_gravel_has_a_coarser_grain) {
    CHECK(gnd("Gravel_Fine_Shallow_Dry").hard > gnd("Gravel_Fine_Deep_Dry").hard);
    CHECK(gnd("Gravel_Fine_Deep_Dry").hard > gnd("Gravel_Fine_Deeper_Dry").hard);
    CHECK(gnd("Gravel_Fine_Deeper_Dry").loose > gnd("Gravel_Fine_Shallow_Dry").loose);
    CHECK(gnd("Gravel_Coarse_Deep_Dry").coarse > gnd("Gravel_Fine_Deep_Dry").coarse);
}
TEST(wet_ground_is_more_slippery_than_dry) {
    CHECK(gnd("Tarmac_Fine_Clean_Wet").slick > gnd("Tarmac_Fine_Clean_Damp").slick);
    CHECK(gnd("Tarmac_Fine_Clean_Damp").slick > gnd("Tarmac_Fine_Clean_Dry").slick);
    CHECK(gnd("Gravel_Medium_Deep_Wet").slick > gnd("Gravel_Medium_Deep_Dry").slick);
    CHECK(gnd("Grass_Soft_Wet").slick > gnd("Grass_Soft_Dry").slick);
}
TEST(grass_is_softer_than_tarmac_and_dirt_sits_between_gravel_and_grass_in_hardness) {
    CHECK(gnd("Grass_Medium_Dry").hard < gnd("Tarmac_Medium_Clean_Dry").hard);
    CHECK(gnd("Dirt_Hard_Dry").hard > gnd("Dirt_Soft_Dry").hard);
    CHECK(gnd("Dirt_Soft_Dry").hard < gnd("Gravel_Medium_Shallow_Dry").hard);
}
TEST(water_has_a_ground_even_though_it_is_not_a_road) {
    const Road::Ground w = gnd("Water_Surface");
    CHECK(w.slick > 0.4f && w.hard < 0.5f);
}

// ---------------------------------------------------------------- which ground a tyre gets
TEST(the_material_name_decides_when_the_id_is_in_the_table) {
    resetMap();
    const int id = idOf("Gravel_Medium_Deep_Dry");
    const Road::Ground g = Road::GroundForTyre(id, Surf::C_SNOW);       // the class would say snow, the name says gravel
    CHECK(near(g.loose, 0.9f) && near(g.hard, 0.55f));
}
TEST(a_map_entry_for_the_id_wins_over_the_name) {
    resetMap();
    const int id = idOf("Gravel_Medium_Deep_Dry");
    Surf::ParseIdMap(std::to_string(id) + ":cobble");
    const Road::Ground g = Road::GroundForTyre(id, Surf::ClassOfId(id));
    CHECK(g.ridged == 1.0f && g.hard == 1.0f);
    resetMap();
}
TEST(an_id_the_table_does_not_have_falls_back_to_its_class) {
    resetMap();
    Surf::ParseIdMap("200:cobble,201:wood");
    CHECK(Road::GroundForTyre(200, Surf::ClassOfId(200)).ridged == 1.0f);
    CHECK(Road::GroundForTyre(201, Surf::ClassOfId(201)).spacing > Road::GroundForTyre(200, Surf::ClassOfId(200)).spacing);   // planks are wider than cobbles
    resetMap();
}
TEST(nothing_known_gives_plain_hard_ground_with_no_special_character) {
    resetMap();
    const Road::Ground g = Road::GroundForTyre(-1, Surf::C_NONE);
    CHECK(g.hard == 1.0f && g.loose == 0.0f && g.slick == 0.0f && g.ridged == 0.0f);
    const Surf::Feel f = Road::Derive(g, Road::Drive());
    CHECK(f.floor == 0.0f && f.spark == 0.0f && f.ridgeMix == 0.0f && f.hz == 1.0f && f.base == 1.0f && f.amp == 1.0f);
}
TEST(an_object_id_is_not_ground_so_the_class_decides) {
    resetMap();
    Road::Ground g;
    CHECK(!Road::GroundOfId(30, g));                                       // a tree
    CHECK(Road::GroundForTyre(30, Surf::C_NONE).hard == 1.0f);
}

// ---------------------------------------------------------------- the road character
TEST(clean_tarmac_stays_close_to_the_plain_feel) {
    const Surf::Feel f = feel("Tarmac_Fine_Clean_Dry", 0.8f);
    CHECK(near(f.amp, 1.0f, 0.01f) && near(f.hz, 1.0f, 0.01f) && f.spark == 0.0f && f.floor < 0.01f && f.ridgeMix == 0.0f);
}
TEST(loose_gravel_is_grainy_with_gaps_stones_and_a_fast_release_compared_with_tarmac) {
    const Surf::Feel g = feel("Gravel_Medium_Deep_Dry"), t = feel("Tarmac_Fine_Clean_Dry");
    CHECK(g.grainAdd > t.grainAdd && g.dropout > 0.1f && g.spark > 0.02f && g.hz > t.hz && g.rel > t.rel);
}
TEST(snow_is_softer_slower_and_lower_than_gravel) {
    const Surf::Feel s = feel("SnowOnGravel_Deep"), g = feel("Gravel_Medium_Deep_Dry");
    CHECK(s.amp < g.amp && s.hz < g.hz && s.atk < g.atk && s.rel < g.rel && s.grainAdd < g.grainAdd);
    CHECK(s.floor > 0.15f);                                                // snow is never silent while the car moves
}
TEST(ice_is_hard_quiet_between_cracks_and_cracks_sharply) {
    const Surf::Feel i = feel("SnowOnIce_Shallow"), t = feel("Tarmac_Fine_Clean_Dry");
    CHECK(i.base < 0.5f && i.spark > 0.05f && i.sparkMul > 1.8f && i.hz > t.hz && i.atk > 0.6f);
}
TEST(grass_is_soft_and_rounder_than_tarmac) {
    const Surf::Feel g = feel("Grass_Medium_Dry"), t = feel("Tarmac_Fine_Clean_Dry");
    CHECK(g.amp < t.amp && g.hz < t.hz && g.atk < t.atk && g.floor > t.floor);
}
TEST(wet_tarmac_is_a_little_quieter_between_hits_than_dry) {
    CHECK(feel("Tarmac_Fine_Clean_Wet").base < feel("Tarmac_Fine_Clean_Dry").base);
}
TEST(standing_still_has_no_stones_flung_and_wheel_slip_flings_more) {
    const Surf::Feel still = feel("Gravel_Medium_Deep_Dry", 0.0f), rolling = feel("Gravel_Medium_Deep_Dry", 0.4f), sliding = feel("Gravel_Medium_Deep_Dry", 1.0f);
    CHECK(still.spark < rolling.spark && rolling.spark < sliding.spark);
    CHECK(still.sparkMul < rolling.sparkMul && rolling.sparkMul < sliding.sparkMul);
}
TEST(activity_comes_from_speed_and_slip_and_slip_counts_most) {
    CHECK(Road::Activity(0.0f, 0.0f) == 0.0f);
    CHECK(Road::Activity(1.0f, 0.0f) < Road::Activity(0.0f, 1.0f));
    CHECK(Road::Activity(0.5f, 0.2f) > Road::Activity(0.5f, 0.0f));
    CHECK(Road::Activity(5.0f, 5.0f) == 1.0f && Road::Activity(-1.0f, -1.0f) == 0.0f);
    CHECK(Road::Activity(NAN, NAN) == 0.0f && Road::Activity(INFINITY, 0.0f) == 0.45f);   // a bad number counts as nothing
}
TEST(ridges_need_hard_ground_that_is_not_mostly_loose) {
    Road::Drive d; d.ridge = 1.0f;
    CHECK(Road::Derive(gnd("Tarmac_Coarse_Clean_Dry"), d).ridgeMix > 0.95f);          // hard tarmac that shakes like cobbles: cobbles
    CHECK(Road::Derive(gnd("Gravel_Coarse_Deeper_Dry"), d).ridgeMix == 0.0f);           // gravel shakes a lot too, and is not cobbled
    CHECK(Road::Derive(gnd("Grass_Soft_Dry"), d).ridgeMix == 0.0f);
    CHECK(Road::Derive(gnd("SnowOnGravel_Deep"), d).ridgeMix == 0.0f);
    const float sprinkled = Road::Derive(gnd("Tarmac_Coarse_Sprinkled_Dry"), d).ridgeMix;
    CHECK(sprinkled > 0.7f && sprinkled < 1.0f);                                         // the probe drive's cobbled stage used this id
}
TEST(ridged_ground_is_much_stronger_sharper_and_locked_to_a_spacing) {
    Road::Drive d; d.ridge = 1.0f;
    const Surf::Feel c = Road::Derive(gnd("Tarmac_Coarse_Sprinkled_Dry"), d), t = feel("Tarmac_Coarse_Sprinkled_Dry", 0.6f, 0.0f);
    CHECK(c.amp > 1.8f * t.amp && c.rel > t.rel && c.floor > 0.3f && c.grainAdd < t.grainAdd);
    CHECK(near(c.ridge, Road::kRidgeSpacing, 1e-4f) && t.ridge == 0.0f);
}
TEST(a_ground_assigned_by_map_as_ridged_has_ridges_without_the_detector) {
    Road::Drive d;                                                                      // no detector score
    CHECK(Road::Derive(Road::GroundOfClass(Surf::C_COBBLE), d).ridgeMix > 0.95f);
    CHECK(Road::Derive(Road::GroundOfClass(Surf::C_WOOD), d).ridge > Road::Derive(Road::GroundOfClass(Surf::C_COBBLE), d).ridge);
}
TEST(the_feel_changes_smoothly_between_two_grounds) {
    const Road::Ground a = gnd("Gravel_Medium_Deep_Dry"), b = gnd("SnowOnGravel_Deep");
    Road::Drive d; d.activity = 0.5f;
    float prevHz = Road::Derive(a, d).hz, worst = 0.0f;
    for (int i = 1; i <= 20; ++i) {
        const float hz = Road::Derive(Road::Mix(a, b, i / 20.0f), d).hz;
        worst = std::max(worst, std::fabs(hz - prevHz)); prevHz = hz;
    }
    CHECK(worst < 0.1f);
    CHECK(near(Road::Derive(Road::Mix(a, b, 0.0f), d).hz, Road::Derive(a, d).hz) && near(Road::Derive(Road::Mix(a, b, 1.0f), d).hz, Road::Derive(b, d).hz));
}
TEST(every_ground_at_every_activity_and_ridge_gives_values_in_range) {
    for (const ffx::MaterialInfo& m : ffx::kMaterials) {
        Road::Ground g;
        if (!Road::GroundOfId(m.id, g)) continue;
        for (float act : {0.0f, 0.3f, 1.0f})
            for (float rg : {0.0f, 0.5f, 1.0f}) {
                Road::Drive d; d.activity = act; d.ridge = rg;
                const Surf::Feel f = Road::Derive(g, d);
                CHECK(finite(f));
                CHECK(f.atk >= 0.05f && f.atk <= 1.0f && f.rel >= 0.05f && f.rel <= 4.0f && f.hz >= 0.3f && f.hz <= 2.0f);
                CHECK(f.dropout >= 0.0f && f.dropout <= 0.6f && f.spark >= 0.0f && f.spark <= 0.5f && f.floor >= 0.0f && f.floor <= 0.6f);
                CHECK(f.amp >= 0.0f && f.base >= 0.0f && f.sparkMul >= 0.5f && f.sparkMul <= 3.0f && f.ridgeMix >= 0.0f && f.ridgeMix <= 1.0f);
            }
    }
}
TEST(out_of_range_properties_and_inputs_are_clamped_not_propagated) {
    Road::Ground g; g.hard = 5.0f; g.loose = -3.0f; g.coarse = 9.0f; g.slick = 7.0f;
    Road::Drive d; d.activity = 50.0f; d.ridge = -4.0f;
    CHECK(finite(Road::Derive(g, d)));
}

TEST(wheelspin_and_slides_dig_into_loose_ground_and_stay_quiet_on_ice) {
    Road::Drive calm, spin, slide; spin.spin = 1.0f; slide.slide = 1.0f;
    const Road::Ground gravel = gnd("Gravel_Coarse_Deeper_Dry"), ice = gnd("SnowOnIce_Shallow");
    const Surf::Feel g0 = Road::Derive(gravel, calm), g1 = Road::Derive(gravel, spin), g2 = Road::Derive(gravel, slide);
    CHECK(g1.amp > g0.amp && g2.amp > g1.amp);                         // a slide digs in harder than wheelspin
    CHECK(g2.spark > g0.spark && g2.floor > g0.floor && g2.hz > g0.hz);
    const Surf::Feel i0 = Road::Derive(ice, calm), i2 = Road::Derive(ice, slide);
    CHECK(i2.amp - i0.amp < g2.amp - g0.amp);                          // little to grab on ice
}
TEST(spin_and_slide_inputs_are_safe_for_bad_numbers) {
    Road::Drive d; d.spin = NAN; d.slide = -5.0f;
    const Surf::Feel f = Road::Derive(gnd("Gravel_Coarse_Deeper_Dry"), d);
    CHECK(std::isfinite(f.amp) && std::isfinite(f.hz) && f.amp > 0.0f);
}

int main() {
    for (const Reg& r : regs()) { int before = g_fail; r.fn(); std::printf("[%s] %s\n", g_fail == before ? " ok " : "FAIL", r.name); }
    std::printf("\n%zu tests, %d checks, %d failures\n", regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
