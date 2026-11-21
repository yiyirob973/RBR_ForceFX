// Tests for the WindscreenFX logic that needs no Windows or Direct3D (camera/ws_logic.hpp): what the tyres throw at the glass,
// the colours of the dirt, the area a wiper blade sweeps and the measuring of a wiper mesh. Build it with any C++17 compiler:
//   g++ -std=c++17 -O1 ws_logic_test.cpp -o ws_logic_test && ./ws_logic_test
//   cl /std:c++17 /EHsc ws_logic_test.cpp && ws_logic_test.exe
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <vector>
#include "../camera/ws_logic.hpp"

using namespace ffx::cam::wsl;

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)
static bool near(float a, float b, float e = 0.01f) { return std::fabs(a - b) <= e; }

static const rsa::MaterialTable& tab() { static rsa::MaterialTable t; return t; }
static Ground ground(int fl, int fr, int rl, int rr) { const int ids[4] = {fl, fr, rl, rr}; return groundFromIds(ids, tab(), GroundCfg()); }
static float shareSum(const float* s) { float t = 0; for (int i = 0; i < kTints; ++i) t += s[i]; return t; }

// material ids used below (NGP's physics.lsp, see materials.hpp)
enum { TarmacCleanDry = 37, TarmacCleanWet = 39, TarmacCoveredDry = 53, GravelFineShallowDry = 17, GravelCoarseDeeperWet = 35,
       DirtSoftWet = 151, DirtSoftDry = 149, SnowOnGravelMedium = 102, GrassSoftWet = 115, WaterSurface = 157, Undefined = 0, Tree = 30, Unlisted = 250 };

// ---------------------------------------------------------------- palette
TEST(palette_dust_is_paler_than_mud) {
    const Rgb mud{0.52f, 0.40f, 0.28f}, d = tintColour(kDust, mud);
    CHECK(d.r > mud.r && d.g > mud.g && d.b > mud.b);
}
TEST(palette_spray_is_dark_and_grey) {
    const Rgb mud{0.52f, 0.40f, 0.28f}, s = tintColour(kSpray, mud);
    CHECK(s.r < mud.r && s.g < mud.g + 0.0f);
    CHECK(std::fabs(s.r - s.b) < 0.1f);
}
TEST(palette_follows_the_mud_colour) {
    const Rgb clay{0.62f, 0.36f, 0.24f}, slate{0.42f, 0.40f, 0.38f};
    CHECK(tintColour(kDust, clay).r - tintColour(kDust, clay).b > tintColour(kDust, slate).r - tintColour(kDust, slate).b);
    CHECK(near(tintColour(kMud, clay).r, 0.62f, 1e-6f));
}
TEST(palette_slush_is_fixed_and_light) {
    const Rgb a = tintColour(kSlush, {0.1f, 0.1f, 0.1f}), b = tintColour(kSlush, {0.9f, 0.9f, 0.9f});
    CHECK(near(a.r, b.r, 1e-6f) && a.r > 0.8f);
    const Rgb f = frostColour(kSlush, {0.5f, 0.4f, 0.3f});
    CHECK(f.r > 0.85f && f.b > 0.9f);
}
TEST(palette_grass_is_greener_than_mud) {
    const Rgb mud{0.52f, 0.40f, 0.28f}, g = tintColour(kGrass, mud);
    CHECK(g.g > g.r - 0.05f && g.g / g.r > mud.g / mud.r);
}

// ---------------------------------------------------------------- ground
TEST(clean_dry_tarmac_throws_nothing) {
    const Ground g = ground(TarmacCleanDry, TarmacCleanDry, TarmacCleanDry, TarmacCleanDry);
    CHECK(g.ok); CHECK(near(g.loose, 0.f)); CHECK(near(g.spray, 0.f)); CHECK(near(g.dust, 0.f)); CHECK(near(g.snow, 0.f));
}
TEST(wet_tarmac_throws_grey_water_not_mud) {
    const Ground g = ground(TarmacCleanWet, TarmacCleanWet, TarmacCleanWet, TarmacCleanWet);
    CHECK(g.ok); CHECK(g.loose < 0.01f); CHECK(g.spray > 0.8f);
    CHECK(near(shareSum(g.fine), 1.f)); CHECK(g.fine[kSpray] > 0.99f);
}
TEST(dry_gravel_throws_dust_and_few_clods) {
    const Ground g = ground(GravelFineShallowDry, GravelFineShallowDry, GravelFineShallowDry, GravelFineShallowDry);
    CHECK(g.loose > 0.6f); CHECK(near(g.mudMul, 0.15f)); CHECK(g.dust > 0.3f); CHECK(g.spray < 0.01f);
    CHECK(g.clod[kDust] > 0.99f); CHECK(g.fine[kDust] > 0.99f);
}
TEST(wet_deep_gravel_throws_mud) {
    const Ground g = ground(GravelCoarseDeeperWet, GravelCoarseDeeperWet, GravelCoarseDeeperWet, GravelCoarseDeeperWet);
    CHECK(g.loose > 0.9f); CHECK(near(g.mudMul, 1.0f)); CHECK(g.clod[kMud] > 0.99f); CHECK(g.spray > 0.5f);
}
TEST(snow_throws_slush) {
    const Ground g = ground(SnowOnGravelMedium, SnowOnGravelMedium, SnowOnGravelMedium, SnowOnGravelMedium);
    CHECK(near(g.snow, 1.f)); CHECK(g.clod[kSlush] > 0.99f); CHECK(near(g.mudMul, 0.6f));
}
TEST(wet_grass_is_green_brown) {
    const Ground g = ground(GrassSoftWet, GrassSoftWet, GrassSoftWet, GrassSoftWet);
    CHECK(g.clod[kGrass] > 0.99f);
}
TEST(water_splash_is_spray) {
    const Ground g = ground(WaterSurface, WaterSurface, WaterSurface, WaterSurface);
    CHECK(g.spray > 0.9f); CHECK(g.fine[kSpray] > 0.99f);
}
TEST(tarmac_covered_in_gravel_throws_some) {
    const Ground a = ground(TarmacCoveredDry, TarmacCoveredDry, TarmacCoveredDry, TarmacCoveredDry);
    const Ground b = ground(TarmacCleanDry, TarmacCleanDry, TarmacCleanDry, TarmacCleanDry);
    CHECK(a.loose > 0.5f); CHECK(b.loose < 0.01f);
}
TEST(not_ground_is_ignored_and_the_rest_renormalised) {
    const Ground one = ground(DirtSoftWet, Undefined, Tree, Unlisted);        // three tyres say nothing
    const Ground all = ground(DirtSoftWet, DirtSoftWet, DirtSoftWet, DirtSoftWet);
    CHECK(one.ok); CHECK(near(one.loose, all.loose, 1e-4f)); CHECK(near(one.mudMul, all.mudMul, 1e-4f));
}
TEST(all_unknown_is_not_ok) {
    CHECK(!ground(Undefined, Tree, Unlisted, -1).ok);
}
TEST(left_grass_right_tarmac_mixes_both) {
    const Ground g = ground(GrassSoftWet, TarmacCleanDry, GrassSoftWet, TarmacCleanDry);
    CHECK(g.ok); CHECK(g.loose > 0.1f && g.loose < 0.45f); CHECK(g.clod[kGrass] > 0.99f);   // tarmac adds no clods, so the clods are all grass
}
TEST(front_tyres_count_more_than_rear) {
    const Ground front = ground(DirtSoftDry, DirtSoftDry, TarmacCleanDry, TarmacCleanDry);
    const Ground rear  = ground(TarmacCleanDry, TarmacCleanDry, DirtSoftDry, DirtSoftDry);
    CHECK(front.loose > rear.loose);
}
TEST(shares_add_up_to_one_when_something_is_thrown) {
    const Ground g = ground(GravelCoarseDeeperWet, DirtSoftDry, TarmacCleanWet, GrassSoftWet);
    CHECK(near(shareSum(g.clod), 1.f, 1e-4f)); CHECK(near(shareSum(g.fine), 1.f, 1e-4f));
}
TEST(pick_tint_follows_the_shares) {
    float s[kTints] = {0.25f, 0, 0.75f, 0, 0};
    CHECK(pickTint(s, 0.10f, kMud) == kMud); CHECK(pickTint(s, 0.30f, kMud) == kDust); CHECK(pickTint(s, 0.999f, kMud) == kDust);
    float z[kTints] = {0, 0, 0, 0, 0};
    CHECK(pickTint(z, 0.5f, kSpray) == kSpray);
}

// ---------------------------------------------------------------- wiper sweep
static const Seg kFlat{30.f, 0.f, 100.f, 0.f};                         // blade along +x, rubber from 30 to 100 px out
static Seg rotated(const Seg& s, float deg) {                          // about the origin, y down
    const float a = deg * 0.0174533f, c = std::cos(a), si = std::sin(a);
    return {s.ix * c - s.iy * si, s.ix * si + s.iy * c, s.ox * c - s.oy * si, s.ox * si + s.oy * c};
}
TEST(sweep_hits_what_the_blade_crossed) {
    const Seg b = rotated(kFlat, 10.f); float mx, my;
    CHECK(sweptBy(kFlat, b, 70.f, 6.f, 2.f, 3.f, mx, my));             // half way along, inside the 10 degree wedge
}
TEST(sweep_misses_outside_the_arc) {
    const Seg b = rotated(kFlat, 10.f); float mx, my;
    CHECK(!sweptBy(kFlat, b, 70.f, 40.f, 2.f, 3.f, mx, my));           // far to the side
    CHECK(!sweptBy(kFlat, b, 70.f, -30.f, 2.f, 3.f, mx, my));          // the other side
}
TEST(sweep_respects_both_ends_of_the_rubber) {
    const Seg b = rotated(kFlat, 10.f); float mx, my;
    CHECK(!sweptBy(kFlat, b, 8.f, 1.f, 2.f, 3.f, mx, my));             // near the pivot: arm, not rubber
    CHECK(!sweptBy(kFlat, b, 150.f, 10.f, 2.f, 3.f, mx, my));          // past the tip
    CHECK(sweptBy(kFlat, b, 103.f, 8.f, 2.f, 3.f, mx, my));            // just at the tip counts (blade width and drop radius)
}
TEST(sweep_follows_the_real_end_points_not_a_circle) {
    // perspective makes the far end of a blade come closer to the pivot while it rises: the outer end moves from (100,0) to (60,-50)
    const Seg a{30.f, 0.f, 100.f, 0.f}, b{20.f, -18.f, 60.f, -50.f}; float mx, my;
    CHECK(sweptBy(a, b, 70.f, -20.f, 1.f, 2.f, mx, my));
    CHECK(!sweptBy(a, b, 95.f, -45.f, 1.f, 2.f, mx, my));              // a circle of radius 100 would have caught this; the real blade never got there
}
TEST(sweep_direction_is_the_movement_of_the_blade) {
    const Seg b = rotated(kFlat, 10.f); float mx = 0, my = 0;
    CHECK(sweptBy(kFlat, b, 70.f, 6.f, 2.f, 3.f, mx, my));
    CHECK(my > 0.f && std::fabs(mx) < my);                             // y grows downward on screen, a positive angle sweeps towards +y
}
TEST(sweep_widens_for_a_big_step) {
    const Seg b = rotated(kFlat, 60.f); float mx, my;
    // on the arc half way through the step the chord is well inside the circle; the sagitta margin keeps most of that bulge covered
    const float a30 = 30.f * 0.0174533f;
    CHECK(sweptBy(kFlat, b, 90.f * std::cos(a30), 90.f * std::sin(a30), 2.f, 3.f, mx, my));
}
TEST(blade_motion_classes) {
    CHECK(bladeMotion(kFlat, kFlat, 500.f) == 0);
    CHECK(bladeMotion(kFlat, rotated(kFlat, 5.f), 500.f) == 1);
    CHECK(bladeMotion(kFlat, Seg{900.f, 0.f, 1000.f, 0.f}, 500.f) == 2);
}

// ---------------------------------------------------------------- wiper mesh extent
static std::vector<uint8_t> mesh(int axis, float lo, float hi, float width, int stride, int n = 40, float poison = 0.f) {
    std::vector<uint8_t> v((size_t)n * (size_t)stride, 0);
    for (int i = 0; i < n; ++i) {
        float p[3] = {0, 0, 0};
        p[axis] = lo + (hi - lo) * (float)i / (float)(n - 1);
        p[(axis + 1) % 3] = (i & 1) ? width : -width;
        p[(axis + 2) % 3] = (i & 2) ? width * 0.5f : -width * 0.5f;
        if (poison != 0.f && i < n / 2) p[0] = poison;
        std::memcpy(&v[(size_t)i * (size_t)stride], p, 12);
    }
    return v;
}
TEST(extent_finds_the_long_axis_of_an_arm) {
    const auto m = mesh(1, 0.f, 0.55f, 0.01f, 32);
    const Extent e = measureExtent(m.data(), 40, 32);
    CHECK(e.ok); CHECK(e.axis == 1); CHECK(near(e.lo, 0.f, 1e-4f)); CHECK(near(e.hi, 0.55f, 1e-4f));
}
TEST(extent_handles_a_negative_direction) {
    const auto m = mesh(2, -0.6f, 0.f, 0.01f, 24);
    const Extent e = measureExtent(m.data(), 40, 24);
    CHECK(e.ok); CHECK(e.axis == 2); CHECK(near(e.lo, -0.6f, 1e-4f)); CHECK(near(e.hi, 0.f, 1e-4f));
}
TEST(extent_rejects_a_blob) {
    const auto m = mesh(0, 0.f, 0.3f, 0.2f, 32);                        // as wide as it is long: not a wiper
    CHECK(!measureExtent(m.data(), 40, 32).ok);
}
TEST(extent_rejects_a_mesh_that_is_far_too_long_or_tiny) {
    CHECK(!measureExtent(mesh(0, 0.f, 3.0f, 0.01f, 32).data(), 40, 32).ok);
    CHECK(!measureExtent(mesh(0, 0.f, 0.05f, 0.002f, 32).data(), 40, 32).ok);
}
TEST(extent_rejects_garbage_vertices) {
    CHECK(!measureExtent(mesh(0, 0.f, 0.5f, 0.01f, 32, 40, 1e6f).data(), 40, 32).ok);
    std::vector<uint8_t> nan((size_t)40 * 32, 0xFF);
    CHECK(!measureExtent(nan.data(), 40, 32).ok);
}
TEST(extent_rejects_bad_arguments) {
    CHECK(!measureExtent(nullptr, 40, 32).ok);
    const auto m = mesh(0, 0.f, 0.5f, 0.01f, 32);
    CHECK(!measureExtent(m.data(), 2, 32).ok); CHECK(!measureExtent(m.data(), 40, 8).ok); CHECK(!measureExtent(m.data(), 40, 4096).ok);
}
TEST(rubber_starts_after_the_arm_for_a_whole_wiper) {
    Extent e; e.ok = true; e.axis = 0; e.lo = 0.f; e.hi = 0.6f;
    float a, b; rubberSpan(e, 0.25f, a, b);
    CHECK(near(a, 0.15f, 1e-5f)); CHECK(near(b, 0.6f, 1e-5f));
}
TEST(rubber_is_the_whole_mesh_for_a_blade_alone) {
    Extent e; e.ok = true; e.axis = 0; e.lo = 0.2f; e.hi = 0.6f;
    float a, b; rubberSpan(e, 0.25f, a, b);
    CHECK(near(a, 0.2f, 1e-5f)); CHECK(near(b, 0.6f, 1e-5f));
}
TEST(rubber_for_a_mesh_centred_on_its_origin_is_all_of_it) {
    Extent e; e.ok = true; e.axis = 0; e.lo = -0.25f; e.hi = 0.25f;
    float a, b; rubberSpan(e, 0.30f, a, b);
    CHECK(near(a, -0.25f, 1e-5f)); CHECK(near(b, 0.25f, 1e-5f));
}
TEST(rubber_of_an_arm_that_points_the_negative_way) {
    Extent e; e.ok = true; e.axis = 2; e.lo = -0.6f; e.hi = 0.f;
    float a, b; rubberSpan(e, 0.25f, a, b);
    CHECK(near(a, -0.15f, 1e-5f)); CHECK(near(b, -0.6f, 1e-5f));          // from the near end to the far end, so a is closer to the pivot
}
TEST(rubber_of_a_blade_alone_on_the_negative_side) {
    Extent e; e.ok = true; e.axis = 2; e.lo = -0.6f; e.hi = -0.2f;
    float a, b; rubberSpan(e, 0.25f, a, b);
    CHECK(near(a, -0.6f, 1e-5f)); CHECK(near(b, -0.2f, 1e-5f));
}

// ---------------------------------------------------------------- smoothing
TEST(smoothing_takes_the_first_ground_at_once) {
    Ground cur, tgt = ground(DirtSoftWet, DirtSoftWet, DirtSoftWet, DirtSoftWet);
    smoothGround(cur, tgt, 0.1f);
    CHECK(cur.ok); CHECK(near(cur.loose, tgt.loose, 1e-6f));
}
TEST(smoothing_eases_towards_a_new_ground) {
    Ground cur = ground(TarmacCleanDry, TarmacCleanDry, TarmacCleanDry, TarmacCleanDry);
    const Ground tgt = ground(GravelCoarseDeeperWet, GravelCoarseDeeperWet, GravelCoarseDeeperWet, GravelCoarseDeeperWet);
    smoothGround(cur, tgt, 0.25f);
    CHECK(cur.loose > 0.1f && cur.loose < tgt.loose * 0.5f);
    for (int i = 0; i < 60; ++i) smoothGround(cur, tgt, 0.25f);
    CHECK(near(cur.loose, tgt.loose, 0.01f));
}
TEST(smoothing_drops_out_when_the_target_is_not_ok) {
    Ground cur = ground(DirtSoftWet, DirtSoftWet, DirtSoftWet, DirtSoftWet);
    smoothGround(cur, ground(Undefined, Undefined, Undefined, Undefined), 0.5f);
    CHECK(!cur.ok);
}


// ---------------------------------------------------------------- mud between stages
static StageHistory hist(bool finished = true, int map = 10, const char* who = "Bob", float since = 120.f) { StageHistory h; h.have = true; h.finished = finished; h.map = map; h.player = who; h.secondsSinceFinish = since; return h; }
TEST(next_stage_of_the_same_rally_keeps_the_mud) { CHECK(cleanReason(true, hist(), 11, "Bob", 10.f) == nullptr); }
TEST(first_stage_cleans) { StageHistory h; CHECK(cleanReason(true, h, 11, "Bob", 10.f) != nullptr); }
TEST(a_stage_that_was_not_finished_cleans) { CHECK(cleanReason(true, hist(false), 11, "Bob", 10.f) != nullptr); }
TEST(the_same_stage_again_cleans) { CHECK(cleanReason(true, hist(true, 10), 10, "Bob", 10.f) != nullptr); }
TEST(a_different_driver_cleans) { CHECK(cleanReason(true, hist(), 11, "Alice", 10.f) != nullptr); }
TEST(a_long_break_cleans) { CHECK(cleanReason(true, hist(true, 10, "Bob", 11 * 60.f), 11, "Bob", 10.f) != nullptr); CHECK(cleanReason(true, hist(true, 10, "Bob", 9 * 60.f), 11, "Bob", 10.f) == nullptr); }
TEST(a_zero_break_limit_never_cleans_by_time) { CHECK(cleanReason(true, hist(true, 10, "Bob", 99999.f), 11, "Bob", 0.f) == nullptr); }
TEST(carry_over_off_always_cleans) { CHECK(cleanReason(false, hist(), 11, "Bob", 10.f) != nullptr); }
TEST(car_change_needs_two_known_cars) {
    CHECK(carChanged("Sunny|3|0", "Sunny|3|1")); CHECK(!carChanged("Sunny|3|0", "Sunny|3|0"));
    CHECK(!carChanged("", "Sunny|3|0")); CHECK(!carChanged("Sunny|3|0", ""));
}

int main() {
    for (const Reg& r : regs()) { const int before = g_fail; r.fn(); std::printf("%s  %s\n", g_fail == before ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
