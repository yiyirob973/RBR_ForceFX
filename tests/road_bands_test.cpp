// Tests for the road bands (road_bands.hpp): the road vibration as three sine effects. Self-contained:
//   g++ -std=c++17 -O1 road_bands_test.cpp -o road_bands_test && ./road_bands_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "../materials.hpp"
#include "../surface_feel.hpp"
#include "../road_feel.hpp"
#include "../road_bands.hpp"

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)
static bool near(float a, float b, float e = 1e-3f) { return std::fabs(a - b) <= e; }

static const float kMin = 25.0f, kMax = 100.0f;
static Surf::Feel withHz(float hz) { Surf::Feel f = Surf::kNeutral; f.hz = hz; return f; }
static Road::SideIn side(float hz, float strength) { Road::SideIn s; s.fe = withHz(hz); s.strength = strength; return s; }
static float energy(const float* sh) { float e = 0; for (int b = 0; b < Road::kBands; ++b) e += sh[b] * sh[b]; return e; }
static const float kOne[2][Road::kBands] = {{1, 1, 1}, {1, 1, 1}};

// ---------------------------------------------------------------- the three bands
TEST(the_three_bands_tile_the_road_range_without_gaps_or_overlap) {
    float lo[3], hi[3];
    for (int i = 0; i < 3; ++i) Road::BandRange(kMin, kMax, i, lo[i], hi[i]);
    CHECK(near(lo[0], kMin) && near(hi[2], kMax));
    CHECK(near(hi[0], lo[1]) && near(hi[1], lo[2]));
    for (int i = 0; i < 3; ++i) CHECK(hi[i] > lo[i]);
    CHECK(near(hi[0] / lo[0], hi[1] / lo[1], 1e-3f) && near(hi[1] / lo[1], hi[2] / lo[2], 1e-3f));   // equal on a pitch scale
}
TEST(a_bad_range_still_gives_a_usable_one) {
    float lo, hi;
    Road::BandRange(NAN, NAN, 1, lo, hi); CHECK(std::isfinite(lo) && std::isfinite(hi) && hi > lo);
    Road::BandRange(50.0f, 20.0f, 0, lo, hi); CHECK(hi > lo);                 // max below min
    Road::BandRange(25.0f, 100.0f, 9, lo, hi); CHECK(near(hi, kMax));          // an index past the end acts as the last band
}
TEST(each_band_sits_in_the_middle_of_its_third_at_the_reference_speed) {
    for (int i = 0; i < 3; ++i) {
        float lo, hi; Road::BandRange(kMin, kMax, i, lo, hi);
        CHECK(near(Road::BandPitch(Road::kRefSpeed, kMin, kMax, i), std::sqrt(lo * hi), 0.01f));
    }
}
TEST(band_pitch_follows_speed_and_stays_inside_its_third) {
    for (int i = 0; i < 3; ++i) {
        float lo, hi; Road::BandRange(kMin, kMax, i, lo, hi);
        float prev = 0;
        for (float v = 0.0f; v <= 80.0f; v += 0.5f) {
            const float f = Road::BandPitch(v, kMin, kMax, i);
            CHECK(f >= lo - 1e-3f && f <= hi + 1e-3f);
            CHECK(f >= prev - 1e-4f);                                           // never falls as the car speeds up
            prev = f;
        }
        CHECK(Road::BandPitch(14.0f, kMin, kMax, i) < Road::BandPitch(18.0f, kMin, kMax, i) || Road::BandPitch(14.0f, kMin, kMax, i) == lo);
    }
    CHECK(Road::BandPitch(NAN, kMin, kMax, 1) > 0 && Road::BandPitch(-5.0f, kMin, kMax, 1) > 0 && std::isfinite(Road::BandPitch(INFINITY, kMin, kMax, 2)));
}

// ---------------------------------------------------------------- what the contact patch lets through
TEST(the_contact_patch_lets_long_bumps_through_and_stops_short_ones) {
    CHECK(Road::PatchGain(30.0f, 60.0f) > 0.95f);                      // 2 m bumps against a 13 cm patch
    CHECK(near(Road::PatchGain(50.0f, 10.0f), 0.436f, 0.01f));         // 20 cm bumps: part way
    CHECK(Road::PatchGain(100.0f, 5.0f) == Road::kPatchMin);           // 5 cm bumps: gone (floor)
    CHECK(Road::PatchGain(50.0f, 0.0f) == Road::kPatchMin);
}
TEST(the_patch_cuts_more_at_low_speed_and_at_high_pitch) {
    for (float hz = 25; hz <= 100; hz += 5) { float prev = 0; for (float v = 1; v <= 60; v += 1) { const float g = Road::PatchGain(hz, v); CHECK(g >= prev - 1e-5f && g >= Road::kPatchMin && g <= 1.0f); prev = g; } }
    for (float v = 3; v <= 40; v += 3) CHECK(Road::PatchGain(30.0f, v) >= Road::PatchGain(90.0f, v));
}
TEST(the_patch_is_the_reports_numbers_for_a_13_cm_tyre) {
    // first null at speed / patch length: 38 Hz at 5 m/s, 77 Hz at 10 m/s, 154 Hz at 20 m/s
    CHECK(Road::PatchGain(38.0f, 5.0f) < 0.1f || Road::PatchGain(38.0f, 5.0f) == Road::kPatchMin);
    CHECK(Road::PatchGain(77.0f, 10.0f) < 0.1f);
    CHECK(Road::PatchGain(100.0f, 40.0f) > 0.8f);                      // at 144 km/h the whole wheel band passes
    CHECK(Road::PatchGain(25.0f, 40.0f) > 0.97f);
}

// ---------------------------------------------------------------- how the ground spreads the strength
TEST(the_ground_moves_the_strength_between_bands_without_changing_how_much_there_is) {
    const float hzs[] = {0.3f, 0.45f, 0.8f, 1.0f, 1.34f, 1.7f, 2.0f};
    for (float h : hzs) {
        float hz[3]; for (int b = 0; b < 3; ++b) hz[b] = Road::BandPitch(20.0f, kMin, kMax, b);
        float sh[3]; Road::BandShares(withHz(h), hz, 1000.0f, 50.0f, sh);       // a huge speed: the contact patch lets everything through
        CHECK(near(energy(sh), 1.0f, 0.01f));
    }
}
TEST(soft_ground_is_low_loose_ground_is_flat_and_glaze_is_bright) {
    float hz[3]; for (int b = 0; b < 3; ++b) hz[b] = Road::BandPitch(20.0f, kMin, kMax, b);
    float soft[3], plain[3], loose[3], glaze[3];
    Road::BandShares(withHz(0.45f), hz, 1000.0f, 50.0f, soft);
    Road::BandShares(withHz(1.0f), hz, 1000.0f, 50.0f, plain);
    Road::BandShares(withHz(1.34f), hz, 1000.0f, 50.0f, loose);
    Road::BandShares(withHz(1.7f), hz, 1000.0f, 50.0f, glaze);
    CHECK(soft[0] > 2.0f * soft[2] && soft[0] > soft[1]);                // the low band dominates
    CHECK(plain[0] > plain[2] && plain[0] < 2.0f * plain[2]);            // a mild lean to the low end
    CHECK(near(loose[0], loose[2], 0.1f));                                // flat
    CHECK(glaze[2] > glaze[0]);                                           // the high band leads
    CHECK(soft[0] > plain[0] && plain[0] > loose[0] && loose[0] > glaze[0]);
}
TEST(low_speed_takes_the_high_bands_away) {
    float hz[3], fast[3], slow[3];
    for (int b = 0; b < 3; ++b) hz[b] = Road::BandPitch(20.0f, kMin, kMax, b);
    Road::BandShares(withHz(1.0f), hz, 40.0f, 50.0f, fast);
    Road::BandShares(withHz(1.0f), hz, 6.0f, 50.0f, slow);
    CHECK(energy(slow) < energy(fast));
    CHECK(slow[2] / fast[2] < slow[0] / fast[0]);                          // the high band loses most
    CHECK(slow[0] > 0.0f && slow[1] > 0.0f && slow[2] > 0.0f);             // none is ever completely dead
}
TEST(shares_stay_finite_and_never_negative_for_bad_input) {
    const float bad[] = {NAN, -3.0f, INFINITY, 0.0f, 1e9f};
    float hz[3] = {30.0f, 50.0f, 80.0f};
    for (float h : bad) for (float v : bad) {
        float sh[3]; Road::BandShares(withHz(h), hz, v, 50.0f, sh);
        for (int b = 0; b < 3; ++b) CHECK(std::isfinite(sh[b]) && sh[b] >= 0.0f);
    }
}

// ---------------------------------------------------------------- the plan for both sides
TEST(each_side_gets_the_character_of_its_own_ground) {
    Road::SideIn s[2] = {side(0.45f, 1000.0f), side(1.7f, 1000.0f)};      // left on something soft, right on glaze
    const Road::Targets t = Road::Plan(s, 40.0f, kMin, kMax);
    CHECK(t.share[0][0] > t.share[0][2]);
    CHECK(t.share[1][2] > t.share[1][0]);
}
TEST(the_pitch_of_the_bands_is_shared_and_follows_speed) {
    Road::SideIn s[2] = {side(1.0f, 500.0f), side(1.0f, 500.0f)};
    const Road::Targets a = Road::Plan(s, 12.0f, kMin, kMax), b = Road::Plan(s, 18.0f, kMin, kMax);
    for (int i = 0; i < 3; ++i) CHECK(b.hz[i] >= a.hz[i] && b.hz[i] <= kMax && a.hz[i] >= kMin);
    CHECK(b.hz[1] > a.hz[1]);
}
TEST(ridged_ground_locks_the_band_that_holds_the_ridge_pitch_and_makes_it_the_strongest) {
    Surf::Feel cobbles = Surf::kNeutral; cobbles.ridge = 0.12f; cobbles.ridgeMix = 1.0f;
    Road::SideIn s[2]; s[0].fe = cobbles; s[1].fe = cobbles; s[0].strength = s[1].strength = 800.0f;
    const float speed = 6.0f;                                              // 6 m/s over 12 cm ridges = 50 Hz
    const Road::Targets t = Road::Plan(s, speed, kMin, kMax);
    CHECK(near(t.hz[1], 50.0f, 0.1f));                                     // the middle band sits exactly on the ridge pitch
    CHECK(t.share[0][1] > t.share[0][0] && t.share[0][1] > t.share[0][2]);
    // a faster car folds the ridge pitch down by octaves, into the high band's third
    const Road::Targets f = Road::Plan(s, 20.0f, kMin, kMax);
    CHECK(f.hz[2] >= 63.0f && f.hz[2] <= 100.0f && f.share[0][2] > f.share[0][0]);
    // no ridges: the pitch stays where speed puts it
    Road::SideIn n[2] = {side(1.0f, 800.0f), side(1.0f, 800.0f)};
    const Road::Targets p = Road::Plan(n, speed, kMin, kMax);
    CHECK(near(p.hz[1], Road::BandPitch(speed, kMin, kMax, 1), 1e-3f));
}
TEST(ridges_on_a_silent_side_do_not_move_the_pitch) {
    Surf::Feel cobbles = Surf::kNeutral; cobbles.ridge = 0.12f; cobbles.ridgeMix = 1.0f;
    Road::SideIn s[2]; s[0].fe = cobbles; s[0].strength = 0.0f; s[1] = side(1.0f, 600.0f);
    const Road::Targets t = Road::Plan(s, 6.0f, kMin, kMax);
    for (int b = 0; b < 3; ++b) CHECK(near(t.hz[b], Road::BandPitch(6.0f, kMin, kMax, b), 1e-3f));
}

// ---------------------------------------------------------------- putting the two sides together
TEST(two_sides_never_cancel_however_alike_they_are) {
    Road::SideIn s[2] = {side(1.0f, 700.0f), side(1.0f, 700.0f)};
    const Road::Targets t = Road::Plan(s, 25.0f, kMin, kMax);
    float mag[3]; Road::Combine(s, t.share, kOne, mag);
    for (int b = 0; b < 3; ++b) CHECK(mag[b] > 0.0f);
    float one[3]; Road::SideIn s1[2] = {side(1.0f, 700.0f), side(1.0f, 0.0f)};
    Road::Combine(s1, t.share, kOne, one);
    for (int b = 0; b < 3; ++b) CHECK(mag[b] > one[b]);                    // the second side adds, it never takes away
}
TEST(two_equal_sides_add_as_energy_not_as_strength) {
    Road::SideIn s[2] = {side(1.0f, 600.0f), side(1.0f, 600.0f)};
    const float sh[2][3] = {{0.5f, 0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}};
    float mag[3]; Road::Combine(s, sh, kOne, mag);
    CHECK(near(mag[0], Road::kSineEq * 600.0f * 0.5f * std::sqrt(2.0f), 0.1f));
}
TEST(a_stronger_side_or_a_bigger_jitter_never_makes_a_band_weaker) {
    Road::SideIn s[2] = {side(1.0f, 300.0f), side(1.0f, 300.0f)};
    const Road::Targets t = Road::Plan(s, 25.0f, kMin, kMax);
    float base[3]; Road::Combine(s, t.share, kOne, base);
    s[0].strength = 600.0f;
    float more[3]; Road::Combine(s, t.share, kOne, more);
    float jit[2][3] = {{2, 2, 2}, {1, 1, 1}};
    float jm[3]; Road::Combine(s, t.share, jit, jm);
    for (int b = 0; b < 3; ++b) CHECK(more[b] > base[b] && jm[b] > more[b]);
}
TEST(the_full_road_stays_in_the_same_range_as_a_single_sine_of_that_strength) {
    // shares that carry all the energy (squares add to 1) and a side at full scale: the effect magnitudes together stay near one sine
    Road::SideIn s[2] = {side(1.0f, 1000.0f), side(1.0f, 0.0f)};
    float hz[3]; for (int b = 0; b < 3; ++b) hz[b] = Road::BandPitch(20.0f, kMin, kMax, b);
    Road::Targets t; for (int b = 0; b < 3; ++b) t.hz[b] = hz[b];
    Road::BandShares(s[0].fe, hz, 1000.0f, 50.0f, t.share[0]);
    float mag[3]; Road::Combine(s, t.share, kOne, mag);
    const float rms = std::sqrt(0.5f * (mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]));
    CHECK(near(rms, Road::kSineEq * 1000.0f * 0.7071f, 5.0f));
}
TEST(combine_stays_finite_and_never_negative_for_bad_input) {
    const float bad[] = {NAN, -5.0f, INFINITY, 0.0f, 1e9f};
    for (float a : bad) for (float b : bad) {
        Road::SideIn s[2]; s[0].strength = a; s[1].strength = b;
        float sh[2][3] = {{a, b, a}, {b, a, b}}, jit[2][3] = {{b, a, b}, {a, b, a}}, mag[3];
        Road::Combine(s, sh, jit, mag);
        for (int i = 0; i < 3; ++i) CHECK(std::isfinite(mag[i]) && mag[i] >= 0.0f);
        const Road::Targets t = Road::Plan(s, a, b, 100.0f);
        for (int i = 0; i < 3; ++i) CHECK(std::isfinite(t.hz[i]) && t.hz[i] > 0.0f);
    }
}

// ---------------------------------------------------------------- with the game's real grounds
TEST(every_ground_in_the_game_gives_a_finite_plan_with_sensible_bands) {
    int n = 0;
    for (const rsa::MaterialInfo& m : rsa::kMaterials) {
        Road::Ground g;
        if (!Road::GroundOfId(m.id, g)) continue;
        for (float act : {0.0f, 0.5f, 1.0f}) for (float v : {2.0f, 8.0f, 20.0f, 45.0f}) {
            Road::Drive d; d.activity = act;
            Road::SideIn s[2]; s[0].fe = s[1].fe = Road::Derive(g, d); s[0].strength = s[1].strength = 500.0f;
            const Road::Targets t = Road::Plan(s, v, kMin, kMax);
            for (int b = 0; b < 3; ++b) {
                CHECK(std::isfinite(t.hz[b]) && t.hz[b] >= kMin - 0.01f && t.hz[b] <= kMax + 0.01f);
                CHECK(std::isfinite(t.share[0][b]) && t.share[0][b] >= 0.0f && t.share[0][b] < 3.0f);
            }
        }
        ++n;
    }
    CHECK(n > 90);
}
TEST(gravel_is_flatter_than_snow_and_ice_is_brighter_than_both) {
    auto gnd = [](const char* name) { for (const rsa::MaterialInfo& m : rsa::kMaterials) if (std::string(m.name) == name) { Road::Ground g; Road::GroundOfId(m.id, g); return g; } return Road::Ground(); };
    auto bands = [&](const char* name) {
        Road::Drive d; d.activity = 0.6f;
        Road::SideIn s[2]; s[0].fe = s[1].fe = Road::Derive(gnd(name), d); s[0].strength = s[1].strength = 500.0f;
        return Road::Plan(s, 40.0f, kMin, kMax);
    };
    const Road::Targets gr = bands("Gravel_Coarse_Deep_Dry"), sn = bands("SnowOnGravel_Deep"), ic = bands("SnowOnIce_Shallow");
    const float leanGr = gr.share[0][0] / gr.share[0][2], leanSn = sn.share[0][0] / sn.share[0][2], leanIc = ic.share[0][0] / ic.share[0][2];
    CHECK(leanSn > leanGr);                    // snow leans to the low end more than gravel
    CHECK(leanGr > leanIc);                    // ice leans to the high end
}

int main() {
    for (const Reg& r : regs()) { int before = g_fail; r.fn(); std::printf("[%s] %s\n", g_fail == before ? " ok " : "FAIL", r.name); }
    std::printf("\n%zu tests, %d checks, %d failures\n", regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
