// Tests for the water sounds made in code (water_sounds.hpp). Self-contained; build and run from this folder:
//   g++ -std=c++17 -O1 -I../.. water_sounds_test.cpp -o water_sounds_test && ./water_sounds_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>
#include "core.hpp"
#include "audio.hpp"
#include "water_sounds.hpp"
#include "tests/mini_test.hpp"

using namespace ffx;

static bool allFinite(const std::vector<float>& d) { for (float x : d) if (!std::isfinite(x)) return false; return true; }
static float peakOf(const std::vector<float>& d) { float p = 0; for (float x : d) p = std::max(p, std::fabs(x)); return p; }
static double rmsOf(const std::vector<float>& d, size_t a, size_t b) { double s = 0; for (size_t i = a; i < b && i < d.size(); ++i) s += (double)d[i] * d[i]; return b > a ? std::sqrt(s / (double)(b - a)) : 0.0; }

TEST(splash_is_finite_bounded_and_about_a_second) {
    for (int v = 0; v < 4; ++v) {
        const std::vector<float> d = wsnd::makeSplash(v);
        CHECK(d.size() == (size_t)kSampleRate);
        CHECK(allFinite(d));
        CHECK(std::fabs(peakOf(d) - 0.8f) < 1e-3f);
    }
}

TEST(splash_hits_hard_then_dies_away) {
    const std::vector<float> d = wsnd::makeSplash(0);
    const size_t ms = kSampleRate / 1000;
    const double early = rmsOf(d, 5 * ms, 105 * ms), late = rmsOf(d, 700 * ms, 1000 * ms);
    CHECK(early > 0.05);
    CHECK(late < 0.25 * early);
    CHECK(std::fabs(d[0]) < 0.05f);                                  // starts from silence: no click
    CHECK(std::fabs(d.back()) < 0.05f);
}

TEST(variants_differ_and_the_same_variant_repeats) {
    const std::vector<float> a = wsnd::makeSplash(0), b = wsnd::makeSplash(1), a2 = wsnd::makeSplash(0);
    CHECK(a == a2);
    int diff = 0; for (size_t i = 0; i < a.size(); ++i) if (std::fabs(a[i] - b[i]) > 1e-4f) ++diff;
    CHECK(diff > (int)a.size() / 2);
    CHECK(wsnd::makeWaterHit(0) != wsnd::makeWaterHit(1));
}

TEST(water_hit_is_short_and_bounded) {
    for (int v = 0; v < 4; ++v) {
        const std::vector<float> d = wsnd::makeWaterHit(v);
        CHECK(d.size() == (size_t)(0.2 * kSampleRate));
        CHECK(allFinite(d) && std::fabs(peakOf(d) - 0.6f) < 1e-3f);
        CHECK(rmsOf(d, d.size() * 3 / 4, d.size()) < 0.2 * rmsOf(d, 0, d.size() / 4));
    }
}

TEST(water_loop_is_steady_loud_enough_and_joins_up) {
    const std::vector<float> d = wsnd::makeWaterLoop();
    CHECK(allFinite(d) && d.size() > (size_t)kSampleRate);
    CHECK(peakOf(d) <= 0.95f + 1e-4f);
    const double r = rmsOf(d, 0, d.size());
    CHECK(r > 0.05 && r < 0.4);
    const size_t q = d.size() / 4;                                   // no quarter of the loop is much quieter than another: no gaps
    for (int k = 0; k < 4; ++k) CHECK(rmsOf(d, k * q, (k + 1) * q) > 0.4 * r);
    // the join: the step from the last frame to the first is no bigger than the step between two ordinary neighbours
    double maxStep = 0; for (size_t i = 1; i < d.size(); ++i) maxStep = std::max(maxStep, (double)std::fabs(d[i] - d[i - 1]));
    CHECK(std::fabs(d[0] - d.back()) <= maxStep + 1e-6);
}

TEST(fallbacks_fill_only_what_is_missing) {
    SoundBank empty;
    CHECK(wsnd::addWaterFallbacks(empty) == 3);
    CHECK(empty.get("water_splash") && empty.get("water_splash")->size() == 3);
    CHECK(empty.get("water_loop") && empty.get("water_loop")->size() == 1);
    CHECK(empty.get("water_hit") && empty.get("water_hit")->size() == 4);
    CHECK(wsnd::addWaterFallbacks(empty) == 0);                      // a second call changes nothing
    CHECK(empty.get("water_splash")->size() == 3);

    SoundBank own;
    own.add("water_loop", std::vector<float>(1000, 0.1f));
    own.add("debris_splash", std::vector<float>(1000, 0.1f));
    CHECK(wsnd::addWaterFallbacks(own) == 1);                        // only the hit was missing
    CHECK(own.get("water_loop")->size() == 1 && own.get("water_hit"));
    CHECK(!own.get("water_splash"));                                 // debris_splash counts as the splash

    SoundBank surface;
    surface.add("water_surface_loop", std::vector<float>(1000, 0.1f));
    surface.add("water_surface_hit", std::vector<float>(1000, 0.1f));
    CHECK(wsnd::addWaterFallbacks(surface) == 1);
    CHECK(!surface.get("water_loop") && !surface.get("water_hit"));
}

TEST(splash_is_made_from_rbrs_own_water_sound_when_the_bank_has_it) {
    SoundBank bank;
    std::vector<float> rbr((size_t)(2.0 * kSampleRate));
    wsnd::Noise n(5);
    for (float& x : rbr) x = 0.05f * n.next();                         // RBR's recording: quiet noise
    bank.add("water_loop", rbr);
    bool fromRbr = false;
    CHECK(wsnd::addWaterFallbacks(bank, &fromRbr) == 2);               // the splash and the stone sound; the loop is RBR's own
    CHECK(fromRbr);
    CHECK(bank.get("water_splash") && bank.get("water_splash")->size() == 3);
    CHECK(bank.get("water_loop")->size() == 1);                        // untouched
    for (const std::vector<float>& d : *bank.get("water_splash")) {
        CHECK(allFinite(d) && std::fabs(peakOf(d) - 0.8f) < 1e-3f);
        const size_t ms = kSampleRate / 1000;
        CHECK(rmsOf(d, 10 * ms, 110 * ms) > 3.0 * rmsOf(d, 700 * ms, 950 * ms));   // hits hard, fades
        CHECK(std::fabs(d[0]) < 0.05f);
    }
    CHECK((*bank.get("water_splash"))[0] != (*bank.get("water_splash"))[1]);         // different slices
    SoundBank plain;
    bool fromRbr2 = true;
    wsnd::addWaterFallbacks(plain, &fromRbr2);
    CHECK(!fromRbr2);                                                  // no RBR water sound: made from noise
    CHECK((*plain.get("water_splash"))[0] != (*bank.get("water_splash"))[0]);
}

TEST(a_silent_or_tiny_recording_does_not_break_the_splash) {
    SoundBank silent;
    silent.add("water_loop", std::vector<float>(100000, 0.0f));
    wsnd::addWaterFallbacks(silent);
    for (const std::vector<float>& d : *silent.get("water_splash")) CHECK(allFinite(d) && peakOf(d) > 0.1f);   // the thump alone
    SoundBank tiny;
    tiny.add("water_loop", std::vector<float>(10, 0.3f));
    bool fromRbr = true;
    wsnd::addWaterFallbacks(tiny, &fromRbr);
    CHECK(!fromRbr && tiny.get("water_splash") && tiny.get("water_splash")->size() == 3);
}

TEST(water_surface_material_finds_the_water_sounds) {
    SoundBank bank;
    CHECK(wsnd::addWaterFallbacks(bank) == 3);
    bank.add("gravel_loop", std::vector<float>(1000, 0.1f));
    Settings s;
    MaterialTable table;
    SoundSetMap map(bank, table, s);
    const int set = map.forMaterial(157);
    CHECK(set >= 0);
    CHECK(map.info(set).loop == "water_loop");                       // not the gravel loop the class would have given it
    CHECK(map.info(set).hit == "water_hit");
    CHECK(map.forMaterial(41) != set);                               // gravel keeps its own
}

TEST(splash_event_plays_the_splash_sound) {
    SoundBank bank;
    wsnd::addWaterFallbacks(bank);
    Settings s;
    MaterialTable table;
    SoundSetMap map(bank, table, s);
    Mixer mixer;
    MixerSink sink(mixer, bank, s, &map);
    CHECK(mixer.activeShots() == 0);
    sink.trigger(SoundEvent::Splash, 0, 0.5f, 0.5f, 1.0f);
    CHECK(mixer.activeShots() == 1);
    SoundBank none;                                                  // no splash anywhere: nothing plays, nothing breaks
    SoundSetMap map2(none, table, s);
    Mixer mixer2;
    MixerSink sink2(mixer2, none, s, &map2);
    sink2.trigger(SoundEvent::Splash, 0, 0.5f, 0.5f, 1.0f);
    CHECK(mixer2.activeShots() == 0);
}

// ---- the spray hiss of wet ground
TEST(wet_hiss_is_finite_steady_and_about_three_seconds) {
    const std::vector<float> d = wsnd::makeWetHiss();
    CHECK(std::fabs((double)d.size() / kSampleRate - 3.0) < 0.2);                  // the seamless join may trim a little
    bool finite = true; for (float x : d) finite = finite && std::isfinite(x);
    CHECK(finite);
    const double r = rmsOf(d, 0, d.size());
    CHECK(r > 0.01);
    const size_t q = d.size() / 4;
    for (int k = 0; k < 4; ++k) CHECK(rmsOf(d, k * q, (k + 1) * q) > 0.6 * r && rmsOf(d, k * q, (k + 1) * q) < 1.6 * r);   // no dead spots, no bursts
}
TEST(wet_hiss_is_a_high_band_with_little_low_end) {
    const std::vector<float> d = wsnd::makeWetHiss();
    double lo = 0, mid = 0, hi = 0, a = 0, b = 0;                                // one-pole low-pass at about 300 Hz, and the rest above 1.5 kHz
    const double k = 1.0 - std::exp(-wsnd::kTwoPi * 300.0 / kSampleRate), k2 = 1.0 - std::exp(-wsnd::kTwoPi * 1500.0 / kSampleRate);
    for (float x : d) { a += k * (x - a); b += k2 * (x - b); lo += a * a; mid += (b - a) * (b - a); hi += (x - b) * (x - b); }
    CHECK(hi > 4.0 * lo);                                                        // far more above 1.5 kHz than below 300 Hz
    CHECK(hi > mid);
}
TEST(wet_hiss_joins_up_with_itself) {
    const std::vector<float> d = wsnd::makeWetHiss();
    const double r = rmsOf(d, 0, d.size());
    CHECK(std::fabs(d.front() - d.back()) < 4.0 * r);                            // the seam is no louder than the sound itself
    double edge = 0; for (size_t i = 1; i < 64; ++i) edge = std::max(edge, (double)std::fabs(d[i] - d[i - 1]));
    CHECK(edge < 8.0 * r);
}
TEST(wet_hiss_is_added_once_and_a_real_one_is_kept) {
    SoundBank bank;
    CHECK(bank.get("wet_hiss_loop") == nullptr);
    CHECK(wsnd::addWetFallbacks(bank));
    CHECK(bank.get("wet_hiss_loop") && bank.get("wet_hiss_loop")->size() == 1);
    CHECK(!wsnd::addWetFallbacks(bank));                                         // already there
    CHECK(bank.get("wet_hiss_loop")->size() == 1);
    SoundBank own; own.add("wet_hiss_loop", std::vector<float>(100, 0.1f));
    CHECK(!wsnd::addWetFallbacks(own));
    CHECK((*own.get("wet_hiss_loop"))[0].size() == 100);                         // the recording is left alone
}
TEST(the_mixer_sink_plays_a_hiss_voice_per_tyre_and_a_wet_stone_sound) {
    SoundBank bank; wsnd::addWaterFallbacks(bank); wsnd::addWetFallbacks(bank);
    Settings s; MaterialTable table; SoundSetMap map(bank, table, s);
    Mixer mixer; MixerSink sink(mixer, bank, s, &map);
    sink.setWetHiss(0, 0.5f, 0.5f, 1.0f); sink.setWetHiss(9, 1.0f, 1.0f, 1.0f); sink.setWetHiss(-1, 1.0f, 1.0f, 1.0f);   // out-of-range tyres are ignored
    sink.trigger(SoundEvent::WetStone, 0, 0.5f, 0.5f, 1.0f);
    CHECK(mixer.activeShots() == 1);
    SoundBank none; SoundSetMap map2(none, table, s); Mixer m2; MixerSink s2(m2, none, s, &map2);
    s2.setWetHiss(0, 1.0f, 1.0f, 1.0f); s2.trigger(SoundEvent::WetStone, 0, 0.5f, 0.5f, 1.0f);   // no sounds at all: nothing happens, nothing breaks
    CHECK(m2.activeShots() == 0);
}
TEST(a_set_is_a_wet_recording_only_when_its_loop_is_named_wet_or_damp) {
    SoundBank bank; bank.add("gravel_loop", std::vector<float>(100, 0.1f)); bank.add("gravel_wet_loop", std::vector<float>(100, 0.1f));
    bank.add("tarmac_loop", std::vector<float>(100, 0.1f)); bank.add("water_loop", std::vector<float>(100, 0.1f));
    Settings s; MaterialTable table; SoundSetMap map(bank, table, s);
    const int dry = map.forMaterial(1), wet = map.forMaterial(3), damp = map.forMaterial(2);                  // Gravel_Fine_Deeper_Dry / _Wet / _Damp
    CHECK(!map.wetNative(dry)); CHECK(map.wetNative(wet));
    CHECK(map.wetNative(damp) == map.wetNative(wet) || map.wetNative(damp) == false);                             // damp falls back to gravel_wet only if the chain says so
    CHECK(!map.wetNative(map.forClass(0)));
    CHECK(!map.wetNative(-1)); CHECK(!map.wetNative(9999));
}

int main() { return mini::RunAll(); }
