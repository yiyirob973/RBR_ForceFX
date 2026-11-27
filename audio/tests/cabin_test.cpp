// Tests for the inside of the car: the camera view deciding how inside the sound is, the cabin filter, the interior noises made in code, and
// how the controller and the mixer put them together. Self-contained; build and run from this folder:
//   g++ -std=c++17 -O1 -I../.. cabin_test.cpp -o cabin_test && ./cabin_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <random>
#include "core.hpp"
#include "audio.hpp"
#include "cabin_sounds.hpp"
#include "tests/mini_test.hpp"

using namespace ffx;

static const float kNaN = std::numeric_limits<float>::quiet_NaN();

// ---------------------------------------------------------------------------------------------------- the view
TEST(camera_types_inside_the_car_are_the_cockpit_family) {
    CHECK(cabin::ViewOf(5, 3, 6) == 1); CHECK(cabin::ViewOf(3, 3, 6) == 1); CHECK(cabin::ViewOf(6, 3, 6) == 1);
    CHECK(cabin::ViewOf(0, 3, 6) == 0); CHECK(cabin::ViewOf(2, 3, 6) == 0); CHECK(cabin::ViewOf(7, 3, 6) == 0);
    CHECK(cabin::ViewOf(-1, 3, 6) == -1);
    CHECK(cabin::ViewOf(5, 5, 5) == 1 && cabin::ViewOf(4, 5, 5) == 0);                 // the range can be set
}
TEST(inside_starts_inside_and_moves_smoothly_to_outside_and_back) {
    cabin::Inside in; CHECK(in.Value() == 1.0f);
    float prev = 1.0f; float v = 1.0f;
    for (int i = 0; i < 30; ++i) { v = in.Step(0, 1.0 / 60); CHECK(v <= prev + 1e-6f); CHECK(prev - v < 0.2f); prev = v; }   // no jump
    CHECK(v < 0.8f && v > 0.0f);
    for (int i = 0; i < 300; ++i) v = in.Step(0, 1.0 / 60);
    CHECK(v < 0.001f);
    prev = v; for (int i = 0; i < 300; ++i) { v = in.Step(1, 1.0 / 60); CHECK(v >= prev - 1e-6f); prev = v; }
    CHECK(v > 0.999f);
}
TEST(a_view_that_is_not_known_counts_as_inside) {
    cabin::Inside in; for (int i = 0; i < 300; ++i) in.Step(0, 1.0 / 60);
    CHECK(in.Value() < 0.01f);
    for (int i = 0; i < 300; ++i) in.Step(-1, 1.0 / 60);
    CHECK(in.Value() > 0.99f);
}
TEST(inside_survives_bad_time_steps) {
    cabin::Inside in; for (int i = 0; i < 20; ++i) in.Step(0, (double)kNaN); in.Step(0, -1.0); in.Step(0, 1e9);
    CHECK(std::isfinite(in.Value()) && in.Value() >= 0.0f && in.Value() <= 1.0f);
}

// ---------------------------------------------------------------------------------------------------- the interior noises' levels
static cabin::In ci(float kph, float rpm, float thr = 0.5f, float rough = 0.3f, float inside = 1.0f, float level = 1.0f) {
    cabin::In i; i.speedKph = kph; i.rpm = rpm; i.throttle = thr; i.rough = rough; i.inside = inside; i.level = level; return i;
}
TEST(outside_the_car_the_interior_is_silent) {
    const cabin::Ambience a = cabin::Make(ci(100, 6000, 1, 1, 0.0f));
    CHECK(a.fan == 0.0f && a.wind == 0.0f && a.buzz == 0.0f);
}
TEST(the_fan_is_steady_and_present_even_when_stopped) {
    const cabin::Ambience a = cabin::Make(ci(0, 0)), b = cabin::Make(ci(100, 6000));
    CHECK(a.fan > 0.0f && a.fan == b.fan); CHECK(a.wind == 0.0f); CHECK(a.buzz == 0.0f);
}
TEST(the_wind_grows_with_the_speed) {
    float prev = -1; for (float k = 0; k <= 200; k += 25) { const float w = cabin::Make(ci(k, 3000)).wind; CHECK(w >= prev); prev = w; }
    CHECK(cabin::Make(ci(150, 3000)).wind > 5.0f * cabin::Make(ci(40, 3000)).wind);
}
TEST(the_buzz_follows_the_engine_speed_and_load) {
    CHECK(cabin::Make(ci(50, 1500)).buzz == 0.0f);                                     // idle: nothing buzzes
    CHECK(cabin::Make(ci(50, 4000)).buzz > 0.0f); CHECK(cabin::Make(ci(50, 6500)).buzz > cabin::Make(ci(50, 4000)).buzz);
    CHECK(cabin::Make(ci(50, 5000, 1.0f)).buzz > cabin::Make(ci(50, 5000, 0.0f)).buzz);          // on the throttle
    CHECK(cabin::Make(ci(50, 5000, 0.5f, 1.0f)).buzz > cabin::Make(ci(50, 5000, 0.5f, 0.0f)).buzz);   // over rough ground
    cabin::In air = ci(50, 5000); air.airborne = 1.0f; CHECK(cabin::Make(air).buzz < cabin::Make(ci(50, 5000)).buzz);
}
TEST(the_buzz_pitch_follows_the_revs_within_limits) {
    CHECK(std::fabs(cabin::Make(ci(50, 3000)).buzzRate - 1.0f) < 1e-5f);
    CHECK(cabin::Make(ci(50, 6000)).buzzRate > cabin::Make(ci(50, 3000)).buzzRate);
    CHECK(cabin::Make(ci(50, 0)).buzzRate >= 0.3f); CHECK(cabin::Make(ci(50, 90000)).buzzRate <= 4.0f);
}
TEST(the_menu_level_scales_the_noises_and_zero_turns_them_off) {
    const cabin::Ambience one = cabin::Make(ci(100, 5000)), two = cabin::Make(ci(100, 5000, 0.5f, 0.3f, 1.0f, 2.0f)), none = cabin::Make(ci(100, 5000, 0.5f, 0.3f, 1.0f, 0.0f));
    CHECK(two.fan > one.fan && two.wind > one.wind && two.buzz > one.buzz);
    CHECK(none.fan == 0.0f && none.wind == 0.0f && none.buzz == 0.0f);
}
TEST(the_levels_stay_in_range_and_finite_for_any_input) {
    for (float k = -50; k <= 600; k += 130) for (float r = -100; r <= 50000; r += 9000) for (float l = -1; l <= 5; l += 2) {
        const cabin::Ambience a = cabin::Make(ci(k, r, 5.0f, 5.0f, 3.0f, l));
        CHECK(std::isfinite(a.fan) && std::isfinite(a.wind) && std::isfinite(a.buzz) && std::isfinite(a.buzzRate));
        CHECK(a.fan >= 0 && a.fan <= 0.5f && a.wind >= 0 && a.wind <= 0.5f && a.buzz >= 0 && a.buzz <= 0.5f);
    }
    const cabin::Ambience n = cabin::Make(ci(kNaN, kNaN, kNaN, kNaN, kNaN, kNaN)); CHECK(std::isfinite(n.fan) && std::isfinite(n.wind) && std::isfinite(n.buzz));
}
TEST(rattles_are_heard_only_inside_and_scale_with_the_event) {
    CHECK(cabin::RattleGain(1.0f, 0.0f, 1.0f) == 0.0f); CHECK(cabin::RattleGain(1.0f, 0.2f, 1.0f) == 0.0f);
    CHECK(cabin::RattleGain(1.0f, 1.0f, 1.0f) > cabin::RattleGain(0.3f, 1.0f, 1.0f));
    CHECK(cabin::RattleGain(5.0f, 1.0f, 2.0f) <= 0.8f); CHECK(cabin::RattleGain(1.0f, 1.0f, 0.0f) == 0.0f);
    CHECK(cabin::RattleGain(kNaN, 1.0f, 1.0f) == 0.0f);
}

// ---------------------------------------------------------------------------------------------------- the cabin filter
static std::vector<float> tone(double hz, int frames) {
    std::vector<float> d(2 * (size_t)frames);
    for (int i = 0; i < frames; ++i) d[2 * i] = d[2 * i + 1] = (float)(0.5 * std::sin(2.0 * 3.14159265358979 * hz * i / kSampleRate));
    return d;
}
static double rmsTail(const std::vector<float>& d) { double s = 0; size_t n = 0; for (size_t i = d.size() / 2; i < d.size(); ++i) { s += (double)d[i] * d[i]; ++n; } return std::sqrt(s / n); }
static void setupFilter(CabinFilter& f) { f.configure(true, cabin::kLowpassHz, cabin::kBassDb, cabin::kReverb, cabin::kBoomDb); f.setTrim(cabin::kTrim); }

TEST(outside_the_car_the_filter_leaves_the_sound_exactly_alone) {
    CabinFilter f; setupFilter(f); f.setInside(0.0f);
    std::vector<float> warm = tone(1000, 44100); f.process(warm.data(), 44100);          // let it settle outside
    std::vector<float> d = tone(3000, 4096), ref = d; f.process(d.data(), 4096);
    for (size_t i = 0; i < d.size(); ++i) CHECK(d[i] == ref[i]);
}
TEST(inside_the_car_the_top_end_is_cut_hard_and_the_low_end_is_kept) {
    CabinFilter f; setupFilter(f); f.setInside(1.0f);
    std::vector<float> hi = tone(5000, 44100), lo = tone(100, 44100), mid = tone(1000, 44100);
    const double hi0 = rmsTail(hi), lo0 = rmsTail(lo), mid0 = rmsTail(mid);
    f.process(hi.data(), 44100); CabinFilter g; setupFilter(g); g.setInside(1.0f); g.process(lo.data(), 44100); CabinFilter h; setupFilter(h); h.setInside(1.0f); h.process(mid.data(), 44100);
    CHECK(rmsTail(hi) < 0.08 * hi0);                         // 5 kHz: well over 20 dB down
    CHECK(rmsTail(mid) < 0.9 * mid0 && rmsTail(mid) > 0.2 * mid0);
    CHECK(rmsTail(lo) > 1.2 * lo0);                          // the low end is boosted: boomy
}
TEST(a_view_switch_crossfades_without_clicks) {
    CabinFilter f; setupFilter(f); f.setInside(1.0f);
    std::vector<float> warm = tone(300, 44100); f.process(warm.data(), 44100);
    f.setInside(0.0f);
    std::vector<float> d = tone(300, 44100); f.process(d.data(), 44100);
    double maxStep = 0; for (size_t i = 2; i < d.size(); i += 2) maxStep = std::max(maxStep, (double)std::fabs(d[i] - d[i - 2]));
    CHECK(maxStep < 0.08);                                   // a 300 Hz tone of 0.5 amplitude moves at most about 0.02 a sample; no jump
    CHECK(f.inside() < 0.01f);
}
TEST(a_disabled_filter_does_nothing_and_it_starts_inside_for_a_view_not_yet_known) {
    CabinFilter off; std::vector<float> d = tone(4000, 2048), ref = d; off.process(d.data(), 2048);
    for (size_t i = 0; i < d.size(); ++i) CHECK(d[i] == ref[i]);
    CabinFilter on; setupFilter(on); CHECK(on.inside() == 1.0f);
    on.setInside(kNaN); std::vector<float> e = tone(4000, 2048); on.process(e.data(), 2048); for (float x : e) CHECK(std::isfinite(x));
}

// ---------------------------------------------------------------------------------------------------- the sounds made in code
static double bandRms(const std::vector<float>& d, double lo, double hi) {         // energy between two one-pole corner frequencies
    double a = 0, b = 0, e = 0;
    const double ka = 1.0 - std::exp(-wsnd::kTwoPi * lo / kSampleRate), kb = 1.0 - std::exp(-wsnd::kTwoPi * hi / kSampleRate);
    for (float x : d) { a += ka * (x - a); b += kb * (x - b); const double y = b - a; e += y * y; }
    return std::sqrt(e / d.size());
}
TEST(the_fan_is_an_airy_midrange_hiss_with_little_below_and_above) {
    const std::vector<float> d = wsnd::makeCabinFan();
    CHECK(std::fabs((double)d.size() / kSampleRate - 4.0) < 0.2);
    bool fin = true; for (float x : d) fin = fin && std::isfinite(x); CHECK(fin);
    CHECK(bandRms(d, 300, 2500) > 2.0 * bandRms(d, 10, 60));
    CHECK(bandRms(d, 300, 2500) > 1.5 * bandRms(d, 8000, 20000));
}
TEST(the_wind_is_low_and_dull) {
    const std::vector<float> d = wsnd::makeCabinWind();
    CHECK(bandRms(d, 20, 800) > 3.0 * bandRms(d, 4000, 15000));
}
TEST(the_buzz_is_a_hundred_hertz_pulse_train_with_whole_periods) {
    const std::vector<float> d = wsnd::makeCabinBuzz();
    const size_t period = (size_t)std::llround(kSampleRate / 100.0);
    CHECK(d.size() % period == 0); CHECK(d.size() > 2 * kSampleRate);
    double corr1 = 0, corr0 = 0, corrHalf = 0;                                    // it repeats every period far better than at half a period
    for (size_t i = 0; i + period < d.size(); ++i) { corr0 += (double)d[i] * d[i]; corr1 += (double)d[i] * d[i + period]; corrHalf += (double)d[i] * d[i + period / 2]; }
    CHECK(corr1 > 0.5 * corr0); CHECK(corr1 > std::fabs(corrHalf) * 1.5);
}
TEST(rattles_are_short_decaying_knocks_and_there_are_variants) {
    for (unsigned v = 1; v <= 4; ++v) {
        const std::vector<float> d = wsnd::makeCabinRattle(v);
        CHECK(d.size() < 0.3 * kSampleRate);
        float pk = 0; for (float x : d) pk = std::max(pk, std::fabs(x)); CHECK(pk > 0.3f && pk <= 0.71f);
        double e1 = 0, e2 = 0; for (size_t i = 0; i < d.size() / 2; ++i) e1 += (double)d[i] * d[i]; for (size_t i = d.size() / 2; i < d.size(); ++i) e2 += (double)d[i] * d[i];
        CHECK(e1 > 3.0 * e2);
    }
    const std::vector<float> a = wsnd::makeCabinRattle(1), b = wsnd::makeCabinRattle(2); CHECK(a != b);
}
TEST(the_fallbacks_are_made_once_and_real_files_are_kept) {
    SoundBank bank; CHECK(wsnd::addCabinFallbacks(bank) == 4);
    CHECK(bank.get("cabin_fan_loop") && bank.get("cabin_wind_loop") && bank.get("cabin_buzz_loop") && bank.get("cabin_rattle")->size() == 4);
    CHECK(wsnd::addCabinFallbacks(bank) == 0);
    SoundBank own; own.add("cabin_fan_loop", std::vector<float>(50, 0.1f)); CHECK(wsnd::addCabinFallbacks(own) == 3);
    CHECK((*own.get("cabin_fan_loop"))[0].size() == 50);
}

// ---------------------------------------------------------------------------------------------------- the controller and the mixer
struct Sink : IAudioSink {
    float inside = -1, level = -1, fan = -1, wind = -1, buzz = -1, rate = -1; int calls = 0;
    void setWheelLoop(int, int, float, float, float) override {}
    void setSlideLoop(int, int, float, float, float) override {}
    void setSquealLoop(int, int, float, float, float) override {}
    void setRumble(float, float) override {}
    void trigger(SoundEvent, int, float, float, float) override {}
    void silence() override {}
    void setInside(float a) override { inside = a; ++calls; }
    void setCabinLevel(float l) override { level = l; }
    void setCabin(float f, float w, float b, float r) override { fan = f; wind = w; buzz = b; rate = r; }
};
struct Reader : ISurfaceMemory {
    bool readMaterials(int out[4]) override { for (int i = 0; i < 4; ++i) out[i] = 1; return true; }
    bool configured() const override { return true; }
    std::string statusText() const override { return "fake"; }
};
static TelemetryFrame frame(double t, double groundMs, float rpm, float thr = 0.5f) {
    TelemetryFrame f; f.time = t; f.speed = (float)(groundMs * 3.6 * 1.609); f.throttle = thr; f.velSurge = (float)groundMs; f.rpm = rpm;
    for (auto& w : f.wheels) w.strutForce = -3500.f;
    return f;
}
static void run(AudioController& c, double seconds, double ms, float rpm, double& t) { for (int i = 0, n = (int)(seconds * 60); i < n; ++i) { t += 1.0 / 60; c.onFrame(frame(t, ms, rpm)); } }

TEST(the_camera_view_drives_the_inside_amount_and_the_interior_noises) {
    Sink sink; Settings st; AudioController c(st, sink, 5); Reader r; c.setSurfaceMemory(&r); double t = 0;
    c.setCameraType(5); run(c, 3.0, 25.0, 5000.f, t);
    CHECK(sink.inside > 0.99f); CHECK(sink.fan > 0.0f); CHECK(sink.wind > 0.0f); CHECK(sink.buzz > 0.0f); CHECK(sink.rate > 1.0f);
    c.setCameraType(1); run(c, 3.0, 25.0, 5000.f, t);
    CHECK(sink.inside < 0.01f); CHECK(sink.fan == 0.0f && sink.wind == 0.0f && sink.buzz == 0.0f);   // the interior is gone once the crossfade has finished
    c.setCameraType(-1); run(c, 3.0, 25.0, 5000.f, t);                              // not known: inside
    CHECK(sink.inside > 0.99f && sink.fan > 0.0f);
}
TEST(the_wind_and_the_buzz_follow_the_car) {
    Sink sink; Settings st; AudioController c(st, sink, 5); Reader r; c.setSurfaceMemory(&r); double t = 0;
    c.setCameraType(5); run(c, 2.0, 8.0, 1500.f, t);
    const float w1 = sink.wind, b1 = sink.buzz;
    run(c, 2.0, 40.0, 6500.f, t);
    CHECK(sink.wind > 3.0f * w1); CHECK(sink.buzz > b1); CHECK(sink.rate > 1.5f);
}
TEST(the_menu_level_and_the_setting_scale_or_remove_the_noises) {
    Sink a; Settings st; AudioController c(st, a, 5); Reader r; c.setSurfaceMemory(&r); double t = 0; c.setCameraType(5); run(c, 2.0, 25.0, 5000.f, t);
    const float fan1 = a.fan; c.setCabinLevel(2.0f); run(c, 1.0, 25.0, 5000.f, t); CHECK(a.fan > 1.5f * fan1 && a.level > 1.9f);
    c.setCabinLevel(0.0f); run(c, 1.0, 25.0, 5000.f, t); CHECK(a.fan == 0.0f && a.wind == 0.0f && a.buzz == 0.0f);
    Sink b; Settings off; off.cabinAmbience = 0.0; AudioController d(off, b, 5); d.setSurfaceMemory(&r); d.setCameraType(5); double t2 = 0; run(d, 2.0, 25.0, 5000.f, t2);
    CHECK(b.fan == 0.0f && b.inside > 0.99f);                                         // the cabin sound itself is not a setting
}
TEST(a_different_range_of_camera_types_can_count_as_inside) {
    Sink sink; Settings st; st.cabinTypeLo = 8; st.cabinTypeHi = 9; AudioController c(st, sink, 5); Reader r; c.setSurfaceMemory(&r); double t = 0;
    c.setCameraType(5); run(c, 3.0, 25.0, 5000.f, t); CHECK(sink.inside < 0.01f);
    c.setCameraType(8); run(c, 3.0, 25.0, 5000.f, t); CHECK(sink.inside > 0.99f);
}
TEST(the_settings_file_sets_the_range_the_level_and_ignores_the_old_cabin_keys) {
    Settings s;
    applySetting(s, "CabinCameraMin", "2"); applySetting(s, "CabinCameraMax", "7"); applySetting(s, "CabinAmbience", "1.5");
    CHECK(s.cabinTypeLo == 2 && s.cabinTypeHi == 7 && std::fabs(s.cabinAmbience - 1.5) < 1e-9);
    applySetting(s, "CabinAmbience", "9"); CHECK(s.cabinAmbience == 2.0);
    applySetting(s, "CabinEffect", "0"); applySetting(s, "CabinLowpassHz", "100");        // old keys: nothing breaks, nothing changes
    CHECK(s.cabinTypeLo == 2);
}
TEST(the_rpm_is_read_from_the_packet_with_its_scale) {
    PacketLayout L; L.rpm = 136; L.rpmScale = 2.0f;
    std::vector<uint8_t> pk(L.NeededLength(), 0);
    const float v = 3000.0f; std::memcpy(pk.data() + 136, &v, 4);
    TelemetryFrame f; CHECK(parsePacket(pk.data(), pk.size(), 1.0, f, L)); CHECK(std::fabs(f.rpm - 6000.0f) < 1e-3f);
    L.rpm = -1; TelemetryFrame g; CHECK(parsePacket(pk.data(), pk.size(), 1.0, g, L)); CHECK(g.rpm == 0.0f);
}
TEST(the_mixer_plays_the_interior_voices_and_rattles_only_inside) {
    SoundBank bank; wsnd::addWaterFallbacks(bank); wsnd::addWetFallbacks(bank); wsnd::addCabinFallbacks(bank);
    bank.add("debris_impact", std::vector<float>(2000, 0.2f));
    Settings s; MaterialTable table; SoundSetMap map(bank, table, s); Mixer mixer; MixerSink sink(mixer, bank, s, &map);
    sink.setInside(0.0f); sink.trigger(SoundEvent::Impact, 0, 0.5f, 0.5f, 1.0f);
    CHECK(mixer.activeShots() == 1);                                                  // the impact only
    sink.setInside(1.0f); sink.trigger(SoundEvent::Impact, 0, 0.5f, 0.5f, 1.0f);
    CHECK(mixer.activeShots() == 3);                                                  // the impact and a rattle
    sink.trigger(SoundEvent::StoneHit, 0, 0.5f, 0.5f, 1.0f);
    CHECK(mixer.activeShots() <= 4);                                                  // a stone is not a rattle
    sink.setCabin(0.1f, 0.2f, 0.1f, 1.5f);
    std::vector<float> out(2 * 2048); for (int i = 0; i < 20; ++i) mixer.read(out.data(), 2048);
    float pk = 0; for (float x : out) pk = std::max(pk, std::fabs(x)); CHECK(pk > 0.001f);
    sink.setCabin(0.0f, 0.0f, 0.0f, 1.0f);
}

int main() { return mini::RunAll(); }
