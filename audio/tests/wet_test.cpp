// Tests for how damp and wet ground change the tyre sounds, and for the audio taking its surface from the plugin's reader. Self-contained;
// build and run from this folder:
//   g++ -std=c++17 -O1 -I.. wet_test.cpp -o wet_test && ./wet_test
#include <cstdio>
#include <functional>
#include <random>
#include "core.hpp"
#include "audio.hpp"

using namespace ffx;

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

// Every set is a class set; `native` makes the sets pretend to be recordings of wet ground.
struct Sets : ISoundSets {
    bool native = false;
    int count() const override { return 4; }
    int forMaterial(int) const override { return -1; }
    int forClass(int c) const override { return c >= 0 && c < 4 ? c : 0; }
    float volume(int) const override { return 1.f; }
    bool wetNative(int) const override { return native; }
};
struct Sink : IAudioSink {
    Sets sets;
    float hiss[4] = {0, 0, 0, 0}, tone[4] = {0, 0, 0, 0}, loop[4] = {0, 0, 0, 0}; float setGain[4] = {0, 0, 0, 0};
    int stones = 0, wetStones = 0;
    void setWheelLoop(int w, int set, float gl, float gr, float) override { if (w == 0) setGain[set] = gl + gr; if (set == 0) loop[w] = 0; loop[w] += gl + gr; }
    void setSlideLoop(int, int, float, float, float) override {}
    void setSquealLoop(int, int, float, float, float) override {}
    void setRumble(float, float) override {}
    void setWheelTone(int w, float t) override { tone[w] = t; }
    void setWetHiss(int w, float gl, float gr, float) override { hiss[w] = gl + gr; }
    void trigger(SoundEvent k, int, float, float, float) override { if (k == SoundEvent::StoneHit) ++stones; if (k == SoundEvent::WetStone) ++wetStones; }
    void silence() override {}
    const ISoundSets* soundSets() const override { return &sets; }
};
// What the tyre reader says is under the tyres: one id (all four), and optionally the plugin's own surface info.
struct Reader : ISurfaceMemory {
    int id = 1; bool haveSurface = false; SurfaceModel::Tyre tyre;
    bool readMaterials(int out[4]) override { for (int i = 0; i < 4; ++i) out[i] = id; return true; }
    bool readSurface(SurfaceModel::Tyre out[4]) override { if (!haveSurface) return false; for (int i = 0; i < 4; ++i) out[i] = tyre; return true; }
    bool configured() const override { return true; }
    std::string statusText() const override { return "fake"; }
};
static TelemetryFrame frame(double t, double groundMs, float strut, float throttle) {
    TelemetryFrame f; f.time = t; f.speed = (float)(groundMs * 3.6 * 1.609); f.throttle = throttle; f.velSurge = (float)groundMs;
    for (auto& w : f.wheels) w.strutForce = strut;
    return f;
}
// Drives a steady 25 m/s (90 km/h) for 20 s on rough ground; returns the sink's final state.
static Sink drive(Reader& r, Settings st = Settings(), float wetLevel = 1.0f, bool native = false, double seconds = 20.0) {
    Sink sink; sink.sets.native = native; AudioController c(st, sink, 5); c.setSurfaceMemory(&r); c.setWetLevel(wetLevel);
    std::mt19937 g(9); std::normal_distribution<float> n(0, 1); double t = 0;
    for (int i = 0, N = (int)(seconds * 60); i < N; ++i) { t += 1.0 / 60; c.onFrame(frame(t, 25.0, -3500.f * (1.f + 0.4f * n(g)), 0.5f)); }
    return sink;
}

TEST(dry_gravel_has_no_hiss_and_wet_gravel_has_some) {
    Reader dry, wet; dry.id = 1; wet.id = 3;
    const Sink a = drive(dry), b = drive(wet);
    CHECK(a.hiss[0] == 0.0f); CHECK(b.hiss[0] > 0.0f);
}
TEST(damp_is_between_dry_and_wet) {
    Reader d, w; d.id = 2; w.id = 3;
    const Sink a = drive(d), b = drive(w);
    CHECK(a.hiss[0] > 0.0f && a.hiss[0] < b.hiss[0]);
    Reader dry; dry.id = 1; CHECK(a.tone[0] < drive(dry).tone[0] && a.tone[0] > b.tone[0]);
}
TEST(wet_gravel_is_duller_and_a_little_quieter_than_dry_gravel) {
    Reader dry, wet; dry.id = 1; wet.id = 3;
    const Sink a = drive(dry), b = drive(wet);
    CHECK(b.tone[0] < a.tone[0] - 0.1f);
    CHECK(b.setGain[1] < a.setGain[1]); CHECK(b.setGain[1] > 0.5f * a.setGain[1]);
}
TEST(wet_tarmac_gets_the_hiss_but_not_duller) {
    Reader dry, wet; dry.id = 37; wet.id = 39;
    const Sink a = drive(dry), b = drive(wet);
    CHECK(b.hiss[0] > 0.0f && a.hiss[0] == 0.0f);
    CHECK(std::fabs(b.tone[0] - a.tone[0]) < 0.02f);
}
TEST(a_real_wet_recording_gets_no_treatment) {
    Reader dry, wet; dry.id = 1; wet.id = 3;
    const Sink a = drive(dry, Settings(), 1.0f, true), b = drive(wet, Settings(), 1.0f, true);
    CHECK(b.hiss[0] == 0.0f); CHECK(std::fabs(b.tone[0] - a.tone[0]) < 1e-4f); CHECK(std::fabs(b.setGain[1] - a.setGain[1]) < 1e-5f);
}
TEST(the_setting_and_the_menu_level_turn_it_off) {
    Reader wet; wet.id = 3; Reader dry; dry.id = 1;
    Settings off; off.wetTreatment = false;
    const Sink s = drive(wet, off), z = drive(wet, Settings(), 0.0f), ref = drive(dry);
    CHECK(s.hiss[0] == 0.0f && z.hiss[0] == 0.0f);
    CHECK(std::fabs(s.tone[0] - ref.tone[0]) < 1e-4f && std::fabs(z.tone[0] - ref.tone[0]) < 1e-4f);
    const Sink twice = drive(wet, Settings(), 2.0f), once = drive(wet);
    CHECK(twice.hiss[0] >= once.hiss[0]); CHECK(twice.tone[0] <= once.tone[0]);
}
TEST(each_amount_has_its_own_setting) {
    Reader wet; wet.id = 3;
    Settings noHiss; noHiss.wetHiss = 0.0; Settings noTone; noTone.wetTone = 0.0;
    const Sink a = drive(wet, noHiss), b = drive(wet, noTone), ref = drive(wet);
    CHECK(a.hiss[0] == 0.0f && a.tone[0] == ref.tone[0]);
    CHECK(b.hiss[0] == ref.hiss[0] && b.tone[0] > ref.tone[0]);
}
TEST(the_hiss_stops_when_the_car_stops) {
    Reader wet; wet.id = 3; Sink sink; Settings st; AudioController c(st, sink, 5); c.setSurfaceMemory(&wet);
    std::mt19937 g(9); std::normal_distribution<float> n(0, 1); double t = 0;
    for (int i = 0; i < 600; ++i) { t += 1.0 / 60; c.onFrame(frame(t, 25.0, -3500.f * (1.f + 0.4f * n(g)), 0.5f)); }
    CHECK(sink.hiss[0] > 0.0f);
    for (int i = 0; i < 300; ++i) { t += 1.0 / 60; c.onFrame(frame(t, 0.0, -3500.f, 0.0f)); }
    CHECK(sink.hiss[0] == 0.0f);
}
TEST(wet_gravel_throws_fewer_stones_and_some_splash) {
    Reader dry, wet; dry.id = 1; wet.id = 3;
    const Sink a = drive(dry, Settings(), 1.0f, false, 40.0), b = drive(wet, Settings(), 1.0f, false, 40.0);
    CHECK(a.stones > 50); CHECK(b.stones < a.stones); CHECK(b.stones > 10);
    CHECK(a.wetStones == 0); CHECK(b.wetStones > 0);
}
TEST(wet_stones_are_not_added_to_a_real_wet_recording) {
    Reader wet; wet.id = 3;
    const Sink n = drive(wet, Settings(), 1.0f, true, 40.0);
    CHECK(n.wetStones == 0); CHECK(n.stones > 0);
}
TEST(the_plugins_reader_decides_the_class_when_it_differs_from_the_materials_table) {
    Reader r; r.id = 1;                                        // the table says gravel (class 1)
    const Sink plain = drive(r);
    CHECK(plain.setGain[1] > plain.setGain[0]);
    r.haveSurface = true; r.tyre.id = 1; r.tyre.audioClass = 0; r.tyre.loose = 0.0f;   // the plugin's reader says tarmac: hard ground
    const Sink hard = drive(r);
    CHECK(hard.setGain[0] > hard.setGain[1]);
}
TEST(the_plugins_reader_decides_how_wet_it_is) {
    Reader r; r.id = 1;                                        // the table says dry
    r.haveSurface = true; r.tyre.id = 1; r.tyre.audioClass = 1; r.tyre.wet = 2; r.tyre.loose = 0.9f;
    const Sink s = drive(r);
    CHECK(s.hiss[0] > 0.0f);
}
TEST(without_the_plugins_reader_the_table_still_works) {
    Reader r; r.id = 3; r.haveSurface = false;
    CHECK(drive(r).hiss[0] > 0.0f);
}

int main() {
    for (const Reg& r : regs()) { int before = g_fail; r.fn(); std::printf("[%s] %s\n", g_fail == before ? " ok " : "FAIL", r.name); }
    std::printf("\n%zu tests, %d checks, %d failures\n", regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
