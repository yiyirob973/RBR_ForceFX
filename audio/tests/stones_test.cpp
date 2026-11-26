// Tests for the telemetry-driven stone spray. Self-contained; build and run from this folder:
//   g++ -std=c++17 -O1 -I.. stones_test.cpp -o stones_test && ./stones_test
#include <cstdio>
#include <functional>
#include <random>
#include <filesystem>
#include <fstream>
#include "core.hpp"
#include "audio.hpp"

using namespace ffx;

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

struct FakeSink : IAudioSink {
    int stones = 0, leftStones = 0, rightStones = 0;
    float lastStoneGain = 0, maxStoneGain = 0;
    void setWheelLoop(int, int, float, float, float) override {}
    void setSlideLoop(int, int, float, float, float) override {}
    void setSquealLoop(int, int, float, float, float) override {}
    void setRumble(float, float) override {}
    void trigger(SoundEvent k, int, float gl, float gr, float) override {
        if (k == SoundEvent::StoneHit) { ++stones; if (gl > gr) ++leftStones; else if (gr > gl) ++rightStones; lastStoneGain = gl + gr; maxStoneGain = std::max(maxStoneGain, gl + gr); }
    }
    void silence() override {}
};

// Same frame shape the real game gives: the velocity vector is the ground speed in m/s, the Speed field is the driven-wheel speedometer.
static TelemetryFrame gframe(double t, double groundMs, double drivenMs, float strut, float throttle) {
    TelemetryFrame f; f.time = t; f.speed = (float)(drivenMs * 3.6 * 1.609); f.throttle = throttle; f.velSurge = (float)groundMs;
    for (auto& w : f.wheels) w.strutForce = strut;
    return f;
}

// The settings ForceFX gives the audio: the game surface is trusted even when the suspension is busy on a road it calls tarmac.
static Settings ff() { return Settings(); }

// The game's own material id under each tyre (LF, RF, LB, RB), as the plugin's surface reader would report it.
struct FakeSurface : ISurfaceMemory {
    int ids[4] = {37, 37, 37, 37};
    bool readMaterials(int out[4]) override { for (int i = 0; i < 4; ++i) out[i] = ids[i]; return true; }
    bool configured() const override { return true; }
    std::string statusText() const override { return "fake"; }
};

// Drives for `seconds` at a steady speed. rough = 0 is a smooth strut force (tarmac); rough > 0 adds noise (loose ground).
// mem = null: no game surface, the controller has only the telemetry to go on.
static int run(double groundMs, double drivenMs, float throttle, float rough, double seconds, Settings st = Settings(), uint32_t seed = 1,
               ISurfaceMemory* mem = nullptr, FakeSink* out = nullptr) {
    FakeSink own; FakeSink& sink = out ? *out : own; AudioController c(st, sink, seed); if (mem) c.setSurfaceMemory(mem); double t = 0;
    std::mt19937 g(9); std::normal_distribution<float> n(0, 1);
    for (int i = 0, N = (int)(seconds * 60); i < N; ++i) { t += 1.0 / 60; c.onFrame(gframe(t, groundMs, drivenMs, -3500.f * (1.f + rough * n(g)), throttle)); }
    return sink.stones;
}

TEST(loose_ground_throws_stones_at_low_speed_too) {
    // 14 km/h on rough ground: the old fixed 25 km/h cut-off made this silent
    CHECK(run(4.0, 4.0, 0.5f, 0.5f, 15.0) > 0);
}
TEST(clean_smooth_tarmac_throws_no_stones_at_any_speed) {
    CHECK(run(30.0, 30.0, 0.5f, 0.0f, 15.0) == 0);
    CHECK(run(8.0, 8.0, 0.5f, 0.0f, 15.0) == 0);
}
TEST(stone_rate_grows_smoothly_with_speed) {
    int slow = run(5.0, 5.0, 0.5f, 0.5f, 30.0), mid = run(15.0, 15.0, 0.5f, 0.5f, 30.0), fast = run(30.0, 30.0, 0.5f, 0.5f, 30.0);
    CHECK(slow > 0 && slow < mid && mid < fast);
}
TEST(stones_are_quieter_at_low_surface_speed) {
    FakeSink slow, fast; Settings st;
    AudioController a(st, slow, 3), b(st, fast, 3); double t = 0;
    std::mt19937 g(9); std::normal_distribution<float> n(0, 1);
    for (int i = 0; i < 1800; ++i) {
        t += 1.0 / 60; float strut = -3500.f * (1.f + 0.5f * n(g));
        a.onFrame(gframe(t, 4.0, 4.0, strut, 0.5f)); b.onFrame(gframe(t, 30.0, 30.0, strut, 0.5f));
    }
    CHECK(slow.stones > 0 && fast.stones > 0);
    CHECK(slow.maxStoneGain < fast.maxStoneGain);
}
TEST(standing_wheelspin_still_throws_stones) {
    // car nearly stationary, driven wheels at 14 m/s: the spinning tyre flings material from a standing start
    CHECK(run(1.0, 14.0, 1.0f, 0.5f, 15.0) > 0);
}
TEST(standing_still_with_no_wheelspin_throws_nothing) {
    CHECK(run(0.0, 0.0, 0.0f, 0.5f, 10.0) == 0);
}
TEST(stones_can_be_switched_off) {
    Settings off; off.enableStones = false;
    CHECK(run(25.0, 25.0, 0.5f, 0.5f, 15.0, off) == 0);
    CHECK(run(25.0, 25.0, 0.5f, 0.5f, 15.0) > 0);
}
TEST(no_per_surface_setting_is_involved) {
    // the same telemetry gives the same stones whatever the sound names say: nothing in Settings maps a surface to a stone level
    Settings a, b; b.surfaceNames[3] = "ice"; b.classVolume[3] = 0.1f;
    CHECK(run(20.0, 20.0, 0.5f, 0.5f, 15.0, a, 5) == run(20.0, 20.0, 0.5f, 0.5f, 15.0, b, 5));
}

// ---------------------------------------------------------------- surface decides what there is to throw
TEST(looseness_comes_from_the_game_material_names) {
    MaterialTable t;
    CHECK(t.looseness(37) == 0.0 && t.looseness(38) == 0.0 && t.looseness(65) == 0.0 && t.looseness(69) == 0.0);   // clean tarmac, dry or damp, any grain
    CHECK(t.looseness(45) > 0.2 && t.looseness(45) < 0.5);                                                          // gravel sprinkled on tarmac
    CHECK(t.looseness(53) > t.looseness(45) && t.looseness(53) < 0.9);                                              // covered in it
    CHECK(t.looseness(1) > 0.7 && t.looseness(35) >= t.looseness(1));                                               // gravel, deeper = more
    CHECK(t.looseness(101) > 0.4 && t.looseness(117) < 0.3);                                                        // snow over gravel throws some, over ice little
    CHECK(t.looseness(113) < 0.5);                                                                                  // grass: clods, not stones
    CHECK(t.looseness(30) == -1.0 && t.looseness(63) == -1.0);                                                      // a tree, a pole: not ground
    CHECK(t.looseness(250) == -1.0 && t.looseness(-1) == -1.0 && t.looseness(999) == -1.0);                        // not in the table at all
}
TEST(looseness_of_an_override_follows_its_class) {
    std::string p = (std::filesystem::temp_directory_path() / "loose_override.csv").string();
    { std::ofstream f(p); f << "250,1,0\n251,0,0\n"; }
    MaterialTable t; CHECK(t.loadOverrides(p));
    CHECK(t.looseness(250) > 0.7);                                         // added as loose ground
    CHECK(t.looseness(251) < 0.2);                                         // added as tarmac
    std::filesystem::remove(p);
}
TEST(clean_tarmac_throws_nothing_however_hard_the_car_works_it) {
    FakeSurface m; for (int& i : m.ids) i = 37;
    CHECK(run(30.0, 30.0, 1.0f, 0.5f, 20.0, ff(), 1, &m) == 0);      // rough telemetry, full throttle, fast
    CHECK(run(2.0, 14.0, 1.0f, 0.5f, 20.0, ff(), 1, &m) == 0);       // wheelspin from a standing start
}
TEST(the_more_gravel_on_the_road_the_more_is_thrown) {
    int counts[4]; int id[4] = {37, 45, 53, 1};                             // clean, sprinkled, covered, gravel
    for (int k = 0; k < 4; ++k) { FakeSurface m; for (int& i : m.ids) i = id[k]; counts[k] = run(25.0, 25.0, 0.5f, 0.5f, 30.0, ff(), 1, &m); }
    CHECK(counts[0] == 0);
    CHECK(counts[1] > 0 && counts[1] < counts[2] && counts[2] < counts[3]);
}
TEST(snow_over_gravel_still_throws_while_snow_over_ice_barely_does) {
    FakeSurface sg, si; for (int& i : sg.ids) i = 102; for (int& i : si.ids) i = 117;
    const int onGravel = run(25.0, 25.0, 0.5f, 0.5f, 30.0, ff(), 1, &sg), onIce = run(25.0, 25.0, 0.5f, 0.5f, 30.0, ff(), 1, &si);
    CHECK(onGravel > 0 && onGravel > 2 * onIce);
}
TEST(an_object_id_gives_no_ground_information_so_the_telemetry_decides) {
    FakeSurface m; for (int& i : m.ids) i = 30;                            // a tree: not ground, nothing to read from it
    CHECK(run(25.0, 25.0, 0.5f, 0.0f, 20.0, ff(), 1, &m) == 0);            // smooth telemetry: no stones
    CHECK(run(25.0, 25.0, 0.5f, 0.5f, 20.0, ff(), 1, &m) == run(25.0, 25.0, 0.5f, 0.5f, 20.0, ff(), 1, nullptr));   // rough: same as with no surface at all
}
TEST(gravel_throws_more_with_speed_and_from_wheelspin_at_a_standstill) {
    FakeSurface m; for (int& i : m.ids) i = 1;
    const int slow = run(5.0, 5.0, 0.5f, 0.5f, 30.0, ff(), 1, &m), fast = run(30.0, 30.0, 0.5f, 0.5f, 30.0, ff(), 1, &m);
    CHECK(slow > 0 && fast > 2 * slow);
    CHECK(run(1.0, 14.0, 1.0f, 0.5f, 15.0, ff(), 1, &m) > 0);
    CHECK(run(0.0, 0.0, 0.0f, 0.5f, 10.0, ff(), 1, &m) == 0);        // parked
}
TEST(each_tyre_throws_from_its_own_ground) {
    FakeSurface m; m.ids[0] = 1; m.ids[2] = 1; m.ids[1] = 37; m.ids[3] = 37;   // left tyres on gravel, right tyres on clean tarmac
    FakeSink sink; run(25.0, 25.0, 0.5f, 0.5f, 30.0, ff(), 1, &m, &sink);
    CHECK(sink.stones > 20);
    CHECK(sink.leftStones > 9 * sink.rightStones);                         // the stones come from the left side
}
TEST(without_the_game_surface_the_telemetry_decides_alone) {
    CHECK(run(25.0, 25.0, 0.5f, 0.0f, 20.0) == 0);                          // smooth: nothing
    CHECK(run(25.0, 25.0, 0.5f, 0.5f, 20.0) > 0);                           // rough: stones
}
TEST(there_is_no_stone_rate_setting_any_more) {
    std::string p = (std::filesystem::temp_directory_path() / "legacy_rate.ini").string();
    { std::ofstream f(p); f << "StoneRate = 500\nStoneLevel = 3\n"; }
    Settings s = ff(); CHECK(loadSettings(p, s)); CHECK(s.stoneLevel > 2.99 && s.stoneLevel < 3.01);        // the old key is ignored, the level still reads
    FakeSurface m; for (int& i : m.ids) i = 1;
    CHECK(run(25.0, 25.0, 0.5f, 0.5f, 30.0, s, 1, &m) == run(25.0, 25.0, 0.5f, 0.5f, 30.0, s, 1, &m));
    std::filesystem::remove(p);
}
TEST(rough_road_that_the_game_calls_tarmac_keeps_its_surface) {
    // very rough ground: the game says tarmac, the suspension is busy. The audio keeps the surface the reader gave it.
    FakeSurface m; for (int& i : m.ids) i = 77;
    FakeSink sink; Settings st = ff(); AudioController c(st, sink, 1); c.setSurfaceMemory(&m);
    std::mt19937 g(9); std::normal_distribution<float> n(0, 1); double t = 0;
    for (int i = 0; i < 60 * 25; ++i) { t += 1.0 / 60; c.onFrame(gframe(t, 25.0, 25.0, -3500.f * (1.f + 0.8f * n(g)), 0.5f)); }
    CHECK(c.usingMemory());                                                                                   // 25 s of heavy roughness later
    CHECK(c.analyzer.wheels[0].dominant() == Tarmac);
}

// ---------------------------------------------------------------- stone loudness
static void writeTestWav(const std::string& path, int samples, double amp) {     // 16-bit mono PCM, decaying burst
    std::vector<int16_t> d(samples);
    for (int i = 0; i < samples; ++i) d[i] = (int16_t)(32767.0 * amp * std::sin(i * 0.3) * (1.0 - (double)i / samples));
    std::ofstream f(path, std::ios::binary);
    auto w32 = [&](uint32_t v) { f.write((const char*)&v, 4); }; auto w16 = [&](uint16_t v) { f.write((const char*)&v, 2); };
    f.write("RIFF", 4); w32(36 + samples * 2); f.write("WAVEfmt ", 8); w32(16); w16(1); w16(1); w32(kSampleRate); w32(kSampleRate * 2); w16(2); w16(16);
    f.write("data", 4); w32(samples * 2); f.write((const char*)d.data(), samples * 2);
}
// Peak output after triggering one stone through a real mixer and sink.
static float stonePeak(double stoneLevel, float live) {
    std::string dir = (std::filesystem::temp_directory_path() / "stone_level_test").string();
    std::filesystem::remove_all(dir); std::filesystem::create_directories(dir);
    writeTestWav(dir + "/debris_stone_1.wav", 4000, 0.8);
    SoundBank bank; bank.load(dir); Settings st; st.stoneLevel = stoneLevel; Mixer mixer; mixer.master = 0.2f;   // low drive keeps the soft limiter linear
    MixerSink sink(mixer, bank, st); sink.setLiveLevels(live, 1.0f);
    sink.trigger(SoundEvent::StoneHit, 0, 0.5f, 0.5f, 1.0f);
    std::vector<float> buf(2 * 512); float peak = 0;
    for (int i = 0; i < 4; ++i) { mixer.read(buf.data(), 512); for (float x : buf) peak = std::max(peak, std::fabs(x)); }
    std::filesystem::remove_all(dir);
    return peak;
}
TEST(stone_level_scales_the_stone_loudness) {
    float base = stonePeak(1.0, 1.0f), loud = stonePeak(2.0, 1.0f), off = stonePeak(0.0, 1.0f);
    CHECK(base > 0.001f);
    CHECK(loud > 1.6f * base && loud < 2.4f * base);
    CHECK(off < 1e-6f);
}
TEST(live_menu_multiplier_stacks_on_the_ini_level) {
    float base = stonePeak(1.0, 1.0f), half = stonePeak(1.0, 0.5f), triple = stonePeak(1.0, 3.0f);
    CHECK(half > 0.4f * base && half < 0.6f * base);
    CHECK(triple > 2.5f * base);
}
TEST(stone_level_ini_key_is_read_and_clamped) {
    std::string p = (std::filesystem::temp_directory_path() / "stone_level.ini").string();
    { std::ofstream f(p); f << "StoneLevel = 3.5\nEventLevel = 0.5\n"; }
    Settings s; CHECK(loadSettings(p, s)); CHECK(s.stoneLevel > 3.49 && s.stoneLevel < 3.51); CHECK(s.eventLevel > 0.49 && s.eventLevel < 0.51);
    { std::ofstream f(p); f << "StoneLevel = 99\n"; }
    Settings s2; CHECK(loadSettings(p, s2)); CHECK(s2.stoneLevel <= 6.0);
    { std::ofstream f(p); f << "# nothing\n"; }
    Settings s3; CHECK(loadSettings(p, s3)); CHECK(s3.stoneLevel > 1.99 && s3.stoneLevel < 2.01);   // default is louder than the old shared level
    std::filesystem::remove(p);
}

TEST(settings_are_read_only_from_the_named_section) {
    std::string p = (std::filesystem::temp_directory_path() / "section.ini").string();
    { std::ofstream f(p);
      f << "[ForceFX]\nStoneLevel = 5\nSpeedIsKph = 0\n"
           "[Audio]   ; trailing comment\nStoneLevel = 1.5\nEventLevel = 0.5\n"
           "[CameraInput]\nEventLevel = 3\n"; }
    Settings s; CHECK(loadSettings(p, s, "Audio"));
    CHECK(s.stoneLevel > 1.49 && s.stoneLevel < 1.51);       // not the 5 from [ForceFX]
    CHECK(s.eventLevel > 0.49 && s.eventLevel < 0.51);       // not the 3 from the next section
    CHECK(s.speedIsKph);                                      // [ForceFX]'s SpeedIsKph=0 is not read here
    Settings t; CHECK(loadSettings(p, t, "audio")); CHECK(t.stoneLevel > 1.49 && t.stoneLevel < 1.51);   // names are case-insensitive
    Settings u; CHECK(loadSettings(p, u, "Missing")); CHECK(u.stoneLevel > 1.99 && u.stoneLevel < 2.01);  // no such section: defaults
    Settings v; CHECK(loadSettings(p, v)); CHECK(v.eventLevel > 2.99 && v.eventLevel < 3.01);             // no section named: every line, last one wins
    CHECK(!loadSettings(p + ".nope", s, "Audio"));
    std::filesystem::remove(p);
}

int main() {
    for (const Reg& r : regs()) { int before = g_fail; r.fn(); std::printf("[%s] %s\n", g_fail == before ? " ok " : "FAIL", r.name); }
    std::printf("\n%zu tests, %d checks, %d failures\n", regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
