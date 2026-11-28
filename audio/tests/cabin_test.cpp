// Tests for the inside of the car: the camera view deciding how inside the sound is, the cabin filter, and how the controller follows the view. Self-contained; build and run from this folder:
//   g++ -std=c++17 -O1 -I../.. cabin_test.cpp -o cabin_test && ./cabin_test
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <random>
#include "core.hpp"
#include "audio.hpp"
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

// ---------------------------------------------------------------------------------------------------- the controller
struct Sink : IAudioSink {
    float inside = -1; int calls = 0;
    void setWheelLoop(int, int, float, float, float) override {}
    void setSlideLoop(int, int, float, float, float) override {}
    void setSquealLoop(int, int, float, float, float) override {}
    void setRumble(float, float) override {}
    void trigger(SoundEvent, int, float, float, float) override {}
    void silence() override {}
    void setInside(float a) override { inside = a; ++calls; }
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

TEST(the_camera_view_drives_the_inside_amount) {
    Sink sink; Settings st; AudioController c(st, sink, 5); Reader r; c.setSurfaceMemory(&r); double t = 0;
    c.setCameraType(5); run(c, 3.0, 25.0, 5000.f, t);
    CHECK(sink.inside > 0.99f);
    c.setCameraType(1); run(c, 3.0, 25.0, 5000.f, t);
    CHECK(sink.inside < 0.01f);                                                       // outside once the crossfade has finished
    c.setCameraType(-1); run(c, 3.0, 25.0, 5000.f, t);                              // not known: inside
    CHECK(sink.inside > 0.99f);
}
TEST(a_different_range_of_camera_types_can_count_as_inside) {
    Sink sink; Settings st; st.cabinTypeLo = 8; st.cabinTypeHi = 9; AudioController c(st, sink, 5); Reader r; c.setSurfaceMemory(&r); double t = 0;
    c.setCameraType(5); run(c, 3.0, 25.0, 5000.f, t); CHECK(sink.inside < 0.01f);
    c.setCameraType(8); run(c, 3.0, 25.0, 5000.f, t); CHECK(sink.inside > 0.99f);
}
TEST(the_settings_file_sets_the_range_and_ignores_the_old_cabin_keys) {
    Settings s;
    applySetting(s, "CabinCameraMin", "2"); applySetting(s, "CabinCameraMax", "7");
    CHECK(s.cabinTypeLo == 2 && s.cabinTypeHi == 7);
    applySetting(s, "CabinAmbience", "1.5"); applySetting(s, "CabinEffect", "0"); applySetting(s, "CabinLowpassHz", "100");        // old keys: nothing breaks, nothing changes
    CHECK(s.cabinTypeLo == 2 && s.cabinTypeHi == 7);
}
TEST(the_rpm_is_read_from_the_packet_with_its_scale) {
    PacketLayout L; L.rpm = 136; L.rpmScale = 2.0f;
    std::vector<uint8_t> pk(L.NeededLength(), 0);
    const float v = 3000.0f; std::memcpy(pk.data() + 136, &v, 4);
    TelemetryFrame f; CHECK(parsePacket(pk.data(), pk.size(), 1.0, f, L)); CHECK(std::fabs(f.rpm - 6000.0f) < 1e-3f);
    L.rpm = -1; TelemetryFrame g; CHECK(parsePacket(pk.data(), pk.size(), 1.0, g, L)); CHECK(g.rpm == 0.0f);
}
int main() { return mini::RunAll(); }
