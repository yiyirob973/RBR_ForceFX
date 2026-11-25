// Tests for the shared tyre surface info (surface_model.hpp), including that its class numbers match the reader's. Self-contained:
//   g++ -std=c++17 -O1 -I.. surface_model_test.cpp -o surface_model_test && ./surface_model_test
#include <cstdio>
#include <functional>
#include <vector>
#include "../surface_feel.hpp"
#include "../surface_model.hpp"

static int g_fail = 0, g_checks = 0;
struct Reg { const char* name; std::function<void()> fn; };
static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
#define TEST(name) static void name(); static Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)

TEST(the_numbers_in_the_mapping_are_the_readers_classes) {
    CHECK(SurfaceModel::AudioClassOf(Surf::C_TARMAC) == 0);
    CHECK(SurfaceModel::AudioClassOf(Surf::C_GRAVEL) == 1);
    CHECK(SurfaceModel::AudioClassOf(Surf::C_SOFT) == 2);
    CHECK(SurfaceModel::AudioClassOf(Surf::C_SNOW) == 3);
    CHECK(SurfaceModel::AudioClassOf(Surf::C_ICE) == 3);
    CHECK(SurfaceModel::AudioClassOf(Surf::C_COBBLE) == 0);
    CHECK(SurfaceModel::AudioClassOf(Surf::C_WOOD) == 0);
    CHECK(SurfaceModel::AudioClassOf(Surf::C_NONE) == -1);
}
TEST(unknown_classes_have_no_audio_class) {
    CHECK(SurfaceModel::AudioClassOf(-1) == -1); CHECK(SurfaceModel::AudioClassOf(99) == -1);
}
TEST(every_audio_class_goes_back_to_a_matching_reader_class) {
    CHECK(SurfaceModel::SurfClassOfAudio(0) == Surf::C_TARMAC);
    CHECK(SurfaceModel::SurfClassOfAudio(1) == Surf::C_GRAVEL);
    CHECK(SurfaceModel::SurfClassOfAudio(2) == Surf::C_SOFT);
    CHECK(SurfaceModel::SurfClassOfAudio(3) == Surf::C_SNOW);
    CHECK(SurfaceModel::SurfClassOfAudio(-1) == Surf::C_NONE); CHECK(SurfaceModel::SurfClassOfAudio(7) == Surf::C_NONE);
    for (int a = 0; a < 4; ++a) CHECK(SurfaceModel::AudioClassOf(SurfaceModel::SurfClassOfAudio(a)) == a);
}
TEST(a_tyre_with_nothing_known_says_so) {
    SurfaceModel::Tyre t; CHECK(t.id == -1 && t.audioClass == -1 && t.wet == 0 && t.loose < 0.0f && !t.ridged);
}

int main() {
    for (auto& r : regs()) { int before = g_fail; r.fn(); std::printf("%s %s\n", before == g_fail ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
