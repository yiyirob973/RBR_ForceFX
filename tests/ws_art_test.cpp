// Checks on the water-drop artwork (camera/ws_art.hpp): no heavy dark fill, round standing drops, runners without a tail.
//   g++ -std=c++17 -O1 -I.. ws_art_test.cpp -o ws_art_test && ./ws_art_test
#include <cmath>
#include <cstdio>
#include "../camera/ws_art.hpp"
using namespace ffx::cam::wsl::art;
static int g_fail = 0, g_checks = 0;
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; std::printf("  FAIL line %d  %s\n", __LINE__, #c); } } while (0)
int main() {
    const int N = 64;
    // the standing drop is round: its width and height (where the height field is above 0.2) are within 12 percent
    for (int cell = 0; cell < 2; ++cell) {
        int minx = N, maxx = -1, miny = N, maxy = -1; double area = 0;
        for (int y = 0; y < N; ++y) for (int x = 0; x < N; ++x)
            if (height(cell, (x + .5f) / N, (y + .5f) / N) > 0.2f) { minx = std::min(minx, x); maxx = std::max(maxx, x); miny = std::min(miny, y); maxy = std::max(maxy, y); area += 1; }
        const float w = (float)(maxx - minx + 1), h = (float)(maxy - miny + 1);
        std::printf("cell %d: %.0f x %.0f px, area %.0f\n", cell, w, h, area);
        if (cell == 0) CHECK(std::fabs(w / h - 1.f) < 0.12f);
        else CHECK(h / w > 1.1f && h / w < 1.9f);                      // a runner is longer than wide, but not a streak
    }
    // a runner has no tail: its width at 10 percent from the top is at least 40 percent of its widest part
    { float widest = 0, top = 0;
      for (int x = 0; x < N; ++x) { for (int y = 0; y < N; ++y) { const float h = height(1, (x + .5f) / N, (y + .5f) / N); (void)h; } }
      auto rowWidth = [&](int y) { int n = 0; for (int x = 0; x < N; ++x) if (height(1, (x + .5f) / N, (y + .5f) / N) > 0.05f) ++n; return (float)n; };
      int y0 = N, y1 = -1; for (int y = 0; y < N; ++y) if (rowWidth(y) > 0) { y0 = std::min(y0, y); y1 = std::max(y1, y); }
      for (int y = y0; y <= y1; ++y) widest = std::max(widest, rowWidth(y));
      top = rowWidth(y0 + (y1 - y0) / 6);
      std::printf("runner: widest %.0f, near the top %.0f\n", widest, top);
      CHECK(top > 0.4f * widest); }
    // the shading adds no body colour: in the middle of the drop it is almost transparent, so the refracted view is not darkened or greyed
    { Px p; shade(0, 0.5f, 0.55f, p); CHECK(p.a < 0.08f); }
    // the edge: dark on the upper side, bright on the lower side, both soft
    { Px up, dn; shade(0, 0.5f, 0.5f - 0.40f, up); shade(0, 0.5f, 0.5f + 0.40f, dn);
      const float lu = up.a > 1e-4f ? (up.c[0] + up.c[1] + up.c[2]) / 3.f / up.a : 0.5f, ld = dn.a > 1e-4f ? (dn.c[0] + dn.c[1] + dn.c[2]) / 3.f / dn.a : 0.5f;
      std::printf("edge colour up %.2f (a %.2f), down %.2f (a %.2f)\n", lu, up.a, ld, dn.a);
      CHECK(lu < ld); CHECK(up.a < 0.7f && dn.a < 0.7f); }
    // outside the drop nothing is drawn
    { Px p; shade(0, 0.02f, 0.02f, p); CHECK(p.a == 0.f); CHECK(mask(0, 0.02f, 0.02f) == 0.f); }
    // the lens is nearly opaque inside
    CHECK(mask(0, 0.5f, 0.5f) > 0.9f);
    // sprite and micro stay inside their unit circle
    { Px p; sprite(0.99f, 0.99f, p); CHECK(p.a == 0.f); Px q; micro(0.99f, 0.99f, q); CHECK(q.a == 0.f); }
    std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
