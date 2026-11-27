// The small test framework every test file shares: TEST registers a test, CHECK counts one check, RunAll runs them in order, prints one line per test
// and a summary, and gives the process exit code (0 = all passed). A test file includes this, writes its TEST blocks and ends with
//   int main() { return mini::RunAll(); }
#pragma once
#include <cstdio>
#include <functional>
#include <vector>

namespace mini {
struct Reg { const char* name; std::function<void()> fn; };
inline std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
inline int g_fail = 0, g_checks = 0;
struct Adder { Adder(const char* n, std::function<void()> f) { regs().push_back({n, f}); } };
inline int RunAll() {
    for (const Reg& r : regs()) { const int before = g_fail; r.fn(); std::printf("%s %s\n", g_fail == before ? "ok  " : "FAIL", r.name); }
    std::printf("\n%d tests, %d checks, %d failures\n", (int)regs().size(), g_checks, g_fail);
    return g_fail ? 1 : 0;
}
}  // namespace mini

#define TEST(name) static void name(); static mini::Adder add_##name(#name, name); static void name()
#define CHECK(c) do { ++mini::g_checks; if (!(c)) { ++mini::g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)
