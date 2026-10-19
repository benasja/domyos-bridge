// Minimal self-registering test runner. Header-only, no dependencies, so the
// host tests build with any C++17 compiler and nothing to download.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

namespace tiny_test {

struct Case {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}
inline int& failures() {
    static int n = 0;
    return n;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, fn}); }
};

inline int run_all() {
    int failed_cases = 0;
    for (const auto& c : registry()) {
        const int before = failures();
        c.fn();
        const bool ok = failures() == before;
        if (!ok) ++failed_cases;
        std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name);
    }
    std::printf("\n%zu test(s), %d failed\n", registry().size(), failed_cases);
    return failed_cases == 0 ? 0 : 1;
}

}  // namespace tiny_test

#define TT_CONCAT2(a, b) a##b
#define TT_CONCAT(a, b) TT_CONCAT2(a, b)

#define TEST(name)                                                                   \
    static void TT_CONCAT(test_fn_, __LINE__)();                                     \
    static tiny_test::Registrar TT_CONCAT(test_reg_, __LINE__)(name,                 \
                                                               TT_CONCAT(test_fn_, __LINE__)); \
    static void TT_CONCAT(test_fn_, __LINE__)()

#define CHECK(cond)                                                                  \
    do {                                                                             \
        if (!(cond)) {                                                               \
            std::printf("    %s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);  \
            ++tiny_test::failures();                                                 \
        }                                                                            \
    } while (0)

#define CHECK_EQ(a, b)                                                               \
    do {                                                                             \
        const auto tt_a = (a);                                                       \
        const auto tt_b = (b);                                                       \
        if (!(tt_a == tt_b)) {                                                       \
            std::printf("    %s:%d: CHECK_EQ(%s, %s) failed: %lld != %lld\n", __FILE__, \
                        __LINE__, #a, #b, static_cast<long long>(tt_a),              \
                        static_cast<long long>(tt_b));                               \
            ++tiny_test::failures();                                                 \
        }                                                                            \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                        \
    do {                                                                             \
        const double tt_a = (a);                                                     \
        const double tt_b = (b);                                                     \
        if (std::fabs(tt_a - tt_b) > (tol)) {                                        \
            std::printf("    %s:%d: CHECK_NEAR(%s, %s) failed: %f vs %f\n", __FILE__, \
                        __LINE__, #a, #b, tt_a, tt_b);                               \
            ++tiny_test::failures();                                                 \
        }                                                                            \
    } while (0)
