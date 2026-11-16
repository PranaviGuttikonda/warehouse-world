#pragma once

// Minimal test harness: no downloads, no dependencies.
// TEST(name) { CHECK(cond); CHECK_EQ(a, b); }

#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace wwtest {

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
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline void fail(const char* file, int line, const std::string& msg) {
    ++failures();
    std::cerr << "  FAIL " << file << ":" << line << "  " << msg << "\n";
}

}  // namespace wwtest

#define WW_CAT2(a, b) a##b
#define WW_CAT(a, b) WW_CAT2(a, b)

#define TEST(name)                                                              \
    static void name();                                                         \
    static ::wwtest::Registrar WW_CAT(reg_, name)(#name, name);                 \
    static void name()

#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) ::wwtest::fail(__FILE__, __LINE__, "CHECK(" #cond ")");    \
    } while (0)

#define CHECK_EQ(a, b)                                                          \
    do {                                                                        \
        auto _va = (a);                                                         \
        auto _vb = (b);                                                         \
        if (!(_va == _vb))                                                      \
            ::wwtest::fail(__FILE__, __LINE__, "CHECK_EQ(" #a ", " #b ")");     \
    } while (0)

#define CHECK_THROWS(expr)                                                      \
    do {                                                                        \
        bool _threw = false;                                                    \
        try { (void)(expr); } catch (...) { _threw = true; }                    \
        if (!_threw) ::wwtest::fail(__FILE__, __LINE__, "expected throw: " #expr); \
    } while (0)
