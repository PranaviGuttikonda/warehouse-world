#include "test_framework.hpp"

int main() {
    int failedCases = 0;
    for (auto& c : wwtest::registry()) {
        int before = wwtest::failures();
        c.fn();
        bool ok = wwtest::failures() == before;
        if (!ok) ++failedCases;
        std::cout << (ok ? "[ ok ] " : "[FAIL] ") << c.name << "\n";
    }
    std::cout << "\n" << wwtest::registry().size() - failedCases << "/" << wwtest::registry().size()
              << " tests passed\n";
    return failedCases == 0 ? 0 : 1;
}
