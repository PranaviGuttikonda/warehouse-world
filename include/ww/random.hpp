#pragma once

#include <cstdint>
#include <random>
#include <utility>

namespace ww {

/// mt19937_64's output sequence is fixed by the standard; the distributions
/// and std::shuffle are not. Use only the helpers below so a seed means the
/// same run on every platform.
using Rng = std::mt19937_64;

/// Given: rng and inclusive bounds lo <= hi.
/// Approach: rejection sampling. Draws below 2^64 mod range are discarded so
/// every value in [lo, hi] is equally likely, then x % range picks one.
inline std::uint64_t uniformInt(Rng& rng, std::uint64_t lo, std::uint64_t hi) {
    const std::uint64_t range = hi - lo + 1;
    if (range == 0) return rng();  // full 64-bit range
    const std::uint64_t threshold = (0 - range) % range;  // 2^64 mod range
    std::uint64_t x;
    do x = rng();
    while (x < threshold);
    return lo + x % range;
}

/// Given: a random-access range. Approach: Fisher–Yates, back to front.
template <typename It>
void shuffle(It first, It last, Rng& rng) {
    const auto n = static_cast<std::uint64_t>(last - first);
    for (std::uint64_t i = n; i > 1; --i) {
        const std::uint64_t j = uniformInt(rng, 0, i - 1);
        using std::swap;
        swap(first[static_cast<std::ptrdiff_t>(i - 1)], first[static_cast<std::ptrdiff_t>(j)]);
    }
}

}  // namespace ww
