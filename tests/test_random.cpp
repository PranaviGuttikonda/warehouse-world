#include <array>
#include <vector>

#include "test_framework.hpp"
#include "ww/random.hpp"
#include "ww/simulation.hpp"

using namespace ww;

// Given: fixed seeds. Approach: pin exact outputs so any platform or code
// change that alters the random stream fails loudly.

TEST(mt19937_64_matches_the_standard) {
    Rng rng;  // default seed 5489
    rng.discard(9999);
    CHECK_EQ(rng(), 9981545732273789042ULL);  // value required by [rand.predef]
}

TEST(uniform_int_pinned_values) {
    Rng a(42);
    std::array<std::uint64_t, 5> small{};
    for (auto& v : small) v = uniformInt(a, 0, 9);
    CHECK((small == std::array<std::uint64_t, 5>{6, 4, 0, 2, 1}));

    Rng b(42);
    CHECK_EQ(uniformInt(b, 100, 1000000), 518281ULL);
    CHECK_EQ(uniformInt(b, 100, 1000000), 770866ULL);
    CHECK_EQ(uniformInt(b, 100, 1000000), 876512ULL);

    Rng c(1);
    CHECK_EQ(uniformInt(c, 0, ~0ULL), 2469588189546311528ULL);  // full range
}

TEST(uniform_int_stays_in_bounds) {
    Rng rng(3);
    for (int i = 0; i < 10000; ++i) {
        std::uint64_t v = uniformInt(rng, 5, 7);
        CHECK(v >= 5 && v <= 7);
    }
    CHECK_EQ(uniformInt(rng, 4, 4), 4ULL);
}

TEST(shuffle_pinned_permutation) {
    Rng rng(7);
    std::array<int, 8> a{0, 1, 2, 3, 4, 5, 6, 7};
    ww::shuffle(a.begin(), a.end(), rng);
    CHECK((a == std::array<int, 8>{2, 3, 5, 6, 1, 0, 4, 7}));
}

TEST(simulation_run_is_pinned) {
    Simulation sim(Grid::parse(defaultMap()), {16, 42});
    for (int t = 0; t < 2000; ++t) sim.step();
    CHECK_EQ(sim.stats().deliveries, 426LL);
    CHECK_EQ(sim.stats().moves, 18345LL);
}
