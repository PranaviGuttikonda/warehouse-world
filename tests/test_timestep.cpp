#include <cmath>

#include "test_framework.hpp"
#include "ww/timestep.hpp"

using namespace ww;

TEST(timestep_runs_whole_steps) {
    FixedTimestep ts(0.1, 100);
    int steps = 0;
    CHECK_EQ(ts.advance(1.0, [&] { ++steps; }), 10);
    CHECK_EQ(steps, 10);
}

TEST(timestep_accumulates_partial_frames) {
    FixedTimestep ts(0.1);
    int steps = 0;
    auto inc = [&] { ++steps; };
    CHECK_EQ(ts.advance(0.06, inc), 0);
    CHECK_EQ(ts.advance(0.06, inc), 1);  // 0.12 accumulated
    CHECK(std::abs(ts.alpha() - 0.2) < 1e-9);
    CHECK_EQ(steps, 1);
}

TEST(timestep_caps_catch_up_and_drops_backlog) {
    FixedTimestep ts(0.1, 4);
    int steps = 0;
    CHECK_EQ(ts.advance(10.0, [&] { ++steps; }), 4);
    CHECK_EQ(steps, 4);
    CHECK(ts.alpha() < 1.0);  // backlog discarded, not carried forever
}

TEST(timestep_ignores_negative_time) {
    FixedTimestep ts(0.1);
    CHECK_EQ(ts.advance(-5.0, [] {}), 0);
    CHECK(ts.alpha() == 0.0);
}
