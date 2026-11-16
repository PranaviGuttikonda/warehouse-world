#pragma once

#include <algorithm>

namespace ww {

/// Fixed-timestep accumulator ("fix your timestep").
///
/// Given: real frame times that vary.
/// Approach: add elapsed time to an accumulator and run the simulation in
/// constant dt steps while it holds at least dt. The sim stays deterministic
/// no matter how fast or slow frames arrive. maxSteps caps catch-up work so a
/// long stall can't snowball ("spiral of death"); leftover time is dropped.
class FixedTimestep {
public:
    FixedTimestep(double dtSeconds, int maxStepsPerFrame = 8)
        : dt_(dtSeconds), maxSteps_(maxStepsPerFrame) {}

    /// Returns how many steps were run.
    template <typename StepFn>
    int advance(double frameSeconds, StepFn&& step) {
        accumulator_ += std::max(0.0, frameSeconds);
        int steps = 0;
        while (accumulator_ >= dt_ && steps < maxSteps_) {
            step();
            accumulator_ -= dt_;
            ++steps;
        }
        if (accumulator_ >= dt_) accumulator_ = 0.0;  // hit the cap: drop backlog
        return steps;
    }

    /// Fraction of the next step already elapsed, in [0, 1). Useful for
    /// interpolating rendering between ticks.
    double alpha() const { return accumulator_ / dt_; }
    double dt() const { return dt_; }

private:
    double dt_;
    int maxSteps_;
    double accumulator_ = 0.0;
};

}  // namespace ww
