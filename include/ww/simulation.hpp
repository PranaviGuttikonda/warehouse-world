#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ww/components.hpp"
#include "ww/ecs.hpp"
#include "ww/grid.hpp"
#include "ww/systems.hpp"

namespace ww {

/// Read-only snapshot of one robot, for rendering and tests.
struct RobotView {
    int id;
    Cell cell;
    Cell goal;
    bool carrying;
    int waitTicks;  // consecutive ticks blocked
};

/// The whole world: static grid + robots + systems, advanced one tick at a time.
class Simulation {
public:
    struct Config {
        int robots = 12;
        std::uint64_t seed = 42;
    };

    /// Given: a layout and config. Approach: spawn robots on distinct random
    /// floor cells, each with a fresh task. Throws if the map can't fit them.
    Simulation(Grid grid, Config config);

    // DistanceCache points at grid_, so the world must stay put in memory.
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    /// One fixed tick: move robots, then update tasks of those on a goal.
    void step();

    const Grid& grid() const { return grid_; }
    const Stats& stats() const { return stats_; }
    std::vector<RobotView> robots() const;

    /// Given: current state. Approach: rebuild occupancy from scratch and
    /// check no two robots share a cell and none stands on a blocked tile.
    /// Returns an empty string if valid, else a description of the problem.
    std::string validate() const;

private:
    Grid grid_;
    Registry reg_;
    Rng rng_;
    DistanceCache dist_;
    std::vector<Entity> occupancy_;
    Stats stats_;
};

}  // namespace ww
