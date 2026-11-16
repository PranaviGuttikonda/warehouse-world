#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ww/components.hpp"
#include "ww/ecs.hpp"
#include "ww/grid.hpp"
#include "ww/reservation.hpp"
#include "ww/systems.hpp"

namespace ww {

/// How robots choose their moves each tick.
enum class Planner {
    Local,      // phase 1: BFS step + local "cell empty at tick start" rule
    SpaceTime,  // space-time A* against a reservation table
    Cbs,        // windowed conflict-based search, all robots jointly
};

/// Given: a planner. Approach: short CLI name ("local", "stastar", "cbs").
const char* plannerName(Planner p);
/// Given: a CLI name. Approach: inverse of plannerName; nullopt if unknown.
std::optional<Planner> parsePlanner(std::string_view name);

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
        Planner planner = Planner::Local;
    };

    /// Given: a layout and config. Approach: spawn robots on distinct random
    /// floor cells, each with a fresh task. Throws if the map can't fit them.
    Simulation(Grid grid, Config config);

    // DistanceCache and ReservationTable point at grid_, so the world must stay put in memory.
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    /// One fixed tick: move robots (with the configured planner), then
    /// update tasks of those on a goal.
    void step();

    const Grid& grid() const { return grid_; }
    Planner planner() const { return planner_; }
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
    Planner planner_;
    ReservationTable reservations_;  // used by Planner::SpaceTime only
    std::vector<Entity> occupancy_;
    Stats stats_;
};

}  // namespace ww
