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

/// Robot-count limits for the viewer: past ~20% of the floor the café
/// gridlocks for every planner (100 robots on 206 cells: 0 served).
constexpr int kRobotStep = 4;            // caps are multiples of this
constexpr double kRobotDensity = 0.20;   // share of floor cells robots may fill
/// Given: a map. Approach: kRobotDensity of its floor (spawnable) cells,
/// rounded down to a multiple of kRobotStep (tiny maps: at least 1).
int robotCap(const Grid& grid);

/// Read-only snapshot of one robot, for rendering and tests.
struct RobotView {
    int id;
    Cell cell;
    Cell goal;
    Cell dropoff;  // where this robot's current order goes
    bool carrying;
    int waitTicks;  // consecutive ticks blocked
    std::vector<Cell> route;  // planned cells from this tick on (space-time A*, CBS); empty for local
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
