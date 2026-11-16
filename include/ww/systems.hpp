#pragma once

#include <unordered_map>
#include <vector>

#include "ww/components.hpp"
#include "ww/ecs.hpp"
#include "ww/grid.hpp"
#include "ww/random.hpp"

namespace ww {

struct Stats {
    long long ticks = 0;
    long long deliveries = 0;
    long long moves = 0;
    long long waits = 0;
    long long cbsSolves = 0;     // CBS planner: joint replans
    long long cbsFallbacks = 0;  // ... of which hit the node budget
};

/// Shortest-path distances from every cell to one goal, ignoring robots.
///
/// Given: a static grid and a fixed set of goal cells (pick faces, stations).
/// Approach: BFS out from the goal once, cache the field per goal. A robot
/// then steps to any neighbour with a smaller distance. Phase 2 swaps this
/// for A* / space-time planning that also accounts for other robots.
class DistanceCache {
public:
    static constexpr int kUnreachable = -1;

    explicit DistanceCache(const Grid& grid) : grid_(grid) {}
    const std::vector<int>& to(Cell goal);

private:
    const Grid& grid_;
    std::unordered_map<int, std::vector<int>> fields_;
};

/// Given: grid and rng. Approach: random shelf face, then random station.
Task makeTask(const Grid& grid, Rng& rng);

/// Robots standing on their goal advance their task: pickup -> dropoff ->
/// new task. Counts deliveries.
void taskSystem(Registry& reg, const Grid& grid, Rng& rng, Stats& stats);

/// Moves every robot at most one cell per tick without collisions.
///
/// Given: occupancy[cellIndex] = robot on that cell at tick start (or null).
/// Approach: visit robots in a rotating order. A robot may enter a cell only
/// if nobody stood there at tick start and nobody else claimed it this tick.
/// That rule alone rules out overlaps and head-on swaps. A robot blocked for
/// kSidestepAfter ticks takes any free neighbour to break local deadlocks.
void movementSystem(Registry& reg, const Grid& grid, DistanceCache& dist,
                    std::vector<Entity>& occupancy, Rng& rng, Stats& stats);

inline constexpr int kSidestepAfter = 3;

}  // namespace ww
