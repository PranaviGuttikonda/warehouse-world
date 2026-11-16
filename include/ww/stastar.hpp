#pragma once

#include <optional>
#include <vector>

#include "ww/components.hpp"
#include "ww/ecs.hpp"
#include "ww/grid.hpp"
#include "ww/reservation.hpp"
#include "ww/systems.hpp"

namespace ww {

struct SpaceTimeLimits {
    int slack = 48;               // extra ticks allowed beyond the no-robot distance
    int maxExpansions = 20000;    // give up (and wait) rather than stall the tick
};

struct SpaceTimePlan {
    std::vector<Cell> path;     // path[k] = cell at startTick + k; parks on back()
    bool reachesGoal;           // false: fallback toward the goal, retry later
    std::size_t goalIndex;      // first k with path[k] == goal (path.size() if none)
};

/// One robot's path through space and time, avoiding everyone else's.
///
/// Given: a reservation table without this robot's own entries, the BFS
/// distance field to goal, the robot at start on tick startTick, and
/// noPark[cell] > 0 for cells it must not end on: every robot's goal (own
/// included) and any other cells to keep clear.
/// Approach: A* over (cell, tick, goal touched yet). Each step is a wait or
/// one of 4 moves and costs one tick; moves into reserved cells and swaps
/// are skipped. A plan touches the goal and then ends on a cell that is
/// not in noPark and that it can hold forever ("touch and go"), so goals are
/// only ever held for single ticks and many robots can line up for the same
/// station. The heuristic (BFS distance, +1 to step off the goal) never
/// overestimates. Ties: lowest f, highest g, lowest cell, then push order,
/// so paths are the same on every platform.
/// If that fails within the limits and allowFallback is set, returns the
/// path to the nearest parkable cell outside noPark instead (reachesGoal false).
std::optional<SpaceTimePlan> planSpaceTime(const Grid& grid, const ReservationTable& table,
                                           const std::vector<int>& distToGoal, Cell start, Cell goal,
                                           long long startTick, const std::vector<int>& noPark,
                                           bool allowFallback = false, SpaceTimeLimits limits = {});

/// Moves every robot one tick along its reserved plan.
///
/// Given: each robot has a Plan whose reservations are in the table, and
/// occupancy[cell] = robot there at tick start.
/// Approach: robots whose goal changed (or that haven't reached it and are
/// due a retry) release their reservations and replan in a rotating order.
/// A robot that can't reach its goal waits where it is (retrying with
/// backoff), unless it stands where nobody may park (another robot's goal, or
/// within kStationClearance of a station): then it moves to the nearest cell
/// outside those, so it can't block a goal or seal a station. Waiting is always safe (the old plan
/// parked it there); mid-path it keeps the old plan instead. Then every
/// robot steps to its plan's next cell, all at once.
void spaceTimeSystem(Registry& reg, const Grid& grid, DistanceCache& dist, ReservationTable& table,
                     std::vector<Entity>& occupancy, Stats& stats);

inline constexpr int kMaxRetryBackoff = 8;   // ticks between replans when stuck
inline constexpr int kStationClearance = 2;  // no parking this close to a station

}  // namespace ww
