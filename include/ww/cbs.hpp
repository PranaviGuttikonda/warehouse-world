#pragma once

#include <vector>

#include "ww/components.hpp"
#include "ww/ecs.hpp"
#include "ww/grid.hpp"
#include "ww/systems.hpp"

namespace ww {

/// One robot in a CBS instance.
struct CbsAgent {
    Cell start;
    Cell goal;
    const std::vector<int>* distToGoal;  // BFS field from DistanceCache
};

/// A constraint on one agent: don't be on cell at tick t (vertex), or don't
/// move from -> cell during t -> t+1 (edge, from >= 0). Ticks are relative.
struct CbsConstraint {
    int agent;
    int cell;
    int from;  // -1 for a vertex constraint
    int t;
};

struct CbsLimits {
    int window = 5;       // ticks of joint conflict checking (>= kCbsReplanEvery)
    int maxNodes = 1500;  // constraint-tree nodes before falling back
};

struct CbsResult {
    std::vector<std::vector<Cell>> paths;  // per agent, window + 1 cells (tick 0..window)
    bool solvedByCbs;                      // false: prioritized fallback was used
    int nodes;                             // constraint-tree nodes expanded
};

/// Shortest windowed path for one agent under its constraints.
///
/// Given: the agent and its constraints. Approach: A* over (cell, tick) up
/// to tick `window`, waits and 4 moves. Done when on the goal with no vertex
/// constraint on it later in the window (cost = arrival tick), or at the
/// window's end (cost = window + distance left). Same tie-breaks as
/// space-time A*. The returned path is padded to window + 1 cells.
/// Returns false only if even waiting is impossible (start constrained).
bool cbsLowLevel(const Grid& grid, const CbsAgent& agent, const std::vector<CbsConstraint>& constraints,
                 int agentIndex, int window, std::vector<Cell>& path, int& cost);

/// Windowed conflict-based search for all agents at once.
///
/// Given: agents on distinct cells. Approach: best-first search over a
/// constraint tree. Each node holds constraints and one path per agent;
/// pick the node with the fewest conflicts (then lowest sum of costs; this
/// greedy order trades optimality for finishing in budget), find
/// the earliest conflict within the window (same cell, or a swap) and split
/// it into two children, each forbidding it for one of the two agents and
/// replanning that agent. Conflicts after the window, or after either
/// agent reaches its goal, are ignored: the caller replans everyone as soon
/// as any robot arrives (its task changes) and at least every window ticks,
/// so those steps never run.
/// If no conflict-free node is found within maxNodes, falls back to
/// planning agents one at a time in the given order, where each avoids the
/// paths before it and the start cells of everyone after it. Waiting is
/// then always possible, so the fallback always returns conflict-free paths.
CbsResult solveCbs(const Grid& grid, const std::vector<CbsAgent>& agents, CbsLimits limits = {});

/// Moves every robot one tick along jointly planned paths.
///
/// Given: every robot has a Plan. Approach: when any robot has a new goal,
/// or kCbsReplanEvery ticks have passed, solve CBS for everyone from where
/// they stand (rotating the fallback order each time for fairness), then
/// step everyone along their path at once.
void cbsSystem(Registry& reg, const Grid& grid, DistanceCache& dist, std::vector<Entity>& occupancy,
               Stats& stats, CbsLimits limits = {});

inline constexpr int kCbsReplanEvery = 5;  // must be <= CbsLimits::window

}  // namespace ww
