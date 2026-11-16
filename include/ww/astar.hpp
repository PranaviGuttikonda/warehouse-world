#pragma once

#include <optional>
#include <vector>

#include "ww/grid.hpp"
#include "ww/types.hpp"

namespace ww {

/// Shortest path for one robot, ignoring other robots.
///
/// Given: a static grid and passable start and goal cells.
/// Approach: A* over 4-connected cells with the Manhattan heuristic, which
/// never overestimates on this grid, so the first time the goal is popped
/// the path is shortest. Ties go to the lowest f, then the highest g (deeper
/// nodes first), then the lowest cell index, so results are deterministic.
/// Returns cells from start to goal inclusive, or nullopt if unreachable.
std::optional<std::vector<Cell>> astar(const Grid& grid, Cell start, Cell goal);

}  // namespace ww
