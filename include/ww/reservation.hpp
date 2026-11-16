#pragma once

#include <map>
#include <vector>

#include "ww/ecs.hpp"
#include "ww/grid.hpp"

namespace ww {

/// Who holds which cell when, for space-time planning.
///
/// Given: robots that commit to a path of one cell per tick and then stay
/// on its last cell until they replan.
/// Approach: per cell, an ordered map tick -> robot for the path cells, plus
/// one "parked from tick t onward" slot for the robot that ends there.
/// Swaps need no separate edge table: robot x moving b -> a during t -> t+1
/// shows up as vertex (b, t) and (a, t+1) both held by x.
class ReservationTable {
public:
    explicit ReservationTable(const Grid& grid);

    /// Given: path[k] = cell at tick start + k. Approach: hold each cell for
    /// its tick, and park the last one from its tick onward.
    void reservePath(Entity e, const std::vector<Cell>& path, long long start);
    /// Undo reservePath for the same arguments (only entries e still owns).
    void releasePath(Entity e, const std::vector<Cell>& path, long long start);

    /// Robot holding cell at tick t (path or park), else kNullEntity.
    Entity holder(Cell c, long long t) const;
    bool isFree(Cell c, long long t) const { return holder(c, t) == kNullEntity; }
    /// Can a robot move from -> to during t -> t+1 (to free at t+1, no swap)?
    /// from == to means waiting in place.
    bool canMove(Cell from, Cell to, long long t) const;
    /// Can a robot stay on c from tick t forever: no park and no path
    /// reservation on c at t or later?
    bool canParkFrom(Cell c, long long t) const;

private:
    struct Park {
        long long from = 0;
        Entity robot = kNullEntity;
    };
    const Grid& grid_;
    std::vector<std::map<long long, Entity>> path_;  // by cell index
    std::vector<Park> park_;                           // by cell index
};

}  // namespace ww
