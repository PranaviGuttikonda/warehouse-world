#pragma once

#include <vector>

#include "ww/types.hpp"

namespace ww {

struct Position {
    Cell cell;
};

/// A pick-and-deliver job: go to a shelf face, then to a station.
struct Task {
    enum class Stage { ToPickup, ToDropoff };
    Cell pickup;
    Cell dropoff;
    Stage stage = Stage::ToPickup;

    Cell goal() const { return stage == Stage::ToPickup ? pickup : dropoff; }
    bool carrying() const { return stage == Stage::ToDropoff; }
};

struct Robot {
    int id = 0;            // display id, 0-based
    int waitTicks = 0;     // consecutive ticks blocked
    int deliveries = 0;
    int moves = 0;
};

/// A robot's committed space-time path (space-time A* planner only).
/// path[k] is its cell at tick start + k; it parks on path.back() after.
struct Plan {
    std::vector<Cell> path;
    long long start = 0;
    Cell goal{-1, -1};      // goal this plan was made for; differs -> replan
    bool stuck = false;     // planning failed, parked in place
    long long retryAt = 0;  // tick of the next attempt when stuck
    int backoff = 0;        // ticks between attempts, doubles while stuck
};

}  // namespace ww
