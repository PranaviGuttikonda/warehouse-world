#pragma once

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

}  // namespace ww
