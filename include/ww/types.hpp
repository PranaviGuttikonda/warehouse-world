#pragma once

#include <cstdint>
#include <cstdlib>
#include <functional>

namespace ww {

/// A cell coordinate on the grid. x = column, y = row.
struct Cell {
    int x = 0;
    int y = 0;

    friend bool operator==(const Cell&, const Cell&) = default;
};

/// Given: two cells. Approach: |dx| + |dy| (4-connected grid distance).
inline int manhattan(Cell a, Cell b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

/// The four moves a robot can make, plus waiting in place.
inline constexpr Cell kNeighbors[4] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

}  // namespace ww
