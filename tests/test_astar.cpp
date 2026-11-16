#include "test_framework.hpp"
#include "ww/astar.hpp"
#include "ww/systems.hpp"

using namespace ww;

namespace {

// Given: a path. Approach: check it starts and ends right and every step is
// one move between passable cells.
bool validPath(const Grid& g, const std::vector<Cell>& path, Cell start, Cell goal) {
    if (path.empty() || path.front() != start || path.back() != goal) return false;
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!g.passable(path[i])) return false;
        if (i > 0 && manhattan(path[i - 1], path[i]) != 1) return false;
    }
    return true;
}

}  // namespace

TEST(astar_matches_bfs_shortest_distance) {
    Grid g = Grid::parse(defaultMap());
    DistanceCache dist(g);
    int checked = 0;
    for (Cell goal : g.stations()) {
        const auto& field = dist.to(goal);
        for (Cell start : g.pickFaces()) {
            auto path = astar(g, start, goal);
            CHECK(path.has_value());
            if (!path) continue;
            CHECK(validPath(g, *path, start, goal));
            CHECK_EQ(static_cast<int>(path->size()) - 1, field[g.index(start)]);
            ++checked;
        }
    }
    CHECK(checked > 100);
}

TEST(astar_goes_around_obstacles) {
    // Straight line from (1,1) to (5,1) is blocked by the wall column at x=3.
    Grid g = Grid::parse(
        "#######\n"
        "#S.#..#\n"
        "#..#=.#\n"
        "#.....#\n"
        "#######\n");
    auto path = astar(g, {1, 1}, {5, 1});
    CHECK(path.has_value());
    if (!path) return;
    CHECK(validPath(g, *path, {1, 1}, {5, 1}));
    CHECK_EQ(path->size(), std::size_t{9});  // 8 moves: down 2, right 4, up 2
    for (Cell c : *path) CHECK(c.x != 3 || c.y == 3);  // crosses the wall line only on row 3
}

TEST(astar_unreachable_goal_returns_nullopt) {
    // Right room is sealed off by walls.
    Grid g = Grid::parse(
        "#######\n"
        "#S.#..#\n"
        "#=.#..#\n"
        "#######\n");
    CHECK(!astar(g, {1, 1}, {4, 1}).has_value());
    CHECK(!astar(g, {1, 1}, {1, 2}).has_value());  // goal is a shelf
    CHECK(!astar(g, {0, 0}, {2, 1}).has_value());  // start is a wall
}

TEST(astar_start_equals_goal) {
    Grid g = Grid::parse(defaultMap());
    Cell c = g.stations().front();
    auto path = astar(g, c, c);
    CHECK(path.has_value());
    if (path) CHECK((*path == std::vector<Cell>{c}));
}

TEST(astar_is_deterministic) {
    Grid g = Grid::parse(defaultMap());
    Cell a = g.stations().front(), b = g.stations().back();
    CHECK((astar(g, a, b) == astar(g, a, b)));
}
