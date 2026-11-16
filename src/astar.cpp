#include "ww/astar.hpp"

#include <algorithm>
#include <queue>

namespace ww {

namespace {

struct Node {
    int f;
    int g;
    int index;
};

// Min-heap order: lowest f, then highest g, then lowest index.
struct Worse {
    bool operator()(const Node& a, const Node& b) const {
        if (a.f != b.f) return a.f > b.f;
        if (a.g != b.g) return a.g < b.g;
        return a.index > b.index;
    }
};

}  // namespace

std::optional<std::vector<Cell>> astar(const Grid& grid, Cell start, Cell goal) {
    if (!grid.passable(start) || !grid.passable(goal)) return std::nullopt;

    const auto size = static_cast<std::size_t>(grid.width() * grid.height());
    constexpr int kUnseen = -1;
    std::vector<int> bestG(size, kUnseen);
    std::vector<int> parent(size, kUnseen);
    std::vector<bool> closed(size, false);
    std::priority_queue<Node, std::vector<Node>, Worse> open;

    const int startIdx = grid.index(start), goalIdx = grid.index(goal);
    bestG[startIdx] = 0;
    open.push({manhattan(start, goal), 0, startIdx});

    while (!open.empty()) {
        Node n = open.top();
        open.pop();
        if (closed[n.index]) continue;  // stale entry: a cheaper copy was expanded
        closed[n.index] = true;

        if (n.index == goalIdx) {
            std::vector<Cell> path;
            for (int i = goalIdx; i != kUnseen; i = parent[i]) path.push_back(grid.cellAt(i));
            std::reverse(path.begin(), path.end());
            return path;
        }

        Cell c = grid.cellAt(n.index);
        for (Cell d : kNeighbors) {
            Cell next{c.x + d.x, c.y + d.y};
            if (!grid.passable(next)) continue;
            int i = grid.index(next);
            int g = n.g + 1;
            if (closed[i] || (bestG[i] != kUnseen && bestG[i] <= g)) continue;
            bestG[i] = g;
            parent[i] = n.index;
            open.push({g + manhattan(next, goal), g, i});
        }
    }
    return std::nullopt;
}

}  // namespace ww
