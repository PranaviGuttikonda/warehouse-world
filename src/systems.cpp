#include "ww/systems.hpp"

#include <algorithm>
#include <array>
#include <deque>

namespace ww {

const std::vector<int>& DistanceCache::to(Cell goal) {
    int key = grid_.index(goal);
    auto it = fields_.find(key);
    if (it != fields_.end()) return it->second;

    std::vector<int> field(static_cast<std::size_t>(grid_.width() * grid_.height()), kUnreachable);
    std::deque<Cell> frontier{goal};
    field[key] = 0;
    while (!frontier.empty()) {
        Cell c = frontier.front();
        frontier.pop_front();
        int d = field[grid_.index(c)];
        for (Cell step : kNeighbors) {
            Cell n{c.x + step.x, c.y + step.y};
            if (!grid_.passable(n) || field[grid_.index(n)] != kUnreachable) continue;
            field[grid_.index(n)] = d + 1;
            frontier.push_back(n);
        }
    }
    return fields_.emplace(key, std::move(field)).first->second;
}

Task makeTask(const Grid& grid, Rng& rng) {
    const auto& faces = grid.pickFaces();
    const auto& stations = grid.stations();
    std::uniform_int_distribution<std::size_t> pickFace(0, faces.size() - 1);
    std::uniform_int_distribution<std::size_t> pickStation(0, stations.size() - 1);
    return Task{faces[pickFace(rng)], stations[pickStation(rng)], Task::Stage::ToPickup};
}

void taskSystem(Registry& reg, const Grid& grid, Rng& rng, Stats& stats) {
    auto& tasks = reg.pool<Task>();
    for (Entity e : tasks.entities()) {
        Task& task = tasks.get(e);
        Cell at = reg.get<Position>(e).cell;
        if (at != task.goal()) continue;
        if (task.stage == Task::Stage::ToPickup) {
            task.stage = Task::Stage::ToDropoff;
        } else {
            ++stats.deliveries;
            ++reg.get<Robot>(e).deliveries;
            task = makeTask(grid, rng);
        }
    }
}

void movementSystem(Registry& reg, const Grid& grid, DistanceCache& dist,
                    std::vector<Entity>& occupancy, Rng& rng, Stats& stats) {
    auto& robots = reg.pool<Robot>();
    const auto& order = robots.entities();
    const std::size_t n = order.size();
    if (n == 0) return;

    // Cells claimed as destinations this tick.
    std::vector<bool> claimed(occupancy.size(), false);
    auto isFree = [&](Cell c) {
        int i = grid.index(c);
        return grid.passable(c) && occupancy[i] == kNullEntity && !claimed[i];
    };

    // Rotate who moves first so no robot always gets priority.
    const std::size_t start = static_cast<std::size_t>(stats.ticks % static_cast<long long>(n));
    std::vector<std::pair<Entity, Cell>> moves;
    moves.reserve(n);

    for (std::size_t k = 0; k < n; ++k) {
        Entity e = order[(start + k) % n];
        Robot& robot = robots.get(e);
        Cell here = reg.get<Position>(e).cell;
        Cell goal = reg.get<Task>(e).goal();
        if (here == goal) continue;

        const std::vector<int>& field = dist.to(goal);
        int hereDist = field[grid.index(here)];

        std::array<Cell, 4> options{};
        for (int i = 0; i < 4; ++i)
            options[i] = {here.x + kNeighbors[i].x, here.y + kNeighbors[i].y};
        std::shuffle(options.begin(), options.end(), rng);  // random tie-break

        Cell chosen = here;
        for (Cell c : options) {
            if (!grid.passable(c) || !isFree(c)) continue;
            int d = field[grid.index(c)];
            if (d != DistanceCache::kUnreachable && d < hereDist) {
                chosen = c;
                break;
            }
        }
        if (chosen == here && robot.waitTicks >= kSidestepAfter) {
            for (Cell c : options) {
                if (isFree(c)) {
                    chosen = c;
                    break;
                }
            }
        }

        if (chosen == here) {
            ++robot.waitTicks;
            ++stats.waits;
            continue;
        }
        claimed[grid.index(chosen)] = true;
        moves.emplace_back(e, chosen);
    }

    // Apply all moves at once, as if they happened simultaneously.
    for (auto& [e, to] : moves) {
        Cell& cell = reg.get<Position>(e).cell;
        occupancy[grid.index(cell)] = kNullEntity;
        cell = to;
        Robot& robot = robots.get(e);
        robot.waitTicks = 0;
        ++robot.moves;
        ++stats.moves;
    }
    for (auto& [e, to] : moves) occupancy[grid.index(to)] = e;
}

}  // namespace ww
