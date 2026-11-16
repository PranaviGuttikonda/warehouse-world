#include "ww/stastar.hpp"

#include <algorithm>
#include <cassert>
#include <queue>
#include <unordered_set>

namespace ww {

namespace {

struct Node {
    int f;
    int g;  // ticks since startTick
    int cell;
    bool done;   // goal already touched on the way here
    int id;      // index into the node store, doubles as push order
    int parent;  // node id, -1 for the start
};

struct Worse {
    bool operator()(const Node& a, const Node& b) const {
        if (a.f != b.f) return a.f > b.f;
        if (a.g != b.g) return a.g < b.g;
        if (a.cell != b.cell) return a.cell > b.cell;
        if (a.done != b.done) return !a.done;
        return a.id > b.id;
    }
};

}  // namespace

std::optional<SpaceTimePlan> planSpaceTime(const Grid& grid, const ReservationTable& table,
                                           const std::vector<int>& distToGoal, Cell start, Cell goal,
                                           long long startTick, const std::vector<int>& noPark,
                                           bool allowFallback, SpaceTimeLimits limits) {
    const int h0 = distToGoal[grid.index(start)];
    if (h0 == DistanceCache::kUnreachable) return std::nullopt;
    const int maxG = h0 + limits.slack;
    const long long cells = grid.width() * grid.height();
    const int goalIdx = grid.index(goal);
    // Before the goal: exact distance, plus one tick to step off it again.
    auto heuristic = [&](int cell, bool done) { return done ? 0 : distToGoal[cell] + 1; };

    std::vector<Node> store;
    std::priority_queue<Node, std::vector<Node>, Worse> open;
    std::unordered_set<long long> closed;  // (g * cells + cell) * 2 + done
    auto key = [&](int g, int cell, bool done) { return (g * cells + cell) * 2 + done; };
    auto push = [&](int g, int cell, bool done, int parent) {
        Node n{g + heuristic(cell, done), g, cell, done, static_cast<int>(store.size()), parent};
        store.push_back(n);
        open.push(n);
    };
    push(0, grid.index(start), start == goal, -1);

    auto planTo = [&](int id, bool reached) {
        SpaceTimePlan plan{{}, reached, 0};
        for (; id != -1; id = store[id].parent) plan.path.push_back(grid.cellAt(store[id].cell));
        std::reverse(plan.path.begin(), plan.path.end());
        auto it = std::find(plan.path.begin(), plan.path.end(), goal);
        plan.goalIndex = static_cast<std::size_t>(it - plan.path.begin());
        return plan;
    };
    int fallback = -1;  // best parkable node short of the goal: nearest, then earliest

    int expansions = 0;
    while (!open.empty() && expansions < limits.maxExpansions) {
        Node n = open.top();
        open.pop();
        if (!closed.insert(key(n.g, n.cell, n.done)).second) continue;
        ++expansions;

        const Cell here = grid.cellAt(n.cell);
        const long long t = startTick + n.g;
        const bool parkable = noPark[n.cell] == 0 && table.canParkFrom(here, t);
        if (n.done && parkable) return planTo(n.id, true);
        if (!n.done && allowFallback && parkable) {
            const Node* best = fallback == -1 ? nullptr : &store[fallback];
            const int h = distToGoal[n.cell];
            if (!best || h < distToGoal[best->cell] || (h == distToGoal[best->cell] && n.g < best->g))
                fallback = n.id;
        }
        if (n.g + 1 > maxG) continue;

        // Wait first, then the four moves; order only matters for ties.
        const Cell steps[5] = {{0, 0}, kNeighbors[0], kNeighbors[1], kNeighbors[2], kNeighbors[3]};
        for (Cell d : steps) {
            const Cell next{here.x + d.x, here.y + d.y};
            if (!grid.passable(next)) continue;
            const int i = grid.index(next);
            if (distToGoal[i] == DistanceCache::kUnreachable) continue;
            const bool done = n.done || i == goalIdx;
            if (!done && n.g + 1 + distToGoal[i] > maxG) continue;  // can't make the horizon
            if (closed.count(key(n.g + 1, i, done))) continue;
            if (!table.canMove(here, next, t)) continue;
            push(n.g + 1, i, done, n.id);
        }
    }
    if (fallback == -1) return std::nullopt;
    return planTo(fallback, false);
}

void spaceTimeSystem(Registry& reg, const Grid& grid, DistanceCache& dist, ReservationTable& table,
                     std::vector<Entity>& occupancy, Stats& stats) {
    auto& robots = reg.pool<Robot>();
    const auto& order = robots.entities();
    const std::size_t n = order.size();
    if (n == 0) return;
    const long long now = stats.ticks;
    const std::size_t first = static_cast<std::size_t>(now % static_cast<long long>(n));

    // No parking on anyone's current goal, nor within kStationClearance of a
    // station: a corner station has only two access cells, and robots
    // parked there would seal it.
    std::vector<int> noPark(occupancy.size(), 0);
    for (Entity e : order) ++noPark[grid.index(reg.get<Task>(e).goal())];
    for (Cell s : grid.stations()) {
        const auto& field = dist.to(s);
        for (std::size_t i = 0; i < field.size(); ++i)
            if (field[i] != DistanceCache::kUnreachable && field[i] <= kStationClearance) ++noPark[i];
    }

    // 1. Replan whoever needs it, in a rotating order for fairness.
    for (std::size_t k = 0; k < n; ++k) {
        const Entity e = order[(first + k) % n];
        Plan& plan = reg.get<Plan>(e);
        const Cell here = reg.get<Position>(e).cell;
        const Cell goal = reg.get<Task>(e).goal();
        const bool newGoal = !(plan.goal == goal);
        if (!newGoal && !(plan.stuck && now >= plan.retryAt)) continue;

        table.releasePath(e, plan.path, plan.start);
        // Standing where nobody may park (someone's goal, a station approach):
        // if the goal is out of reach, move out of the way instead of waiting.
        const bool inTheWay = !(here == goal) && noPark[grid.index(here)] > 0;
        auto result = planSpaceTime(grid, table, dist.to(goal), here, goal, now, noPark, inTheWay);
        const int backoff = newGoal ? 1 : std::min(plan.backoff * 2, kMaxRetryBackoff);
        if (result && result->reachesGoal) {
            plan = Plan{std::move(result->path), now, goal, false, 0, 0};
        } else if (result) {
            plan = Plan{std::move(result->path), now, goal, true, now + backoff, backoff};
        } else {
            if (table.canParkFrom(here, now)) {
                plan = Plan{{here}, now, goal, true, now + backoff, backoff};
            } else {
                // Mid-path (e.g. passing over its goal early): parking here could
                // hit a later reservation, so keep the old, still-valid plan.
                plan.goal = goal;
                plan.stuck = true;
                plan.retryAt = now + backoff;
                plan.backoff = backoff;
            }
        }
        table.reservePath(e, plan.path, plan.start);
    }

    // 2. Everyone steps along their plan at once.
    std::vector<std::pair<Entity, Cell>> moves;
    for (Entity e : order) {
        const Plan& plan = reg.get<Plan>(e);
        Robot& robot = robots.get(e);
        const Cell here = reg.get<Position>(e).cell;
        const auto k = static_cast<std::size_t>(now + 1 - plan.start);
        const Cell next = k < plan.path.size() ? plan.path[k] : plan.path.back();
        assert(manhattan(here, next) <= 1);
        if (next == here) {
            if (!(here == reg.get<Task>(e).goal())) {
                ++robot.waitTicks;
                ++stats.waits;
            }
            continue;
        }
        moves.emplace_back(e, next);
    }
    for (auto& [e, to] : moves) {
        Cell& cell = reg.get<Position>(e).cell;
        if (occupancy[grid.index(cell)] == e) occupancy[grid.index(cell)] = kNullEntity;
        cell = to;
        Robot& robot = robots.get(e);
        robot.waitTicks = 0;
        ++robot.moves;
        ++stats.moves;
    }
    for (auto& [e, to] : moves) occupancy[grid.index(to)] = e;
}

}  // namespace ww
