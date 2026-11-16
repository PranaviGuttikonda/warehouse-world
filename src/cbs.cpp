#include "ww/cbs.hpp"

#include <algorithm>
#include <cassert>
#include <queue>
#include <unordered_set>

namespace ww {

namespace {

struct LowNode {
    int f;
    int g;
    int cell;
    int id;
    int parent;
};

struct LowWorse {
    bool operator()(const LowNode& a, const LowNode& b) const {
        if (a.f != b.f) return a.f > b.f;
        if (a.g != b.g) return a.g < b.g;
        if (a.cell != b.cell) return a.cell > b.cell;
        return a.id > b.id;
    }
};

struct Conflict {
    int t = -1;
    int a = -1, b = -1;
    int cell = -1;  // vertex: shared cell; edge: a's from cell
    int to = -1;    // edge only: a's to cell (b moves to -> cell)
};

// Given: paths of window + 1 cells and each agent's last relevant tick
// (arrival at its goal, else the window end). Approach: per tick, map
// active agents to cells; a repeat is a vertex conflict, a mirrored move is
// a swap. Ticks after an agent arrives never run: arriving changes its
// task, which makes everyone replan. Returns the earliest conflict (vertex
// before swap at the same tick, then lowest agents) and counts all of them.
Conflict findConflicts(const Grid& grid, const std::vector<std::vector<Cell>>& paths, const std::vector<int>& end,
                       int window, int& count) {
    const int cells = grid.width() * grid.height();
    const int n = static_cast<int>(paths.size());
    std::vector<int> now(cells, -1), next(cells, -1);
    Conflict first;
    count = 0;
    auto fill = [&](std::vector<int>& at, int t) {
        std::fill(at.begin(), at.end(), -1);
        for (int a = 0; a < n; ++a) {
            if (t > end[a]) continue;
            const int c = grid.index(paths[a][t]);
            if (at[c] == -1) at[c] = a;
        }
    };
    fill(now, 0);
    for (int t = 0; t <= window; ++t) {
        // Vertex conflicts at t.
        std::vector<int> seen(cells, -1);
        for (int a = 0; a < n; ++a) {
            if (t > end[a]) continue;
            const int c = grid.index(paths[a][t]);
            if (seen[c] != -1) {
                ++count;
                if (first.t == -1) first = {t, seen[c], a, c, -1};
            } else {
                seen[c] = a;
            }
        }
        if (t == window) break;
        // Swaps during t -> t+1.
        fill(next, t + 1);
        for (int a = 0; a < n; ++a) {
            if (t + 1 > end[a]) continue;
            const int u = grid.index(paths[a][t]), v = grid.index(paths[a][t + 1]);
            if (u == v) continue;
            const int b = now[v];
            if (b > a && t + 1 <= end[b] && grid.index(paths[b][t + 1]) == u) {
                ++count;
                if (first.t == -1) first = {t, a, b, u, v};
            }
        }
        std::swap(now, next);
    }
    return first;
}

struct CtNode {
    std::vector<CbsConstraint> constraints;
    std::vector<std::vector<Cell>> paths;
    std::vector<int> costs;
    int cost = 0;
    int conflicts = 0;
    Conflict first;
};

struct CtRef {
    int cost, conflicts, id;
};

// Greedy order: fewest conflicts first, then cost. Plain CBS (cost first)
// almost never finished within budget at 40 robots; this finds
// conflict-free plans fast at the price of optimality.
struct CtWorse {
    bool operator()(const CtRef& a, const CtRef& b) const {
        if (a.conflicts != b.conflicts) return a.conflicts > b.conflicts;
        if (a.cost != b.cost) return a.cost > b.cost;
        return a.id > b.id;
    }
};

}  // namespace

bool cbsLowLevel(const Grid& grid, const CbsAgent& agent, const std::vector<CbsConstraint>& constraints,
                 int agentIndex, int window, std::vector<Cell>& path, int& cost) {
    const long long cells = grid.width() * grid.height();
    const int goal = grid.index(agent.goal);
    const std::vector<int>& dist = *agent.distToGoal;
    std::unordered_set<long long> vertex, edge;
    int lastGoalConstraint = -1;
    for (const CbsConstraint& c : constraints) {
        if (c.agent != agentIndex) continue;
        if (c.from < 0) {
            vertex.insert(c.t * cells + c.cell);
            if (c.cell == goal) lastGoalConstraint = std::max(lastGoalConstraint, c.t);
        } else {
            edge.insert((c.t * cells + c.from) * cells + c.cell);
        }
    }
    const int start = grid.index(agent.start);
    if (vertex.count(start)) return false;
    auto h = [&](int cell) { return dist[cell] == DistanceCache::kUnreachable ? 0 : dist[cell]; };

    std::vector<LowNode> store;
    std::priority_queue<LowNode, std::vector<LowNode>, LowWorse> open;
    std::unordered_set<long long> closed;
    auto push = [&](int g, int cell, int parent) {
        LowNode n{g + h(cell), g, cell, static_cast<int>(store.size()), parent};
        store.push_back(n);
        open.push(n);
    };
    push(0, start, -1);

    while (!open.empty()) {
        LowNode n = open.top();
        open.pop();
        if (!closed.insert(n.g * cells + n.cell).second) continue;
        const bool atGoal = n.cell == goal && n.g > lastGoalConstraint;
        if (atGoal || n.g == window) {
            path.assign(static_cast<std::size_t>(window + 1), grid.cellAt(n.cell));
            for (int id = n.id; id != -1; id = store[id].parent)
                path[static_cast<std::size_t>(store[id].g)] = grid.cellAt(store[id].cell);
            cost = atGoal ? n.g : window + h(n.cell);
            return true;
        }
        const Cell here = grid.cellAt(n.cell);
        const Cell steps[5] = {{0, 0}, kNeighbors[0], kNeighbors[1], kNeighbors[2], kNeighbors[3]};
        for (Cell d : steps) {
            const Cell next{here.x + d.x, here.y + d.y};
            if (!grid.passable(next)) continue;
            const int i = grid.index(next);
            const int g = n.g + 1;
            if (closed.count(g * cells + i) || vertex.count(g * cells + i)) continue;
            if (i != n.cell && edge.count((n.g * cells + n.cell) * cells + i)) continue;
            push(g, i, n.id);
        }
    }
    return false;
}

CbsResult solveCbs(const Grid& grid, const std::vector<CbsAgent>& agents, CbsLimits limits) {
    const int n = static_cast<int>(agents.size());
    const int window = limits.window;
    CbsResult result{std::vector<std::vector<Cell>>(static_cast<std::size_t>(n)), true, 0};
    if (n == 0) return result;

    std::vector<CtNode> tree;
    std::priority_queue<CtRef, std::vector<CtRef>, CtWorse> open;
    std::vector<int> end(static_cast<std::size_t>(n));
    auto evaluate = [&](CtNode& node) {
        node.cost = 0;
        for (int a = 0; a < n; ++a) {
            node.cost += node.costs[a];
            end[a] = std::min(node.costs[a], window);  // cost = arrival tick when it arrives
        }
        node.first = findConflicts(grid, node.paths, end, window, node.conflicts);
    };

    CtNode root;
    root.paths.resize(static_cast<std::size_t>(n));
    root.costs.resize(static_cast<std::size_t>(n));
    bool rootOk = true;
    for (int a = 0; a < n; ++a)
        rootOk = rootOk && cbsLowLevel(grid, agents[a], root.constraints, a, window, root.paths[a], root.costs[a]);
    if (rootOk) {
        evaluate(root);
        tree.push_back(std::move(root));
        open.push({tree[0].cost, tree[0].conflicts, 0});
    }

    while (!open.empty()) {
        const int id = open.top().id;
        open.pop();
        if (tree[id].conflicts == 0) {
            result.paths = tree[id].paths;
            return result;
        }
        if (++result.nodes > limits.maxNodes) break;

        const Conflict c = tree[id].first;
        // One child per agent in the conflict, each forbidding it for that agent.
        const CbsConstraint split[2] = {
            c.to < 0 ? CbsConstraint{c.a, c.cell, -1, c.t} : CbsConstraint{c.a, c.to, c.cell, c.t},
            c.to < 0 ? CbsConstraint{c.b, c.cell, -1, c.t} : CbsConstraint{c.b, c.cell, c.to, c.t},
        };
        for (const CbsConstraint& con : split) {
            CtNode child;
            child.constraints = tree[id].constraints;
            child.constraints.push_back(con);
            child.paths = tree[id].paths;
            child.costs = tree[id].costs;
            if (!cbsLowLevel(grid, agents[con.agent], child.constraints, con.agent, window,
                             child.paths[con.agent], child.costs[con.agent]))
                continue;
            evaluate(child);
            tree.push_back(std::move(child));
            const int cid = static_cast<int>(tree.size()) - 1;
            open.push({tree[cid].cost, tree[cid].conflicts, cid});
        }
    }

    // Fallback: one at a time. Agent a avoids the paths of 0..a-1 (cells and
    // swaps) and the start cells of a+1..n-1 at every tick, so earlier agents
    // never enter a later agent's cell and every agent can at least wait.
    result.solvedByCbs = false;
    std::vector<CbsConstraint> cons;
    for (int a = 0; a < n; ++a) {
        cons.clear();
        for (int b = 0; b < a; ++b) {
            const auto& p = result.paths[b];
            for (int t = 0; t <= window; ++t) {
                cons.push_back({a, grid.index(p[t]), -1, t});
                if (t < window && !(p[t] == p[t + 1]))
                    cons.push_back({a, grid.index(p[t]), grid.index(p[t + 1]), t});  // no b-swap
            }
        }
        for (int b = a + 1; b < n; ++b)
            for (int t = 0; t <= window; ++t) cons.push_back({a, grid.index(agents[b].start), -1, t});
        int cost = 0;
        const bool ok = cbsLowLevel(grid, agents[a], cons, a, window, result.paths[a], cost);
        assert(ok && "waiting in place is always allowed");
        (void)ok;
    }
    return result;
}

void cbsSystem(Registry& reg, const Grid& grid, DistanceCache& dist, std::vector<Entity>& occupancy,
               Stats& stats, CbsLimits limits) {
    auto& robots = reg.pool<Robot>();
    const auto& order = robots.entities();
    const std::size_t n = order.size();
    if (n == 0) return;
    const long long now = stats.ticks;

    bool replan = false;
    for (Entity e : order) {
        const Plan& plan = reg.get<Plan>(e);
        replan = replan || !(plan.goal == reg.get<Task>(e).goal()) || now - plan.start >= kCbsReplanEvery;
    }
    if (replan) {
        // Rotate who goes first, which only matters if the fallback runs.
        const std::size_t first = static_cast<std::size_t>(now % static_cast<long long>(n));
        std::vector<Entity> rotated(n);
        std::vector<CbsAgent> agents(n);
        for (std::size_t k = 0; k < n; ++k) {
            const Entity e = order[(first + k) % n];
            const Cell goal = reg.get<Task>(e).goal();
            rotated[k] = e;
            agents[k] = {reg.get<Position>(e).cell, goal, &dist.to(goal)};
        }
        CbsResult solved = solveCbs(grid, agents, limits);
        ++stats.cbsSolves;
        if (!solved.solvedByCbs) ++stats.cbsFallbacks;
        for (std::size_t k = 0; k < n; ++k)
            reg.get<Plan>(rotated[k]) = Plan{std::move(solved.paths[k]), now, agents[k].goal, false, 0, 0};
    }

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
