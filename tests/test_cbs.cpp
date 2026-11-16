#include <algorithm>

#include "test_framework.hpp"
#include "ww/cbs.hpp"
#include "ww/random.hpp"
#include "ww/simulation.hpp"

using namespace ww;

namespace {

constexpr const char* kRoom =
    "########\n"
    "#S.....#\n"
    "#......#\n"
    "#=....S#\n"
    "########\n";

// Row 1 is a corridor; (3,2) is a side bay.
constexpr const char* kBay =
    "#######\n"
    "#S...S#\n"
    "##=.=##\n"
    "#######\n";

// Last tick that matters for an agent: its arrival, else the window end.
int endOf(const std::vector<Cell>& path, Cell goal) {
    auto it = std::find(path.begin(), path.end(), goal);
    return it == path.end() ? static_cast<int>(path.size()) - 1 : static_cast<int>(it - path.begin());
}

// Given: CBS paths. Approach: check every pair for a shared cell or a swap
// at every tick both are still active (fullWindow: ignore arrivals).
bool conflictFree(const std::vector<std::vector<Cell>>& paths, const std::vector<CbsAgent>& agents,
                  bool fullWindow = false) {
    const std::size_t n = paths.size();
    for (std::size_t a = 0; a < n; ++a) {
        for (std::size_t b = a + 1; b < n; ++b) {
            const int last = static_cast<int>(paths[a].size()) - 1;
            const int end = fullWindow ? last : std::min(endOf(paths[a], agents[a].goal), endOf(paths[b], agents[b].goal));
            for (int t = 0; t <= end; ++t) {
                if (paths[a][t] == paths[b][t]) return false;
                if (t < end && paths[a][t] == paths[b][t + 1] && paths[b][t] == paths[a][t + 1]) return false;
            }
        }
    }
    return true;
}

bool validSteps(const Grid& g, const std::vector<Cell>& p) {
    for (std::size_t i = 0; i < p.size(); ++i) {
        if (!g.passable(p[i])) return false;
        if (i > 0 && manhattan(p[i - 1], p[i]) > 1) return false;
    }
    return true;
}

// Given: a seed. Approach: n agents on distinct random floor cells, each
// heading to a random pick face or station.
std::vector<CbsAgent> randomAgents(const Grid& g, DistanceCache& dist, int n, std::uint64_t seed) {
    Rng rng(seed);
    std::vector<Cell> cells = g.floorCells();
    ww::shuffle(cells.begin(), cells.end(), rng);
    std::vector<CbsAgent> agents;
    for (int i = 0; i < n; ++i) {
        const auto& targets = (i % 2) ? g.stations() : g.pickFaces();
        Cell goal = targets[uniformInt(rng, 0, targets.size() - 1)];
        agents.push_back({cells[static_cast<std::size_t>(i)], goal, &dist.to(goal)});
    }
    return agents;
}

}  // namespace

TEST(cbs_low_level_respects_constraints) {
    Grid g = Grid::parse(kRoom);
    DistanceCache dist(g);
    CbsAgent agent{{1, 1}, {4, 1}, &dist.to({4, 1})};
    std::vector<Cell> path;
    int cost = 0;

    CHECK(cbsLowLevel(g, agent, {}, 0, 6, path, cost));
    CHECK_EQ(cost, 3);
    CHECK_EQ(path.size(), std::size_t{7});  // padded to the window
    CHECK(path[3] == (Cell{4, 1}) && path[6] == (Cell{4, 1}));

    std::vector<CbsConstraint> cons{{0, g.index({2, 1}), -1, 1}};  // not on (2,1) at t=1
    CHECK(cbsLowLevel(g, agent, cons, 0, 6, path, cost));
    CHECK(!(path[1] == (Cell{2, 1})));
    CHECK(validSteps(g, path) && endOf(path, agent.goal) <= 4);

    cons = {{0, g.index({2, 1}), g.index({1, 1}), 0}};  // no (1,1) -> (2,1) during 0 -> 1
    CHECK(cbsLowLevel(g, agent, cons, 0, 6, path, cost));
    CHECK(!(path[0] == (Cell{1, 1}) && path[1] == (Cell{2, 1})));

    cons = {{1, g.index({2, 1}), -1, 1}};  // someone else's constraint: ignored
    CHECK(cbsLowLevel(g, agent, cons, 0, 6, path, cost));
    CHECK_EQ(cost, 3);
}

TEST(cbs_solves_a_corridor_swap) {
    // A and B start on each other's goal in a one-lane corridor: one of them
    // has to duck into the bay, which needs both planned together.
    Grid g = Grid::parse(kBay);
    DistanceCache dist(g);
    std::vector<CbsAgent> agents{{{1, 1}, {5, 1}, &dist.to({5, 1})}, {{5, 1}, {1, 1}, &dist.to({1, 1})}};
    CbsResult r = solveCbs(g, agents, {12, 2000});
    CHECK(r.solvedByCbs);
    CHECK(conflictFree(r.paths, agents));
    CHECK(r.paths[0].back() == (Cell{5, 1}));
    CHECK(r.paths[1].back() == (Cell{1, 1}));
    bool usedBay = false;
    for (const auto& p : r.paths) usedBay = usedBay || std::find(p.begin(), p.end(), Cell{3, 2}) != p.end();
    CHECK(usedBay);
}

TEST(cbs_paths_are_conflict_free) {
    Grid g = Grid::parse(defaultMap());
    DistanceCache dist(g);
    for (std::uint64_t seed = 1; seed <= 5; ++seed) {
        auto agents = randomAgents(g, dist, 30, seed);
        CbsResult r = solveCbs(g, agents);
        CHECK(conflictFree(r.paths, agents, !r.solvedByCbs));
        for (std::size_t a = 0; a < agents.size(); ++a) {
            CHECK(r.paths[a].front() == agents[a].start);
            CHECK(validSteps(g, r.paths[a]));
        }
    }
}

TEST(cbs_fallback_is_conflict_free) {
    Grid g = Grid::parse(defaultMap());
    DistanceCache dist(g);
    auto agents = randomAgents(g, dist, 60, 9);
    CbsResult r = solveCbs(g, agents, {8, 0});  // no budget: straight to the fallback
    CHECK(!r.solvedByCbs);
    CHECK(conflictFree(r.paths, agents, true));  // whole window, arrivals or not
    int moved = 0;
    for (const auto& p : r.paths) moved += !(p.back() == p.front());
    CHECK(moved > 0);
}

TEST(cbs_simulation_stays_collision_free) {
    Simulation sim(Grid::parse(defaultMap()), {40, 3, Planner::Cbs});
    std::vector<RobotView> before = sim.robots();
    bool ok = true;
    for (int t = 0; t < 3000 && ok; ++t) {
        sim.step();
        std::vector<RobotView> after = sim.robots();
        if (std::string problem = sim.validate(); !problem.empty()) {
            ::wwtest::fail(__FILE__, __LINE__, "tick " + std::to_string(t) + ": " + problem);
            ok = false;
        }
        for (std::size_t i = 0; i < after.size() && ok; ++i) {
            if (manhattan(before[i].cell, after[i].cell) > 1) ok = false;
            for (std::size_t j = i + 1; j < after.size(); ++j)
                if (after[i].cell == before[j].cell && after[j].cell == before[i].cell) ok = false;
        }
        before = std::move(after);
    }
    CHECK(ok);
}

TEST(cbs_fixes_the_stastar_gridlock_seeds) {
    // With 40 robots, space-time A* gridlocks on seeds 1, 3 and 4 (3,212 /
    // 4,533 / 2,661 deliveries in 20,000 ticks, stalled by tick ~4,000-5,600).
    // CBS keeps delivering: ~23,500 in 20,000 ticks; check the first 6,000.
    for (std::uint64_t seed : {1, 3, 4}) {
        Simulation sim(Grid::parse(defaultMap()), {40, seed, Planner::Cbs});
        for (int t = 0; t < 6000; ++t) sim.step();
        CHECK(sim.stats().deliveries > 5000);
        CHECK(sim.validate().empty());
    }
}

TEST(cbs_simulation_is_deterministic) {
    Simulation a(Grid::parse(defaultMap()), {24, 5, Planner::Cbs});
    Simulation b(Grid::parse(defaultMap()), {24, 5, Planner::Cbs});
    for (int t = 0; t < 1000; ++t) a.step(), b.step();
    CHECK_EQ(a.stats().deliveries, b.stats().deliveries);
    CHECK_EQ(a.stats().moves, b.stats().moves);
    CHECK_EQ(a.stats().cbsFallbacks, b.stats().cbsFallbacks);
}
