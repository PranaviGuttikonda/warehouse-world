#include "test_framework.hpp"
#include "ww/reservation.hpp"
#include "ww/simulation.hpp"
#include "ww/stastar.hpp"

using namespace ww;

namespace {

// Open 6x2 room; stations and a shelf only so the map parses.
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

constexpr Entity kA = 1, kB = 2;

Cell at(const std::vector<Cell>& path, long long start, long long t) {
    if (t <= start) return path.front();
    auto k = static_cast<std::size_t>(t - start);
    return k < path.size() ? path[k] : path.back();
}

// Given: two committed paths (each parks on its last cell). Approach: walk
// both through time and look for a shared cell or a swap.
bool conflictFree(const std::vector<Cell>& a, long long sa, const std::vector<Cell>& b, long long sb) {
    const long long end = std::max(sa + static_cast<long long>(a.size()), sb + static_cast<long long>(b.size())) + 2;
    for (long long t = std::min(sa, sb); t < end; ++t) {
        if (at(a, sa, t) == at(b, sb, t)) return false;
        if (at(a, sa, t) == at(b, sb, t + 1) && at(b, sb, t) == at(a, sa, t + 1)) return false;
    }
    return true;
}

std::vector<int> goalsAt(const Grid& g, std::initializer_list<Cell> cells) {
    std::vector<int> goals(static_cast<std::size_t>(g.width() * g.height()), 0);
    for (Cell c : cells) ++goals[g.index(c)];
    return goals;
}

bool stepsAreUnit(const std::vector<Cell>& p) {
    for (std::size_t i = 1; i < p.size(); ++i)
        if (manhattan(p[i - 1], p[i]) > 1) return false;
    return true;
}

}  // namespace

TEST(reservation_holds_path_cells_then_parks) {
    Grid g = Grid::parse(kRoom);
    ReservationTable table(g);
    std::vector<Cell> path{{1, 1}, {2, 1}, {3, 1}};
    table.reservePath(kA, path, 10);

    CHECK_EQ(table.holder({1, 1}, 10), kA);
    CHECK_EQ(table.holder({2, 1}, 11), kA);
    CHECK_EQ(table.holder({3, 1}, 12), kA);
    CHECK_EQ(table.holder({3, 1}, 500), kA);  // parked from 12 onward
    CHECK(table.isFree({3, 1}, 11));          // not there yet
    CHECK(table.isFree({1, 1}, 11));          // already left
    CHECK(!table.canParkFrom({2, 1}, 5));     // A passes through at 11
    CHECK(table.canParkFrom({2, 1}, 12));
    CHECK(!table.canParkFrom({3, 1}, 50));    // A is parked there

    table.releasePath(kA, path, 10);
    CHECK(table.isFree({2, 1}, 11));
    CHECK(table.isFree({3, 1}, 500));
    CHECK(table.canParkFrom({3, 1}, 0));
}

TEST(reservation_rejects_swaps_but_allows_following) {
    Grid g = Grid::parse(kRoom);
    ReservationTable table(g);
    table.reservePath(kA, {{2, 1}, {3, 1}}, 0);  // A moves right during 0 -> 1
    CHECK(!table.canMove({3, 1}, {2, 1}, 0));     // head-on swap with A
    CHECK(table.canMove({1, 1}, {2, 1}, 0));      // following A into the cell it leaves
    CHECK(!table.canMove({4, 1}, {3, 1}, 0));     // A arrives there at t=1
    CHECK(table.canMove({2, 2}, {2, 2}, 0));      // waiting
}

TEST(stastar_waits_for_a_crossing_robot) {
    Grid g = Grid::parse(kBay);
    DistanceCache dist(g);
    ReservationTable table(g);
    // A pops out of the bay into the corridor for two ticks, then goes back.
    std::vector<Cell> a{{3, 2}, {3, 1}, {3, 1}, {3, 2}};
    table.reservePath(kA, a, 0);

    auto b = planSpaceTime(g, table, dist.to({5, 1}), {1, 1}, {5, 1}, 0, goalsAt(g, {{5, 1}}));
    CHECK(b.has_value() && b->reachesGoal);
    if (!b) return;
    CHECK(b->path.front() == (Cell{1, 1}));
    CHECK(stepsAreUnit(b->path));
    CHECK_EQ(b->goalIndex, std::size_t{5});    // 4 moves would cross A; B waits one tick
    CHECK(b->path.back() == (Cell{4, 1}));     // steps off the goal: touch and go
    CHECK(conflictFree(a, 0, b->path, 0));
}

TEST(stastar_falls_back_next_to_a_parked_goal) {
    Grid g = Grid::parse(kRoom);
    DistanceCache dist(g);
    ReservationTable table(g);
    table.reservePath(kA, {{6, 1}}, 0);  // A parked on B's goal for good
    const auto goals = goalsAt(g, {{6, 1}});
    CHECK(!planSpaceTime(g, table, dist.to({6, 1}), {1, 1}, {6, 1}, 0, goals).has_value());

    auto b = planSpaceTime(g, table, dist.to({6, 1}), {1, 1}, {6, 1}, 0, goals, true);
    CHECK(b.has_value());
    if (!b) return;
    CHECK(!b->reachesGoal);
    CHECK(b->path.back() == (Cell{5, 1}));  // nearest parkable cell, reached first
    CHECK(stepsAreUnit(b->path));
}

TEST(stastar_touches_goal_between_crossings) {
    Grid g = Grid::parse(kRoom);
    DistanceCache dist(g);
    ReservationTable table(g);
    // A sits at (6,2), steps onto (6,1) at t=6 and back at t=7.
    std::vector<Cell> a{{6, 2}, {6, 2}, {6, 2}, {6, 2}, {6, 2}, {6, 2}, {6, 1}, {6, 2}};
    table.reservePath(kA, a, 0);

    // B touches (6,1) at t=5 and must be gone before A arrives at t=6. It
    // can't swap with A into (6,2), so it backs out to (5,1).
    auto b = planSpaceTime(g, table, dist.to({6, 1}), {1, 1}, {6, 1}, 0, goalsAt(g, {{6, 1}}));
    CHECK(b.has_value() && b->reachesGoal);
    if (!b) return;
    CHECK_EQ(b->goalIndex, std::size_t{5});
    CHECK(b->path.back() == (Cell{5, 1}));
    CHECK_EQ(b->path.size(), std::size_t{7});
    CHECK(conflictFree(a, 0, b->path, 0));
}

TEST(stastar_robots_line_up_for_one_station) {
    Grid g = Grid::parse(kRoom);
    DistanceCache dist(g);
    ReservationTable table(g);
    const Cell station{6, 1};
    std::vector<int> goals = goalsAt(g, {station});
    ++goals[g.index(station)];  // both robots want it

    auto a = planSpaceTime(g, table, dist.to(station), {1, 2}, station, 0, goals);
    CHECK(a.has_value() && a->reachesGoal);
    if (!a) return;
    table.reservePath(kA, a->path, 0);
    auto b = planSpaceTime(g, table, dist.to(station), {1, 1}, station, 0, goals);
    CHECK(b.has_value() && b->reachesGoal);  // A only holds the station for one tick
    if (!b) return;
    CHECK(a->goalIndex != b->goalIndex);
    CHECK(conflictFree(a->path, 0, b->path, 0));
}

TEST(planner_names_round_trip) {
    for (Planner p : {Planner::Local, Planner::SpaceTime}) CHECK(parsePlanner(plannerName(p)) == p);
    CHECK(!parsePlanner("cbs").has_value());
}

TEST(stastar_simulation_stays_collision_free) {
    Simulation sim(Grid::parse(defaultMap()), {30, 7, Planner::SpaceTime});
    std::vector<RobotView> before = sim.robots();
    bool ok = true;
    for (int t = 0; t < 3000 && ok; ++t) {
        sim.step();
        std::vector<RobotView> after = sim.robots();
        std::string problem = sim.validate();
        if (!problem.empty()) {
            ::wwtest::fail(__FILE__, __LINE__, "tick " + std::to_string(t) + ": " + problem);
            ok = false;
        }
        for (std::size_t i = 0; i < after.size() && ok; ++i) {
            if (manhattan(before[i].cell, after[i].cell) > 1) ok = false;
            for (std::size_t j = i + 1; j < after.size(); ++j)
                if (after[i].cell == before[j].cell && after[j].cell == before[i].cell &&
                    !(before[i].cell == before[j].cell))
                    ok = false;  // swapped
        }
        before = std::move(after);
    }
    CHECK(ok);
    CHECK(sim.stats().deliveries > 0);
}

TEST(stastar_simulation_is_deterministic) {
    Simulation a(Grid::parse(defaultMap()), {20, 3, Planner::SpaceTime});
    Simulation b(Grid::parse(defaultMap()), {20, 3, Planner::SpaceTime});
    for (int t = 0; t < 1000; ++t) a.step(), b.step();
    CHECK_EQ(a.stats().deliveries, b.stats().deliveries);
    CHECK_EQ(a.stats().moves, b.stats().moves);
    CHECK_EQ(a.stats().waits, b.stats().waits);
}

TEST(stastar_keeps_stations_clear_in_a_crowd) {
    // Regression: with 28 robots, robots stepping off a corner station onto
    // its only two access cells sealed it, and by tick ~4600 every robot
    // was waiting (2491 deliveries total). The no-parking zone keeps it open.
    Simulation sim(Grid::parse(defaultMap()), {28, 1, Planner::SpaceTime});
    for (int t = 0; t < 6000; ++t) sim.step();
    CHECK(sim.stats().deliveries > 3500);
    CHECK(sim.validate().empty());
}
