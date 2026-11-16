#include "test_framework.hpp"
#include "ww/simulation.hpp"
#include "ww/systems.hpp"

using namespace ww;

namespace {

// Single corridor: robots can only pass by backing out of the way.
constexpr const char* kCorridor =
    "########\n"
    "#S....S#\n"
    "#======#\n"
    "########\n";

Entity spawn(Registry& reg, const Grid& g, std::vector<Entity>& occ, int id, Cell at, Cell goal) {
    Entity e = reg.create();
    reg.add(e, Position{at});
    reg.add(e, Robot{id});
    reg.add(e, Task{goal, goal, Task::Stage::ToPickup});
    occ[g.index(at)] = e;
    return e;
}

}  // namespace

TEST(distance_field_is_shortest_path_around_obstacles) {
    Grid g = Grid::parse(
        "#####\n"
        "#S.=#\n"
        "#.#.#\n"
        "#...#\n"
        "#####\n");
    DistanceCache dist(g);
    const auto& f = dist.to({1, 1});
    CHECK_EQ(f[g.index({1, 1})], 0);
    CHECK_EQ(f[g.index({3, 3})], 4);  // has to walk around the wall
    CHECK_EQ(f[g.index({3, 1})], DistanceCache::kUnreachable);  // shelf
    CHECK(&dist.to({1, 1}) == &f);  // cached
}

TEST(robots_never_swap_head_on) {
    Grid g = Grid::parse(kCorridor);
    Registry reg;
    std::vector<Entity> occ(static_cast<std::size_t>(g.width() * g.height()), kNullEntity);
    Entity a = spawn(reg, g, occ, 0, {3, 1}, {5, 1});
    Entity b = spawn(reg, g, occ, 1, {4, 1}, {2, 1});
    DistanceCache dist(g);
    Rng rng(1);
    Stats stats;

    for (int t = 0; t < 50; ++t) {
        Cell pa = reg.get<Position>(a).cell, pb = reg.get<Position>(b).cell;
        movementSystem(reg, g, dist, occ, rng, stats);
        ++stats.ticks;
        Cell na = reg.get<Position>(a).cell, nb = reg.get<Position>(b).cell;
        CHECK(!(na == nb));
        CHECK(!(na == pb && nb == pa));
        CHECK(manhattan(pa, na) <= 1);
        CHECK(manhattan(pb, nb) <= 1);
    }
    CHECK(stats.waits > 0);  // the head-on standoff happened
}

TEST(occupancy_stays_in_sync_with_positions) {
    Grid g = Grid::parse(defaultMap());
    Registry reg;
    std::vector<Entity> occ(static_cast<std::size_t>(g.width() * g.height()), kNullEntity);
    Rng rng(7);
    std::vector<Entity> robots;
    for (int i = 0; i < 10; ++i) {
        Cell at = g.floorCells()[static_cast<std::size_t>(i * 7)];
        robots.push_back(spawn(reg, g, occ, i, at, g.stations()[static_cast<std::size_t>(i) % g.stations().size()]));
    }
    DistanceCache dist(g);
    Stats stats;
    for (int t = 0; t < 200; ++t) {
        movementSystem(reg, g, dist, occ, rng, stats);
        ++stats.ticks;
    }
    int occupied = 0;
    for (Entity e : occ) occupied += e != kNullEntity;
    CHECK_EQ(occupied, 10);
    for (Entity e : robots) CHECK_EQ(occ[g.index(reg.get<Position>(e).cell)], e);
}

TEST(simulation_stays_valid_under_load) {
    Simulation sim(Grid::parse(defaultMap()), {40, 123});
    for (int t = 0; t < 5000; ++t) {
        sim.step();
        std::string problem = sim.validate();
        if (!problem.empty()) {
            CHECK(problem.empty());
            break;
        }
    }
    CHECK(sim.stats().deliveries > 0);
}

TEST(simulation_completes_deliveries) {
    Simulation sim(Grid::parse(defaultMap()), {8, 5});
    for (int t = 0; t < 2000; ++t) sim.step();
    CHECK(sim.stats().deliveries >= 50);
}

TEST(simulation_is_deterministic_for_a_seed) {
    Simulation a(Grid::parse(defaultMap()), {15, 99});
    Simulation b(Grid::parse(defaultMap()), {15, 99});
    for (int t = 0; t < 1000; ++t) {
        a.step();
        b.step();
    }
    CHECK_EQ(a.stats().deliveries, b.stats().deliveries);
    CHECK_EQ(a.stats().moves, b.stats().moves);
    auto ra = a.robots(), rb = b.robots();
    for (std::size_t i = 0; i < ra.size(); ++i) CHECK(ra[i].cell == rb[i].cell);
}

TEST(simulation_rejects_too_many_robots) {
    Grid g = Grid::parse(kCorridor);
    CHECK_THROWS(Simulation(Grid::parse(kCorridor), {static_cast<int>(g.floorCells().size()) + 1, 1}));
}
