#include "test_framework.hpp"
#include "ww/ecs.hpp"

using namespace ww;

namespace {
struct Health { int hp; };
struct Tag {};
}  // namespace

TEST(ecs_add_get_has) {
    Registry reg;
    Entity a = reg.create();
    Entity b = reg.create();
    reg.add(a, Health{10});
    CHECK(reg.has<Health>(a));
    CHECK(!reg.has<Health>(b));
    CHECK_EQ(reg.get<Health>(a).hp, 10);
    reg.get<Health>(a).hp = 7;
    CHECK_EQ(reg.get<Health>(a).hp, 7);
}

TEST(ecs_remove_keeps_other_components_correct) {
    // Removing from the middle swaps the last element into the hole;
    // the moved entity must still map to its own data.
    Registry reg;
    Entity e0 = reg.create(), e1 = reg.create(), e2 = reg.create();
    reg.add(e0, Health{0});
    reg.add(e1, Health{1});
    reg.add(e2, Health{2});
    reg.remove<Health>(e0);
    CHECK(!reg.has<Health>(e0));
    CHECK_EQ(reg.get<Health>(e1).hp, 1);
    CHECK_EQ(reg.get<Health>(e2).hp, 2);
    CHECK_EQ(reg.pool<Health>().size(), std::size_t{2});
}

TEST(ecs_pool_storage_is_dense) {
    Registry reg;
    for (int i = 0; i < 5; ++i) reg.add(reg.create(), Health{i});
    reg.remove<Health>(2);
    auto& pool = reg.pool<Health>();
    CHECK_EQ(pool.data().size(), pool.entities().size());
    for (std::size_t i = 0; i < pool.size(); ++i)
        CHECK_EQ(pool.data()[i].hp, static_cast<int>(pool.entities()[i]));
}

TEST(ecs_destroy_clears_all_pools_and_recycles_id) {
    Registry reg;
    Entity e = reg.create();
    reg.add(e, Health{5});
    reg.add(e, Tag{});
    reg.destroy(e);
    CHECK(!reg.alive(e));
    CHECK(!reg.has<Health>(e));
    CHECK(!reg.has<Tag>(e));
    Entity reused = reg.create();
    CHECK_EQ(reused, e);
    CHECK(!reg.has<Health>(reused));
}

TEST(ecs_const_pool_of_unused_type_is_empty) {
    const Registry reg;
    CHECK_EQ(reg.pool<Health>().size(), std::size_t{0});
}
