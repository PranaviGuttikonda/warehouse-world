#include "test_framework.hpp"
#include "ww/grid.hpp"

using namespace ww;

TEST(grid_parses_dimensions_and_tiles) {
    Grid g = Grid::parse(
        "#####\n"
        "#S.=#\n"
        "#####\n");
    CHECK_EQ(g.width(), 5);
    CHECK_EQ(g.height(), 3);
    CHECK(g.at({1, 1}) == Tile::Station);
    CHECK(g.at({2, 1}) == Tile::Floor);
    CHECK(g.at({3, 1}) == Tile::Shelf);
    CHECK(g.at({0, 0}) == Tile::Wall);
}

TEST(grid_passable_rules) {
    Grid g = Grid::parse("#S.=#\n");
    CHECK(!g.passable({0, 0}));   // wall
    CHECK(g.passable({1, 0}));    // station
    CHECK(g.passable({2, 0}));    // floor
    CHECK(!g.passable({3, 0}));   // shelf
    CHECK(!g.passable({-1, 0}));  // out of bounds
    CHECK(!g.passable({0, 5}));
}

TEST(grid_finds_stations_and_pick_faces) {
    Grid g = Grid::parse(
        "#####\n"
        "#S.=#\n"
        "#..=#\n"
        "#####\n");
    CHECK_EQ(g.stations().size(), std::size_t{1});
    // floor cells touching a shelf: (2,1) and (2,2)
    CHECK_EQ(g.pickFaces().size(), std::size_t{2});
    CHECK(g.pickFaces()[0] == (Cell{2, 1}));
}

TEST(grid_handles_crlf_line_endings) {
    Grid g = Grid::parse("#S.=#\r\n#...#\r\n");
    CHECK_EQ(g.width(), 5);
    CHECK_EQ(g.height(), 2);
}

TEST(grid_rejects_bad_maps) {
    CHECK_THROWS(Grid::parse(""));
    CHECK_THROWS(Grid::parse("#S.=#\n#..#\n"));  // ragged rows
    CHECK_THROWS(Grid::parse("#S.?#\n"));        // unknown char
    CHECK_THROWS(Grid::parse("#..=#\n"));        // no station
    CHECK_THROWS(Grid::parse("#S..#\n"));        // no shelf
}

TEST(default_map_is_valid) {
    Grid g = Grid::parse(defaultMap());
    CHECK(g.stations().size() >= 1);
    CHECK(g.pickFaces().size() >= 1);
}

TEST(grid_tables_make_drop_off_cells_and_plants_block) {
    Grid g = Grid::parse(
        "######\n"
        "#.0..#\n"
        "#*..=#\n"
        "######\n");
    CHECK(g.at({2, 1}) == Tile::Table);
    CHECK(g.at({1, 2}) == Tile::Plant);
    CHECK(!g.passable({2, 1}));  // table
    CHECK(!g.passable({1, 2}));  // plant
    // floor next to the table: (1,1), (3,1), (2,2)
    CHECK_EQ(g.stations().size(), std::size_t{3});
    CHECK(g.stations()[0] == (Cell{1, 1}));
    CHECK(!g.passable({0, 0}));
}

TEST(cafe_map_has_one_counter_along_the_top_wall) {
    Grid g = Grid::parse(cafeMap());
    for (Cell c : g.pickFaces()) CHECK(c.y <= 2);  // every pickup is at the counter
    CHECK_EQ(g.pickFaces().size(), std::size_t{16});
    CHECK_EQ(g.stations().size(), std::size_t{40});  // 10 tables x 4 sides
    for (Cell s : g.stations())
        for (Cell p : g.pickFaces()) CHECK(!(s == p));
    CHECK(builtinMap("cafe") == cafeMap());
    CHECK(builtinMap("warehouse") == defaultMap());
    CHECK(!builtinMap("castle").has_value());
}
