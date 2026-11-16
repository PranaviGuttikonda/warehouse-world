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
