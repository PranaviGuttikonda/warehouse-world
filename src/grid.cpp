#include "ww/grid.hpp"

#include <stdexcept>

namespace ww {

namespace {

Tile toTile(char ch, int row, int col) {
    switch (ch) {
        case '.': return Tile::Floor;
        case '#': return Tile::Wall;
        case '=': return Tile::Shelf;
        case 'S': return Tile::Station;
        case '0': return Tile::Table;
        case '*': return Tile::Plant;
        default:
            throw std::invalid_argument("unknown map char '" + std::string(1, ch) +
                                        "' at row " + std::to_string(row) +
                                        ", col " + std::to_string(col));
    }
}

}  // namespace

Grid Grid::parse(std::string_view ascii) {
    std::vector<std::string_view> rows;
    std::size_t start = 0;
    while (start <= ascii.size()) {
        std::size_t end = ascii.find('\n', start);
        if (end == std::string_view::npos) end = ascii.size();
        std::string_view line = ascii.substr(start, end - start);
        if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
        if (!line.empty()) rows.push_back(line);
        start = end + 1;
    }
    if (rows.empty()) throw std::invalid_argument("map is empty");

    Grid g;
    g.height_ = static_cast<int>(rows.size());
    g.width_ = static_cast<int>(rows[0].size());
    g.tiles_.reserve(static_cast<std::size_t>(g.width_ * g.height_));

    for (int y = 0; y < g.height_; ++y) {
        if (static_cast<int>(rows[y].size()) != g.width_)
            throw std::invalid_argument("row " + std::to_string(y) + " has a different width");
        for (int x = 0; x < g.width_; ++x) g.tiles_.push_back(toTile(rows[y][x], y, x));
    }

    for (int i = 0; i < g.width_ * g.height_; ++i) {
        Cell c = g.cellAt(i);
        Tile t = g.tiles_[i];
        if (t == Tile::Station) g.stations_.push_back(c);
        if (t != Tile::Floor) continue;
        g.floor_.push_back(c);
        bool face = false, seat = false;
        for (Cell d : kNeighbors) {
            Cell n{c.x + d.x, c.y + d.y};
            if (!g.inBounds(n)) continue;
            face = face || g.at(n) == Tile::Shelf;
            seat = seat || g.at(n) == Tile::Table;
        }
        if (face) g.pickFaces_.push_back(c);
        if (seat) g.stations_.push_back(c);
    }
    if (g.stations_.empty()) throw std::invalid_argument("map needs a station 'S' or a table '0'");
    if (g.pickFaces_.empty()) throw std::invalid_argument("map needs at least one shelf '=' next to floor");
    return g;
}

std::string_view defaultMap() {
    return
        "##########################\n"
        "#S......................S#\n"
        "#........................#\n"
        "#..===..===..===..===....#\n"
        "#..===..===..===..===....#\n"
        "#........................#\n"
        "#..===..===..===..===....#\n"
        "#..===..===..===..===....#\n"
        "#........................#\n"
        "#S......................S#\n"
        "##########################\n";
}

std::string_view cafeMap() {
    return
        "##########################\n"
        "#*....==============....*#\n"
        "#........................#\n"
        "#........................#\n"
        "#...0.....0....0.....0...#\n"
        "#........................#\n"
        "#*.....0.....**.....0...*#\n"
        "#........................#\n"
        "#...0.....0....0.....0...#\n"
        "#........................#\n"
        "#*..........**..........*#\n"
        "##########################\n";
}

std::optional<std::string_view> builtinMap(std::string_view name) {
    if (name == "warehouse") return defaultMap();
    if (name == "cafe") return cafeMap();
    return std::nullopt;
}

}  // namespace ww
