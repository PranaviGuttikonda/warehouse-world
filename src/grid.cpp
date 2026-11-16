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
        for (Cell d : kNeighbors) {
            Cell n{c.x + d.x, c.y + d.y};
            if (g.inBounds(n) && g.at(n) == Tile::Shelf) {
                g.pickFaces_.push_back(c);
                break;
            }
        }
    }
    if (g.stations_.empty()) throw std::invalid_argument("map needs at least one station 'S'");
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

}  // namespace ww
