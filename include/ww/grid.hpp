#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "ww/types.hpp"

namespace ww {

enum class Tile : char {
    Floor = '.',
    Wall = '#',
    Shelf = '=',
    Station = 'S',
};

/// Static warehouse layout. Robots are not stored here; see Registry.
class Grid {
public:
    /// Given: ASCII rows using '.', '#', '=', 'S'.
    /// Approach: validate equal row widths, map each char to a Tile.
    /// Throws std::invalid_argument on bad input.
    static Grid parse(std::string_view ascii);

    int width() const { return width_; }
    int height() const { return height_; }

    bool inBounds(Cell c) const {
        return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_;
    }
    Tile at(Cell c) const { return tiles_[index(c)]; }

    /// Robots may stand on floor and station tiles only.
    bool passable(Cell c) const {
        if (!inBounds(c)) return false;
        Tile t = at(c);
        return t == Tile::Floor || t == Tile::Station;
    }

    int index(Cell c) const { return c.y * width_ + c.x; }
    Cell cellAt(int idx) const { return {idx % width_, idx / width_}; }

    const std::vector<Cell>& stations() const { return stations_; }
    /// Floor cells next to a shelf: where a robot picks up an item.
    const std::vector<Cell>& pickFaces() const { return pickFaces_; }
    const std::vector<Cell>& floorCells() const { return floor_; }

private:
    int width_ = 0;
    int height_ = 0;
    std::vector<Tile> tiles_;
    std::vector<Cell> stations_;
    std::vector<Cell> pickFaces_;
    std::vector<Cell> floor_;
};

/// Built-in demo layout.
std::string_view defaultMap();

}  // namespace ww
