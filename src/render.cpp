#include "ww/render.hpp"

#include <vector>

namespace ww {

namespace {

constexpr const char* kReset = "\033[0m";
constexpr const char* kDim = "\033[90m";
constexpr const char* kShelf = "\033[33m";
constexpr const char* kStation = "\033[1;32m";
constexpr const char* kEmpty = "\033[36m";
constexpr const char* kLoaded = "\033[1;35m";

char robotGlyph(int id, bool carrying) {
    if (id >= 26) return carrying ? '@' : '&';
    return static_cast<char>((carrying ? 'A' : 'a') + id);
}

}  // namespace

std::string renderFrame(const Simulation& sim, bool color) {
    const Grid& g = sim.grid();
    std::vector<int> robotAt(static_cast<std::size_t>(g.width() * g.height()), -1);
    std::vector<RobotView> robots = sim.robots();
    for (std::size_t i = 0; i < robots.size(); ++i) robotAt[g.index(robots[i].cell)] = static_cast<int>(i);

    std::string out;
    out.reserve(static_cast<std::size_t>(g.width() * g.height() * 8));
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            Cell c{x, y};
            int r = robotAt[g.index(c)];
            if (r >= 0) {
                const RobotView& rv = robots[r];
                if (color) out += rv.carrying ? kLoaded : kEmpty;
                out += robotGlyph(rv.id, rv.carrying);
                if (color) out += kReset;
                continue;
            }
            Tile t = g.at(c);
            if (color) {
                switch (t) {
                    case Tile::Shelf: out += kShelf; break;
                    case Tile::Station: out += kStation; break;
                    default: out += kDim; break;
                }
            }
            out += static_cast<char>(t);
            if (color) out += kReset;
        }
        out += '\n';
    }

    const Stats& s = sim.stats();
    out += "tick " + std::to_string(s.ticks) + "  robots " + std::to_string(robots.size()) +
           "  deliveries " + std::to_string(s.deliveries) + "  moves " + std::to_string(s.moves) +
           "  waits " + std::to_string(s.waits) + "\n";
    return out;
}

}  // namespace ww
