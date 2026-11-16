#include "ww/simulation.hpp"

#include <algorithm>
#include <stdexcept>

namespace ww {

Simulation::Simulation(Grid grid, Config config)
    : grid_(std::move(grid)), rng_(config.seed), dist_(grid_) {
    occupancy_.assign(static_cast<std::size_t>(grid_.width() * grid_.height()), kNullEntity);

    std::vector<Cell> spawns = grid_.floorCells();
    if (config.robots < 0 || config.robots > static_cast<int>(spawns.size()))
        throw std::invalid_argument("robot count must be between 0 and " +
                                    std::to_string(spawns.size()));
    ww::shuffle(spawns.begin(), spawns.end(), rng_);

    for (int i = 0; i < config.robots; ++i) {
        Entity e = reg_.create();
        reg_.add(e, Position{spawns[i]});
        reg_.add(e, Robot{i});
        reg_.add(e, makeTask(grid_, rng_));
        occupancy_[grid_.index(spawns[i])] = e;
    }
}

void Simulation::step() {
    movementSystem(reg_, grid_, dist_, occupancy_, rng_, stats_);
    taskSystem(reg_, grid_, rng_, stats_);
    ++stats_.ticks;
}

std::vector<RobotView> Simulation::robots() const {
    const Registry& reg = reg_;
    std::vector<RobotView> out;
    for (Entity e : reg.pool<Robot>().entities()) {
        const Task& t = reg.get<Task>(e);
        out.push_back({reg.get<Robot>(e).id, reg.get<Position>(e).cell, t.goal(), t.carrying()});
    }
    std::sort(out.begin(), out.end(), [](const RobotView& a, const RobotView& b) { return a.id < b.id; });
    return out;
}

std::string Simulation::validate() const {
    std::vector<int> seen(occupancy_.size(), -1);
    for (const RobotView& r : robots()) {
        if (!grid_.passable(r.cell))
            return "robot " + std::to_string(r.id) + " is on a blocked tile";
        int i = grid_.index(r.cell);
        if (seen[i] != -1)
            return "robots " + std::to_string(seen[i]) + " and " + std::to_string(r.id) +
                   " share cell (" + std::to_string(r.cell.x) + "," + std::to_string(r.cell.y) + ")";
        seen[i] = r.id;
    }
    return {};
}

}  // namespace ww
