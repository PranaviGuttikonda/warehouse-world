#include "ww/simulation.hpp"

#include <algorithm>
#include <stdexcept>

#include "ww/cbs.hpp"
#include "ww/stastar.hpp"

namespace ww {

const char* plannerName(Planner p) {
    switch (p) {
        case Planner::Local: return "local";
        case Planner::SpaceTime: return "stastar";
        case Planner::Cbs: return "cbs";
    }
    return "?";
}

std::optional<Planner> parsePlanner(std::string_view name) {
    for (Planner p : {Planner::Local, Planner::SpaceTime, Planner::Cbs})
        if (name == plannerName(p)) return p;
    return std::nullopt;
}

Simulation::Simulation(Grid grid, Config config)
    : grid_(std::move(grid)),
      rng_(config.seed),
      dist_(grid_),
      planner_(config.planner),
      reservations_(grid_) {
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
        if (planner_ == Planner::SpaceTime) {
            // Park on the spawn cell; the first step replans (goal unset).
            Plan& plan = reg_.add(e, Plan{{spawns[i]}, 0});
            reservations_.reservePath(e, plan.path, plan.start);
        } else if (planner_ == Planner::Cbs) {
            reg_.add(e, Plan{{spawns[i]}, 0});  // goal unset: the first step plans
        }
    }
}

void Simulation::step() {
    switch (planner_) {
        case Planner::Local: movementSystem(reg_, grid_, dist_, occupancy_, rng_, stats_); break;
        case Planner::SpaceTime: spaceTimeSystem(reg_, grid_, dist_, reservations_, occupancy_, stats_); break;
        case Planner::Cbs: cbsSystem(reg_, grid_, dist_, occupancy_, stats_); break;
    }
    taskSystem(reg_, grid_, rng_, stats_);
    ++stats_.ticks;
}

std::vector<RobotView> Simulation::robots() const {
    const Registry& reg = reg_;
    std::vector<RobotView> out;
    for (Entity e : reg.pool<Robot>().entities()) {
        const Task& t = reg.get<Task>(e);
        const Robot& r = reg.get<Robot>(e);
        std::vector<Cell> route;
        if (reg.pool<Plan>().has(e)) {
            const Plan& plan = reg.get<Plan>(e);
            const auto k = static_cast<std::size_t>(std::max(0LL, stats_.ticks - plan.start));
            if (k < plan.path.size()) route.assign(plan.path.begin() + static_cast<std::ptrdiff_t>(k), plan.path.end());
        }
        out.push_back({r.id, reg.get<Position>(e).cell, t.goal(), t.dropoff, t.carrying(), r.waitTicks, std::move(route)});
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
