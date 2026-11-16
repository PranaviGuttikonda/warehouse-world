#include "ww/reservation.hpp"

#include <cassert>

namespace ww {

ReservationTable::ReservationTable(const Grid& grid)
    : grid_(grid),
      path_(static_cast<std::size_t>(grid.width() * grid.height())),
      park_(static_cast<std::size_t>(grid.width() * grid.height())) {}

void ReservationTable::reservePath(Entity e, const std::vector<Cell>& path, long long start) {
    assert(!path.empty());
    for (std::size_t k = 0; k + 1 < path.size(); ++k) {
        auto [it, inserted] = path_[grid_.index(path[k])].emplace(start + static_cast<long long>(k), e);
        assert((inserted || it->second == e) && "vertex already reserved");
        (void)it, (void)inserted;
    }
    Park& p = park_[grid_.index(path.back())];
    assert((p.robot == kNullEntity || p.robot == e) && "cell already parked");
    p = {start + static_cast<long long>(path.size()) - 1, e};
}

void ReservationTable::releasePath(Entity e, const std::vector<Cell>& path, long long start) {
    for (std::size_t k = 0; k + 1 < path.size(); ++k) {
        auto& cell = path_[grid_.index(path[k])];
        auto it = cell.find(start + static_cast<long long>(k));
        if (it != cell.end() && it->second == e) cell.erase(it);
    }
    if (path.empty()) return;
    Park& p = park_[grid_.index(path.back())];
    if (p.robot == e) p = {};
}

Entity ReservationTable::holder(Cell c, long long t) const {
    const int i = grid_.index(c);
    const auto& cell = path_[i];
    if (auto it = cell.find(t); it != cell.end()) return it->second;
    const Park& p = park_[i];
    return p.robot != kNullEntity && p.from <= t ? p.robot : kNullEntity;
}

bool ReservationTable::canMove(Cell from, Cell to, long long t) const {
    if (!isFree(to, t + 1)) return false;
    if (from == to) return true;
    const Entity other = holder(to, t);  // swap: other goes to -> from meanwhile
    return other == kNullEntity || holder(from, t + 1) != other;
}

bool ReservationTable::canParkFrom(Cell c, long long t) const {
    const int i = grid_.index(c);
    if (park_[i].robot != kNullEntity) return false;
    return path_[i].lower_bound(t) == path_[i].end();
}

}  // namespace ww
