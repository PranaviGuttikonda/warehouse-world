#pragma once

#include <string>

#include "ww/simulation.hpp"

namespace ww {

/// Given: a simulation. Approach: draw the grid as text, overlay robots
/// (lowercase = heading to pickup, uppercase = carrying), add a stats line.
/// color adds ANSI colours for terminals that support them.
std::string renderFrame(const Simulation& sim, bool color);

}  // namespace ww
