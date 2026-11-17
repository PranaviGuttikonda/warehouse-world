// warehouse: run the warehouse robot world in the terminal.
//
//   ./warehouse                         live view, 12 robots
//   ./warehouse --robots 20 --hz 15     more robots, faster ticks
//   ./warehouse --map maps/small.txt    custom layout (or --map cafe)
//   ./warehouse --headless --ticks 100000   benchmark, no drawing
//   ./warehouse --planner stastar       space-time A* instead of local moves
//   ./warehouse --planner cbs           conflict-based search, robots jointly

#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "ww/grid.hpp"
#include "ww/render.hpp"
#include "ww/simulation.hpp"
#include "ww/timestep.hpp"

namespace {

volatile std::sig_atomic_t gStop = 0;
void onSigint(int) { gStop = 1; }

struct Options {
    int robots = 12;
    long long ticks = 0;  // 0 = run until Ctrl-C (live mode)
    double hz = 8.0;
    std::uint64_t seed = 42;
    ww::Planner planner = ww::Planner::Local;
    bool headless = false;
    bool color = true;
    std::string mapPath;
};

[[noreturn]] void usage(int code) {
    std::cout << "usage: warehouse [--robots N] [--ticks N] [--hz R] [--seed S]\n"
                 "                 [--map FILE|cafe|warehouse] [--planner local|stastar|cbs] [--headless] [--no-color]\n";
    std::exit(code);
}

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage(1);
            return argv[++i];
        };
        try {
            if (a == "--robots") o.robots = std::stoi(next());
            else if (a == "--ticks") o.ticks = std::stoll(next());
            else if (a == "--hz") o.hz = std::stod(next());
            else if (a == "--seed") o.seed = std::stoull(next());
            else if (a == "--map") o.mapPath = next();
            else if (a == "--planner") {
                auto p = ww::parsePlanner(next());
                if (!p) usage(1);
                o.planner = *p;
            }
            else if (a == "--headless") o.headless = true;
            else if (a == "--no-color") o.color = false;
            else if (a == "-h" || a == "--help") usage(0);
            else usage(1);
        } catch (const std::logic_error&) {
            std::cerr << "bad value for " << a << "\n";
            usage(1);
        }
    }
    if (o.hz <= 0) o.hz = 8.0;
    return o;
}

std::string loadMap(const std::string& path) {
    if (path.empty()) return std::string(ww::defaultMap());
    if (auto builtin = ww::builtinMap(path)) return std::string(*builtin);
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open map file: " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

int runHeadless(ww::Simulation& sim, long long ticks) {
    if (ticks <= 0) ticks = 100000;
    auto t0 = std::chrono::steady_clock::now();
    for (long long i = 0; i < ticks; ++i) sim.step();
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    std::string problem = sim.validate();
    const ww::Stats& s = sim.stats();
    std::cout << "planner      " << ww::plannerName(sim.planner()) << "\n"
              << "ticks        " << s.ticks << "\n"
              << "robots       " << sim.robots().size() << "\n"
              << "deliveries   " << s.deliveries << "\n"
              << "moves        " << s.moves << "\n"
              << "waits        " << s.waits << "\n";
    if (sim.planner() == ww::Planner::Cbs)
        std::cout << "cbs solves   " << s.cbsSolves << " (" << s.cbsFallbacks << " fell back)\n";
    std::cout << "elapsed      " << secs << " s\n"
              << "ticks/sec    " << static_cast<long long>(s.ticks / std::max(secs, 1e-9)) << "\n"
              << "state valid  " << (problem.empty() ? "yes" : problem) << "\n";
    return problem.empty() ? 0 : 2;
}

// Given: text with '\n' line ends. Approach: insert \033[K (clear to end of
// line) before each newline so shorter lines erase the previous frame's.
std::string clearLineEnds(const std::string& text) {
    std::string out;
    out.reserve(text.size() + text.size() / 16);
    for (char ch : text) {
        if (ch == '\n') out += "\033[K";
        out += ch;
    }
    return out;
}

int runLive(ww::Simulation& sim, const Options& o) {
    using clock = std::chrono::steady_clock;
    ww::FixedTimestep timestep(1.0 / o.hz);
    const auto frame = std::chrono::milliseconds(16);  // ~60 fps redraw cap
    auto last = clock::now();

    std::signal(SIGINT, onSigint);  // restore the cursor on Ctrl-C
    std::cout << "\033[2J\033[?25l";  // clear screen, hide cursor
    while (!gStop && (o.ticks == 0 || sim.stats().ticks < o.ticks)) {
        auto now = clock::now();
        double elapsed = std::chrono::duration<double>(now - last).count();
        last = now;
        if (timestep.advance(elapsed, [&] { sim.step(); }) > 0) {
            // Home, draw, then \033[J clears anything left below the frame.
            std::cout << "\033[H"
                      << clearLineEnds(ww::renderFrame(sim, o.color) +
                                       "lowercase = to shelf, UPPERCASE = carrying.  Ctrl-C to quit.\n")
                      << "\033[J" << std::flush;
        }
        std::this_thread::sleep_for(frame);
    }
    std::cout << "\033[?25h\n";  // show cursor
    const ww::Stats& s = sim.stats();
    std::cout << s.deliveries << " deliveries in " << s.ticks << " ticks\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Options o = parseArgs(argc, argv);
    try {
        ww::Grid grid = ww::Grid::parse(loadMap(o.mapPath));
        ww::Simulation sim(std::move(grid), {o.robots, o.seed, o.planner});
        return o.headless ? runHeadless(sim, o.ticks) : runLive(sim, o);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
