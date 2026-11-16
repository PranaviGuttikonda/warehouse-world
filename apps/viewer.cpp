// viewer: graphical view of the warehouse robot world (raylib).
//
//   ./viewer                              default map, 16 robots
//   ./viewer --robots 24 --hz 15 --seed 3
//   ./viewer --record docs/viewer.gif     write a short GIF and exit
//
// Keys: space pause, +/- speed, up/down robot count.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "raylib.h"
#include "ww/grid.hpp"
#include "ww/simulation.hpp"
#include "ww/timestep.hpp"

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#else
#include "external/msf_gif.h"  // ships with raylib, compiled into libraylib
#endif

namespace {

using ww::Cell;

constexpr int kCell = 32;
constexpr int kMargin = 16;
constexpr int kHudH = 36;
constexpr int kFooterH = 28;
constexpr double kHzSteps[] = {2, 4, 8, 15, 30, 60, 120};
constexpr double kRateWindow = 10.0;  // seconds of running time for deliveries/min

const Color kBg{17, 19, 24, 255};
const Color kWall{27, 30, 37, 255};
const Color kFloorDot{34, 38, 47, 255};
const Color kShelf{56, 61, 74, 255};
const Color kStation{46, 190, 176, 255};
const Color kToShelf{150, 118, 255, 255};
const Color kCarrying{255, 127, 107, 255};
const Color kText{205, 209, 219, 255};
const Color kDimText{110, 116, 130, 255};

struct Options {
    int robots = 16;
    std::uint64_t seed = 42;
    int hzIndex = 2;  // 8 Hz, same as the terminal app
    std::string mapPath;
    std::string recordPath;
    int recordFrames = 240;
};

struct App {
    Options opt;
    std::string mapText;
    std::unique_ptr<ww::Simulation> sim;
    ww::FixedTimestep timestep{1.0 / kHzSteps[2]};
    std::vector<Cell> prev, cur;  // by robot id: before and after the last tick
    std::vector<Rectangle> shelves;
    bool paused = false;
    double runTime = 0.0;  // seconds spent unpaused
    std::deque<std::pair<double, long long>> rateSamples;
    int width = 0, height = 0;
};

double hz(const App& a) { return kHzSteps[a.opt.hzIndex]; }

// Given: the grid. Approach: flood-fill each shelf block and keep its
// bounding box, so a block draws as one rounded rect instead of beads.
std::vector<Rectangle> shelfBlocks(const ww::Grid& g) {
    std::vector<Rectangle> out;
    std::vector<bool> seen(static_cast<std::size_t>(g.width() * g.height()), false);
    for (int i = 0; i < g.width() * g.height(); ++i) {
        Cell start = g.cellAt(i);
        if (seen[i] || g.at(start) != ww::Tile::Shelf) continue;
        int x0 = start.x, y0 = start.y, x1 = start.x, y1 = start.y;
        std::vector<Cell> stack{start};
        seen[i] = true;
        while (!stack.empty()) {
            Cell c = stack.back();
            stack.pop_back();
            x0 = std::min(x0, c.x), y0 = std::min(y0, c.y);
            x1 = std::max(x1, c.x), y1 = std::max(y1, c.y);
            for (Cell d : ww::kNeighbors) {
                Cell n{c.x + d.x, c.y + d.y};
                if (!g.inBounds(n) || seen[g.index(n)] || g.at(n) != ww::Tile::Shelf) continue;
                seen[g.index(n)] = true;
                stack.push_back(n);
            }
        }
        const float inset = 4.0f;
        out.push_back({kMargin + x0 * kCell + inset, kHudH + y0 * kCell + inset,
                       (x1 - x0 + 1) * kCell - 2 * inset, (y1 - y0 + 1) * kCell - 2 * inset});
    }
    return out;
}

void snapshot(const ww::Simulation& sim, std::vector<Cell>& into) {
    std::vector<ww::RobotView> robots = sim.robots();
    into.resize(robots.size());
    for (const auto& r : robots) into[static_cast<std::size_t>(r.id)] = r.cell;
}

// Given: options. Approach: rebuild the world from scratch; keeps the seed so
// the same count always gives the same run.
void restart(App& a) {
    a.sim = std::make_unique<ww::Simulation>(ww::Grid::parse(a.mapText),
                                             ww::Simulation::Config{a.opt.robots, a.opt.seed});
    a.timestep = ww::FixedTimestep(1.0 / hz(a));
    snapshot(*a.sim, a.cur);
    a.prev = a.cur;
    a.rateSamples.clear();
}

void handleInput(App& a) {
    auto pressed = [](int key) { return IsKeyPressed(key) || IsKeyPressedRepeat(key); };
    if (IsKeyPressed(KEY_SPACE)) a.paused = !a.paused;

    int hzIndex = a.opt.hzIndex;
    if (pressed(KEY_EQUAL) || pressed(KEY_KP_ADD)) ++hzIndex;
    if (pressed(KEY_MINUS) || pressed(KEY_KP_SUBTRACT)) --hzIndex;
    hzIndex = std::clamp(hzIndex, 0, static_cast<int>(std::size(kHzSteps)) - 1);
    if (hzIndex != a.opt.hzIndex) {
        a.opt.hzIndex = hzIndex;
        a.timestep = ww::FixedTimestep(1.0 / hz(a));
        a.rateSamples.clear();
    }

    int robots = a.opt.robots;
    if (pressed(KEY_UP)) ++robots;
    if (pressed(KEY_DOWN)) --robots;
    robots = std::clamp(robots, 1, static_cast<int>(a.sim->grid().floorCells().size()));
    if (robots != a.opt.robots) {
        a.opt.robots = robots;
        restart(a);
    }
}

// Given: frame time. Approach: run fixed ticks; before each, keep the old
// positions so drawing can lerp prev -> cur by FixedTimestep::alpha().
void update(App& a, double frameSeconds) {
    if (a.paused) return;
    a.runTime += frameSeconds;
    a.timestep.advance(frameSeconds, [&] {
        a.prev = a.cur;
        a.sim->step();
        snapshot(*a.sim, a.cur);
    });
    a.rateSamples.emplace_back(a.runTime, a.sim->stats().deliveries);
    while (a.rateSamples.size() > 2 && a.runTime - a.rateSamples.front().first > kRateWindow)
        a.rateSamples.pop_front();
}

double deliveriesPerMinute(const App& a) {
    if (a.rateSamples.size() < 2) return 0.0;
    auto [t0, d0] = a.rateSamples.front();
    auto [t1, d1] = a.rateSamples.back();
    return t1 - t0 < 1.0 ? 0.0 : static_cast<double>(d1 - d0) / (t1 - t0) * 60.0;
}

// Given: current state. Approach: draw the scene in window coordinates; the
// caller decides the target (screen, or an offscreen texture for recording).
void drawScene(const App& a) {
    const ww::Grid& g = a.sim->grid();
    ClearBackground(kBg);

    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            const int px = kMargin + x * kCell, py = kHudH + y * kCell;
            switch (g.at({x, y})) {
                case ww::Tile::Wall: DrawRectangle(px, py, kCell, kCell, kWall); break;
                case ww::Tile::Floor: DrawCircle(px + kCell / 2, py + kCell / 2, 1.5f, kFloorDot); break;
                case ww::Tile::Station: {
                    const int s = kCell * 9 / 20;
                    DrawRectangle(px + (kCell - s) / 2, py + (kCell - s) / 2, s, s, kStation);
                    break;
                }
                case ww::Tile::Shelf: break;  // drawn as blocks below
            }
        }
    }
    for (const Rectangle& r : a.shelves) DrawRectangleRounded(r, 0.3f, 6, kShelf);

    const float alpha = a.paused ? 1.0f : static_cast<float>(a.timestep.alpha());
    int waiting = 0;
    for (const ww::RobotView& r : a.sim->robots()) {
        const auto id = static_cast<std::size_t>(r.id);
        const Cell from = id < a.prev.size() ? a.prev[id] : r.cell;
        const Cell to = id < a.cur.size() ? a.cur[id] : r.cell;
        const float x = from.x + (to.x - from.x) * alpha, y = from.y + (to.y - from.y) * alpha;
        DrawCircleV({kMargin + (x + 0.5f) * kCell, kHudH + (y + 0.5f) * kCell}, kCell * 0.34f,
                    r.carrying ? kCarrying : kToShelf);
        waiting += r.waitTicks > 0;
    }

    char hud[160];
    std::snprintf(hud, sizeof hud, "tick %lld    %.0f deliveries/min    %d waiting    %d robots    %g Hz%s",
                  a.sim->stats().ticks, deliveriesPerMinute(a), waiting, a.opt.robots, hz(a),
                  a.paused ? "    paused" : "");
    DrawText(hud, kMargin, 10, 18, kText);
    DrawText("space pause    +/- speed    up/down robots", kMargin, a.height - kFooterH + 6, 14, kDimText);
}

void draw(const App& a) {
    BeginDrawing();
    drawScene(a);
    EndDrawing();
}

void frame(void* arg) {
    App& a = *static_cast<App*>(arg);
    handleInput(a);
    update(a, GetFrameTime());
    draw(a);
}

#if !defined(__EMSCRIPTEN__)
// Given: an open window. Approach: warm up so aisles are busy, then render
// fixed 1/30 s frames into a 2x offscreen texture, downscale (cheap
// antialiasing) and feed each frame to raylib's bundled GIF encoder.
int record(App& a) {
    for (int i = 0; i < 120; ++i) {
        a.prev = a.cur;
        a.sim->step();
        snapshot(*a.sim, a.cur);
    }
    constexpr int kSuper = 2;
    RenderTexture2D target = LoadRenderTexture(a.width * kSuper, a.height * kSuper);
    Camera2D camera{};
    camera.zoom = kSuper;
    MsfGifState gif{};
    bool ok = msf_gif_begin(&gif, a.width, a.height);
    for (int f = 0; ok && f < a.opt.recordFrames && !WindowShouldClose(); ++f) {
        update(a, 1.0 / 30.0);
        BeginTextureMode(target);
        BeginMode2D(camera);
        drawScene(a);
        EndMode2D();
        EndTextureMode();

        BeginDrawing();  // mirror to the window so progress is visible
        ClearBackground(kBg);
        DrawTexturePro(target.texture, {0, 0, float(target.texture.width), -float(target.texture.height)},
                       {0, 0, float(a.width), float(a.height)}, {0, 0}, 0, WHITE);
        EndDrawing();

        Image img = LoadImageFromTexture(target.texture);
        ImageFlipVertical(&img);  // GL textures are bottom-up
        ImageResize(&img, a.width, a.height);
        ImageFormat(&img, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        ok = msf_gif_frame(&gif, static_cast<std::uint8_t*>(img.data), 3, 16, img.width * 4);
        UnloadImage(img);
    }
    UnloadRenderTexture(target);
    MsfGifResult result = msf_gif_end(&gif);
    ok = ok && result.data && SaveFileData(a.opt.recordPath.c_str(), result.data,
                                           static_cast<int>(result.dataSize));
    msf_gif_free(result);
    std::cout << (ok ? "wrote " : "failed to write ") << a.opt.recordPath << "\n";
    return ok ? 0 : 1;
}

std::string loadMap(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open map file: " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}
#endif

[[noreturn]] void usage(int code) {
    std::cout << "usage: viewer [--robots N] [--seed S] [--hz 2|4|8|15|30|60|120] [--map FILE]\n"
                 "              [--record OUT.gif [--frames N]]\n";
    std::exit(code);
}

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage(1);
            return argv[++i];
        };
        try {
            if (arg == "--robots") o.robots = std::stoi(next());
            else if (arg == "--seed") o.seed = std::stoull(next());
            else if (arg == "--hz") {
                const double want = std::stod(next());
                auto it = std::find(std::begin(kHzSteps), std::end(kHzSteps), want);
                if (it == std::end(kHzSteps)) usage(1);
                o.hzIndex = static_cast<int>(it - std::begin(kHzSteps));
            } else if (arg == "--map") o.mapPath = next();
            else if (arg == "--record") o.recordPath = next();
            else if (arg == "--frames") o.recordFrames = std::stoi(next());
            else if (arg == "-h" || arg == "--help") usage(0);
            else usage(1);
        } catch (const std::logic_error&) {
            usage(1);
        }
    }
    return o;
}

}  // namespace

int main(int argc, char** argv) {
    App a;
    a.opt = parseArgs(argc, argv);
    try {
#if defined(__EMSCRIPTEN__)
        a.mapText = std::string(ww::defaultMap());
#else
        a.mapText = a.opt.mapPath.empty() ? std::string(ww::defaultMap()) : loadMap(a.opt.mapPath);
#endif
        restart(a);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    const ww::Grid& g = a.sim->grid();
    a.width = 2 * kMargin + g.width() * kCell;
    a.height = kHudH + g.height() * kCell + kFooterH;
    a.shelves = shelfBlocks(g);

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(a.width, a.height, "warehouse-world");

#if defined(__EMSCRIPTEN__)
    emscripten_set_main_loop_arg(frame, &a, 0, 1);
#else
    if (!a.opt.recordPath.empty()) {
        int rc = record(a);
        CloseWindow();
        return rc;
    }
    SetTargetFPS(60);
    while (!WindowShouldClose()) frame(&a);
    CloseWindow();
#endif
    return 0;
}
