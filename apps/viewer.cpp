// viewer: Rush Hour Café, the graphical view of the robot world (raylib).
//
//   ./viewer                              café map, 16 robots
//   ./viewer --map warehouse              the benchmark warehouse map
//   ./viewer --robots 24 --hz 15 --seed 3 --planner stastar
//   ./viewer --record docs/viewer.gif     write a short GIF and exit
//
// Keys: space pause, +/- speed, up/down robot count, P cycle planner,
// T show all paths. Every control is also a button in the bottom bar
// (mouse or touch); hover or click a robot to see its planned path.

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
#include "ui_font.h"  // generated: kUiFontData, IBM Plex Sans (SIL OFL 1.1)
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
constexpr int kHudH = 32;
constexpr int kFooterH = 40;
constexpr int kMinWidth = 720;  // room for the control bar on small maps
constexpr double kHzSteps[] = {2, 4, 8, 15, 30, 60, 120};
constexpr double kRateWindow = 10.0;  // seconds of running time for orders/min
constexpr double kPopSeconds = 0.8;   // delivery fade
constexpr int kStuckTicks = 3;        // a robot counts as stuck after this many blocked ticks

// Minimal dark palette (docs/VISION.md). Amber is the only accent, and it
// only ever means food: an order waiting, or a robot carrying one.
const Color kBg{22, 24, 29, 255};          // #16181D
const Color kFloor{28, 31, 38, 255};       // #1C1F26
const Color kWall{42, 46, 55, 255};        // #2A2E37
const Color kCounter{47, 51, 61, 255};     // #2F333D
const Color kMat{38, 42, 50, 255};         // #262A32
const Color kTableEdge{58, 63, 74, 255};   // #3A3F4A
const Color kPlant{51, 64, 54, 255};       // #334036
const Color kInk{230, 226, 218, 255};      // #E6E2DA robots, main text
const Color kMuted{138, 143, 152, 255};    // #8A8F98 secondary text
const Color kFaint{90, 96, 107, 255};      // #5A606B paths
const Color kDisabled{74, 79, 89, 255};
const Color kAmber{230, 161, 90, 255};     // #E6A15A food and orders

// Planner buttons, in P-key cycling order.
struct PlannerChoice {
    ww::Planner planner;
    const char* label;
};
constexpr PlannerChoice kPlanners[] = {
    {ww::Planner::Local, "Local"},
    {ww::Planner::SpaceTime, "Space-time A*"},
    {ww::Planner::Cbs, "CBS"},
};

// The UI font, loaded once after the window opens.
Font gFont{};

// Given: an open window. Approach: rasterise the embedded TTF once at a size
// big enough for the 2x GIF render, with bilinear filtering for the smaller
// sizes we draw. ASCII plus the middle dot used in the HUD.
void loadUiFont() {
    std::vector<int> codepoints;
    for (int c = 32; c < 127; ++c) codepoints.push_back(c);
    codepoints.push_back(0x00B7);  // ·
    gFont = LoadFontFromMemory(".ttf", kUiFontData, static_cast<int>(sizeof(kUiFontData)), 32, codepoints.data(),
                               static_cast<int>(codepoints.size()));
    SetTextureFilter(gFont.texture, TEXTURE_FILTER_BILINEAR);
}

void text(const std::string& s, float x, float y, float size, Color c) {
    DrawTextEx(gFont, s.c_str(), {x, y}, size, 0, c);
}

float measure(const std::string& s, float size) { return MeasureTextEx(gFont, s.c_str(), size, 0).x; }

struct Options {
    int robots = 16;
    std::uint64_t seed = 42;
    int hzIndex = 2;  // 8 Hz, same as the terminal app
    ww::Planner planner = ww::Planner::Local;
    std::string mapPath;
    std::string recordPath;
    int recordFrames = 240;
    int recordFps = 30;
};

struct App {
    Options opt;
    std::string mapText;
    std::unique_ptr<ww::Simulation> sim;
    ww::FixedTimestep timestep{1.0 / kHzSteps[2]};
    std::vector<Cell> prev, cur;  // by robot id: before and after the last tick
    std::vector<Cell> facing;     // by robot id: last direction moved (kept while waiting)
    std::vector<Rectangle> shelves;
    bool paused = false;
    bool showAllPaths = false;
    int selected = -1;  // robot id whose path stays visible, -1 for none
    double runTime = 0.0;  // seconds spent unpaused
    std::deque<std::pair<double, long long>> rateSamples;
    std::vector<std::pair<Cell, double>> pops;  // delivered-at drop-off cell, runTime
    int width = 0, height = 0;
    int gridX = kMargin;  // left edge of the grid (centred when the window is wider)
    int prevTouches = 0;
};

double hz(const App& a) { return kHzSteps[a.opt.hzIndex]; }

// Given: the grid. Approach: flood-fill each counter block and keep its
// bounding box, so a block draws as one band instead of separate cells.
std::vector<Rectangle> shelfBlocks(const ww::Grid& g, int originX) {
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
        const float inset = 3.0f;
        out.push_back({originX + x0 * kCell + inset, kHudH + y0 * kCell + inset,
                       (x1 - x0 + 1) * kCell - 2 * inset, (y1 - y0 + 1) * kCell - 2 * inset});
    }
    return out;
}

void snapshot(const ww::Simulation& sim, std::vector<Cell>& into) {
    std::vector<ww::RobotView> robots = sim.robots();
    into.resize(robots.size());
    for (const auto& r : robots) into[static_cast<std::size_t>(r.id)] = r.cell;
}

// Given: options. Approach: rebuild the world from scratch with the same
// seed, so switching planner or robot count always compares like with like.
void restart(App& a) {
    a.sim = std::make_unique<ww::Simulation>(
        ww::Grid::parse(a.mapText), ww::Simulation::Config{a.opt.robots, a.opt.seed, a.opt.planner});
    a.timestep = ww::FixedTimestep(1.0 / hz(a));
    snapshot(*a.sim, a.cur);
    a.prev = a.cur;
    a.facing.assign(a.cur.size(), Cell{1, 0});
    a.rateSamples.clear();
    a.pops.clear();
    a.selected = -1;
}

enum class Action { None, Pause, Slower, Faster, FewerRobots, MoreRobots, Planner };

struct Button {
    Rectangle rect;
    std::string label;
    Action action;
    int planner = -1;     // index into kPlanners for Action::Planner
    bool active = false;  // current planner
    bool enabled = true;
};

constexpr float kButtonText = 14;

int maxRobots(const App& a) { return static_cast<int>(a.sim->grid().floorCells().size()); }

// Given: app state. Approach: one layout used for both drawing and hit
// testing, so what you see is what you click. Value labels are buttons
// with Action::None. Hit boxes stay 28 px tall even though the text is small.
std::vector<Button> layoutButtons(const App& a) {
    std::vector<Button> out;
    const float y = static_cast<float>(a.height - kFooterH + 6), h = 28;
    float x = kMargin - 6;
    auto add = [&](std::string label, Action act, bool enabled = true) {
        const float w = measure(label, kButtonText) + 14;
        out.push_back({{x, y, w, h}, std::move(label), act, -1, false, enabled});
        x += w;
    };
    const int last = static_cast<int>(std::size(kHzSteps)) - 1;
    char value[32];

    add(a.paused ? "Play" : "Pause", Action::Pause);
    x += 14;
    add("-", Action::Slower, a.opt.hzIndex > 0);
    std::snprintf(value, sizeof value, "%g Hz", hz(a));
    add(value, Action::None);
    add("+", Action::Faster, a.opt.hzIndex < last);
    x += 14;
    add("-", Action::FewerRobots, a.opt.robots > 1);
    std::snprintf(value, sizeof value, "%d robots", a.opt.robots);
    add(value, Action::None);
    add("+", Action::MoreRobots, a.opt.robots < maxRobots(a));

    // Planner buttons, right-aligned.
    x = static_cast<float>(a.width - kMargin + 6);
    for (int i = static_cast<int>(std::size(kPlanners)) - 1; i >= 0; --i) {
        const float w = measure(kPlanners[i].label, kButtonText) + 14;
        x -= w;
        out.push_back({{x, y, w, h}, kPlanners[i].label, Action::Planner, i,
                       kPlanners[i].planner == a.opt.planner, true});
    }
    return out;
}

int plannerIndex(const App& a) {
    for (int i = 0; i < static_cast<int>(std::size(kPlanners)); ++i)
        if (kPlanners[i].planner == a.opt.planner) return i;
    return 0;
}

void setHz(App& a, int hzIndex) {
    hzIndex = std::clamp(hzIndex, 0, static_cast<int>(std::size(kHzSteps)) - 1);
    if (hzIndex == a.opt.hzIndex) return;
    a.opt.hzIndex = hzIndex;
    a.timestep = ww::FixedTimestep(1.0 / hz(a));
    a.rateSamples.clear();
}

void setRobots(App& a, int robots) {
    robots = std::clamp(robots, 1, maxRobots(a));
    if (robots == a.opt.robots) return;
    a.opt.robots = robots;
    restart(a);
}

void setPlanner(App& a, int index) {
    const ww::Planner p = kPlanners[index].planner;
    if (p == a.opt.planner) return;
    a.opt.planner = p;
    restart(a);
}

void apply(App& a, Action act, int planner = -1) {
    switch (act) {
        case Action::Pause: a.paused = !a.paused; break;
        case Action::Slower: setHz(a, a.opt.hzIndex - 1); break;
        case Action::Faster: setHz(a, a.opt.hzIndex + 1); break;
        case Action::FewerRobots: setRobots(a, a.opt.robots - 1); break;
        case Action::MoreRobots: setRobots(a, a.opt.robots + 1); break;
        case Action::Planner: setPlanner(a, planner); break;
        case Action::None: break;
    }
}

Vector2 cellCentre(const App& a, float x, float y) {
    return {a.gridX + (x + 0.5f) * kCell, kHudH + (y + 0.5f) * kCell};
}

// Given: app state. Approach: each robot's drawn position, lerped from its
// previous to its current cell by FixedTimestep::alpha(); indexed by id.
std::vector<Vector2> robotPositions(const App& a) {
    const float alpha = a.paused ? 1.0f : static_cast<float>(a.timestep.alpha());
    std::vector<Vector2> pos(a.cur.size());
    for (std::size_t id = 0; id < a.cur.size(); ++id) {
        const Cell from = id < a.prev.size() ? a.prev[id] : a.cur[id], to = a.cur[id];
        pos[id] = cellCentre(a, from.x + (to.x - from.x) * alpha, from.y + (to.y - from.y) * alpha);
    }
    return pos;
}

// Robot id under a point, or -1.
int robotAt(const App& a, Vector2 p) {
    const std::vector<Vector2> pos = robotPositions(a);
    for (std::size_t id = 0; id < pos.size(); ++id)
        if (std::fabs(p.x - pos[id].x) <= kCell * 0.4f && std::fabs(p.y - pos[id].y) <= kCell * 0.4f)
            return static_cast<int>(id);
    return -1;
}

// Given: a click position. Approach: buttons first (same layout as drawn);
// otherwise clicking a robot pins its path, clicking elsewhere unpins.
void clickAt(App& a, Vector2 at) {
    for (const Button& b : layoutButtons(a)) {
        if (b.action != Action::None && b.enabled && CheckCollisionPointRec(at, b.rect)) {
            apply(a, b.action, b.planner);
            return;
        }
    }
    const int id = robotAt(a, at);
    a.selected = id == a.selected ? -1 : id;
}

// Given: this frame's input. Approach: a click is a mouse press, or else a
// touch that began this frame (raylib's web touch moves the mouse but sets
// no button; desktop reports a mouse press as a touch too, hence the else).
void handleInput(App& a) {
    auto pressed = [](int key) { return IsKeyPressed(key) || IsKeyPressedRepeat(key); };
    if (IsKeyPressed(KEY_SPACE)) apply(a, Action::Pause);
    if (pressed(KEY_EQUAL) || pressed(KEY_KP_ADD)) apply(a, Action::Faster);
    if (pressed(KEY_MINUS) || pressed(KEY_KP_SUBTRACT)) apply(a, Action::Slower);
    if (pressed(KEY_UP)) apply(a, Action::MoreRobots);
    if (pressed(KEY_DOWN)) apply(a, Action::FewerRobots);
    if (IsKeyPressed(KEY_P))
        apply(a, Action::Planner, (plannerIndex(a) + 1) % static_cast<int>(std::size(kPlanners)));
    if (IsKeyPressed(KEY_T)) a.showAllPaths = !a.showAllPaths;

    const int touches = GetTouchPointCount();
    bool clicked = false;
    Vector2 at{};
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        clicked = true;
        at = GetMousePosition();
    } else if (touches > 0 && a.prevTouches == 0) {
        clicked = true;
        at = GetTouchPosition(0);
    }
    a.prevTouches = touches;
    if (clicked) clickAt(a, at);
}

// Given: frame time. Approach: run fixed ticks; before each, keep the old
// positions so drawing can lerp prev -> cur by FixedTimestep::alpha(), note
// which way each robot moved, and catch deliveries for the fade.
void update(App& a, double frameSeconds) {
    if (a.paused) return;
    a.runTime += frameSeconds;
    a.timestep.advance(frameSeconds, [&] {
        a.prev = a.cur;
        const std::vector<ww::RobotView> before = a.sim->robots();
        a.sim->step();
        snapshot(*a.sim, a.cur);
        for (std::size_t id = 0; id < a.cur.size() && id < a.facing.size(); ++id)
            if (!(a.cur[id] == a.prev[id])) a.facing[id] = {a.cur[id].x - a.prev[id].x, a.cur[id].y - a.prev[id].y};
        const std::vector<ww::RobotView> after = a.sim->robots();
        for (std::size_t i = 0; i < before.size() && i < after.size(); ++i)
            if (before[i].carrying && !after[i].carrying) a.pops.emplace_back(before[i].dropoff, a.runTime);
    });
    while (!a.pops.empty() && a.runTime - a.pops.front().second > kPopSeconds) a.pops.erase(a.pops.begin());
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

// Given: a drop-off cell. Approach: the table it serves is the first table
// next to it; on maps with 'S' stations the cell itself.
Cell tableOf(const ww::Grid& g, Cell dropoff) {
    for (Cell d : ww::kNeighbors) {
        const Cell n{dropoff.x + d.x, dropoff.y + d.y};
        if (g.inBounds(n) && g.at(n) == ww::Tile::Table) return n;
    }
    return dropoff;
}

Color withAlpha(Color c, float a) {
    c.a = static_cast<unsigned char>(255 * std::clamp(a, 0.0f, 1.0f));
    return c;
}

// Faint dots every few pixels along a polyline.
void drawDottedPath(const std::vector<Vector2>& pts) {
    constexpr float kGap = 6.0f;
    float carry = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const Vector2 a = pts[i - 1], b = pts[i];
        const float len = std::hypot(b.x - a.x, b.y - a.y);
        for (float d = kGap - carry; d < len; d += kGap)
            DrawCircleV({a.x + (b.x - a.x) * d / len, a.y + (b.y - a.y) * d / len}, 1.2f, kFaint);
        carry = std::fmod(carry + len, kGap);
    }
}

// Rounded square, ~60% of a cell: hollow off-white on the way to the
// kitchen, filled amber when carrying. Two tiny eyes look where it's going.
void drawRobot(Vector2 c, bool carrying, Cell facing, bool selected) {
    const float s = kCell * 0.6f;
    const Rectangle body{c.x - s / 2, c.y - s / 2, s, s};
    const float round = 4.0f / s;  // ~2 px corner radius
    if (selected) DrawRectangleRoundedLinesEx({body.x - 4, body.y - 4, s + 8, s + 8}, 8.0f / (s + 8), 6, 1.0f, kMuted);
    if (carrying) {
        DrawRectangleRounded(body, round, 4, kAmber);
    } else {
        DrawRectangleRounded(body, round, 4, kBg);
        DrawRectangleRoundedLinesEx({body.x + 0.75f, body.y + 0.75f, s - 1.5f, s - 1.5f}, round, 4, 1.5f, kInk);
    }
    const Vector2 d{static_cast<float>(facing.x), static_cast<float>(facing.y)};
    const Vector2 side{-d.y, d.x};
    const Color eye = carrying ? kBg : kInk;
    for (float k : {-1.0f, 1.0f})
        DrawCircleV({c.x + d.x * s * 0.2f + side.x * k * s * 0.17f, c.y + d.y * s * 0.2f + side.y * k * s * 0.17f},
                    1.6f, eye);
}

// Given: current state. Approach: draw the scene in window coordinates; the
// caller decides the target (screen, or an offscreen texture for recording).
// Flat shapes, thin edges, no shadows; amber only for food.
void drawScene(const App& a) {
    const ww::Grid& g = a.sim->grid();
    ClearBackground(kBg);
    auto at = [&](Cell c) { return cellCentre(a, float(c.x), float(c.y)); };
    const std::vector<ww::RobotView> robots = a.sim->robots();
    const auto cells = static_cast<std::size_t>(g.width() * g.height());

    for (int y = 0; y < g.height(); ++y)
        for (int x = 0; x < g.width(); ++x)
            DrawRectangle(a.gridX + x * kCell, kHudH + y * kCell, kCell, kCell,
                          g.at({x, y}) == ww::Tile::Wall ? kWall : kFloor);
    for (Cell f : g.pickFaces()) {
        const Vector2 c = at(f);
        DrawRectangleRounded({c.x - kCell * 0.4f, c.y - kCell * 0.4f, kCell * 0.8f, kCell * 0.8f}, 0.2f, 4, kMat);
    }
    for (const Rectangle& r : a.shelves) DrawRectangleRounded(r, 0.12f, 4, kCounter);

    // Orders waiting per table.
    std::vector<int> orders(cells, 0);
    int stuck = 0;
    for (const ww::RobotView& r : robots) {
        ++orders[g.index(tableOf(g, r.dropoff))];
        stuck += r.waitTicks >= kStuckTicks;
    }
    for (std::size_t i = 0; i < cells; ++i) {
        const Cell c = g.cellAt(static_cast<int>(i));
        const ww::Tile t = g.at(c);
        if (t == ww::Tile::Plant) DrawCircleV(at(c), kCell * 0.2f, kPlant);
        if (t != ww::Tile::Table && t != ww::Tile::Station) continue;
        DrawCircleV(at(c), kCell * 0.32f + 1.5f, kTableEdge);
        DrawCircleV(at(c), kCell * 0.32f, kFloor);
        if (orders[i] == 0) continue;
        DrawCircleV(at(c), 3.5f, kAmber);
        if (orders[i] > 1) text(std::to_string(orders[i]), at(c).x + kCell * 0.32f + 4, at(c).y - 8, 11, kMuted);
    }
    // Deliveries: the order dot fades out and a small "+1" drifts up (newest
    // per table only, so they never stack).
    std::vector<double> newest(cells, -1.0);
    for (const auto& [dropoff, t0] : a.pops) newest[g.index(tableOf(g, dropoff))] = t0;
    for (std::size_t i = 0; i < cells; ++i) {
        if (newest[i] < 0) continue;
        const float k = static_cast<float>((a.runTime - newest[i]) / kPopSeconds);
        const Vector2 c = at(g.cellAt(static_cast<int>(i)));
        DrawCircleV(c, 3.5f, withAlpha(kAmber, 1.0f - k));
        text("+1", c.x - 6, c.y - 18 - 12 * k, 12, withAlpha(kMuted, 1.0f - k));
    }

    // Paths: hovered or pinned robot, or all with T. Planned cells while
    // there are any (space-time A*, CBS), then straight to the goal.
    const std::vector<Vector2> pos = robotPositions(a);
    const int hovered = robotAt(a, GetMousePosition());
    for (const ww::RobotView& r : robots) {
        if (!a.showAllPaths && r.id != hovered && r.id != a.selected) continue;
        std::vector<Vector2> pts{pos[static_cast<std::size_t>(r.id)]};
        bool reached = r.cell == r.goal;
        for (std::size_t j = 1; j < r.route.size() && !reached; ++j) {
            pts.push_back(at(r.route[j]));
            reached = r.route[j] == r.goal;
        }
        if (!reached) pts.push_back(at(r.goal));
        drawDottedPath(pts);
    }
    for (const ww::RobotView& r : robots) {
        const auto id = static_cast<std::size_t>(r.id);
        drawRobot(pos[id], r.carrying, id < a.facing.size() ? a.facing[id] : Cell{1, 0}, r.id == a.selected);
    }

    // HUD: one quiet line.
    char hud[96];
    std::snprintf(hud, sizeof hud, "%lld served · %.0f/min · %d stuck%s", a.sim->stats().deliveries,
                  deliveriesPerMinute(a), stuck, a.paused ? " · paused" : "");
    text(hud, kMargin, 8, 15, kInk);
    const std::string keys = "space pause · +/- speed · up/down robots · P planner · T paths";
    text(keys, a.width - kMargin - measure(keys, 12), 10, 12, kMuted);

    // Controls: quiet text buttons; the active planner in off-white with a
    // thin amber underline.
    const Vector2 mouse = GetMousePosition();
    for (const Button& b : layoutButtons(a)) {
        const float tw = measure(b.label, kButtonText);
        const float tx = b.rect.x + (b.rect.width - tw) / 2, ty = b.rect.y + 6;
        Color c = kMuted;
        if (b.action == Action::None) c = kMuted;
        else if (!b.enabled) c = kDisabled;
        else if (b.active || CheckCollisionPointRec(mouse, b.rect)) c = kInk;
        text(b.label, tx, ty, kButtonText, c);
        if (b.active) DrawRectangleRec({tx, ty + kButtonText + 3, tw, 1.5f}, kAmber);
    }
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
// fixed 1/fps frames into a 2x offscreen texture, downscale (cheap
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
        update(a, 1.0 / a.opt.recordFps);
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
        ok = msf_gif_frame(&gif, static_cast<std::uint8_t*>(img.data), 100 / a.opt.recordFps, 16, img.width * 4);
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
    std::cout << "usage: viewer [--robots N] [--seed S] [--hz 2|4|8|15|30|60|120] [--map FILE|cafe|warehouse]\n"
                 "              [--planner local|stastar|cbs] [--record OUT.gif [--frames N] [--fps N]]\n";
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
            } else if (arg == "--planner") {
                auto p = ww::parsePlanner(next());
                if (!p) usage(1);
                o.planner = *p;
            } else if (arg == "--map") o.mapPath = next();
            else if (arg == "--record") o.recordPath = next();
            else if (arg == "--frames") o.recordFrames = std::stoi(next());
            else if (arg == "--fps") o.recordFps = std::clamp(std::stoi(next()), 1, 50);
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
        if (a.opt.mapPath.empty()) a.mapText = std::string(ww::cafeMap());
        else if (auto builtin = ww::builtinMap(a.opt.mapPath)) a.mapText = std::string(*builtin);
#if !defined(__EMSCRIPTEN__)
        else a.mapText = loadMap(a.opt.mapPath);
#endif
        restart(a);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    const ww::Grid& g = a.sim->grid();
    a.width = std::max(kMinWidth, 2 * kMargin + g.width() * kCell);
    a.height = kHudH + g.height() * kCell + kFooterH;
    a.gridX = (a.width - g.width() * kCell) / 2;
    a.shelves = shelfBlocks(g, a.gridX);

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(a.width, a.height, "Rush Hour Café");
    loadUiFont();

#if defined(__EMSCRIPTEN__)
    emscripten_set_main_loop_arg(frame, &a, 0, 1);
#else
    int rc = 0;
    if (!a.opt.recordPath.empty()) {
        rc = record(a);
    } else {
        SetTargetFPS(60);
        while (!WindowShouldClose()) frame(&a);
    }
    UnloadFont(gFont);
    CloseWindow();
    return rc;
#endif
    return 0;
}
