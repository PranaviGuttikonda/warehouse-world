// viewer: Rush Hour Café, the graphical view of the robot world (raylib).
//
//   ./viewer                              split-screen: Local vs CBS on the café
//   ./viewer --left stastar --right cbs   pick each side's planner
//   ./viewer --single --planner cbs       one café, one planner
//   ./viewer --map warehouse              the benchmark warehouse map
//   ./viewer --record docs/viewer.gif     write a short GIF and exit
//
// Keys: space pause, +/- speed, up/down robot count, P cycle a planner
// (the right side in split-screen), S split/single, H heatmap, T show all
// planned paths. Every control is also clickable (mouse or touch); hover a
// robot for its card, click it to pin its path. An intro card explains the
// scene on launch (skip with --no-intro); "i" reopens it.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
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

constexpr float kCellSingle = 32;  // cell size in pixels, single view
constexpr float kCellSplit = 22;   // ... split-screen (two cafés side by side)
constexpr int kMargin = 16;
constexpr int kHudH = 46;       // lunch-rush panel and served badge
constexpr int kSideH = 30;      // per-side header (split-screen only)
constexpr int kLegendH = 26;    // legend row under the floor
constexpr int kFooterH = 42;    // pills and planner buttons
constexpr int kMinWidth = 720;  // room for the control bar on small maps
constexpr double kHzSteps[] = {2, 4, 8, 15, 30, 60, 120};
constexpr double kRateWindow = 10.0;    // seconds of running time for orders/min
constexpr double kPopSeconds = 0.8;     // delivery: bubble pops, "+1" floats up
constexpr double kRushSeconds = 120.0;  // lunch-rush timer; loops until the game loop exists
constexpr int kStuckTicks = 3;          // a robot counts as stuck after this many blocked ticks
constexpr float kHeatDecay = 0.97f;     // per tick: a wait fades to half in ~23 ticks
constexpr float kHeatFull = 6.0f;       // heat at which the tint is strongest
constexpr float kHeatMaxAlpha = 0.25f;  // strongest tint

// Cozy, toned-down café palette (docs/VISION.md). No shadows, no gradients.
const Color kLine{90, 70, 54, 255};          // #5A4636 soft brown outlines and text
const Color kLineSoft{154, 133, 116, 255};   // secondary text
const Color kPage{251, 244, 233, 255};       // #FBF4E9
const Color kFloorA{250, 242, 230, 255};     // #FAF2E6
const Color kFloorB{244, 232, 214, 255};     // #F4E8D6
const Color kCounter{210, 122, 104, 255};    // #D27A68
const Color kMat{235, 203, 130, 255};        // #EBCB82
const Color kWhite{255, 255, 255, 255};
const Color kRim{233, 199, 192, 255};        // #E9C7C0
const Color kLeafA{143, 192, 138, 255};      // #8FC08A
const Color kLeafB{156, 203, 150, 255};      // #9CCB96
const Color kPot{201, 139, 99, 255};         // #C98B63
const Color kRobot{140, 196, 224, 255};      // #8CC4E0
const Color kFood{210, 122, 104, 255};       // #D27A68
const Color kCup{154, 107, 79, 255};         // #9A6B4F
const Color kBadge{246, 223, 160, 255};      // #F6DFA0
constexpr float kEdge = 1.5f;   // outline width
constexpr float kFrame = 2.0f;  // floor frame width

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

const char* plannerLabel(ww::Planner p) {
    for (const PlannerChoice& c : kPlanners)
        if (c.planner == p) return c.label;
    return "?";
}

// The UI font, loaded once after the window opens.
Font gFont{};
// Cell size for the current view; every grid shape scales with it.
float gCell = kCellSplit;

// Given: an open window. Approach: rasterise the embedded TTF once at a size
// big enough for the 2x GIF render, with bilinear filtering for the smaller
// sizes we draw. ASCII plus the middle dot and the é in "Café".
void loadUiFont() {
    std::vector<int> codepoints;
    for (int c = 32; c < 127; ++c) codepoints.push_back(c);
    codepoints.push_back(0x00B7);
    codepoints.push_back(0x00E9);
    gFont = LoadFontFromMemory(".ttf", kUiFontData, static_cast<int>(sizeof(kUiFontData)), 32, codepoints.data(),
                               static_cast<int>(codepoints.size()));
    SetTextureFilter(gFont.texture, TEXTURE_FILTER_BILINEAR);
}

void text(const std::string& s, float x, float y, float size, Color c) {
    DrawTextEx(gFont, s.c_str(), {x, y}, size, 0, c);
}

float measure(const std::string& s, float size) { return MeasureTextEx(gFont, s.c_str(), size, 0).x; }

Color withAlpha(Color c, float a) {
    c.a = static_cast<unsigned char>(255 * std::clamp(a, 0.0f, 1.0f));
    return c;
}

struct Options {
    int robots = 16;
    std::uint64_t seed = 42;
    int hzIndex = 2;  // 8 Hz, same as the terminal app
    ww::Planner planners[2] = {ww::Planner::Local, ww::Planner::Cbs};  // left, right
    bool split = true;
    std::string mapPath;
    std::string recordPath;
    int recordFrames = 240;
    int recordFps = 30;
    bool intro = true;
};

// One café: a simulation plus everything the view keeps about it.
struct Side {
    ww::Planner planner = ww::Planner::Local;
    std::unique_ptr<ww::Simulation> sim;
    std::vector<Cell> prev, cur;  // by robot id: before and after the last tick
    std::vector<Cell> facing;     // by robot id: last direction moved (kept while waiting)
    std::vector<float> heat;      // by cell: decaying count of robot wait-ticks
    std::vector<std::pair<Cell, double>> pops;  // delivered-at drop-off cell, runTime
    std::deque<std::pair<double, long long>> rateSamples;
    std::vector<Rectangle> shelves;
    float originX = 0, originY = 0;  // top-left of the grid
};

struct App {
    Options opt;
    std::string mapText;
    std::vector<Side> sides;  // two in split-screen, one in single view
    bool split = true;
    bool heatmap = true;
    ww::FixedTimestep timestep{1.0 / kHzSteps[2]};
    bool paused = false;
    bool showAllPaths = false;
    bool showIntro = false;  // the intro card is up; the run waits behind it
    int selectedSide = -1, selectedId = -1;  // robot whose path stays visible
    double runTime = 0.0;  // seconds spent unpaused
    int gridW = 0, gridH = 0;  // grid size in cells
    int width = 0, height = 0;
    float cell = kCellSplit;
    int prevTouches = 0;
};

double hz(const App& a) { return kHzSteps[a.opt.hzIndex]; }

float gridPxW(const App& a) { return a.gridW * a.cell; }
float gridPxH(const App& a) { return a.gridH * a.cell; }
float gridTop(const App& a) { return static_cast<float>(kHudH + (a.split ? kSideH : 0)); }

// Given: the grid. Approach: flood-fill each counter block and keep its
// bounding box, so a block draws as one counter instead of separate cells.
std::vector<Rectangle> shelfBlocks(const ww::Grid& g, float originX, float originY, float cell) {
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
        const float inset = cell * 0.125f;
        out.push_back({originX + x0 * cell + inset, originY + y0 * cell + inset, (x1 - x0 + 1) * cell - 2 * inset,
                       (y1 - y0 + 1) * cell - 2 * inset});
    }
    return out;
}

// Given: the view mode and grid size. Approach: pick the cell size, place
// each side's grid, and size the window; resizes it if it's already open.
void layoutView(App& a) {
    a.cell = a.split ? kCellSplit : kCellSingle;
    gCell = a.cell;
    const float top = gridTop(a);
    if (a.split) {
        a.width = static_cast<int>(2 * gridPxW(a) + 3 * kMargin);
        for (std::size_t s = 0; s < a.sides.size(); ++s)
            a.sides[s].originX = kMargin + s * (gridPxW(a) + kMargin);
    } else {
        a.width = std::max(kMinWidth, static_cast<int>(gridPxW(a)) + 2 * kMargin);
        a.sides[0].originX = (a.width - gridPxW(a)) / 2;
    }
    a.height = static_cast<int>(top + gridPxH(a)) + kLegendH + kFooterH;
    for (Side& s : a.sides) {
        s.originY = top;
        if (s.sim) s.shelves = shelfBlocks(s.sim->grid(), s.originX, s.originY, a.cell);
    }
    if (IsWindowReady()) SetWindowSize(a.width, a.height);
}

void snapshot(const ww::Simulation& sim, std::vector<Cell>& into) {
    std::vector<ww::RobotView> robots = sim.robots();
    into.resize(robots.size());
    for (const auto& r : robots) into[static_cast<std::size_t>(r.id)] = r.cell;
}

// Given: options. Approach: rebuild every side from scratch with the same
// seed and robot count, so the two planners always start from the same café
// and orders, and stay tick for tick in step.
void restart(App& a) {
    a.sides.resize(a.split ? 2 : 1);
    for (std::size_t s = 0; s < a.sides.size(); ++s) {
        Side& side = a.sides[s];
        side.planner = a.opt.planners[s];
        side.sim = std::make_unique<ww::Simulation>(
            ww::Grid::parse(a.mapText), ww::Simulation::Config{a.opt.robots, a.opt.seed, side.planner});
        snapshot(*side.sim, side.cur);
        side.prev = side.cur;
        side.facing.assign(side.cur.size(), Cell{1, 0});
        side.heat.assign(static_cast<std::size_t>(side.sim->grid().width() * side.sim->grid().height()), 0.0f);
        side.pops.clear();
        side.rateSamples.clear();
    }
    a.gridW = a.sides[0].sim->grid().width();
    a.gridH = a.sides[0].sim->grid().height();
    a.timestep = ww::FixedTimestep(1.0 / hz(a));
    a.selectedSide = a.selectedId = -1;
    layoutView(a);
}

enum class Action { None, Pause, Slower, Faster, FewerRobots, MoreRobots, Planner, Info, Split, Heatmap };

// A clickable (or label-only, Action::None) item. Items sharing a group sit
// inside one pill; planner buttons have group -1 and draw as keys.
struct Button {
    Rectangle rect;
    std::string label;
    Action action;
    int planner = -1;     // index into kPlanners for Action::Planner
    bool active = false;  // current planner, or a toggle that's on
    bool enabled = true;
    int group = -1;
    int side = 0;         // which side a planner key belongs to
};

constexpr float kButtonText = 13;

int maxRobots(const App& a) { return static_cast<int>(a.sides[0].sim->grid().floorCells().size()); }

int stuckCount(const Side& s) {
    int n = 0;
    for (const ww::RobotView& r : s.sim->robots()) n += r.waitTicks >= kStuckTicks;
    return n;
}

// Planner keys for one side, laid out from x; returns the x after them.
float addPlannerKeys(std::vector<Button>& out, const App& a, int side, float x, float y, float h, float size) {
    for (int i = 0; i < static_cast<int>(std::size(kPlanners)); ++i) {
        const float w = measure(kPlanners[i].label, size) + 2 * std::round(size * 0.8f);
        out.push_back({{x, y, w, h}, kPlanners[i].label, Action::Planner, i,
                       kPlanners[i].planner == a.sides[side].planner, true, -1, side});
        x += w + 6;
    }
    return x;
}

// Given: app state. Approach: one layout used for both drawing and hit
// testing, so what you see is what you click. Bottom left: pills for pause,
// speed, (stuck,) robot count, compare, heatmap and info. Planner keys sit
// bottom right in single view, or in each side's header in split-screen.
std::vector<Button> layoutButtons(const App& a) {
    std::vector<Button> out;
    const float y = static_cast<float>(a.height - kFooterH + 8), h = 26;
    float x = kMargin;
    int group = 0;
    auto add = [&](std::string label, Action act, bool enabled = true, float pad = 10, bool active = false) {
        const float w = measure(label, kButtonText) + 2 * pad;
        out.push_back({{x, y, w, h}, std::move(label), act, -1, active, enabled, group, 0});
        x += w;
    };
    auto nextPill = [&] {
        ++group;
        x += 8;
    };
    const int last = static_cast<int>(std::size(kHzSteps)) - 1;
    char value[32];

    add(a.paused ? "Play" : "Pause", Action::Pause, true, 12);
    nextPill();
    add("-", Action::Slower, a.opt.hzIndex > 0, 9);
    std::snprintf(value, sizeof value, "%g Hz", hz(a));
    add(value, Action::None, true, 2);
    add("+", Action::Faster, a.opt.hzIndex < last, 9);
    if (!a.split) {  // in split-screen each side's header shows its own stuck count
        nextPill();
        std::snprintf(value, sizeof value, "%d stuck", stuckCount(a.sides[0]));
        add(value, Action::None, true, 12);
    }
    nextPill();
    add("-", Action::FewerRobots, a.opt.robots > 1, 9);
    std::snprintf(value, sizeof value, "%d robots", a.opt.robots);
    add(value, Action::None, true, 2);
    add("+", Action::MoreRobots, a.opt.robots < maxRobots(a), 9);
    nextPill();
    add("Compare", Action::Split, true, 12, a.split);
    nextPill();
    add("Heatmap", Action::Heatmap, true, 12, a.heatmap);
    nextPill();
    add("i", Action::Info, true, 10);

    if (a.split) {
        for (int s = 0; s < 2; ++s)
            addPlannerKeys(out, a, s, a.sides[s].originX, static_cast<float>(kHudH + 3), 22, 12);
    } else {
        float w = 0;
        for (const PlannerChoice& c : kPlanners) w += measure(c.label, kButtonText) + 2 * std::round(kButtonText * 0.8f) + 6;
        addPlannerKeys(out, a, 0, a.width - kMargin - (w - 6), y, h, kButtonText);
    }
    return out;
}

void setHz(App& a, int hzIndex) {
    hzIndex = std::clamp(hzIndex, 0, static_cast<int>(std::size(kHzSteps)) - 1);
    if (hzIndex == a.opt.hzIndex) return;
    a.opt.hzIndex = hzIndex;
    a.timestep = ww::FixedTimestep(1.0 / hz(a));
    for (Side& s : a.sides) s.rateSamples.clear();
}

void setRobots(App& a, int robots) {
    robots = std::clamp(robots, 1, maxRobots(a));
    if (robots == a.opt.robots) return;
    a.opt.robots = robots;
    restart(a);
}

void setPlanner(App& a, int side, int index) {
    const ww::Planner p = kPlanners[index].planner;
    if (p == a.opt.planners[side]) return;
    a.opt.planners[side] = p;
    restart(a);  // both sides, so they stay in step
}

void setSplit(App& a, bool split) {
    if (split == a.split) return;
    a.split = split;
    a.heatmap = split;  // on by default in split-screen
    restart(a);
}

void apply(App& a, Action act, int planner = -1, int side = 0) {
    switch (act) {
        case Action::Pause: a.paused = !a.paused; break;
        case Action::Slower: setHz(a, a.opt.hzIndex - 1); break;
        case Action::Faster: setHz(a, a.opt.hzIndex + 1); break;
        case Action::FewerRobots: setRobots(a, a.opt.robots - 1); break;
        case Action::MoreRobots: setRobots(a, a.opt.robots + 1); break;
        case Action::Planner: setPlanner(a, side, planner); break;
        case Action::Info: a.showIntro = true; break;
        case Action::Split: setSplit(a, !a.split); break;
        case Action::Heatmap: a.heatmap = !a.heatmap; break;
        case Action::None: break;
    }
}

int plannerIndex(ww::Planner p) {
    for (int i = 0; i < static_cast<int>(std::size(kPlanners)); ++i)
        if (kPlanners[i].planner == p) return i;
    return 0;
}

Vector2 cellCentre(const Side& s, float x, float y) {
    return {s.originX + (x + 0.5f) * gCell, s.originY + (y + 0.5f) * gCell};
}

// Given: a side. Approach: each robot's drawn position, lerped from its
// previous to its current cell by FixedTimestep::alpha(); indexed by id.
std::vector<Vector2> robotPositions(const App& a, const Side& s) {
    const float alpha = a.paused ? 1.0f : static_cast<float>(a.timestep.alpha());
    std::vector<Vector2> pos(s.cur.size());
    for (std::size_t id = 0; id < s.cur.size(); ++id) {
        const Cell from = id < s.prev.size() ? s.prev[id] : s.cur[id], to = s.cur[id];
        pos[id] = cellCentre(s, from.x + (to.x - from.x) * alpha, from.y + (to.y - from.y) * alpha);
    }
    return pos;
}

// The robot under a point: {side, id}, or {-1, -1}.
std::pair<int, int> robotAt(const App& a, Vector2 p) {
    for (int side = 0; side < static_cast<int>(a.sides.size()); ++side) {
        const std::vector<Vector2> pos = robotPositions(a, a.sides[side]);
        for (std::size_t id = 0; id < pos.size(); ++id)
            if (std::fabs(p.x - pos[id].x) <= gCell * 0.4f && std::fabs(p.y - pos[id].y) <= gCell * 0.4f)
                return {side, static_cast<int>(id)};
    }
    return {-1, -1};
}

// Given: a robot. Approach: one line for the hover card.
std::string robotCard(const Side& s, int id) {
    for (const ww::RobotView& r : s.sim->robots()) {
        if (r.id != id) continue;
        char line[96];
        std::snprintf(line, sizeof line, "#%d · %s · waited %d tick%s", r.id,
                      r.carrying ? "carrying" : "going to kitchen", r.waitTicks, r.waitTicks == 1 ? "" : "s");
        return line;
    }
    return {};
}

// Given: a click position. Approach: with the intro card up, any click
// starts the run. Otherwise buttons first (same layout as drawn), then
// clicking a robot pins its path, clicking elsewhere unpins.
void clickAt(App& a, Vector2 at) {
    if (a.showIntro) {
        a.showIntro = false;
        return;
    }
    for (const Button& b : layoutButtons(a)) {
        if (b.action != Action::None && b.enabled && CheckCollisionPointRec(at, b.rect)) {
            apply(a, b.action, b.planner, b.side);
            return;
        }
    }
    const auto [side, id] = robotAt(a, at);
    const bool same = side == a.selectedSide && id == a.selectedId;
    a.selectedSide = same ? -1 : side;
    a.selectedId = same ? -1 : id;
}

// Given: this frame's input. Approach: a click is a mouse press, or else a
// touch that began this frame (raylib's web touch moves the mouse but sets
// no button; desktop reports a mouse press as a touch too, hence the else).
void handleInput(App& a) {
    auto pressed = [](int key) { return IsKeyPressed(key) || IsKeyPressedRepeat(key); };
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

    if (a.showIntro) {  // Space, Enter or a click starts; other keys wait
        if (clicked || IsKeyPressed(KEY_SPACE) || IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER))
            a.showIntro = false;
        return;
    }
    if (IsKeyPressed(KEY_SPACE)) apply(a, Action::Pause);
    if (pressed(KEY_EQUAL) || pressed(KEY_KP_ADD)) apply(a, Action::Faster);
    if (pressed(KEY_MINUS) || pressed(KEY_KP_SUBTRACT)) apply(a, Action::Slower);
    if (pressed(KEY_UP)) apply(a, Action::MoreRobots);
    if (pressed(KEY_DOWN)) apply(a, Action::FewerRobots);
    if (IsKeyPressed(KEY_P)) {
        const int side = a.split ? 1 : 0;
        apply(a, Action::Planner, (plannerIndex(a.opt.planners[side]) + 1) % static_cast<int>(std::size(kPlanners)), side);
    }
    if (IsKeyPressed(KEY_S)) apply(a, Action::Split);
    if (IsKeyPressed(KEY_H)) apply(a, Action::Heatmap);
    if (IsKeyPressed(KEY_T)) a.showAllPaths = !a.showAllPaths;
    if (clicked) clickAt(a, at);
}

// Given: frame time. Approach: run fixed ticks on every side together;
// before each, keep the old positions so drawing can lerp prev -> cur by
// FixedTimestep::alpha(). After each, note which way robots moved, decay
// the heatmap and add this tick's waits, and catch deliveries for the pop.
void update(App& a, double frameSeconds) {
    if (a.paused || a.showIntro) return;
    a.runTime += frameSeconds;
    a.timestep.advance(frameSeconds, [&] {
        for (Side& s : a.sides) {
            s.prev = s.cur;
            const std::vector<ww::RobotView> before = s.sim->robots();
            s.sim->step();
            snapshot(*s.sim, s.cur);
            for (std::size_t id = 0; id < s.cur.size() && id < s.facing.size(); ++id)
                if (!(s.cur[id] == s.prev[id])) s.facing[id] = {s.cur[id].x - s.prev[id].x, s.cur[id].y - s.prev[id].y};
            const std::vector<ww::RobotView> after = s.sim->robots();
            for (float& h : s.heat) h *= kHeatDecay;
            const ww::Grid& g = s.sim->grid();
            for (std::size_t i = 0; i < after.size(); ++i) {
                if (after[i].waitTicks > 0) s.heat[g.index(after[i].cell)] += 1.0f;
                if (i < before.size() && before[i].carrying && !after[i].carrying)
                    s.pops.emplace_back(before[i].dropoff, a.runTime);
            }
        }
    });
    for (Side& s : a.sides) {
        while (!s.pops.empty() && a.runTime - s.pops.front().second > kPopSeconds) s.pops.erase(s.pops.begin());
        s.rateSamples.emplace_back(a.runTime, s.sim->stats().deliveries);
        while (s.rateSamples.size() > 2 && a.runTime - s.rateSamples.front().first > kRateWindow)
            s.rateSamples.pop_front();
    }
}

double deliveriesPerMinute(const Side& s) {
    if (s.rateSamples.size() < 2) return 0.0;
    auto [t0, d0] = s.rateSamples.front();
    auto [t1, d1] = s.rateSamples.back();
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

// --- Drawing. Every shape takes its size, so the legend reuses them small.

void outlinedCircle(Vector2 c, float r, Color fill, float edge = kEdge) {
    DrawCircleV(c, r + edge, kLine);
    DrawCircleV(c, r, fill);
}

void outlinedRect(Rectangle r, float radius, Color fill, float edge = kEdge) {
    const Rectangle o{r.x - edge, r.y - edge, r.width + 2 * edge, r.height + 2 * edge};
    DrawRectangleRounded(o, (radius + edge) * 2 / std::min(o.width, o.height), 8, kLine);
    DrawRectangleRounded(r, radius * 2 / std::min(r.width, r.height), 8, fill);
}

void drawPlant(Vector2 c, float size) {  // size: overall width (~55% of a cell)
    const float s = size;
    outlinedRect({c.x - s * 0.24f, c.y + s * 0.08f, s * 0.48f, s * 0.36f}, s * 0.08f, kPot, 1.2f);
    const Vector2 leaves[3] = {{c.x - s * 0.2f, c.y - s * 0.08f}, {c.x + s * 0.2f, c.y - s * 0.08f},
                               {c.x, c.y - s * 0.26f}};
    for (Vector2 l : leaves) DrawCircleV(l, s * 0.2f + 1.2f, kLine);
    DrawCircleV(leaves[0], s * 0.2f, kLeafA);
    DrawCircleV(leaves[1], s * 0.2f, kLeafB);
    DrawCircleV(leaves[2], s * 0.2f, kLeafA);
}

void drawTable(Vector2 c) {
    const float r = gCell * 0.3f;  // ~60% of a cell across
    outlinedCircle(c, r, kWhite);
    for (int i = 0; i < 12; ++i) {  // faint dashed rim
        const float a0 = i * 30.0f;
        DrawRing(c, r * 0.72f, r * 0.72f + 1.0f, a0, a0 + 16.0f, 4, kRim);
    }
}

// A thin yellow mat along one edge of a cell: `toward` is the side the
// counter is on, so the mat hugs the counter.
void drawMat(Vector2 c, float length, Cell toward) {
    const float thick = std::max(2.5f, length * 0.2f), off = gCell * 0.5f - thick / 2 - gCell * 0.08f;
    const bool across = toward.y != 0;  // counter above/below: a horizontal strip
    const Vector2 m{c.x + toward.x * off, c.y + toward.y * off};
    const Rectangle r = across ? Rectangle{m.x - length / 2, m.y - thick / 2, length, thick}
                               : Rectangle{m.x - thick / 2, m.y - length / 2, thick, length};
    DrawRectangleRounded(r, 1.0f, 6, kMat);
}

// White plate with a coral dish on it, about `w` wide.
void drawPlate(Vector2 c, float w) {
    DrawEllipse(static_cast<int>(c.x), static_cast<int>(c.y), w / 2 + 1.2f, w * 0.17f + 1.2f, kLine);
    DrawEllipse(static_cast<int>(c.x), static_cast<int>(c.y), w / 2, w * 0.17f, kWhite);
    DrawCircleSector({c.x, c.y - 0.5f}, w * 0.26f + 1.0f, 180, 360, 12, kLine);
    DrawCircleSector({c.x, c.y - 0.5f}, w * 0.26f, 180, 360, 12, kFood);
}

void drawCup(Vector2 c, float h) {  // small brown cup, h tall
    DrawRing({c.x + h * 0.42f, c.y}, h * 0.16f, h * 0.28f, 0, 360, 12, kCup);  // handle
    DrawRectangleRounded({c.x - h * 0.4f, c.y - h * 0.45f, h * 0.8f, h * 0.9f}, 0.35f, 6, kCup);
}

// Small speech bubble with a cup over a table; `cell` sets its size (~55% of
// a cell wide), `scale` > 1 and `alpha` < 1 while it pops away.
void drawBubble(Vector2 table, float cell, float scale, float alpha, int orders) {
    const float k = cell / 32.0f;
    const float w = cell * 0.55f * scale, h = cell * 0.42f * scale;
    const Vector2 c{table.x, table.y - cell * 0.62f};
    const Rectangle r{c.x - w / 2, c.y - h / 2, w, h};
    const Color line = withAlpha(kLine, alpha), fill = withAlpha(kWhite, alpha);
    DrawTriangle({c.x - 3.5f * k * scale, r.y + h - 0.5f}, {c.x, r.y + h + 4.5f * k * scale},
                 {c.x + 3.5f * k * scale, r.y + h - 0.5f}, line);
    DrawRectangleRounded({r.x - kEdge, r.y - kEdge, w + 2 * kEdge, h + 2 * kEdge}, 0.6f, 8, line);
    DrawRectangleRounded(r, 0.6f, 8, fill);
    DrawTriangle({c.x - 2.2f * k * scale, r.y + h - 1}, {c.x, r.y + h + 2.6f * k * scale},
                 {c.x + 2.2f * k * scale, r.y + h - 1}, fill);
    if (alpha >= 1.0f) drawCup({c.x - h * 0.08f, c.y + 0.5f}, h * 0.5f);
    if (orders > 1 && alpha >= 1.0f) {
        const Vector2 b{r.x + w, r.y};
        DrawCircleV(b, 5.0f * k + 0.5f, kFood);
        const std::string n = std::to_string(orders);
        const float size = std::max(7.0f, 9.0f * k);
        text(n, b.x - measure(n, size) / 2, b.y - size * 0.6f, size, kWhite);
    }
}

// Rounded-square waiter, ~50% of a cell: two eyes with pupils looking the
// way it's going, a small smile, and a short antenna (replaced by a plate
// with a dish when carrying).
void drawRobot(Vector2 c, bool carrying, Cell facing, float size) {
    const float s = size;
    const Rectangle body{c.x - s / 2, c.y - s / 2, s, s};
    if (!carrying) {
        DrawLineEx({c.x, body.y}, {c.x, body.y - s * 0.3f}, 1.2f, kLine);
        outlinedCircle({c.x, body.y - s * 0.34f}, s * 0.1f, kMat, 1.0f);
    }
    outlinedRect(body, s * 0.26f, kRobot);
    const Vector2 look{facing.x * s * 0.04f, facing.y * s * 0.04f};
    for (float side : {-1.0f, 1.0f}) {
        const Vector2 eye{c.x + side * s * 0.2f, c.y - s * 0.08f};
        DrawCircleV(eye, s * 0.13f, kWhite);
        DrawCircleV({eye.x + look.x, eye.y + look.y}, s * 0.065f, kLine);
    }
    DrawRing({c.x, c.y + s * 0.06f}, s * 0.13f, s * 0.13f + 1.0f, 30, 150, 10, kLine);
    if (carrying) drawPlate({c.x, body.y - s * 0.1f}, s * 1.05f);
}

void drawStar(Vector2 c, float r, Color fill) {
    Vector2 pts[10];
    for (int i = 0; i < 10; ++i) {
        const float ang = -PI / 2 + i * PI / 5, rad = (i % 2) ? r * 0.45f : r;
        pts[i] = {c.x + std::cos(ang) * rad, c.y + std::sin(ang) * rad};
    }
    for (int i = 0; i < 10; ++i) DrawTriangle(c, pts[(i + 1) % 10], pts[i], fill);
}

// Dots every few pixels along a polyline.
void drawDottedPath(const std::vector<Vector2>& pts, Color color) {
    const float gap = std::max(4.0f, gCell * 0.16f);
    float carry = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const Vector2 a = pts[i - 1], b = pts[i];
        const float len = std::hypot(b.x - a.x, b.y - a.y);
        for (float d = gap - carry; d < len; d += gap)
            DrawCircleV({a.x + (b.x - a.x) * d / len, a.y + (b.y - a.y) * d / len}, 1.0f, color);
        carry = std::fmod(carry + len, gap);
    }
}

void drawPill(Rectangle r, Color fill) {
    DrawRectangleRounded({r.x - kEdge, r.y - kEdge, r.width + 2 * kEdge, r.height + 2 * kEdge}, 1.0f, 12, kLine);
    DrawRectangleRounded(r, 1.0f, 12, fill);
}

// Legend: tiny drawn icons with small brown labels on one line (the
// heatmap entry only when it's showing). Returns the width it takes; with
// draw = false it only measures.
float drawLegend(float x, float y, bool heat, bool draw = true) {
    const float start = x;
    auto item = [&](auto icon, const char* label) {
        if (draw) {
            icon(Vector2{x, y});
            text(label, x + 12, y - 7, 12, kLine);
        }
        x += 12 + measure(label, 12) + 22;
    };
    item([&](Vector2 c) { drawRobot({c.x, c.y + 1}, false, Cell{1, 0}, 11); }, "robot waiter");
    item([&](Vector2 c) { drawRobot({c.x, c.y + 2}, true, Cell{1, 0}, 11); }, "carrying an order");
    item([&](Vector2 c) { drawBubble({c.x, c.y + 32 * 0.5f}, 32 * 0.8f, 1.0f, 1.0f, 0); }, "order waiting");
    item([&](Vector2 c) {
        DrawRectangleRounded({c.x - 6, c.y - 6, 12, 2.5f}, 1.0f, 4, kCounter);
        DrawRectangleRounded({c.x - 5.5f, c.y - 1.5f, 11, 3.5f}, 1.0f, 4, kMat);
    }, "pickup spot");
    item([&](Vector2 c) { drawPlant({c.x, c.y + 1}, 14); }, "plant (obstacle)");
    if (heat)
        item([&](Vector2 c) { DrawRectangleRounded({c.x - 6, c.y - 6, 12, 12}, 0.3f, 4, withAlpha(kCounter, kHeatMaxAlpha)); },
             "congestion (heatmap)");
    return x - 22 - start;
}

// Given: the scene drawn behind. Approach: dim it, then a white card with
// the title, what's going on, the legend, one line per planner and a Start
// button. Any click, Space or Enter starts (see handleInput / clickAt).
void drawIntro(const App& a) {
    DrawRectangle(0, 0, a.width, a.height, withAlpha(kLine, 0.28f));
    const float w = 640, h = 316;
    const Rectangle card{(a.width - w) / 2, (a.height - h) / 2, w, h};
    outlinedRect(card, 12, kWhite);
    float x = card.x + 28, y = card.y + 22;
    text("Rush Hour Café", x, y, 24, kLine);
    y += 38;
    const char* story[] = {
        "It's lunch rush. Robot waiters carry orders from the kitchen to hungry tables.",
        "The hard part isn't one robot; it's dozens sharing narrow aisles without",
        "colliding or getting stuck. Left and right run the same lunch with two planners.",
    };
    for (const char* line : story) {
        text(line, x, y, 13, kLine);
        y += 19;
    }
    y += 14;
    drawLegend(card.x + (w - drawLegend(0, 0, true, false)) / 2 + 6, y + 6, true);
    y += 30;
    const char* planners[][2] = {
        {"Local:", "each robot just avoids its neighbours"},
        {"Space-time A*:", "robots book their path in time, one by one"},
        {"CBS:", "robots are planned together, resolving conflicts"},
    };
    for (const auto& p : planners) {
        text(p[0], x, y, 13, kCounter);
        text(p[1], x + 104, y, 13, kLine);
        y += 20;
    }
    const std::string start = "Start";
    const float bw = measure(start, 14) + 36;
    const Rectangle btn{card.x + w - 28 - bw, card.y + h - 50, bw, 28};
    DrawRectangleRounded({btn.x - 1.8f, btn.y - 1.8f, btn.width + 3.6f, btn.height + 4.8f}, 0.35f, 8, kLine);
    DrawRectangleRounded(btn, 0.32f, 8, kRobot);
    text(start, btn.x + (bw - measure(start, 14)) / 2, btn.y + 6, 14, kLine);
    text("Try switching planners while it runs; the tint shows where robots wait.", x, btn.y + 7, 12, kLineSoft);
}

// Given: one side. Approach: floor, heatmap tint, counter, tables, plants,
// paths, robots, then bubbles and delivery pops on top.
void drawCafe(const App& a, int sideIndex, std::pair<int, int> hovered) {
    const Side& s = a.sides[sideIndex];
    const ww::Grid& g = s.sim->grid();
    const float cell = gCell;
    auto at = [&](Cell c) { return cellCentre(s, float(c.x), float(c.y)); };
    const std::vector<ww::RobotView> robots = s.sim->robots();
    const auto cells = static_cast<std::size_t>(g.width() * g.height());

    // Floor: a faint checker inside a 2 px frame; walls are just page.
    for (int y = 0; y < g.height(); ++y)
        for (int x = 0; x < g.width(); ++x)
            if (g.at({x, y}) != ww::Tile::Wall)
                DrawRectangleRec({s.originX + x * cell, s.originY + y * cell, cell, cell}, (x + y) % 2 ? kFloorB : kFloorA);
    // Congestion: where robots have been waiting lately, tinted coral.
    if (a.heatmap) {
        for (std::size_t i = 0; i < cells; ++i) {
            if (s.heat[i] < 0.05f) continue;
            const Cell c = g.cellAt(static_cast<int>(i));
            const float k = std::min(1.0f, s.heat[i] / kHeatFull);
            DrawRectangleRec({s.originX + c.x * cell, s.originY + c.y * cell, cell, cell},
                             withAlpha(kCounter, kHeatMaxAlpha * k));
        }
    }
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            if (g.at({x, y}) == ww::Tile::Wall) continue;
            const float px = s.originX + x * cell, py = s.originY + y * cell;
            auto wallAt = [&](int dx, int dy) {
                const Cell n{x + dx, y + dy};
                return !g.inBounds(n) || g.at(n) == ww::Tile::Wall;
            };
            if (wallAt(0, -1)) DrawRectangleRec({px, py - kFrame, cell, kFrame}, kLine);
            if (wallAt(0, 1)) DrawRectangleRec({px, py + cell, cell, kFrame}, kLine);
            if (wallAt(-1, 0)) DrawRectangleRec({px - kFrame, py - kFrame, kFrame, cell + 2 * kFrame}, kLine);
            if (wallAt(1, 0)) DrawRectangleRec({px + cell, py - kFrame, kFrame, cell + 2 * kFrame}, kLine);
        }
    }
    for (Cell f : g.pickFaces()) {
        Cell toward{0, -1};
        for (Cell d : ww::kNeighbors) {
            const Cell n{f.x + d.x, f.y + d.y};
            if (g.inBounds(n) && g.at(n) == ww::Tile::Shelf) {
                toward = d;
                break;
            }
        }
        drawMat(at(f), cell * 0.72f, toward);
    }
    for (const Rectangle& r : s.shelves) {
        outlinedRect(r, cell * 0.16f, kCounter);
        const std::string label = "Kitchen";
        const float size = std::max(10.0f, cell * 0.4f);
        if (r.width > measure(label, size) + 16)
            text(label, r.x + (r.width - measure(label, size)) / 2, r.y + (r.height - size) / 2 - 1, size, kWhite);
    }

    // Orders waiting per table.
    std::vector<int> orders(cells, 0);
    for (const ww::RobotView& r : robots) ++orders[g.index(tableOf(g, r.dropoff))];
    for (std::size_t i = 0; i < cells; ++i) {
        const Cell c = g.cellAt(static_cast<int>(i));
        const ww::Tile t = g.at(c);
        if (t == ww::Tile::Plant) drawPlant(at(c), cell * 0.55f);
        if (t == ww::Tile::Table || t == ww::Tile::Station) drawTable(at(c));
    }

    // Flow: a faint dotted line from every carrying robot to its table
    // (planned cells while there are any, then straight). Hover, a click
    // or T shows any robot's full planned path to its current goal.
    const std::vector<Vector2> pos = robotPositions(a, s);
    auto pathTo = [&](const ww::RobotView& r, Cell target) {
        std::vector<Vector2> pts{pos[static_cast<std::size_t>(r.id)]};
        bool reached = r.cell == target;
        for (std::size_t j = 1; j < r.route.size() && !reached; ++j) {
            pts.push_back(at(r.route[j]));
            reached = r.route[j] == target;
        }
        if (!reached) pts.push_back(at(target));
        return pts;
    };
    // A carrying robot's line goes to its drop-off cell and then ends on the
    // edge of the table it serves, facing the robot, under the bubble.
    auto toTable = [&](const ww::RobotView& r) {
        std::vector<Vector2> pts = pathTo(r, r.dropoff);
        const Cell table = tableOf(g, r.dropoff);
        if (!(table == r.dropoff)) {
            const Vector2 t = at(table), from = pts.back();
            const float dx = from.x - t.x, dy = from.y - t.y, len = std::max(1.0f, std::hypot(dx, dy));
            const float rim = cell * 0.3f + 3.0f;
            pts.push_back({t.x + dx / len * rim, t.y + dy / len * rim});
        }
        return pts;
    };
    auto isPinned = [&](int id) { return a.selectedSide == sideIndex && a.selectedId == id; };
    auto isHovered = [&](int id) { return hovered.first == sideIndex && hovered.second == id; };
    for (const ww::RobotView& r : robots) {
        if (a.showAllPaths || isHovered(r.id) || isPinned(r.id))
            drawDottedPath(r.carrying ? toTable(r) : pathTo(r, r.goal), withAlpha(kLine, 0.7f));
        else if (r.carrying) drawDottedPath(toTable(r), withAlpha(kLine, 0.35f));
    }
    for (const ww::RobotView& r : robots) {
        const auto id = static_cast<std::size_t>(r.id);
        if (isPinned(r.id))
            DrawRectangleRoundedLinesEx({pos[id].x - cell * 0.38f, pos[id].y - cell * 0.38f, cell * 0.76f, cell * 0.76f},
                                        0.4f, 8, 1.0f, withAlpha(kLine, 0.5f));
        drawRobot(pos[id], r.carrying, id < s.facing.size() ? s.facing[id] : Cell{1, 0}, cell * 0.5f);
    }

    // Bubbles over tables with orders; on delivery the bubble pops away and
    // "+1" floats up (newest delivery per table).
    std::vector<double> newest(cells, -1.0);
    for (const auto& [dropoff, t0] : s.pops) newest[g.index(tableOf(g, dropoff))] = t0;
    for (std::size_t i = 0; i < cells; ++i) {
        const Vector2 c = at(g.cellAt(static_cast<int>(i)));
        if (orders[i] > 0) drawBubble(c, cell, 1.0f, 1.0f, orders[i]);
        if (newest[i] < 0) continue;
        const float k = static_cast<float>((a.runTime - newest[i]) / kPopSeconds);
        if (k < 0.4f && orders[i] == 0) drawBubble(c, cell, 1.0f + k * 0.8f, 1.0f - k / 0.4f, 0);
        const float size = std::max(10.0f, cell * 0.38f);
        text("+1", c.x + cell * 0.32f, c.y - cell * 0.55f - k * cell * 0.45f, size, withAlpha(kFood, 1.0f - k));
    }
}

// Small white card with grey text next to the hovered robot.
void drawHoverCard(const App& a, std::pair<int, int> hovered) {
    if (hovered.first < 0) return;
    const Side& s = a.sides[hovered.first];
    const std::string line = robotCard(s, hovered.second);
    if (line.empty()) return;
    const Vector2 p = robotPositions(a, s)[static_cast<std::size_t>(hovered.second)];
    const float w = measure(line, 11) + 14, h = 20;
    const float x = std::clamp(p.x + gCell * 0.5f, 4.0f, a.width - w - 4.0f);
    const float y = std::clamp(p.y - gCell * 0.5f - h, 4.0f, a.height - h - 4.0f);
    outlinedRect({x, y, w, h}, 6, kWhite, 1.0f);
    text(line, x + 7, y + 3.5f, 11, kLineSoft);
}

// Given: current state. Approach: draw the whole window in window
// coordinates; the caller decides the target (screen, or an offscreen
// texture for recording).
void drawScene(const App& a) {
    gCell = a.cell;
    ClearBackground(kPage);
    const std::pair<int, int> hovered = a.showIntro ? std::pair{-1, -1} : robotAt(a, GetMousePosition());
    for (int side = 0; side < static_cast<int>(a.sides.size()); ++side) drawCafe(a, side, hovered);

    // Top: "Lunch rush" panel on the left (shared clock).
    const float rush = static_cast<float>(std::fmod(a.runTime, kRushSeconds));
    const Rectangle panel{kMargin, 9, 250, 28};
    outlinedRect(panel, 8, kWhite);
    text("Lunch rush", panel.x + 10, panel.y + 6, 13, kLine);
    const Rectangle bar{panel.x + 92, panel.y + 12.5f, 104, 3};
    DrawRectangleRounded(bar, 1.0f, 6, kFloorB);
    DrawRectangleRounded({bar.x, bar.y, bar.width * rush / static_cast<float>(kRushSeconds), bar.height}, 1.0f, 6, kCounter);
    char clock[16];
    std::snprintf(clock, sizeof clock, "%d:%02d", static_cast<int>(rush) / 60, static_cast<int>(rush) % 60);
    text(clock, panel.x + panel.width - 10 - measure(clock, 13), panel.y + 6, 13, kLine);

    // Served badge: one per side, at the right end of its header (split) or
    // the top right (single), with orders/min and stuck beside it.
    for (int side = 0; side < static_cast<int>(a.sides.size()); ++side) {
        const Side& s = a.sides[side];
        char served[48];
        std::snprintf(served, sizeof served, "%lld served", s.sim->stats().deliveries);
        const float size = a.split ? 12.0f : 13.0f, bh = a.split ? 22.0f : 28.0f;
        const float bw = measure(served, size) + (a.split ? 34 : 40);
        const float right = a.split ? s.originX + gridPxW(a) : static_cast<float>(a.width - kMargin);
        const Rectangle badge{right - bw, a.split ? kHudH + 3.0f : 9.0f, bw, bh};
        outlinedRect(badge, bh / 2, kBadge);
        drawStar({badge.x + bh / 2 + 1, badge.y + bh / 2}, bh * 0.23f, kLine);
        drawStar({badge.x + bh / 2 + 1, badge.y + bh / 2}, bh * 0.18f, kCounter);
        text(served, badge.x + bh + 2, badge.y + (bh - size) / 2 - 1, size, kLine);
        char rate[48];
        if (a.split)
            std::snprintf(rate, sizeof rate, "%.0f/min · %d stuck", deliveriesPerMinute(s), stuckCount(s));
        else
            std::snprintf(rate, sizeof rate, "%.0f / min", deliveriesPerMinute(s));
        text(rate, badge.x - 10 - measure(rate, 12), badge.y + (bh - 12) / 2 - 1, 12, kLineSoft);
    }
    if (a.paused) text("paused", panel.x + panel.width + 12, panel.y + 7, 12, kLineSoft);

    drawLegend(kMargin + 8, gridTop(a) + gridPxH(a) + kLegendH / 2.0f + 2, a.heatmap);

    // Bottom pills and the planner keys (bottom right, or in side headers).
    const std::vector<Button> buttons = layoutButtons(a);
    for (int gidx = 0;; ++gidx) {
        Rectangle u{};
        bool any = false, active = false;
        for (const Button& b : buttons) {
            if (b.group != gidx) continue;
            u = any ? Rectangle{u.x, u.y, b.rect.x + b.rect.width - u.x, u.height} : b.rect;
            any = true;
            active = active || b.active;
        }
        if (!any) break;
        drawPill(u, active ? kBadge : kWhite);
    }
    const Vector2 mouse = GetMousePosition();
    for (const Button& b : buttons) {
        const float size = b.rect.height < 24 ? 12.0f : kButtonText;
        const float tw = measure(b.label, size);
        const float tx = b.rect.x + (b.rect.width - tw) / 2, ty = b.rect.y + (b.rect.height - size) / 2 - 1;
        const bool hover = b.action != Action::None && b.enabled && CheckCollisionPointRec(mouse, b.rect);
        if (b.group < 0) {  // planner key: 1.8 px outline, a thicker bottom edge
            const Rectangle r = b.rect;
            DrawRectangleRounded({r.x - 1.8f, r.y - 1.8f, r.width + 3.6f, r.height + 4.8f}, 0.35f, 8, kLine);
            DrawRectangleRounded(r, 0.32f, 8, b.active ? kRobot : hover ? kFloorB : kWhite);
        }
        const Color c = !b.enabled ? withAlpha(kLine, 0.3f) : (b.action == Action::None ? kLineSoft : kLine);
        text(b.label, tx, ty, size, hover && b.group >= 0 ? kCounter : c);
    }
    if (a.split) {
        const std::string keys = "S single view · H heatmap · P right planner · T paths";
        text(keys, a.width - kMargin - measure(keys, 12), a.height - kFooterH + 14, 12, kLineSoft);
    }
    drawHoverCard(a, hovered);
    if (a.showIntro) drawIntro(a);
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
// GIF colour handling. msf_gif lowers the bit depth of a whole frame until it
// has under 256 colours, then dithers; antialiased edges push it so low that
// the cream floor turns pink and blotchy. So frames are quantized here first,
// to one palette for the whole GIF, and msf_gif keeps 16-bit colour.
struct GifPalette {
    std::vector<Color> colors;
    std::unordered_map<std::uint32_t, std::uint8_t> nearest;  // rgb -> palette index
};

std::uint32_t rgbKey(const unsigned char* p) { return (p[0] << 16) | (p[1] << 8) | p[2]; }

// Given: colours with pixel counts. Approach: median cut: split the box with
// the widest channel range at its weighted median until there are n boxes;
// each box becomes its count-weighted mean colour.
std::vector<Color> medianCut(std::vector<std::pair<std::uint32_t, int>> hist, int n) {
    using Entry = std::pair<std::uint32_t, int>;
    auto channel = [](std::uint32_t c, int ch) { return static_cast<int>((c >> (16 - 8 * ch)) & 255); };
    std::vector<std::vector<Entry>> boxes{std::move(hist)};
    auto widest = [&](const std::vector<Entry>& box, int& which) {
        int best = -1;
        for (int ch = 0; ch < 3; ++ch) {
            int lo = 255, hi = 0;
            for (const Entry& e : box) lo = std::min(lo, channel(e.first, ch)), hi = std::max(hi, channel(e.first, ch));
            if (hi - lo > best) best = hi - lo, which = ch;
        }
        return best;
    };
    while (static_cast<int>(boxes.size()) < n) {
        int pick = -1, range = 0, ch = 0;
        for (std::size_t b = 0; b < boxes.size(); ++b) {
            int c = 0;
            const int r = boxes[b].size() > 1 ? widest(boxes[b], c) : 0;
            if (r > range) range = r, pick = static_cast<int>(b), ch = c;
        }
        if (pick < 0) break;
        std::vector<Entry>& box = boxes[pick];
        std::sort(box.begin(), box.end(), [&](const Entry& x, const Entry& y) { return channel(x.first, ch) < channel(y.first, ch); });
        long long total = 0, run = 0;
        for (const Entry& e : box) total += e.second;
        std::size_t cut = 1;
        for (; cut < box.size() - 1; ++cut) {
            run += box[cut - 1].second;
            if (run * 2 >= total) break;
        }
        boxes.emplace_back(box.begin() + static_cast<std::ptrdiff_t>(cut), box.end());
        boxes[pick].resize(cut);
    }
    std::vector<Color> out;
    for (const auto& box : boxes) {
        double r = 0, g = 0, b = 0, w = 0;
        for (const Entry& e : box) {
            r += channel(e.first, 0) * double(e.second), g += channel(e.first, 1) * double(e.second);
            b += channel(e.first, 2) * double(e.second), w += e.second;
        }
        if (w > 0) out.push_back({static_cast<unsigned char>(r / w + 0.5), static_cast<unsigned char>(g / w + 0.5),
                                  static_cast<unsigned char>(b / w + 0.5), 255});
    }
    return out;
}

// Given: the first frame. Approach: the theme's flat colours (and heatmap
// tints over both floor shades) exactly, plus a median cut of the frame's
// own colours, ~128 in all, leaving room for msf_gif's light dithering.
GifPalette buildPalette(const Image& frame) {
    GifPalette p;
    const Color flat[] = {kPage, kFloorA, kFloorB, kLine, kLineSoft, kCounter, kMat, kWhite,
                          kRim, kLeafA, kLeafB, kPot, kRobot, kCup, kBadge};
    p.colors.assign(std::begin(flat), std::end(flat));
    for (Color floor : {kFloorA, kFloorB})
        for (float k : {0.25f, 0.5f, 0.75f, 1.0f}) {
            const float al = kHeatMaxAlpha * k;
            p.colors.push_back({static_cast<unsigned char>(floor.r + (kCounter.r - floor.r) * al),
                                static_cast<unsigned char>(floor.g + (kCounter.g - floor.g) * al),
                                static_cast<unsigned char>(floor.b + (kCounter.b - floor.b) * al), 255});
        }
    std::unordered_map<std::uint32_t, int> counts;
    const auto* px = static_cast<const unsigned char*>(frame.data);
    for (int i = 0; i < frame.width * frame.height; ++i) ++counts[rgbKey(px + 4 * i)];
    std::vector<std::pair<std::uint32_t, int>> hist(counts.begin(), counts.end());
    std::sort(hist.begin(), hist.end());  // deterministic order
    for (Color c : medianCut(std::move(hist), 128 - static_cast<int>(p.colors.size()))) p.colors.push_back(c);
    return p;
}

void quantize(Image& img, GifPalette& p) {
    auto* px = static_cast<unsigned char*>(img.data);
    for (int i = 0; i < img.width * img.height; ++i) {
        unsigned char* c = px + 4 * i;
        const std::uint32_t key = rgbKey(c);
        auto it = p.nearest.find(key);
        if (it == p.nearest.end()) {
            int best = 0, bestD = 1 << 30;
            for (std::size_t j = 0; j < p.colors.size(); ++j) {
                const int dr = c[0] - p.colors[j].r, dg = c[1] - p.colors[j].g, db = c[2] - p.colors[j].b;
                const int d = 3 * dr * dr + 4 * dg * dg + 2 * db * db;
                if (d < bestD) bestD = d, best = static_cast<int>(j);
            }
            it = p.nearest.emplace(key, static_cast<std::uint8_t>(best)).first;
        }
        const Color q = p.colors[it->second];
        c[0] = q.r, c[1] = q.g, c[2] = q.b, c[3] = 255;
    }
}

// Given: an open window. Approach: warm up so aisles are busy, then render
// fixed 1/fps frames into a 2x offscreen texture, downscale (cheap
// antialiasing), quantize to one palette and feed each frame to raylib's
// bundled GIF encoder.
int record(App& a) {
    for (int i = 0; i < 120; ++i) update(a, 1.0 / hz(a));  // one tick each; builds up the heatmap too
    constexpr int kSuper = 2;
    RenderTexture2D target = LoadRenderTexture(a.width * kSuper, a.height * kSuper);
    Camera2D camera{};
    camera.zoom = kSuper;
    MsfGifState gif{};
    GifPalette palette;
    bool ok = msf_gif_begin(&gif, a.width, a.height);
    for (int f = 0; ok && f < a.opt.recordFrames && !WindowShouldClose(); ++f) {
        update(a, 1.0 / a.opt.recordFps);
        BeginTextureMode(target);
        BeginMode2D(camera);
        drawScene(a);
        EndMode2D();
        EndTextureMode();

        BeginDrawing();  // mirror to the window so progress is visible
        ClearBackground(kPage);
        DrawTexturePro(target.texture, {0, 0, float(target.texture.width), -float(target.texture.height)},
                       {0, 0, float(a.width), float(a.height)}, {0, 0}, 0, WHITE);
        EndDrawing();

        Image img = LoadImageFromTexture(target.texture);
        ImageFlipVertical(&img);  // GL textures are bottom-up
        ImageResize(&img, a.width, a.height);
        ImageFormat(&img, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        if (f == 0) palette = buildPalette(img);
        quantize(img, palette);
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
                 "              [--left P] [--right P] [--single [--planner P]]   P: local|stastar|cbs\n"
                 "              [--no-intro] [--record OUT.gif [--frames N] [--fps N]]\n";
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
        auto planner = [&]() {
            auto p = ww::parsePlanner(next());
            if (!p) usage(1);
            return *p;
        };
        try {
            if (arg == "--robots") o.robots = std::stoi(next());
            else if (arg == "--seed") o.seed = std::stoull(next());
            else if (arg == "--hz") {
                const double want = std::stod(next());
                auto it = std::find(std::begin(kHzSteps), std::end(kHzSteps), want);
                if (it == std::end(kHzSteps)) usage(1);
                o.hzIndex = static_cast<int>(it - std::begin(kHzSteps));
            } else if (arg == "--left") o.planners[0] = planner();
            else if (arg == "--right") o.planners[1] = planner();
            else if (arg == "--planner") o.planners[0] = planner();  // the single view's planner
            else if (arg == "--single") o.split = false;
            else if (arg == "--map") o.mapPath = next();
            else if (arg == "--record") o.recordPath = next();
            else if (arg == "--frames") o.recordFrames = std::stoi(next());
            else if (arg == "--fps") o.recordFps = std::clamp(std::stoi(next()), 1, 50);
            else if (arg == "--no-intro") o.intro = false;
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
    a.split = a.opt.split;
    a.heatmap = a.split;  // on by default in split-screen
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
    a.showIntro = a.opt.intro && a.opt.recordPath.empty();

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
