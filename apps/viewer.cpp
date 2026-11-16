// viewer: Rush Hour Café, the graphical view of the robot world (raylib).
//
//   ./viewer                              café map, 16 robots
//   ./viewer --map warehouse              the benchmark warehouse map
//   ./viewer --robots 24 --hz 15 --seed 3 --planner stastar
//   ./viewer --record docs/viewer.gif     write a short GIF and exit
//
// Keys: space pause, +/- speed, up/down robot count, P cycle planner,
// T show all planned paths. Every control is also clickable (mouse or
// touch); hover or click a robot to see its planned path. An intro card
// explains the scene on launch (skip with --no-intro); "i" reopens it.

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
constexpr int kHudH = 46;      // lunch-rush panel and served badge
constexpr int kLegendH = 26;   // legend row under the floor
constexpr int kFooterH = 42;   // pills and planner buttons
constexpr int kMinWidth = 720;  // room for the control bar on small maps
constexpr double kHzSteps[] = {2, 4, 8, 15, 30, 60, 120};
constexpr double kRateWindow = 10.0;   // seconds of running time for orders/min
constexpr double kPopSeconds = 0.8;    // delivery: bubble pops, "+1" floats up
constexpr double kRushSeconds = 120.0;  // lunch-rush timer; loops until the game loop exists
constexpr int kStuckTicks = 3;          // a robot counts as stuck after this many blocked ticks

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

// The UI font, loaded once after the window opens.
Font gFont{};

// Given: an open window. Approach: rasterise the embedded TTF once at a size
// big enough for the 2x GIF render, with bilinear filtering for the smaller
// sizes we draw. ASCII plus the é in "Café".
void loadUiFont() {
    std::vector<int> codepoints;
    for (int c = 32; c < 127; ++c) codepoints.push_back(c);
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
    ww::Planner planner = ww::Planner::Local;
    std::string mapPath;
    std::string recordPath;
    int recordFrames = 240;
    int recordFps = 30;
    bool intro = true;
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
    bool showIntro = false;  // the intro card is up; the run waits behind it
    int selected = -1;  // robot id whose path stays visible, -1 for none
    double runTime = 0.0;  // seconds spent unpaused
    std::deque<std::pair<double, long long>> rateSamples;
    std::vector<std::pair<Cell, double>> pops;  // delivered-at drop-off cell, runTime
    int width = 0, height = 0;
    int gridX = kMargin;  // left edge of the grid (centred when the window is wider)
    int gridH = 0;        // grid height in pixels
    int prevTouches = 0;
};

double hz(const App& a) { return kHzSteps[a.opt.hzIndex]; }

// Given: the grid. Approach: flood-fill each counter block and keep its
// bounding box, so a block draws as one counter instead of separate cells.
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
        const float inset = 4.0f;
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

enum class Action { None, Pause, Slower, Faster, FewerRobots, MoreRobots, Planner, Info };

// A clickable (or label-only, Action::None) item. Items sharing a group
// sit inside one pill; planner buttons have group -1 and draw as keys.
struct Button {
    Rectangle rect;
    std::string label;
    Action action;
    int planner = -1;     // index into kPlanners for Action::Planner
    bool active = false;  // current planner
    bool enabled = true;
    int group = -1;
};

constexpr float kButtonText = 13;

int maxRobots(const App& a) { return static_cast<int>(a.sim->grid().floorCells().size()); }

int stuckCount(const App& a) {
    int n = 0;
    for (const ww::RobotView& r : a.sim->robots()) n += r.waitTicks >= kStuckTicks;
    return n;
}

// Given: app state. Approach: one layout used for both drawing and hit
// testing, so what you see is what you click. Left: pills for pause,
// speed, stuck count and robot count; right: planner buttons.
std::vector<Button> layoutButtons(const App& a) {
    std::vector<Button> out;
    const float y = static_cast<float>(a.height - kFooterH + 8), h = 26;
    float x = kMargin;
    int group = 0;
    auto add = [&](std::string label, Action act, bool enabled = true, float pad = 10) {
        const float w = measure(label, kButtonText) + 2 * pad;
        out.push_back({{x, y, w, h}, std::move(label), act, -1, false, enabled, group});
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
    nextPill();
    std::snprintf(value, sizeof value, "%d stuck", stuckCount(a));
    add(value, Action::None, true, 12);
    nextPill();
    add("-", Action::FewerRobots, a.opt.robots > 1, 9);
    std::snprintf(value, sizeof value, "%d robots", a.opt.robots);
    add(value, Action::None, true, 2);
    add("+", Action::MoreRobots, a.opt.robots < maxRobots(a), 9);
    nextPill();
    add("i", Action::Info, true, 10);

    // Planner buttons, right-aligned.
    x = static_cast<float>(a.width - kMargin);
    for (int i = static_cast<int>(std::size(kPlanners)) - 1; i >= 0; --i) {
        const float w = measure(kPlanners[i].label, kButtonText) + 22;
        x -= w;
        out.push_back({{x, y, w, h}, kPlanners[i].label, Action::Planner, i,
                       kPlanners[i].planner == a.opt.planner, true, -1});
        x -= 6;
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
        case Action::Info: a.showIntro = true; break;
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
    if (a.showIntro && (IsKeyPressed(KEY_SPACE) || IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER))) {
        a.showIntro = false;
        return;
    }
    if (a.showIntro) {  // only a click (below) closes the card; other keys wait
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || (GetTouchPointCount() > 0 && a.prevTouches == 0))
            clickAt(a, GetMousePosition());
        a.prevTouches = GetTouchPointCount();
        return;
    }
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
// which way each robot moved, and catch deliveries for the pop.
void update(App& a, double frameSeconds) {
    if (a.paused || a.showIntro) return;
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
    const float r = kCell * 0.3f;  // ~60% of a cell across
    outlinedCircle(c, r, kWhite);
    for (int i = 0; i < 12; ++i) {  // faint dashed rim
        const float a0 = i * 30.0f;
        DrawRing(c, r * 0.72f, r * 0.72f + 1.0f, a0, a0 + 16.0f, 4, kRim);
    }
}

// A thin yellow mat along one edge of a cell: `toward` is the side the
// counter is on, so the mat hugs the counter.
void drawMat(Vector2 c, float length, Cell toward) {
    const float thick = std::max(3.0f, length * 0.2f), off = kCell * 0.5f - thick / 2 - 2.5f;
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

// Small speech bubble with a cup, ~55% of a cell wide; `scale` < 1 while popping.
void drawBubble(Vector2 table, float scale, float alpha, int orders) {
    const float w = kCell * 0.55f * scale, h = kCell * 0.42f * scale;
    const Vector2 c{table.x, table.y - kCell * 0.62f};
    const Rectangle r{c.x - w / 2, c.y - h / 2, w, h};
    const Color line = withAlpha(kLine, alpha), fill = withAlpha(kWhite, alpha);
    DrawTriangle({c.x - 3.5f * scale, r.y + h - 0.5f}, {c.x, r.y + h + 4.5f * scale}, {c.x + 3.5f * scale, r.y + h - 0.5f},
                 line);
    DrawRectangleRounded({r.x - kEdge, r.y - kEdge, w + 2 * kEdge, h + 2 * kEdge}, 0.6f, 8, line);
    DrawRectangleRounded(r, 0.6f, 8, fill);
    DrawTriangle({c.x - 2.2f * scale, r.y + h - 1}, {c.x, r.y + h + 2.6f * scale}, {c.x + 2.2f * scale, r.y + h - 1},
                 fill);
    if (alpha >= 1.0f) drawCup({c.x - h * 0.08f, c.y + 0.5f}, h * 0.5f);
    if (orders > 1 && alpha >= 1.0f) {
        const Vector2 b{r.x + w, r.y};
        DrawCircleV(b, 5.0f, kFood);
        const std::string n = std::to_string(orders);
        text(n, b.x - measure(n, 9) / 2, b.y - 5.5f, 9, kWhite);
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
    constexpr float kGap = 5.0f;
    float carry = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const Vector2 a = pts[i - 1], b = pts[i];
        const float len = std::hypot(b.x - a.x, b.y - a.y);
        for (float d = kGap - carry; d < len; d += kGap)
            DrawCircleV({a.x + (b.x - a.x) * d / len, a.y + (b.y - a.y) * d / len}, 1.0f, color);
        carry = std::fmod(carry + len, kGap);
    }
}

void drawPill(Rectangle r) {
    DrawRectangleRounded({r.x - kEdge, r.y - kEdge, r.width + 2 * kEdge, r.height + 2 * kEdge}, 1.0f, 12, kLine);
    DrawRectangleRounded(r, 1.0f, 12, kWhite);
}

// Legend: tiny drawn icons with small brown labels on one line. Returns
// the width it takes; with draw = false it only measures.
float drawLegend(float x, float y, bool draw = true) {
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
    item([&](Vector2 c) { drawBubble({c.x, c.y + kCell * 0.62f}, 0.8f, 1.0f, 0); }, "order waiting");
    item([&](Vector2 c) {
        DrawRectangleRounded({c.x - 6, c.y - 6, 12, 2.5f}, 1.0f, 4, kCounter);
        DrawRectangleRounded({c.x - 5.5f, c.y - 1.5f, 11, 3.5f}, 1.0f, 4, kMat);
    }, "pickup spot");
    item([&](Vector2 c) { drawPlant({c.x, c.y + 1}, 14); }, "plant (obstacle)");
    return x - 22 - start;
}

// Given: the scene drawn behind. Approach: dim it, then a white card with
// the title, what's going on, the legend, one line per planner and a Start
// button. Any click, Space or Enter starts (see handleInput / clickAt).
void drawIntro(const App& a) {
    DrawRectangle(0, 0, a.width, a.height, withAlpha(kLine, 0.28f));
    const float w = 560, h = 292;
    const Rectangle card{(a.width - w) / 2, (a.height - h) / 2, w, h};
    outlinedRect(card, 12, kWhite);
    float x = card.x + 28, y = card.y + 22;
    text("Rush Hour Caf\u00e9", x, y, 24, kLine);
    y += 38;
    const char* story[] = {
        "It's lunch rush. Robot waiters carry orders from the kitchen to hungry tables.",
        "The hard part isn't one robot; it's dozens sharing narrow aisles without",
        "colliding or getting stuck.",
    };
    for (const char* line : story) {
        text(line, x, y, 13, kLine);
        y += 19;
    }
    y += 14;
    drawLegend(card.x + (w - drawLegend(0, 0, false)) / 2 + 6, y + 6);
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
    text("Try switching planners while it runs.", x, btn.y + 7, 12, kLineSoft);
}

// Given: current state. Approach: draw the scene in window coordinates; the
// caller decides the target (screen, or an offscreen texture for recording).
void drawScene(const App& a) {
    const ww::Grid& g = a.sim->grid();
    ClearBackground(kPage);
    auto at = [&](Cell c) { return cellCentre(a, float(c.x), float(c.y)); };
    const std::vector<ww::RobotView> robots = a.sim->robots();
    const auto cells = static_cast<std::size_t>(g.width() * g.height());

    // Floor: a faint checker inside a 2 px frame; walls are just page.
    for (int y = 0; y < g.height(); ++y)
        for (int x = 0; x < g.width(); ++x)
            if (g.at({x, y}) != ww::Tile::Wall)
                DrawRectangle(a.gridX + x * kCell, kHudH + y * kCell, kCell, kCell, (x + y) % 2 ? kFloorB : kFloorA);
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            if (g.at({x, y}) == ww::Tile::Wall) continue;
            const float px = static_cast<float>(a.gridX + x * kCell), py = static_cast<float>(kHudH + y * kCell);
            auto wallAt = [&](int dx, int dy) {
                const Cell n{x + dx, y + dy};
                return !g.inBounds(n) || g.at(n) == ww::Tile::Wall;
            };
            if (wallAt(0, -1)) DrawRectangleRec({px, py - kFrame, kCell, kFrame}, kLine);
            if (wallAt(0, 1)) DrawRectangleRec({px, py + kCell, kCell, kFrame}, kLine);
            if (wallAt(-1, 0)) DrawRectangleRec({px - kFrame, py - kFrame, kFrame, kCell + 2 * kFrame}, kLine);
            if (wallAt(1, 0)) DrawRectangleRec({px + kCell, py - kFrame, kFrame, kCell + 2 * kFrame}, kLine);
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
        drawMat(at(f), kCell * 0.72f, toward);
    }
    for (const Rectangle& r : a.shelves) {
        outlinedRect(r, 5.0f, kCounter);
        const std::string label = "Kitchen";
        if (r.width > measure(label, 13) + 16)
            text(label, r.x + (r.width - measure(label, 13)) / 2, r.y + (r.height - 13) / 2 - 1, 13, kWhite);
    }

    // Orders waiting per table.
    std::vector<int> orders(cells, 0);
    for (const ww::RobotView& r : robots) ++orders[g.index(tableOf(g, r.dropoff))];
    for (std::size_t i = 0; i < cells; ++i) {
        const Cell c = g.cellAt(static_cast<int>(i));
        const ww::Tile t = g.at(c);
        if (t == ww::Tile::Plant) drawPlant(at(c), kCell * 0.55f);
        if (t == ww::Tile::Table || t == ww::Tile::Station) drawTable(at(c));
    }

    // Flow: a faint dotted line from every carrying robot to its table
    // (planned cells while there are any, then straight). Hover, a click
    // or T shows any robot's full planned path to its current goal.
    const std::vector<Vector2> pos = robotPositions(a);
    const int hovered = robotAt(a, GetMousePosition());
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
            const float rim = kCell * 0.3f + 3.0f;
            pts.push_back({t.x + dx / len * rim, t.y + dy / len * rim});
        }
        return pts;
    };
    for (const ww::RobotView& r : robots) {
        if (a.showAllPaths || r.id == hovered || r.id == a.selected)
            drawDottedPath(r.carrying ? toTable(r) : pathTo(r, r.goal), withAlpha(kLine, 0.7f));
        else if (r.carrying) drawDottedPath(toTable(r), withAlpha(kLine, 0.35f));
    }
    for (const ww::RobotView& r : robots) {
        const auto id = static_cast<std::size_t>(r.id);
        if (r.id == a.selected)
            DrawRectangleRoundedLinesEx({pos[id].x - kCell * 0.38f, pos[id].y - kCell * 0.38f, kCell * 0.76f, kCell * 0.76f},
                                        0.4f, 8, 1.0f, withAlpha(kLine, 0.5f));
        drawRobot(pos[id], r.carrying, id < a.facing.size() ? a.facing[id] : Cell{1, 0}, kCell * 0.5f);
    }

    // Bubbles over tables with orders; on delivery the bubble pops away and
    // "+1" floats up (newest delivery per table).
    std::vector<double> newest(cells, -1.0);
    for (const auto& [dropoff, t0] : a.pops) newest[g.index(tableOf(g, dropoff))] = t0;
    for (std::size_t i = 0; i < cells; ++i) {
        const Vector2 c = at(g.cellAt(static_cast<int>(i)));
        if (orders[i] > 0) drawBubble(c, 1.0f, 1.0f, orders[i]);
        if (newest[i] < 0) continue;
        const float k = static_cast<float>((a.runTime - newest[i]) / kPopSeconds);
        if (k < 0.4f && orders[i] == 0) drawBubble(c, 1.0f + k * 0.8f, 1.0f - k / 0.4f, 0);
        text("+1", c.x + kCell * 0.32f, c.y - kCell * 0.55f - k * 14, 12, withAlpha(kFood, 1.0f - k));
    }

    // Top: "Lunch rush" panel on the left, served badge on the right.
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

    char served[48];
    std::snprintf(served, sizeof served, "%lld served", a.sim->stats().deliveries);
    const float bw = measure(served, 13) + 40;
    const Rectangle badge{a.width - kMargin - bw, 9, bw, 28};
    outlinedRect(badge, 14, kBadge);
    drawStar({badge.x + 15, badge.y + 14}, 6.5f, kLine);
    drawStar({badge.x + 15, badge.y + 14}, 5.0f, kCounter);
    text(served, badge.x + 27, badge.y + 6, 13, kLine);
    char rate[32];
    std::snprintf(rate, sizeof rate, "%.0f / min%s", deliveriesPerMinute(a), a.paused ? "  (paused)" : "");
    text(rate, badge.x - 10 - measure(rate, 12), badge.y + 7, 12, kLineSoft);

    drawLegend(kMargin + 8, static_cast<float>(kHudH + a.gridH + kLegendH / 2 + 2));

    // Bottom: pills on the left, planner keys on the right.
    const std::vector<Button> buttons = layoutButtons(a);
    for (int gidx = 0;; ++gidx) {
        Rectangle u{};
        bool any = false;
        for (const Button& b : buttons) {
            if (b.group != gidx) continue;
            u = any ? Rectangle{u.x, u.y, b.rect.x + b.rect.width - u.x, u.height} : b.rect;
            any = true;
        }
        if (!any) break;
        drawPill(u);
    }
    const Vector2 mouse = GetMousePosition();
    for (const Button& b : buttons) {
        const float tw = measure(b.label, kButtonText);
        const float tx = b.rect.x + (b.rect.width - tw) / 2, ty = b.rect.y + (b.rect.height - kButtonText) / 2 - 1;
        const bool hover = b.action != Action::None && b.enabled && CheckCollisionPointRec(mouse, b.rect);
        if (b.group < 0) {  // planner key: 1.8 px outline, a thicker bottom edge
            const Rectangle r = b.rect;
            DrawRectangleRounded({r.x - 1.8f, r.y - 1.8f, r.width + 3.6f, r.height + 4.8f}, 0.35f, 8, kLine);
            DrawRectangleRounded(r, 0.32f, 8, b.active ? kRobot : hover ? kFloorB : kWhite);
        }
        const Color c = !b.enabled ? withAlpha(kLine, 0.3f) : (b.action == Action::None ? kLineSoft : kLine);
        text(b.label, tx, ty, kButtonText, hover && b.group >= 0 ? kCounter : c);
    }
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
        ClearBackground(kPage);
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
                 "              [--planner local|stastar|cbs] [--no-intro] [--record OUT.gif [--frames N] [--fps N]]\n";
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
    a.gridH = g.height() * kCell;
    a.height = kHudH + a.gridH + kLegendH + kFooterH;
    a.gridX = (a.width - g.width() * kCell) / 2;
    a.shelves = shelfBlocks(g, a.gridX);
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
