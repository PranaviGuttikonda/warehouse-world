// viewer: Rush Hour Café, the graphical view of the robot world (raylib).
//
//   ./viewer                              café map, 16 robots
//   ./viewer --map warehouse              the benchmark warehouse map
//   ./viewer --robots 24 --hz 15 --seed 3 --planner stastar
//   ./viewer --record docs/viewer.gif     write a short GIF and exit
//
// Keys: space pause, +/- speed, up/down robot count, P cycle planner.
// Every control is also a button in the bottom bar (mouse or touch).

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
constexpr int kFooterH = 48;
constexpr int kMinWidth = 720;  // room for the control bar on small maps
constexpr double kHzSteps[] = {2, 4, 8, 15, 30, 60, 120};
constexpr double kRateWindow = 10.0;  // seconds of running time for deliveries/min

// Café palette (docs/VISION.md).
const Color kOutline{59, 42, 32, 255};      // #3B2A20, chunky outlines and the frame
const Color kFloorA{255, 233, 199, 255};    // #FFE9C7
const Color kFloorB{255, 221, 176, 255};    // #FFDDB0
const Color kWall{122, 82, 56, 255};
const Color kCounter{232, 93, 74, 255};     // #E85D4A
const Color kCounterTop{242, 129, 111, 255};
const Color kTable{255, 255, 255, 255};
const Color kShadow{59, 42, 32, 40};
const Color kPlant{92, 193, 92, 255};       // #5CC15C
const Color kPlantDark{63, 160, 72, 255};
const Color kPot{200, 105, 62, 255};
const Color kRobot{79, 195, 247, 255};      // #4FC3F7
const Color kRobotShade{47, 168, 221, 255};
const Color kFood{232, 93, 74, 255};
const Color kFoodYellow{255, 201, 60, 255};
const Color kText{255, 233, 199, 255};      // cream on the dark frame
const Color kDimText{196, 160, 128, 255};
const Color kButton{255, 243, 224, 255};
const Color kButtonHover{255, 224, 178, 255};
const Color kButtonText{59, 42, 32, 255};
const Color kButtonDisabled{170, 140, 115, 255};
const Color kChair{185, 119, 74, 255};
const Color kTray{236, 239, 241, 255};
const Color kSteam{191, 165, 138, 255};
const Color kMat{244, 199, 161, 255};
const Color kStar{255, 201, 60, 255};
// Soft per-robot colours linking a carrying robot, its route and its table.
const Color kRouteColors[] = {{100, 181, 246, 255}, {186, 104, 200, 255}, {77, 182, 172, 255},
                              {255, 167, 77, 255},  {240, 98, 146, 255},  {149, 117, 205, 255}};
// Customer looks: shirt and skin, picked per table and seat.
const Color kShirts[] = {{255, 138, 101, 255}, {186, 104, 200, 255}, {129, 199, 132, 255}, {255, 213, 79, 255}};
const Color kSkins[] = {{241, 194, 125, 255}, {224, 172, 105, 255}, {141, 85, 36, 255}, {255, 219, 172, 255}};
constexpr double kPopSeconds = 0.9;  // delivery "+1" and star

// A robot counts as stuck after this many blocked ticks in a row.
constexpr int kStuckTicks = 3;

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
    std::vector<Rectangle> shelves;
    bool paused = false;
    double runTime = 0.0;  // seconds spent unpaused
    std::deque<std::pair<double, long long>> rateSamples;
    int width = 0, height = 0;
    int gridX = kMargin;  // left edge of the grid (centred when the window is wider)
    int prevTouches = 0;
    std::vector<std::pair<Cell, double>> pops;  // delivered-at table, runTime of delivery
};

double hz(const App& a) { return kHzSteps[a.opt.hzIndex]; }

// Given: the grid. Approach: flood-fill each shelf block and keep its
// bounding box, so a block draws as one rounded rect instead of beads.
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
    a.rateSamples.clear();
    a.pops.clear();
}

enum class Action { None, Pause, Slower, Faster, FewerRobots, MoreRobots, Planner };

struct Button {
    Rectangle rect;
    std::string label;
    Action action;
    int planner = -1;     // index into kPlanners for Action::Planner
    bool active = false;  // highlighted (current planner)
    bool enabled = true;
};

int maxRobots(const App& a) { return static_cast<int>(a.sim->grid().floorCells().size()); }

// Given: app state. Approach: one layout used for both drawing and hit
// testing, so what you see is what you click. Value labels are buttons
// with Action::None.
std::vector<Button> layoutButtons(const App& a) {
    std::vector<Button> out;
    const float y = static_cast<float>(a.height - kFooterH + 10), h = 28;
    float x = kMargin;
    auto add = [&](float w, std::string label, Action act, bool enabled = true) {
        out.push_back({{x, y, w, h}, std::move(label), act, -1, false, enabled});
        x += w;
    };
    const int last = static_cast<int>(std::size(kHzSteps)) - 1;
    char text[32];

    add(72, a.paused ? "Play" : "Pause", Action::Pause);
    x += 16;
    add(28, "-", Action::Slower, a.opt.hzIndex > 0);
    std::snprintf(text, sizeof text, "%g Hz", hz(a));
    add(64, text, Action::None);
    add(28, "+", Action::Faster, a.opt.hzIndex < last);
    x += 16;
    add(28, "-", Action::FewerRobots, a.opt.robots > 1);
    std::snprintf(text, sizeof text, "%d robots", a.opt.robots);
    add(96, text, Action::None);
    add(28, "+", Action::MoreRobots, a.opt.robots < maxRobots(a));

    // Planner buttons, right-aligned.
    x = static_cast<float>(a.width - kMargin);
    for (int i = static_cast<int>(std::size(kPlanners)) - 1; i >= 0; --i) {
        const float w = static_cast<float>(MeasureText(kPlanners[i].label, 16) + 24);
        x -= w;
        out.push_back({{x, y, w, h}, kPlanners[i].label, Action::Planner, i,
                       kPlanners[i].planner == a.opt.planner, true});
        x -= 8;
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

// Given: a click position. Approach: hit-test the same layout that is drawn.
void clickAt(App& a, Vector2 at) {
    for (const Button& b : layoutButtons(a)) {
        if (b.action != Action::None && b.enabled && CheckCollisionPointRec(at, b.rect)) {
            apply(a, b.action, b.planner);
            return;
        }
    }
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
// positions so drawing can lerp prev -> cur by FixedTimestep::alpha().
void update(App& a, double frameSeconds) {
    if (a.paused) return;
    a.runTime += frameSeconds;
    a.timestep.advance(frameSeconds, [&] {
        a.prev = a.cur;
        const std::vector<ww::RobotView> before = a.sim->robots();
        a.sim->step();
        snapshot(*a.sim, a.cur);
        // A robot that was carrying and now isn't just served its table.
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

// Given: a centre and radius. Approach: outline circle first, fill on top,
// so every shape gets the same chunky dark edge.
void outlinedCircle(Vector2 c, float r, Color fill, float edge = 2.5f) {
    DrawCircleV(c, r + edge, kOutline);
    DrawCircleV(c, r, fill);
}

void outlinedRect(Rectangle r, float roundness, Color fill, float edge = 2.5f) {
    DrawRectangleRounded({r.x - edge, r.y - edge, r.width + 2 * edge, r.height + 2 * edge}, roundness, 8, kOutline);
    DrawRectangleRounded(r, roundness, 8, fill);
}

void drawPlant(Vector2 c) {
    const float s = kCell;
    outlinedRect({c.x - s * 0.22f, c.y + s * 0.05f, s * 0.44f, s * 0.32f}, 0.3f, kPot, 2.0f);
    const Vector2 leaves[3] = {{c.x - s * 0.16f, c.y - s * 0.06f}, {c.x + s * 0.16f, c.y - s * 0.06f},
                               {c.x, c.y - s * 0.2f}};
    for (Vector2 l : leaves) DrawCircleV(l, s * 0.17f + 2.0f, kOutline);
    for (Vector2 l : leaves) DrawCircleV(l, s * 0.17f, kPlant);
    DrawCircleV({c.x + s * 0.05f, c.y - s * 0.24f}, s * 0.05f, kPlantDark);
}

void drawTable(Vector2 c) {
    DrawEllipse(static_cast<int>(c.x), static_cast<int>(c.y + kCell * 0.12f), kCell * 0.42f, kCell * 0.3f, kShadow);
    outlinedCircle(c, kCell * 0.36f, kTable);
    DrawCircleLinesV(c, kCell * 0.27f, Color{240, 226, 210, 255});
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

// Chairs around a table: 2-4 of them, fixed per table, on sides with floor.
// Sides: 0 north, 1 west, 2 east, 3 south (customers sit on N/W/E only, so
// the table never hides them).
std::vector<int> chairSides(const ww::Grid& g, Cell table) {
    static constexpr int order[4] = {1, 2, 0, 3};  // W, E, N, S
    static constexpr Cell dir[4] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};
    const int want = 2 + (table.x * 5 + table.y * 3) % 3;
    std::vector<int> sides;
    for (int side : order) {
        const Cell n{table.x + dir[side].x, table.y + dir[side].y};
        if (static_cast<int>(sides.size()) < want && g.passable(n)) sides.push_back(side);
    }
    return sides;
}

Vector2 sideOffset(int side, float dist) {
    static constexpr float dx[4] = {0, -1, 1, 0}, dy[4] = {-1, 0, 0, 1};
    return {dx[side] * dist, dy[side] * dist};
}

void drawChair(Vector2 table, int side) {
    const Vector2 o = sideOffset(side, kCell * 0.5f);
    const float s = kCell * 0.3f;
    outlinedRect({table.x + o.x - s / 2, table.y + o.y - s / 2, s, s}, 0.35f, kChair, 2.0f);
}

void drawCustomer(Vector2 table, int side, int look) {
    const Vector2 o = sideOffset(side, kCell * 0.52f);
    const Vector2 c{table.x + o.x, table.y + o.y};
    outlinedRect({c.x - kCell * 0.17f, c.y - kCell * 0.06f, kCell * 0.34f, kCell * 0.26f}, 0.6f,
                 kShirts[look % 4], 2.0f);
    outlinedCircle({c.x, c.y - kCell * 0.17f}, kCell * 0.14f, kSkins[look % 4], 2.0f);
}

// A bowl of food with steam: the dish everywhere (counter, bubble, tray).
void drawBowl(Vector2 c, float w, bool steam) {
    if (steam) {
        for (int i = -1; i <= 1; ++i) {
            const float x = c.x + i * w * 0.32f, y = c.y - w * 0.45f;
            DrawLineEx({x, y}, {x + 2, y - w * 0.22f}, 1.6f, kSteam);
            DrawLineEx({x + 2, y - w * 0.22f}, {x, y - w * 0.44f}, 1.6f, kSteam);
        }
    }
    DrawCircleV({c.x - w * 0.1f, c.y - w * 0.12f}, w * 0.2f, kFood);  // food peeking over the rim
    DrawCircleV({c.x + w * 0.16f, c.y - w * 0.1f}, w * 0.16f, kFoodYellow);
    DrawCircleSector(c, w * 0.5f + 2.0f, 0, 180, 16, kOutline);
    DrawCircleSector(c, w * 0.5f, 0, 180, 16, kTable);
    DrawRectangleRounded({c.x - w * 0.55f - 1, c.y - 2.5f, w * 1.1f + 2, 4}, 1.0f, 4, kOutline);
}

void drawOrderBubble(Vector2 table, int orders, Color edge) {
    const float w = kCell * 0.62f, h = kCell * 0.56f;
    const Rectangle r{table.x - w / 2, table.y - kCell * 1.12f - h / 2, w, h};
    const Vector2 tip{table.x, table.y - kCell * 0.62f};
    DrawTriangle({tip.x - 6, r.y + h - 1}, {tip.x, tip.y + 2.5f}, {tip.x + 6, r.y + h - 1}, edge);
    DrawRectangleRounded({r.x - 2.5f, r.y - 2.5f, r.width + 5, r.height + 5}, 0.5f, 8, edge);
    DrawRectangleRounded(r, 0.5f, 8, kTable);
    DrawTriangle({tip.x - 4, r.y + h - 2}, tip, {tip.x + 4, r.y + h - 2}, kTable);
    drawBowl({r.x + w / 2, r.y + h * 0.72f}, w * 0.55f, true);
    if (orders > 1) {  // count badge on the corner
        const Vector2 b{r.x + w + 1, r.y - 1};
        outlinedCircle(b, 6.5f, kCounter, 1.5f);
        char n[8];
        std::snprintf(n, sizeof n, "%d", orders);
        DrawText(n, static_cast<int>(b.x - MeasureText(n, 10) / 2.0f), static_cast<int>(b.y - 4.5f), 10, kTable);
    }
}

// Rounded blue body, eyes, smile, antenna. Carrying: a tray wider than the
// robot, held up with both arms, with a bowl on it.
void drawRobot(Vector2 c, bool carrying, Color tint) {
    const float r = kCell * 0.34f;
    DrawEllipse(static_cast<int>(c.x), static_cast<int>(c.y + r * 0.95f), r * 0.9f, r * 0.3f, kShadow);
    const Rectangle body{c.x - r, c.y - r * 0.9f, 2 * r, 1.9f * r};
    const Vector2 antennaTop{c.x - r * 0.45f, body.y - r * 0.55f};
    DrawLineEx({c.x - r * 0.3f, body.y + 1}, antennaTop, 4.5f, kOutline);
    DrawLineEx({c.x - r * 0.3f, body.y + 1}, antennaTop, 2.0f, kRobotShade);
    outlinedCircle(antennaTop, r * 0.17f, kCounter, 2.0f);
    outlinedRect(body, 0.7f, kRobot);
    DrawRectangleRounded({body.x + 3, body.y + body.height * 0.62f, body.width - 6, body.height * 0.3f}, 0.8f, 6,
                         kRobotShade);
    for (float side : {-1.0f, 1.0f}) {
        const Vector2 eye{c.x + side * r * 0.38f, c.y - r * 0.18f};
        DrawCircleV(eye, r * 0.16f, kOutline);
        DrawCircleV({eye.x + r * 0.05f, eye.y - r * 0.06f}, r * 0.06f, kTable);
    }
    DrawRing({c.x, c.y + r * 0.02f}, r * 0.3f, r * 0.3f + 2.0f, 25, 155, 12, kOutline);
    if (!carrying) return;
    const Vector2 tray{c.x, body.y - r * 0.75f};
    for (float side : {-1.0f, 1.0f}) {  // arms reaching out and up to the tray rim
        const Vector2 shoulder{c.x + side * r * 0.9f, c.y - r * 0.05f};
        const Vector2 elbow{c.x + side * r * 1.35f, c.y - r * 0.45f};
        const Vector2 hand{c.x + side * r * 1.2f, tray.y + 2};
        for (auto [from, to] : {std::pair{shoulder, elbow}, std::pair{elbow, hand}}) {
            DrawLineEx(from, to, 6.0f, kOutline);
            DrawLineEx(from, to, 3.0f, kRobot);
        }
    }
    DrawEllipse(static_cast<int>(tray.x), static_cast<int>(tray.y), r * 1.4f + 2.5f, r * 0.4f + 2.5f, kOutline);
    DrawEllipse(static_cast<int>(tray.x), static_cast<int>(tray.y), r * 1.4f, r * 0.4f, tint);
    DrawEllipse(static_cast<int>(tray.x), static_cast<int>(tray.y - 1), r * 1.18f, r * 0.29f, kTray);
    drawBowl({tray.x, tray.y - 1}, r * 0.95f, false);
}

// Dots every few pixels along a polyline; the phase drifts so it reads as
// "heading that way".
void drawDottedPath(const std::vector<Vector2>& pts, Color color, float phase) {
    constexpr float kGap = 7.0f;
    float carry = std::fmod(phase, kGap);
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const Vector2 a = pts[i - 1], b = pts[i];
        const float len = std::hypot(b.x - a.x, b.y - a.y);
        for (float d = kGap - carry; d < len; d += kGap)
            DrawCircleV({a.x + (b.x - a.x) * d / len, a.y + (b.y - a.y) * d / len}, 1.8f, color);
        carry = len <= 0 ? carry : std::fmod(carry + len, kGap);
    }
}

void drawStar(Vector2 c, float r, Color fill) {
    Vector2 pts[10];
    for (int i = 0; i < 10; ++i) {
        const float ang = -PI / 2 + i * PI / 5, rad = (i % 2) ? r * 0.45f : r;
        pts[i] = {c.x + std::cos(ang) * rad, c.y + std::sin(ang) * rad};
    }
    for (int i = 0; i < 10; ++i) DrawTriangle(c, pts[(i + 1) % 10], pts[i], fill);
}

// Given: current state. Approach: draw the scene in window coordinates; the
// caller decides the target (screen, or an offscreen texture for recording).
// Order: floor and walls, wall edges, pickup mats, counter and its dishes,
// chairs and customers, tables, plants, routes, robots, then bubbles and
// delivery pops on top so they're never hidden.
void drawScene(const App& a) {
    const ww::Grid& g = a.sim->grid();
    ClearBackground(kOutline);
    auto centre = [&](float x, float y) { return Vector2{a.gridX + (x + 0.5f) * kCell, kHudH + (y + 0.5f) * kCell}; };
    auto at = [&](Cell c) { return centre(float(c.x), float(c.y)); };
    const std::vector<ww::RobotView> robots = a.sim->robots();

    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            const int px = a.gridX + x * kCell, py = kHudH + y * kCell;
            const bool wall = g.at({x, y}) == ww::Tile::Wall;
            DrawRectangle(px, py, kCell, kCell, wall ? kWall : ((x + y) % 2 ? kFloorB : kFloorA));
        }
    }
    // Chunky edge wherever floor meets wall.
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            if (g.at({x, y}) == ww::Tile::Wall) continue;
            const int px = a.gridX + x * kCell, py = kHudH + y * kCell;
            auto wallAt = [&](int dx, int dy) {
                const Cell n{x + dx, y + dy};
                return !g.inBounds(n) || g.at(n) == ww::Tile::Wall;
            };
            if (wallAt(0, -1)) DrawRectangle(px, py, kCell, 3, kOutline);
            if (wallAt(0, 1)) DrawRectangle(px, py + kCell - 3, kCell, 3, kOutline);
            if (wallAt(-1, 0)) DrawRectangle(px, py, 3, kCell, kOutline);
            if (wallAt(1, 0)) DrawRectangle(px + kCell - 3, py, 3, kCell, kOutline);
        }
    }
    // Pickup mats under the counter.
    for (Cell f : g.pickFaces()) {
        const Vector2 c = at(f);
        DrawRectangleRounded({c.x - kCell * 0.36f, c.y - kCell * 0.3f, kCell * 0.72f, kCell * 0.6f}, 0.4f, 6, kMat);
    }
    for (const Rectangle& r : a.shelves) {
        outlinedRect(r, 0.25f, kCounter, 3.0f);
        DrawRectangleRounded({r.x + 3, r.y + 3, r.width - 6, std::min(r.height * 0.35f, 8.0f)}, 0.6f, 6, kCounterTop);
    }
    // A ready dish on the counter next to every pickup a robot is heading to.
    std::vector<bool> dish(static_cast<std::size_t>(g.width() * g.height()), false);
    for (const ww::RobotView& r : robots) {
        if (r.carrying) continue;
        for (Cell d : ww::kNeighbors) {
            const Cell n{r.goal.x + d.x, r.goal.y + d.y};
            if (g.inBounds(n) && g.at(n) == ww::Tile::Shelf && !dish[g.index(n)]) {
                dish[g.index(n)] = true;
                drawBowl({at(n).x, at(n).y + kCell * 0.1f}, kCell * 0.42f, true);
                break;
            }
        }
    }

    // Orders per table, and which carrying robot (colour) is on its way.
    const auto cells = static_cast<std::size_t>(g.width() * g.height());
    std::vector<int> orders(cells, 0);
    std::vector<int> courier(cells, -1);
    int stuck = 0;
    for (const ww::RobotView& r : robots) {
        const int t = g.index(tableOf(g, r.dropoff));
        ++orders[t];
        if (r.carrying && courier[t] == -1) courier[t] = r.id;
        stuck += r.waitTicks >= kStuckTicks;
    }
    auto routeColor = [](int id) { return kRouteColors[id % static_cast<int>(std::size(kRouteColors))]; };

    for (int i = 0; i < static_cast<int>(cells); ++i) {
        const Cell c = g.cellAt(i);
        const ww::Tile t = g.at(c);
        if (t != ww::Tile::Table && t != ww::Tile::Station) continue;
        const std::vector<int> sides = t == ww::Tile::Table ? chairSides(g, c) : std::vector<int>{};
        for (int side : sides) drawChair(at(c), side);
        int seated = 0;
        for (int side : sides)
            if (side != 3 && seated < orders[i]) drawCustomer(at(c), side, i * 3 + seated++);
        drawTable(at(c));
    }
    for (int i = 0; i < static_cast<int>(cells); ++i)
        if (g.at(g.cellAt(i)) == ww::Tile::Plant) drawPlant(at(g.cellAt(i)));

    // Routes of carrying robots: planned cells while there are any, then a
    // straight line to the drop-off cell.
    const float alpha = a.paused ? 1.0f : static_cast<float>(a.timestep.alpha());
    std::vector<Vector2> pos(robots.size());
    for (std::size_t k = 0; k < robots.size(); ++k) {
        const ww::RobotView& r = robots[k];
        const auto id = static_cast<std::size_t>(r.id);
        const Cell from = id < a.prev.size() ? a.prev[id] : r.cell;
        const Cell to = id < a.cur.size() ? a.cur[id] : r.cell;
        pos[k] = centre(from.x + (to.x - from.x) * alpha, from.y + (to.y - from.y) * alpha);
        if (!r.carrying) continue;
        std::vector<Vector2> pts{pos[k]};
        for (std::size_t j = 1; j < r.route.size() && !(r.route[j - 1] == r.dropoff); ++j) pts.push_back(at(r.route[j]));
        if (!(r.route.empty() ? r.cell == r.dropoff : std::find(r.route.begin(), r.route.end(), r.dropoff) != r.route.end()))
            pts.push_back(at(r.dropoff));
        Color c = routeColor(r.id);
        c.a = 190;
        drawDottedPath(pts, c, static_cast<float>(a.runTime * 14.0));
    }
    for (std::size_t k = 0; k < robots.size(); ++k) drawRobot(pos[k], robots[k].carrying, routeColor(robots[k].id));

    for (int i = 0; i < static_cast<int>(cells); ++i) {
        if (orders[i] == 0) continue;
        drawOrderBubble(at(g.cellAt(i)), orders[i], courier[i] >= 0 ? routeColor(courier[i]) : kOutline);
    }
    // Delivery pops: "+1" rising and a star bursting, fading out, to the
    // right of the bubble. Only the newest pop per table, so they don't stack.
    std::vector<double> newest(cells, -1.0);
    for (const auto& [dropoff, t0] : a.pops) newest[g.index(tableOf(g, dropoff))] = t0;
    for (int i = 0; i < static_cast<int>(cells); ++i) {
        if (newest[i] < 0) continue;
        const double t0 = newest[i];
        const float age = static_cast<float>(a.runTime - t0), k = age / static_cast<float>(kPopSeconds);
        const Vector2 c = at(g.cellAt(i));
        const unsigned char fade = static_cast<unsigned char>(255 * std::clamp(1.0f - k, 0.0f, 1.0f));
        const float grow = std::min(1.0f, age / 0.15f);
        const Vector2 star{c.x + kCell * 0.95f, c.y - kCell * 0.75f - k * 10};
        drawStar(star, 9.0f * grow + 2.0f, Color{kOutline.r, kOutline.g, kOutline.b, fade});
        drawStar(star, 9.0f * grow, Color{kStar.r, kStar.g, kStar.b, fade});
        const int tx = static_cast<int>(c.x + kCell * 0.42f), ty = static_cast<int>(c.y - kCell * 1.3f - k * 18);
        DrawText("+1", tx + 1, ty + 1, 18, Color{kOutline.r, kOutline.g, kOutline.b, fade});
        DrawText("+1", tx, ty, 18, Color{kCounter.r, kCounter.g, kCounter.b, fade});
    }

    char hud[160];
    std::snprintf(hud, sizeof hud, "orders served %lld    %.0f / min    robots stuck %d    tick %lld%s",
                  a.sim->stats().deliveries, deliveriesPerMinute(a), stuck, a.sim->stats().ticks,
                  a.paused ? "    paused" : "");
    DrawText(hud, kMargin, 10, 18, kText);
    const char* keys = "keys: space  +/-  up/down  P";
    DrawText(keys, a.width - kMargin - MeasureText(keys, 14), 13, 14, kDimText);

    const Vector2 mouse = GetMousePosition();
    for (const Button& b : layoutButtons(a)) {
        const int tw = MeasureText(b.label.c_str(), 16);
        const int tx = static_cast<int>(b.rect.x + (b.rect.width - tw) / 2), ty = static_cast<int>(b.rect.y + 6);
        if (b.action == Action::None) {
            DrawText(b.label.c_str(), tx, ty, 16, kText);
            continue;
        }
        const bool hover = b.enabled && CheckCollisionPointRec(mouse, b.rect);
        const Color fill = b.active ? kCounter : hover ? kButtonHover : kButton;
        DrawRectangleRounded(b.rect, 0.35f, 6, fill);
        DrawText(b.label.c_str(), tx, ty, 16, b.active ? kTable : b.enabled ? kButtonText : kButtonDisabled);
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
        ClearBackground(kOutline);
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
