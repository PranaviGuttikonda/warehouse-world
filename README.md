# warehouse-world

![Robots moving between shelves and packing stations in the graphical viewer](docs/viewer.gif)

A grid warehouse where robots pick items from shelves and deliver them to
packing stations without colliding. The core is C++20 with no dependencies;
the optional graphical viewer uses [raylib](https://www.raylib.com).

```
#S...........le.........S#      lowercase  robot heading to a shelf
#..h.......j.............#      UPPERCASE  robot carrying an item
#..===..===..===..===....#      =          shelf
#........F.b.............#      S          packing station
#I.===.a===..===..===....#
#S.....................KS#
tick 40  robots 12  deliveries 11  moves 402  waits 77
```

## Build and run

Needs CMake 3.20+ and a C++20 compiler (on macOS: `xcode-select --install`,
`brew install cmake`).

```sh
cmake -S . -B build
cmake --build build -j
./build/ww_tests                         # run the tests
./build/warehouse                        # live view, Ctrl-C to quit
./build/warehouse --robots 20 --hz 15
./build/warehouse --map maps/small.txt --robots 6
./build/warehouse --headless --robots 16 --ticks 200000   # benchmark
./build/warehouse --planner stastar --robots 28           # space-time A*
./build/warehouse --planner cbs --robots 40               # conflict-based search
```

`--planner` picks how robots move: `local` (phase 1, the default),
`stastar` (space-time A* with reservations) or `cbs` (conflict-based
search, all robots planned together); see below.

### Graphical viewer

The viewer is opt-in, because configuring it downloads raylib 5.5 (pinned by
hash) on the first run.

```sh
cmake -S . -B build -DWW_VIEWER=ON
cmake --build build -j
./build/viewer                           # purple = to shelf, coral = carrying
./build/viewer --robots 40 --hz 15 --seed 3 --planner cbs
./build/viewer --robots 24 --planner stastar --record docs/viewer.gif  # the GIF above
```

The bottom bar has buttons for everything, so the browser build works
without a keyboard: pause, speed −/+, robot count −/+, and one button per
planner (the active one is highlighted). Changing the planner or robot count
restarts the run with the same seed, so comparisons are fair. Keys do the
same: <kbd>space</kbd> pause, <kbd>+</kbd>/<kbd>-</kbd> speed,
<kbd>↑</kbd>/<kbd>↓</kbd> robots, <kbd>P</kbd> next planner. The HUD shows
the tick, deliveries per minute at the current speed (over the last 10 s)
and how many robots are blocked. On Linux, raylib also needs
the X11 and OpenGL development packages.

### In the browser (WebAssembly)

Needs [Emscripten](https://emscripten.org) (`brew install emscripten`, or
emsdk). Tests and the terminal app are skipped in this build.

```sh
emcmake cmake -S . -B build-web -DWW_VIEWER=ON
cmake --build build-web -j --target viewer
python3 -m http.server -d build-web 8000  # open http://localhost:8000/viewer.html
```

The output is `viewer.html`, `viewer.js` and `viewer.wasm`; any static file
host can serve them.

## How it works

| Piece | File | What it does |
|---|---|---|
| Grid | `grid.hpp` | Parses an ASCII map; knows walls, shelves, stations, pick faces |
| ECS | `ecs.hpp` | Sparse-set component pools; dense arrays, O(1) add/remove |
| Random | `random.hpp` | Portable Fisher–Yates shuffle and uniform ints on `mt19937_64` |
| Fixed timestep | `timestep.hpp` | Accumulator loop: sim ticks at a constant rate, independent of frame rate, with a catch-up cap |
| Local movement | `systems.cpp` | `--planner local`: one cell per tick along a cached BFS distance field; reservation rule prevents overlaps and head-on swaps |
| A* | `astar.hpp` | Single-robot shortest path (Manhattan heuristic, deterministic ties); the base for space-time planning |
| Reservations | `reservation.hpp` | Who holds which cell at which tick, plus "parked from tick t on"; swap checks fall out of the vertex entries |
| Space-time A* | `stastar.hpp` | `--planner stastar`: A* over (cell, tick) against the reservation table; replans on a new goal |
| CBS | `cbs.hpp` | `--planner cbs`: windowed conflict-based search, all robots planned jointly; replans on any new goal |
| Tasks | `systems.cpp` | Shelf face → station → new task; counts deliveries |
| Viewer | `apps/viewer.cpp` | raylib window or WebAssembly page; robots move smoothly between cells using `FixedTimestep::alpha()` |
| Simulation | `simulation.hpp` | Owns the world, runs systems in order, validates invariants |

Same seed, same run: the simulation is fully deterministic, on every
platform.

## Phase 1 baseline (local collision avoidance only)

Deliveries in 20,000 ticks on the default map, over seeds 1–5
(`scripts/bench.sh`):

| Robots | 4 | 8 | 12 | 16 | 20 | 24 | 28 | 32 | 40 |
|---|---|---|---|---|---|---|---|---|---|
| Mean | 1872 | 3624 | 4029 | 4545 | 4037 | 3638 | 1973 | 1283 | 985 |
| Min-max | 846-2223 | 3592-3683 | 3818-4390 | 4349-4645 | 3011-4561 | 1920-4289 | 310-3750 | 146-2475 | 631-1872 |

Throughput peaks around 16 robots and then collapses, and past 20 robots
the outcome swings widely with the seed. Each robot plans alone and only
reacts to its neighbours, so crowded aisles gridlock. Fixing that is the
point of phase 2.

Earlier versions of this table used `std::shuffle` and
`std::uniform_int_distribution`, which differ between standard libraries;
the RNG is now portable (`random.hpp`), so these numbers reproduce anywhere.

## Phase 2: space-time A*

Deliveries in 20,000 ticks on the default map, over seeds 1–5
(`scripts/bench.sh --planner local` and `--planner stastar`):

| Robots | 4 | 8 | 12 | 16 | 20 | 24 | 28 | 32 | 40 |
|---|---|---|---|---|---|---|---|---|---|
| Local, mean | 1872 | 3624 | 4029 | 4545 | 4037 | 3638 | 1973 | 1283 | 985 |
| Local, min-max | 846-2223 | 3592-3683 | 3818-4390 | 4349-4645 | 3011-4561 | 1920-4289 | 310-3750 | 146-2475 | 631-1872 |
| **Space-time A\*, mean** | **2562** | **5041** | **7372** | **9536** | **11491** | **13196** | **14612** | **14726** | **8816** |
| Space-time A\*, min-max | 2552-2568 | 5022-5069 | 7348-7389 | 9483-9562 | 11469-11533 | 13125-13242 | 14544-14688 | 10711-15803 | 2661-16891 |
| Ratio of means | 1.4× | 1.4× | 1.8× | 2.1× | 2.8× | 3.6× | 7.4× | 11.5× | 9.0× |

Space-time A* beats local at every robot count, and throughput keeps
climbing up to 28–32 robots instead of collapsing after 16. It is also far
less sensitive to the seed: up to 28 robots the five runs land within about 1% of
each other. At 32 robots one seed falls to 10,711. At 40 robots three of
five seeds still gridlock (2,661–4,533 deliveries) while the other two reach
about 16,800. It costs more CPU: about 0.8 ms per tick at 16 robots,
against microseconds for local, which is still fine for the live view.

How it works (`stastar.hpp`):

- **Plan in space and time.** Each robot searches over (cell, tick) with
  wait plus 4 moves, skipping cells another robot holds at that tick and
  head-on swaps. The heuristic is the exact BFS distance, so it never
  overestimates. Robots plan one at a time, in a rotating order, against
  everyone else's reservations.
- **Always safe.** Every plan ends with the robot parked on a cell it can
  keep forever, and nobody may plan through a parked cell. A robot whose
  planning fails stays where it is, which its previous plan already made
  safe. The tests check thousands of ticks with no shared cell or swap.
- **Touch and go.** A plan visits its goal and then continues to a cell
  that is nobody's goal. Goals are only ever held for single ticks, so
  many robots can line up for the same station. In an earlier version
  robots parked on their goal: the next robot couldn't even plan towards
  a station until the previous one left, and at 16 robots it made 225
  deliveries in the first 1,200 ticks to local's 303 (seed 1).
- **Keep station approaches clear.** No robot parks within 2 cells of a
  station. A corner station has only two access cells, and robots stepping
  off it onto them sealed it; at 28 robots (seed 1) every robot ended up
  queued behind one corner by tick 4,600: 2,491 deliveries, against 14,587
  with the clearance. A robot stuck
  inside that zone, or on another robot's goal, moves out of the way
  instead of waiting.

**Limits.** Robots plan one after another and can't ask each other to
move, so at high density they can still box each other in (40 robots
fill a fifth of the floor). Of the variations tried, a wider station
clearance (3) and letting every stuck robot step aside were both worse.
CBS, below, plans robots jointly and fixes these cases.

## Phase 2: conflict-based search (CBS)

Deliveries in 20,000 ticks on the default map, mean over seeds 1–5
(`scripts/bench.sh --planner cbs`):

| Robots | 4 | 8 | 12 | 16 | 20 | 24 | 28 | 32 | 40 |
|---|---|---|---|---|---|---|---|---|---|
| Local | 1872 | 3624 | 4029 | 4545 | 4037 | 3638 | 1973 | 1283 | 985 |
| Space-time A\* | 2562 | 5041 | 7372 | 9536 | 11491 | 13196 | 14612 | 14726 | 8816 |
| **CBS** | **2568** | **5101** | **7625** | **10064** | **12467** | **14844** | **17106** | **19337** | **23577** |
| CBS, min-max | 2549-2583 | 5081-5125 | 7591-7673 | 10034-10111 | 12431-12493 | 14808-14866 | 17058-17143 | 19303-19396 | 23474-23640 |
| CBS vs space-time A\* | 1.00× | 1.01× | 1.03× | 1.06× | 1.08× | 1.12× | 1.17× | 1.31× | 2.67× |

With few robots CBS and space-time A* are about the same: robots rarely
meet. The gap opens with density, and at 40 robots CBS never gridlocks:
throughput keeps rising almost linearly instead of collapsing, and all
five seeds land within 1% of each other. The three seeds where space-time
A* gridlocked (1, 3, 4: 3,212 / 4,533 / 2,661 deliveries) now deliver
23,640 / 23,563 / 23,474. Cost: about 0.23 ms per tick at 40 robots; 4–10
of the ~18,600 solves per run hit the node budget and used the fallback.

How it works (`cbs.hpp`):

- **Windowed.** Robots get new tasks all the time, so CBS plans everyone
  jointly for the next 5 ticks only. Everyone replans as soon as any
  robot reaches its goal (its task changes), and at least every 5 ticks,
  so every step a robot takes was checked against every other robot.
  Conflicts after a robot's arrival are ignored: those ticks never run.
- **Constraint tree.** Each node holds constraints ("agent i not on cell c
  at tick t", or "not along edge u → v") and one path per robot. The
  earliest conflict (same cell, or a swap) splits a node into two
  children, one constraint for each of the two robots; only that robot is
  replanned, with space-time A* depth-limited to the window.
- **Greedy order.** Nodes are expanded fewest conflicts first, then by
  total cost. Plain cost-first CBS almost never finished within its budget
  at 40 robots (every solve fell back); greedy ordering trades optimality
  for solving in time.
- **Safe fallback.** If 1,500 nodes aren't enough, robots are planned one
  at a time: each avoids the paths before it and the cells of the robots
  after it, so everyone can at least wait. It is always conflict-free,
  just conservative; it ran in under 0.1% of solves at 40 robots.

## Roadmap

The engine stays a general multi-agent path finding sim; its showcase is
becoming a game, Rush Hour Café (robot waiters, kitchen counter, tables).
See [docs/VISION.md](docs/VISION.md).

- [x] Core engine: grid, ECS, fixed timestep, local movement, terminal view, tests
- [x] Graphical viewer: raylib, desktop and WebAssembly
- [x] A*
- [x] Space-time A* with planner buttons
- [ ] Conflict-based search (CBS)
- [ ] Café theme + map editor
- [ ] One-way aisles
- [ ] Challenge levels
- [ ] Web deploy
