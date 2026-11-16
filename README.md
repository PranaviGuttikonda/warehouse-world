# warehouse-world

A grid warehouse where robots pick items from shelves and deliver them to
packing stations without colliding. Written in C++20 with no dependencies.

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
```

## How it works

| Piece | File | What it does |
|---|---|---|
| Grid | `grid.hpp` | Parses an ASCII map; knows walls, shelves, stations, pick faces |
| ECS | `ecs.hpp` | Sparse-set component pools; dense arrays, O(1) add/remove |
| Fixed timestep | `timestep.hpp` | Accumulator loop: sim ticks at a constant rate, independent of frame rate, with a catch-up cap |
| Movement | `systems.cpp` | One cell per tick along a cached BFS distance field; reservation rule prevents overlaps and head-on swaps |
| Tasks | `systems.cpp` | Shelf face → station → new task; counts deliveries |
| Simulation | `simulation.hpp` | Owns the world, runs systems in order, validates invariants |

Same seed, same run: the simulation is fully deterministic.

## Phase 1 baseline (local collision avoidance only)

Deliveries in 20,000 ticks on the default map (seed 42):

| Robots | 4 | 8 | 12 | 16 | 20 | 24 | 28 | 32 | 40 |
|---|---|---|---|---|---|---|---|---|---|
| Deliveries | 2258 | 3669 | 4127 | **4758** | 4044 | 4203 | 428 | 2835 | 139 |

Throughput peaks around 16 robots and then collapses. Each robot plans
alone and only reacts to its neighbours, so crowded aisles gridlock. Fixing
that is the point of phase 2.

## Roadmap

- [x] **Phase 1, core engine:** grid, ECS, fixed timestep, collision-free local movement, terminal view, tests
- [ ] **Phase 2, warehouse brain:** A*, space-time A* with reservations, conflict-based search (CBS), smarter task assignment; beat the baseline above
- [ ] **Phase 3, authoritative server:** sim runs as a server, streams state over TCP at a fixed tick rate; viewer and order-injection clients
