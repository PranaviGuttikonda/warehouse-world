# Vision: Rush Hour Café

The engine stays a general multi-agent path finding (MAPF) simulator: grid,
planners, reservations, deterministic ticks. Its showcase becomes an
interactive game, **Rush Hour Café**, where robot waiters carry orders from
the kitchen to tables during a lunch rush.

## Same engine, new names

| Café | Engine today |
|---|---|
| Kitchen counter | Shelves and their pick faces (pickup) |
| Table | Station (drop-off) |
| Walls, plants, pillars | Obstacles |
| Orders served per minute | Deliveries per minute |

One kitchen counter along one wall makes a natural bottleneck: every order
starts at the same few cells. That's where robots crowd, so the difference
between planners is easy to see.

## Look and feel

Minimal and dark: flat shapes, thin outlines (1.5–2 px), no shadows or
gradients, and one accent colour that only ever means food.

![The café in the viewer: robots as small squares, tables with amber order dots, the kitchen counter along the top wall](cafe.png)

| Element | Look |
|---|---|
| Background / floor / walls | `#16181D` / flat `#1C1F26` / `#2A2E37` |
| Kitchen counter | a flat band `#2F333D`, pickup mats `#262A32` |
| Tables | `#1C1F26` with a thin `#3A3F4A` outline |
| Plants | small flat `#334036` circles |
| Robots, main text | off-white `#E6E2DA`; secondary text `#8A8F98`; paths `#5A606B` |
| Accent | amber `#E6A15A`, for food and orders only |

- **Robots** are small rounded squares (about 60% of a cell), so they never
  look like the round tables: a hollow off-white outline on the way to the
  kitchen, filled amber while carrying. Two tiny eyes look the way the robot
  is moving.
- **Orders:** a small amber dot in the centre of a table, with a count beside
  it if there's more than one. On delivery the dot fades and a small grey
  "+1" drifts up.
- **Paths** are hidden; hover or click a robot to see its planned path as a
  faint dotted line, or press T for all of them.
- **HUD:** one line of small text, e.g. "142 served · 38/min · 2 stuck".
  Controls are quiet text buttons; the active planner is off-white with a
  thin amber underline.
- Font: IBM Plex Sans (SIL Open Font License), committed in `assets/fonts/`.

## Game loop

A lunch-rush timer, an orders-served counter, and a 1–3 star rating at the
end of each round.

## Planned features

- Comparison mode: two planners side by side on the same map and seed, a
  congestion heatmap, and a hover card per robot.
- Map editor: place tables, walls and plants.
- One-way aisle arrows.
- Challenge levels: "serve N orders in T ticks with R robots".
- Runs in the browser via WebAssembly.

## Order of work

A* ✅ → space-time A* ✅ → CBS ✅ → café look and map → space-time A*
clearance fix → comparison mode → map editor → one-way aisles → challenge
levels → web deploy.
