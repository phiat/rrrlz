# Pathfinding Visualizer

SDL2-based step-through visualizer for Dijkstra and A* on a 20x20 grid.

## Build

```bash
clang -O2 visualizer.c -o visualizer $(pkg-config --cflags --libs sdl2)
```

Or via just/build script:
```bash
just visualizer
bash build_all.sh visualizer
```

## Run

```bash
just run
./visualizer/visualizer
```

## Controls

| Key | Action |
|---|---|
| Space | Step one node expansion |
| Enter | Toggle auto-run (animated) |
| R | Reset current algorithm |
| 1 | Switch to Dijkstra |
| 2 | Switch to A* |
| Tab | Cycle maps |
| +/- | Speed up / slow down |
| Q / Esc | Quit |

## Maps

| # | Name | Description |
|---|---|---|
| 0 | Original | Same scattered-obstacle grid as dijkstra.c and astar.c |
| 1 | Diagonal | Two crossing diagonal walls with gaps — simpler, shows heuristic advantage |
| 2 | Arena | Open center with perimeter walls and pillars — watch the flood fill |
| 3 | Maze | Tight single-width corridors — stress test for both algorithms |

## Colors

- **Light gray** — empty cell
- **Dark gray** — wall
- **Blue** — open set (frontier)
- **Orange** — closed set (visited/expanded)
- **Green** — final path
- **Yellow** — start (0,0)
- **Red** — end (19,19)

## Requirements

- `libsdl2-dev` (`apt install libsdl2-dev`)
- X11/WSLg display (`DISPLAY` env var)
