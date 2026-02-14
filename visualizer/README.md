# Pathfinding Visualizer

SDL2-based step-through visualizer for 14 pathfinding algorithms on variable-size grids (up to 100x100).

## Build

```bash
just visualizer    # standard build
just check         # build with -Wall -Wextra
```

## Run

```bash
just run
./visualizer/visualizer
```

## Algorithms

| Key | Algorithm | Description |
|---|---|---|
| 1 | Dijkstra | Uniform-cost expansion |
| 2 | A* | Manhattan heuristic-guided search |
| 3 | Bellman-Ford | Edge relaxation, full-pass stepping |
| 4 | IDA* | Iterative deepening with f-cost threshold |
| 5 | Floyd-Warshall | All-pairs shortest paths |
| 6 | JPS | Jump Point Search — prunes straight-line nodes |
| 7 | Fringe | IDA* variant preserving frontier between iterations |
| 8 | Flow Field | Reversed Dijkstra from goal producing vector field |
| 9 | D* Lite | Incremental replanning (backward goal→start) |
| 0 | Theta* | Any-angle A* with 8-dir movement and LOS shortcuts |
| F1 | RSR | Rectangular Symmetry Reduction — A* on rectangle perimeters |
| F2 | Subgoal | Subgoal Graphs — A* on obstacle-corner sparse graph |
| F3 | CH | Contraction Hierarchies — bidirectional hierarchical search |
| F4 | BiDir-A* | Bidirectional A* — two frontiers meeting in the middle |

## Controls

| Key | Action |
|---|---|
| Space | Step one node expansion |
| Enter | Toggle auto-run (animated) |
| R | Reset current algorithm |
| B | Benchmark (instant run, comparison table) |
| Tab | Cycle maps |
| +/- | Speed up / slow down |
| Q / Esc | Quit |

## Colors

- **Light gray** — empty cell
- **Dark gray** — wall
- **Blue** — open set (frontier)
- **Orange** — closed set (visited/expanded)
- **Green** — final path
- **Yellow** — start
- **Red** — end
- **Dim teal** — preprocessing phase (RSR, Subgoal, CH)

## Requirements

- `libsdl2-dev` (`apt install libsdl2-dev`)
- X11/WSLg display (`DISPLAY` env var)
