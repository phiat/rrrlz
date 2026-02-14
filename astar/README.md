# A* Pathfinding — LLVM Optimization Study

## What This Is

A self-contained C implementation of the A* search algorithm on a 20x20 grid, designed to produce meaningful differences across LLVM optimization levels (O1 through Oz).

## Algorithm

A* extends Dijkstra by adding a heuristic estimate of remaining distance to the goal. Each node is scored as **f(n) = g(n) + h(n)** where g is the known cost from start and h is the heuristic estimate to the goal. This focuses the search toward the destination.

**Key properties:**
- Guarantees optimal path (with an admissible heuristic)
- Explores far fewer nodes than Dijkstra on the same grid
- Manhattan distance heuristic for 4-directional movement
- Time complexity: O((V + E) log V) worst case, but typically much better in practice

## Implementation Details

- **Grid**: Same 20x20 map as the Dijkstra implementation (identical walls/start/end)
- **Priority queue**: Array-based min-heap, ordered by f-cost
- **Heuristic**: Manhattan distance — `|row - goal_row| + |col - goal_col|`
- **Movement**: 4-directional, uniform cost of 1
- **Path reconstruction**: Parent array backtracking

### Helper functions (optimization targets)

| Function | Purpose | Optimization interest |
|---|---|---|
| `get_index(r, c)` | 2D → 1D index | Inlining, strength reduction |
| `is_valid(r, c)` | Bounds + wall check | Branch elimination, inlining |
| `heuristic(r, c)` | Manhattan distance | Constant propagation, algebraic simplification |
| `heap_push()` | Insert into min-heap | Loop optimization |
| `heap_pop()` | Extract minimum | Loop unrolling, branch prediction |

## Building

```bash
# From project root
bash build_all.sh astar

# Or build everything
bash build_all.sh
```

## Running

```bash
./astar/astar_opt_O2
```

Output:
```
A* Pathfinding (20x20 grid)
------------------------------------------
S....#........#.....
*##..#.##.##..#.##..
*#......#..#.....#..
***###..#....###....
..***#....#....#..#.
##..*..##.#.#....##.
(...)
Path cost:      38
Path length:    39 nodes
Nodes explored: 100
```

## A* vs Dijkstra Comparison

Both algorithms find the same optimal path (cost 38), but:

| Metric | Dijkstra | A* |
|---|---|---|
| Nodes explored | 296 | 100 |
| Explores toward goal? | No (uniform) | Yes (heuristic-guided) |
| Extra computation | None | Heuristic evaluation per node |

The heuristic gives A* a ~3x reduction in explored nodes on this grid.

## Comparing Optimization Levels

```bash
diff astar_opt_O1.ll astar_opt_O3.ll
diff astar_opt_O1.s astar_opt_O3.s
```

### What to look for

- **`heuristic()` inlining**: At O2+, the Manhattan distance calculation gets inlined and the abs() branches may be simplified
- **Heap operations**: O3 may unroll the bubble-up/bubble-down loops
- **`is_valid()` chain**: The 4-condition check gets different branch layouts at different levels
- **Os/Oz**: Trades speed for size — less inlining, compact instruction sequences
