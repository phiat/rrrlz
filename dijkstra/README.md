# Dijkstra's Pathfinding — LLVM Optimization Study

## What This Is

A self-contained C implementation of Dijkstra's shortest-path algorithm on a 20x20 grid, designed to produce meaningful differences across LLVM optimization levels (O1 through Oz).

## Algorithm

Dijkstra's algorithm finds the shortest path from a source to all reachable nodes by always expanding the lowest-cost unexplored node. It makes no assumptions about the goal location — it expands outward uniformly until the destination is reached.

**Key properties:**
- Guarantees optimal (shortest) path
- Explores nodes in order of increasing cost from the start
- No heuristic — explores broadly compared to A*
- Time complexity: O((V + E) log V) with a binary heap

## Implementation Details

- **Grid**: 20x20 hardcoded map with walls (`#`), open cells (`.`), start (`S`), end (`E`)
- **Priority queue**: Array-based min-heap with bubble-up/bubble-down
- **Movement**: 4-directional (up, down, left, right), uniform cost of 1
- **Path reconstruction**: Parent array backtracking from end to start

### Helper functions (optimization targets)

| Function | Purpose | Optimization interest |
|---|---|---|
| `get_index(r, c)` | 2D → 1D index conversion | Inlining, strength reduction |
| `is_valid(r, c)` | Bounds + wall check | Branch elimination, inlining |
| `heap_push()` | Insert into min-heap | Loop optimization |
| `heap_pop()` | Extract minimum from heap | Loop unrolling, branch prediction |

## Building

```bash
# From project root
bash build_all.sh dijkstra

# Or build everything
bash build_all.sh
```

This produces for each optimization level (O1/O2/O3/Os/Oz):
- `dijkstra_opt_<level>.ll` — Optimized LLVM IR
- `dijkstra_opt_<level>.s` — x86-64 assembly
- `dijkstra_opt_<level>` — Executable binary

## Running

```bash
./dijkstra/dijkstra_opt_O2
```

Output:
```
Dijkstra Pathfinding (20x20 grid)
------------------------------------------
S....#........#.....
*##..#.##.##..#.##..
*#......#..#.....#..
***###..#....###....
..***#....#....#..#.
##..*..##.#.#....##.
...#*...#...#..#....
.#.#*##....#...#.#..
.#..****.#.#.#.....#
....#.#*.#...#..#...
(...)
Path cost:      38
Path length:    39 nodes
Nodes explored: 296
```

## Comparing Optimization Levels

```bash
# IR differences (inlining, loop transforms)
diff dijkstra_opt_O1.ll dijkstra_opt_O3.ll

# Assembly differences (instruction selection, scheduling)
diff dijkstra_opt_O1.s dijkstra_opt_O3.s

# Size comparison
ls -lhS dijkstra_opt_*.ll dijkstra_opt_*.s
```

### What to look for

- **O1 → O2**: Function inlining (`get_index`, `is_valid` get absorbed), basic loop optimizations
- **O2 → O3**: Loop unrolling in heap operations, more aggressive inlining
- **Os/Oz**: Smaller code — fewer inlined copies, no unrolling, size-optimized instruction selection
