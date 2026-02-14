# Bellman-Ford Algorithm

Single-source shortest path on a weighted graph. Unlike Dijkstra, handles **negative edge weights** and detects negative cycles.

## How It Works

Bellman-Ford relaxes every edge in the graph, repeated V-1 times. After V-1 iterations, all shortest paths (with at most V-1 edges) are guaranteed correct. A final pass detects negative cycles.

```
  Start: dist[S] = 0, dist[*] = INF
  Build edge list from grid (each cell->neighbor = 1 directed edge)
  Repeat V-1 times:
    For each edge (u, v, w):
      if dist[u] + w < dist[v]:
        dist[v] = dist[u] + w
        parent[v] = u
  Check: any edge still relaxable? -> negative cycle
```

## ASCII Walkthrough: Edge Relaxation

```
Grid (S=start, E=end, #=wall, .=open):

  S . . #
  # . # .
  . . . .
  . # . E

Edge list (subset): S->B, B->C, B->F, F->I, ...

Iteration 1: Relax all edges
  Before:  dist = [S:0, B:INF, C:INF, F:INF, ...]
  Edge S->B: dist[S]+1 = 1 < INF  => dist[B] = 1
  Edge B->C: dist[B]+1 = 2 < INF  => dist[C] = 2
  Edge B->F: dist[B]+1 = 2 < INF  => dist[F] = 2
  After:   dist = [S:0, B:1, C:2, F:2, ...]

  [0] 1  2  #         Numbers show dist values
   #  2  #  .         updated during this iteration.
   .  .  .  .         Multiple hops can update in one pass
   .  #  .  E         if edge order is favorable.

Iteration 2: Relax all edges again
  Edge F->I: dist[F]+1 = 3 < INF  => dist[I] = 3
  Edge F->J: dist[F]+1 = 3 < INF  => dist[J] = 3
  ...more updates propagate outward

  [0] 1  2  #
   #  2  #  .
   .  3  .  .
   .  #  .  E

  ... continues for up to V-1 iterations
  (early exit if no updates in a full pass)
```

## Key Insight: Brute-Force Relaxation

```
  Dijkstra:        Bellman-Ford:

  Uses heap to     Scans ALL edges
  pick cheapest    every iteration.
  node next.       No priority queue.
  Smart order.     Brute force.

  Iteration flow:

  Pass 1: ──edge1──edge2──...──edgeE──  (some updates)
  Pass 2: ──edge1──edge2──...──edgeE──  (fewer updates)
  Pass 3: ──edge1──edge2──...──edgeE──  (even fewer)
  ...
  Pass K: ──edge1──edge2──...──edgeE──  (no updates -> stop)

  Worst case: V-1 passes needed (path visits every node).
  Best case: 1 pass if edges are in topological order.
```

## Why Bellman-Ford Over Dijkstra?

```
  Graph with negative edge:

    A --(-3)--> B --(2)--> C
    |                      ^
    +-------(5)-----------+

  Dijkstra visits A, picks B (cost -3??) -- breaks with negatives.
  Bellman-Ford: just relaxes everything repeatedly. Works fine.

  On our grid (all weights = 1): same result as Dijkstra,
  but slower due to scanning all edges each pass.
```

## Negative Cycle Detection

```
  After V-1 passes, all shortest paths are settled.
  One more pass over all edges:

  For each edge (u, v, w):
    if dist[u] + w < dist[v]:
      NEGATIVE CYCLE EXISTS!

  Why? If we can still improve after V-1 relaxations,
  some path has >= V edges, meaning a cycle exists,
  and that cycle has negative total weight.

  Our grid: all weights = 1, no negative cycles possible.
  But the detection code demonstrates the algorithm fully.
```

## Python Pseudocode

```python
def bellman_ford(grid, start, end):
    # Build edge list from grid
    edges = []
    for r, c in all_cells:
        if grid[r][c] == WALL:
            continue
        for nr, nc in neighbors(r, c):
            if is_valid(nr, nc):
                edges.append((index(r,c), index(nr,nc), 1))

    dist = {node: INF for node in all_nodes}
    parent = {node: None for node in all_nodes}
    dist[start] = 0

    # Relax all edges V-1 times
    for i in range(V - 1):
        updated = False
        for u, v, w in edges:
            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                parent[v] = u
                updated = True
        if not updated:
            break  # early exit

    # Negative cycle check
    for u, v, w in edges:
        if dist[u] + w < dist[v]:
            raise Exception("Negative cycle detected")

    return reconstruct_path(parent, end)
```

## Complexity

| Metric | Value |
|---|---|
| Time | O(V * E) |
| Space | O(V + E) for dist, parent, edge list |
| Optimality | Yes (if no negative cycles) |
| Completeness | Yes |

Where V = vertices, E = edges. On our 20x20 grid: V=400, E~1400.

Worst case: 400 * 1400 = 560,000 relaxation checks (vs Dijkstra's ~5,600 with heap). Early exit helps significantly in practice.

## LLVM Optimization Interest

1. **Edge iteration loop** -- the inner loop scans a flat array of structs sequentially. This is excellent for auto-vectorization and cache prefetching at O2+. Compare to Dijkstra's heap operations which have irregular memory access patterns.
2. **No heap / no pointer chasing** -- Bellman-Ford's hot loop is a simple linear scan. LLVM can optimize this more aggressively than heap-based code: loop unrolling, SIMD comparison, branch prediction hints.
3. **`dist[u] != INT_MAX` guard** -- this branch is initially mostly-taken (skip), then shifts to mostly-not-taken as distances fill in. Profile-guided optimization (PGO) would excel here.
4. **`is_valid` boundary checks** -- only called during edge list construction (once), not in the hot path. Different inlining profile from Dijkstra where it's called per-relaxation.
5. **Early exit optimization** -- the `any_update` flag adds a branch per iteration. At O2+ LLVM may hoist loop-invariant parts and optimize the flag tracking.
6. **Struct-of-arrays vs array-of-structs** -- the Edge struct (from, to, weight) is AoS layout. LLVM at O3 may attempt scalar replacement or partial vectorization. Converting to SoA (separate from[], to[], weight[] arrays) would enable full SIMD -- an interesting experiment.
7. **Comparison to Dijkstra profile** -- Dijkstra is heap-bound (random access, pointer-heavy), Bellman-Ford is iteration-bound (sequential access, arithmetic-heavy). The LLVM optimization passes that matter most are fundamentally different between the two.
