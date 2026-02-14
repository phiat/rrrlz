# Floyd-Warshall Algorithm

All-pairs shortest paths on a weighted graph. Computes the shortest distance between **every** pair of vertices simultaneously.

## How It Works

Floyd-Warshall uses dynamic programming with an intermediate vertex relaxation. For each potential intermediate vertex k, it checks whether routing through k improves the shortest known path between every pair (i, j).

```
  Initialize:
    dist[i][j] = weight(i,j) if edge exists
    dist[i][i] = 0
    dist[i][j] = INF otherwise
    next[i][j] = j if edge exists (for path reconstruction)

  For k = 0 to V-1:
    For i = 0 to V-1:
      For j = 0 to V-1:
        if dist[i][k] + dist[k][j] < dist[i][j]:
          dist[i][j] = dist[i][k] + dist[k][j]
          next[i][j] = next[i][k]
```

## ASCII Walkthrough: The k-Intermediate Vertex Concept

```
  The key idea: can we improve path i->j by going through k?

  Before considering k=3:

    i ─────────────────► j       dist[i][j] = 10
                                  (best known so far)

  Check: is i->k->j shorter?

    i ────► k=3 ────► j          dist[i][3] + dist[3][j] = 4 + 5 = 9
            │
            ▼
         (vertex 3 as
          intermediate)

  9 < 10, so update:
    dist[i][j] = 9
    next[i][j] = next[i][3]     (path now routes through 3)
```

### Building Up Paths Through Successive k Values

```
  k=0: Can any pair improve by routing through vertex 0?
  ┌───┐
  │ 0 │──── check all (i,j) pairs
  └───┘

  k=1: Can any pair improve by routing through vertex 1?
  ┌───┐     Paths may already use vertex 0 from previous step.
  │ 1 │──── So i->1 might itself go through 0.
  └───┘

  k=2: Can any pair improve by routing through vertex 2?
  ┌───┐     Paths may already use vertices {0, 1}.
  │ 2 │──── Chains build up: i -> 0 -> 2 -> 1 -> j
  └───┘

  ...

  k=V-1: Final pass. All possible intermediate vertices considered.
         dist[i][j] is now the true shortest path for all (i,j).
```

### Grid Example (4x4)

```
  Grid:           Adjacency (weight 1 for open neighbors):

  S . # .         0──1     3
  . . . .         │  │     │
  # . . .            5──6──7
  . . . E         │  │  │  │
                  8  9──10─11
                  │     │  │
                      13─14─15=E

  After Floyd-Warshall, dist[S][E] = shortest path through
  any combination of intermediate vertices.
```

## Python Pseudocode

```python
def floyd_warshall(grid):
    V = ROWS * COLS
    INF = float('inf')

    # Initialize matrices
    dist = [[INF] * V for _ in range(V)]
    next_node = [[-1] * V for _ in range(V)]

    for i in range(V):
        dist[i][i] = 0
        next_node[i][i] = i

    # Build adjacency from grid
    for r in range(ROWS):
        for c in range(COLS):
            if grid[r][c] == 1:
                continue  # wall
            u = r * COLS + c
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] == 0:
                    v = nr * COLS + nc
                    dist[u][v] = 1
                    next_node[u][v] = v

    # Floyd-Warshall core
    for k in range(V):
        for i in range(V):
            if dist[i][k] == INF:
                continue  # prune
            for j in range(V):
                if dist[k][j] == INF:
                    continue  # prune
                through_k = dist[i][k] + dist[k][j]
                if through_k < dist[i][j]:
                    dist[i][j] = through_k
                    next_node[i][j] = next_node[i][k]

    return dist, next_node


def reconstruct_path(next_node, start, end, dist):
    if dist[start][end] == float('inf'):
        return []  # no path

    path = [start]
    cur = start
    while cur != end:
        cur = next_node[cur][end]
        path.append(cur)
    return path
```

## Complexity

| Metric | Value |
|---|---|
| Time | O(V^3) — triple nested loop |
| Space | O(V^2) for dist and next matrices |
| Optimality | Yes (no negative cycles) |
| Completeness | Yes |

Where V = vertices. On our 20x20 grid: V=400, so ~64 million iterations in the core loop (with pruning reducing this significantly for sparse grids with walls).

### Comparison with Dijkstra

| | Dijkstra | Floyd-Warshall |
|---|---|---|
| Computes | Single-source shortest paths | All-pairs shortest paths |
| Time | O((V+E) log V) | O(V^3) |
| Space | O(V) | O(V^2) |
| Best for | One start, one end | Need all pairs, or dense graphs |

For a single source-to-destination query, Dijkstra is far more efficient. Floyd-Warshall is useful when you need shortest paths between all pairs, or when the graph is dense and represented as an adjacency matrix.

## LLVM Optimization Interest

The triple-nested loop is the defining feature from an optimization perspective:

1. **Triple loop nest** — `for k { for i { for j { ... } } }` is a textbook target for loop optimizations. At O3, LLVM applies:
   - **Loop unrolling**: The innermost j-loop body is simple (compare, add, conditional store), making it ideal for unrolling by 4x or 8x
   - **Vectorization**: The j-loop iterates over contiguous `dist[i][j]` and `next[i][j]` memory, enabling SIMD operations (SSE/AVX `vpmind` for min, `vpaddd` for add)
   - **LICM (Loop-Invariant Code Motion)**: `dist[i][k]` is invariant across the j-loop and gets hoisted out

2. **Branch pruning** — The `if (dist[i][k] == INF) continue` check skips entire j-loop iterations. LLVM can hoist this above the j-loop, turning it into an i-loop-level branch that skips ~60% of work on sparse grids

3. **Memory access patterns** — `dist[i][j]` and `dist[k][j]` are row-major sequential reads in the j-loop (cache-friendly). `dist[i][k]` is loop-invariant. This access pattern is favorable for hardware prefetching

4. **Static arrays** — The `static int dist[400][400]` declaration gives LLVM full visibility into sizes, enabling bounds-check elimination and potential alias analysis

5. **No function calls in hot loop** — The core triple loop has no calls, so the entire hot path lives in registers + L1/L2 cache. Compare to Dijkstra where heap operations introduce function call overhead in the hot loop

6. **O2 vs O3 delta** — Expect a notable difference: O2 applies LICM and basic unrolling, but O3 enables aggressive vectorization of the j-loop. The `Os`/`Oz` levels will avoid unrolling, showing the size-vs-speed tradeoff clearly in the generated assembly
