# A* Search Algorithm

Best-first search that uses a **heuristic** to guide exploration toward the goal. Finds optimal paths when the heuristic is admissible (never overestimates).

## How It Works

Like Dijkstra, but the priority queue is ordered by **f = g + h**:
- **g** = actual cost from start to current node
- **h** = estimated cost from current node to goal (heuristic)
- **f** = total estimated cost of path through this node

```
  Start: g[S] = 0, f[S] = h(S)
  Repeat:
    1. Pop node with smallest f from priority queue
    2. Mark it closed
    3. For each neighbor: if g[current] + weight < g[neighbor],
       update g[neighbor], set f = g + h, push to queue
```

## ASCII Walkthrough

```
Grid: S=start, E=end, h=Manhattan distance to E

  S(h=6) .(h=5) .(h=4) #
  #      .(h=4) #      .(h=2)
  .(h=4) .(h=3) .(h=2) .(h=1)
  .(h=3) #      .(h=1) E(h=0)

Step 1: Expand S, f=0+6=6
  Push (0,1) with f=1+5=6
  Push (1,0) — wall, skip

Step 2: Expand (0,1), f=6. Ties broken arbitrarily.
  Push (0,2) with f=2+4=6

Step 3: Expand (0,2), f=6
  (0,3) is wall. Push (1,2) — wall, skip.
  Must go down...

  A* follows the heuristic gradient toward E,
  exploring far fewer nodes than Dijkstra.
```

## The Heuristic: Manhattan Distance

```
  For 4-connected grid (no diagonals):
  h(node) = |node.row - end.row| + |node.col - end.col|

  Example: node at (3,5), end at (19,19)
  h = |3-19| + |5-19| = 16 + 14 = 30

  Properties:
  - Admissible: never overestimates (each step costs 1,
    Manhattan = minimum steps needed)
  - Consistent: h(a) <= cost(a,b) + h(b) for all edges
  - Result: A* with Manhattan on a grid is optimal
```

## Dijkstra vs A*: Expansion Comparison

```
  Dijkstra: expands in rings        A*: expands toward goal
  (no goal awareness)               (heuristic-guided)

  . . 4 4 4 4 . . . .              . . . . . . . . . .
  . 3 3 3 3 3 3 . . .              . . . . . . . . . .
  . 3 2 2 2 2 3 . . .              . . . . 4 . . . . .
  . 3 2 1 1 2 3 . . .              . . . 3 3 3 . . . .
  . 3 2 1 S 2 3 4 . .              . . . 3 S 2 3 . . .
  . 3 2 1 1 2 3 . . .              . . . . 1 2 3 . . .
  . 3 2 2 2 2 3 . . .              . . . . . 2 3 4 . .
  . 3 3 3 3 3 3 . . .              . . . . . . 3 4 . .
  . . 4 4 4 4 . . . .              . . . . . . . 4 E .

  Explored: ~80 nodes               Explored: ~25 nodes
  Same path, same cost — A* just finds it faster.
```

## Python Pseudocode

```python
def astar(grid, start, end):
    g_cost = {node: INF for node in all_nodes}
    parent = {node: None for node in all_nodes}
    g_cost[start] = 0
    closed = set()
    pq = [(heuristic(start, end), start)]  # (f_cost, node)

    while pq:
        f, node = heappop(pq)

        if node in closed:
            continue
        closed.add(node)

        if node == end:
            break

        for neighbor in get_neighbors(node):
            if neighbor in closed:
                continue
            new_g = g_cost[node] + weight(node, neighbor)
            if new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                parent[neighbor] = node
                f = new_g + heuristic(neighbor, end)
                heappush(pq, (f, neighbor))

    return reconstruct_path(parent, end)

def heuristic(node, goal):
    """Manhattan distance — admissible for 4-connected grid."""
    return abs(node.row - goal.row) + abs(node.col - goal.col)
```

## Complexity

| Metric | Value |
|---|---|
| Time | O(E log V) worst case, but typically much less than Dijkstra |
| Space | O(V) for g_cost, parent, closed arrays + heap |
| Optimality | Yes (with admissible heuristic) |
| Completeness | Yes |

Practical performance depends heavily on heuristic quality. Perfect heuristic = straight-line expansion. Zero heuristic = degrades to Dijkstra.

## LLVM Optimization Interest

1. **`heuristic` function** — small arithmetic function (subtract, abs, add) called in hot inner loop. Key inlining candidate at O2+
2. **Abs without branches** — `(dr < 0 ? -dr : dr)` is a conditional move opportunity. Check if LLVM uses `cmov` or arithmetic trick
3. **Same heap as Dijkstra** — but priority is f=g+h instead of just g. Compare: does the extra addition in the hot path change optimization decisions?
4. **Fewer iterations** — A* explores fewer nodes, so the main loop runs fewer times. Does this affect loop unrolling heuristics?
5. **`heuristic` is pure** — no side effects, same inputs = same output. LLVM can CSE/hoist repeated calls
