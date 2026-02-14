# Dijkstra's Algorithm

Single-source shortest path on a weighted graph with **non-negative** edge weights.

## How It Works

Dijkstra greedily expands the unvisited node with the smallest known distance from the source. Once a node is visited, its distance is final.

```
  Start: dist[S] = 0, dist[*] = INF
  Repeat:
    1. Pop node with smallest dist from priority queue
    2. Mark it visited
    3. For each neighbor: if dist[current] + weight < dist[neighbor],
       update dist[neighbor] and push to queue
```

## ASCII Walkthrough

```
Grid (S=start, E=end, #=wall, .=open):

  S . . #
  # . # .
  . . . .
  . # . E

Step 1: Expand S (dist=0)
  [S] .  .  #        queue: {(1,0)=1, (0,1)=1}
   #  .  #  .        dist:  S=0
   .  .  .  .
   .  #  .  E

Step 2: Expand (0,1) (dist=1)
  [S][1] .  #        queue: {(1,0)=1, (0,2)=2}
   #  .  #  .
   .  .  .  .
   .  #  .  E

Step 3: Expand (1,0)... wait, it's a wall. Expand (0,2) (dist=2)
  [S][1][2] #        queue: {(1,1)=2}
   #  .  #  .        Walls block expansion —
   .  .  .  .        must find path around
   .  #  .  E

  ... continues until E is popped from queue
```

## Key Insight: Greedy on Distance

```
  Priority Queue (min-heap by dist):

  Pop 0──► S     expand neighbors, push with dist=1
  Pop 1──► A     expand neighbors, push with dist=2
  Pop 1──► B     expand neighbors, push with dist=2
  Pop 2──► C     ...
  Pop 2──► D     ...
  ...
  Pop N──► E     DONE — shortest path found

  Nodes come out in order of increasing distance.
  First time we pop a node = optimal distance to it.
```

## Why No Heuristic?

Dijkstra explores in **concentric rings** outward from the source — it doesn't know where the goal is. Compare to A* which biases toward the goal.

```
  Dijkstra expansion pattern:       A* expansion pattern:

       3 3 3                              3
     3 2 2 2 3                          3 2 3
   3 2 1 1 1 2 3                      3 2 1 2
   3 2 1 S 1 2 3         vs         3 2 1 S 1 2
   3 2 1 1 1 2 3                      3 2 1 2
     3 2 2 2 3                          3 2 2 3
       3 3 3                              3 3 E
                                    (biased toward E)
```

## Python Pseudocode

```python
def dijkstra(grid, start, end):
    dist = {node: INF for node in all_nodes}
    parent = {node: None for node in all_nodes}
    dist[start] = 0
    visited = set()
    pq = [(0, start)]  # (distance, node)

    while pq:
        d, node = heappop(pq)

        if node in visited:
            continue
        visited.add(node)

        if node == end:
            break

        for neighbor in get_neighbors(node):
            if neighbor in visited:
                continue
            new_dist = d + weight(node, neighbor)
            if new_dist < dist[neighbor]:
                dist[neighbor] = new_dist
                parent[neighbor] = node
                heappush(pq, (new_dist, neighbor))

    return reconstruct_path(parent, end)
```

## Complexity

| Metric | Value |
|---|---|
| Time | O((V + E) log V) with binary heap |
| Space | O(V) for dist, parent, visited arrays |
| Optimality | Yes (with non-negative weights) |
| Completeness | Yes |

Where V = vertices, E = edges. On our 20x20 grid: V=400, E~1400.

## LLVM Optimization Interest

1. **Heap operations** — bubble-up/down are tight loops with comparisons and swaps. O2 inlines `heap_swap`, O3 may unroll
2. **`is_valid` boundary checks** — 4 comparisons + array access, called in inner loop. Prime inlining target
3. **`get_index` arithmetic** — `r * COLS + c` strength-reduced at O2+
4. **Main loop structure** — while + for(4 directions) is a classic loop nest for LICM
5. **No recursion** — purely iterative, so optimization profile differs from recursive algos like IDA*
