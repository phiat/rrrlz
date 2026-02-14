# IDA* (Iterative Deepening A*)

## Overview

IDA* combines the optimality guarantee of A* with the memory efficiency of
iterative deepening depth-first search (IDDFS). Instead of maintaining an open
list in memory (like A*'s priority queue), IDA* performs repeated depth-first
searches, each bounded by an f-cost threshold that increases with every
iteration.

Where A* needs O(b^d) memory for its open/closed lists, IDA* needs only O(d)
-- the depth of the current path stored on the call stack.

## Algorithm

### Core Idea

1. Set initial threshold = h(start) (the heuristic estimate)
2. Perform a depth-first search, pruning any node where f(n) = g(n) + h(n)
   exceeds the current threshold
3. Track the minimum f-value that exceeded the threshold during the search
4. Set the next threshold to that minimum exceeded f-value
5. Repeat until the goal is found or no path exists

### Python Pseudocode

```python
def ida_star(start, goal, h):
    threshold = h(start)
    path = [start]

    while True:
        result = search(path, 0, threshold, goal, h)
        if result == FOUND:
            return path
        if result == INF:
            return None   # no path
        threshold = result  # next iteration uses tighter bound

def search(path, g, threshold, goal, h):
    node = path[-1]
    f = g + h(node)

    if f > threshold:
        return f          # report minimum exceeded cost

    if node == goal:
        return FOUND

    min_exceeded = INF

    for neighbor in successors(node):
        if neighbor in path:
            continue      # cycle detection

        path.append(neighbor)
        t = search(path, g + cost(node, neighbor), threshold, goal, h)

        if t == FOUND:
            return FOUND

        min_exceeded = min(min_exceeded, t)
        path.pop()

    return min_exceeded
```

## How the Threshold Expands

Each iteration widens the f-cost horizon by exactly the minimum amount needed
to include at least one more node:

```
Iteration 1: threshold = h(S) = 38
    +-----------+
    |S          |     DFS explores only nodes with f <= 38
    | \         |     Most nodes pruned immediately
    |  ...      |
    |         E |
    +-----------+
    min exceeded f = 39 --> next threshold

Iteration 2: threshold = 39
    +-----------+
    |S**        |     Explores slightly deeper
    |  *        |     Still pruning nodes with f > 39
    |  ...      |
    |         E |
    +-----------+
    min exceeded f = 40 --> next threshold

    ...iterations continue...

Iteration K: threshold = 33 + h_errors
    +-----------+
    |S*****     |     Finally reaches goal
    |     *     |     with optimal path cost = 33
    |     ****  |
    |         E |
    +-----------+
    FOUND!
```

### F-cost Contour Expansion

```
    f=38    f=39    f=40    ...    f=optimal
    +--+    +---+   +----+         +--------+
    |S |    |S  |   |S   |         |S*******|
    |  |    | . |   | .. |   ...   |  *   * |
    +--+    +---+   | .. |         |  *****E|
                    +----+         +--------+

    Each contour includes all nodes with f <= threshold.
    Nodes beyond the contour are pruned in that iteration.
```

## Complexity Analysis

### Time Complexity

- **Worst case:** O(b^d) per iteration, with multiple iterations
- In practice, with a consistent heuristic, the number of distinct f-values
  determines iteration count. For unit-cost grids, this equals the optimal
  path cost minus h(start), often modest.
- Nodes are re-explored across iterations (the key tradeoff for memory savings)

### Space Complexity

- **O(d)** where d = depth of optimal solution
- Only the current path is stored (on the call stack / path array)
- Contrast with A*: O(b^d) for open list + closed set
- For the 20x20 grid: IDA* uses ~400 bytes for path; A* uses ~6400+ bytes
  for g_cost, parent, closed arrays plus heap

### When to Prefer IDA* Over A*

| Factor          | A*                  | IDA*                    |
|-----------------|---------------------|-------------------------|
| Memory          | O(b^d) -- can blow  | O(d) -- always fits     |
| Redundant work  | None (closed list)  | Re-explores each iter   |
| Implementation  | Heap + arrays       | Simple recursion        |
| Best for        | Small-medium graphs | Memory-constrained, or  |
|                 |                     | very large state spaces |

## LLVM Optimization Notes

IDA* presents a fundamentally different optimization profile from A* due to
its recursive, stack-heavy structure vs A*'s heap-based, iterative approach.

### Recursive DFS = Deep Call Chains

The `search()` function recurses up to depth d (optimal path length). For
path cost 33, this means 33+ nested calls. Key LLVM considerations:

1. **Inlining decisions (`-inline-threshold`)**
   - `search()` calls itself recursively -- LLVM will NOT inline recursive
     calls (would be infinite expansion)
   - However, `heuristic()`, `is_valid()`, and `get_index()` ARE inlined
     into `search()`, reducing per-call overhead
   - At `-O2`, expect the helper functions to be fully inlined, leaving
     `search()` as a tight recursive function

2. **Tail call optimization**
   - `search()` is NOT tail-recursive (it does work after the recursive call:
     min comparison, path pop). LLVM cannot apply tail call elimination.
   - Each recursive call consumes a full stack frame

3. **Stack frame layout**
   - LLVM will optimize the stack frame of `search()` to be minimal
   - Local variables (r, c, f, d, nr, nc, etc.) fit in registers
   - At `-O2`/`-O3`, register allocation minimizes spills to stack

### Stack-Based vs Heap-Based: Contrast with A*

```
A* Memory Access Pattern:          IDA* Memory Access Pattern:
+------------------+               +------------------+
| heap[] array     | random        | call stack       | sequential
| g_cost[] array   | random        |   frame N        | (push/pop)
| parent[] array   | random        |   frame N-1      |
| closed[] array   | random        |   ...            |
+------------------+               |   frame 0        |
                                   +------------------+
Cache behavior: scattered          Cache behavior: hot stack top
```

- **A* at `-O2`:** LLVM optimizes heap operations (bubble up/down), may
  vectorize array initialization, benefits from loop unrolling in the
  main while loop
- **IDA* at `-O2`:** LLVM focuses on minimizing the recursive function's
  frame size and maximizing register usage. The branch predictor matters
  more here (pruning branches dominate)

### Key Passes to Compare (via `opt -print-passes`)

| Pass              | Effect on A*          | Effect on IDA*             |
|-------------------|-----------------------|----------------------------|
| `inline`          | Inlines helpers+heap  | Inlines helpers only       |
| `sroa`            | Array decomposition   | Frame variable promotion   |
| `instcombine`     | Simplifies heap math  | Simplifies f-cost calc     |
| `loop-unroll`     | Unrolls direction loop| Unrolls 4-direction loop   |
| `simplifycfg`     | Cleans branching      | Critical: prune branches   |
| `tailcallelim`    | N/A                   | Cannot apply (not tail)    |
| `gvn`             | CSE in heap ops       | CSE across recursive setup |

### Comparing IR Output

```bash
# Generate unoptimized IR
clang -S -emit-llvm -O1 -Xclang -disable-llvm-optzns ida_star/ida_star.c -o ida_star/ida_star_O0.ll

# Compare search() function size across optimization levels
opt -S -O1 ida_star/ida_star_O0.ll -o ida_star/ida_star_opt_O1.ll
opt -S -O3 ida_star/ida_star_O0.ll -o ida_star/ida_star_opt_O3.ll

# Look at: function @search() in each .ll file
# O3 will show inlined helpers, unrolled direction loop,
# and aggressive register promotion
```
