# rrrlz — LLVM Optimization Comparison Lab

A collection of C programs built through the LLVM toolchain at multiple optimization levels to study how different passes transform code.

## Programs

| Directory | Algorithm | Description |
|---|---|---|
| `hello/` | Hello World | Minimal baseline — shows optimization pipeline with trivial code |
| `dijkstra/` | Dijkstra's Algorithm | Shortest path on a 20x20 grid, uniform expansion |
| `astar/` | A* Search | Heuristic-guided shortest path on the same grid |

## How It Works

Each program goes through a 4-stage LLVM pipeline:

```
source.c → clean IR (.ll) → optimized IR → assembly (.s) → binary
           clang -O1         opt -O{level}   llc              clang
           (no opts applied)
```

This is run at 5 optimization levels: **O1, O2, O3, Os, Oz**

The key insight: by generating clean (unoptimized) IR first and then running `opt` separately, you can see exactly what each optimization level changes in the IR — independent of clang's frontend decisions.

## Quick Start

```bash
# Build everything
bash build_all.sh

# Build one target
bash build_all.sh dijkstra
bash build_all.sh astar
bash build_all.sh hello

# Run
./dijkstra/dijkstra_opt_O2
./astar/astar_opt_O2
```

## Comparing Optimizations

```bash
# Compare IR between optimization levels
diff dijkstra/dijkstra_opt_O1.ll dijkstra/dijkstra_opt_O3.ll
diff astar/astar_opt_O1.ll astar/astar_opt_Os.ll

# Compare assembly
diff -u dijkstra/dijkstra_opt_O1.s dijkstra/dijkstra_opt_O3.s

# Side-by-side
vimdiff astar/astar_opt_O2.ll astar/astar_opt_O3.ll
```

## What to Look For

### O1 → O2
- Small helper functions get inlined (`get_index`, `is_valid`, `heuristic`)
- Basic loop optimizations (LICM, dead store elimination)

### O2 → O3
- Loop unrolling in heap operations
- More aggressive inlining thresholds
- Vectorization opportunities (if any)

### Os / Oz (size optimization)
- Functions stay outlined (less inlining)
- No loop unrolling
- Smaller instruction sequences preferred over faster ones
- Oz is more aggressive than Os about size

## Optimization-Visible Code Patterns

The pathfinding implementations deliberately include patterns that produce different IR at different levels:

1. **Small helper functions** — inlining threshold differences
2. **Nested loops** — unrolling and vectorization
3. **Array indexing** — strength reduction
4. **Multi-condition branches** — branch prediction, dead branch elimination
5. **Arithmetic** (heuristic) — constant propagation, algebraic simplification

## Project Structure

```
rrrlz/
├── README.md              # This file
├── build_all.sh           # Build script for all targets
├── hello/
│   └── hello.c            # Minimal baseline program
├── dijkstra/
│   ├── README.md
│   └── dijkstra.c         # Dijkstra's shortest path
└── astar/
    ├── README.md
    └── astar.c            # A* search with Manhattan heuristic
```

## Requirements

- `clang` (C compiler + IR generation)
- `opt` (LLVM optimizer)
- `llc` (LLVM static compiler — IR to assembly)

All part of the LLVM/Clang toolchain.
