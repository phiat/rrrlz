# rrrlz — LLVM Optimization Comparison Lab

# Build everything (LLVM pipeline + visualizer)
all: llvm visualizer

# Build all LLVM pipeline targets (hello, dijkstra, astar)
llvm:
    bash build_all.sh hello dijkstra astar

# Build individual targets through LLVM pipeline
hello:
    bash build_all.sh hello

dijkstra:
    bash build_all.sh dijkstra

astar:
    bash build_all.sh astar

# Build visualizer (SDL2, no LLVM pipeline)
visualizer:
    clang -O2 visualizer/visualizer.c -o visualizer/visualizer $(pkg-config --cflags --libs sdl2)

# Run visualizer
run: visualizer
    ./visualizer/visualizer

# Clean all build artifacts
clean:
    rm -f hello/hello_* dijkstra/dijkstra_* astar/astar_* visualizer/visualizer
