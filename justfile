# rrrlz — LLVM Optimization Comparison Lab

# Build everything (LLVM pipeline + visualizer)
all: llvm visualizer

# Build all LLVM pipeline targets (hello, dijkstra, astar, bellman_ford, floyd_warshall, ida_star)
llvm:
    bash build_all.sh hello dijkstra astar bellman_ford floyd_warshall ida_star

# Build individual targets through LLVM pipeline
hello:
    bash build_all.sh hello

dijkstra:
    bash build_all.sh dijkstra

astar:
    bash build_all.sh astar

bellman_ford:
    bash build_all.sh bellman_ford

floyd_warshall:
    bash build_all.sh floyd_warshall

ida_star:
    bash build_all.sh ida_star

# Build visualizer (SDL2, no LLVM pipeline)
visualizer:
    clang -O2 visualizer/visualizer.c -o visualizer/visualizer $(pkg-config --cflags --libs sdl2)

# Run visualizer
run: visualizer
    ./visualizer/visualizer

# Clean all build artifacts
clean:
    rm -f hello/hello_* dijkstra/dijkstra_* astar/astar_* bellman_ford/bellman_ford_* floyd_warshall/floyd_warshall_* ida_star/ida_star_* visualizer/visualizer
