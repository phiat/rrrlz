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
    clang -O2 visualizer/visualizer.c \
        visualizer/algo_dijkstra.c visualizer/algo_astar.c \
        visualizer/algo_bellman_ford.c visualizer/algo_ida_star.c \
        visualizer/algo_floyd_warshall.c visualizer/algo_jps.c \
        visualizer/algo_fringe.c visualizer/algo_flowfield.c \
        visualizer/algo_dstar_lite.c visualizer/algo_theta.c \
        visualizer/algo_rsr.c visualizer/algo_subgoal.c \
        visualizer/algo_ch.c visualizer/algo_anya.c \
        -o visualizer/visualizer $(pkg-config --cflags --libs sdl2) -lm

# Build visualizer with all warnings
check:
    clang -Wall -Wextra -O2 visualizer/visualizer.c \
        visualizer/algo_dijkstra.c visualizer/algo_astar.c \
        visualizer/algo_bellman_ford.c visualizer/algo_ida_star.c \
        visualizer/algo_floyd_warshall.c visualizer/algo_jps.c \
        visualizer/algo_fringe.c visualizer/algo_flowfield.c \
        visualizer/algo_dstar_lite.c visualizer/algo_theta.c \
        visualizer/algo_rsr.c visualizer/algo_subgoal.c \
        visualizer/algo_ch.c visualizer/algo_anya.c \
        -o visualizer/visualizer $(pkg-config --cflags --libs sdl2) -lm

# Run visualizer
run: visualizer
    ./visualizer/visualizer

# List all algorithm source files
algo-files:
    @ls -1 visualizer/algo_*.c

# Clean all build artifacts
clean:
    rm -f hello/hello_* dijkstra/dijkstra_* astar/astar_* bellman_ford/bellman_ford_* floyd_warshall/floyd_warshall_* ida_star/ida_star_* visualizer/visualizer
