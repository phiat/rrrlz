/*
 * algo_astar.c — A* algorithm step machine
 */

#include "algo.h"

typedef struct {
    AlgoVis vis;
    Heap heap;
    int cost[MAX_NODES];
    int parent[MAX_NODES];
    int closed[MAX_NODES];
    const MapDef *map;
} AstarState;

static AstarState state;

static AlgoVis *astar_init(const MapDef *map) {
    memset(&state, 0, sizeof(state));
    state.map = map;
    vis_init_cells(&state.vis, map);
    heap_init(&state.heap);

    int total = map->rows * map->cols;
    for (int i = 0; i < total; i++) {
        state.cost[i] = INT_MAX;
        state.parent[i] = -1;
    }

    int start = state.vis.start_node;
    state.cost[start] = 0;
    heap_push(&state.heap, start,
              manhattan(map->start_r, map->start_c, map->end_r, map->end_c));

    return &state.vis;
}

static int astar_step(AlgoVis *vis) {
    AstarState *s = (AstarState *)vis;
    if (s->vis.done) return 0;
    if (s->heap.size == 0) { s->vis.done = 1; return 0; }

    HeapEntry cur = heap_pop(&s->heap);
    int node = cur.node;
    int cols = s->vis.cols;
    int r = node / cols, c = node % cols;
    s->vis.steps++;

    if (s->closed[node]) return 1;

    s->closed[node] = 1;
    s->vis.nodes_explored++;

    if (node != s->vis.start_node && node != s->vis.end_node)
        s->vis.cells[node] = VIS_CLOSED;

    if (node == s->vis.end_node) {
        s->vis.done = 1;
        s->vis.found = 1;
        vis_trace_path(&s->vis, s->parent, s->cost);
        return 1;
    }

    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!is_valid(s->map, nr, nc)) continue;
        int neighbor = get_index(cols, nr, nc);
        if (s->closed[neighbor]) continue;

        int new_g = s->cost[node] + 1;
        if (new_g < s->cost[neighbor]) {
            s->vis.relaxations++;
            s->cost[neighbor] = new_g;
            s->parent[neighbor] = node;
            heap_push(&s->heap, neighbor,
                      new_g + manhattan(nr, nc, s->map->end_r, s->map->end_c));

            if (neighbor != s->vis.start_node &&
                neighbor != s->vis.end_node)
                s->vis.cells[neighbor] = VIS_OPEN;
        }
    }

    return 1;
}

AlgoPlugin algo_astar = {
    .name = "A*",
    .init = astar_init,
    .step = astar_step,
};
