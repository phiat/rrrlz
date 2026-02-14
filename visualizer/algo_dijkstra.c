/*
 * algo_dijkstra.c — Dijkstra's algorithm step machine
 */

#include "algo.h"

typedef struct {
    AlgoVis vis;
    Heap heap;
    int cost[MAX_NODES];
    int parent[MAX_NODES];
    int closed[MAX_NODES];
    const int (*map)[COLS];
} DijkstraState;

static DijkstraState state;

static AlgoVis *dijkstra_init(const int (*map)[COLS]) {
    memset(&state, 0, sizeof(state));
    state.map = map;
    vis_init_cells(&state.vis, map);
    heap_init(&state.heap);

    for (int i = 0; i < MAX_NODES; i++) {
        state.cost[i] = INT_MAX;
        state.parent[i] = -1;
    }

    int start = get_index(START_R, START_C);
    state.cost[start] = 0;
    heap_push(&state.heap, start, 0);

    return &state.vis;
}

static int dijkstra_step(AlgoVis *vis) {
    DijkstraState *s = (DijkstraState *)vis;
    if (s->vis.done) return 0;
    if (s->heap.size == 0) { s->vis.done = 1; return 0; }

    HeapEntry cur = heap_pop(&s->heap);
    int node = cur.node;
    int r = node / COLS, c = node % COLS;
    s->vis.steps++;

    if (s->closed[node]) return 1; /* stale entry */

    s->closed[node] = 1;
    s->vis.nodes_explored++;

    if (node != get_index(START_R, START_C) && node != get_index(END_R, END_C))
        s->vis.cells[node] = VIS_CLOSED;

    int end = get_index(END_R, END_C);
    if (node == end) {
        s->vis.done = 1;
        s->vis.found = 1;
        vis_trace_path(&s->vis, s->parent, s->cost);
        return 1;
    }

    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!is_valid(s->map, nr, nc)) continue;
        int neighbor = get_index(nr, nc);
        if (s->closed[neighbor]) continue;

        int new_g = s->cost[node] + 1;
        if (new_g < s->cost[neighbor]) {
            s->cost[neighbor] = new_g;
            s->parent[neighbor] = node;
            heap_push(&s->heap, neighbor, new_g);

            if (neighbor != get_index(START_R, START_C) &&
                neighbor != get_index(END_R, END_C))
                s->vis.cells[neighbor] = VIS_OPEN;
        }
    }

    return 1;
}

AlgoPlugin algo_dijkstra = {
    .name = "Dijkstra",
    .init = dijkstra_init,
    .step = dijkstra_step,
};
