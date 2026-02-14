/*
 * algo_bellman_ford.c — Bellman-Ford step machine
 *
 * Builds an edge list at init, then relaxes all edges in one full pass per step.
 * Checks for changes after each pass and exits early if none.
 */

#include "algo.h"

typedef struct {
    int from;
    int to;
} Edge;

/* Max edges: each cell has up to 4 neighbors, stored directed */
#define MAX_EDGES (MAX_NODES * 4)

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    Edge edges[MAX_EDGES];
    int edge_count;
    int cost[MAX_NODES];
    int parent[MAX_NODES];
    int reached[MAX_NODES];   /* has this node ever been reached? */
    int bf_iter;              /* current pass number (0..V-1) */
    int bf_changed;           /* did this pass relax anything? */
    int total_nodes;          /* rows * cols for this map */
} BellmanFordState;

static BellmanFordState state;

static AlgoVis *bellman_ford_init(const MapDef *map) {
    memset(&state, 0, sizeof(state));
    state.map = map;
    vis_init_cells(&state.vis, map);
    state.total_nodes = map->rows * map->cols;

    for (int i = 0; i < state.total_nodes; i++) {
        state.cost[i] = INT_MAX;
        state.parent[i] = -1;
    }

    /* Build edge list */
    int cols = map->cols;
    state.edge_count = 0;
    for (int r = 0; r < map->rows; r++) {
        for (int c = 0; c < map->cols; c++) {
            if (map->data[r * cols + c]) continue;
            int u = get_index(cols, r, c);
            for (int d = 0; d < 4; d++) {
                int nr = r + DR[d], nc = c + DC[d];
                if (!is_valid(map, nr, nc)) continue;
                state.edges[state.edge_count].from = u;
                state.edges[state.edge_count].to = get_index(cols, nr, nc);
                state.edge_count++;
            }
        }
    }

    int start = state.vis.start_node;
    state.cost[start] = 0;
    state.reached[start] = 1;

    state.bf_iter = 0;
    state.bf_changed = 0;

    return &state.vis;
}

static int bellman_ford_step(AlgoVis *vis) {
    BellmanFordState *s = (BellmanFordState *)vis;
    if (s->vis.done) return 0;

    /* No edges means no paths possible */
    if (s->edge_count == 0) { s->vis.done = 1; return 0; }

    /* One full pass over all edges per step */
    s->bf_changed = 0;
    for (int ei = 0; ei < s->edge_count; ei++) {
        Edge *e = &s->edges[ei];
        if (s->cost[e->from] == INT_MAX) continue;
        int new_cost = s->cost[e->from] + 1;
        if (new_cost < s->cost[e->to]) {
            s->vis.relaxations++;
            s->cost[e->to] = new_cost;
            s->parent[e->to] = e->from;
            s->bf_changed = 1;

            if (!s->reached[e->to]) {
                s->reached[e->to] = 1;
                s->vis.nodes_explored++;
            }

            /* Color newly reached node */
            if (e->to != s->vis.start_node &&
                e->to != s->vis.end_node)
                s->vis.cells[e->to] = VIS_OPEN;
        }
    }

    s->vis.steps++;
    s->bf_iter++;

    if (!s->bf_changed || s->bf_iter >= s->total_nodes - 1) {
        /* Done — mark all reached nodes as closed */
        s->vis.done = 1;
        for (int i = 0; i < s->total_nodes; i++) {
            if (s->reached[i] &&
                i != s->vis.start_node &&
                i != s->vis.end_node)
                s->vis.cells[i] = VIS_CLOSED;
        }

        int end = s->vis.end_node;
        if (s->cost[end] != INT_MAX) {
            s->vis.found = 1;
            vis_trace_path(&s->vis, s->parent, s->cost);
        }
        return 1;
    }

    return 1;
}

AlgoPlugin algo_bellman_ford = {
    .name = "Bellman-Ford",
    .init = bellman_ford_init,
    .step = bellman_ford_step,
};
