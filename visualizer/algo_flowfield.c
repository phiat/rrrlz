/*
 * algo_flowfield.c — Flow Field step machine
 *
 * Dijkstra from goal producing a vector field. Every cell points toward
 * its lowest-cost neighbor. Phase 1 builds the integration field, Phase 2
 * traces the path from start to goal following the flow directions.
 */

#include "algo.h"

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    Heap heap;
    int int_cost[MAX_NODES];   /* integration cost from goal */
    int flow_dir[MAX_NODES];   /* 0-3 cardinal direction, -1 = unset */
    int closed[MAX_NODES];
    int phase;                 /* 0 = integration, 1 = path extraction */
    int trace_node;            /* current position during path extraction */
} FlowFieldState;

static FlowFieldState *state;

static AlgoVis *flowfield_init(const MapDef *map) {
    free(state);
    state = calloc(1, sizeof(*state));
    state->map = map;
    vis_init_cells(&state->vis, map);
    heap_init(&state->heap);

    int total = map->rows * map->cols;
    for (int i = 0; i < total; i++) {
        state->int_cost[i] = INT_MAX;
        state->flow_dir[i] = -1;
    }

    /* Start Dijkstra from GOAL (reversed) */
    int goal = state->vis.end_node;
    state->int_cost[goal] = 0;
    heap_push(&state->heap, goal, 0);
    state->phase = 0;
    state->trace_node = -1;

    return &state->vis;
}

static int flowfield_step(AlgoVis *vis) {
    FlowFieldState *s = (FlowFieldState *)vis;
    if (s->vis.done) return 0;

    int cols = s->vis.cols;
    s->vis.steps++;

    if (s->phase == 0) {
        /* Phase 1: Integration — one Dijkstra expansion from goal */
        if (s->heap.size == 0) {
            /* Integration complete, compute flow directions */
            int total = s->map->rows * s->map->cols;
            for (int i = 0; i < total; i++) {
                if (s->int_cost[i] == INT_MAX) continue;
                int r = i / cols, c = i % cols;
                int best_cost = s->int_cost[i];
                int best_dir = -1;
                for (int d = 0; d < 4; d++) {
                    int nr = r + DR[d], nc = c + DC[d];
                    if (!is_valid(s->map, nr, nc)) continue;
                    int ni = get_index(cols, nr, nc);
                    if (s->int_cost[ni] < best_cost) {
                        best_cost = s->int_cost[ni];
                        best_dir = d;
                    }
                }
                s->flow_dir[i] = best_dir;
            }

            /* Check if start is reachable */
            if (s->int_cost[s->vis.start_node] == INT_MAX) {
                s->vis.done = 1;
                return 0;
            }

            s->phase = 1;
            s->trace_node = s->vis.start_node;
            s->vis.path_len = 1;  /* count start node */
            return 1;
        }

        HeapEntry cur = heap_pop(&s->heap);
        int node = cur.node;
        if (s->closed[node]) return 1;

        s->closed[node] = 1;
        s->vis.nodes_explored++;

        if (node != s->vis.start_node && node != s->vis.end_node)
            s->vis.cells[node] = VIS_OPEN;

        /* Check if we've reached the start (wave from goal hit start) */
        if (node == s->vis.start_node) {
            /* Mark closed but keep expanding for full flow field */
        }

        int r = node / cols, c = node % cols;
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (!is_valid(s->map, nr, nc)) continue;
            int neighbor = get_index(cols, nr, nc);
            if (s->closed[neighbor]) continue;

            int new_cost = s->int_cost[node] + 1;
            if (new_cost < s->int_cost[neighbor]) {
                s->vis.relaxations++;
                s->int_cost[neighbor] = new_cost;
                heap_push(&s->heap, neighbor, new_cost);
            }
        }

        return 1;
    } else {
        /* Phase 2: Path extraction — follow flow_dir from start to goal */
        int cur = s->trace_node;
        if (cur == s->vis.end_node) {
            s->vis.done = 1;
            s->vis.found = 1;
            s->vis.path_cost = s->int_cost[s->vis.start_node];
            return 0;
        }

        int dir = s->flow_dir[cur];
        if (dir < 0) {
            /* Stuck — no path */
            s->vis.done = 1;
            return 0;
        }

        int r = cur / cols, c = cur % cols;
        int nr = r + DR[dir], nc = c + DC[dir];
        int next = get_index(cols, nr, nc);

        if (next != s->vis.start_node && next != s->vis.end_node)
            s->vis.cells[next] = VIS_PATH;
        s->vis.path_len++;

        s->trace_node = next;
        return 1;
    }
}

AlgoPlugin algo_flowfield = {
    .name = "FlowField",
    .init = flowfield_init,
    .step = flowfield_step,
};
