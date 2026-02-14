/*
 * algo_anya.c — Bidirectional A* (ANYA fallback)
 *
 * Two A* frontiers meeting in the middle. Forward search from start,
 * backward search from goal. Visually distinct with two colored wavefronts.
 */

#include "algo.h"

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    Heap fwd_heap, bwd_heap;
    int fwd_cost[MAX_NODES], bwd_cost[MAX_NODES];
    int fwd_parent[MAX_NODES], bwd_parent[MAX_NODES];
    int fwd_closed[MAX_NODES], bwd_closed[MAX_NODES];
    int mu;         /* best known path cost */
    int meet_node;  /* node where frontiers meet */
    int fwd_turn;   /* 1 = forward turn, 0 = backward turn */
} BiAstarState;

static BiAstarState *state;

static AlgoVis *bidir_init(const MapDef *map) {
    free(state);
    state = calloc(1, sizeof(*state));
    state->map = map;
    vis_init_cells(&state->vis, map);
    heap_init(&state->fwd_heap);
    heap_init(&state->bwd_heap);

    int total = map->rows * map->cols;
    for (int i = 0; i < total; i++) {
        state->fwd_cost[i] = INT_MAX;
        state->bwd_cost[i] = INT_MAX;
        state->fwd_parent[i] = -1;
        state->bwd_parent[i] = -1;
    }

    int start = state->vis.start_node;
    int goal = state->vis.end_node;

    state->fwd_cost[start] = 0;
    state->bwd_cost[goal] = 0;

    int h_fwd = manhattan(map->start_r, map->start_c, map->end_r, map->end_c);
    int h_bwd = h_fwd;
    heap_push(&state->fwd_heap, start, h_fwd);
    heap_push(&state->bwd_heap, goal, h_bwd);

    state->mu = INT_MAX;
    state->meet_node = -1;
    state->fwd_turn = 1;

    return &state->vis;
}

static int bidir_step(AlgoVis *vis) {
    BiAstarState *s = (BiAstarState *)vis;
    if (s->vis.done) return 0;

    int cols = s->vis.cols;
    s->vis.steps++;

    /* Check termination */
    int min_key = INT_MAX;
    if (s->fwd_heap.size > 0 && s->fwd_heap.data[0].priority < min_key)
        min_key = s->fwd_heap.data[0].priority;
    if (s->bwd_heap.size > 0 && s->bwd_heap.data[0].priority < min_key)
        min_key = s->bwd_heap.data[0].priority;

    if (s->fwd_heap.size == 0 && s->bwd_heap.size == 0) {
        s->vis.done = 1;
        if (s->meet_node >= 0) goto found;
        return 0;
    }

    if (min_key >= s->mu && s->meet_node >= 0) {
        goto found;
    }

    if (s->fwd_turn && s->fwd_heap.size > 0) {
        /* Forward expansion */
        HeapEntry cur = heap_pop(&s->fwd_heap);
        int node = cur.node;
        s->fwd_turn = 0;

        if (s->fwd_closed[node]) return 1;
        s->fwd_closed[node] = 1;
        s->vis.nodes_explored++;

        if (node != s->vis.start_node && node != s->vis.end_node)
            s->vis.cells[node] = VIS_OPEN;  /* forward frontier color */

        /* Check if backward search has reached this node */
        if (s->bwd_cost[node] != INT_MAX) {
            int total_cost = s->fwd_cost[node] + s->bwd_cost[node];
            if (total_cost < s->mu) {
                s->mu = total_cost;
                s->meet_node = node;
            }
        }

        int r = node / cols, c = node % cols;
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (!is_valid(s->map, nr, nc)) continue;
            int neighbor = get_index(cols, nr, nc);
            if (s->fwd_closed[neighbor]) continue;

            int new_g = s->fwd_cost[node] + 1;
            if (new_g < s->fwd_cost[neighbor]) {
                s->vis.relaxations++;
                s->fwd_cost[neighbor] = new_g;
                s->fwd_parent[neighbor] = node;
                int h = manhattan(nr, nc, s->map->end_r, s->map->end_c);
                heap_push(&s->fwd_heap, neighbor, new_g + h);
            }
        }
    } else if (s->bwd_heap.size > 0) {
        /* Backward expansion */
        HeapEntry cur = heap_pop(&s->bwd_heap);
        int node = cur.node;
        s->fwd_turn = 1;

        if (s->bwd_closed[node]) return 1;
        s->bwd_closed[node] = 1;
        s->vis.nodes_explored++;

        if (node != s->vis.start_node && node != s->vis.end_node)
            s->vis.cells[node] = VIS_CLOSED;  /* backward frontier color */

        if (s->fwd_cost[node] != INT_MAX) {
            int total_cost = s->fwd_cost[node] + s->bwd_cost[node];
            if (total_cost < s->mu) {
                s->mu = total_cost;
                s->meet_node = node;
            }
        }

        int r = node / cols, c = node % cols;
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (!is_valid(s->map, nr, nc)) continue;
            int neighbor = get_index(cols, nr, nc);
            if (s->bwd_closed[neighbor]) continue;

            int new_g = s->bwd_cost[node] + 1;
            if (new_g < s->bwd_cost[neighbor]) {
                s->vis.relaxations++;
                s->bwd_cost[neighbor] = new_g;
                s->bwd_parent[neighbor] = node;
                int h = manhattan(nr, nc, s->map->start_r, s->map->start_c);
                heap_push(&s->bwd_heap, neighbor, new_g + h);
            }
        }
    } else {
        s->fwd_turn = !s->fwd_turn;
    }

    return 1;

found:
    s->vis.done = 1;
    s->vis.found = 1;
    s->vis.path_cost = s->mu;
    /* Trace forward path: meet_node → start */
    {
        int cur = s->meet_node;
        while (cur != -1) {
            if (cur != s->vis.start_node && cur != s->vis.end_node)
                s->vis.cells[cur] = VIS_PATH;
            s->vis.path_len++;
            cur = s->fwd_parent[cur];
        }
    }
    /* Trace backward path: meet_node → goal (skip meet_node, already counted) */
    {
        int cur = s->bwd_parent[s->meet_node];
        while (cur != -1) {
            if (cur != s->vis.start_node && cur != s->vis.end_node)
                s->vis.cells[cur] = VIS_PATH;
            s->vis.path_len++;
            cur = s->bwd_parent[cur];
        }
    }
    return 0;
}

AlgoPlugin algo_anya = {
    .name = "BiDir-A*",
    .init = bidir_init,
    .step = bidir_step,
};
