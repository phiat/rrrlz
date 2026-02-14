/*
 * algo_dstar_lite.c — D* Lite step machine
 *
 * Incremental replanning algorithm. Searches backward (goal→start),
 * maintains g/rhs consistency. Supports replanning when walls are added
 * via the W key in the visualizer.
 */

#include "algo.h"

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    int map_data[MAX_NODES]; /* mutable copy of map data */
    Heap heap;
    int g[MAX_NODES];
    int rhs[MAX_NODES];
    int parent[MAX_NODES];
    int in_heap[MAX_NODES];
    int km;  /* key modifier for replanning */
    int phase;  /* 0 = initial search, 1 = path found, 2 = replanning */
} DStarState;

static DStarState *state;

static int dstar_key(DStarState *s, int node) {
    int g = s->g[node], rhs = s->rhs[node];
    int mn = g < rhs ? g : rhs;
    if (mn == INT_MAX) return INT_MAX;
    int r = node / s->vis.cols, c = node % s->vis.cols;
    int sr = s->vis.start_node / s->vis.cols;
    int sc = s->vis.start_node % s->vis.cols;
    int h = manhattan(r, c, sr, sc);
    return mn + h + s->km;
}

static void dstar_update_node(DStarState *s, int node) {
    int cols = s->vis.cols;
    int r = node / cols, c = node % cols;

    if (node != s->vis.end_node) {
        /* rhs = min over successors (g[succ] + cost) */
        int best = INT_MAX;
        int best_parent = -1;
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (nr < 0 || nr >= s->map->rows || nc < 0 || nc >= s->map->cols) continue;
            int ni = get_index(cols, nr, nc);
            if (s->map_data[ni] != 0) continue;  /* wall */
            if (s->g[ni] != INT_MAX && s->g[ni] + 1 < best) {
                best = s->g[ni] + 1;
                best_parent = ni;
            }
        }
        s->rhs[node] = best;
        if (best_parent >= 0) s->parent[node] = best_parent;
    }

    /* Remove from heap if present, re-add if inconsistent */
    /* (we use a lazy heap, so just push new entry) */
    if (s->g[node] != s->rhs[node]) {
        int key = dstar_key(s, node);
        if (key != INT_MAX) {
            heap_push(&s->heap, node, key);
            s->in_heap[node] = 1;
        }
    }
}

static AlgoVis *dstar_init(const MapDef *map) {
    free(state);
    state = calloc(1, sizeof(*state));
    state->map = map;
    vis_init_cells(&state->vis, map);
    heap_init(&state->heap);

    int total = map->rows * map->cols;
    /* Mutable copy of map data */
    for (int i = 0; i < total; i++)
        state->map_data[i] = map->data[i];

    for (int i = 0; i < total; i++) {
        state->g[i] = INT_MAX;
        state->rhs[i] = INT_MAX;
        state->parent[i] = -1;
    }

    /* Goal node: rhs = 0 */
    int goal = state->vis.end_node;
    state->rhs[goal] = 0;
    state->km = 0;
    state->phase = 0;

    int key = dstar_key(state, goal);
    heap_push(&state->heap, goal, key);
    state->in_heap[goal] = 1;

    return &state->vis;
}

static int dstar_step(AlgoVis *vis) {
    DStarState *s = (DStarState *)vis;
    if (s->vis.done) return 0;

    int cols = s->vis.cols;
    s->vis.steps++;

    int start = s->vis.start_node;

    if (s->heap.size == 0) {
        /* Check if start is consistent */
        if (s->g[start] == s->rhs[start] && s->g[start] != INT_MAX) {
            goto trace_path;
        }
        s->vis.done = 1;
        return 0;
    }

    int start_key = dstar_key(s, start);

    /* Check termination: start is consistent and min key >= start key */
    if (s->rhs[start] == s->g[start] && s->g[start] != INT_MAX) {
        if (s->heap.size == 0 || s->heap.data[0].priority >= start_key) {
            goto trace_path;
        }
    }

    HeapEntry cur = heap_pop(&s->heap);
    int node = cur.node;
    s->in_heap[node] = 0;

    /* Skip stale entries */
    int cur_key = dstar_key(s, node);
    if (cur.priority > cur_key && s->g[node] != s->rhs[node]) {
        heap_push(&s->heap, node, cur_key);
        s->in_heap[node] = 1;
        return 1;
    }

    s->vis.nodes_explored++;

    if (node != s->vis.start_node && node != s->vis.end_node)
        s->vis.cells[node] = VIS_CLOSED;

    int r = node / cols, c = node % cols;

    if (s->g[node] > s->rhs[node]) {
        /* Overconsistent: make consistent */
        s->g[node] = s->rhs[node];
        /* Update predecessors */
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (nr < 0 || nr >= s->map->rows || nc < 0 || nc >= s->map->cols) continue;
            int ni = get_index(cols, nr, nc);
            if (s->map_data[ni] != 0) continue;
            s->vis.relaxations++;
            dstar_update_node(s, ni);
            if (ni != s->vis.start_node && ni != s->vis.end_node &&
                s->vis.cells[ni] != VIS_CLOSED)
                s->vis.cells[ni] = VIS_OPEN;
        }
    } else {
        /* Underconsistent: reset and update */
        s->g[node] = INT_MAX;
        dstar_update_node(s, node);
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (nr < 0 || nr >= s->map->rows || nc < 0 || nc >= s->map->cols) continue;
            int ni = get_index(cols, nr, nc);
            if (s->map_data[ni] != 0) continue;
            s->vis.relaxations++;
            dstar_update_node(s, ni);
        }
    }

    return 1;

trace_path:
    s->vis.done = 1;
    s->vis.found = 1;
    s->vis.path_cost = s->g[start];
    /* Trace path: follow rhs-optimal neighbors from start to goal */
    {
        int cur_node = start;
        while (cur_node != s->vis.end_node && cur_node != -1) {
            if (cur_node != s->vis.start_node && cur_node != s->vis.end_node)
                s->vis.cells[cur_node] = VIS_PATH;
            s->vis.path_len++;
            int cr = cur_node / cols, cc = cur_node % cols;
            int best = INT_MAX;
            int best_n = -1;
            for (int d = 0; d < 4; d++) {
                int nr = cr + DR[d], nc = cc + DC[d];
                if (nr < 0 || nr >= s->map->rows || nc < 0 || nc >= s->map->cols) continue;
                int ni = get_index(cols, nr, nc);
                if (s->map_data[ni] != 0) continue;
                if (s->g[ni] < best) {
                    best = s->g[ni];
                    best_n = ni;
                }
            }
            cur_node = best_n;
        }
        if (cur_node == s->vis.end_node) s->vis.path_len++;
    }
    return 0;
}

/* Called by visualizer when a wall is toggled */
void dstar_notify_change(DStarState *s, int node) {
    int cols = s->vis.cols;
    int r = node / cols, c = node % cols;

    /* Update affected neighbors */
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (nr < 0 || nr >= s->map->rows || nc < 0 || nc >= s->map->cols) continue;
        int ni = get_index(cols, nr, nc);
        if (s->map_data[ni] != 0) continue;
        dstar_update_node(s, ni);
    }

    /* Also update the changed node itself */
    if (s->map_data[node] == 0)
        dstar_update_node(s, node);

    s->vis.done = 0;
    s->vis.found = 0;
    s->vis.path_len = 0;
    s->vis.path_cost = 0;

    /* Clear path cells */
    int total = s->map->rows * s->map->cols;
    for (int i = 0; i < total; i++) {
        if (s->vis.cells[i] == VIS_PATH)
            s->vis.cells[i] = VIS_CLOSED;
    }
}

/* Get mutable map data pointer for wall toggles */
int *dstar_get_map_data(void) { return state ? state->map_data : NULL; }
DStarState *dstar_get_state(void) { return state; }

AlgoPlugin algo_dstar_lite = {
    .name = "D*Lite",
    .init = dstar_init,
    .step = dstar_step,
};
