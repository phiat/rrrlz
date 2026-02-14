/*
 * algo_fringe.c — Fringe Search step machine
 *
 * IDA* variant that preserves the frontier between iterations instead
 * of restarting from scratch. Uses two lists (now, later) implemented
 * as arrays with prev/next indices.
 */

#include "algo.h"

typedef struct {
    int prev, next;
    int f;        /* cached f-value */
    int g;
    int in_list;  /* 0=none, 1=now, 2=later */
} FringeNode;

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    FringeNode nodes[MAX_NODES];
    int parent[MAX_NODES];
    int threshold;
    int next_threshold;
    int now_head;   /* head of 'now' list (-1 = empty) */
    int later_head; /* head of 'later' list (-1 = empty) */
    int phase;      /* 0 = searching, 1 = done */
} FringeState;

static FringeState *state;

/* Doubly-linked list helpers */
static void list_remove(FringeState *s, int node) {
    FringeNode *n = &s->nodes[node];
    int p = n->prev, nx = n->next;
    if (p >= 0) s->nodes[p].next = nx;
    if (nx >= 0) s->nodes[nx].prev = p;
    /* Update list heads */
    if (n->in_list == 1 && s->now_head == node) s->now_head = nx;
    if (n->in_list == 2 && s->later_head == node) s->later_head = nx;
    n->prev = n->next = -1;
    n->in_list = 0;
}

static void list_prepend_now(FringeState *s, int node) {
    FringeNode *n = &s->nodes[node];
    n->prev = -1;
    n->next = s->now_head;
    if (s->now_head >= 0) s->nodes[s->now_head].prev = node;
    s->now_head = node;
    n->in_list = 1;
}

static void list_prepend_later(FringeState *s, int node) {
    FringeNode *n = &s->nodes[node];
    n->prev = -1;
    n->next = s->later_head;
    if (s->later_head >= 0) s->nodes[s->later_head].prev = node;
    s->later_head = node;
    n->in_list = 2;
}

static AlgoVis *fringe_init(const MapDef *map) {
    free(state);
    state = calloc(1, sizeof(*state));
    state->map = map;
    vis_init_cells(&state->vis, map);

    int total = map->rows * map->cols;
    for (int i = 0; i < total; i++) {
        state->nodes[i].prev = -1;
        state->nodes[i].next = -1;
        state->nodes[i].f = INT_MAX;
        state->nodes[i].g = INT_MAX;
        state->nodes[i].in_list = 0;
        state->parent[i] = -1;
    }

    state->now_head = -1;
    state->later_head = -1;
    state->next_threshold = INT_MAX;

    int start = state->vis.start_node;
    int h = manhattan(map->start_r, map->start_c, map->end_r, map->end_c);
    state->nodes[start].g = 0;
    state->nodes[start].f = h;
    state->threshold = h;

    list_prepend_now(state, start);

    return &state->vis;
}

static int fringe_step(AlgoVis *vis) {
    FringeState *s = (FringeState *)vis;
    if (s->vis.done) return 0;

    /* If now list is empty, swap later → now */
    if (s->now_head < 0) {
        if (s->later_head < 0 || s->next_threshold == INT_MAX) {
            s->vis.done = 1;
            return 0;
        }
        s->threshold = s->next_threshold;
        s->next_threshold = INT_MAX;
        s->now_head = s->later_head;
        s->later_head = -1;
        /* Relabel all later nodes as now */
        int cur = s->now_head;
        while (cur >= 0) {
            s->nodes[cur].in_list = 1;
            cur = s->nodes[cur].next;
        }
        return 1;
    }

    int node = s->now_head;
    int cols = s->vis.cols;
    s->vis.steps++;

    /* If f > threshold, move to later */
    if (s->nodes[node].f > s->threshold) {
        int f = s->nodes[node].f;
        if (f < s->next_threshold) s->next_threshold = f;
        int nx = s->nodes[node].next;
        list_remove(s, node);
        list_prepend_later(s, node);
        (void)nx;
        return 1;
    }

    /* Expand node */
    list_remove(s, node);
    s->vis.nodes_explored++;

    if (node != s->vis.start_node && node != s->vis.end_node)
        s->vis.cells[node] = VIS_CLOSED;

    if (node == s->vis.end_node) {
        s->vis.done = 1;
        s->vis.found = 1;
        /* Trace path */
        s->vis.path_cost = s->nodes[node].g;
        int cur = node;
        while (cur != -1) {
            if (cur != s->vis.start_node && cur != s->vis.end_node)
                s->vis.cells[cur] = VIS_PATH;
            s->vis.path_len++;
            cur = s->parent[cur];
        }
        return 1;
    }

    int r = node / cols, c = node % cols;
    int g = s->nodes[node].g;

    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!is_valid(s->map, nr, nc)) continue;
        int neighbor = get_index(cols, nr, nc);

        int new_g = g + 1;
        if (new_g >= s->nodes[neighbor].g) continue;

        s->vis.relaxations++;
        s->nodes[neighbor].g = new_g;
        int h = manhattan(nr, nc, s->map->end_r, s->map->end_c);
        s->nodes[neighbor].f = new_g + h;
        s->parent[neighbor] = node;

        /* Remove from whatever list it's in, prepend to now */
        if (s->nodes[neighbor].in_list)
            list_remove(s, neighbor);
        list_prepend_now(s, neighbor);

        if (neighbor != s->vis.start_node && neighbor != s->vis.end_node)
            s->vis.cells[neighbor] = VIS_OPEN;
    }

    return 1;
}

AlgoPlugin algo_fringe = {
    .name = "Fringe",
    .init = fringe_init,
    .step = fringe_step,
};
