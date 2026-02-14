/*
 * algo_jps.c — Jump Point Search step machine (4-directional)
 *
 * JPS prunes the A* search space by "jumping" over intermediate nodes
 * in straight lines, only adding nodes to the open set when a forced
 * neighbor is found (wall opens up perpendicular to movement) or when
 * the goal is reached.
 *
 * Each step = one node expansion (pop from heap, jump in all 4 dirs).
 */

#include "algo.h"

typedef struct {
    AlgoVis vis;
    Heap heap;
    int cost[MAX_NODES];
    int parent[MAX_NODES];
    int closed[MAX_NODES];
    const MapDef *map;
} JPSState;

static JPSState state;

static void jps_trace_path(JPSState *s);

/* Jump iteratively in direction (dr,dc) from (r,c), coloring intermediate cells */
static int jps_jump_iter(JPSState *s, int r, int c, int dr, int dc) {
    const MapDef *map = s->map;
    int cols = map->cols;
    int end_node = s->vis.end_node;

    int cr = r, cc = c;
    for (;;) {
        int nr = cr + dr;
        int nc = cc + dc;

        if (!is_valid(map, nr, nc)) {
            /* Hit wall/boundary: return last valid cell as jump point
             * if it has perpendicular neighbors to explore */
            if (cr != r || cc != c) {
                int p1r = dc, p1c = -dr;
                int p2r = -dc, p2c = dr;
                if (is_valid(map, cr + p1r, cc + p1c) ||
                    is_valid(map, cr + p2r, cc + p2c))
                    return get_index(cols, cr, cc);
            }
            return -1;
        }

        int idx = get_index(cols, nr, nc);

        /* Color intermediate jumped cells */
        if (idx != s->vis.start_node && idx != s->vis.end_node &&
            s->vis.cells[idx] == VIS_EMPTY)
            s->vis.cells[idx] = VIS_OPEN;

        if (idx == end_node)
            return idx;

        /* Check forced neighbors */
        int p1r = dc, p1c = -dr;
        int p2r = -dc, p2c = dr;

        if (is_valid(map, nr + p1r, nc + p1c) &&
            !is_valid(map, nr + p1r - dr, nc + p1c - dc))
            return idx;

        if (is_valid(map, nr + p2r, nc + p2c) &&
            !is_valid(map, nr + p2r - dr, nc + p2c - dc))
            return idx;

        cr = nr;
        cc = nc;
    }
}

static AlgoVis *jps_init(const MapDef *map) {
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

static int jps_step(AlgoVis *vis) {
    JPSState *s = (JPSState *)vis;
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
        /* Custom path trace: jump points may not be adjacent */
        jps_trace_path(s);
        return 1;
    }

    /* Jump in all 4 cardinal directions */
    for (int d = 0; d < 4; d++) {
        int jp = jps_jump_iter(s, r, c, DR[d], DC[d]);
        if (jp < 0) continue;

        if (s->closed[jp]) continue;

        int jr = jp / cols, jc = jp % cols;
        /* Cost = manhattan distance between current and jump point (straight line) */
        int jump_cost = (jr == r) ? (jc > c ? jc - c : c - jc) : (jr > r ? jr - r : r - jr);
        int new_g = s->cost[node] + jump_cost;

        if (new_g < s->cost[jp]) {
            s->vis.relaxations++;
            s->cost[jp] = new_g;
            s->parent[jp] = node;
            int h = manhattan(jr, jc, s->map->end_r, s->map->end_c);
            heap_push(&s->heap, jp, new_g + h);
        }
    }

    return 1;
}

/* Trace path through jump points, filling intermediate cells */
static void jps_trace_path(JPSState *s) {
    int end = s->vis.end_node;
    int cols = s->vis.cols;
    s->vis.path_cost = s->cost[end];

    int cur = end;
    while (cur != -1) {
        int prev = s->parent[cur];
        if (prev != -1) {
            /* Fill intermediate cells between prev and cur */
            int cr = cur / cols, cc = cur % cols;
            int pr = prev / cols, pc = prev % cols;
            int dr = 0, dc = 0;
            if (cr < pr) dr = 1; else if (cr > pr) dr = -1;
            if (cc < pc) dc = 1; else if (cc > pc) dc = -1;

            int ir = cr, ic = cc;
            while (ir != pr || ic != pc) {
                int idx = get_index(cols, ir, ic);
                if (idx != s->vis.start_node && idx != s->vis.end_node)
                    s->vis.cells[idx] = VIS_PATH;
                s->vis.path_len++;
                ir += dr;
                ic += dc;
            }
        } else {
            /* Start node */
            s->vis.path_len++;
        }
        cur = prev;
    }
}

AlgoPlugin algo_jps = {
    .name = "JPS",
    .init = jps_init,
    .step = jps_step,
};
