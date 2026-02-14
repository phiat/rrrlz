/*
 * algo_theta.c — Theta* step machine (any-angle pathfinding)
 *
 * A* with line-of-sight parent shortcuts. Expands 8-directional neighbors
 * but allows parent to be any ancestor with clear LOS. Uses euclidean100
 * for costs (×100 integer to avoid floats).
 */

#include "algo.h"

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    Heap heap;
    int cost[MAX_NODES];    /* g × 100 */
    int parent[MAX_NODES];
    int closed[MAX_NODES];
} ThetaState;

static ThetaState state;

static AlgoVis *theta_init(const MapDef *map) {
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
    int h = euclidean100(map->start_r, map->start_c, map->end_r, map->end_c);
    heap_push(&state.heap, start, h);

    return &state.vis;
}

/* Trace path through parent pointers (may skip cells), rasterize segments */
static void theta_trace_path(ThetaState *s) {
    int end = s->vis.end_node;
    int cols = s->vis.cols;
    s->vis.path_cost = s->cost[end];  /* ×100 euclidean */

    int cur = end;
    while (cur != -1) {
        int prev = s->parent[cur];
        if (prev != -1) {
            /* Rasterize line from prev to cur using Bresenham */
            int cr = cur / cols, cc = cur % cols;
            int pr = prev / cols, pc = prev % cols;

            int dr = cr - pr < 0 ? -(cr - pr) : (cr - pr);
            int dc = cc - pc < 0 ? -(cc - pc) : (cc - pc);
            int sr = pr < cr ? 1 : -1;
            int sc = pc < cc ? 1 : -1;
            int err = dr - dc;

            int ir = pr, ic = pc;
            while (ir != cr || ic != cc) {
                int idx = get_index(cols, ir, ic);
                if (idx != s->vis.start_node && idx != s->vis.end_node)
                    s->vis.cells[idx] = VIS_PATH;
                s->vis.path_len++;

                int e2 = 2 * err;
                if (e2 > -dc) { err -= dc; ir += sr; }
                if (e2 < dr) { err += dr; ic += sc; }
            }
        } else {
            s->vis.path_len++;
        }
        cur = prev;
    }
}

static int theta_step(AlgoVis *vis) {
    ThetaState *s = (ThetaState *)vis;
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
        theta_trace_path(s);
        return 1;
    }

    for (int d = 0; d < 8; d++) {
        int nr = r + DR8[d], nc = c + DC8[d];
        if (nr < 0 || nr >= s->map->rows || nc < 0 || nc >= s->map->cols) continue;
        if (s->map->data[nr * cols + nc] != 0) continue;

        /* For diagonal moves, check that both adjacent cardinal cells are passable */
        if (d >= 4) {
            int adj_r1 = r + DR8[d], adj_c1 = c;
            int adj_r2 = r, adj_c2 = c + DC8[d];
            if (s->map->data[adj_r1 * cols + adj_c1] != 0) continue;
            if (s->map->data[adj_r2 * cols + adj_c2] != 0) continue;
        }

        int neighbor = get_index(cols, nr, nc);
        if (s->closed[neighbor]) continue;

        /* Try Theta* shortcut: check LOS from parent[current] to neighbor */
        int par = s->parent[node];
        int used_shortcut = 0;

        if (par >= 0) {
            int pr = par / cols, pc = par % cols;
            if (line_of_sight(s->map, pr, pc, nr, nc)) {
                int new_g = s->cost[par] + euclidean100(pr, pc, nr, nc);
                if (new_g < s->cost[neighbor]) {
                    s->vis.relaxations++;
                    s->cost[neighbor] = new_g;
                    s->parent[neighbor] = par;
                    int h = euclidean100(nr, nc, s->map->end_r, s->map->end_c);
                    heap_push(&s->heap, neighbor, new_g + h);
                    if (neighbor != s->vis.start_node && neighbor != s->vis.end_node)
                        s->vis.cells[neighbor] = VIS_OPEN;
                    used_shortcut = 1;
                }
            }
        }

        if (!used_shortcut) {
            /* Standard A* relaxation */
            int new_g = s->cost[node] + euclidean100(r, c, nr, nc);
            if (new_g < s->cost[neighbor]) {
                s->vis.relaxations++;
                s->cost[neighbor] = new_g;
                s->parent[neighbor] = node;
                int h = euclidean100(nr, nc, s->map->end_r, s->map->end_c);
                heap_push(&s->heap, neighbor, new_g + h);
                if (neighbor != s->vis.start_node && neighbor != s->vis.end_node)
                    s->vis.cells[neighbor] = VIS_OPEN;
            }
        }
    }

    return 1;
}

AlgoPlugin algo_theta = {
    .name = "Theta*",
    .init = theta_init,
    .step = theta_step,
};
