/*
 * algo_rsr.c — Rectangular Symmetry Reduction step machine
 *
 * Phase 1: Decompose grid into maximal empty rectangles using greedy scan.
 * Phase 2: Run A* on perimeter nodes only, with macro-edges crossing rects.
 */

#include "algo.h"

#define MAX_RECTS (MAX_NODES / 4)

typedef struct {
    int r1, c1, r2, c2;  /* top-left and bottom-right */
    int id;
} RSRRect;

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    /* Phase 1: rectangle decomposition */
    RSRRect rects[MAX_RECTS];
    int rect_count;
    int rect_id[MAX_NODES];    /* which rect each cell belongs to, -1 = none/wall */
    int assigned[MAX_NODES];   /* already assigned to a rect */
    int scan_r, scan_c;       /* current scan position for decomposition */
    int phase;                 /* 0 = decomposition, 1 = A* search, 2 = done */
    /* Phase 2: A* on perimeter */
    Heap heap;
    int cost[MAX_NODES];
    int parent[MAX_NODES];
    int closed[MAX_NODES];
    int is_perimeter[MAX_NODES]; /* 1 if on rect perimeter */
} RSRState;

static RSRState state;

/* Try to grow a maximal rectangle starting at (r,c) */
static int rsr_grow_rect(RSRState *s, int sr, int sc, RSRRect *out) {
    const MapDef *map = s->map;
    int max_c = map->cols;

    /* Extend right as far as possible on first row */
    int ec = sc;
    while (ec < max_c && !s->assigned[sr * map->cols + ec] &&
           map->data[sr * map->cols + ec] == 0)
        ec++;
    ec--;
    if (ec < sc) return 0;

    /* Extend down while full row is free */
    int er = sr;
    for (int r = sr + 1; r < map->rows; r++) {
        int ok = 1;
        for (int c = sc; c <= ec; c++) {
            int idx = r * map->cols + c;
            if (s->assigned[idx] || map->data[idx] != 0) { ok = 0; break; }
        }
        if (!ok) break;
        er = r;
    }

    out->r1 = sr; out->c1 = sc;
    out->r2 = er; out->c2 = ec;
    return 1;
}

static void rsr_mark_perimeter(RSRState *s) {
    int cols = s->map->cols;
    int total = s->map->rows * s->map->cols;
    memset(s->is_perimeter, 0, total * sizeof(int));

    for (int i = 0; i < s->rect_count; i++) {
        RSRRect *r = &s->rects[i];
        /* Mark all perimeter cells */
        for (int c = r->c1; c <= r->c2; c++) {
            s->is_perimeter[r->r1 * cols + c] = 1;
            s->is_perimeter[r->r2 * cols + c] = 1;
        }
        for (int row = r->r1; row <= r->r2; row++) {
            s->is_perimeter[row * cols + r->c1] = 1;
            s->is_perimeter[row * cols + r->c2] = 1;
        }
    }
    /* Start and end are always searchable */
    s->is_perimeter[s->vis.start_node] = 1;
    s->is_perimeter[s->vis.end_node] = 1;
}

static AlgoVis *rsr_init(const MapDef *map) {
    memset(&state, 0, sizeof(state));
    state.map = map;
    vis_init_cells(&state.vis, map);
    heap_init(&state.heap);

    int total = map->rows * map->cols;
    for (int i = 0; i < total; i++) {
        state.rect_id[i] = -1;
        state.cost[i] = INT_MAX;
        state.parent[i] = -1;
    }
    state.phase = 0;
    state.scan_r = 0;
    state.scan_c = 0;

    return &state.vis;
}

static int rsr_step(AlgoVis *vis) {
    RSRState *s = (RSRState *)vis;
    if (s->vis.done) return 0;

    int cols = s->vis.cols;
    s->vis.steps++;

    if (s->phase == 0) {
        /* Phase 1: Find next maximal rectangle */
        const MapDef *map = s->map;

        while (s->scan_r < map->rows) {
            int idx = s->scan_r * cols + s->scan_c;
            if (!s->assigned[idx] && map->data[idx] == 0) {
                RSRRect rect;
                if (rsr_grow_rect(s, s->scan_r, s->scan_c, &rect) &&
                    s->rect_count < MAX_RECTS) {
                    rect.id = s->rect_count;
                    s->rects[s->rect_count] = rect;

                    /* Mark cells */
                    for (int r = rect.r1; r <= rect.r2; r++) {
                        for (int c = rect.c1; c <= rect.c2; c++) {
                            int ci = r * cols + c;
                            s->assigned[ci] = 1;
                            s->rect_id[ci] = rect.id;
                            /* Interior cells = preprocess, perimeter = open */
                            int is_edge = (r == rect.r1 || r == rect.r2 ||
                                           c == rect.c1 || c == rect.c2);
                            if (ci != s->vis.start_node && ci != s->vis.end_node) {
                                s->vis.cells[ci] = is_edge ? VIS_OPEN : VIS_PREPROCESS;
                            }
                        }
                    }
                    s->rect_count++;

                    /* Advance scan */
                    s->scan_c = rect.c2 + 1;
                    if (s->scan_c >= cols) {
                        s->scan_c = 0;
                        s->scan_r++;
                    }
                    return 1;
                }
            }
            s->scan_c++;
            if (s->scan_c >= cols) {
                s->scan_c = 0;
                s->scan_r++;
            }
        }

        /* Decomposition complete — start A* on perimeter */
        s->phase = 1;
        rsr_mark_perimeter(s);

        int start = s->vis.start_node;
        s->cost[start] = 0;
        int sr = start / cols, sc = start % cols;
        int h = manhattan(sr, sc, map->end_r, map->end_c);
        heap_push(&s->heap, start, h);
        return 1;
    }

    if (s->phase == 1) {
        /* Phase 2: A* on perimeter nodes */
        if (s->heap.size == 0) { s->vis.done = 1; return 0; }

        HeapEntry cur = heap_pop(&s->heap);
        int node = cur.node;
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

        int r = node / cols, c = node % cols;

        /* Expand to adjacent perimeter nodes */
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

                /* If non-perimeter interior, skip-traverse to other side */
                if (!s->is_perimeter[neighbor]) {
                    /* Walk through interior to opposite perimeter */
                    int wr = nr, wc = nc;
                    int dist = 1;
                    while (wr + DR[d] >= 0 && wr + DR[d] < s->map->rows &&
                           wc + DC[d] >= 0 && wc + DC[d] < s->map->cols) {
                        int next_idx = get_index(cols, wr + DR[d], wc + DC[d]);
                        if (s->map->data[next_idx] != 0 || s->is_perimeter[next_idx])
                            break;
                        wr += DR[d];
                        wc += DC[d];
                        dist++;
                    }
                    int far = get_index(cols, wr, wc);
                    int far_g = s->cost[node] + dist;
                    if (far_g < s->cost[far]) {
                        s->cost[far] = far_g;
                        s->parent[far] = node;
                        int h = manhattan(wr, wc, s->map->end_r, s->map->end_c);
                        heap_push(&s->heap, far, far_g + h);
                    }
                    continue;
                }

                int h = manhattan(nr, nc, s->map->end_r, s->map->end_c);
                heap_push(&s->heap, neighbor, new_g + h);
                if (neighbor != s->vis.start_node && neighbor != s->vis.end_node)
                    s->vis.cells[neighbor] = VIS_OPEN;
            }
        }

        return 1;
    }

    return 0;
}

AlgoPlugin algo_rsr = {
    .name = "RSR",
    .init = rsr_init,
    .step = rsr_step,
};
