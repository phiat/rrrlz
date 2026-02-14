/*
 * algo_subgoal.c — Subgoal Graphs step machine
 *
 * Phase 1: Identify subgoals at obstacle corners.
 * Phase 2: Build edges between direct-h-reachable subgoals.
 * Phase 3: A* on the subgoal graph.
 */

#include "algo.h"

#define MAX_SUBGOALS 1000
#define MAX_ADJ 32

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    /* Subgoal data */
    int subgoals[MAX_SUBGOALS]; /* node indices of subgoals */
    int sg_count;
    int sg_idx[MAX_NODES];      /* map node → subgoal index, -1 if not */
    /* Adjacency */
    int sg_adj[MAX_SUBGOALS][MAX_ADJ];
    int sg_adj_cost[MAX_SUBGOALS][MAX_ADJ];
    int sg_adj_count[MAX_SUBGOALS];
    /* Phase tracking */
    int phase;      /* 0=identify, 1=edges, 2=search */
    int scan_pos;   /* current scan position */
    int edge_i;     /* current subgoal for edge building */
    /* A* on subgoal graph */
    Heap heap;
    int cost[MAX_SUBGOALS + 2]; /* +2 for start/end virtual subgoals */
    int parent[MAX_SUBGOALS + 2];
    int closed_sg[MAX_SUBGOALS + 2];
    int start_sg, end_sg; /* subgoal indices for start/end */
} SubgoalState;

static SubgoalState state;

/* Check if a cell is a subgoal: adjacent to an L-shaped wall corner */
static int is_subgoal(const MapDef *map, int r, int c) {
    if (map->data[r * map->cols + c] != 0) return 0;
    int cols = map->cols, rows = map->rows;
    /* Check 4 L-shaped corners: NW, NE, SW, SE */
    /* NW: wall at N and W */
    int n = (r > 0) ? map->data[(r-1) * cols + c] : 1;
    int s = (r < rows-1) ? map->data[(r+1) * cols + c] : 1;
    int w = (c > 0) ? map->data[r * cols + (c-1)] : 1;
    int e = (c < cols-1) ? map->data[r * cols + (c+1)] : 1;

    if (n && w) return 1;
    if (n && e) return 1;
    if (s && w) return 1;
    if (s && e) return 1;
    return 0;
}

/* Check direct-h-reachability: same row/col, no walls between, no other subgoal between */
static int direct_reachable(SubgoalState *s, int sg1, int sg2) {
    int n1 = s->subgoals[sg1], n2 = s->subgoals[sg2];
    int cols = s->vis.cols;
    int r1 = n1 / cols, c1 = n1 % cols;
    int r2 = n2 / cols, c2 = n2 % cols;

    if (r1 == r2) {
        /* Same row */
        int minc = c1 < c2 ? c1 : c2;
        int maxc = c1 > c2 ? c1 : c2;
        for (int c = minc + 1; c < maxc; c++) {
            if (s->map->data[r1 * cols + c] != 0) return 0;
            int ni = get_index(cols, r1, c);
            if (s->sg_idx[ni] >= 0) return 0; /* another subgoal between */
        }
        return 1;
    } else if (c1 == c2) {
        /* Same column */
        int minr = r1 < r2 ? r1 : r2;
        int maxr = r1 > r2 ? r1 : r2;
        for (int r = minr + 1; r < maxr; r++) {
            if (s->map->data[r * cols + c1] != 0) return 0;
            int ni = get_index(cols, r, c1);
            if (s->sg_idx[ni] >= 0) return 0;
        }
        return 1;
    }
    return 0;
}

static AlgoVis *subgoal_init(const MapDef *map) {
    memset(&state, 0, sizeof(state));
    state.map = map;
    vis_init_cells(&state.vis, map);
    heap_init(&state.heap);

    int total = map->rows * map->cols;
    for (int i = 0; i < total; i++)
        state.sg_idx[i] = -1;
    for (int i = 0; i < MAX_SUBGOALS + 2; i++) {
        state.cost[i] = INT_MAX;
        state.parent[i] = -1;
    }

    state.phase = 0;
    state.scan_pos = 0;
    state.start_sg = -1;
    state.end_sg = -1;

    return &state.vis;
}

static int subgoal_step(AlgoVis *vis) {
    SubgoalState *s = (SubgoalState *)vis;
    if (s->vis.done) return 0;

    int cols = s->vis.cols;
    s->vis.steps++;

    if (s->phase == 0) {
        /* Phase 1: Identify subgoals */
        const MapDef *map = s->map;
        int total = map->rows * map->cols;

        while (s->scan_pos < total) {
            int pos = s->scan_pos++;
            int r = pos / cols, c = pos % cols;
            if (is_subgoal(map, r, c) && s->sg_count < MAX_SUBGOALS) {
                int idx = s->sg_count;
                s->subgoals[idx] = pos;
                s->sg_idx[pos] = idx;
                s->sg_count++;

                if (pos != s->vis.start_node && pos != s->vis.end_node)
                    s->vis.cells[pos] = VIS_PREPROCESS;

                /* Check if start/end are subgoals */
                if (pos == s->vis.start_node) s->start_sg = idx;
                if (pos == s->vis.end_node) s->end_sg = idx;

                return 1;
            }
        }

        /* Add start/end as virtual subgoals if not already */
        if (s->start_sg < 0 && s->sg_count < MAX_SUBGOALS) {
            s->start_sg = s->sg_count;
            s->subgoals[s->sg_count] = s->vis.start_node;
            s->sg_idx[s->vis.start_node] = s->sg_count;
            s->sg_count++;
        }
        if (s->end_sg < 0 && s->sg_count < MAX_SUBGOALS) {
            s->end_sg = s->sg_count;
            s->subgoals[s->sg_count] = s->vis.end_node;
            s->sg_idx[s->vis.end_node] = s->sg_count;
            s->sg_count++;
        }

        s->phase = 1;
        s->edge_i = 0;
        return 1;
    }

    if (s->phase == 1) {
        /* Phase 2: Build edges */
        if (s->edge_i >= s->sg_count) {
            /* Start A* */
            s->phase = 2;
            s->cost[s->start_sg] = 0;
            int sn = s->subgoals[s->start_sg];
            int sr = sn / cols, sc = sn % cols;
            int h = manhattan(sr, sc, s->map->end_r, s->map->end_c);
            heap_push(&s->heap, s->start_sg, h);
            return 1;
        }

        int i = s->edge_i++;
        for (int j = i + 1; j < s->sg_count; j++) {
            if (direct_reachable(s, i, j)) {
                int n1 = s->subgoals[i], n2 = s->subgoals[j];
                int r1 = n1 / cols, c1 = n1 % cols;
                int r2 = n2 / cols, c2 = n2 % cols;
                int dist = manhattan(r1, c1, r2, c2);

                if (s->sg_adj_count[i] < MAX_ADJ) {
                    int k = s->sg_adj_count[i]++;
                    s->sg_adj[i][k] = j;
                    s->sg_adj_cost[i][k] = dist;
                }
                if (s->sg_adj_count[j] < MAX_ADJ) {
                    int k = s->sg_adj_count[j]++;
                    s->sg_adj[j][k] = i;
                    s->sg_adj_cost[j][k] = dist;
                }
            }
        }
        return 1;
    }

    if (s->phase == 2) {
        /* Phase 3: A* on subgoal graph */
        if (s->heap.size == 0) { s->vis.done = 1; return 0; }

        HeapEntry cur = heap_pop(&s->heap);
        int sg = cur.node;
        if (s->closed_sg[sg]) return 1;

        s->closed_sg[sg] = 1;
        s->vis.nodes_explored++;
        int node = s->subgoals[sg];

        if (node != s->vis.start_node && node != s->vis.end_node)
            s->vis.cells[node] = VIS_CLOSED;

        if (sg == s->end_sg) {
            s->vis.done = 1;
            s->vis.found = 1;
            /* Trace path through subgoal parents */
            s->vis.path_cost = s->cost[sg];
            int csg = sg;
            while (csg >= 0) {
                int cn = s->subgoals[csg];
                int psg = s->parent[csg];
                if (psg >= 0) {
                    /* Fill cells between parent subgoal and current */
                    int pn = s->subgoals[psg];
                    int cr = cn / cols, cc = cn % cols;
                    int pr = pn / cols, pc = pn % cols;
                    int dr = 0, dc = 0;
                    if (cr < pr) dr = 1; else if (cr > pr) dr = -1;
                    if (cc < pc) dc = 1; else if (cc > pc) dc = -1;
                    int ir = cr, ic = cc;
                    while (ir != pr || ic != pc) {
                        int idx = get_index(cols, ir, ic);
                        if (idx != s->vis.start_node && idx != s->vis.end_node)
                            s->vis.cells[idx] = VIS_PATH;
                        s->vis.path_len++;
                        if (ir != pr) ir += dr;
                        else if (ic != pc) ic += dc;
                    }
                } else {
                    s->vis.path_len++;
                }
                csg = psg;
            }
            return 1;
        }

        for (int i = 0; i < s->sg_adj_count[sg]; i++) {
            int nsg = s->sg_adj[sg][i];
            if (s->closed_sg[nsg]) continue;

            int new_g = s->cost[sg] + s->sg_adj_cost[sg][i];
            if (new_g < s->cost[nsg]) {
                s->vis.relaxations++;
                s->cost[nsg] = new_g;
                s->parent[nsg] = sg;
                int nn = s->subgoals[nsg];
                int nr = nn / cols, nc = nn % cols;
                int h = manhattan(nr, nc, s->map->end_r, s->map->end_c);
                heap_push(&s->heap, nsg, new_g + h);
                if (nn != s->vis.start_node && nn != s->vis.end_node)
                    s->vis.cells[nn] = VIS_OPEN;
            }
        }

        return 1;
    }

    return 0;
}

AlgoPlugin algo_subgoal = {
    .name = "Subgoal",
    .init = subgoal_init,
    .step = subgoal_step,
};
