/*
 * algo_ch.c — Contraction Hierarchies step machine
 *
 * Phase 1: Contract nodes by importance (edge-difference heuristic),
 *          adding shortcut edges.
 * Phase 2: Bidirectional search ascending the hierarchy.
 */

#include "algo.h"

#define MAX_SHORTCUTS 40000
#define MAX_CH_ADJ 16

typedef struct {
    int from, to, cost;
    int mid;  /* intermediate node for shortcut unpacking, -1 if original edge */
} Shortcut;

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    /* Contraction */
    int level[MAX_NODES];       /* contraction order (higher = more important) */
    int contracted[MAX_NODES];
    Shortcut shortcuts[MAX_SHORTCUTS];
    int shortcut_count;
    int contract_order;         /* next contraction level to assign */
    int phase;                  /* 0=contraction, 1=search */
    /* Adjacency (upward graph) */
    int up_adj[MAX_NODES][MAX_CH_ADJ];
    int up_cost[MAX_NODES][MAX_CH_ADJ];
    int up_count[MAX_NODES];
    int up_mid[MAX_NODES][MAX_CH_ADJ]; /* shortcut mid-node, -1 if original */
    /* Bidirectional search */
    Heap fwd_heap, bwd_heap;
    int fwd_dist[MAX_NODES], bwd_dist[MAX_NODES];
    int fwd_parent[MAX_NODES], bwd_parent[MAX_NODES];
    int fwd_closed[MAX_NODES], bwd_closed[MAX_NODES];
    int mu;       /* best path cost found */
    int meet_node;
    int fwd_turn; /* alternate forward/backward */
    int total_nodes;
    /* For node ordering: priority queue of contraction candidates */
    int edge_diff[MAX_NODES]; /* cached edge-difference */
} CHState;

static CHState state;

/* Count edges to/from uncontracted neighbors */
static void ch_count_edges(CHState *s, int node, int *in_deg, int *out_deg) {
    int cols = s->vis.cols;
    int r = node / cols, c = node % cols;
    *in_deg = 0; *out_deg = 0;
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!is_valid(s->map, nr, nc)) continue;
        int ni = get_index(cols, nr, nc);
        if (s->contracted[ni]) continue;
        (*in_deg)++;
        (*out_deg)++;
    }
}

/* Simple witness search: is there a path from u to v of cost <= limit
   without going through 'exclude'? (3-hop BFS) */
static int witness_exists(CHState *s, int u, int v, int limit, int exclude) {
    if (u == v) return 1;
    int cols = s->vis.cols;
    /* Simple BFS up to 3 hops */
    int queue[64], dist_q[64];
    int qfront = 0, qback = 0;
    queue[qback] = u;
    dist_q[qback] = 0;
    qback++;

    int visited[64];
    int visit_count = 0;

    for (int iter = 0; iter < 64 && qfront < qback; iter++) {
        int cur = queue[qfront];
        int d = dist_q[qfront];
        qfront++;

        if (cur == v && d <= limit) return 1;
        if (d >= 3) continue;  /* 3-hop limit */

        int cr = cur / cols, cc = cur % cols;
        for (int dd = 0; dd < 4; dd++) {
            int nr = cr + DR[dd], nc = cc + DC[dd];
            if (!is_valid(s->map, nr, nc)) continue;
            int ni = get_index(cols, nr, nc);
            if (ni == exclude || s->contracted[ni]) continue;
            int nd = d + 1;
            if (nd > limit) continue;

            /* Check if already visited */
            int found = 0;
            for (int k = 0; k < visit_count; k++)
                if (visited[k] == ni) { found = 1; break; }
            if (found) continue;

            if (visit_count < 64) visited[visit_count++] = ni;
            if (qback < 64) {
                queue[qback] = ni;
                dist_q[qback] = nd;
                qback++;
            }
        }
    }
    return 0;
}

/* Find the uncontracted node with lowest edge-difference */
static int ch_find_next(CHState *s) {
    int best = -1, best_diff = INT_MAX;
    int total = s->total_nodes;
    for (int i = 0; i < total; i++) {
        if (s->contracted[i]) continue;
        if (s->map->data[i] != 0) continue;
        int in_d, out_d;
        ch_count_edges(s, i, &in_d, &out_d);
        /* Edge difference: shortcuts needed - edges removed */
        int shortcuts_needed = 0;
        int cols = s->vis.cols;
        int r = i / cols, c = i % cols;

        /* Count how many shortcuts would be needed */
        for (int d1 = 0; d1 < 4; d1++) {
            int nr1 = r + DR[d1], nc1 = c + DC[d1];
            if (!is_valid(s->map, nr1, nc1)) continue;
            int n1 = get_index(cols, nr1, nc1);
            if (s->contracted[n1]) continue;
            for (int d2 = d1 + 1; d2 < 4; d2++) {
                int nr2 = r + DR[d2], nc2 = c + DC[d2];
                if (!is_valid(s->map, nr2, nc2)) continue;
                int n2 = get_index(cols, nr2, nc2);
                if (s->contracted[n2]) continue;
                if (!witness_exists(s, n1, n2, 2, i))
                    shortcuts_needed++;
            }
        }
        int diff = shortcuts_needed - (in_d + out_d);
        if (diff < best_diff) {
            best_diff = diff;
            best = i;
        }
    }
    return best;
}

/* Add upward edge (from lower to higher level) */
static void add_up_edge(CHState *s, int from, int to, int cost, int mid) {
    if (s->up_count[from] < MAX_CH_ADJ) {
        int k = s->up_count[from]++;
        s->up_adj[from][k] = to;
        s->up_cost[from][k] = cost;
        s->up_mid[from][k] = mid;
    }
}

static AlgoVis *ch_init(const MapDef *map) {
    memset(&state, 0, sizeof(state));
    state.map = map;
    vis_init_cells(&state.vis, map);
    heap_init(&state.fwd_heap);
    heap_init(&state.bwd_heap);

    state.total_nodes = map->rows * map->cols;
    for (int i = 0; i < state.total_nodes; i++) {
        state.fwd_dist[i] = INT_MAX;
        state.bwd_dist[i] = INT_MAX;
        state.fwd_parent[i] = -1;
        state.bwd_parent[i] = -1;
    }
    state.mu = INT_MAX;
    state.meet_node = -1;
    state.phase = 0;
    state.contract_order = 0;

    return &state.vis;
}

static void ch_unpack_path(CHState *s, int from, int to) {
    /* Find the edge from→to in up_adj and check for mid node */
    for (int i = 0; i < s->up_count[from]; i++) {
        if (s->up_adj[from][i] == to) {
            int mid = s->up_mid[from][i];
            if (mid >= 0) {
                ch_unpack_path(s, from, mid);
                ch_unpack_path(s, mid, to);
                return;
            }
        }
    }
    /* Also check reverse direction */
    for (int i = 0; i < s->up_count[to]; i++) {
        if (s->up_adj[to][i] == from) {
            int mid = s->up_mid[to][i];
            if (mid >= 0) {
                ch_unpack_path(s, from, mid);
                ch_unpack_path(s, mid, to);
                return;
            }
        }
    }
    /* Direct edge — mark 'to' on path */
    if (to != s->vis.start_node && to != s->vis.end_node)
        s->vis.cells[to] = VIS_PATH;
    s->vis.path_len++;
}

static int ch_step(AlgoVis *vis) {
    CHState *s = (CHState *)vis;
    if (s->vis.done) return 0;

    s->vis.steps++;

    if (s->phase == 0) {
        /* Phase 1: Batch-contract nodes per step for responsiveness */
        int cols = s->vis.cols;
        int batch = s->total_nodes / 50;
        if (batch < 10) batch = 10;

        for (int b = 0; b < batch; b++) {
            int node = ch_find_next(s);
            if (node < 0) {
                /* All contracted, build upward graph and start search */
                s->phase = 1;
                s->fwd_dist[s->vis.start_node] = 0;
                s->bwd_dist[s->vis.end_node] = 0;
                heap_push(&s->fwd_heap, s->vis.start_node, 0);
                heap_push(&s->bwd_heap, s->vis.end_node, 0);
                s->fwd_turn = 1;

                /* Add original edges as upward edges (lower→higher level) */
                for (int i = 0; i < s->total_nodes; i++) {
                    if (s->map->data[i] != 0) continue;
                    int ir = i / cols, ic = i % cols;
                    for (int d = 0; d < 4; d++) {
                        int nr = ir + DR[d], nc = ic + DC[d];
                        if (!is_valid(s->map, nr, nc)) continue;
                        int ni = get_index(cols, nr, nc);
                        if (s->level[ni] > s->level[i])
                            add_up_edge(s, i, ni, 1, -1);
                    }
                }

                return 1;
            }

            s->contracted[node] = 1;
            s->level[node] = s->contract_order++;

            if (node != s->vis.start_node && node != s->vis.end_node)
                s->vis.cells[node] = VIS_PREPROCESS;

            /* Add shortcuts */
            int r = node / cols, c = node % cols;
            for (int d1 = 0; d1 < 4; d1++) {
                int nr1 = r + DR[d1], nc1 = c + DC[d1];
                if (!is_valid(s->map, nr1, nc1)) continue;
                int n1 = get_index(cols, nr1, nc1);
                if (s->contracted[n1]) continue;
                for (int d2 = d1 + 1; d2 < 4; d2++) {
                    int nr2 = r + DR[d2], nc2 = c + DC[d2];
                    if (!is_valid(s->map, nr2, nc2)) continue;
                    int n2 = get_index(cols, nr2, nc2);
                    if (s->contracted[n2]) continue;

                    if (!witness_exists(s, n1, n2, 2, node) &&
                        s->shortcut_count < MAX_SHORTCUTS) {
                        Shortcut *sc = &s->shortcuts[s->shortcut_count++];
                        sc->from = n1; sc->to = n2; sc->cost = 2; sc->mid = node;
                    }
                }
            }

            s->vis.nodes_explored++;
        }
        return 1;
    }

    if (s->phase == 1) {
        /* Phase 2: Bidirectional search ascending hierarchy */
        /* Also add shortcut upward edges now */
        for (int i = 0; i < s->shortcut_count; i++) {
            Shortcut *sc = &s->shortcuts[i];
            if (s->level[sc->to] > s->level[sc->from])
                add_up_edge(s, sc->from, sc->to, sc->cost, sc->mid);
            if (s->level[sc->from] > s->level[sc->to])
                add_up_edge(s, sc->to, sc->from, sc->cost, sc->mid);
        }
        s->shortcut_count = 0; /* Processed */
        s->phase = 2;
        return 1;
    }

    if (s->phase == 2) {
        /* Alternate forward/backward Dijkstra ascending hierarchy */
        if (s->fwd_turn) {
            s->fwd_turn = 0;
            if (s->fwd_heap.size > 0) {
                HeapEntry cur = heap_pop(&s->fwd_heap);
                int node = cur.node;
                if (!s->fwd_closed[node]) {
                    s->fwd_closed[node] = 1;
                    s->vis.nodes_explored++;
                    if (node != s->vis.start_node && node != s->vis.end_node)
                        s->vis.cells[node] = VIS_OPEN;

                    /* Check meeting */
                    if (s->bwd_dist[node] != INT_MAX) {
                        int total_cost = s->fwd_dist[node] + s->bwd_dist[node];
                        if (total_cost < s->mu) {
                            s->mu = total_cost;
                            s->meet_node = node;
                        }
                    }

                    /* Relax upward neighbors */
                    for (int i = 0; i < s->up_count[node]; i++) {
                        int nb = s->up_adj[node][i];
                        int nc = s->fwd_dist[node] + s->up_cost[node][i];
                        if (nc < s->fwd_dist[nb]) {
                            s->vis.relaxations++;
                            s->fwd_dist[nb] = nc;
                            s->fwd_parent[nb] = node;
                            heap_push(&s->fwd_heap, nb, nc);
                        }
                    }
                }
            }
        } else {
            s->fwd_turn = 1;
            if (s->bwd_heap.size > 0) {
                HeapEntry cur = heap_pop(&s->bwd_heap);
                int node = cur.node;
                if (!s->bwd_closed[node]) {
                    s->bwd_closed[node] = 1;
                    s->vis.nodes_explored++;
                    if (node != s->vis.start_node && node != s->vis.end_node)
                        s->vis.cells[node] = VIS_CLOSED;

                    if (s->fwd_dist[node] != INT_MAX) {
                        int total_cost = s->fwd_dist[node] + s->bwd_dist[node];
                        if (total_cost < s->mu) {
                            s->mu = total_cost;
                            s->meet_node = node;
                        }
                    }

                    for (int i = 0; i < s->up_count[node]; i++) {
                        int nb = s->up_adj[node][i];
                        int nc = s->bwd_dist[node] + s->up_cost[node][i];
                        if (nc < s->bwd_dist[nb]) {
                            s->vis.relaxations++;
                            s->bwd_dist[nb] = nc;
                            s->bwd_parent[nb] = node;
                            heap_push(&s->bwd_heap, nb, nc);
                        }
                    }
                }
            }
        }

        /* Check termination */
        if (s->fwd_heap.size == 0 && s->bwd_heap.size == 0) {
            if (s->meet_node >= 0) goto found_path;
            s->vis.done = 1;
            return 0;
        }

        int min_key = INT_MAX;
        if (s->fwd_heap.size > 0 && s->fwd_heap.data[0].priority < min_key)
            min_key = s->fwd_heap.data[0].priority;
        if (s->bwd_heap.size > 0 && s->bwd_heap.data[0].priority < min_key)
            min_key = s->bwd_heap.data[0].priority;

        if (min_key >= s->mu && s->meet_node >= 0) {
            goto found_path;
        }

        return 1;

found_path:
        s->vis.done = 1;
        s->vis.found = 1;
        s->vis.path_cost = s->mu;
        /* Unpack path: forward from start to meet, backward from goal to meet */
        {
            /* Forward path */
            int cur = s->meet_node;
            while (s->fwd_parent[cur] >= 0) {
                ch_unpack_path(s, s->fwd_parent[cur], cur);
                cur = s->fwd_parent[cur];
            }
            s->vis.path_len++; /* start node */

            /* Backward path */
            cur = s->meet_node;
            while (s->bwd_parent[cur] >= 0) {
                ch_unpack_path(s, cur, s->bwd_parent[cur]);
                cur = s->bwd_parent[cur];
            }
        }
        return 0;
    }

    return 0;
}

AlgoPlugin algo_ch = {
    .name = "CH",
    .init = ch_init,
    .step = ch_step,
};
