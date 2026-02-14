/*
 * algo_floyd_warshall.c — Floyd-Warshall step machine
 *
 * Each step processes one intermediate vertex k (all i,j pairs).
 * After each k-step, colors newly reachable nodes from start.
 * When done, traces the shortest path using the next-hop matrix.
 *
 * Capped at FW_MAX_NODES to keep memory reasonable (~50MB static).
 */

#include "algo.h"

#define FW_MAX_NODES 2500
#define FW_INF (FW_MAX_NODES * 10)

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    int node_count;                /* number of non-wall cells */
    int node_id[MAX_NODES];        /* grid index → compressed ID (-1 if wall) */
    int grid_idx[FW_MAX_NODES];    /* compressed ID → grid index */
    int fw_k;                      /* current intermediate vertex */
} FloydWarshallState;

static FloydWarshallState state;

/* These are large, so static file-scope rather than in the struct */
static int dist[FW_MAX_NODES][FW_MAX_NODES];
static int nxt[FW_MAX_NODES][FW_MAX_NODES];

static AlgoVis *floyd_warshall_init(const MapDef *map) {
    memset(&state, 0, sizeof(state));
    state.map = map;
    vis_init_cells(&state.vis, map);

    int cols = map->cols;
    int total = map->rows * map->cols;

    /* Build compressed node IDs (only non-wall cells) */
    state.node_count = 0;
    for (int i = 0; i < total; i++) {
        if (map->data[i] == 0) {
            state.node_id[i] = state.node_count;
            state.grid_idx[state.node_count] = i;
            state.node_count++;
        } else {
            state.node_id[i] = -1;
        }
    }

    int V = state.node_count;

    /* Initialize distance matrix */
    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            dist[i][j] = (i == j) ? 0 : FW_INF;
            nxt[i][j] = -1;
        }
    }

    /* Add edges (4-directional adjacency) */
    for (int r = 0; r < map->rows; r++) {
        for (int c = 0; c < map->cols; c++) {
            if (map->data[r * cols + c]) continue;
            int u = state.node_id[get_index(cols, r, c)];
            for (int d = 0; d < 4; d++) {
                int nr = r + DR[d], nc = c + DC[d];
                if (!is_valid(map, nr, nc)) continue;
                int v = state.node_id[get_index(cols, nr, nc)];
                dist[u][v] = 1;
                nxt[u][v] = v;
            }
        }
    }

    state.fw_k = 0;
    return &state.vis;
}

static int floyd_warshall_step(AlgoVis *vis) {
    FloydWarshallState *s = (FloydWarshallState *)vis;
    if (s->vis.done) return 0;

    int V = s->node_count;
    if (s->fw_k >= V) {
        /* Algorithm complete — trace path */
        s->vis.done = 1;

        int start_id = s->node_id[s->vis.start_node];
        int end_id = s->node_id[s->vis.end_node];

        if (start_id < 0 || end_id < 0 || dist[start_id][end_id] >= FW_INF)
            return 0;

        s->vis.found = 1;
        s->vis.path_cost = dist[start_id][end_id];

        /* Trace path using next-hop matrix */
        int cur = start_id;
        while (cur != end_id && cur != -1) {
            int grid = s->grid_idx[cur];
            if (grid != s->vis.start_node &&
                grid != s->vis.end_node)
                s->vis.cells[grid] = VIS_PATH;
            s->vis.path_len++;
            cur = nxt[cur][end_id];
        }
        if (cur == end_id) s->vis.path_len++; /* count end node */

        return 1;
    }

    s->vis.steps++;

    /* Process all (i,j) pairs with intermediate vertex k */
    int k = s->fw_k;
    for (int i = 0; i < V; i++) {
        if (dist[i][k] >= FW_INF) continue;
        for (int j = 0; j < V; j++) {
            if (dist[k][j] >= FW_INF) continue;
            int through_k = dist[i][k] + dist[k][j];
            if (through_k < dist[i][j]) {
                s->vis.relaxations++;
                dist[i][j] = through_k;
                nxt[i][j] = nxt[i][k];
            }
        }
    }

    /* Color: show which nodes are reachable from start after this k-step */
    int start_id = s->node_id[s->vis.start_node];
    if (start_id >= 0) {
        for (int j = 0; j < V; j++) {
            if (dist[start_id][j] < FW_INF && j != start_id) {
                int grid = s->grid_idx[j];
                if (grid != s->vis.start_node &&
                    grid != s->vis.end_node) {
                    if (s->vis.cells[grid] != VIS_OPEN) {
                        s->vis.cells[grid] = VIS_OPEN;
                        s->vis.nodes_explored++;
                    }
                }
            }
        }
    }

    /* Color intermediate vertex k as closed */
    int k_grid = s->grid_idx[k];
    if (k_grid != s->vis.start_node &&
        k_grid != s->vis.end_node)
        s->vis.cells[k_grid] = VIS_CLOSED;

    s->fw_k++;
    return 1;
}

AlgoPlugin algo_floyd_warshall = {
    .name = "Floyd-Warshall",
    .init = floyd_warshall_init,
    .step = floyd_warshall_step,
    .max_nodes = FW_MAX_NODES,
};
