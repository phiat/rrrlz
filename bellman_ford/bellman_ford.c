#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define ROWS 20
#define COLS 20
#define MAX_NODES (ROWS * COLS)
#define MAX_EDGES (MAX_NODES * 4) /* each cell has at most 4 directed edges */

/* 0 = open, 1 = wall */
static const int grid[ROWS][COLS] = {
    {0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
    {0,1,1,0,0,1,0,1,1,0,1,1,0,0,1,0,1,1,0,0},
    {0,1,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,0,0},
    {0,0,0,1,1,1,0,0,1,0,0,0,0,1,1,1,0,0,0,0},
    {0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,1,0},
    {1,1,0,0,0,0,0,1,1,0,1,0,1,0,0,0,0,1,1,0},
    {0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,1,0,0,0,0},
    {0,1,0,1,0,1,1,0,0,0,0,1,0,0,0,1,0,1,0,0},
    {0,1,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,1},
    {0,0,0,0,1,0,1,0,0,1,0,0,0,1,0,0,1,0,0,0},
    {0,1,1,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,1,0},
    {0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0},
    {0,0,1,1,0,1,0,0,1,0,1,0,0,1,1,0,0,1,0,0},
    {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
    {1,1,0,0,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,0},
    {0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0},
    {0,1,0,1,0,1,0,0,0,0,1,0,0,1,0,0,1,0,1,0},
    {0,1,0,0,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,1,0,1,0,0},
    {0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0},
};

static const int START_R = 0, START_C = 0;
static const int END_R = 19, END_C = 19;

/* Direction offsets: up, down, left, right */
static const int DR[4] = {-1, 1, 0, 0};
static const int DC[4] = {0, 0, -1, 1};

typedef struct {
    int from;
    int to;
    int weight;
} Edge;

static Edge edges[MAX_EDGES];
static int edge_count = 0;

static inline int get_index(int r, int c) {
    return r * COLS + c;
}

static inline int is_valid(int r, int c) {
    return r >= 0 && r < ROWS && c >= 0 && c < COLS && grid[r][c] == 0;
}

static void build_edge_list(void) {
    edge_count = 0;
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            if (grid[r][c] != 0) continue;
            int from = get_index(r, c);
            for (int d = 0; d < 4; d++) {
                int nr = r + DR[d];
                int nc = c + DC[d];
                if (!is_valid(nr, nc)) continue;
                int to = get_index(nr, nc);
                edges[edge_count].from = from;
                edges[edge_count].to = to;
                edges[edge_count].weight = 1;
                edge_count++;
            }
        }
    }
}

int main(void) {
    int dist[MAX_NODES];
    int parent[MAX_NODES];
    int iterations = 0;

    /* Build edge list from grid */
    build_edge_list();

    /* Initialize distances */
    for (int i = 0; i < MAX_NODES; i++) {
        dist[i] = INT_MAX;
        parent[i] = -1;
    }

    int start = get_index(START_R, START_C);
    int end = get_index(END_R, END_C);
    dist[start] = 0;

    /* Count nodes actually reachable (open cells) for exploration metric */
    int nodes_explored = 0;

    /* Relax all edges V-1 times */
    for (int i = 0; i < MAX_NODES - 1; i++) {
        int any_update = 0;
        for (int e = 0; e < edge_count; e++) {
            int u = edges[e].from;
            int v = edges[e].to;
            int w = edges[e].weight;
            if (dist[u] != INT_MAX && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                parent[v] = u;
                any_update = 1;
            }
        }
        iterations++;
        /* Early exit: no relaxation occurred */
        if (!any_update) break;
    }

    /* Count explored nodes (those with finite distance) */
    for (int i = 0; i < MAX_NODES; i++) {
        if (dist[i] != INT_MAX) {
            nodes_explored++;
        }
    }

    /* Negative cycle detection pass */
    int has_negative_cycle = 0;
    for (int e = 0; e < edge_count; e++) {
        int u = edges[e].from;
        int v = edges[e].to;
        int w = edges[e].weight;
        if (dist[u] != INT_MAX && dist[u] + w < dist[v]) {
            has_negative_cycle = 1;
            break;
        }
    }

    if (has_negative_cycle) {
        printf("Negative cycle detected!\n");
        return 1;
    }

    /* Reconstruct path */
    char path_grid[ROWS][COLS];
    for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
            path_grid[r][c] = grid[r][c] ? '#' : '.';

    int path_len = 0;
    if (dist[end] != INT_MAX) {
        int cur = end;
        while (cur != -1) {
            int r = cur / COLS;
            int c = cur % COLS;
            path_grid[r][c] = '*';
            cur = parent[cur];
            path_len++;
        }
    }

    path_grid[START_R][START_C] = 'S';
    path_grid[END_R][END_C] = 'E';

    /* Print grid */
    printf("Bellman-Ford Pathfinding (%dx%d grid)\n", ROWS, COLS);
    printf("------------------------------------------\n");
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++)
            putchar(path_grid[r][c]);
        putchar('\n');
    }
    printf("------------------------------------------\n");
    printf("Path cost:      %d\n", dist[end] != INT_MAX ? dist[end] : -1);
    printf("Path length:    %d nodes\n", path_len);
    printf("Nodes explored: %d\n", nodes_explored);
    printf("Edges:          %d\n", edge_count);
    printf("Iterations:     %d\n", iterations);

    return 0;
}
