#include <stdio.h>
#include <limits.h>

#define ROWS 20
#define COLS 20
#define MAX_NODES (ROWS * COLS)
#define INF (MAX_NODES + 1)

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

static inline int get_index(int r, int c) {
    return r * COLS + c;
}

static inline int is_valid(int r, int c) {
    return r >= 0 && r < ROWS && c >= 0 && c < COLS && grid[r][c] == 0;
}

/* 400x400 matrices — 640KB each, declared static to avoid stack overflow */
static int dist[MAX_NODES][MAX_NODES];
static int next[MAX_NODES][MAX_NODES];

int main(void) {
    int V = MAX_NODES;

    /* Initialize dist and next matrices */
    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            if (i == j) {
                dist[i][j] = 0;
                next[i][j] = i;
            } else {
                dist[i][j] = INF;
                next[i][j] = -1;
            }
        }
    }

    /* Build adjacency: edge weight 1 between adjacent open cells */
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            if (grid[r][c] == 1) continue; /* skip walls */
            int u = get_index(r, c);
            for (int d = 0; d < 4; d++) {
                int nr = r + DR[d];
                int nc = c + DC[d];
                if (!is_valid(nr, nc)) continue;
                int v = get_index(nr, nc);
                dist[u][v] = 1;
                next[u][v] = v;
            }
        }
    }

    /* Floyd-Warshall: all-pairs shortest paths */
    for (int k = 0; k < V; k++) {
        /* Skip wall nodes as intermediates — they have no edges */
        int kr = k / COLS;
        int kc = k % COLS;
        if (grid[kr][kc] == 1) continue;

        for (int i = 0; i < V; i++) {
            if (dist[i][k] == INF) continue; /* prune: no path i->k */
            for (int j = 0; j < V; j++) {
                if (dist[k][j] == INF) continue; /* prune: no path k->j */
                int through_k = dist[i][k] + dist[k][j];
                if (through_k < dist[i][j]) {
                    dist[i][j] = through_k;
                    next[i][j] = next[i][k];
                }
            }
        }
    }

    int start = get_index(START_R, START_C);
    int end = get_index(END_R, END_C);

    /* Reconstruct path from start to end */
    char path_grid[ROWS][COLS];
    for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
            path_grid[r][c] = grid[r][c] ? '#' : '.';

    int path_len = 0;
    if (dist[start][end] != INF) {
        int cur = start;
        while (cur != end) {
            int r = cur / COLS;
            int c = cur % COLS;
            path_grid[r][c] = '*';
            cur = next[cur][end];
            path_len++;
        }
        /* Mark the end node */
        path_len++;
    }

    path_grid[START_R][START_C] = 'S';
    path_grid[END_R][END_C] = 'E';

    /* Count open vertices */
    int total_vertices = 0;
    for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
            if (grid[r][c] == 0) total_vertices++;

    /* Print grid */
    printf("Floyd-Warshall Pathfinding (%dx%d grid)\n", ROWS, COLS);
    printf("------------------------------------------\n");
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++)
            putchar(path_grid[r][c]);
        putchar('\n');
    }
    printf("------------------------------------------\n");
    printf("Path cost:      %d\n", dist[start][end] != INF ? dist[start][end] : -1);
    printf("Path length:    %d nodes\n", path_len);
    printf("Total vertices: %d\n", total_vertices);

    return 0;
}
