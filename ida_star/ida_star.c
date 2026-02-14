#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define ROWS 20
#define COLS 20
#define MAX_NODES (ROWS * COLS)

/* 0 = open, 1 = wall — same grid as Dijkstra */
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

static int heuristic(int r, int c) {
    int dr = r - END_R;
    int dc = c - END_C;
    /* Manhattan distance: abs(dr) + abs(dc) */
    return (dr < 0 ? -dr : dr) + (dc < 0 ? -dc : dc);
}

/* IDA* state — globals for recursive search */
static int path_stack[MAX_NODES];  /* current path (node indices) */
static int path_len;               /* current path length */
static int on_path[MAX_NODES];     /* 1 if node is on current path (cycle check) */
static int nodes_explored;         /* total nodes explored across all iterations */
static int found;                  /* 1 if goal was reached */

/*
 * Recursive DFS with f-cost threshold.
 * Returns:
 *   -1    if goal was found (path is in path_stack[0..path_len-1])
 *   INT_MAX if no path possible through this branch
 *   otherwise, the minimum f-cost that exceeded the threshold
 */
static int search(int g, int threshold) {
    int node = path_stack[path_len - 1];
    int r = node / COLS;
    int c = node % COLS;
    int f = g + heuristic(r, c);

    if (f > threshold)
        return f;

    if (node == get_index(END_R, END_C)) {
        found = 1;
        return -1;
    }

    nodes_explored++;

    int min_exceeded = INT_MAX;

    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d];
        int nc = c + DC[d];
        if (!is_valid(nr, nc)) continue;
        int neighbor = get_index(nr, nc);
        if (on_path[neighbor]) continue;  /* avoid cycles */

        /* Push neighbor onto path stack */
        path_stack[path_len] = neighbor;
        path_len++;
        on_path[neighbor] = 1;

        int t = search(g + 1, threshold);

        if (t == -1)
            return -1;  /* found — unwind, keeping path intact */

        if (t < min_exceeded)
            min_exceeded = t;

        /* Pop neighbor from path stack */
        path_len--;
        on_path[neighbor] = 0;
    }

    return min_exceeded;
}

int main(void) {
    int start = get_index(START_R, START_C);
    int threshold = heuristic(START_R, START_C);
    int iterations = 0;

    /* Initialize path with start node */
    path_stack[0] = start;
    path_len = 1;
    on_path[start] = 1;
    nodes_explored = 0;
    found = 0;

    for (int i = 0; i < MAX_NODES; i++)
        on_path[i] = 0;
    on_path[start] = 1;

    while (!found) {
        iterations++;
        int t = search(0, threshold);

        if (t == -1)
            break;  /* path found */

        if (t == INT_MAX) {
            /* No path exists */
            path_len = 0;
            break;
        }

        threshold = t;
    }

    /* Build display grid */
    char path_grid[ROWS][COLS];
    for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
            path_grid[r][c] = grid[r][c] ? '#' : '.';

    int path_cost = -1;
    if (found) {
        path_cost = path_len - 1;  /* edges = nodes - 1 */
        for (int i = 0; i < path_len; i++) {
            int r = path_stack[i] / COLS;
            int c = path_stack[i] % COLS;
            path_grid[r][c] = '*';
        }
    }

    path_grid[START_R][START_C] = 'S';
    path_grid[END_R][END_C] = 'E';

    /* Print grid */
    printf("IDA* Pathfinding (%dx%d grid)\n", ROWS, COLS);
    printf("------------------------------------------\n");
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++)
            putchar(path_grid[r][c]);
        putchar('\n');
    }
    printf("------------------------------------------\n");
    printf("Path cost:      %d\n", path_cost);
    printf("Path length:    %d nodes\n", found ? path_len : 0);
    printf("Nodes explored: %d\n", nodes_explored);
    printf("Iterations:     %d\n", iterations);

    return 0;
}
