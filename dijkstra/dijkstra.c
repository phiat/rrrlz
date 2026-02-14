#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define ROWS 20
#define COLS 20
#define MAX_NODES (ROWS * COLS)

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
    int node;
    int cost;
} HeapEntry;

static HeapEntry heap[MAX_NODES];
static int heap_size = 0;

static inline int get_index(int r, int c) {
    return r * COLS + c;
}

static inline int is_valid(int r, int c) {
    return r >= 0 && r < ROWS && c >= 0 && c < COLS && grid[r][c] == 0;
}

static void heap_swap(int i, int j) {
    HeapEntry tmp = heap[i];
    heap[i] = heap[j];
    heap[j] = tmp;
}

static void heap_push(int node, int cost) {
    int i = heap_size++;
    heap[i].node = node;
    heap[i].cost = cost;
    /* Bubble up */
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (heap[parent].cost <= heap[i].cost) break;
        heap_swap(i, parent);
        i = parent;
    }
}

static HeapEntry heap_pop(void) {
    HeapEntry top = heap[0];
    heap[0] = heap[--heap_size];
    /* Bubble down */
    int i = 0;
    for (;;) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        if (left < heap_size && heap[left].cost < heap[smallest].cost)
            smallest = left;
        if (right < heap_size && heap[right].cost < heap[smallest].cost)
            smallest = right;
        if (smallest == i) break;
        heap_swap(i, smallest);
        i = smallest;
    }
    return top;
}

int main(void) {
    int dist[MAX_NODES];
    int parent[MAX_NODES];
    int visited[MAX_NODES];
    int nodes_explored = 0;

    for (int i = 0; i < MAX_NODES; i++) {
        dist[i] = INT_MAX;
        parent[i] = -1;
        visited[i] = 0;
    }

    int start = get_index(START_R, START_C);
    int end = get_index(END_R, END_C);
    dist[start] = 0;
    heap_push(start, 0);

    while (heap_size > 0) {
        HeapEntry cur = heap_pop();
        int node = cur.node;
        int r = node / COLS;
        int c = node % COLS;

        if (visited[node]) continue;
        visited[node] = 1;
        nodes_explored++;

        if (node == end) break;

        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d];
            int nc = c + DC[d];
            if (!is_valid(nr, nc)) continue;
            int neighbor = get_index(nr, nc);
            if (visited[neighbor]) continue;
            int new_cost = dist[node] + 1;
            if (new_cost < dist[neighbor]) {
                dist[neighbor] = new_cost;
                parent[neighbor] = node;
                heap_push(neighbor, new_cost);
            }
        }
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
    printf("Dijkstra Pathfinding (%dx%d grid)\n", ROWS, COLS);
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

    return 0;
}
