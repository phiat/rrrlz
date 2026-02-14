/*
 * algo.h — Plugin interface for pathfinding algorithm visualizer
 *
 * Each algorithm implements an AlgoPlugin with init() and step().
 * Algorithm state structs must have AlgoVis as their first member.
 */

#ifndef ALGO_H
#define ALGO_H

#include <limits.h>
#include <string.h>

/* ── Grid constants ───────────────────────────────────────────────── */

#define ROWS 20
#define COLS 20
#define MAX_NODES (ROWS * COLS)

#define START_R 0
#define START_C 0
#define END_R   19
#define END_C   19

static const int DR[4] = {-1, 1, 0, 0};
static const int DC[4] = {0, 0, -1, 1};

/* ── Cell visualization enum ──────────────────────────────────────── */

enum CellVis {
    VIS_EMPTY,
    VIS_WALL,
    VIS_OPEN,     /* frontier / reachable */
    VIS_CLOSED,   /* expanded / visited */
    VIS_PATH,
    VIS_START,
    VIS_END,
};

/* ── Visualization state (first member of every algo state struct) ─ */

typedef struct {
    int cells[MAX_NODES];
    int done;
    int found;
    int nodes_explored;
    int steps;
    int path_len;
    int path_cost;
} AlgoVis;

/* ── Plugin descriptor ────────────────────────────────────────────── */

typedef struct {
    const char *name;
    AlgoVis *(*init)(const int (*map)[COLS]);
    int      (*step)(AlgoVis *vis);
} AlgoPlugin;

/* ── Inline helpers ───────────────────────────────────────────────── */

static inline int get_index(int r, int c) { return r * COLS + c; }

static inline int manhattan(int r, int c) {
    int dr = r - END_R;
    int dc = c - END_C;
    return (dr < 0 ? -dr : dr) + (dc < 0 ? -dc : dc);
}

static inline int is_valid(const int (*map)[COLS], int r, int c) {
    return r >= 0 && r < ROWS && c >= 0 && c < COLS && map[r][c] == 0;
}

/* Helper: initialize cells array from map */
static inline void vis_init_cells(AlgoVis *vis, const int (*map)[COLS]) {
    for (int i = 0; i < MAX_NODES; i++) {
        int r = i / COLS, c = i % COLS;
        vis->cells[i] = map[r][c] ? VIS_WALL : VIS_EMPTY;
    }
    vis->cells[get_index(START_R, START_C)] = VIS_START;
    vis->cells[get_index(END_R, END_C)] = VIS_END;
    vis->done = 0;
    vis->found = 0;
    vis->nodes_explored = 0;
    vis->steps = 0;
    vis->path_len = 0;
    vis->path_cost = 0;
}

/* Helper: trace path from end to start using parent array */
static inline void vis_trace_path(AlgoVis *vis, const int *parent, const int *cost) {
    int end = get_index(END_R, END_C);
    vis->path_cost = cost[end];
    int cur = end;
    while (cur != -1) {
        if (cur != get_index(START_R, START_C) && cur != get_index(END_R, END_C))
            vis->cells[cur] = VIS_PATH;
        vis->path_len++;
        cur = parent[cur];
    }
}

/* ── Min-heap (used by Dijkstra and A*) ───────────────────────────── */

typedef struct {
    int node;
    int priority;
} HeapEntry;

typedef struct {
    HeapEntry data[MAX_NODES * 4];
    int size;
} Heap;

static inline void heap_init(Heap *h) { h->size = 0; }

static inline void heap_push(Heap *h, int node, int priority) {
    int i = h->size++;
    h->data[i].node = node;
    h->data[i].priority = priority;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (h->data[p].priority <= h->data[i].priority) break;
        HeapEntry tmp = h->data[i];
        h->data[i] = h->data[p];
        h->data[p] = tmp;
        i = p;
    }
}

static inline HeapEntry heap_pop(Heap *h) {
    HeapEntry top = h->data[0];
    h->data[0] = h->data[--h->size];
    int i = 0;
    for (;;) {
        int l = 2 * i + 1, r = 2 * i + 2, s = i;
        if (l < h->size && h->data[l].priority < h->data[s].priority) s = l;
        if (r < h->size && h->data[r].priority < h->data[s].priority) s = r;
        if (s == i) break;
        HeapEntry tmp = h->data[i];
        h->data[i] = h->data[s];
        h->data[s] = tmp;
        i = s;
    }
    return top;
}

#endif /* ALGO_H */
