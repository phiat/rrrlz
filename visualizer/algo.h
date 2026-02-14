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

/* ── Grid upper bounds ───────────────────────────────────────────── */

#define MAX_ROWS 100
#define MAX_COLS 100
#define MAX_NODES (MAX_ROWS * MAX_COLS)

static const int DR[4] = {-1, 1, 0, 0};
static const int DC[4] = {0, 0, -1, 1};

/* ── Map definition ──────────────────────────────────────────────── */

typedef struct {
    const char *name;
    int rows, cols;
    int start_r, start_c, end_r, end_c;
    const int *data;  /* flat row-major array */
} MapDef;

/* ── Cell visualization enum ─────────────────────────────────────── */

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
    int relaxations;
    int rows, cols;
    int start_node, end_node;
} AlgoVis;

/* ── Plugin descriptor ───────────────────────────────────────────── */

typedef struct {
    const char *name;
    AlgoVis *(*init)(const MapDef *map);
    int      (*step)(AlgoVis *vis);
    int      max_nodes;  /* 0=unlimited, >0=skip if map has more nodes */
} AlgoPlugin;

/* ── Inline helpers ──────────────────────────────────────────────── */

static inline int get_index(int cols, int r, int c) { return r * cols + c; }

static inline int manhattan(int r, int c, int end_r, int end_c) {
    int dr = r - end_r;
    int dc = c - end_c;
    return (dr < 0 ? -dr : dr) + (dc < 0 ? -dc : dc);
}

static inline int is_valid(const MapDef *map, int r, int c) {
    return r >= 0 && r < map->rows && c >= 0 && c < map->cols
        && map->data[r * map->cols + c] == 0;
}

/* Helper: initialize cells array from map */
static inline void vis_init_cells(AlgoVis *vis, const MapDef *map) {
    int total = map->rows * map->cols;
    vis->rows = map->rows;
    vis->cols = map->cols;
    vis->start_node = get_index(map->cols, map->start_r, map->start_c);
    vis->end_node = get_index(map->cols, map->end_r, map->end_c);

    for (int i = 0; i < total; i++)
        vis->cells[i] = map->data[i] ? VIS_WALL : VIS_EMPTY;
    for (int i = total; i < MAX_NODES; i++)
        vis->cells[i] = VIS_EMPTY;

    vis->cells[vis->start_node] = VIS_START;
    vis->cells[vis->end_node] = VIS_END;
    vis->done = 0;
    vis->found = 0;
    vis->nodes_explored = 0;
    vis->steps = 0;
    vis->path_len = 0;
    vis->path_cost = 0;
    vis->relaxations = 0;
}

/* Helper: trace path from end to start using parent array */
static inline void vis_trace_path(AlgoVis *vis, const int *parent, const int *cost) {
    int end = vis->end_node;
    vis->path_cost = cost[end];
    int cur = end;
    while (cur != -1) {
        if (cur != vis->start_node && cur != vis->end_node)
            vis->cells[cur] = VIS_PATH;
        vis->path_len++;
        cur = parent[cur];
    }
}

/* ── Min-heap (used by Dijkstra, A*, JPS) ────────────────────────── */

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
