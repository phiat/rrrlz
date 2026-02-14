/*
 * Grid Pathfinding Visualizer — SDL2
 *
 * Animates Dijkstra and A* step-by-step on the same 20x20 grid
 * used by the other programs in this project.
 *
 * Controls:
 *   Space       Step one node expansion
 *   Enter       Run to completion (animated)
 *   R           Reset current algorithm
 *   1           Switch to Dijkstra
 *   2           Switch to A*
 *   Tab         Cycle maps (original, diagonal, arena, maze)
 *   +/-         Speed up / slow down animation
 *   Q / Escape  Quit
 *
 * Build:
 *   clang -O2 visualizer.c -o visualizer $(pkg-config --cflags --libs sdl2)
 */

#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>

/* ── Grid & Maps ──────────────────────────────────────────────────── */

#define ROWS 20
#define COLS 20
#define MAX_NODES (ROWS * COLS)

/* Map 0: Original — scattered obstacles (same as dijkstra.c / astar.c) */
static const int map_original[ROWS][COLS] = {
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

/* Map 1: Diagonal walls — two crossing diagonal barriers with gaps */
static const int map_diagonal[ROWS][COLS] = {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
    {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
    {0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
    {0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
    {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
    {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};

/* Map 2: Arena — open center with perimeter walls and scattered pillars */
static const int map_arena[ROWS][COLS] = {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
    {0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0},
    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0},
    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
    {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};

/* Map 3: Maze — tight single-width corridors */
static const int map_maze[ROWS][COLS] = {
    {0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0},
    {0,1,0,1,1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,0},
    {0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0},
    {1,1,0,1,0,1,1,1,1,1,0,1,0,1,1,1,1,0,1,0},
    {0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0},
    {0,1,1,1,1,1,0,1,0,1,0,1,1,1,1,1,1,0,1,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0},
    {0,1,1,1,0,1,1,1,1,1,1,1,0,1,1,0,1,1,1,1},
    {0,0,0,1,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0},
    {1,1,0,1,1,1,0,1,1,1,0,1,1,0,1,1,1,1,1,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,0,1,0,1,1,0,1,0,1,1,1,1,1,0},
    {0,0,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0,0,0,0},
    {0,1,1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1},
    {0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0},
    {0,1,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,1,0},
    {0,0,0,1,0,0,1,0,1,0,0,1,0,0,0,0,0,0,0,0},
    {0,1,1,1,0,1,1,0,1,1,1,0,1,1,1,1,1,0,1,0},
    {0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0},
    {0,0,1,1,0,0,0,1,0,1,1,0,0,0,1,1,0,0,0,0},
};

#define MAP_COUNT 4
static const char *map_names[MAP_COUNT] = {
    "Original", "Diagonal", "Arena", "Maze"
};
static const int (*maps[MAP_COUNT])[COLS] = {
    map_original, map_diagonal, map_arena, map_maze
};
static int current_map = 0;

static const int START_R = 0, START_C = 0;
static const int END_R = 19, END_C = 19;

static const int DR[4] = {-1, 1, 0, 0};
static const int DC[4] = {0, 0, -1, 1};

static inline int get_index(int r, int c) { return r * COLS + c; }

static inline int is_valid(int r, int c) {
    return r >= 0 && r < ROWS && c >= 0 && c < COLS
        && maps[current_map][r][c] == 0;
}

static int manhattan(int r, int c) {
    int dr = r - END_R;
    int dc = c - END_C;
    return (dr < 0 ? -dr : dr) + (dc < 0 ? -dc : dc);
}

/* ── Min-heap ─────────────────────────────────────────────────────── */

typedef struct {
    int node;
    int priority;
} HeapEntry;

typedef struct {
    HeapEntry data[MAX_NODES * 4]; /* allow duplicates */
    int size;
} Heap;

static void heap_init(Heap *h) { h->size = 0; }

static void heap_push(Heap *h, int node, int priority) {
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

static HeapEntry heap_pop(Heap *h) {
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

/* ── Algorithm state ──────────────────────────────────────────────── */

enum CellState {
    CELL_EMPTY,
    CELL_WALL,
    CELL_OPEN,    /* in the open set (frontier) */
    CELL_CLOSED,  /* expanded / visited */
    CELL_PATH,
    CELL_START,
    CELL_END,
};

enum Algorithm {
    ALG_DIJKSTRA,
    ALG_ASTAR,
    ALG_COUNT,
};

static const char *alg_names[ALG_COUNT] = {"Dijkstra", "A*"};

typedef struct {
    Heap heap;
    int cost[MAX_NODES];      /* g-cost */
    int parent[MAX_NODES];
    int closed[MAX_NODES];
    int in_open[MAX_NODES];   /* for coloring frontier */
    int cell_state[MAX_NODES];
    int done;                 /* search finished? */
    int found;                /* path found? */
    int nodes_explored;
    int steps;            /* total heap pops (including stale skips) */
    int path_len;
    double step_us;       /* last step time in microseconds */
    double total_us;      /* cumulative algo time in microseconds */
    enum Algorithm alg;
} SearchState;

static void search_init(SearchState *s, enum Algorithm alg) {
    memset(s, 0, sizeof(*s));
    heap_init(&s->heap);
    s->alg = alg;
    s->done = 0;
    s->found = 0;
    s->nodes_explored = 0;
    s->steps = 0;
    s->path_len = 0;
    s->step_us = 0.0;
    s->total_us = 0.0;

    for (int i = 0; i < MAX_NODES; i++) {
        s->cost[i] = INT_MAX;
        s->parent[i] = -1;
        s->closed[i] = 0;
        s->in_open[i] = 0;
        int r = i / COLS, c = i % COLS;
        s->cell_state[i] = maps[current_map][r][c] ? CELL_WALL : CELL_EMPTY;
    }

    int start = get_index(START_R, START_C);
    s->cost[start] = 0;
    s->in_open[start] = 1;
    s->cell_state[start] = CELL_START;

    int priority = (alg == ALG_ASTAR) ? manhattan(START_R, START_C) : 0;
    heap_push(&s->heap, start, priority);

    s->cell_state[get_index(END_R, END_C)] = CELL_END;
}

/* Expand one node. Returns 1 if a step happened, 0 if done. */
static int search_step(SearchState *s) {
    if (s->done) return 0;
    if (s->heap.size == 0) { s->done = 1; return 0; }

    /* Pop next node */
    HeapEntry cur = heap_pop(&s->heap);
    int node = cur.node;
    int r = node / COLS, c = node % COLS;
    s->steps++;

    if (s->closed[node]) return 1; /* skip stale entry, but count as a step */

    s->closed[node] = 1;
    s->in_open[node] = 0;
    s->nodes_explored++;

    /* Color it closed (preserve start/end markers) */
    if (node != get_index(START_R, START_C) && node != get_index(END_R, END_C))
        s->cell_state[node] = CELL_CLOSED;

    int end = get_index(END_R, END_C);
    if (node == end) {
        s->done = 1;
        s->found = 1;
        /* Trace path */
        int cur_node = end;
        while (cur_node != -1) {
            if (cur_node != get_index(START_R, START_C) &&
                cur_node != get_index(END_R, END_C))
                s->cell_state[cur_node] = CELL_PATH;
            s->path_len++;
            cur_node = s->parent[cur_node];
        }
        return 1;
    }

    /* Expand neighbors */
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!is_valid(nr, nc)) continue;
        int neighbor = get_index(nr, nc);
        if (s->closed[neighbor]) continue;

        int new_g = s->cost[node] + 1;
        if (new_g < s->cost[neighbor]) {
            s->cost[neighbor] = new_g;
            s->parent[neighbor] = node;
            s->in_open[neighbor] = 1;

            int priority = new_g;
            if (s->alg == ALG_ASTAR)
                priority += manhattan(nr, nc);
            heap_push(&s->heap, neighbor, priority);

            /* Color frontier (preserve start/end) */
            if (neighbor != get_index(START_R, START_C) &&
                neighbor != get_index(END_R, END_C))
                s->cell_state[neighbor] = CELL_OPEN;
        }
    }

    return 1;
}

/* ── Rendering ────────────────────────────────────────────────────── */

#define CELL_SIZE 32
#define GRID_PAD  1
#define INFO_H    60
#define WIN_W     (COLS * CELL_SIZE)
#define WIN_H     (ROWS * CELL_SIZE + INFO_H)

/* Colors (R, G, B) */
static const SDL_Color COL_BG       = {30,  30,  30,  255};
static const SDL_Color COL_WALL     = {60,  60,  70,  255};
static const SDL_Color COL_EMPTY    = {200, 200, 200, 255};
static const SDL_Color COL_OPEN     = {100, 180, 255, 255};  /* frontier: blue */
static const SDL_Color COL_CLOSED   = {255, 160, 80,  255};  /* visited: orange */
static const SDL_Color COL_PATH     = {50,  230, 100, 255};  /* path: green */
static const SDL_Color COL_START    = {255, 255, 60,  255};  /* yellow */
static const SDL_Color COL_END      = {230, 50,  50,  255};  /* red */
static const SDL_Color COL_GRID_LINE = {45, 45,  50,  255};

static SDL_Color cell_color(int state) {
    switch (state) {
        case CELL_WALL:   return COL_WALL;
        case CELL_OPEN:   return COL_OPEN;
        case CELL_CLOSED: return COL_CLOSED;
        case CELL_PATH:   return COL_PATH;
        case CELL_START:  return COL_START;
        case CELL_END:    return COL_END;
        default:          return COL_EMPTY;
    }
}

static void render_grid(SDL_Renderer *ren, SearchState *s) {
    /* Background */
    SDL_SetRenderDrawColor(ren, COL_BG.r, COL_BG.g, COL_BG.b, 255);
    SDL_RenderClear(ren);

    /* Cells */
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            int idx = get_index(r, c);
            SDL_Color col = cell_color(s->cell_state[idx]);
            SDL_Rect rect = {
                c * CELL_SIZE + GRID_PAD,
                r * CELL_SIZE + GRID_PAD,
                CELL_SIZE - 2 * GRID_PAD,
                CELL_SIZE - 2 * GRID_PAD
            };
            SDL_SetRenderDrawColor(ren, col.r, col.g, col.b, 255);
            SDL_RenderFillRect(ren, &rect);
        }
    }

    /* Grid lines */
    SDL_SetRenderDrawColor(ren, COL_GRID_LINE.r, COL_GRID_LINE.g,
                           COL_GRID_LINE.b, 255);
    for (int r = 0; r <= ROWS; r++)
        SDL_RenderDrawLine(ren, 0, r * CELL_SIZE, WIN_W, r * CELL_SIZE);
    for (int c = 0; c <= COLS; c++)
        SDL_RenderDrawLine(ren, c * CELL_SIZE, 0, c * CELL_SIZE, ROWS * CELL_SIZE);
}

/* Simple text via filled rects — no SDL_ttf needed */
static void draw_char_block(SDL_Renderer *ren, int x, int y, int w, int h) {
    SDL_Rect r = {x, y, w, h};
    SDL_RenderFillRect(ren, &r);
}

static void render_info(SDL_Renderer *ren, SearchState *s, int step_ms) {
    int y0 = ROWS * CELL_SIZE + 4;

    /* Info bar background */
    SDL_Rect bar = {0, ROWS * CELL_SIZE, WIN_W, INFO_H};
    SDL_SetRenderDrawColor(ren, 20, 20, 25, 255);
    SDL_RenderFillRect(ren, &bar);

    /* Algorithm indicator — colored block */
    SDL_Color ac = (s->alg == ALG_DIJKSTRA) ? COL_CLOSED : COL_OPEN;
    SDL_SetRenderDrawColor(ren, ac.r, ac.g, ac.b, 255);
    draw_char_block(ren, 8, y0 + 4, 12, 12);

    /* Status indicators on the right side */
    if (s->done) {
        SDL_Color sc = s->found ? COL_PATH : COL_END;
        SDL_SetRenderDrawColor(ren, sc.r, sc.g, sc.b, 255);
        draw_char_block(ren, WIN_W - 20, y0 + 4, 12, 12);
    }

    /* Legend blocks at bottom */
    int lx = 8, ly = y0 + 28;
    struct { SDL_Color c; } legend[] = {
        {COL_EMPTY}, {COL_WALL}, {COL_OPEN}, {COL_CLOSED}, {COL_PATH},
        {COL_START}, {COL_END},
    };
    for (int i = 0; i < 7; i++) {
        SDL_SetRenderDrawColor(ren, legend[i].c.r, legend[i].c.g,
                               legend[i].c.b, 255);
        draw_char_block(ren, lx, ly, 14, 14);
        lx += 22;
    }

    /* Node count as bar width (proportional to explored / total open cells) */
    int total_open = 0;
    for (int i = 0; i < MAX_NODES; i++)
        if (!maps[current_map][i / COLS][i % COLS]) total_open++;
    int bar_w = (s->nodes_explored * (WIN_W - 16)) / (total_open > 0 ? total_open : 1);
    if (bar_w > WIN_W - 16) bar_w = WIN_W - 16;
    SDL_SetRenderDrawColor(ren, 80, 80, 100, 255);
    SDL_Rect prog = {8, y0 + 48, bar_w, 6};
    SDL_RenderFillRect(ren, &prog);
}

/* ── Terminal stats (TUI) ─────────────────────────────────────────── */

#define STATS_LINES 4

/* Perform a timed search step — updates step_us and total_us */
static void timed_step(SearchState *s) {
    Uint64 t0 = SDL_GetPerformanceCounter();
    search_step(s);
    Uint64 t1 = SDL_GetPerformanceCounter();
    double us = (double)(t1 - t0) * 1e6 / (double)SDL_GetPerformanceFrequency();
    s->step_us = us;
    s->total_us += us;
}

static void print_stats(SearchState *s, int step_ms, int first) {
    /* Move cursor up to overwrite previous stats (not on first print) */
    if (!first)
        printf("\033[%dA", STATS_LINES);

    const char *status = s->done ? (s->found ? "FOUND" : "NO PATH") : "searching";
    int path_cost = s->found ? s->cost[get_index(END_R, END_C)] : -1;

    /* Line 1: map + algo + status */
    printf("\033[K  %-10s %-10s %s\n",
           map_names[current_map], alg_names[s->alg], status);

    /* Line 2: explored + steps + path */
    char step_buf[32], total_buf[32];
    snprintf(step_buf, sizeof(step_buf), "%.1fus", s->step_us);
    snprintf(total_buf, sizeof(total_buf), "%.1fus", s->total_us);

    if (s->found)
        printf("\033[K  explored: %-8d steps: %-8d  path: %d (%d nodes)\n",
               s->nodes_explored, s->steps, path_cost, s->path_len);
    else
        printf("\033[K  explored: %-8d steps: %-8d  path: --\n",
               s->nodes_explored, s->steps);

    /* Line 3: timing */
    printf("\033[K  step:     %-8s total: %-8s speed: %dms\n",
           step_buf, total_buf, step_ms);

    /* Line 4: nodes/sec */
    double nps = (s->total_us > 0.0) ? (s->nodes_explored * 1e6 / s->total_us) : 0.0;
    char nps_buf[32];
    snprintf(nps_buf, sizeof(nps_buf), "%.0f", nps);
    printf("\033[K  nodes/s:  %s\n", nps_buf);

    fflush(stdout);
}

/* ── Main ─────────────────────────────────────────────────────────── */

int main(int argc, char *argv[]) {
    (void)argc; (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *win = SDL_CreateWindow(
        "rrrlz — Pathfinding Visualizer",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_W, WIN_H, 0
    );
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (!ren)
        ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    if (!ren) {
        fprintf(stderr, "SDL_CreateRenderer: %s\n", SDL_GetError());
        SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }

    SearchState state;
    enum Algorithm current_alg = ALG_DIJKSTRA;
    search_init(&state, current_alg);

    int running = 1;
    int auto_run = 0;
    int step_ms = 40;
    Uint32 last_step = 0;

    /* Print header + initial stats block */
    printf("Pathfinding Visualizer\n");
    printf("  Space = step       Enter = auto-run   R   = reset\n");
    printf("  1     = Dijkstra   2     = A*         Tab = next map\n");
    printf("  +/-   = speed      Q/Esc = quit\n");
    printf("\n");
    print_stats(&state, step_ms, 1);

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = 0;
            } else if (ev.type == SDL_KEYDOWN) {
                switch (ev.key.keysym.sym) {
                case SDLK_q:
                case SDLK_ESCAPE:
                    running = 0;
                    break;
                case SDLK_SPACE:
                    auto_run = 0;
                    timed_step(&state);
                    break;
                case SDLK_RETURN:
                    auto_run = !auto_run;
                    break;
                case SDLK_r:
                    search_init(&state, current_alg);
                    auto_run = 0;
                    break;
                case SDLK_1:
                    current_alg = ALG_DIJKSTRA;
                    search_init(&state, current_alg);
                    auto_run = 0;
                    break;
                case SDLK_2:
                    current_alg = ALG_ASTAR;
                    search_init(&state, current_alg);
                    auto_run = 0;
                    break;
                case SDLK_TAB:
                    current_map = (current_map + 1) % MAP_COUNT;
                    search_init(&state, current_alg);
                    auto_run = 0;
                    break;
                case SDLK_EQUALS:
                case SDLK_PLUS:
                    if (step_ms > 5) step_ms -= 5;
                    break;
                case SDLK_MINUS:
                    if (step_ms < 500) step_ms += 5;
                    break;
                default:
                    break;
                }
            }
        }

        /* Auto-step */
        if (auto_run && !state.done) {
            Uint32 now = SDL_GetTicks();
            if (now - last_step >= (Uint32)step_ms) {
                timed_step(&state);
                last_step = now;
            }
        }

        /* Render */
        render_grid(ren, &state);
        render_info(ren, &state, step_ms);
        SDL_RenderPresent(ren);

        /* Update terminal stats */
        print_stats(&state, step_ms, 0);

        SDL_Delay(8);
    }

    /* Final newline so shell prompt doesn't clobber stats */
    printf("\n");

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
