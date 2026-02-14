/*
 * Grid Pathfinding Visualizer — SDL2
 *
 * Animates pathfinding algorithms step-by-step on 20x20 grids.
 * Algorithm-agnostic: each algorithm is a plugin (AlgoPlugin).
 *
 * Controls:
 *   Space       Step one node expansion
 *   Enter       Run to completion (animated)
 *   R           Reset current algorithm
 *   1           Dijkstra
 *   2           A*
 *   3           Bellman-Ford
 *   4           IDA*
 *   5           Floyd-Warshall
 *   Tab         Cycle maps (original, diagonal, arena, maze)
 *   +/-         Speed up / slow down animation
 *   Q / Escape  Quit
 *
 * Build:
 *   just visualizer
 */

#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "algo.h"

/* ── Maps ─────────────────────────────────────────────────────────── */

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

/* ── Algorithm plugins ────────────────────────────────────────────── */

extern AlgoPlugin algo_dijkstra;
extern AlgoPlugin algo_astar;
extern AlgoPlugin algo_bellman_ford;
extern AlgoPlugin algo_ida_star;
extern AlgoPlugin algo_floyd_warshall;

static AlgoPlugin *algorithms[] = {
    &algo_dijkstra, &algo_astar, &algo_bellman_ford,
    &algo_ida_star, &algo_floyd_warshall,
};
#define ALG_COUNT 5

static int current_alg = 0;
static AlgoVis *vis = NULL;

/* Per-algorithm info bar colors */
static const SDL_Color alg_colors[ALG_COUNT] = {
    {255, 160, 80,  255},  /* Dijkstra: orange */
    {100, 180, 255, 255},  /* A*: blue */
    {50,  230, 100, 255},  /* Bellman-Ford: green */
    {180, 100, 255, 255},  /* IDA*: purple */
    {255, 220, 50,  255},  /* Floyd-Warshall: yellow */
};

/* ── Timing ───────────────────────────────────────────────────────── */

static double step_us  = 0.0;
static double total_us = 0.0;

static void timed_step(void) {
    Uint64 t0 = SDL_GetPerformanceCounter();
    algorithms[current_alg]->step(vis);
    Uint64 t1 = SDL_GetPerformanceCounter();
    double us = (double)(t1 - t0) * 1e6 / (double)SDL_GetPerformanceFrequency();
    step_us = us;
    total_us += us;
}

static void init_algorithm(void) {
    vis = algorithms[current_alg]->init(maps[current_map]);
    step_us = 0.0;
    total_us = 0.0;
}

/* ── Rendering ────────────────────────────────────────────────────── */

#define CELL_SIZE 32
#define GRID_PAD  1
#define INFO_H    60
#define WIN_W     (COLS * CELL_SIZE)
#define WIN_H     (ROWS * CELL_SIZE + INFO_H)

static const SDL_Color COL_BG        = {30,  30,  30,  255};
static const SDL_Color COL_WALL      = {60,  60,  70,  255};
static const SDL_Color COL_EMPTY     = {200, 200, 200, 255};
static const SDL_Color COL_OPEN      = {100, 180, 255, 255};
static const SDL_Color COL_CLOSED    = {255, 160, 80,  255};
static const SDL_Color COL_PATH      = {50,  230, 100, 255};
static const SDL_Color COL_START     = {255, 255, 60,  255};
static const SDL_Color COL_END       = {230, 50,  50,  255};
static const SDL_Color COL_GRID_LINE = {45,  45,  50,  255};

static SDL_Color cell_color(int state) {
    switch (state) {
        case VIS_WALL:   return COL_WALL;
        case VIS_OPEN:   return COL_OPEN;
        case VIS_CLOSED: return COL_CLOSED;
        case VIS_PATH:   return COL_PATH;
        case VIS_START:  return COL_START;
        case VIS_END:    return COL_END;
        default:         return COL_EMPTY;
    }
}

static void render_grid(SDL_Renderer *ren) {
    SDL_SetRenderDrawColor(ren, COL_BG.r, COL_BG.g, COL_BG.b, 255);
    SDL_RenderClear(ren);

    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            int idx = get_index(r, c);
            SDL_Color col = cell_color(vis->cells[idx]);
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

    SDL_SetRenderDrawColor(ren, COL_GRID_LINE.r, COL_GRID_LINE.g,
                           COL_GRID_LINE.b, 255);
    for (int r = 0; r <= ROWS; r++)
        SDL_RenderDrawLine(ren, 0, r * CELL_SIZE, WIN_W, r * CELL_SIZE);
    for (int c = 0; c <= COLS; c++)
        SDL_RenderDrawLine(ren, c * CELL_SIZE, 0, c * CELL_SIZE, ROWS * CELL_SIZE);
}

static void draw_char_block(SDL_Renderer *ren, int x, int y, int w, int h) {
    SDL_Rect r = {x, y, w, h};
    SDL_RenderFillRect(ren, &r);
}

static void render_info(SDL_Renderer *ren, int step_ms) {
    int y0 = ROWS * CELL_SIZE + 4;

    SDL_Rect bar = {0, ROWS * CELL_SIZE, WIN_W, INFO_H};
    SDL_SetRenderDrawColor(ren, 20, 20, 25, 255);
    SDL_RenderFillRect(ren, &bar);

    /* Algorithm indicator — colored block */
    SDL_Color ac = alg_colors[current_alg];
    SDL_SetRenderDrawColor(ren, ac.r, ac.g, ac.b, 255);
    draw_char_block(ren, 8, y0 + 4, 12, 12);

    /* Status indicator */
    if (vis->done) {
        SDL_Color sc = vis->found ? COL_PATH : COL_END;
        SDL_SetRenderDrawColor(ren, sc.r, sc.g, sc.b, 255);
        draw_char_block(ren, WIN_W - 20, y0 + 4, 12, 12);
    }

    /* Legend blocks */
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

    /* Progress bar */
    int total_open = 0;
    for (int i = 0; i < MAX_NODES; i++)
        if (!maps[current_map][i / COLS][i % COLS]) total_open++;
    int bar_w = (vis->nodes_explored * (WIN_W - 16)) / (total_open > 0 ? total_open : 1);
    if (bar_w > WIN_W - 16) bar_w = WIN_W - 16;
    SDL_SetRenderDrawColor(ren, 80, 80, 100, 255);
    SDL_Rect prog = {8, y0 + 48, bar_w, 6};
    SDL_RenderFillRect(ren, &prog);
}

/* ── Terminal stats ───────────────────────────────────────────────── */

#define STATS_LINES 4

static void print_stats(int step_ms, int first) {
    if (!first)
        printf("\033[%dA", STATS_LINES);

    const char *status = vis->done ? (vis->found ? "FOUND" : "NO PATH") : "searching";
    int path_cost = vis->found ? vis->path_cost : -1;

    printf("\033[K  %-10s %-14s %s\n",
           map_names[current_map], algorithms[current_alg]->name, status);

    char step_buf[32], total_buf[32];
    snprintf(step_buf, sizeof(step_buf), "%.1fus", step_us);
    snprintf(total_buf, sizeof(total_buf), "%.1fus", total_us);

    if (vis->found)
        printf("\033[K  explored: %-8d steps: %-8d  path: %d (%d nodes)\n",
               vis->nodes_explored, vis->steps, path_cost, vis->path_len);
    else
        printf("\033[K  explored: %-8d steps: %-8d  path: --\n",
               vis->nodes_explored, vis->steps);

    printf("\033[K  step:     %-8s total: %-8s speed: %dms\n",
           step_buf, total_buf, step_ms);

    double nps = (total_us > 0.0) ? (vis->nodes_explored * 1e6 / total_us) : 0.0;
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

    init_algorithm();

    int running = 1;
    int auto_run = 0;
    int step_ms = 40;
    Uint32 last_step = 0;

    printf("Pathfinding Visualizer\n");
    printf("  Space = step       Enter = auto-run   R   = reset\n");
    printf("  1 = Dijkstra  2 = A*  3 = Bellman-Ford  4 = IDA*  5 = Floyd-Warshall\n");
    printf("  Tab = next map     +/- = speed        Q/Esc = quit\n");
    printf("\n");
    print_stats(step_ms, 1);

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
                    timed_step();
                    break;
                case SDLK_RETURN:
                    auto_run = !auto_run;
                    break;
                case SDLK_r:
                    init_algorithm();
                    auto_run = 0;
                    break;
                case SDLK_1:
                    current_alg = 0;
                    init_algorithm();
                    auto_run = 0;
                    break;
                case SDLK_2:
                    current_alg = 1;
                    init_algorithm();
                    auto_run = 0;
                    break;
                case SDLK_3:
                    current_alg = 2;
                    init_algorithm();
                    auto_run = 0;
                    break;
                case SDLK_4:
                    current_alg = 3;
                    init_algorithm();
                    auto_run = 0;
                    break;
                case SDLK_5:
                    current_alg = 4;
                    init_algorithm();
                    auto_run = 0;
                    break;
                case SDLK_TAB:
                    current_map = (current_map + 1) % MAP_COUNT;
                    init_algorithm();
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

        if (auto_run && !vis->done) {
            Uint32 now = SDL_GetTicks();
            if (now - last_step >= (Uint32)step_ms) {
                timed_step();
                last_step = now;
            }
        }

        render_grid(ren);
        render_info(ren, step_ms);
        SDL_RenderPresent(ren);

        print_stats(step_ms, 0);

        SDL_Delay(8);
    }

    printf("\n");

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
