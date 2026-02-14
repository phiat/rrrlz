/*
 * algo_ida_star.c — IDA* (Iterative Deepening A*) step machine
 *
 * Uses an explicit stack for DFS. Each step either pushes a neighbor
 * (expanding) or pops (backtracking). When the stack empties, bumps
 * the threshold and restarts.
 */

#include "algo.h"

#define IDA_MAX_STACK (MAX_NODES * 2)

typedef struct {
    int node;
    int g;
    int next_dir;  /* next direction to try (0-3), 4 = done expanding */
} StackFrame;

typedef struct {
    AlgoVis vis;
    const MapDef *map;
    StackFrame stack[IDA_MAX_STACK];
    int sp;                    /* stack pointer */
    int threshold;
    int next_threshold;        /* min f that exceeded threshold */
    int on_path[MAX_NODES];    /* nodes currently on the DFS stack */
    int visited[MAX_NODES];    /* nodes visited in current iteration (for coloring) */
    int parent[MAX_NODES];     /* for path tracing */
    int cost[MAX_NODES];       /* for path cost reporting */
} IDAStarState;

static IDAStarState state;

static void ida_start_iteration(IDAStarState *s) {
    int total = s->map->rows * s->map->cols;
    s->sp = 0;
    s->next_threshold = INT_MAX;
    memset(s->on_path, 0, total * sizeof(int));
    memset(s->visited, 0, total * sizeof(int));

    /* Reset cell colors (keep walls, start, end) */
    for (int i = 0; i < total; i++) {
        if (s->vis.cells[i] != VIS_WALL &&
            i != s->vis.start_node &&
            i != s->vis.end_node)
            s->vis.cells[i] = VIS_EMPTY;
    }

    int start = s->vis.start_node;
    s->stack[0].node = start;
    s->stack[0].g = 0;
    s->stack[0].next_dir = 0;
    s->sp = 1;
    s->on_path[start] = 1;
    s->visited[start] = 1;
}

static AlgoVis *ida_star_init(const MapDef *map) {
    memset(&state, 0, sizeof(state));
    state.map = map;
    vis_init_cells(&state.vis, map);

    int total = map->rows * map->cols;
    for (int i = 0; i < total; i++) {
        state.parent[i] = -1;
        state.cost[i] = INT_MAX;
    }
    state.cost[state.vis.start_node] = 0;

    state.threshold = manhattan(map->start_r, map->start_c,
                                map->end_r, map->end_c);
    ida_start_iteration(&state);

    return &state.vis;
}

static int ida_star_step(AlgoVis *vis) {
    IDAStarState *s = (IDAStarState *)vis;
    if (s->vis.done) return 0;

    int cols = s->vis.cols;
    s->vis.steps++;

    /* Stack empty → need new iteration */
    if (s->sp == 0) {
        if (s->next_threshold == INT_MAX) {
            /* No path exists */
            s->vis.done = 1;
            return 0;
        }
        s->threshold = s->next_threshold;
        ida_start_iteration(s);
        return 1;
    }

    StackFrame *top = &s->stack[s->sp - 1];
    int node = top->node;
    int r = node / cols, c = node % cols;
    int g = top->g;

    /* Try to expand to next direction */
    while (top->next_dir < 4) {
        int d = top->next_dir++;
        int nr = r + DR[d], nc = c + DC[d];
        if (!is_valid(s->map, nr, nc)) continue;

        int neighbor = get_index(cols, nr, nc);
        if (s->on_path[neighbor]) continue;

        int new_g = g + 1;
        int f = new_g + manhattan(nr, nc, s->map->end_r, s->map->end_c);

        if (f > s->threshold) {
            if (f < s->next_threshold)
                s->next_threshold = f;
            continue;
        }

        /* Push neighbor */
        s->vis.relaxations++;
        s->on_path[neighbor] = 1;
        s->parent[neighbor] = node;
        s->cost[neighbor] = new_g;

        if (!s->visited[neighbor]) {
            s->visited[neighbor] = 1;
            s->vis.nodes_explored++;
        }

        /* Color: on current path */
        if (neighbor != s->vis.start_node &&
            neighbor != s->vis.end_node)
            s->vis.cells[neighbor] = VIS_OPEN;

        /* Check if we found the goal */
        if (neighbor == s->vis.end_node) {
            s->vis.done = 1;
            s->vis.found = 1;
            vis_trace_path(&s->vis, s->parent, s->cost);
            return 1;
        }

        /* Push new frame */
        if (s->sp < IDA_MAX_STACK) {
            s->stack[s->sp].node = neighbor;
            s->stack[s->sp].g = new_g;
            s->stack[s->sp].next_dir = 0;
            s->sp++;
        }

        return 1;
    }

    /* All directions exhausted — backtrack */
    s->sp--;
    s->on_path[node] = 0;

    if (node != s->vis.start_node &&
        node != s->vis.end_node)
        s->vis.cells[node] = VIS_CLOSED;

    return 1;
}

AlgoPlugin algo_ida_star = {
    .name = "IDA*",
    .init = ida_star_init,
    .step = ida_star_step,
};
