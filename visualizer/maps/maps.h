/* maps.h — Map registry for pathfinding visualizer */

#ifndef MAPS_H
#define MAPS_H

#include "map_original.h"
#include "map_diagonal.h"
#include "map_arena.h"
#include "map_maze.h"
#include "map_wide_open.h"
#include "map_bottleneck.h"
#include "map_spiral_50.h"
#include "map_rooms_50.h"

#define MAP_COUNT 8

static const MapDef *all_maps[MAP_COUNT] = {
    &map_original,
    &map_diagonal,
    &map_arena,
    &map_maze,
    &map_wide_open,
    &map_bottleneck,
    &map_spiral_50,
    &map_rooms_50,
};

#endif /* MAPS_H */
