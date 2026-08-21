/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.h 1713 2003-08-23 04:03:43Z inspectorg $
 **************************************************************************/

#ifndef MAP_H
#define MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
struct _rtk_fig_t;


// Limits
#define MAP_WIFI_MAX_LEVELS 8


// Description for a single map cell.
typedef struct
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int occ_state;

  // Distance to the nearest occupied cell
  double occ_dist;

  // Wifi levels
  //int wifi_levels[MAP_WIFI_MAX_LEVELS];

} map_cell_t;


// Description for a map
typedef struct
{
  // Map origin; the map is a viewport onto a conceptual larger map. This is
  // the centre of the viewport, in world coordinates.
  double origin_x, origin_y;

  // Orientation of the map's own axes about z, in world coordinates, with its
  // sine and cosine held beside it so that the conversion macros below need
  // not recompute them on every call.
  double origin_yaw, origin_sin_yaw, origin_cos_yaw;

  // Map scale (m/cell)
  double scale;

  // Map dimensions (number of cells)
  int size_x, size_y;

  // The map data, stored as a grid
  map_cell_t *cells;

  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;

} map_t;



/*******************************************************************************
 * Basic map functions
 ******************************************************************************/

/*******************************************************************************
 */
// Create a new (empty) map
map_t *map_alloc(void);

/*******************************************************************************
 */
// Destroy a map
void map_free(map_t *map);

/*******************************************************************************
 */
// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa);

/*******************************************************************************
 */
// Load an occupancy map
int map_load_occ(map_t *map, const char *filename, double scale, int negate);

/*******************************************************************************
 */
// Load a wifi signal strength map
//int map_load_wifi(map_t *map, const char *filename, int index);

/*******************************************************************************
 */
// Update the cspace distances
void map_update_cspace(map_t *map, double max_occ_dist);


/*******************************************************************************
 * Range functions
 ******************************************************************************/

/*******************************************************************************
 */
// Extract a single range reading from the map
double map_calc_range(map_t *map, double ox, double oy, double oa, double max_range);


/*******************************************************************************
 * GUI/diagnostic functions
 ******************************************************************************/

/*******************************************************************************
 */
// Draw the occupancy grid
void map_draw_occ(map_t *map, struct _rtk_fig_t *fig);

/*******************************************************************************
 */
// Draw the cspace map
void map_draw_cspace(map_t *map, struct _rtk_fig_t *fig);

/*******************************************************************************
 */
// Draw a wifi map
void map_draw_wifi(map_t *map, struct _rtk_fig_t *fig, int index);


/*******************************************************************************
 * Map manipulation macros
 *******************************************************************************/

// Offset of a map index from the map centre, along the map's own axes
#define MAP_OXGX(map, i) (((i) - map->size_x / 2) * map->scale)
#define MAP_OYGY(map, j) (((j) - map->size_y / 2) * map->scale)

// Convert from map index to world coords. Both indices are needed on each
// axis: where the map's own axes are turned against the world's, a cell's
// world x depends on its row as well as on its column.
#define MAP_WXGX(map, i, j) (map->origin_x \
  + map->origin_cos_yaw * MAP_OXGX(map, i) \
  - map->origin_sin_yaw * MAP_OYGY(map, j))
#define MAP_WYGY(map, i, j) (map->origin_y \
  + map->origin_sin_yaw * MAP_OXGX(map, i) \
  + map->origin_cos_yaw * MAP_OYGY(map, j))

// Offset of a world position from the map centre, along the map's own axes
#define MAP_OXWX(map, x, y) ( \
    map->origin_cos_yaw * ((x) - map->origin_x) \
  + map->origin_sin_yaw * ((y) - map->origin_y))
#define MAP_OYWY(map, x, y) ( \
  - map->origin_sin_yaw * ((x) - map->origin_x) \
  + map->origin_cos_yaw * ((y) - map->origin_y))

// Convert from world coords to map coords
#define MAP_GXWX(map, x, y) \
  (floor(MAP_OXWX(map, x, y) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, x, y) \
  (floor(MAP_OYWY(map, x, y) / map->scale + 0.5) + map->size_y / 2)

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords.
#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)

#ifdef __cplusplus
}
#endif

#endif
