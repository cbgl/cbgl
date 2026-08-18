// Copyright 2026 Alexandros Filotheou
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CBGL__FREE_SPACE_HPP_
#define CBGL__FREE_SPACE_HPP_

#include <fsm_localisation/occupancy_grid_map.hpp>

#include "cbgl/cbgl.hpp"

namespace cbgl
{
/**
 * @brief The free space an occupancy grid offers.
 * @details Every cell the map calls free, as a world position, alongside the
 * resolution those positions were drawn at. The map converts its own cells to
 * the world through its own origin, translation and yaw both, so a rotated map
 * costs no second copy of that arithmetic here.
 * @remark A cell is free exactly when the map says it is, so the occupancy
 * threshold and the treatment of unknown cells are the map's business and are
 * not restated.
 */
FreeSpace freeSpaceOf(const fsm_localisation::OccupancyGridMap& map);

}

#endif
