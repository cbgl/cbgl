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

/**
 * @brief `cbgl::freeSpaceOf`.
 */
#include "cbgl/free_space.hpp"

namespace cbgl
{

/**
 * @details Walks the grid column by column within each row, asking the map's
 * own `worldFromCell` for each cell's centre and its own `isFree` for whether
 * that centre stands on unblocked ground. Neither the occupancy threshold nor
 * the origin's translation and rotation are restated here.
 */
FreeSpace freeSpaceOf(const fsm_localisation::OccupancyGridMap& map)
{
  FreeSpace free_space{.cells = {}, .resolution = map.resolution()};

  for (unsigned int row = 0U; row < map.height(); row++)
    for (unsigned int column = 0U; column < map.width(); column++)
    {
      const fsm_localisation::WorldPoint centre = map.worldFromCell(
        fsm_localisation::CellCoordinates{
          .column = column + 0.5, .row = row + 0.5});

      if (map.isFree(centre.x, centre.y))
        free_space.cells.push_back(Point{.x = centre.x, .y = centre.y});
    }

  return free_space;
}

}
