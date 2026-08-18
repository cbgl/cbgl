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
 * @brief `cbgl::freeSpaceOf`: the world position of every cell an occupancy
 * grid calls free, and the resolution those positions were drawn at.
 * @details
 * **A cell is free exactly when the map says it is.** The occupancy threshold
 * and the treatment of unknown cells belong to the map, which decides them once
 * for the caster and for every `isFree` any part of this package asks. So the
 * expected set in the cases below is built by walking the grid and asking the
 * map itself, not by restating its rule. What is asserted is that nothing is
 * added to that answer, nothing is left out of it, and each cell is offered at
 * its own centre.
 * **The centre, and why it matters that it is the centre.** A hypothesis
 * position is a point drawn from this list, and a caster starts a ray from the
 * point it is given. A cell offered at its corner would put every hypothesis
 * half a cell diagonally away from where the algorithm means to stand, and on a
 * twenty centimetre map that is fourteen centimetres of error the refinement
 * has to absorb before it starts.
 * **The origin.** The map converts its own cells to the world through its full
 * origin, translation and yaw both, so a rotated grid needs no second copy of
 * that arithmetic here.
 * `ARotatedAndTranslatedOriginIsHonouredThroughTheMapsOwnArithmetic` is what
 * says the second copy has not been written: the ROS 1 build kept one and got
 * the sign of the origin wrong in it, so a map whose centre was not the world
 * origin sampled from the wrong rectangle altogether.
 * **Both halves of the answer are needed.** The count of hypotheses is
 * proportional to the free area, which is the cell count times the square of
 * the resolution, and neither half of that product is recoverable from the
 * other.
 * **Why no case carries a grid of no cells at all.** A grid of zero width or
 * zero height cannot be built: the map refuses it and names the field. The
 * emptiest map this function can ever be handed is therefore one cell, and
 * `AMapWhoseOnlyCellIsUnknownOffersNoFreeSpace` is that map.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <string>
#include <vector>

#include "cbgl/free_space.hpp"

namespace
{
using cbgl::FreeSpace;
using cbgl::Point;
using fsm_localisation::CastingMethod;
using fsm_localisation::CellCoordinates;
using fsm_localisation::OccupancyGridMap;
using fsm_localisation::WorldPoint;

const std::int8_t kBlocked = 100;
const std::int8_t kFree = 0;
const std::int8_t kUnknown = -1;

const int kOccupancyThreshold = 65;

const unsigned int kColumns = 7U;
const unsigned int kRows = 5U;
const double kResolution = 0.05;

/**
 * @brief One cell value, and whether a map built at `kOccupancyThreshold` is
 * expected to call a cell carrying it free.
 */
struct ValueCase
{
  std::int8_t value{kFree};
  bool free{true};
};

/**
 * @brief A grid's geometry, so that a case can place the same cells at a
 * different origin without a second grid builder.
 */
struct Geometry
{
  unsigned int columns{kColumns};
  unsigned int rows{kRows};
  double resolution{kResolution};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
};

class FreeSpaceOfCellValue : public testing::TestWithParam<ValueCase>
{
};

bool beforeInReading(const Point& first, const Point& second)
{
  return first.x != second.x ? first.x < second.x : first.y < second.y;
}

/**
 * @brief The cells the map itself calls free, at their own centres, in reading
 * order. This is the answer `freeSpaceOf` is held against, and it is the map's
 * own answer rather than a second reading of the message.
 */
std::vector<Point> everyCellTheMapCallsFree(const OccupancyGridMap& map)
{
  std::vector<Point> cells;

  for (unsigned int column = 0U; column < map.width(); column++)
    for (unsigned int row = 0U; row < map.height(); row++)
    {
      const WorldPoint centre = map.worldFromCell(CellCoordinates{
        .column = column + 0.5, .row = row + 0.5});

      if (map.isFree(centre.x, centre.y))
        cells.push_back(Point{.x = centre.x, .y = centre.y});
    }

  std::sort(cells.begin(), cells.end(), beforeInReading);

  return cells;
}

nav_msgs::msg::OccupancyGrid gridMessage(const Geometry& geometry,
  const std::int8_t fill)
{
  nav_msgs::msg::OccupancyGrid message;

  message.info.width = geometry.columns;
  message.info.height = geometry.rows;
  message.info.resolution = static_cast<float>(geometry.resolution);
  message.info.origin.position.x = geometry.origin_x;
  message.info.origin.position.y = geometry.origin_y;
  message.info.origin.orientation.z = std::sin(geometry.origin_yaw / 2.0);
  message.info.origin.orientation.w = std::cos(geometry.origin_yaw / 2.0);

  message.data.assign(
    static_cast<std::size_t>(geometry.columns) * geometry.rows, fill);

  return message;
}

std::shared_ptr<const OccupancyGridMap> mapOf(
  const nav_msgs::msg::OccupancyGrid& message)
{
  const auto built = OccupancyGridMap::build(
    message, kOccupancyThreshold, CastingMethod::ray_marching);

  return built.has_value() ? built.value() : nullptr;
}

/**
 * @brief A grid of mixed cells: a blocked border, one unknown cell, one cell
 * one below the occupancy threshold and one exactly at it.
 */
nav_msgs::msg::OccupancyGrid mixedMessage(const Geometry& geometry)
{
  nav_msgs::msg::OccupancyGrid message = gridMessage(geometry, kFree);

  const auto put =
    [&message, &geometry](const unsigned int column, const unsigned int row,
      const std::int8_t value)
    {
      message.data[static_cast<std::size_t>(row) * geometry.columns + column] =
        value;
    };

  for (unsigned int column = 0U; column < geometry.columns; column++)
  {
    put(column, 0U, kBlocked);
    put(column, geometry.rows - 1U, kBlocked);
  }

  for (unsigned int row = 0U; row < geometry.rows; row++)
  {
    put(0U, row, kBlocked);
    put(geometry.columns - 1U, row, kBlocked);
  }

  put(2U, 2U, kUnknown);
  put(3U, 2U, static_cast<std::int8_t>(kOccupancyThreshold - 1));
  put(4U, 2U, static_cast<std::int8_t>(kOccupancyThreshold));

  return message;
}

std::string valueCaseSuffix(const testing::TestParamInfo<ValueCase>& info)
{
  return std::format("{}{}_is_{}",
    info.param.value < 0 ? "minus_" : "",
    std::abs(static_cast<int>(info.param.value)),
    info.param.free ? "free" : "blocked");
}

}

TEST(FreeSpaceOf, AMapWhoseOnlyCellIsFreeOffersThatOneCentre)
{
  const Geometry geometry{
    .columns = 1U, .rows = 1U, .resolution = 0.20,
    .origin_x = -1.4, .origin_y = 2.6, .origin_yaw = 0.0};

  const std::shared_ptr<const OccupancyGridMap> map =
    mapOf(gridMessage(geometry, kFree));
  ASSERT_NE(map, nullptr);

  const FreeSpace free_space = cbgl::freeSpaceOf(*map);

  ASSERT_EQ(free_space.cells.size(), 1u)
    << "the map has one cell and it is free, so the free space is that one "
    << "cell. At least one hypothesis is drawn wherever a free cell exists, so "
    << "this is the smallest map this algorithm can still answer on.";

  EXPECT_DOUBLE_EQ(free_space.cells[0].x,
    geometry.origin_x + map->resolution() / 2.0)
    << "the only cell's centre is half a cell along each axis from the map's "
    << "origin, at half of " << map->resolution() << " metres rather than at "
    << "the origin itself.";
  EXPECT_DOUBLE_EQ(free_space.cells[0].y,
    geometry.origin_y + map->resolution() / 2.0);
}

TEST(FreeSpaceOf, AMapWhoseOnlyCellIsUnknownOffersNoFreeSpace)
{
  const std::shared_ptr<const OccupancyGridMap> map =
    mapOf(gridMessage(Geometry{.columns = 1U, .rows = 1U}, kUnknown));
  ASSERT_NE(map, nullptr);

  EXPECT_TRUE(cbgl::freeSpaceOf(*map).cells.empty())
    << "a map of one unknown cell offers nowhere to stand. It is the emptiest "
    << "map this function can be handed, a grid of no cells at all being "
    << "refused at the map's own construction, and the run it belongs to is "
    << "refused for want of free space rather than answered from a hypothesis "
    << "drawn out of thin air.";
}

TEST(FreeSpaceOf, AMapWithNoFreeCellOffersNoFreeSpace)
{
  const std::shared_ptr<const OccupancyGridMap> map =
    mapOf(gridMessage(Geometry{}, kBlocked));
  ASSERT_NE(map, nullptr);

  const FreeSpace free_space = cbgl::freeSpaceOf(*map);

  EXPECT_TRUE(free_space.cells.empty())
    << "every one of the " << kColumns * kRows << " cells is blocked, so the "
    << "free space carries none of them. It offered "
    << free_space.cells.size() << ".";

  EXPECT_EQ(free_space.resolution, map->resolution())
    << "a map with no free cell still has a resolution, and a host that reads "
    << "it back to compute an area needs it whether or not the area is zero.";
}

TEST(FreeSpaceOf, ARotatedAndTranslatedOriginIsHonouredThroughTheMapsOwnArithmetic)
{
  const Geometry geometry{
    .columns = kColumns, .rows = kRows, .resolution = kResolution,
    .origin_x = -3.2, .origin_y = 7.4, .origin_yaw = 0.7};

  const std::shared_ptr<const OccupancyGridMap> map =
    mapOf(mixedMessage(geometry));
  ASSERT_NE(map, nullptr);

  const std::vector<Point> expected = everyCellTheMapCallsFree(*map);
  ASSERT_FALSE(expected.empty());

  std::vector<Point> offered = cbgl::freeSpaceOf(*map).cells;
  std::sort(offered.begin(), offered.end(), beforeInReading);

  ASSERT_EQ(offered.size(), expected.size())
    << "the map calls " << expected.size() << " of its "
    << geometry.columns * geometry.rows << " cells free and " << offered.size()
    << " were offered.";

  for (std::size_t i = 0; i < expected.size(); i++)
  {
    EXPECT_DOUBLE_EQ(offered[i].x, expected[i].x)
      << "free cell " << i << " is offered at x " << offered[i].x << " where "
      << "the map's own origin, translated to (" << geometry.origin_x << ", "
      << geometry.origin_y << ") and turned by " << geometry.origin_yaw
      << " radians, puts it at " << expected[i].x << ". An origin applied with "
      << "the wrong sign, or a rotation left out, disperses hypotheses over a "
      << "part of the world the map does not describe.";
    EXPECT_DOUBLE_EQ(offered[i].y, expected[i].y)
      << "free cell " << i << " is offered at y " << offered[i].y << " where "
      << "the map puts it at " << expected[i].y << ".";
  }
}

TEST(FreeSpaceOf, EveryFreeCellIsOfferedAtItsOwnCentre)
{
  const Geometry geometry{
    .columns = kColumns, .rows = kRows, .resolution = kResolution,
    .origin_x = 1.25, .origin_y = -0.75, .origin_yaw = 0.0};

  const std::shared_ptr<const OccupancyGridMap> map =
    mapOf(mixedMessage(geometry));
  ASSERT_NE(map, nullptr);

  const std::vector<Point> expected = everyCellTheMapCallsFree(*map);

  const std::size_t inner =
    static_cast<std::size_t>(geometry.columns - 2U) * (geometry.rows - 2U);

  ASSERT_EQ(expected.size(), inner - 2u)
    << "the grid is bordered, so of its " << geometry.columns << " by "
    << geometry.rows << " cells only the " << inner << " inner ones can be "
    << "free. Of those, one is unknown and one stands at the occupancy "
    << "threshold, so " << inner - 2u << " are free. The case is worthless if "
    << "that ever stops being so.";

  std::vector<Point> offered = cbgl::freeSpaceOf(*map).cells;
  std::sort(offered.begin(), offered.end(), beforeInReading);

  ASSERT_EQ(offered.size(), expected.size())
    << expected.size() << " cells of this grid are free, the cell just below "
    << "the occupancy threshold among them, and " << offered.size()
    << " were offered. A cell at the threshold counted as free, or an unknown "
    << "cell counted as free, would offer two more; a border cell counted as "
    << "free would offer many more.";

  for (std::size_t i = 0; i < expected.size(); i++)
  {
    EXPECT_DOUBLE_EQ(offered[i].x, expected[i].x)
      << "free cell " << i << " is offered at x " << offered[i].x << " where "
      << "its own centre is " << expected[i].x << ". Half a cell out is a "
      << "corner rather than a centre, and every hypothesis drawn from this "
      << "list would stand half a cell diagonally from where the algorithm "
      << "means it to.";
    EXPECT_DOUBLE_EQ(offered[i].y, expected[i].y)
      << "free cell " << i << " is offered at y " << offered[i].y << " where "
      << "its own centre is " << expected[i].y << ".";
  }
}

TEST(FreeSpaceOf, TheResolutionOfferedIsTheMapsOwn)
{
  const Geometry geometry{
    .columns = kColumns, .rows = kRows, .resolution = 0.20,
    .origin_x = 0.0, .origin_y = 0.0, .origin_yaw = 0.0};

  const std::shared_ptr<const OccupancyGridMap> map =
    mapOf(mixedMessage(geometry));
  ASSERT_NE(map, nullptr);

  const FreeSpace free_space = cbgl::freeSpaceOf(*map);

  EXPECT_EQ(free_space.resolution, map->resolution())
    << "the free space carries " << free_space.resolution
    << " metres a cell where the map carries " << map->resolution()
    << ". The count of hypotheses is the density times the cell count times "
    << "the square of this number, so a resolution that is not the map's own "
    << "draws the wrong number of hypotheses over the right cells.";
}

TEST_P(FreeSpaceOfCellValue, ACellIsFreeExactlyWhereTheMapSaysItIs)
{
  const ValueCase value_case = GetParam();

  nav_msgs::msg::OccupancyGrid message = gridMessage(Geometry{}, kBlocked);
  message.data[static_cast<std::size_t>(2U) * kColumns + 3U] =
    value_case.value;

  const std::shared_ptr<const OccupancyGridMap> map = mapOf(message);
  ASSERT_NE(map, nullptr);

  const FreeSpace free_space = cbgl::freeSpaceOf(*map);

  EXPECT_EQ(free_space.cells.size(), value_case.free ? 1u : 0u)
    << "one cell of an otherwise blocked grid carries "
    << static_cast<int>(value_case.value) << " at an occupancy threshold of "
    << kOccupancyThreshold << ", and the map calls such a cell "
    << (value_case.free ? "free" : "blocked") << ": a cell is blocked at or "
    << "above the threshold, and a negative value is unknown and therefore "
    << "blocked as well. The free space offered "
    << free_space.cells.size() << " cells. The threshold is the map's rule and "
    << "not this function's, so a disagreement here is a second copy of it.";
}

INSTANTIATE_TEST_SUITE_P(
  AcrossTheOccupancyThreshold, FreeSpaceOfCellValue,
  testing::Values(
    ValueCase{.value = kFree, .free = true},
    ValueCase{.value = static_cast<std::int8_t>(kOccupancyThreshold - 1),
      .free = true},
    ValueCase{.value = static_cast<std::int8_t>(kOccupancyThreshold),
      .free = false},
    ValueCase{.value = 100, .free = false},
    ValueCase{.value = kUnknown, .free = false}),
  valueCaseSuffix);
