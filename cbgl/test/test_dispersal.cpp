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
 * @brief `cbgl::Localiser::disperse`: how many hypothesis poses a map's free
 * space is worth, where they are allowed to land, and what makes a draw
 * repeatable.
 * @details
 * **How many.** `round(dl * free_area)` positions, where the free area is the
 * number of free cells times the square of the resolution. Both halves of
 * `FreeSpace` are needed for that product and neither is recoverable from the
 * other, which is why the count is asserted against the product rather than
 * against the cell count.
 * At least one position is drawn wherever the map offers a free cell, however
 * small `dl` and the free area make their product. A map of a single cell that
 * drew nothing would refuse a run for want of hypotheses on a map that plainly
 * has somewhere to stand.
 * **Where.** A position is the centre of a free cell chosen uniformly at
 * random, which is the point `FreeSpace` already carries. Every drawn position
 * is therefore on free space by construction rather than by rejection, and a
 * map whose free space is a small part of its extent costs no more to sample
 * than a full one. `EveryDrawLandsOnTheCentreOfAFreeCell` compares the drawn
 * position against that point exactly and allows no tolerance: a position
 * recomputed from an index rather than read from the free space is a second
 * copy of the same arithmetic, and it is the copy that drifts.
 * **Repeatability.** A pinned `rng_seed` repeats a draw exactly, a different
 * seed draws differently, and a seed of zero draws from hardware entropy. That
 * is what lets one run be re-run, and what lets two builds be handed the same
 * hypothesis set.
 * **Why no map stands behind these cases.** Dispersal reads the free space and
 * never casts, so the map here answers every cast with the maximum range and
 * every position as free. A case that needed a real room would be measuring
 * the sift.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <format>
#include <memory>
#include <string>
#include <vector>

#include "cbgl/cbgl.hpp"

namespace
{
using cbgl::FreeSpace;
using cbgl::Localiser;
using cbgl::Parameters;
using cbgl::Point;
using cbgl::Pose;

/**
 * @brief The count of positions a density and a free space are worth, and the
 * free space itself.
 */
struct CountCase
{
  double dl{15.0};
  unsigned int columns{1U};
  unsigned int rows{1U};
  double resolution{0.05};
};

/**
 * @brief A map that answers what dispersal asks of it, which is nothing.
 * @details Dispersal reads the free space it was constructed with and casts no
 * ray, so a map that reports every position free and every ray at the maximum
 * range is enough to construct a localiser around. A map that did more would
 * invite a reader to think a case here depended on it.
 */
class UncastMap : public FSM::GridReference
{
public:
  bool isFree(const double, const double) const noexcept override
  {
    return true;
  }

  std::vector<double> ranges(const FSM::Pose&, const unsigned int num_rays,
    const double max_range) const override
  {
    return std::vector<double>(num_rays, max_range);
  }
};

class DispersalCount : public testing::TestWithParam<CountCase>
{
};

std::string countCaseSuffix(const testing::TestParamInfo<CountCase>& info)
{
  return std::format("dl{}_over_{}_cells_of_{}_metres",
    static_cast<unsigned int>(1000.0 * info.param.dl),
    info.param.columns * info.param.rows,
    static_cast<unsigned int>(1000.0 * info.param.resolution));
}

/**
 * @brief The centres of a rectangle of cells whose first cell's corner is the
 * origin, which is where a map with no origin of its own would put them.
 */
FreeSpace freeSpaceOfCells(const unsigned int columns, const unsigned int rows,
  const double resolution)
{
  FreeSpace free_space;
  free_space.resolution = resolution;

  for (unsigned int column = 0U; column < columns; column++)
    for (unsigned int row = 0U; row < rows; row++)
      free_space.cells.push_back(Point{
        .x = (column + 0.5) * resolution,
        .y = (row + 0.5) * resolution});

  return free_space;
}

std::size_t indexOfCell(const Point& point, const FreeSpace& free_space)
{
  for (std::size_t i = 0; i < free_space.cells.size(); i++)
    if (free_space.cells[i].x == point.x && free_space.cells[i].y == point.y)
      return i;

  return free_space.cells.size();
}

std::shared_ptr<const FSM::GridReference> mapStandingIn()
{
  return std::make_shared<const UncastMap>();
}

Parameters parametersOf(const double dl, const unsigned int rng_seed)
{
  Parameters parameters;
  parameters.dl = dl;
  parameters.rng_seed = rng_seed;
  return parameters;
}

bool sameDraw(const std::vector<Pose>& first, const std::vector<Pose>& second)
{
  if (first.size() != second.size())
    return false;

  for (std::size_t i = 0; i < first.size(); i++)
    if (first[i].x != second[i].x || first[i].y != second[i].y ||
        first[i].t != second[i].t)
      return false;

  return true;
}

}

TEST(Dispersal, ADifferentSeedDrawsADifferentSet)
{
  const FreeSpace free_space = freeSpaceOfCells(4U, 4U, 0.5);

  const Localiser one(parametersOf(15.0, 1U), mapStandingIn(), free_space);
  const Localiser two(parametersOf(15.0, 2U), mapStandingIn(), free_space);

  const std::vector<Pose> from_one = one.disperse();
  const std::vector<Pose> from_two = two.disperse();

  ASSERT_EQ(from_one.size(), from_two.size())
    << "the count comes from the density and the free area, which are the "
    << "same here, so only the draw itself may differ.";
  ASSERT_GT(from_one.size(), 1u);

  EXPECT_FALSE(sameDraw(from_one, from_two))
    << "seed 1 and seed 2 drew the same " << from_one.size() << " poses to the "
    << "last bit. Either the seed is not reaching the generator or the "
    << "generator is not being used, and in both cases pinning a seed would "
    << "give a false sense of having pinned anything.";
}

TEST(Dispersal, AtLeastOnePositionIsDrawnWhereverAFreeCellExists)
{
  const double resolution = 0.05;
  const unsigned int cells = 4U;
  const double dl = 0.01;

  const FreeSpace free_space = freeSpaceOfCells(cells, 1U, resolution);

  const double free_area = cells * resolution * resolution;
  const long rounded = std::lround(dl * free_area);

  ASSERT_EQ(rounded, 0)
    << "this case exists because the product of the density and the free area "
    << "rounds to nothing. " << dl << " times " << free_area << " is "
    << dl * free_area << ", and if that ever rounds to one the case is no "
    << "longer testing what it was written for.";

  const Localiser localiser(parametersOf(dl, 7U), mapStandingIn(), free_space);

  EXPECT_EQ(localiser.disperse().size(), 1u)
    << "the map offers " << cells << " free cells, so there is somewhere to "
    << "stand and the search must be given one place to start from. Drawing "
    << "nothing would refuse a run on a map that plainly has free space.";
}

TEST(Dispersal, EveryDrawLandsOnTheCentreOfAFreeCell)
{
  const FreeSpace free_space = freeSpaceOfCells(5U, 4U, 0.2);

  const Localiser localiser(parametersOf(50.0, 11U), mapStandingIn(), free_space);

  const std::vector<Pose> drawn = localiser.disperse();

  ASSERT_FALSE(drawn.empty());

  for (std::size_t i = 0; i < drawn.size(); i++)
  {
    EXPECT_LT(indexOfCell(Point{.x = drawn[i].x, .y = drawn[i].y}, free_space),
      free_space.cells.size())
      << "pose " << i << " was drawn at (" << drawn[i].x << ", " << drawn[i].y
      << "), which is not one of the " << free_space.cells.size()
      << " points the free space carries. Every position must be one of those "
      << "points exactly, since that is what makes a drawn pose free by "
      << "construction rather than by a check. A position within a rounding of "
      << "one of them was recomputed rather than read, and a recomputation is "
      << "a second copy of the map's own cell arithmetic.";
  }
}

TEST(Dispersal, EveryFreeCellIsDrawnAtLeastOnceOverManyDraws)
{
  const double resolution = 1.0;
  const unsigned int columns = 2U;
  const unsigned int rows = 2U;
  const double dl = 50.0;

  const FreeSpace free_space = freeSpaceOfCells(columns, rows, resolution);

  const Localiser localiser(parametersOf(dl, 3U), mapStandingIn(), free_space);

  const std::vector<Pose> drawn = localiser.disperse();

  const std::size_t cells = free_space.cells.size();
  const double expected_draws = dl * cells * resolution * resolution;

  ASSERT_EQ(drawn.size(), static_cast<std::size_t>(expected_draws));

  std::vector<unsigned int> times_drawn(cells, 0U);
  for (const Pose& pose : drawn)
  {
    const std::size_t cell =
      indexOfCell(Point{.x = pose.x, .y = pose.y}, free_space);
    ASSERT_LT(cell, cells);
    times_drawn[cell]++;
  }

  for (std::size_t cell = 0; cell < cells; cell++)
  {
    EXPECT_GT(times_drawn[cell], 0u)
      << "cell " << cell << " of " << cells << " was never drawn in "
      << drawn.size() << " draws. A uniform choice among " << cells
      << " cells misses one of them with probability " << cells << " times "
      << "(1 - 1/" << cells << ")^" << drawn.size() << ", which is far below "
      << "anything that happens. A cell that is never drawn is a generator "
      << "that does not reach the whole free space, and the part it cannot "
      << "reach is a part of the map this algorithm can never localise in.";
  }
}

TEST(Dispersal, EveryOrientationLiesOnTheHalfOpenTurn)
{
  const FreeSpace free_space = freeSpaceOfCells(4U, 4U, 0.5);

  const Localiser localiser(parametersOf(20.0, 5U), mapStandingIn(), free_space);

  const std::vector<Pose> drawn = localiser.disperse();

  ASSERT_GT(drawn.size(), 10u);

  for (std::size_t i = 0; i < drawn.size(); i++)
  {
    EXPECT_GE(drawn[i].t, -M_PI)
      << "pose " << i << " carries an orientation of " << drawn[i].t
      << ", below -pi. The orientation is drawn uniformly on [-pi, pi), which "
      << "is the interval every other angle in this package is wrapped into.";

    EXPECT_LT(drawn[i].t, M_PI)
      << "pose " << i << " carries an orientation of " << drawn[i].t
      << ", at or above pi. Half of the turn would then be drawn twice over "
      << "and the interval would not be half open.";
  }
}

TEST(Dispersal, NoPositionIsDrawnFromAMapWithNoFreeCell)
{
  FreeSpace free_space;
  free_space.resolution = 0.05;

  const Localiser localiser(parametersOf(15.0, 1U), mapStandingIn(), free_space);

  EXPECT_TRUE(localiser.disperse().empty())
    << "a map with no free cell offers nowhere to stand, so there is no "
    << "hypothesis to draw. Drawing one anyway would put it somewhere the "
    << "free space never named.";
}

TEST(Dispersal, OneFreeCellIsDrawnAtThatCellAndNowhereElse)
{
  const double resolution = 0.2;

  FreeSpace free_space;
  free_space.resolution = resolution;
  free_space.cells.push_back(Point{.x = 3.7, .y = -2.1});

  const Localiser localiser(parametersOf(500.0, 9U), mapStandingIn(), free_space);

  const std::vector<Pose> drawn = localiser.disperse();

  ASSERT_EQ(drawn.size(), static_cast<std::size_t>(std::lround(
      500.0 * 1 * resolution * resolution)))
    << "one cell of " << resolution << " metres is "
    << resolution * resolution << " square metres of free space, and at a "
    << "density of 500 that is " << 500.0 * resolution * resolution
    << " positions.";

  for (std::size_t i = 0; i < drawn.size(); i++)
  {
    EXPECT_EQ(drawn[i].x, free_space.cells[0].x)
      << "pose " << i << " stands at x " << drawn[i].x << " on a map whose "
      << "only free cell is at x " << free_space.cells[0].x << ".";
    EXPECT_EQ(drawn[i].y, free_space.cells[0].y)
      << "pose " << i << " stands at y " << drawn[i].y << " on a map whose "
      << "only free cell is at y " << free_space.cells[0].y << ".";
  }
}

TEST(Dispersal, TheDrawIsRepeatedExactlyByASecondLocaliserWithTheSameSeed)
{
  const FreeSpace free_space = freeSpaceOfCells(4U, 4U, 0.5);

  const Localiser one(parametersOf(15.0, 20260818U), mapStandingIn(), free_space);
  const Localiser two(parametersOf(15.0, 20260818U), mapStandingIn(), free_space);

  const std::vector<Pose> from_one = one.disperse();
  const std::vector<Pose> from_two = two.disperse();

  ASSERT_FALSE(from_one.empty());

  EXPECT_TRUE(sameDraw(from_one, from_two))
    << "two localisers given the same pinned seed must draw the same set to "
    << "the last bit, or a run cannot be repeated and two builds cannot be "
    << "handed the same hypothesis set to be compared over.";
}

TEST(Dispersal, TheSameLocaliserDrawsTheSameSetTwice)
{
  const FreeSpace free_space = freeSpaceOfCells(4U, 4U, 0.5);

  const Localiser localiser(
    parametersOf(15.0, 4242U), mapStandingIn(), free_space);

  const std::vector<Pose> first = localiser.disperse();
  const std::vector<Pose> second = localiser.disperse();

  ASSERT_FALSE(first.empty());

  EXPECT_TRUE(sameDraw(first, second))
    << "nothing survives a call, so a second draw from the same localiser "
    << "must repeat the first. A generator carried across calls would make the "
    << "second localisation of a run depend on the first, and a pinned seed "
    << "would then pin only the first of them.";
}

TEST(Dispersal, TwoLocalisersDrawingFromHardwareEntropyDisagree)
{
  const FreeSpace free_space = freeSpaceOfCells(4U, 4U, 0.5);

  const Localiser one(parametersOf(20.0, 0U), mapStandingIn(), free_space);
  const Localiser two(parametersOf(20.0, 0U), mapStandingIn(), free_space);

  const std::vector<Pose> from_one = one.disperse();
  const std::vector<Pose> from_two = two.disperse();

  ASSERT_GT(from_one.size(), 10u);

  EXPECT_FALSE(sameDraw(from_one, from_two))
    << "a seed of zero draws from hardware entropy, so two localisers must "
    << "disagree over " << from_one.size() << " poses. Agreeing means zero was "
    << "taken as a seed of its own, and every run of a node that never set a "
    << "seed would then search from the same places.";
}

TEST_P(DispersalCount, TheNumberOfPositionsIsTheDensityTimesTheFreeArea)
{
  const CountCase parameters = GetParam();

  const FreeSpace free_space = freeSpaceOfCells(
    parameters.columns, parameters.rows, parameters.resolution);

  const std::size_t cells = free_space.cells.size();
  const double free_area =
    cells * parameters.resolution * parameters.resolution;
  const long expected = std::lround(parameters.dl * free_area);

  const Localiser localiser(
    parametersOf(parameters.dl, 13U), mapStandingIn(), free_space);

  EXPECT_EQ(localiser.disperse().size(), static_cast<std::size_t>(expected))
    << "the free space carries " << cells << " cells of "
    << parameters.resolution << " metres, which is " << free_area
    << " square metres, and at a density of " << parameters.dl
    << " positions the square metre that is " << parameters.dl * free_area
    << " positions, or " << expected << " of them once rounded. A count taken "
    << "from the cell count rather than from the area would give "
    << std::lround(parameters.dl * cells) << " instead, which is the same "
    << "number only on a map of one metre cells.";
}

INSTANTIATE_TEST_SUITE_P(
  OverDensitiesAndResolutions, DispersalCount,
  testing::Values(
    CountCase{.dl = 15.0, .columns = 20U, .rows = 10U, .resolution = 0.1},
    CountCase{.dl = 2.5, .columns = 20U, .rows = 10U, .resolution = 0.2},
    CountCase{.dl = 15.0, .columns = 7U, .rows = 7U, .resolution = 0.05},
    CountCase{.dl = 0.75, .columns = 12U, .rows = 8U, .resolution = 0.5},
    CountCase{.dl = 40.0, .columns = 3U, .rows = 3U, .resolution = 0.2}),
  countCaseSuffix);
