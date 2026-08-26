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
 * @brief The sift: which candidates survive a search, in what order, and what
 * the answer must not depend on.
 * @details
 * The sift scores every candidate of every hypothesis by the metric and keeps
 * the cheapest `top_k_caers` of them. What is asserted here is the ranking
 * itself, read off `Result::survivors`, which carries the survivors in ranking
 * order with the score each was chosen by.
 * **How the expected ranking is built.** Every candidate of every hypothesis is
 * enumerated in the case itself, through `candidatesOf` and `caer`, and sorted
 * by score. The sift is then held against that list. Whether a candidate's
 * ranges are the ranges of the pose it is labelled with is
 * `test_rotation_labelling.cpp`'s question and whether the metric adds up is
 * `test_caer.cpp`'s; what is left for this file is whether the sift ranks by
 * that metric over that candidate set, keeps the right number of them, and
 * loses none of them to a tie.
 * **Ties are ordinary, not a corner.** Dispersal draws a cell uniformly at
 * random and a small map has few cells, so the same cell is drawn twice on many
 * runs. Two identical hypotheses produce two identical candidate sets and
 * therefore exact ties throughout, which is the tie this file uses: it needs no
 * symmetric room to arrange and it is exact rather than nearly so.
 * **Where `top_k_caers` exceeds the candidate count** every candidate survives.
 * The ROS 1 build partially sorted to an index past the end of its own list and
 * then read that far into it, so a map with little free space or a high
 * `top_k_caers` read out of bounds.
 * **A candidate that met nothing** ranks behind every candidate that scored
 * anything at all, and therefore never displaces one from the survivors while
 * any remain.
 * **The thread count changes nothing.** The sift may run concurrently and the
 * ranking is computed from the whole scored set afterwards, so the same run at
 * one, two, four and as many threads as the hardware offers must give the same
 * survivors and the same answer, to the last bit.
 * **What the room is for.** A bordered room with a pillar off both of its axes
 * of symmetry, so no two of the poses below see the same thing and no ranking
 * here can be right by coincidence. The measurement is cast from the map
 * itself, which makes this file measure the sift rather than the rasterisation.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <vector>

#include "cbgl/cbgl.hpp"
#include "fsm_localisation/occupancy_grid_map.hpp"

namespace
{
using cbgl::Candidate;
using cbgl::Error;
using cbgl::FreeSpace;
using cbgl::Localiser;
using cbgl::Parameters;
using cbgl::Point;
using cbgl::Pose;
using cbgl::Result;
using cbgl::ScoredPose;
using fsm_localisation::CastingMethod;
using fsm_localisation::CellCoordinates;
using fsm_localisation::OccupancyGridMap;
using fsm_localisation::WorldPoint;

const std::int8_t kBlocked = 100;
const std::int8_t kFree = 0;

const int kOccupancyThreshold = 50;

const unsigned int kColumns = 64U;
const unsigned int kRows = 48U;
const double kResolution = 0.05;

const double kFullTurn = 2.0 * M_PI;

const unsigned int kPillarFirstColumn = 44U;
const unsigned int kPillarLastColumn = 47U;
const unsigned int kPillarFirstRow = 20U;
const unsigned int kPillarLastRow = 23U;

const unsigned int kRayCount = 360U;

const unsigned int kOrientations = 8U;
const unsigned int kTopK = 4U;

/**
 * @brief A candidate and what the metric made of it, as this file works the
 * expected ranking out for itself.
 */
struct Ranked
{
  Pose pose;
  double score{0.0};
};

/**
 * @brief The centre of a cell, in the world, reached through the map's own cell
 * arithmetic.
 */
Pose atCell(const OccupancyGridMap& map, const double column, const double row,
  const double heading)
{
  const WorldPoint point = map.worldFromCell(CellCoordinates{
    .column = column + 0.5, .row = row + 0.5});

  return Pose{.x = point.x, .y = point.y, .t = heading};
}

/**
 * @brief Every candidate of every hypothesis, scored, in ranking order. This is
 * the list the sift is held against.
 */
std::vector<Ranked> everyCandidateInRankingOrder(const OccupancyGridMap& map,
  const std::vector<Pose>& hypotheses, const std::vector<double>& measured,
  const unsigned int da, const double range_max)
{
  std::vector<Ranked> ranked;

  for (const Pose& hypothesis : hypotheses)
  {
    const std::vector<double> cast =
      map.ranges(hypothesis, kRayCount, range_max);

    for (const Candidate& candidate : cbgl::candidatesOf(hypothesis, cast, da))
      ranked.push_back(Ranked{
        .pose = candidate.pose,
        .score = cbgl::caer(measured, candidate.ranges, range_max)});
  }

  std::stable_sort(ranked.begin(), ranked.end(),
    [](const Ranked& first, const Ranked& second)
    { return first.score < second.score; });

  return ranked;
}

FreeSpace freeSpaceOfTheRoom(const OccupancyGridMap& map)
{
  FreeSpace free_space;
  free_space.resolution = map.resolution();

  for (unsigned int column = 0U; column < map.width(); column++)
    for (unsigned int row = 0U; row < map.height(); row++)
    {
      const WorldPoint point = map.worldFromCell(CellCoordinates{
        .column = column + 0.5, .row = row + 0.5});

      if (map.isFree(point.x, point.y))
        free_space.cells.push_back(Point{.x = point.x, .y = point.y});
    }

  return free_space;
}

nav_msgs::msg::OccupancyGrid gridMessage()
{
  nav_msgs::msg::OccupancyGrid message;

  message.info.width = kColumns;
  message.info.height = kRows;
  message.info.resolution = static_cast<float>(kResolution);
  message.info.origin.orientation.w = 1.0;

  message.data.assign(static_cast<std::size_t>(kColumns) * kRows, kFree);

  const auto block =
    [&message](const unsigned int column, const unsigned int row)
    {
      message.data[static_cast<std::size_t>(row) * kColumns + column] = kBlocked;
    };

  for (unsigned int column = 0U; column < kColumns; column++)
  {
    block(column, 0U);
    block(column, kRows - 1U);
  }

  for (unsigned int row = 0U; row < kRows; row++)
  {
    block(0U, row);
    block(kColumns - 1U, row);
  }

  for (unsigned int column = kPillarFirstColumn;
       column <= kPillarLastColumn; column++)
    for (unsigned int row = kPillarFirstRow; row <= kPillarLastRow; row++)
      block(column, row);

  return message;
}

/**
 * @brief The hypotheses every case but one starts from: four poses spread over
 * the room, the first of them the pose the measurement was cast at, so that at
 * least one survivor is a pose the refinement is bound to accept.
 */
std::vector<Pose> hypothesesOf(const OccupancyGridMap& map)
{
  return std::vector<Pose>{
    atCell(map, 20.0, 30.0, 0.37),
    atCell(map, 10.0, 10.0, -1.2),
    atCell(map, 50.0, 38.0, 2.9),
    atCell(map, 30.0, 15.0, 0.9)};
}

std::shared_ptr<const OccupancyGridMap> mapOfTheRoom()
{
  const auto built = OccupancyGridMap::build(
    gridMessage(), kOccupancyThreshold, CastingMethod::ray_marching);

  return built.has_value() ? built.value() : nullptr;
}

Parameters parametersOf(const unsigned int top_k_caers,
  const unsigned int da, const unsigned int num_threads)
{
  Parameters parameters;
  parameters.da = da;
  parameters.top_k_caers = top_k_caers;
  parameters.num_threads = num_threads;
  parameters.rng_seed = 20260818U;

  parameters.matcher.size_scan = kRayCount;
  parameters.matcher.rng_seed = 20260818U;
  parameters.matcher.num_iterations = 3U;
  parameters.matcher.xy_bound = 1.2;
  parameters.matcher.t_bound = 0.786;
  parameters.matcher.enforce_early_gearup = true;
  parameters.matcher.enforce_translation_bound = true;
  parameters.matcher.recover_on_failed_sift = true;
  parameters.matcher.occupancy_threshold = kOccupancyThreshold;

  return parameters;
}

/**
 * @brief Twice the grid's own diagonal, so that no ray inside the room can
 * report having met nothing.
 */
double rangeMax()
{
  return 2.0 * std::hypot(kColumns * kResolution, kRows * kResolution);
}

bool someCandidateCarries(const std::vector<Ranked>& ranked,
  const ScoredPose& survivor)
{
  for (const Ranked& candidate : ranked)
    if (candidate.pose.x == survivor.pose.x &&
        candidate.pose.y == survivor.pose.y &&
        candidate.pose.t == survivor.pose.t &&
        candidate.score == survivor.score)
      return true;

  return false;
}

/**
 * @brief The measurement every case is scored against: the scan the map itself
 * would cast at the first hypothesis.
 */
std::vector<double> theMeasurement(const OccupancyGridMap& map)
{
  return map.ranges(hypothesesOf(map)[0], kRayCount, rangeMax());
}
}

TEST(Sift, ACandidateThatMetNothingIsRankedBehindEveryCandidateThatScored)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOfTheRoom();
  ASSERT_NE(map, nullptr);

  const std::vector<double> measured = theMeasurement(*map);

  std::vector<Pose> hypotheses = hypothesesOf(*map);
  const WorldPoint outside = map->worldFromCell(CellCoordinates{
    .column = -20.5, .row = -20.5});
  hypotheses.push_back(Pose{.x = outside.x, .y = outside.y, .t = 0.0});

  const std::vector<Ranked> ranked =
    everyCandidateInRankingOrder(*map, hypotheses, measured, kOrientations,
      rangeMax());

  unsigned int met_nothing = 0U;
  for (const Ranked& candidate : ranked)
    if (std::isinf(candidate.score))
      met_nothing++;

  ASSERT_EQ(met_nothing, kOrientations)
    << "the last hypothesis stands off the grid, so every one of its "
    << kOrientations << " candidates came back at the maximum range and "
    << "scores infinity. This case is worthless unless that is so.";

  const Localiser localiser(parametersOf(kTopK, kOrientations, 1U), map,
    freeSpaceOfTheRoom(*map));

  const std::expected<Result, Error> result =
    localiser.localise(measured, rangeMax(), hypotheses, kFullTurn);

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->survivors.size(), kTopK);

  for (std::size_t i = 0; i < result->survivors.size(); i++)
    EXPECT_FALSE(std::isinf(result->survivors[i].score))
      << "survivor " << i << " of " << result->survivors.size()
      << " scored infinity while " << ranked.size() - met_nothing
      << " candidates that met something were available. A candidate the "
      << "metric cannot grade must rank behind every candidate it can, or a "
      << "pose off the map is refined in place of a pose on it.";
}

TEST(Sift, EveryCandidateSurvivesWhereTopKAsksForMoreThanExist)
{
  const unsigned int da = 4U;
  const unsigned int top_k = 1000U;

  const std::shared_ptr<const OccupancyGridMap> map = mapOfTheRoom();
  ASSERT_NE(map, nullptr);

  const std::vector<double> measured = theMeasurement(*map);

  const std::vector<Pose> hypotheses{
    hypothesesOf(*map)[0], hypothesesOf(*map)[1]};

  const Localiser localiser(parametersOf(top_k, da, 1U), map,
    freeSpaceOfTheRoom(*map));

  const std::expected<Result, Error> result =
    localiser.localise(measured, rangeMax(), hypotheses, kFullTurn);

  ASSERT_TRUE(result.has_value());

  const std::size_t candidates = hypotheses.size() * da;

  EXPECT_EQ(result->num_candidates, candidates)
    << hypotheses.size() << " hypotheses at " << da << " orientations each is "
    << candidates << " candidates.";

  EXPECT_EQ(result->survivors.size(), candidates)
    << "there are " << candidates << " candidates and " << top_k
    << " were asked for, so every candidate survives and the count of "
    << "survivors is the count of candidates. The ROS 1 build sorted to an "
    << "index " << top_k << " into a list of " << candidates << " and then "
    << "read that far into it.";
}

TEST(Sift, TheRankingDoesNotDependOnTheNumberOfThreads)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOfTheRoom();
  ASSERT_NE(map, nullptr);

  const std::vector<double> measured = theMeasurement(*map);
  const std::vector<Pose> hypotheses = hypothesesOf(*map);

  const unsigned int thread_counts[] = {1U, 2U, 4U, 0U};

  std::vector<ScoredPose> reference;
  Pose reference_pose;

  for (const unsigned int threads : thread_counts)
  {
    const Localiser localiser(parametersOf(kTopK, kOrientations, threads), map,
      freeSpaceOfTheRoom(*map));

    const std::expected<Result, Error> result =
      localiser.localise(measured, rangeMax(), hypotheses, kFullTurn);

    ASSERT_TRUE(result.has_value())
      << "the search refused a run at " << threads << " threads that it "
      << "answered at another count.";

    if (reference.empty())
    {
      reference = result->survivors;
      reference_pose = result->pose;
      ASSERT_FALSE(reference.empty());
      continue;
    }

    ASSERT_EQ(result->survivors.size(), reference.size())
      << "at " << threads << " threads the sift kept "
      << result->survivors.size() << " survivors where one thread kept "
      << reference.size() << ".";

    for (std::size_t i = 0; i < reference.size(); i++)
    {
      EXPECT_EQ(result->survivors[i].score, reference[i].score)
        << "survivor " << i << " scores " << result->survivors[i].score
        << " at " << threads << " threads and " << reference[i].score
        << " at one. The score of a candidate cannot depend on which thread "
        << "computed it, so this is a shared piece of mutable state or a "
        << "reduction whose order the thread count decides.";

      EXPECT_EQ(result->survivors[i].pose.x, reference[i].pose.x)
        << "survivor " << i << " stands elsewhere at " << threads
        << " threads than at one. The ranking is computed from the whole "
        << "scored set after the scoring, so the thread count must not reach "
        << "it.";
      EXPECT_EQ(result->survivors[i].pose.y, reference[i].pose.y);
      EXPECT_EQ(result->survivors[i].pose.t, reference[i].pose.t);
    }

    EXPECT_EQ(result->pose.x, reference_pose.x)
      << "the answer moved from " << reference_pose.x << " to "
      << result->pose.x << " when the thread count changed to " << threads
      << ". A localisation that answers differently on a busier machine is not "
      << "reproducible and cannot be compared against another build.";
    EXPECT_EQ(result->pose.y, reference_pose.y);
    EXPECT_EQ(result->pose.t, reference_pose.t);
  }
}

TEST(Sift, TheSurvivorsAreTheCheapestCandidatesInRankingOrder)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOfTheRoom();
  ASSERT_NE(map, nullptr);

  const std::vector<double> measured = theMeasurement(*map);
  const std::vector<Pose> hypotheses = hypothesesOf(*map);

  const std::vector<Ranked> ranked =
    everyCandidateInRankingOrder(*map, hypotheses, measured, kOrientations,
      rangeMax());

  ASSERT_GE(ranked.size(), kTopK)
    << "this case holds the " << kTopK << " survivors against the " << kTopK
    << " cheapest of the candidates it enumerated for itself, and it "
    << "enumerated only " << ranked.size() << ". Reading further into that "
    << "list than it is long is the defect shape this port exists to remove.";

  const Localiser localiser(parametersOf(kTopK, kOrientations, 1U), map,
    freeSpaceOfTheRoom(*map));

  const std::expected<Result, Error> result =
    localiser.localise(measured, rangeMax(), hypotheses, kFullTurn);

  ASSERT_TRUE(result.has_value());

  EXPECT_EQ(result->num_candidates, hypotheses.size() * kOrientations)
    << hypotheses.size() << " hypotheses at " << kOrientations
    << " orientations each is " << hypotheses.size() * kOrientations
    << " candidates, and every one of them is scored.";

  ASSERT_EQ(result->survivors.size(), kTopK)
    << kTopK << " survivors were asked for out of " << ranked.size()
    << " candidates.";

  for (std::size_t i = 0; i < result->survivors.size(); i++)
  {
    EXPECT_DOUBLE_EQ(result->survivors[i].score, ranked[i].score)
      << "survivor " << i << " scores " << result->survivors[i].score
      << " where the " << i << "th cheapest of the " << ranked.size()
      << " candidates scores " << ranked[i].score << ". The survivors are the "
      << "cheapest candidates by the metric, in ranking order, and the score "
      << "each is reported with is the score it was chosen by.";

    EXPECT_TRUE(someCandidateCarries(ranked, result->survivors[i]))
      << "survivor " << i << " stands at (" << result->survivors[i].pose.x
      << ", " << result->survivors[i].pose.y << ", "
      << result->survivors[i].pose.t << ") with a score of "
      << result->survivors[i].score << ", and no candidate of any hypothesis "
      << "carries that pose at that score. A survivor is one of the "
      << "candidates, not a pose derived from one.";

    if (i > 0)
    {
      EXPECT_LE(result->survivors[i - 1].score, result->survivors[i].score)
        << "survivor " << i - 1 << " scores more than survivor " << i
        << ", so the survivors are not in ranking order. The refinement is "
        << "handed them in this order and a host publishes them in it.";
    }
  }
}

TEST(Sift, TiedCandidatesAreNeitherLostNorDuplicated)
{
  const std::shared_ptr<const OccupancyGridMap> map = mapOfTheRoom();
  ASSERT_NE(map, nullptr);

  const std::vector<double> measured = theMeasurement(*map);

  std::vector<Pose> hypotheses = hypothesesOf(*map);
  hypotheses.push_back(hypotheses[0]);

  const std::vector<Ranked> ranked =
    everyCandidateInRankingOrder(*map, hypotheses, measured, kOrientations,
      rangeMax());

  ASSERT_GE(ranked.size(), kTopK)
    << "the case compares the " << kTopK << " survivors and the two cheapest "
    << "candidates against a list of " << ranked.size() << " candidates it "
    << "enumerated for itself, which is shorter than what it reads.";

  ASSERT_EQ(ranked[0].score, ranked[1].score)
    << "the same hypothesis was handed in twice, so the two cheapest "
    << "candidates are the same candidate twice over and their scores are "
    << "equal to the last bit. Without that this case tests nothing.";

  const Localiser localiser(parametersOf(kTopK, kOrientations, 1U), map,
    freeSpaceOfTheRoom(*map));

  const std::expected<Result, Error> result =
    localiser.localise(measured, rangeMax(), hypotheses, kFullTurn);

  ASSERT_TRUE(result.has_value());

  EXPECT_EQ(result->num_candidates, hypotheses.size() * kOrientations)
    << "a hypothesis handed in twice is scored twice. Dispersal draws a cell "
    << "uniformly at random, so a small map yields the same cell twice on many "
    << "runs, and a sift that quietly dropped the repeat would report fewer "
    << "candidates than it was given.";

  ASSERT_EQ(result->survivors.size(), kTopK);

  EXPECT_EQ(result->survivors[0].score, result->survivors[1].score)
    << "the two cheapest survivors score " << result->survivors[0].score
    << " and " << result->survivors[1].score << " where the two cheapest "
    << "candidates are a tie. A tie broken by discarding one of the two loses "
    << "a candidate the refinement was entitled to.";

  for (std::size_t i = 0; i < result->survivors.size(); i++)
    EXPECT_DOUBLE_EQ(result->survivors[i].score, ranked[i].score)
      << "survivor " << i << " does not carry the " << i
      << "th cheapest score of the " << ranked.size() << " candidates, so the "
      << "tie disturbed the order beyond the tied pair itself.";
}
