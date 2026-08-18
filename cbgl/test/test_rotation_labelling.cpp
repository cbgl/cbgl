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
 * @brief `cbgl::candidatesOf`: that the ranges a rotated cast hands back are
 * the ranges a cast taken at the orientation the candidate is labelled with
 * would hand back, for every candidate, on three grids.
 * @details
 * This is the file the whole saving of this algorithm rests on, and the file
 * that would have caught the labelling error the ROS 1 build shipped with.
 * **What the saving is.** A cast over a full turn taken at one orientation is
 * a cast at any other orientation of the same position, read from a different
 * starting sample. `da` candidate orientations of a position therefore cost one
 * cast between them instead of `da` of them. What the saving depends on is that
 * the orientation each candidate is labelled with is the orientation its ranges
 * were actually taken at, because the label is what the refinement starts from
 * and the ranges are what the candidate was chosen by. A label that disagrees
 * with its own ranges hands the refinement a pose that does not match the scan
 * it was picked for.
 * **Where the sign comes from.** Ray `i` of `n` leaves a pose at
 * ```
 * t_ray = i * 2*M_PI / n + pose.t - M_PI
 * ```
 * which is `FSM::GridReference::ranges`'s own contract and what
 * `fsm_localisation::OccupancyGridMap::ranges` computes. Substituting a pose
 * whose orientation is `pose.t + s * 2*M_PI / n` gives
 * `t_ray = (i + s) * 2*M_PI / n + pose.t - M_PI`, which is ray `i + s` of the
 * original cast. So reading the original cast from sample `s` onwards is the
 * cast at an orientation *greater* by `s * 2*M_PI / n`, and the label carries a
 * plus sign. The sign is derived here from that expression rather than by
 * analogy with the ROS 1 code, whose caster walks the angles the other way
 * because it read the grid's rows in reverse and compensated with a parameter
 * this package no longer carries.
 * **What went wrong before, which is what the failure messages below say.**
 * The ROS 1 build shifted the range vector by `round(n / da) * k` samples and
 * labelled the result `-2 * pi * k / da`. At `n = 360` and `da = 32` the shift
 * is `round(11.25) = 11` samples, so 11 degrees a step, while the label moves
 * in steps of 11.25 degrees. The two part company by a quarter of a degree a
 * step and by 7.75 degrees at the last of the 32 candidates, and the label
 * moves in the opposite direction to the ranges besides. The refinement then
 * starts from a pose whose orientation disagrees with the scan it was chosen
 * for by up to that much on top of the whole sign error.
 * **The three grids.** Both resolutions the harness generates, so that a
 * disagreement expressed in cells is caught at either cell size, and one grid
 * whose origin is translated and rotated, because a recorded map has one and
 * because an implementation that quietly assumed a grid axis aligned with the
 * world would pass on the first two.
 * **The three orientation counts.** 8 divides 360, so every shift is exact and
 * the label carries no drift at all: that case fails on the sign alone. 32 is
 * what the package ships and what the drift above was measured at. 7 divides
 * 360 into no whole number of samples either, so a second, differently
 * shaped drift has to survive the same rule.
 * **The tolerances.** The two casts being compared are the same caster reading
 * the same grid at the same heading, reached through two orderings of one angle
 * expression, so they may differ only by the rounding of that expression. A
 * hundredth of a cell is generous for that and three orders of magnitude below
 * the whole cells a mislabelled candidate moves a range by. The label's own
 * tolerance is a thousand ulps of half a turn, which is what two orderings of
 * the same sum can differ by and thirteen orders of magnitude below the quarter
 * degree a step the ROS 1 formula drifts by.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "cbgl/cbgl.hpp"
#include "fsm_localisation/occupancy_grid_map.hpp"

namespace
{
using cbgl::Candidate;
using cbgl::Pose;
using fsm_localisation::CastingMethod;
using fsm_localisation::CellCoordinates;
using fsm_localisation::OccupancyGridMap;
using fsm_localisation::WorldPoint;

const std::int8_t kBlocked = 100;
const std::int8_t kFree = 0;

const int kOccupancyThreshold = 50;

const unsigned int kColumns = 64U;
const unsigned int kRows = 48U;

const unsigned int kPillarFirstColumn = 44U;
const unsigned int kPillarLastColumn = 47U;
const unsigned int kPillarFirstRow = 20U;
const unsigned int kPillarLastRow = 23U;

const unsigned int kSensorColumn = 20U;
const unsigned int kSensorRow = 30U;
const double kSensorHeading = 0.37;

const unsigned int kRayCount = 360U;

/**
 * @brief One grid to cast into: a cell size and an origin.
 */
struct GridCase
{
  const char* label{""};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
};

const GridCase kGrids[] = {
  GridCase{.label = "fine", .resolution = 0.05,
    .origin_x = 0.0, .origin_y = 0.0, .origin_yaw = 0.0},
  GridCase{.label = "coarse", .resolution = 0.20,
    .origin_x = 0.0, .origin_y = 0.0, .origin_yaw = 0.0},
  GridCase{.label = "rotated_origin", .resolution = 0.05,
    .origin_x = -3.2, .origin_y = 7.4, .origin_yaw = 0.6}};

class RotationLabelling
  : public testing::TestWithParam<std::tuple<GridCase, unsigned int>>
{
};

/**
 * @brief The difference between two orientations, wrapped onto the half open
 * turn so that a label just under pi and one just over minus pi are the small
 * difference apart they really are.
 */
double angleDifference(const double first, const double second)
{
  return fmod(first - second + 5*M_PI, 2*M_PI) - M_PI;
}

std::string gridCaseSuffix(
  const testing::TestParamInfo<std::tuple<GridCase, unsigned int>>& info)
{
  return std::format("{}_at_{}_orientations",
    std::get<0>(info.param).label, std::get<1>(info.param));
}

nav_msgs::msg::OccupancyGrid gridMessage(const GridCase& geometry)
{
  nav_msgs::msg::OccupancyGrid message;

  message.info.width = kColumns;
  message.info.height = kRows;
  message.info.resolution = static_cast<float>(geometry.resolution);
  message.info.origin.position.x = geometry.origin_x;
  message.info.origin.position.y = geometry.origin_y;
  message.info.origin.orientation.z = std::sin(geometry.origin_yaw / 2.0);
  message.info.origin.orientation.w = std::cos(geometry.origin_yaw / 2.0);

  message.data.assign(
    static_cast<std::size_t>(kColumns) * kRows, kFree);

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

std::shared_ptr<const OccupancyGridMap> mapOf(const GridCase& geometry)
{
  const auto built = OccupancyGridMap::build(
    gridMessage(geometry), kOccupancyThreshold, CastingMethod::ray_marching);

  return built.has_value() ? built.value() : nullptr;
}

/**
 * @brief Twice the grid's own diagonal, so that no ray inside the room can
 * report having met nothing.
 */
double rangeMaxOf(const GridCase& geometry)
{
  return 2.0 * std::hypot(
    kColumns * geometry.resolution, kRows * geometry.resolution);
}

/**
 * @brief The pose the one panoramic cast is taken from: the centre of a free
 * cell, reached through the map's own cell arithmetic so that the same cell is
 * meant on every grid however its origin is placed.
 */
Pose sensorPose(const OccupancyGridMap& map)
{
  const WorldPoint point = map.worldFromCell(CellCoordinates{
    .column = kSensorColumn + 0.5, .row = kSensorRow + 0.5});

  return Pose{.x = point.x, .y = point.y, .t = kSensorHeading};
}

/**
 * @brief The shift candidate `k` is read from, in samples, which is the whole
 * of what a candidate is.
 */
long shiftOfCandidate(const unsigned int k, const unsigned int num_rays,
  const unsigned int da)
{
  return std::lround(static_cast<double>(k) * num_rays / da);
}

}

TEST_P(RotationLabelling, TheCountOfCandidatesIsTheOrientationCountAsked)
{
  const GridCase geometry = std::get<0>(GetParam());
  const unsigned int da = std::get<1>(GetParam());

  const std::shared_ptr<const OccupancyGridMap> map = mapOf(geometry);
  ASSERT_NE(map, nullptr);

  const Pose pose = sensorPose(*map);
  ASSERT_TRUE(map->isFree(pose.x, pose.y));

  const std::vector<double> cast =
    map->ranges(pose, kRayCount, rangeMaxOf(geometry));

  const std::vector<Candidate> candidates = cbgl::candidatesOf(pose, cast, da);

  ASSERT_EQ(candidates.size(), da)
    << "one cast is asked to yield " << da << " candidate orientations and "
    << "yielded " << candidates.size() << ". The count the sift scores, and "
    << "the count the hypothesis total is computed from, is this one.";

  for (unsigned int k = 0U; k < da; k++)
    EXPECT_EQ(candidates[k].ranges.size(), cast.size())
      << "candidate " << k << " carries " << candidates[k].ranges.size()
      << " ranges where the cast it was read from carries " << cast.size()
      << ". A candidate shorter than the measurement is the crop the ROS 1 "
      << "build performed in unsigned arithmetic, which wrapped and read past "
      << "the end of the vector whenever the cast was the shorter of the two.";
}

TEST_P(RotationLabelling, TheOrientationOfCandidateKIsTheShiftThatWasActuallyApplied)
{
  const GridCase geometry = std::get<0>(GetParam());
  const unsigned int da = std::get<1>(GetParam());

  const double tolerance =
    1024.0 * std::numeric_limits<double>::epsilon() * M_PI;

  const std::shared_ptr<const OccupancyGridMap> map = mapOf(geometry);
  ASSERT_NE(map, nullptr);

  const Pose pose = sensorPose(*map);

  const std::vector<double> cast =
    map->ranges(pose, kRayCount, rangeMaxOf(geometry));

  const std::vector<Candidate> candidates = cbgl::candidatesOf(pose, cast, da);
  ASSERT_EQ(candidates.size(), da);

  for (unsigned int k = 0U; k < da; k++)
  {
    const long shift = shiftOfCandidate(k, kRayCount, da);
    const double expected = pose.t + 2*M_PI * shift / kRayCount;
    const double drift = angleDifference(candidates[k].pose.t, expected);

    EXPECT_LE(std::abs(drift), tolerance)
      << "candidate " << k << " of " << da << " is read from sample " << shift
      << " of " << kRayCount << ", which is a rotation of "
      << 360.0 * shift / kRayCount << " degrees, so its orientation must be "
      << expected << " radians. It is " << candidates[k].pose.t
      << " radians, which is " << 180.0 * drift / M_PI << " degrees away.\n"
      << "The plus sign is not a convention to choose: ray i of a cast leaves "
      << "at i * 2pi / n + t - pi, so reading the cast from sample s onwards "
      << "is the cast at an orientation greater by s * 2pi / n.\n"
      << "The ROS 1 build labelled this candidate -2 * pi * k / da while "
      << "shifting by round(n / da) * k samples. At n = 360 and da = 32 that "
      << "is a shift of 11 samples a step against a label of 11.25 degrees a "
      << "step: a quarter of a degree a step, 7.75 degrees by the last "
      << "candidate, and in the opposite direction throughout. The refinement "
      << "then starts from a pose whose orientation disagrees with the scan it "
      << "was chosen for.";
  }
}

TEST_P(RotationLabelling, ThePositionOfEveryCandidateIsThePositionTheCastWasTakenFrom)
{
  const GridCase geometry = std::get<0>(GetParam());
  const unsigned int da = std::get<1>(GetParam());

  const std::shared_ptr<const OccupancyGridMap> map = mapOf(geometry);
  ASSERT_NE(map, nullptr);

  const Pose pose = sensorPose(*map);

  const std::vector<double> cast =
    map->ranges(pose, kRayCount, rangeMaxOf(geometry));

  const std::vector<Candidate> candidates = cbgl::candidatesOf(pose, cast, da);
  ASSERT_EQ(candidates.size(), da);

  for (unsigned int k = 0U; k < da; k++)
  {
    EXPECT_EQ(candidates[k].pose.x, pose.x)
      << "candidate " << k << " stands at x " << candidates[k].pose.x
      << " where the cast was taken at x " << pose.x << ". Rotating a range "
      << "vector turns the sensor on the spot and moves it nowhere.";

    EXPECT_EQ(candidates[k].pose.y, pose.y)
      << "candidate " << k << " stands at y " << candidates[k].pose.y
      << " where the cast was taken at y " << pose.y << ".";
  }
}

TEST_P(RotationLabelling, TheRangesOfCandidateKAreAFreshCastAtTheOrientationItIsLabelledWith)
{
  const GridCase geometry = std::get<0>(GetParam());
  const unsigned int da = std::get<1>(GetParam());

  const double range_max = rangeMaxOf(geometry);
  const double tolerance = geometry.resolution / 100.0;

  const std::shared_ptr<const OccupancyGridMap> map = mapOf(geometry);
  ASSERT_NE(map, nullptr);

  const Pose pose = sensorPose(*map);

  const std::vector<double> cast = map->ranges(pose, kRayCount, range_max);

  const std::vector<Candidate> candidates = cbgl::candidatesOf(pose, cast, da);
  ASSERT_EQ(candidates.size(), da);

  for (unsigned int k = 0U; k < da; k++)
  {
    const Candidate& candidate = candidates[k];
    ASSERT_EQ(candidate.ranges.size(), kRayCount);

    const std::vector<double> fresh =
      map->ranges(candidate.pose, kRayCount, range_max);

    double worst = 0.0;
    unsigned int worst_ray = 0U;
    unsigned int disagreeing = 0U;

    for (unsigned int i = 0U; i < kRayCount; i++)
    {
      const double difference = std::abs(candidate.ranges[i] - fresh[i]);

      if (difference > worst)
      {
        worst = difference;
        worst_ray = i;
      }

      if (difference > tolerance)
        disagreeing++;
    }

    EXPECT_LE(worst, tolerance)
      << "candidate " << k << " of " << da << " on the " << geometry.label
      << " grid carries the ranges of a cast at " << candidate.pose.t
      << " radians, but a cast actually taken at that orientation disagrees "
      << "with them on " << disagreeing << " of " << kRayCount << " rays, "
      << "worst at ray " << worst_ray << " by " << worst << " metres, which is "
      << worst / geometry.resolution << " cells of this grid.\n"
      << "A candidate is one cast read from sample "
      << shiftOfCandidate(k, kRayCount, da) << " onwards, and the orientation "
      << "it is labelled with has to be the orientation that reading amounts "
      << "to: the label is what the refinement starts from and the ranges are "
      << "what the candidate was chosen by.\n"
      << "The ROS 1 build got this wrong in both directions at once. It "
      << "shifted by round(n / da) * k samples and labelled the result "
      << "-2 * pi * k / da. At n = 360 and da = 32 the shift is 11 samples, "
      << "so 11 degrees a step, while the label moves 11.25 degrees a step: "
      << "the label drifts a quarter of a degree a step and stands 7.75 "
      << "degrees from the truth by the last of the 32 candidates, and it "
      << "moves in the opposite direction to the ranges throughout.";
  }
}

INSTANTIATE_TEST_SUITE_P(
  OverGridsAndOrientationCounts, RotationLabelling,
  testing::Combine(testing::ValuesIn(kGrids), testing::Values(8U, 32U, 7U)),
  gridCaseSuffix);
