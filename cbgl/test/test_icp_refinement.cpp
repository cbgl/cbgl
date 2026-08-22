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
 * @brief `cbgl::IcpRefiner`: one estimate walked onto one cast of the map, and
 * `cbgl::validate`, which says what settings that walk may be run with.
 * @details
 * **The map is a room rather than a grid.** A grid stops a ray at a cell, so
 * the best answer a five centimetre grid can support is five centimetres, and
 * a bound of a few centimetres cannot tell a correct composition from one that
 * is nearly correct. The rooms here are polygons, cast analytically, so the
 * only error left in an answer is the solver's own and a case may hold it to a
 * bound tight enough to mean something. `FSM::GridReference` is an interface of
 * two methods, which is what makes that possible without a grid anywhere in
 * sight, and it is the same interface a host hands an occupancy grid over.
 * **One cast, taken once, at the estimate.** That is the whole of what
 * separates this library from `fsm_localisation::Corrector`, which reads the
 * map afresh at every pose it tries, and it is the property a later change is
 * most likely to break while every recovery case still passes. So the map is
 * asked directly: a room that records what it was asked for reports exactly one
 * cast, from exactly the estimate, of exactly as many rays as the scan carries,
 * on a refinement the solver spent several cycles on. A refinement that cast
 * per cycle would be a different algorithm wearing this one's name and would
 * still recover a displaced pose, which is why recovery alone is not asked to
 * catch it.
 * **The answer is composed onto the estimate, turned first.** The solver
 * answers in the reference cast's frame, and the reference was cast at the
 * estimate, so the translation it reports has to be turned by the estimate's
 * own heading before it is added. Adding it unturned is the one arithmetic
 * mistake here that leaves every answer plausible and every answer wrong, and
 * it is invisible while the estimate's heading is near nought. One case
 * therefore refines from a heading of two radians and works out, from what came
 * back, where the unturned arithmetic would have put the answer, so that the
 * two numbers can be seen not to overlap.
 * **The room is placed twice.** Once with its own frame as the world's, once
 * moved and turned, at the offset and heading the rotated grid under
 * `test/fixtures` carries. Poses in and out are then in a frame the map's cells
 * are not aligned with, which is the ordinary case on a real site and the case
 * where a transform left out reads as a small constant error rather than as a
 * failure.
 * **Poses are sensor poses, in and out.** Nothing here is a robot pose and
 * nothing is wrapped between a sensor to base transform, because there is
 * nothing to wrap it between. The scan a case measures is cast from the room at
 * the sensor's true pose, which is what a perfect sensor standing there would
 * report.
 * **Both settings sets are run.** The struct's own defaults, which are what the
 * ROS 1 build falls back to when it is told nothing, and the set that build's
 * own settings file ships. They differ in almost every value, so a recovery
 * that held only for one of them would be a recovery that depended on a lucky
 * configuration rather than on the algorithm.
 * **A refusal is asserted by its reason.** There are six, and each is reached
 * by a different route: a scan of no rays, a scan of fewer rays than the solver
 * takes, a scan of more, a scan with no usable reading in it, a map that
 * answered a request for one length of cast with another, and a pair of scans
 * the solver itself will not match. Asserting merely that something failed
 * would pass whichever of the six came back, and the two that a host answers
 * by mending its own arithmetic would then read as the one it answers by
 * looking at its map.
 * **A reading of a hundred metres or more is not carried into a match as a
 * distance.** The solver refuses whole any scan that offers it one, so the
 * library marks such a reading unusable on the way in and matches what is left.
 * A room with a way out of it is cast here at a far bound sitting on that
 * ceiling and at one beyond it, so that the rays leaving through the gap come
 * back at the bound they were cast to. Without that treatment every refinement
 * of a lidar whose own far bound reaches that high is refused, and refused as a
 * failed match with nothing in it to read a cause out of.
 * **One case pins a limit rather than a virtue.** A candidate a quarter turn
 * out comes back marked good, still a quarter turn out, with its position
 * dragged off a pose that was exact. It is written down because a host cannot
 * read that off the answer and has to be told, and because the only thing that
 * separates such an answer from a real one is the correspondence error measured
 * against another candidate of the same scan.
 * @remark The shipped set switches the angle based rejection of
 * correspondences on, at a neighbourhood of twenty rays. That is the path the
 * solver's neighbour gathering is patched in this image, so these cases are
 * also the ones that would notice the patch going missing: without it the
 * standard library's own bounds check stops the process and no match completes
 * at all.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "cbgl/icp_refinement.hpp"

namespace
{
using cbgl::IcpError;
using cbgl::IcpParameters;
using cbgl::IcpRefiner;
using cbgl::Pose;

/**
 * @brief How many rays every scan and every cast here carries.
 * @details The length the algorithm's own settings file reduces a scan to.
 */
const unsigned int kRayCount = 360U;

/**
 * @brief The two bounds the scan is taken to have been reported with.
 * @details Every wall of every room here is nearer than the far bound and
 * further than the near one, so no reading of a good scan sits at either.
 */
const double kRangeMin = 0.05;
const double kRangeMax = 20.0;

/**
 * @brief The fewest and the most rays the solver will take a scan at.
 * @details Its own two bounds, which the library checks up front so that a host
 * is told which of them it crossed rather than being handed a failed match.
 */
const unsigned int kFewestRays = 10U;
const unsigned int kMostRays = 10000U;

/**
 * @brief The reading at which the solver stops treating a distance as one.
 * @details A scan carrying a reading of this or more, marked usable, is refused
 * whole rather than in part, so the library marks such a reading unusable
 * instead.
 */
const double kLargestUsableReading = 100.0;

/**
 * @brief The room, in its own frame: a rectangle with a rectangular pillar
 * standing in it.
 * @details The rectangle alone would fix a pose, being longer than it is deep,
 * but only just: two of its walls are parallel to the other two and a solver
 * that slid one pair along itself would be corrected only by the corners. The
 * pillar removes that, and it is also what a scan taken from two nearby poses
 * disagrees about, since each pose sees a little more of it than the other and
 * the disagreement is exactly the sort a refinement must not be thrown by.
 */
const double kRoomWidth = 4.0;
const double kRoomDepth = 3.0;
const double kPillarLeft = 2.40;
const double kPillarRight = 3.00;
const double kPillarBottom = 1.00;
const double kPillarTop = 1.60;

/**
 * @brief The length taken out of the room's far wall where the room is given a
 * way out of it.
 * @details Eight tenths of a metre out of a wall four metres long, which from
 * where the sensor stands subtends about twenty of the three hundred and sixty
 * rays. Enough of them for a scan to carry readings at its own far bound, and
 * few enough that what is left still fixes a pose on its own.
 */
const double kGapLeft = 1.60;
const double kGapRight = 2.40;

/**
 * @brief Where the sensor really stood, in the room's own frame, for the cases
 * that displace an estimate from it.
 */
const Pose kTruthInTheRoom{1.30, 0.90, 0.40};

/**
 * @brief Where the room is put, for the cases that ask the same question of a
 * map that is not the world.
 * @details The offset and heading the rotated grid under `test/fixtures`
 * carries, so that the placement a case is run at is one this package already
 * holds a map at rather than one invented here.
 */
const Pose kTurnedPlacement{-1.35, 2.70, 0.60};

/**
 * @brief How far an estimate is put from the truth before it is refined.
 * @details Twenty three centimetres and three degrees, both well inside the
 * bounds either settings set allows a correction to reach, and both far enough
 * out that an answer landing on the truth cannot be the estimate having been
 * handed back.
 */
const Pose kDisplacement{0.18, -0.14, 0.05};

/**
 * @brief The accuracy an answer is held to.
 * @details Five centimetres, which is the cell of the maps this algorithm is
 * run on and, as the library says of itself, the best an answer taken in such a
 * map can be. The rooms here have no cells at all, so an answer that cannot beat
 * what a five centimetre map would have supported has not refined anything.
 */
const double kSupportableMetres = 0.05;

/**
 * @brief One wall, as the two ends of a segment in the room's own frame.
 */
struct Segment
{
  double x1{0.0};
  double y1{0.0};
  double x2{0.0};
  double y2{0.0};
};

/**
 * @brief What one cast asked the map for.
 */
struct Cast
{
  Pose pose;
  unsigned int num_rays{0U};
  double max_range{0.0};
};

/**
 * @brief A room to recover a pose in, and the settings to recover it with.
 */
struct RecoveryCase
{
  const char* label{""};
  Pose placement;
  IcpParameters parameters;
};

/**
 * @brief An estimate whose position is right and whose heading is not, by
 * `degrees`, and the settings the refinement is run with.
 */
struct HeadingCase
{
  const char* label{""};
  double degrees{0.0};
  IcpParameters parameters;
};

/**
 * @brief A scan that carries no usable reading: the one value every ray of it
 * is filled with, and the near bound it is reported with.
 * @details The near bound is here because a reading of nought is answered by
 * two rules rather than one and only one of them can be reached at any given
 * bound.
 */
struct UnusableScan
{
  const char* label{""};
  double range{0.0};
  double range_min{kRangeMin};
};

/**
 * @brief A number of rays the solver will take a scan at, and which sits at one
 * end or the other of what it will take.
 */
struct AcceptedScanLength
{
  const char* label{""};
  unsigned int num_rays{0U};
};

/**
 * @brief A number of rays outside what the solver will take a scan at, and the
 * reason the library is required to name.
 */
struct RefusedScanLength
{
  const char* label{""};
  unsigned int num_rays{0U};
  IcpError reason{IcpError::matching_failed};
};

/**
 * @brief A number of ranges a map answers a request with, whatever number of
 * them it was asked for.
 */
struct WrongLengthCast
{
  const char* label{""};
  unsigned int num_ranges_answered{0U};
};

/**
 * @brief A far bound sitting at or beyond the reading the solver stops treating
 * as a distance, and the settings a scan carrying it is refined with.
 */
struct CeilingCase
{
  const char* label{""};
  double far_bound{0.0};
  IcpParameters parameters;
};

/**
 * @brief A setting put outside its permitted range, and the two things the
 * refusal has to name.
 */
struct Refusal
{
  const char* label{""};
  std::function<void(IcpParameters&)> apply;
  const char* setting{""};
  const char* value{""};
};

/**
 * @brief A setting put at a value that is permitted, however unusual it looks.
 */
struct Acceptance
{
  const char* label{""};
  std::function<void(IcpParameters&)> apply;
};

/**
 * @brief A map that answers every ray with nothing at all.
 * @details Nought on every ray is what a caster reports for a sensor standing
 * inside an obstacle, and a reading of nought is carried into a match as
 * unusable rather than as a distance. So the reference the solver is given here
 * has not one usable point in it while the measured scan is perfectly good,
 * which is the one way to reach the solver's own refusal without giving it
 * anything to complain about on the measured side.
 */
class MapAnsweringNothing : public FSM::GridReference
{
public:
  bool isFree(const double, const double) const noexcept override
  {
    return true;
  }

  std::vector<double> ranges(const Pose&, const unsigned int num_rays,
    const double) const override
  {
    return std::vector<double>(num_rays, 0.0);
  }
};

/**
 * @brief A map that answers every request with the same number of ranges,
 * whatever number of them it was asked for.
 * @details The caster's own interface promises no length. A map that answers a
 * request for `n` ranges with `m` of them puts the two scans at different
 * angular spacings, ray `i` of the cast sitting at `-pi + i * 2 * pi / m` while
 * ray `i` of the measurement sits at `-pi + i * 2 * pi / n`, and the two would
 * then be matched index against index while meaning different directions.
 * Every range it answers with is the same distance, half the far bound it was
 * cast to, since the length is refused before a value of it is read.
 */
class MapAnsweringTheWrongLength : public FSM::GridReference
{
public:
  explicit MapAnsweringTheWrongLength(const unsigned int num_ranges_answered)
  : num_ranges_answered_(num_ranges_answered)
  {
  }

  bool isFree(const double, const double) const noexcept override
  {
    return true;
  }

  std::vector<double> ranges(const Pose&, const unsigned int,
    const double max_range) const override
  {
    return std::vector<double>(num_ranges_answered_, max_range / 2.0);
  }

private:
  unsigned int num_ranges_answered_;
};

/**
 * @brief A rectangular room with a pillar in it, cast analytically, standing
 * wherever it is placed in the world.
 * @details A ray is intersected with each wall segment and stops at the nearest
 * one in front of it, which is exact to the arithmetic rather than to a cell.
 * Ray `i` is sent at `pose.t - pi + i * 2 * pi / n`, which is where
 * `FSM::Utils::findGrid` reads the ray caster's own ray `i` to be, so a scan
 * cast here and a scan cast from an occupancy grid mean the same thing by index.
 * The room is defined once, in its own frame, and `placement` is where that
 * frame sits in the world. Every pose crossing this interface is a world pose:
 * a cast converts it into the room's frame, casts there, and hands back ranges,
 * which are the same numbers in either frame.
 * A room may be given a way out of it, which takes a length out of the middle
 * of its far wall and leaves the two ends of that wall standing. A ray sent
 * through the gap meets nothing and comes back at the far bound it was cast to,
 * which is how a scan carrying readings at its own far bound is had here
 * without one being written in by hand.
 * @remark `isFree` answers for the room whole, gap or no gap. Nothing a
 * refinement does reads it, the map being consulted for ranges alone.
 */
class Room : public FSM::GridReference
{
public:
  explicit Room(const Pose& placement = Pose{},
    const bool with_a_way_out = false)
  : placement_(placement),
    with_a_way_out_(with_a_way_out)
  {
  }

  bool isFree(const double x, const double y) const noexcept override
  {
    const Pose here = roomFrom(Pose{x, y, 0.0});

    const bool inside = here.x > 0.0 && here.x < kRoomWidth
      && here.y > 0.0 && here.y < kRoomDepth;

    const bool in_the_pillar = here.x > kPillarLeft && here.x < kPillarRight
      && here.y > kPillarBottom && here.y < kPillarTop;

    return inside && !in_the_pillar;
  }

  std::vector<double> ranges(const Pose& pose, const unsigned int num_rays,
    const double max_range) const override
  {
    const Pose here = roomFrom(pose);
    const std::vector<Segment> walls = everyWall(with_a_way_out_);

    std::vector<double> cast(num_rays, max_range);

    for (unsigned int i = 0; i < num_rays; i++)
    {
      const double angle = here.t - M_PI + i * 2.0 * M_PI / num_rays;
      const double towards_x = std::cos(angle);
      const double towards_y = std::sin(angle);

      double nearest = max_range;

      for (const Segment& wall : walls)
      {
        const double along_x = wall.x2 - wall.x1;
        const double along_y = wall.y2 - wall.y1;

        const double denominator = towards_x * along_y - towards_y * along_x;

        if (denominator == 0.0)
          continue;

        const double to_x = wall.x1 - here.x;
        const double to_y = wall.y1 - here.y;

        const double range = (to_x * along_y - to_y * along_x) / denominator;
        const double along = (to_x * towards_y - to_y * towards_x) / denominator;

        if (range >= 0.0 && along >= 0.0 && along <= 1.0 && range < nearest)
          nearest = range;
      }

      cast[i] = nearest;
    }

    return cast;
  }

  /**
   * @brief A pose of the room's own frame, as a pose of the world's.
   */
  Pose worldFrom(const Pose& room_pose) const
  {
    return Pose{
      placement_.x + room_pose.x * std::cos(placement_.t)
        - room_pose.y * std::sin(placement_.t),
      placement_.y + room_pose.x * std::sin(placement_.t)
        + room_pose.y * std::cos(placement_.t),
      std::remainder(room_pose.t + placement_.t, 2.0 * M_PI)};
  }

private:
  static std::vector<Segment> everyWall(const bool with_a_way_out)
  {
    std::vector<Segment> walls{
      Segment{0.0, 0.0, kRoomWidth, 0.0},
      Segment{kRoomWidth, 0.0, kRoomWidth, kRoomDepth},
      Segment{kRoomWidth, kRoomDepth, 0.0, kRoomDepth},
      Segment{0.0, kRoomDepth, 0.0, 0.0},
      Segment{kPillarLeft, kPillarBottom, kPillarRight, kPillarBottom},
      Segment{kPillarRight, kPillarBottom, kPillarRight, kPillarTop},
      Segment{kPillarRight, kPillarTop, kPillarLeft, kPillarTop},
      Segment{kPillarLeft, kPillarTop, kPillarLeft, kPillarBottom}};

    if (with_a_way_out)
    {
      walls[2] = Segment{kRoomWidth, kRoomDepth, kGapRight, kRoomDepth};
      walls.push_back(Segment{kGapLeft, kRoomDepth, 0.0, kRoomDepth});
    }

    return walls;
  }

  Pose roomFrom(const Pose& world_pose) const
  {
    const double away_x = world_pose.x - placement_.x;
    const double away_y = world_pose.y - placement_.y;

    return Pose{
      away_x * std::cos(placement_.t) + away_y * std::sin(placement_.t),
      -away_x * std::sin(placement_.t) + away_y * std::cos(placement_.t),
      std::remainder(world_pose.t - placement_.t, 2.0 * M_PI)};
  }

  Pose placement_;
  bool with_a_way_out_;
};

/**
 * @brief A room that remembers every cast it was asked for.
 * @details It answers exactly as the room it stands in front of does, so a case
 * using it measures the same refinement as a case that does not, and it holds
 * what it was asked rather than what it answered, since the question is how many
 * times the map was consulted and from where.
 */
class RecordingRoom : public FSM::GridReference
{
public:
  explicit RecordingRoom(std::shared_ptr<const Room> room)
  : room_(std::move(room))
  {
  }

  const std::vector<Cast>& everyCast() const
  {
    return casts_;
  }

  bool isFree(const double x, const double y) const noexcept override
  {
    return room_->isFree(x, y);
  }

  std::vector<double> ranges(const Pose& pose, const unsigned int num_rays,
    const double max_range) const override
  {
    casts_.push_back(Cast{pose, num_rays, max_range});

    return room_->ranges(pose, num_rays, max_range);
  }

private:
  mutable std::vector<Cast> casts_;
  std::shared_ptr<const Room> room_;
};

class AcceptedLength : public testing::TestWithParam<AcceptedScanLength>
{
};

class AcceptedSettings : public testing::TestWithParam<Acceptance>
{
};

class CeilingReadings : public testing::TestWithParam<CeilingCase>
{
};

class HeadingRecovery : public testing::TestWithParam<HeadingCase>
{
};

class Recovery : public testing::TestWithParam<RecoveryCase>
{
};

class RefusedCast : public testing::TestWithParam<WrongLengthCast>
{
};

class RefusedLength : public testing::TestWithParam<RefusedScanLength>
{
};

class RefusedScan : public testing::TestWithParam<UnusableScan>
{
};

class RefusedSettings : public testing::TestWithParam<Refusal>
{
};

std::string acceptanceSuffix(const testing::TestParamInfo<Acceptance>& info)
{
  return info.param.label;
}

std::string acceptedScanLengthSuffix(
  const testing::TestParamInfo<AcceptedScanLength>& info)
{
  return info.param.label;
}

/**
 * @brief The scan a perfect sensor standing at `pose` would report.
 * @details At the ray count and the far bound every case runs at unless it is
 * the ray count or the far bound that the case is about.
 */
std::vector<double> castOf(const FSM::GridReference& map, const Pose& pose,
  const unsigned int num_rays = kRayCount,
  const double far_bound = kRangeMax)
{
  return map.ranges(pose, num_rays, far_bound);
}

std::string ceilingCaseSuffix(const testing::TestParamInfo<CeilingCase>& info)
{
  return info.param.label;
}

/**
 * @brief The heading a displacement of `metres` subtends at the range the scan
 * actually reaches to.
 * @details A heading is only ever as well determined as the walls it is read
 * off are far away: the same error in a wall's position is twice the error in
 * heading when the wall is half as far. So the bound on a heading is stated as
 * the bound on a position, converted at the scan's own mean range, rather than
 * as an angle picked to suit a room.
 */
double headingBound(const std::vector<double>& scan, const double metres)
{
  double total = 0.0;

  for (const double range : scan)
    total += range;

  return std::atan(metres / (total / scan.size()));
}

std::string headingCaseSuffix(const testing::TestParamInfo<HeadingCase>& info)
{
  return info.param.label;
}

/**
 * @brief How far apart two headings are, whichever way round the turn they are
 * written.
 */
double headingError(const double first, const double second)
{
  return std::abs(std::remainder(first - second, 2.0 * M_PI));
}

/**
 * @brief How far apart two poses stand.
 */
double positionError(const Pose& first, const Pose& second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

/**
 * @brief The readings of `scan` the ceiling leaves as distances.
 * @details A heading is read off the walls a scan reached, and a ray that met
 * nothing reached none, so a bound stated at a scan's own mean range has to be
 * stated at the mean of these rather than of the whole scan. Twenty rays
 * standing at a far bound of a hundred metres would otherwise carry that mean
 * up past anything the room contains and quietly turn a bound of five
 * centimetres into a far tighter one.
 */
std::vector<double> readingsUnderTheCeiling(const std::vector<double>& scan)
{
  std::vector<double> under;

  for (const double range : scan)
  {
    if (range < kLargestUsableReading)
      under.push_back(range);
  }

  return under;
}

std::string recoveryCaseSuffix(const testing::TestParamInfo<RecoveryCase>& info)
{
  return info.param.label;
}

std::string refusalSuffix(const testing::TestParamInfo<Refusal>& info)
{
  return info.param.label;
}

std::string refusedScanLengthSuffix(
  const testing::TestParamInfo<RefusedScanLength>& info)
{
  return info.param.label;
}

/**
 * @brief The settings the ROS 1 build's own settings file ships.
 * @details Almost every one of them differs from the value this library falls
 * back to when it is told nothing, so a case run at both is a case that does not
 * depend on either. The two that matter most to a refinement here are the
 * correspondence distance, a metre rather than thirty centimetres, and the
 * angular correction allowed, half a turn rather than an eighth of one.
 */
IcpParameters shippedParameters()
{
  IcpParameters parameters;

  parameters.max_correspondence_dist = 1.0;
  parameters.sigma = 0.01;
  parameters.max_angular_correction_deg = 180.0;
  parameters.max_linear_correction = 10.0;
  parameters.max_iterations = 100;
  parameters.epsilon_xy = 0.00001;
  parameters.epsilon_theta = 0.00001;
  parameters.use_corr_tricks = false;
  parameters.restart = false;
  parameters.restart_threshold_mean_error = 0.0001;
  parameters.restart_dt = 0.01;
  parameters.restart_dtheta = 0.01;
  parameters.clustering_threshold = 1.025;
  parameters.orientation_neighbourhood = 20;
  parameters.use_point_to_line_distance = true;
  parameters.do_alpha_test = true;
  parameters.do_alpha_test_thresholdDeg = 5.0;
  parameters.outliers_maxPerc = 1.0;
  parameters.outliers_adaptive_order = 1.0;
  parameters.outliers_adaptive_mult = 2.0;
  parameters.do_visibility_test = false;
  parameters.outliers_remove_doubles = false;
  parameters.do_compute_covariance = false;
  parameters.debug_verify_tricks = false;
  parameters.use_ml_weights = false;
  parameters.use_sigma_weights = false;

  return parameters;
}

std::string unusableScanSuffix(const testing::TestParamInfo<UnusableScan>& info)
{
  return info.param.label;
}

std::string wrongLengthCastSuffix(
  const testing::TestParamInfo<WrongLengthCast>& info)
{
  return info.param.label;
}

const RecoveryCase kEveryRecoveryCase[] = {
  RecoveryCase{.label = "default_settings_in_a_room_at_the_world_origin",
    .placement = Pose{}, .parameters = IcpParameters{}},
  RecoveryCase{.label = "default_settings_in_a_room_moved_and_turned",
    .placement = kTurnedPlacement, .parameters = IcpParameters{}},
  RecoveryCase{.label = "shipped_settings_in_a_room_at_the_world_origin",
    .placement = Pose{}, .parameters = shippedParameters()},
  RecoveryCase{.label = "shipped_settings_in_a_room_moved_and_turned",
    .placement = kTurnedPlacement, .parameters = shippedParameters()}};

const CeilingCase kEveryCeilingCase[] = {
  CeilingCase{.label = "a_far_bound_on_the_ceiling_at_the_default_settings",
    .far_bound = kLargestUsableReading, .parameters = IcpParameters{}},
  CeilingCase{.label = "a_far_bound_on_the_ceiling_at_the_shipped_settings",
    .far_bound = kLargestUsableReading, .parameters = shippedParameters()},
  CeilingCase{.label = "a_far_bound_beyond_the_ceiling_at_the_default_settings",
    .far_bound = 1.2 * kLargestUsableReading, .parameters = IcpParameters{}},
  CeilingCase{.label = "a_far_bound_beyond_the_ceiling_at_the_shipped_settings",
    .far_bound = 1.2 * kLargestUsableReading,
    .parameters = shippedParameters()}};

const HeadingCase kEveryHeadingCase[] = {
  HeadingCase{.label = "off_by_two_and_a_third_degrees_at_the_default_settings",
    .degrees = 2.3, .parameters = IcpParameters{}},
  HeadingCase{.label = "off_by_five_and_seven_tenths_degrees_at_the_default_settings",
    .degrees = 5.7, .parameters = IcpParameters{}},
  HeadingCase{.label = "off_by_ten_and_two_fifths_degrees_at_the_default_settings",
    .degrees = 10.4, .parameters = IcpParameters{}},
  HeadingCase{.label = "off_by_seventeen_and_nine_tenths_degrees_at_the_default_settings",
    .degrees = 17.9, .parameters = IcpParameters{}},
  HeadingCase{.label = "off_by_two_and_a_third_degrees_at_the_shipped_settings",
    .degrees = 2.3, .parameters = shippedParameters()},
  HeadingCase{.label = "off_by_five_and_seven_tenths_degrees_at_the_shipped_settings",
    .degrees = 5.7, .parameters = shippedParameters()},
  HeadingCase{.label = "off_by_ten_and_two_fifths_degrees_at_the_shipped_settings",
    .degrees = 10.4, .parameters = shippedParameters()},
  HeadingCase{.label = "off_by_seventeen_and_nine_tenths_degrees_at_the_shipped_settings",
    .degrees = 17.9, .parameters = shippedParameters()}};

}

/**
 * @brief A heading wrong by a quarter turn is neither turned back nor refused.
 * @details The candidates the ROS 1 build handed its refinement all carried an
 * orientation that disagreed with the scan they had been chosen for, so what
 * the refinement does with a badly turned candidate is not an academic
 * question. Several degrees it takes out entirely, as the cases over headings
 * do. A quarter turn it does not, and this is what a host has to know about it:
 * the answer comes back marked good, from a solver allowed half a turn of
 * correction, with the heading still about where it went in and the position,
 * which was exact, dragged the better part of a metre off it.
 * Nothing in the answer says so. The one thing that does is the correspondence
 * error, which is why the library reports it: measured against another
 * candidate of the same scan, the lost one is an order of magnitude worse. That
 * comparison is the only signal there is, and a host that ranks its candidates
 * by it can tell them apart. One that reads a single answer cannot.
 * The lost candidate's error is held to a floor of its own as well as to a
 * multiple of the recovered one's. The recovered one is a match of a room cast
 * exactly, so its error can fall as far as the arithmetic allows and any
 * multiple of it can be satisfied by two numbers that are both nothing. The
 * floor is what the same correspondences would have come to had the match
 * landed on the pose, at the accuracy the finest map this algorithm runs on
 * could support.
 * @remark This case pins a limit rather than a virtue. It fails if the
 * refinement is ever made to notice this, which is the right way round: a
 * change that made it notice would want to be seen rather than absorbed.
 */
TEST(Refinement, AHeadingWrongByAQuarterTurnIsNeitherTurnedBackNorRefused)
{
  const IcpParameters parameters = shippedParameters();

  const auto room = std::make_shared<const Room>();

  const Pose truth = room->worldFrom(kTruthInTheRoom);

  const double wrong_by = 90.4 * M_PI / 180.0;

  const Pose good{truth.x + kDisplacement.x, truth.y + kDisplacement.y,
    std::remainder(truth.t + kDisplacement.t, 2.0 * M_PI)};

  const Pose lost{truth.x, truth.y,
    std::remainder(truth.t + wrong_by, 2.0 * M_PI)};

  const std::vector<double> scan = castOf(*room, truth);

  const IcpRefiner refiner(parameters, room);

  const auto from_good = refiner.refine(scan, good, kRangeMin, kRangeMax);
  const auto from_lost = refiner.refine(scan, lost, kRangeMin, kRangeMax);

  ASSERT_TRUE(from_good.has_value());

  ASSERT_TRUE(from_lost.has_value())
    << "the refinement refused a candidate a quarter turn out. That would be "
    << "better than what is asserted below, and this case is what would have "
    << "to be rewritten to say so.";

  EXPECT_GT(headingError(from_lost->corrected.t, truth.t), wrong_by / 2.0)
    << "the heading went in " << wrong_by << " radians out and came back "
    << headingError(from_lost->corrected.t, truth.t) << " radians out. These "
    << "settings allow a correction of "
    << parameters.max_angular_correction_deg << " degrees, so the solver was "
    << "permitted to make the whole of it and did not.";

  EXPECT_GT(positionError(from_lost->corrected, truth), kSupportableMetres)
    << "the position went in exact and came back "
    << positionError(from_lost->corrected, truth) << " metres out.";

  EXPECT_GT(from_lost->error, 10.0 * from_good->error)
    << "the lost candidate reported a correspondence error of "
    << from_lost->error << " against " << from_good->error << " from a "
    << "candidate of the same scan that was recovered. If those two were not "
    << "far apart there would be nothing at all to tell them apart by, since "
    << "both answers are marked good.";

  EXPECT_GT(from_lost->error, from_lost->num_valid * kSupportableMetres)
    << "the lost candidate reported " << from_lost->error << " over "
    << from_lost->num_valid << " correspondences, which is "
    << from_lost->error / from_lost->num_valid << " metres apiece. The solver "
    << "sums the distance from each measured point to the segment it was "
    << "paired with, so a candidate that had landed on the pose would sit "
    << "inside the " << kSupportableMetres << " metres a map of this "
    << "algorithm's own cell size can support and this one has to be well "
    << "outside it. The comparison against the recovered candidate above "
    << "cannot say that on its own: that one is a match of a room cast exactly "
    << "and its error can fall as far as the arithmetic allows, so ten times "
    << "it would still be satisfied by a separation of nothing at all.";
}

/**
 * @brief A scan of no rays at all is refused before the map is touched.
 * @details The length of the scan is the length the cast is taken to, so a scan
 * of nothing would ask the map for a cast of nothing and hand the solver two
 * empty scans. The refusal comes first, and the recording room is here to say
 * so: nought casts, not one.
 */
TEST(Refinement, AnEmptyScanIsRefusedWithoutConsultingTheMap)
{
  const auto room = std::make_shared<const Room>();
  const auto map = std::make_shared<const RecordingRoom>(room);

  const IcpRefiner refiner(IcpParameters{}, map);

  const auto refined = refiner.refine(std::span<const double>{},
    room->worldFrom(kTruthInTheRoom), kRangeMin, kRangeMax);

  ASSERT_FALSE(refined.has_value())
    << "a scan of no rays cannot be refined into anything, and an answer here "
    << "would be an answer computed from nothing at all.";

  EXPECT_EQ(refined.error(), IcpError::scan_empty)
    << "the reason has to be the one that fits, since a host chooses what to do "
    << "with a refusal by reading it. An empty scan is a host's own mistake and "
    << "a failed match is not, and a scan of no rays is a scan of nothing "
    << "rather than a scan of too few: nought is under the " << kFewestRays
    << " rays the solver takes, so the two rules both reach it and the one "
    << "that names it for what it is has to be the one that answers.";

  EXPECT_EQ(map->everyCast().size(), 0U)
    << "the map was consulted " << map->everyCast().size() << " times for a "
    << "scan that was never going to be matched. A cast is the expensive half "
    << "of a refinement and the refusal costs nothing, so the refusal comes "
    << "first.";
}

/**
 * @brief An estimate that is already exact comes back as it went in.
 * @details The reference is cast at the estimate and the measurement was taken
 * at the truth, so when the two poses are the same pose the two scans are the
 * same scan, ray for ray, and there is nothing for the solver to find. What is
 * asserted is that the composition adds nothing where nothing is what came back:
 * the corrected pose has to be the estimate to within the solver's own stopping
 * thresholds, which are the smallest step it promises to notice and therefore
 * the tightest an answer can honestly be held to.
 */
TEST(Refinement, AnEstimateAlreadyExactComesBackUnchanged)
{
  const IcpParameters parameters = shippedParameters();

  const auto room = std::make_shared<const Room>(kTurnedPlacement);

  const Pose truth = room->worldFrom(kTruthInTheRoom);

  const std::vector<double> scan = castOf(*room, truth);

  const IcpRefiner refiner(parameters, room);

  const auto refined = refiner.refine(scan, truth, kRangeMin, kRangeMax);

  ASSERT_TRUE(refined.has_value())
    << "two identical scans are the easiest match there is, and a refusal here "
    << "means no pair of scans can be matched at all.";

  EXPECT_LT(positionError(refined->corrected, truth), parameters.epsilon_xy)
    << "the estimate was already the truth and the answer moved it "
    << positionError(refined->corrected, truth) << " metres, which is further "
    << "than the " << parameters.epsilon_xy << " metre step the solver stops "
    << "at. A composition that adds something where the solver reported "
    << "nothing adds it to every answer.";

  EXPECT_LT(headingError(refined->corrected.t, truth.t),
    parameters.epsilon_theta)
    << "the estimate was already the truth and the answer turned it "
    << headingError(refined->corrected.t, truth.t) << " radians, which is more "
    << "than the " << parameters.epsilon_theta << " radian step the solver "
    << "stops at.";

  EXPECT_LT(refined->error, kRayCount * 1e-16)
    << "the two scans are identical, so every correspondence is at nought "
    << "distance and the sum of their squares is nought. The bound allows each "
    << "of the " << kRayCount << " rays a residual of ten nanometres, which is "
    << "already far below anything a lidar or a map could mean.";
}

/**
 * @brief A map with nothing in it to match against is the solver's own refusal.
 * @details The measured scan is a good scan of a real room. The map answers
 * every ray with nought, which is carried into the match as a reading that
 * cannot be used, so the solver is handed a reference of no usable points and
 * refuses. That refusal belongs to the solver rather than to this library, and
 * it has to arrive as such: the ROS 1 build answers it by keeping the estimate
 * it started from, and a host here cannot make that choice unless it can tell
 * this refusal from a malformed scan.
 */
TEST(Refinement, AReferenceWithNoUsableReadingIsTheSolversOwnRefusal)
{
  const auto room = std::make_shared<const Room>();
  const auto nothing = std::make_shared<const MapAnsweringNothing>();

  const std::vector<double> scan =
    castOf(*room, room->worldFrom(kTruthInTheRoom));

  const IcpRefiner refiner(shippedParameters(), nothing);

  const auto refined = refiner.refine(scan,
    room->worldFrom(kTruthInTheRoom), kRangeMin, kRangeMax);

  ASSERT_FALSE(refined.has_value())
    << "there was not one usable point in the reference and the refinement "
    << "reported an answer. Whatever that answer is, it was not computed from "
    << "the map.";

  EXPECT_EQ(refined.error(), IcpError::matching_failed)
    << "the scan handed in was a good scan, so the refusal is the solver's "
    << "rather than the scan's. A host that reads this as a malformed scan "
    << "would go looking at its sensor for a fault that is in its map.";
}

/**
 * @brief Refining twice gives the same answer twice, and refining something
 * else in between changes neither.
 * @details The library states that nothing survives a refinement. If something
 * did, the answer a candidate got would depend on which candidates were refined
 * before it, and a host refining ten survivors of a sift would get ten answers
 * that could not be reproduced one at a time. The two answers are required to
 * be equal to the bit rather than to a tolerance, since anything else is a
 * tolerance chosen to hide a difference.
 */
TEST(Refinement, RefiningTwiceGivesTheSameAnswerBothTimes)
{
  const auto room = std::make_shared<const Room>(kTurnedPlacement);

  const Pose truth = room->worldFrom(kTruthInTheRoom);
  const Pose estimate{truth.x + kDisplacement.x, truth.y + kDisplacement.y,
    std::remainder(truth.t + kDisplacement.t, 2.0 * M_PI)};

  const Pose elsewhere = room->worldFrom(Pose{2.10, 2.00, -1.20});

  const std::vector<double> scan = castOf(*room, truth);
  const std::vector<double> another = castOf(*room, elsewhere);

  const IcpRefiner refiner(shippedParameters(), room);

  const auto first = refiner.refine(scan, estimate, kRangeMin, kRangeMax);
  const auto between = refiner.refine(another, elsewhere, kRangeMin, kRangeMax);
  const auto second = refiner.refine(scan, estimate, kRangeMin, kRangeMax);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(between.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_DOUBLE_EQ(second->corrected.x, first->corrected.x)
    << "the same scan and the same estimate gave two different answers with "
    << "another refinement in between, so something the first call left behind "
    << "was read by the third.";
  EXPECT_DOUBLE_EQ(second->corrected.y, first->corrected.y);
  EXPECT_DOUBLE_EQ(second->corrected.t, first->corrected.t);
  EXPECT_DOUBLE_EQ(second->error, first->error);
  EXPECT_EQ(second->iterations, first->iterations);
  EXPECT_EQ(second->num_valid, first->num_valid);
}

/**
 * @brief The correction is turned by the estimate's heading before it is added.
 * @details The solver answers in the frame of the reference cast, and the
 * reference was cast at the estimate, so its answer is a translation in the
 * estimate's own frame and has to be turned into the world's before it is added
 * to the estimate's position. Adding it unturned leaves an answer that looks
 * like an answer, and the mistake is invisible whenever the estimate's heading
 * is near nought, which is where a case written without thinking about it would
 * put it.
 * The estimate here therefore faces two radians, a hundred and fifteen degrees
 * away from nought, and stands twenty five centimetres from the truth. The
 * correction is that same twenty five centimetres, so turning it and not
 * turning it are two answers `2 * sin(1.0) * 0.25` apart, which is forty two
 * centimetres. The bound below is five. Forty two is not five and no arithmetic
 * satisfies both, which is the point: the unturned answer is worked out below
 * from what actually came back, and is required to be the far one of the two.
 */
TEST(Refinement, TheCorrectionIsTurnedByTheEstimatesHeadingBeforeItIsAdded)
{
  const auto room = std::make_shared<const Room>();

  const double heading = 2.0;
  const double away_x = 0.20;
  const double away_y = 0.15;

  const Pose truth = room->worldFrom(Pose{1.30, 0.90, heading});
  const Pose estimate{truth.x + away_x, truth.y + away_y, truth.t};

  const std::vector<double> scan = castOf(*room, truth);

  const IcpRefiner refiner(shippedParameters(), room);

  const auto refined = refiner.refine(scan, estimate, kRangeMin, kRangeMax);

  ASSERT_TRUE(refined.has_value());

  const double moved_x = refined->corrected.x - estimate.x;
  const double moved_y = refined->corrected.y - estimate.y;

  const double solver_x =
    moved_x * std::cos(estimate.t) + moved_y * std::sin(estimate.t);
  const double solver_y =
    -moved_x * std::sin(estimate.t) + moved_y * std::cos(estimate.t);

  const Pose unturned{estimate.x + solver_x, estimate.y + solver_y,
    refined->corrected.t};

  ASSERT_GT(positionError(unturned, truth), 4.0 * kSupportableMetres)
    << "the two arithmetics have to be far enough apart for this case to tell "
    << "them apart, and here they are only "
    << positionError(unturned, truth) << " metres apart against a bound of "
    << kSupportableMetres << ". Either the estimate's heading or its distance "
    << "from the truth is too small for the case to mean anything.";

  EXPECT_LT(positionError(refined->corrected, truth), kSupportableMetres)
    << "the answer is " << positionError(refined->corrected, truth)
    << " metres from the truth. Adding the solver's translation without "
    << "turning it by the estimate's heading would have put it "
    << positionError(unturned, truth) << " metres away instead.";

  EXPECT_LT(headingError(refined->corrected.t, truth.t),
    headingBound(scan, kSupportableMetres))
    << "the heading is out by " << headingError(refined->corrected.t, truth.t)
    << " radians, more than the " << headingBound(scan, kSupportableMetres)
    << " radians a five centimetre error at this scan's own range comes to.";
}

/**
 * @brief The map is cast from once, at the estimate, and not again.
 * @details This is the whole of the library. The solver walks the measured scan
 * onto one frozen cast and the map is not read again however far it moves, which
 * is what makes this a match between two scans rather than the search over the
 * map that `fsm_localisation::Corrector` runs. A refinement that cast afresh at
 * every pose it tried would recover this displacement too, so recovery cannot
 * be what says the difference; the count can.
 * The cast is required to be at the estimate itself, to the ray of the scan
 * handed in, and to the scan's own far bound. A cast taken anywhere else is a
 * reference for a pose nobody asked about, and a cast of a different length is
 * a reference the two scans would have to be resampled between.
 */
TEST(Refinement, TheMapIsCastFromOnceAtTheEstimateAndNotAgain)
{
  const auto room = std::make_shared<const Room>(kTurnedPlacement);
  const auto map = std::make_shared<const RecordingRoom>(room);

  const Pose truth = room->worldFrom(kTruthInTheRoom);
  const Pose estimate{truth.x + kDisplacement.x, truth.y + kDisplacement.y,
    std::remainder(truth.t + kDisplacement.t, 2.0 * M_PI)};

  const std::vector<double> scan = castOf(*room, truth);

  const IcpRefiner refiner(shippedParameters(), map);

  const auto refined = refiner.refine(scan, estimate, kRangeMin, kRangeMax);

  ASSERT_TRUE(refined.has_value());

  ASSERT_GE(refined->iterations, 2)
    << "the solver stopped after " << refined->iterations << " cycle, so this "
    << "case cannot tell one cast from one cast per cycle. The estimate has to "
    << "start far enough from the truth to make the solver work for it.";

  ASSERT_EQ(map->everyCast().size(), 1U)
    << "the map was consulted " << map->everyCast().size() << " times over "
    << refined->iterations << " solver cycles. One cast, taken once, is what "
    << "separates this refinement from a search over the map, and a second "
    << "cast means the map is being read as the solver moves.";

  EXPECT_DOUBLE_EQ(map->everyCast()[0].pose.x, estimate.x)
    << "the one cast has to be taken where the estimate says the sensor was, "
    << "since that is the frame the solver's answer comes back in and is "
    << "composed onto.";
  EXPECT_DOUBLE_EQ(map->everyCast()[0].pose.y, estimate.y);
  EXPECT_DOUBLE_EQ(map->everyCast()[0].pose.t, estimate.t);

  EXPECT_EQ(map->everyCast()[0].num_rays, kRayCount)
    << "the cast has to carry as many rays as the scan handed in, or the two "
    << "scans do not share one angle array and ray " << kRayCount - 1U
    << " of one is not ray " << kRayCount - 1U << " of the other.";

  EXPECT_DOUBLE_EQ(map->everyCast()[0].max_range, kRangeMax)
    << "the cast has to reach as far as the scan says it can see, or a ray "
    << "that met nothing reports one distance in the measurement and another "
    << "in the reference.";
}

TEST(Validate, TheDefaultSettingsAreUsable)
{
  EXPECT_EQ(cbgl::validate(IcpParameters{}), "")
    << "a default constructed settings set has to be usable, or a host that "
    << "declares a setting it has no opinion about cannot start. It was "
    << "refused with: " << cbgl::validate(IcpParameters{});
}

/**
 * @brief The settings the ROS 1 build runs this solver with are usable.
 * @details Several of them sit exactly on the edge of what is permitted: the
 * proportion of correspondences to keep is one, the percentile the adaptive
 * threshold is taken at is one, and both are the largest value their rule
 * allows. A validator written with the comparisons the wrong way round would
 * refuse the very settings the algorithm ships, and nothing else here would
 * notice.
 */
TEST(Validate, TheShippedSettingsAreUsable)
{
  const IcpParameters parameters = shippedParameters();

  EXPECT_EQ(cbgl::validate(parameters), "")
    << "these are the values the ROS 1 package's own settings file ships, and "
    << "the values every comparison between the two builds is measured at. A "
    << "refusal here means the shipped configuration cannot be launched. It "
    << "was refused with: " << cbgl::validate(parameters);
}

/**
 * @brief A scan at either end of what the solver will take is refined.
 * @details The solver reads the length of a scan before it reads anything else
 * in it and will not take one of fewer than ten rays or of more than ten
 * thousand. The library carries the same two bounds so that it can say which of
 * them was crossed, and a bound written one out on either side would take one
 * of the two lengths here away from a host that is entitled to it.
 * The estimate is the truth, so the measured scan and the cast are the same
 * scan ray for ray and there is nothing for the solver to find. The answer has
 * to come back as the estimate went in, to within the smallest step these
 * settings promise to notice, which is what says the length was taken rather
 * than that something further in made an answer out of it anyway.
 * The cast is asked for at that same length, since a length the library accepts
 * is a length it holds both scans at.
 */
TEST_P(AcceptedLength, AScanAtTheEdgeOfWhatTheSolverTakesIsRefined)
{
  const unsigned int num_rays = GetParam().num_rays;

  const IcpParameters parameters = shippedParameters();

  const auto room = std::make_shared<const Room>();
  const auto map = std::make_shared<const RecordingRoom>(room);

  const Pose truth = room->worldFrom(kTruthInTheRoom);

  const std::vector<double> scan = castOf(*room, truth, num_rays);

  const IcpRefiner refiner(parameters, map);

  const auto refined = refiner.refine(scan, truth, kRangeMin, kRangeMax);

  ASSERT_TRUE(refined.has_value())
    << "a scan of " << num_rays << " rays is inside the " << kFewestRays
    << " to " << kMostRays << " the solver takes, and it is matched here "
    << "against a cast from the very pose it was taken at, which is the "
    << "easiest match there is.";

  ASSERT_EQ(map->everyCast().size(), 1U);

  EXPECT_EQ(map->everyCast()[0].num_rays, num_rays)
    << "the map was asked for " << map->everyCast()[0].num_rays << " ranges "
    << "for a scan of " << num_rays << " rays. Both scans are held at the "
    << "length handed in, whichever end of the solver's own bounds it sits at.";

  EXPECT_LT(positionError(refined->corrected, truth), parameters.epsilon_xy)
    << "the estimate was already the truth and the answer moved it "
    << positionError(refined->corrected, truth) << " metres, which is further "
    << "than the " << parameters.epsilon_xy << " metre step the solver stops "
    << "at.";

  EXPECT_LT(headingError(refined->corrected.t, truth.t),
    parameters.epsilon_theta)
    << "the estimate was already the truth and the answer turned it "
    << headingError(refined->corrected.t, truth.t) << " radians, which is more "
    << "than the " << parameters.epsilon_theta << " radian step the solver "
    << "stops at.";
}

TEST_P(AcceptedSettings, ASettingInsideItsRangeIsAccepted)
{
  IcpParameters parameters;
  GetParam().apply(parameters);

  EXPECT_EQ(cbgl::validate(parameters), "")
    << "the case " << GetParam().label << " puts a setting at a value that is "
    << "permitted, and it was refused with: " << cbgl::validate(parameters)
    << ". A validator that refuses a usable value is as costly as one that "
    << "accepts an unusable one: it stops a run that would have answered.";
}

/**
 * @brief A scan reaching to the hundred metre ceiling is refined from what is
 * left of it.
 * @details The solver refuses whole any scan that offers it a usable reading of
 * a hundred metres or more. A lidar whose own far bound is that high marks that
 * bound usable, so every one of its rays that met nothing would carry the whole
 * scan into that refusal, and it would arrive as a failed match: a host would
 * be told that this pair of scans could not be matched, would go looking at its
 * map, and would find every refinement it ever ran refused for the same reason
 * it could not read. The library marks such a reading unusable instead, exactly
 * as a reading past the far bound already is, and matches the rest.
 * The room here is given a way out of it, so about twenty of the three hundred
 * and sixty rays leave through the gap and come back at the far bound they were
 * cast to. That far bound is put on the ceiling itself in two of these cases
 * and beyond it in the other two, since a rule written as **above** a hundred
 * rather than **at or above** it would let a bound of exactly a hundred through
 * and refuse the scan. Both settings sets are run at both.
 * The estimate is displaced by the same twenty three centimetres and three
 * degrees the recovery cases use, and what is asked is that it still comes
 * back: the rays that met nothing were never going to give the match anything,
 * so losing them costs it nothing either.
 */
TEST_P(CeilingReadings, AScanReachingToTheCeilingIsRefinedFromWhatIsLeft)
{
  const CeilingCase ceiling = GetParam();

  const auto room = std::make_shared<const Room>(kTurnedPlacement, true);

  const Pose truth = room->worldFrom(kTruthInTheRoom);
  const Pose estimate{truth.x + kDisplacement.x, truth.y + kDisplacement.y,
    std::remainder(truth.t + kDisplacement.t, 2.0 * M_PI)};

  const std::vector<double> scan =
    castOf(*room, truth, kRayCount, ceiling.far_bound);

  const std::vector<double> under = readingsUnderTheCeiling(scan);

  ASSERT_LT(under.size(), scan.size())
    << "not one of the " << scan.size() << " rays came back at or beyond the "
    << kLargestUsableReading << " metres the solver stops treating a reading "
    << "as a distance at, so this case would pass with that rule taken out "
    << "altogether. The room has to be one a ray can leave.";

  ASSERT_GT(positionError(estimate, truth), 4.0 * kSupportableMetres)
    << "the estimate starts " << positionError(estimate, truth) << " metres "
    << "from the truth against a bound of " << kSupportableMetres << ". A case "
    << "whose estimate starts inside the bound is passed by a refinement that "
    << "does nothing at all.";

  const IcpRefiner refiner(ceiling.parameters, room);

  const auto refined = refiner.refine(scan, estimate, kRangeMin,
    ceiling.far_bound);

  ASSERT_TRUE(refined.has_value())
    << scan.size() - under.size() << " of the " << scan.size() << " rays left "
    << "the room and came back at the far bound of " << ceiling.far_bound
    << " metres they were cast to. Marked usable, any one of them is enough "
    << "for the solver to refuse the whole scan, and it refuses it as a match "
    << "that failed, which is the one thing a host cannot act on.";

  EXPECT_LT(positionError(refined->corrected, truth), kSupportableMetres)
    << "the answer is " << positionError(refined->corrected, truth)
    << " metres from the truth, having started "
    << positionError(estimate, truth) << " metres from it. The "
    << scan.size() - under.size() << " rays that met nothing were never going "
    << "to tell the match where a wall was, so dropping them can cost it "
    << "nothing.";

  EXPECT_LT(headingError(refined->corrected.t, truth.t),
    headingBound(under, kSupportableMetres))
    << "the heading is out by " << headingError(refined->corrected.t, truth.t)
    << " radians, more than the " << headingBound(under, kSupportableMetres)
    << " radians a five centimetre error subtends at the mean range of the "
    << "readings this scan reached a wall at.";
}

/**
 * @brief An estimate whose heading is wrong by degrees is turned back onto the
 * truth without its position being traded away.
 * @details This is the case the ROS 1 build lived in. Every candidate that
 * reached its refinement carried an orientation that disagreed with the scan it
 * had been chosen for, so a refinement that only tidied up a position would
 * hand back the same wrong heading it was given, dressed as an answer.
 * The estimate here stands exactly where the sensor stood and faces the wrong
 * way by two and a third, five and seven tenths, ten and two fifths, and
 * seventeen and nine tenths degrees, at each of the two settings sets. What is
 * required of each is that the heading comes back at least ten times better
 * than it went in and inside the angle a five centimetre error subtends at this
 * scan's own range, which for this room is about one and seven tenths degrees;
 * and that the position, which was already exact, is not spent buying that
 * heading.
 * **None of those angles is a whole number of degrees, and that is the whole
 * reason they read as they do.** Three hundred and sixty rays over the turn put
 * one ray every degree, so a heading wrong by a whole number of degrees casts a
 * reference that is the measured scan with its rays relabelled: the same
 * distances in the same order, starting from a different index. The solver
 * matches that exactly, to the last bit of the arithmetic, and a case built on
 * one would report a recovery it never had to work for. Off the lattice the two
 * scans sample the walls at different points and the answer is the solver's own
 * rather than a coincidence of the ray count.
 * The bound at the largest of these is the five centimetre one rather than the
 * tenth, seventeen and nine tenths degrees being a little over ten times the
 * angle five centimetres subtends here. Both are asserted at every case regardless,
 * since which of the two binds is a property of the room and not something to
 * be tuned per case.
 */
TEST_P(HeadingRecovery, AnEstimateFacingTheWrongWayIsTurnedBack)
{
  const HeadingCase heading = GetParam();

  const double wrong_by = heading.degrees * M_PI / 180.0;

  const auto room = std::make_shared<const Room>(kTurnedPlacement);

  const Pose truth = room->worldFrom(kTruthInTheRoom);
  const Pose estimate{truth.x, truth.y,
    std::remainder(truth.t + wrong_by, 2.0 * M_PI)};

  const std::vector<double> scan = castOf(*room, truth);

  const IcpRefiner refiner(heading.parameters, room);

  const auto refined = refiner.refine(scan, estimate, kRangeMin, kRangeMax);

  ASSERT_TRUE(refined.has_value())
    << "the estimate is " << heading.degrees << " degrees off and no further "
    << "from the truth than that, which is inside the "
    << heading.parameters.max_angular_correction_deg << " degrees these "
    << "settings allow a correction to reach.";

  EXPECT_LT(headingError(refined->corrected.t, truth.t), wrong_by / 10.0)
    << "the heading went in " << wrong_by << " radians out and came back "
    << headingError(refined->corrected.t, truth.t) << " radians out, which is "
    << "not the order of magnitude a refinement is there to buy. A candidate "
    << "whose heading is merely carried through is a candidate that was not "
    << "refined.";

  EXPECT_LT(headingError(refined->corrected.t, truth.t),
    headingBound(scan, kSupportableMetres))
    << "the heading is out by " << headingError(refined->corrected.t, truth.t)
    << " radians, more than the " << headingBound(scan, kSupportableMetres)
    << " radians a five centimetre error at this scan's own mean range comes "
    << "to.";

  EXPECT_LT(positionError(refined->corrected, truth), kSupportableMetres)
    << "the position was exact before the refinement and is now "
    << positionError(refined->corrected, truth) << " metres out. A heading "
    << "bought by sliding the position is no answer: the two are reported "
    << "together and used together.";
}

/**
 * @brief A displaced estimate is walked back onto the truth.
 * @details Twenty three centimetres and three degrees from where the sensor
 * really stood, in a room whose frame is the world's and in the same room moved
 * and turned, at each of the two settings sets. The answer is required inside
 * five centimetres, which is what the finest map this algorithm is run on could
 * have supported and therefore a bound no honest refinement of an exactly cast
 * room should struggle with.
 * The displacement is asserted first, against that same bound, so that the case
 * cannot quietly become the trivial one: an estimate already inside the bound
 * would be recovered by handing it straight back.
 */
TEST_P(Recovery, ADisplacedEstimateIsWalkedBackOntoTheTruth)
{
  const RecoveryCase recovery = GetParam();

  const auto room = std::make_shared<const Room>(recovery.placement);

  const Pose truth = room->worldFrom(kTruthInTheRoom);
  const Pose estimate{truth.x + kDisplacement.x, truth.y + kDisplacement.y,
    std::remainder(truth.t + kDisplacement.t, 2.0 * M_PI)};

  const std::vector<double> scan = castOf(*room, truth);

  ASSERT_GT(positionError(estimate, truth), 4.0 * kSupportableMetres)
    << "the estimate starts " << positionError(estimate, truth) << " metres "
    << "from the truth against a bound of " << kSupportableMetres << ". A case "
    << "whose estimate starts inside the bound is passed by a refinement that "
    << "does nothing at all.";

  const IcpRefiner refiner(recovery.parameters, room);

  const auto refined = refiner.refine(scan, estimate, kRangeMin, kRangeMax);

  ASSERT_TRUE(refined.has_value())
    << "the scan is a perfect scan of the room the map describes and the "
    << "estimate is a quarter of a metre from where it was taken, which is "
    << "inside every bound either settings set puts on a correction.";

  EXPECT_LT(positionError(refined->corrected, truth), kSupportableMetres)
    << "the answer is " << positionError(refined->corrected, truth)
    << " metres from the truth, having started "
    << positionError(estimate, truth) << " metres from it. The room is cast "
    << "exactly rather than off a grid, so nothing but the solver stands "
    << "between the answer and the truth.";

  EXPECT_LT(headingError(refined->corrected.t, truth.t),
    headingBound(scan, kSupportableMetres))
    << "the heading is out by " << headingError(refined->corrected.t, truth.t)
    << " radians, more than the " << headingBound(scan, kSupportableMetres)
    << " radians a five centimetre error at this scan's own mean range comes "
    << "to.";

  EXPECT_LT(positionError(refined->corrected, truth),
    positionError(estimate, truth) / 20.0)
    << "the answer is " << positionError(refined->corrected, truth)
    << " metres out where the estimate was " << positionError(estimate, truth)
    << " metres out. An order of magnitude is the least a refinement can be "
    << "worth, and this one is handed a room cast exactly rather than off a "
    << "grid, so twenty times is asked of it.";

  EXPECT_LE(refined->num_valid, static_cast<int>(scan.size()))
    << "the solver was still holding " << refined->num_valid
    << " correspondences over a scan of " << scan.size() << " rays. Both scans "
    << "are built at the length handed in and a cast that comes back at any "
    << "other length is refused, so there are only that many pairs there to "
    << "hold and a larger count is a count of something else.";
}

/**
 * @brief A cast of another length than the scan is refused rather than matched.
 * @details The caster's own interface promises no length. A map that answers a
 * request for three hundred and sixty ranges with three hundred and fifty nine
 * has put its ray `i` at `-pi + i * 2 * pi / 359` while the measurement's ray
 * `i` sits at `-pi + i * 2 * pi / 360`, and the two would be matched index
 * against index while meaning different directions. Nothing further in would
 * notice: the solver takes two scans of different lengths quite happily and
 * answers with a pose, and that pose is wrong with nothing at all to show for
 * it. This is the one way this library could be quietly wrong, so the length is
 * required to be the scan's own and refused by its own reason where it is not.
 * A cast one ray short, a cast one ray long and a cast of nothing at all are
 * asked separately, since a check written as **fewer than** rather than **not**
 * the scan's length would pass two of the three.
 */
TEST_P(RefusedCast, ACastOfAnotherLengthIsRefusedRatherThanMatched)
{
  const WrongLengthCast wrong = GetParam();

  const auto room = std::make_shared<const Room>();
  const auto map = std::make_shared<const MapAnsweringTheWrongLength>(
    wrong.num_ranges_answered);

  const Pose truth = room->worldFrom(kTruthInTheRoom);

  const std::vector<double> scan = castOf(*room, truth);

  const IcpRefiner refiner(shippedParameters(), map);

  const auto refined = refiner.refine(scan, truth, kRangeMin, kRangeMax);

  ASSERT_FALSE(refined.has_value())
    << "the map answered a request for " << scan.size() << " ranges with "
    << wrong.num_ranges_answered << " of them and an answer came back anyway. "
    << "Whatever that answer is, it was computed between two scans that do not "
    << "share one angle array.";

  EXPECT_EQ(refined.error(), IcpError::cast_length_mismatch)
    << "the reason has to name the cast. The scan handed in was a good scan "
    << "and the solver was never reached, so a host reading this as a failed "
    << "match would go looking for a pose its map could not support rather "
    << "than for a caster that answered at a length nobody asked it for.";
}

/**
 * @brief A scan of fewer or more rays than the solver takes is refused by the
 * bound it crossed, and before the map is touched.
 * @details Ten rays and ten thousand are the solver's own bounds. It checks
 * them itself and answers a scan outside them with the one refusal it has,
 * which says only that the match failed. The library checks them first so that
 * the two are told apart: a host handed a scan too short reduced it too far,
 * one handed a scan too long did not reduce it at all, and neither is looking
 * at the same fault as a host whose map could not support the pose.
 * Each case sits one ray outside its bound, which is where a comparison written
 * with the wrong edge shows and nowhere else. The cast is the expensive half of
 * a refinement and the scan's length is free to read, so the recording room is
 * here to say the refusal came first: nought casts, not one.
 */
TEST_P(RefusedLength, AScanOutsideTheSolversRayCountIsRefusedNamingTheBound)
{
  const RefusedScanLength refused = GetParam();

  const auto room = std::make_shared<const Room>();
  const auto map = std::make_shared<const RecordingRoom>(room);

  const Pose truth = room->worldFrom(kTruthInTheRoom);

  const std::vector<double> scan = castOf(*room, truth, refused.num_rays);

  const IcpRefiner refiner(shippedParameters(), map);

  const auto refined = refiner.refine(scan, truth, kRangeMin, kRangeMax);

  ASSERT_FALSE(refined.has_value())
    << "a scan of " << refused.num_rays << " rays is outside the "
    << kFewestRays << " to " << kMostRays << " the solver will take one at, "
    << "and an answer came back. The solver cannot have produced it.";

  EXPECT_EQ(refined.error(), refused.reason)
    << "the refusal has to name the bound that was crossed. A scan of "
    << refused.num_rays << " rays crossed one of the two, and a host told only "
    << "that the match failed cannot tell a scan it reduced too far from one "
    << "it never reduced.";

  EXPECT_EQ(map->everyCast().size(), 0U)
    << "the map was consulted " << map->everyCast().size() << " times for a "
    << "scan the solver would not have looked at. The length is the first "
    << "thing about a scan that can be read and casting is the last thing a "
    << "refusal should pay for.";
}

/**
 * @brief A scan with no usable reading in it is refused before the map is
 * touched.
 * @details Each case is one of the ways a driver says it had nothing to report:
 * a ray of nought, a ray at infinity, a ray that is not a number, a ray at a
 * negative distance, and a ray beyond the far bound it was reported with. Each
 * is a separate rule in the library and any one of them could stop being
 * recognised without the others noticing, so each is its own case rather than
 * one case with a list of values in it.
 * A ray of nought is asked twice over, at two near bounds, because two rules
 * answer it and only one of them can run at any one bound. Where the near bound
 * is above nought, as every other case here has it, a reading of nought is
 * under it and is put away for that; the rule written for an exact nought is
 * reached only where the near bound is nought itself, which is the case it
 * exists for. A driver that reports nought reports having had nothing to say,
 * not a wall against the lens, and a near bound of nought is what a driver with
 * no near bound to declare declares.
 * A scan of these is not a scan the solver should be asked about, and the
 * recording room says it was not asked: the refusal comes before the cast.
 */
TEST_P(RefusedScan, AScanWithNoUsableReadingIsRefusedWithoutConsultingTheMap)
{
  const auto room = std::make_shared<const Room>();
  const auto map = std::make_shared<const RecordingRoom>(room);

  const std::vector<double> scan(kRayCount, GetParam().range);

  const IcpRefiner refiner(shippedParameters(), map);

  const auto refined = refiner.refine(scan,
    room->worldFrom(kTruthInTheRoom), GetParam().range_min, kRangeMax);

  ASSERT_FALSE(refined.has_value())
    << "every ray of this scan is " << GetParam().label << ", so there is not "
    << "one distance in it to match anything against, and an answer would be "
    << "an answer computed from nothing.";

  EXPECT_EQ(refined.error(), IcpError::scan_entirely_invalid)
    << "the reason has to say the scan was unusable rather than that the match "
    << "failed. A host reading a failed match looks at its map; a host reading "
    << "an unusable scan looks at its sensor.";

  EXPECT_EQ(map->everyCast().size(), 0U)
    << "the map was consulted " << map->everyCast().size() << " times for a "
    << "scan carrying no usable reading. The scan is read before the map is, "
    << "because reading the scan is free and casting is not.";
}

/**
 * @brief A setting outside what it is permitted is refused, and the refusal
 * names both the setting and the value it was given.
 * @details Two of these have a range of one value rather than of an interval.
 * `use_sigma_weights` is permitted only off, nothing here filling the per
 * reading deviation it weights each correspondence by, and
 * `orientation_neighbourhood` is bounded above by the most rays a scan can
 * carry, since the solver turns it into an allocation of twice its size for
 * every ray of every scan and one it cannot satisfy takes the process with it.
 * Neither reads as a range and both are refused by the same rule as one.
 */
TEST_P(RefusedSettings, ASettingOutsideItsRangeIsRefusedNamingItAndTheValue)
{
  const Refusal refusal = GetParam();

  IcpParameters parameters;
  refusal.apply(parameters);

  const std::string sentence = cbgl::validate(parameters);

  ASSERT_FALSE(sentence.empty())
    << refusal.setting << " was given " << refusal.value << ", which is "
    << "outside its permitted range, and the settings were accepted. The run "
    << "would then start and fail somewhere further in, where the cause is no "
    << "longer legible.";

  EXPECT_NE(sentence.find(refusal.setting), std::string::npos)
    << "the refusal must name the setting it is about. It said \"" << sentence
    << "\", which does not contain \"" << refusal.setting << "\", so an "
    << "operator reading a log cannot tell which of the settings to correct.";

  EXPECT_NE(sentence.find(refusal.value), std::string::npos)
    << "the refusal must name the value it was given. It said \"" << sentence
    << "\", which does not contain \"" << refusal.value << "\", so an operator "
    << "cannot tell whether the value they meant to set is the value that "
    << "arrived.";
}

INSTANTIATE_TEST_SUITE_P(
  AtEitherEndOfWhatTheSolverTakes, AcceptedLength,
  testing::Values(
    AcceptedScanLength{.label = "at_the_fewest_rays_the_solver_takes",
      .num_rays = kFewestRays},
    AcceptedScanLength{.label = "at_the_most_rays_the_solver_takes",
      .num_rays = kMostRays}),
  acceptedScanLengthSuffix);

INSTANTIATE_TEST_SUITE_P(
  AtTheEdgesOfEveryRange, AcceptedSettings,
  testing::Values(
    Acceptance{.label = "clustering_threshold_barely_above_none",
      .apply = [](IcpParameters& p) { p.clustering_threshold = 1e-9; }},
    Acceptance{.label = "do_alpha_test_thresholdDeg_of_none",
      .apply = [](IcpParameters& p) { p.do_alpha_test_thresholdDeg = 0.0; }},
    Acceptance{.label = "epsilon_theta_of_none",
      .apply = [](IcpParameters& p) { p.epsilon_theta = 0.0; }},
    Acceptance{.label = "epsilon_xy_of_none",
      .apply = [](IcpParameters& p) { p.epsilon_xy = 0.0; }},
    Acceptance{.label = "max_angular_correction_of_none",
      .apply = [](IcpParameters& p) { p.max_angular_correction_deg = 0.0; }},
    Acceptance{.label = "max_angular_correction_of_half_a_turn",
      .apply = [](IcpParameters& p) { p.max_angular_correction_deg = 180.0; }},
    Acceptance{.label = "max_iterations_of_one",
      .apply = [](IcpParameters& p) { p.max_iterations = 1; }},
    Acceptance{.label = "max_linear_correction_of_none",
      .apply = [](IcpParameters& p) { p.max_linear_correction = 0.0; }},
    Acceptance{.label = "orientation_neighbourhood_of_one",
      .apply = [](IcpParameters& p) { p.orientation_neighbourhood = 1; }},
    Acceptance{.label = "orientation_neighbourhood_at_the_most_rays",
      .apply = [](IcpParameters& p)
        { p.orientation_neighbourhood = static_cast<int>(kMostRays); }},
    Acceptance{.label = "outliers_adaptive_mult_of_none",
      .apply = [](IcpParameters& p) { p.outliers_adaptive_mult = 0.0; }},
    Acceptance{.label = "outliers_adaptive_order_at_the_whole",
      .apply = [](IcpParameters& p) { p.outliers_adaptive_order = 1.0; }},
    Acceptance{.label = "outliers_maxPerc_at_the_whole",
      .apply = [](IcpParameters& p) { p.outliers_maxPerc = 1.0; }},
    Acceptance{.label = "restart_dt_of_none",
      .apply = [](IcpParameters& p) { p.restart_dt = 0.0; }},
    Acceptance{.label = "restart_dtheta_of_none",
      .apply = [](IcpParameters& p) { p.restart_dtheta = 0.0; }},
    Acceptance{.label = "restart_threshold_mean_error_of_none",
      .apply = [](IcpParameters& p)
        { p.restart_threshold_mean_error = 0.0; }},
    Acceptance{.label = "sigma_barely_above_none",
      .apply = [](IcpParameters& p) { p.sigma = 1e-9; }}),
  acceptanceSuffix);

INSTANTIATE_TEST_SUITE_P(
  AtTheCeilingAndBeyondIt, CeilingReadings,
  testing::ValuesIn(kEveryCeilingCase),
  ceilingCaseSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverEveryWayAHeadingCanBeWrong, HeadingRecovery,
  testing::ValuesIn(kEveryHeadingCase),
  headingCaseSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverEveryRoomAndEverySettingsSet, Recovery,
  testing::ValuesIn(kEveryRecoveryCase),
  recoveryCaseSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverEveryWayACastCanBeTheWrongLength, RefusedCast,
  testing::Values(
    WrongLengthCast{.label = "a_cast_one_ray_short",
      .num_ranges_answered = kRayCount - 1U},
    WrongLengthCast{.label = "a_cast_one_ray_long",
      .num_ranges_answered = kRayCount + 1U},
    WrongLengthCast{.label = "a_cast_of_nothing_at_all",
      .num_ranges_answered = 0U}),
  wrongLengthCastSuffix);

INSTANTIATE_TEST_SUITE_P(
  OneRayOutsideEitherBound, RefusedLength,
  testing::Values(
    RefusedScanLength{.label = "one_ray_under_the_fewest",
      .num_rays = kFewestRays - 1U, .reason = IcpError::scan_too_short},
    RefusedScanLength{.label = "one_ray_over_the_most",
      .num_rays = kMostRays + 1U, .reason = IcpError::scan_too_long}),
  refusedScanLengthSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverEveryWayAScanCanBeUnusable, RefusedScan,
  testing::Values(
    UnusableScan{.label = "every_ray_at_none", .range = 0.0},
    UnusableScan{.label = "every_ray_at_infinity",
      .range = std::numeric_limits<double>::infinity()},
    UnusableScan{.label = "every_ray_not_a_number",
      .range = std::numeric_limits<double>::quiet_NaN()},
    UnusableScan{.label = "every_ray_below_none", .range = -1.0},
    UnusableScan{.label = "every_ray_beyond_the_far_bound",
      .range = 2.0 * kRangeMax},
    UnusableScan{.label = "every_ray_at_none_with_the_near_bound_at_none",
      .range = 0.0, .range_min = 0.0}),
  unusableScanSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverEverySettingWithARange, RefusedSettings,
  testing::Values(
    Refusal{.label = "clustering_threshold_of_none",
      .apply = [](IcpParameters& p) { p.clustering_threshold = 0.0; },
      .setting = "clustering_threshold", .value = "0"},
    Refusal{.label = "clustering_threshold_below_none",
      .apply = [](IcpParameters& p) { p.clustering_threshold = -0.25; },
      .setting = "clustering_threshold", .value = "-0.25"},
    Refusal{.label = "do_alpha_test_thresholdDeg_below_none",
      .apply = [](IcpParameters& p) { p.do_alpha_test_thresholdDeg = -20.0; },
      .setting = "do_alpha_test_thresholdDeg", .value = "-20"},
    Refusal{.label = "epsilon_theta_below_none",
      .apply = [](IcpParameters& p) { p.epsilon_theta = -0.001; },
      .setting = "epsilon_theta", .value = "-0.001"},
    Refusal{.label = "epsilon_xy_below_none",
      .apply = [](IcpParameters& p) { p.epsilon_xy = -0.001; },
      .setting = "epsilon_xy", .value = "-0.001"},
    Refusal{.label = "epsilon_xy_not_a_number",
      .apply = [](IcpParameters& p)
        { p.epsilon_xy = std::numeric_limits<double>::quiet_NaN(); },
      .setting = "epsilon_xy", .value = "nan"},
    Refusal{.label = "max_angular_correction_below_none",
      .apply = [](IcpParameters& p) { p.max_angular_correction_deg = -45.0; },
      .setting = "max_angular_correction_deg", .value = "-45"},
    Refusal{.label = "max_correspondence_dist_of_none",
      .apply = [](IcpParameters& p) { p.max_correspondence_dist = 0.0; },
      .setting = "max_correspondence_dist", .value = "0"},
    Refusal{.label = "max_correspondence_dist_below_none",
      .apply = [](IcpParameters& p) { p.max_correspondence_dist = -0.3; },
      .setting = "max_correspondence_dist", .value = "-0.3"},
    Refusal{.label = "max_iterations_of_none",
      .apply = [](IcpParameters& p) { p.max_iterations = 0; },
      .setting = "max_iterations", .value = "0"},
    Refusal{.label = "max_iterations_below_none",
      .apply = [](IcpParameters& p) { p.max_iterations = -10; },
      .setting = "max_iterations", .value = "-10"},
    Refusal{.label = "max_linear_correction_below_none",
      .apply = [](IcpParameters& p) { p.max_linear_correction = -0.5; },
      .setting = "max_linear_correction", .value = "-0.5"},
    Refusal{.label = "orientation_neighbourhood_of_none",
      .apply = [](IcpParameters& p) { p.orientation_neighbourhood = 0; },
      .setting = "orientation_neighbourhood", .value = "0"},
    Refusal{.label = "orientation_neighbourhood_above_the_most_rays",
      .apply = [](IcpParameters& p)
        { p.orientation_neighbourhood = static_cast<int>(kMostRays) + 1; },
      .setting = "orientation_neighbourhood", .value = "10001"},
    Refusal{.label = "outliers_adaptive_mult_below_none",
      .apply = [](IcpParameters& p) { p.outliers_adaptive_mult = -2.0; },
      .setting = "outliers_adaptive_mult", .value = "-2"},
    Refusal{.label = "outliers_adaptive_order_of_none",
      .apply = [](IcpParameters& p) { p.outliers_adaptive_order = 0.0; },
      .setting = "outliers_adaptive_order", .value = "0"},
    Refusal{.label = "outliers_adaptive_order_above_the_whole",
      .apply = [](IcpParameters& p) { p.outliers_adaptive_order = 1.5; },
      .setting = "outliers_adaptive_order", .value = "1.5"},
    Refusal{.label = "outliers_maxPerc_of_none",
      .apply = [](IcpParameters& p) { p.outliers_maxPerc = 0.0; },
      .setting = "outliers_maxPerc", .value = "0"},
    Refusal{.label = "outliers_maxPerc_above_the_whole",
      .apply = [](IcpParameters& p) { p.outliers_maxPerc = 1.5; },
      .setting = "outliers_maxPerc", .value = "1.5"},
    Refusal{.label = "restart_dt_below_none",
      .apply = [](IcpParameters& p) { p.restart_dt = -1.0; },
      .setting = "restart_dt", .value = "-1"},
    Refusal{.label = "restart_dtheta_below_none",
      .apply = [](IcpParameters& p) { p.restart_dtheta = -0.1; },
      .setting = "restart_dtheta", .value = "-0.1"},
    Refusal{.label = "restart_threshold_mean_error_below_none",
      .apply = [](IcpParameters& p)
        { p.restart_threshold_mean_error = -0.01; },
      .setting = "restart_threshold_mean_error", .value = "-0.01"},
    Refusal{.label = "sigma_of_none",
      .apply = [](IcpParameters& p) { p.sigma = 0.0; },
      .setting = "sigma", .value = "0"},
    Refusal{.label = "sigma_below_none",
      .apply = [](IcpParameters& p) { p.sigma = -0.01; },
      .setting = "sigma", .value = "-0.01"},
    Refusal{.label = "use_sigma_weights_switched_on",
      .apply = [](IcpParameters& p) { p.use_sigma_weights = true; },
      .setting = "use_sigma_weights", .value = "true"}),
  refusalSuffix);

