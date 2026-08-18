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
 * @brief `cbgl::Localiser::localise`: a pose recovered from one scan with no
 * estimate to start from, and every reason this library has for producing no
 * pose at all.
 * @details
 * **The room.** One room, drawn three times: at five centimetre cells, at
 * twenty centimetre cells, and at five centimetre cells with the origin
 * translated and turned. The two cell sizes describe the same room exactly, the
 * wall band and the pillar each being a whole number of cells at either size,
 * so the only difference between those two cases is the resolution. The room is
 * a rectangle with a pillar off both of its axes of symmetry, so no half turn
 * or reflection of the truth sees what the truth sees and a recovered pose
 * cannot be right by coincidence.
 * The three grids are read from `test/fixtures`, in the same plain text the
 * harness's own generator writes, so that what is localised in is data rather
 * than a room this file draws for itself.
 * **Two kinds of measurement, and two tolerances.**
 * A scan cast from the map itself makes the case measure the search: the only
 * error left is what the search does with a scan that agrees with the map
 * exactly. The bound is two cells, being one cell for what a map can support at
 * all, since a ray stops at a cell, and another for the half cell diagonal
 * between the truth and the centre of the cell the hypothesis was drawn at. The
 * heading gets the same two cells expressed as an angle, `atan(2 * resolution /
 * mean_range)`, because a heading wrong by `dt` moves what the sensor would see
 * at range `r` sideways by `r * dt`.
 * A scan cast from the room's own polygons is the harder case, and the honest
 * one: the grid is then the only approximation in the chain. A caster answers
 * with a distance to a cell rather than to the wall's face, and the ray
 * marching caster can overshoot a face by up to `sqrt(2)` cells on top of the
 * cell itself, so the scan and the map disagree by around three cells on a ray
 * and the pose that best explains the scan can stand about that far from the
 * truth. The bound is four cells, which is that with a cell to spare, and it is
 * asserted at the five centimetre grids alone: at twenty centimetre cells four
 * cells is most of the width of this room, and a bound that loose would pass
 * on a wrong answer.
 * **Every refusal.** All six: a map with no free space, an empty hypothesis
 * set, a scan shorter than the matcher asks for, a scan carrying no usable
 * range, a refinement that refused every survivor, and a pose the metric cannot
 * grade. A refinement that refused every survivor is reached by handing in
 * hypotheses that stand inside the room's own wall, which the matcher refuses
 * one by one for standing off free space, and the count it reports per reason
 * is asserted rather than merely the refusal. The ROS 1 build published a pose
 * in that case, read from index minus one of its own list of candidates.
 * **The sixth reason, and the reference it takes to reach it.** A run also
 * refuses where the pose it would have reported cannot be graded by the metric
 * it is selected by. No occupancy grid can provoke that, for the reason
 * `AReferenceThatCastsNothingFromAFreePoseYieldsNoAnswer` sets out at length,
 * and that case reaches it through a reference that is not a grid: a localiser
 * is constructed with any `FSM::GridReference`, so the guard defends the
 * boundary of a public interface rather than an impossibility. The nearest an
 * occupancy grid comes is a map with nothing in range, which the matcher
 * refuses on its own first cast, and that is
 * `AMapWithNothingInRangeIsRefusedBeforeAnUnscorablePoseCanWin`.
 * **What the density is set to.** Fifteen hypothesis positions per square metre
 * of free space, which is what the package ships. The room is the same at both
 * cell sizes and so is its free area, so the same density draws the same number
 * of hypotheses at either resolution and the two cases differ in the map's
 * resolution and in nothing else.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <ranges>
#include <sstream>
#include <string>
#include <vector>

#include "cbgl/cbgl.hpp"
#include "fsm_localisation/occupancy_grid_map.hpp"

namespace
{
using cbgl::Error;
using cbgl::FreeSpace;
using cbgl::Localiser;
using cbgl::Parameters;
using cbgl::Point;
using cbgl::Pose;
using cbgl::Result;
using fsm_localisation::CastingMethod;
using fsm_localisation::CellCoordinates;
using fsm_localisation::CorrectionError;
using fsm_localisation::Corrector;
using fsm_localisation::OccupancyGridMap;
using fsm_localisation::WorldPoint;

const int kOccupancyThreshold = 65;

const unsigned int kRayCount = 360U;
const unsigned int kOrientations = 32U;
const unsigned int kTopK = 10U;

const double kDensity = 15.0;

/**
 * @brief The maximum range the fixture's scans were cast at, which is what a
 * ray meeting nothing reports in them.
 */
const double kRangeMax = 100.0;

/**
 * @brief The scans cast from the room's own polygons, and the poses they were
 * cast at, expressed in the room's own frame.
 */
const char* const kScansFixture = "pillar_room_scans.csv";
const char* const kTruthFixture = "pillar_room_truth.csv";

/**
 * @brief One of the grids under `test/fixtures`, by the name a failure message
 * should call it and the file it is read from.
 */
struct GridCase
{
  const char* label{""};
  const char* file{""};
};

const GridCase kEveryGrid[] = {
  GridCase{.label = "fine", .file = "pillar_room_grid_fine.csv"},
  GridCase{.label = "coarse", .file = "pillar_room_grid_coarse.csv"},
  GridCase{.label = "rotated_origin", .file = "pillar_room_grid_rotated.csv"}};

const GridCase kFineGrids[] = {
  GridCase{.label = "fine", .file = "pillar_room_grid_fine.csv"},
  GridCase{.label = "rotated_origin", .file = "pillar_room_grid_rotated.csv"}};

/**
 * @brief A grid as its fixture carries it: the message to build a map from, and
 * the yaw of the origin, which the map applies to its own cells but does not
 * report back.
 */
struct GridFixture
{
  nav_msgs::msg::OccupancyGrid message;
  double origin_yaw{0.0};
};

/**
 * @brief A reference that calls every position free and casts every ray at
 * zero.
 * @details Zero on every ray is what a caster really reports for a sensor
 * standing inside an obstacle: the distance to blocked space is nothing, so the
 * ray stops where it started. What no occupancy grid can also do is call that
 * position free, and that is the whole of the contradiction this stands for. It
 * is a reference rather than a map, which the interface a localiser is
 * constructed with allows, and it exists so that the one refusal an occupancy
 * grid cannot provoke is provoked by something.
 * @remark `max_range` is deliberately ignored. A cast of zeros is not a cast
 * that met nothing at the far end of its reach; it is a cast that met something
 * at no distance at all, which is the case the metric cannot grade because no
 * ray of it is greater than zero.
 */
class FreeEverywhereCastingNothing : public FSM::GridReference
{
public:
  bool isFree(const double, const double) const noexcept override
  {
    return true;
  }

  std::vector<double> ranges(const FSM::Pose&, const unsigned int num_rays,
    const double) const override
  {
    return std::vector<double>(num_rays, 0.0);
  }
};

class LocaliserOnAFineGrid : public testing::TestWithParam<GridCase>
{
};

class LocaliserOnEveryGrid : public testing::TestWithParam<GridCase>
{
};

std::string fixturePath(const std::string& name)
{
  return std::string(CBGL_TEST_FIXTURES) + "/" + name;
}

FreeSpace freeSpaceOfMap(const OccupancyGridMap& map)
{
  FreeSpace free_space;
  free_space.resolution = map.resolution();

  for (unsigned int column = 0U; column < map.width(); column++)
    for (unsigned int row = 0U; row < map.height(); row++)
    {
      const WorldPoint centre = map.worldFromCell(CellCoordinates{
        .column = column + 0.5, .row = row + 0.5});

      if (map.isFree(centre.x, centre.y))
        free_space.cells.push_back(Point{.x = centre.x, .y = centre.y});
    }

  return free_space;
}

std::string gridCaseSuffix(const testing::TestParamInfo<GridCase>& info)
{
  return info.param.label;
}

/**
 * @brief The bound on a heading, being a bound of `cells` cells seen at the
 * ranges the scan under test actually carries.
 */
double headingBound(const std::vector<double>& scan, const double resolution,
  const double cells)
{
  const double mean =
    std::accumulate(scan.begin(), scan.end(), 0.0) / scan.size();

  return std::atan(cells * resolution / mean);
}

/**
 * @brief How far apart two headings are, wrapped onto the half open turn so
 * that a heading just under pi and one just over minus pi are the small
 * difference apart they really are rather than a whole turn.
 */
double headingError(const double first, const double second)
{
  return std::abs(fmod(first - second + 5*M_PI, 2*M_PI) - M_PI);
}

std::shared_ptr<const OccupancyGridMap> mapOf(const GridFixture& fixture)
{
  const auto built = OccupancyGridMap::build(
    fixture.message, kOccupancyThreshold, CastingMethod::ray_marching);

  return built.has_value() ? built.value() : nullptr;
}

Parameters parametersOf(const unsigned int top_k_caers)
{
  Parameters parameters;
  parameters.dl = kDensity;
  parameters.da = kOrientations;
  parameters.top_k_caers = top_k_caers;
  parameters.num_threads = 0U;
  parameters.rng_seed = 20260818U;

  parameters.matcher.size_scan = kRayCount;
  parameters.matcher.rng_seed = 20260818U;
  parameters.matcher.num_iterations = 3U;
  parameters.matcher.xy_bound = 1.2;
  parameters.matcher.t_bound = 0.786;
  parameters.matcher.max_recoveries = 10U;
  parameters.matcher.enforce_early_gearup = true;
  parameters.matcher.enforce_translation_bound = true;
  parameters.matcher.recover_on_failed_sift = true;
  parameters.matcher.occupancy_threshold = kOccupancyThreshold;

  return parameters;
}

GridFixture readGrid(const std::string& name)
{
  std::ifstream file(fixturePath(name));
  EXPECT_TRUE(file.good()) << "cannot open " << fixturePath(name);

  unsigned int width = 0U;
  unsigned int height = 0U;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;

  file >> width >> height >> resolution >> origin_x >> origin_y >> origin_yaw;

  GridFixture fixture;
  fixture.origin_yaw = origin_yaw;

  fixture.message.info.width = width;
  fixture.message.info.height = height;
  fixture.message.info.resolution = static_cast<float>(resolution);
  fixture.message.info.origin.position.x = origin_x;
  fixture.message.info.origin.position.y = origin_y;
  fixture.message.info.origin.orientation.z = std::sin(origin_yaw / 2.0);
  fixture.message.info.origin.orientation.w = std::cos(origin_yaw / 2.0);

  fixture.message.data.resize(
    static_cast<std::size_t>(width) * height);

  for (std::size_t i = 0; i < fixture.message.data.size(); i++)
  {
    int value = 0;
    file >> value;
    fixture.message.data[i] = static_cast<std::int8_t>(value);
  }

  return fixture;
}

std::vector<std::vector<double>> readScans(const std::string& name)
{
  std::ifstream file(fixturePath(name));
  EXPECT_TRUE(file.good()) << "cannot open " << fixturePath(name);

  std::size_t count = 0;
  std::size_t size = 0;
  file >> count >> size;

  std::vector<std::vector<double>> scans(count, std::vector<double>(size));

  for (std::size_t s = 0; s < count; s++)
    for (std::size_t i = 0; i < size; i++)
      file >> scans[s][i];

  return scans;
}

/**
 * @brief The truth poses, in the room's own frame. Every grid places the same
 * room, so a pose of the room is placed in each grid's world by that grid's own
 * origin.
 */
std::vector<Pose> readTruth(const std::string& name)
{
  std::ifstream file(fixturePath(name));
  EXPECT_TRUE(file.good()) << "cannot open " << fixturePath(name);

  std::vector<Pose> poses;

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line))
  {
    if (line.empty())
      continue;

    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);

    unsigned int index = 0U;
    Pose pose;
    values >> index >> pose.x >> pose.y >> pose.t;

    poses.push_back(pose);
  }

  return poses;
}

/**
 * @brief The same grid, with every one of its cells free.
 * @details A map that describes free space and nothing standing in it. The
 * geometry and the origin stay the fixture's own, so what changes between this
 * and the room it came from is the presence of anything to cast at.
 */
GridFixture withNoObstacleAtAll(GridFixture fixture)
{
  std::ranges::fill(fixture.message.data, 0);
  return fixture;
}

/**
 * @brief A pose of the room, placed in the world the grid's origin is given in.
 * @details The position goes through the map's own cell arithmetic rather than
 * through a second copy of the origin's translation and rotation, a metre of the
 * room being a whole number of cells of either grid. The heading is the room's
 * heading turned by the origin's own yaw, which the map applies to its cells but
 * does not report.
 */
Pose worldFromRoom(const OccupancyGridMap& map, const double origin_yaw,
  const Pose& room)
{
  const WorldPoint point = map.worldFromCell(CellCoordinates{
    .column = room.x / map.resolution(), .row = room.y / map.resolution()});

  return Pose{
    .x = point.x,
    .y = point.y,
    .t = fmod(room.t + origin_yaw + 5*M_PI, 2*M_PI) - M_PI};
}

}

TEST(Localiser, AHypothesisSetHandedInIsTheSetTheSearchUses)
{
  const GridFixture fixture = readGrid("pillar_room_grid_fine.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no first pose to read and nothing for this case to localise towards.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y));

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);
  const std::vector<Pose> hypotheses{truth};

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMax, hypotheses);

  ASSERT_TRUE(result.has_value())
    << "the one hypothesis handed in is the pose the measurement was cast at, "
    << "so the search cannot fail to answer from it.";

  EXPECT_EQ(result->num_candidates, hypotheses.size() * kOrientations)
    << "one hypothesis at " << kOrientations << " orientations is "
    << kOrientations << " candidates, and no hypothesis of the localiser's own "
    << "drawing may be added to a set the caller handed in. Handing the set in "
    << "is what lets two builds be compared over one draw, and a search that "
    << "quietly dispersed as well would be comparing two different sets.";

  EXPECT_NEAR(result->pose.x, truth.x, 2.0 * map->resolution());
  EXPECT_NEAR(result->pose.y, truth.y, 2.0 * map->resolution());
}

TEST(Localiser, AMapWithNoFreeSpaceIsRefused)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  FreeSpace nowhere;
  nowhere.resolution = map->resolution();

  const Localiser localiser(parametersOf(kTopK), map, nowhere);

  const std::vector<double> measured(kRayCount, 1.0);

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMax);

  ASSERT_FALSE(result.has_value())
    << "a map with no free cell offers nowhere for the sensor to have been, so "
    << "there is no hypothesis to draw and no pose to report. Answering anyway "
    << "would put the sensor somewhere the map never called free.";

  EXPECT_EQ(result.error().reason, Error::Reason::map_has_no_free_space);

  EXPECT_TRUE(result.error().refusals.empty())
    << "the refusal counts belong to the refinement, which never ran here, so "
    << "reporting one would name a matcher that was never asked.";
}

/**
 * @brief The nearest a run comes to a winner whose own cast met nothing, and
 * the check that stops it.
 * @details A pose is unscorable when a fresh cast from it meets nothing at all,
 * and a map with nothing standing in it is where every pose is unscorable. The
 * matcher refuses such a survivor before it can ever be corrected, on its own
 * first cast, so the run ends by naming the refinement rather than by naming
 * the selection. The case asserts that boundary in both directions: that a cast
 * from inside this map really does score infinity, and that the run still
 * refuses for the reason the refinement gives rather than reporting a pose
 * nobody could grade.
 */
TEST(Localiser, AMapWithNothingInRangeIsRefusedBeforeAnUnscorablePoseCanWin)
{
  const GridFixture fixture =
    withNoObstacleAtAll(readGrid("pillar_room_grid_coarse.csv"));
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const FreeSpace free_space = freeSpaceOfMap(*map);

  ASSERT_EQ(free_space.cells.size(),
    static_cast<std::size_t>(map->width()) * map->height())
    << "every cell of this map was made free, so its free space is the whole "
    << "grid and the run cannot be refused for want of somewhere to stand.";

  const WorldPoint middle = map->worldFromCell(CellCoordinates{
    .column = map->width() / 2U + 0.5, .row = map->height() / 2U + 0.5});

  const Pose standing{.x = middle.x, .y = middle.y, .t = 0.3};

  const std::vector<double> measured(kRayCount, 1.5);

  ASSERT_TRUE(std::isinf(cbgl::caer(measured,
      map->ranges(standing, kRayCount, kRangeMax), kRangeMax)))
    << "a cast from the middle of a map with nothing in it must meet nothing "
    << "on every ray and therefore score infinity. Without that this case is "
    << "not about an unscorable pose at all.";

  const Localiser localiser(parametersOf(kTopK), map, free_space);

  const std::vector<Pose> hypotheses{standing};

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMax, hypotheses);

  ASSERT_FALSE(result.has_value())
    << "every pose on this map is unscorable, since no ray from any of them "
    << "meets anything, so there is no pose the run is entitled to report.";

  EXPECT_EQ(result.error().reason, Error::Reason::every_survivor_refused)
    << "the matcher refuses a survivor whose own first cast met nothing, so "
    << "the survivor never reaches the selection and the run is refused for "
    << "the refinement's reason rather than for the selection's. A run that "
    << "reported no scorable pose here would be naming the later of two checks "
    << "when the earlier one fired.";

  EXPECT_EQ(
    result.error().refusals.count(CorrectionError::first_cast_entirely_at_max_range),
    1u)
    << "the refusal counted must be the matcher's first cast check, so that a "
    << "host can say the map had nothing in range rather than merely that the "
    << "refinement failed.";
}

/**
 * @brief An empty hypothesis set is refused for being empty.
 * @details The three argument entry point exists so a caller can hand in a
 * hypothesis set of its own, and an empty one is the caller's doing rather than
 * the map's. Reporting it as a map with no free space is a false sentence about
 * a map with thousands of free cells, and it sends whoever reads the log to
 * look at the map instead of at the set they handed in. The same map and the
 * same scan are localised again from a set of one, so the refusal is shown to
 * be about the set rather than about anything else in the run.
 */
TEST(Localiser, AnEmptyHypothesisSetIsRefusedForBeingEmpty)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no first pose to read and nothing for this case to localise towards.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y));

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  const FreeSpace free_space = freeSpaceOfMap(*map);
  ASSERT_FALSE(free_space.cells.empty())
    << "this case needs a healthy map, or a refusal naming the map would be "
    << "the truth rather than the lie it exists to keep out.";

  const Localiser localiser(parametersOf(kTopK), map, free_space);

  const std::vector<Pose> none;

  const std::expected<Result, Error> refused =
    localiser.localise(measured, kRangeMax, none);

  ASSERT_FALSE(refused.has_value())
    << "there is no hypothesis to score, so there is nothing to rank, nothing "
    << "to refine and no pose to report.";

  EXPECT_EQ(refused.error().reason, Error::Reason::no_hypotheses_given)
    << "the map offers " << free_space.cells.size() << " free cells, so a "
    << "refusal that named the map as having none would be a false sentence "
    << "about a healthy map. What was empty is the set the caller handed in.";

  const std::vector<Pose> one{truth};

  EXPECT_TRUE(localiser.localise(measured, kRangeMax, one).has_value())
    << "the same localiser, the same map and the same scan answer from a set "
    << "of one pose, so the refusal above is about the set being empty and "
    << "about nothing else in the run.";
}

/**
 * @brief A scan carrying more rays than the matcher asks for is reduced to that
 * many, and the pose is still recovered.
 * @details Every other case here hands in exactly as many rays as `size_scan`
 * asks for, which leaves untouched the branch that interpolates a scan and
 * resamples it. A real sensor reports whatever it reports and the parameter
 * decides what the run works at, so that branch carries every run of a scan
 * finer than the setting.
 * The bound is the two cells of a scan the map itself cast, and it is the same
 * bound for the same reason: reducing a cast of twice as many rays onto this
 * many is the coarser cast itself. Ray `i` of `n` leaves a pose at
 * `i * 2*M_PI / n + pose.t - M_PI`, so ray `2 * j` of a cast of `2 * n` leaves
 * at `j * 2*M_PI / n + pose.t - M_PI`, which is ray `j` of the cast of `n`. The
 * two therefore differ by no more than the rounding of one of those
 * expressions against the other, and nothing about the search should notice
 * which of the two it was handed.
 */
/**
 * @brief A pose the metric cannot grade is refused rather than reported, even
 * where the refinement accepted it.
 * @details This is the one refusal an occupancy grid cannot provoke, and it is
 * reached here through a reference that is not one.
 * **What the case asserts, and what it does not.** It asserts that the guard
 * fires: a corrected pose whose own fresh cast shares no contributing ray with
 * the measurement is refused, and no pose is reported. It asserts nothing about
 * how a map behaves, because no occupancy grid can hold the contradiction the
 * reference below stands for. A cast of zeros on every ray is what a caster
 * reports for a sensor inside an obstacle, and a position inside an obstacle is
 * not free, so a grid that answered as this reference answers would be
 * disagreeing with itself.
 * **Why an occupancy grid cannot reach it.** The matcher closes that route
 * itself, and the translation bound has nothing to do with it: the translation
 * stage tests every step it takes with the map's own free space test,
 * `FSM::Utils::isPositionInMap`, which for a grid reference is that grid's own
 * `isFree`. A step that would leave free space or leave the grid is turned into
 * a recovery instead of being taken, and a survivor whose recoveries run out is
 * refused rather than corrected. So a corrected pose is always on free space of
 * the same map, and a cast from free space in a bounded room always meets a
 * wall. This holds with the translation bound on and with it off, the check
 * being a separate condition from the bound in the same statement.
 * **The measurement behind that claim.** Eighty attempts were run before this
 * case was written, over both fixture grids, five free cells hugging a border,
 * four measurements chosen to send the matcher outward, and the translation
 * bound both on and off: seventy seven answered and three were refused, not one
 * corrected pose landed off free space, not one rescored to infinity, and the
 * furthest any pose walked was 1.554 m. A future reader need not repeat it.
 * **Why the guard is kept, and why this case is kept with it.** A localiser is
 * constructed with a `std::shared_ptr<const FSM::GridReference>`, which is any
 * reference a caller cares to stand behind that interface and not only an
 * occupancy grid. The guard is what stops such a reference producing a
 * confident answer out of a cast that met nothing, so it defends the boundary of
 * a public interface rather than an internal impossibility. Without a case
 * reaching it, the next reader has every reason to believe it unreachable and
 * delete it.
 * **The neighbouring refusal, for contrast.** A reference that casts every ray
 * at the maximum range instead of at zero never reaches this guard: the matcher
 * refuses such a survivor on its own first cast, and the run is refused for the
 * refinement's reason.
 * `AMapWithNothingInRangeIsRefusedBeforeAnUnscorablePoseCanWin` is that case,
 * on a real grid.
 */
TEST(Localiser, AReferenceThatCastsNothingFromAFreePoseYieldsNoAnswer)
{
  const std::shared_ptr<const FSM::GridReference> reference =
    std::make_shared<const FreeEverywhereCastingNothing>();

  const Pose standing{.x = 1.0, .y = 2.0, .t = 0.3};

  std::vector<double> measured(kRayCount);
  for (unsigned int i = 0; i < kRayCount; i++)
    measured[i] = 2.0 + std::sin(3.0 * i * 2*M_PI / kRayCount);

  ASSERT_TRUE(std::ranges::all_of(measured,
      [](const double range) { return range > 0.0; }))
    << "the measurement must be one every ray of which the metric can count, "
    << "so that what cannot be graded below is the cast and not the scan.";

  const std::vector<double> cast =
    reference->ranges(standing, kRayCount, kRangeMax);

  ASSERT_TRUE(std::isinf(cbgl::caer(measured, cast, kRangeMax)))
    << "a cast of zeros shares no contributing ray with the measurement, so "
    << "the metric cannot grade it and scores it infinity. Without that this "
    << "case is not about an unscorable pose at all.";

  FreeSpace free_space;
  free_space.resolution = 0.05;
  free_space.cells.push_back(Point{.x = standing.x, .y = standing.y});

  const Localiser localiser(parametersOf(kTopK), reference, free_space);

  const std::vector<Pose> hypotheses{standing};

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMax, hypotheses);

  ASSERT_FALSE(result.has_value())
    << "the pose the run would have reported cannot be graded by the very "
    << "metric the run selects by, so there is nothing to report. Reporting it "
    << "anyway would publish a pose chosen by a comparison that never "
    << "happened.";

  EXPECT_EQ(result.error().reason, Error::Reason::no_scorable_pose);

  EXPECT_TRUE(result.error().refusals.empty())
    << "the refinement accepted every survivor it was handed here, the "
    << "reference calling every position free, so the refusal comes from the "
    << "selection rather than from the refinement and there is no refusal of "
    << "the matcher's to count. A count here would name the wrong stage.";
}

TEST(Localiser, AScanOfMoreRaysThanTheMatcherAsksForIsReducedAndStillRecovers)
{
  const GridFixture fixture = readGrid("pillar_room_grid_fine.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no first pose to read and nothing for this case to localise towards.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y));

  const Parameters parameters = parametersOf(kTopK);
  const unsigned int rays = 2U * kRayCount;

  ASSERT_GT(rays, parameters.matcher.size_scan)
    << "the scan must carry more rays than the matcher asks for, or the "
    << "reduction this case exists for never runs.";

  const std::vector<double> measured = map->ranges(truth, rays, kRangeMax);
  ASSERT_EQ(measured.size(), rays);

  const double resolution = map->resolution();
  const double cells = 2.0;

  const Localiser localiser(parameters, map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMax);

  ASSERT_TRUE(result.has_value())
    << "the scan was cast from this very map at a pose inside the room, at "
    << rays << " rays rather than at the " << parameters.matcher.size_scan
    << " the matcher works at, and the run produced no pose at all.";

  EXPECT_NEAR(result->pose.x, truth.x, cells * resolution)
    << "a scan of " << rays << " rays reduced to "
    << parameters.matcher.size_scan << " is the cast the map would have given "
    << "at " << parameters.matcher.size_scan << " rays, every other ray of the "
    << "finer cast being a ray of the coarser one, so the answer is held to "
    << "the same " << cells << " cells of " << resolution << " metres as a "
    << "scan handed in at that size. The search placed the sensor at ("
    << result->pose.x << ", " << result->pose.y << ") where the truth is ("
    << truth.x << ", " << truth.y << ").";
  EXPECT_NEAR(result->pose.y, truth.y, cells * resolution);

  EXPECT_LE(headingError(result->pose.t, truth.t),
    headingBound(measured, resolution, cells))
    << "the scan was cast at a heading of " << truth.t << " radians and the "
    << "search reported " << result->pose.t << ".";
}

/**
 * @brief The two scan refusals this library decides are the matcher's own
 * refusals under another name, and must stay that way.
 * @details A scan shorter than `size_scan`, and a scan carrying no usable
 * range, are both refused here before a hypothesis is drawn, and both are
 * refused again by `fsm_localisation::Corrector::correct` when a survivor
 * reaches it. The duplication is deliberate: each is required to be a distinct
 * reason of this library's own, and deferring to the matcher would report both
 * as every survivor having been refused, which names the refinement for a fault
 * in the measurement.
 * Two definitions of one rule can part company. This case pins them: the very
 * scan this library refuses is handed to the matcher, and the matcher has to
 * refuse it for its own equivalent reason. If the matcher ever changes what
 * either means, this goes red rather than the two quietly disagreeing.
 */
TEST(Localiser, AScanRefusedHereIsRefusedByTheMatcherForTheSameThing)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no free pose to hand the matcher.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y))
    << "the estimate handed to the matcher must stand on free space, or it "
    << "refuses for that instead and the case measures the wrong rule.";

  const Parameters parameters = parametersOf(kTopK);

  const Corrector corrector(parameters.matcher, map);
  const Localiser localiser(parameters, map, freeSpaceOfMap(*map));

  const std::vector<double> too_short(kRayCount / 36U, 1.0);

  const std::expected<Result, Error> library_on_short =
    localiser.localise(too_short, kRangeMax);

  ASSERT_FALSE(library_on_short.has_value());
  ASSERT_EQ(library_on_short.error().reason, Error::Reason::scan_too_short);

  const std::expected<fsm_localisation::CorrectionResult, CorrectionError>
    matcher_on_short = corrector.correct(too_short, truth, kRangeMax);

  ASSERT_FALSE(matcher_on_short.has_value())
    << "this library refuses a scan of " << too_short.size() << " rays as "
    << "shorter than the " << parameters.matcher.size_scan << " the matcher "
    << "asks for, and the matcher accepted it. The two no longer mean the same "
    << "thing by too short, and the earlier of the two checks is now guessing.";

  EXPECT_EQ(matcher_on_short.error(), CorrectionError::scan_too_short)
    << "the matcher refused the same scan for a different reason, so what this "
    << "library reports as a scan too short is no longer what the matcher "
    << "would have called it.";

  const std::vector<double> unusable(kRayCount, 0.0);

  const std::expected<Result, Error> library_on_unusable =
    localiser.localise(unusable, kRangeMax);

  ASSERT_FALSE(library_on_unusable.has_value());
  ASSERT_EQ(library_on_unusable.error().reason,
    Error::Reason::scan_entirely_invalid);

  const std::expected<fsm_localisation::CorrectionResult, CorrectionError>
    matcher_on_unusable = corrector.correct(unusable, truth, kRangeMax);

  ASSERT_FALSE(matcher_on_unusable.has_value())
    << "this library refuses a scan of " << unusable.size() << " rays that "
    << "carries no usable range, and the matcher accepted it. One of the two "
    << "has changed its mind about what a usable range is.";

  EXPECT_EQ(matcher_on_unusable.error(), CorrectionError::scan_entirely_invalid)
    << "the matcher refused the same scan for a different reason, so what this "
    << "library reports as a scan carrying nothing usable is no longer what "
    << "the matcher would have called it.";
}

TEST(Localiser, AScanShorterThanTheMatcherAsksForIsRefused)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::vector<double> too_short(kRayCount / 36U, 1.0);

  const std::expected<Result, Error> result =
    localiser.localise(too_short, kRangeMax);

  ASSERT_FALSE(result.has_value())
    << "the matcher is set to " << kRayCount << " rays and the scan carries "
    << too_short.size() << ", so it cannot be reduced to the size every stage "
    << "of the run works at.";

  EXPECT_EQ(result.error().reason, Error::Reason::scan_too_short);
}

TEST(Localiser, AScanWithNoUsableRangeIsRefused)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::vector<double> unusable(kRayCount, 0.0);

  const std::expected<Result, Error> result =
    localiser.localise(unusable, kRangeMax);

  ASSERT_FALSE(result.has_value())
    << "every ray of the scan is zero, which is one of the three ways a lidar "
    << "reports a ray it could not trust, so the scan carries no distance to "
    << "match against anything.";

  EXPECT_EQ(result.error().reason, Error::Reason::scan_entirely_invalid);
}

TEST(Localiser, EveryLocalisationOfTheSameScanGivesTheSameAnswer)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no first pose to read and nothing for this case to localise towards.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> first =
    localiser.localise(measured, kRangeMax);
  const std::expected<Result, Error> second =
    localiser.localise(measured, kRangeMax);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->pose.x, second->pose.x)
    << "the same scan localised twice by the same localiser, at a pinned seed, "
    << "gave " << first->pose.x << " and then " << second->pose.x
    << ". Nothing survives a call, so the second answer must repeat the first "
    << "to the last bit; a difference means state was carried across, and the "
    << "second localisation of a run then depends on the first.";
  EXPECT_EQ(first->pose.y, second->pose.y);
  EXPECT_EQ(first->pose.t, second->pose.t);

  const Localiser second_localiser(
    parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> elsewhere =
    second_localiser.localise(measured, kRangeMax);

  ASSERT_TRUE(elsewhere.has_value());

  EXPECT_EQ(elsewhere->pose.x, first->pose.x)
    << "a second localiser built from the same parameters and the same map "
    << "answered differently, so a run cannot be repeated from its own "
    << "configuration alone.";
  EXPECT_EQ(elsewhere->pose.y, first->pose.y);
  EXPECT_EQ(elsewhere->pose.t, first->pose.t);
}

TEST(Localiser, EverySurvivorRefusedByTheMatcherRefusesTheRun)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no first pose to read and nothing for this case to localise towards.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  std::vector<Pose> in_the_wall;
  for (unsigned int column = 0U; column < 4U; column++)
  {
    const WorldPoint centre = map->worldFromCell(CellCoordinates{
      .column = column + 0.5, .row = 0.5});

    ASSERT_FALSE(map->isFree(centre.x, centre.y))
      << "cell (" << column << ", 0) is meant to be part of the room's own "
      << "wall, and this case needs every hypothesis to stand off free space.";

    in_the_wall.push_back(Pose{.x = centre.x, .y = centre.y, .t = 0.0});
  }

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMax, in_the_wall);

  ASSERT_FALSE(result.has_value())
    << "every hypothesis handed in stands inside the room's wall, so the "
    << "matcher refuses every survivor and there is no corrected pose to "
    << "report. The ROS 1 build published one regardless, read from index "
    << "minus one of its own list.";

  EXPECT_EQ(result.error().reason, Error::Reason::every_survivor_refused);

  const std::map<CorrectionError, unsigned int>& refusals =
    result.error().refusals;

  ASSERT_FALSE(refusals.empty())
    << "a run refused for want of a single accepted survivor has to say how "
    << "the refinement went wrong, or a host can report only that it did.";

  EXPECT_EQ(refusals.count(CorrectionError::estimate_not_in_free_space), 1u)
    << "every hypothesis stands off free space, which is the first thing the "
    << "matcher checks, so that is the reason it must have counted.";

  unsigned int counted = 0U;
  for (const auto& refusal : refusals)
    counted += refusal.second;

  const std::size_t candidates = in_the_wall.size() * kOrientations;
  const std::size_t survivors = std::min<std::size_t>(kTopK, candidates);

  EXPECT_EQ(counted, survivors)
    << "the refinement was handed " << survivors << " survivors and the "
    << "refusals add up to " << counted << ". Every survivor was refused, so "
    << "every one of them must be counted once under the reason it was refused "
    << "for.";
}

TEST(Localiser, TheHypothesesDrawnAreTheHypothesesSearched)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no first pose to read and nothing for this case to localise towards.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::size_t drawn = localiser.disperse().size();
  ASSERT_GT(drawn, 0u);

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMax);

  ASSERT_TRUE(result.has_value());

  EXPECT_EQ(result->num_candidates, drawn * kOrientations)
    << "the draw yields " << drawn << " positions and each is scored at "
    << kOrientations << " orientations, so " << drawn * kOrientations
    << " candidates are scored. The localisation reported "
    << result->num_candidates << ". Localising without a hypothesis set of "
    << "one's own is the same thing as drawing one and then localising from "
    << "it, and at a pinned seed the two draws are the same draw.";

  EXPECT_GT(result->execution_time, 0.0)
    << "a run that took no measurable time did not happen. The execution time "
    << "is what a host reports and what the two builds are compared on.";
}

TEST_P(LocaliserOnAFineGrid, APoseIsRecoveredFromAScanTheMapNeverCast)
{
  const GridCase grid = GetParam();

  const GridFixture fixture = readGrid(grid.file);
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  const std::vector<std::vector<double>> scans = readScans(kScansFixture);

  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose.";
  ASSERT_FALSE(scans.empty())
    << "the scan fixture " << kScansFixture << " carried no scan. Two empty "
    << "files agree on their count, so the comparison below would pass while "
    << "the case measured nothing.";

  ASSERT_EQ(scans.size(), room_poses.size());
  ASSERT_EQ(scans[0].size(), kRayCount);

  const double resolution = map->resolution();
  const double cells = 4.0;

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  for (std::size_t s = 0; s < scans.size(); s++)
  {
    const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[s]);
    ASSERT_TRUE(map->isFree(truth.x, truth.y));

    const std::expected<Result, Error> result =
      localiser.localise(scans[s], kRangeMax);

    ASSERT_TRUE(result.has_value())
      << "scan " << s << " on the " << grid.label << " grid produced no pose "
      << "at all, though it was cast from inside the very room the grid draws.";

    EXPECT_NEAR(result->pose.x, truth.x, cells * resolution)
      << "scan " << s << " was cast from the room's own polygons at ("
      << truth.x << ", " << truth.y << ", " << truth.t << ") and the search "
      << "placed the sensor at (" << result->pose.x << ", " << result->pose.y
      << ", " << result->pose.t << ") on the " << grid.label << " grid. The "
      << "bound is " << cells << " cells of " << resolution << " metres: a "
      << "cell for what a grid can support at all, another " << std::sqrt(2.0)
      << " for how far past a wall's face the ray marching caster can stop, "
      << "half a cell for the centre of the cell the hypothesis was drawn at, "
      << "and a cell of slack over the three.";
    EXPECT_NEAR(result->pose.y, truth.y, cells * resolution);

    EXPECT_LE(headingError(result->pose.t, truth.t),
      headingBound(scans[s], resolution, cells))
      << "scan " << s << " was cast at a heading of " << truth.t
      << " radians and the search reported " << result->pose.t << ". A heading "
      << "wrong by dt moves what the sensor would see at range r sideways by "
      << "r * dt, so the bound is " << cells << " cells seen at the mean range "
      << "this scan carries.";
  }
}

TEST_P(LocaliserOnEveryGrid, APoseIsRecoveredFromAScanTheMapItselfCast)
{
  const GridCase grid = GetParam();

  const GridFixture fixture = readGrid(grid.file);
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so the loop "
    << "below would assert nothing at all.";

  const double resolution = map->resolution();
  const double cells = 2.0;

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  for (std::size_t s = 0; s < room_poses.size(); s++)
  {
    const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[s]);
    ASSERT_TRUE(map->isFree(truth.x, truth.y))
      << "truth pose " << s << " of the room falls outside the free space of "
      << "the " << grid.label << " grid, so the case is asking for a pose the "
      << "search is not allowed to reach.";

    const std::vector<double> measured =
      map->ranges(truth, kRayCount, kRangeMax);

    const std::expected<Result, Error> result =
      localiser.localise(measured, kRangeMax);

    ASSERT_TRUE(result.has_value())
      << "the measurement was cast from this very map at truth pose " << s
      << ", so a candidate that agrees with it exactly exists wherever the "
      << "draw reaches that cell, and the run produced no pose at all.";

    ASSERT_FALSE(result->survivors.empty())
      << "a successful run refined at least one survivor, so it must report at "
      << "least one.";

    EXPECT_LE(result->survivors.size(), kTopK)
      << "the sift was asked for " << kTopK << " survivors and reported "
      << result->survivors.size() << ".";

    EXPECT_NEAR(result->pose.x, truth.x, cells * resolution)
      << "truth pose " << s << " on the " << grid.label << " grid stands at ("
      << truth.x << ", " << truth.y << ", " << truth.t << ") and the search "
      << "placed the sensor at (" << result->pose.x << ", " << result->pose.y
      << ", " << result->pose.t << "). The bound is " << cells << " cells of "
      << resolution << " metres: one cell for what a map can support at all, "
      << "since a ray stops at a cell, and one for the half cell diagonal "
      << "between the truth and the centre of the cell a hypothesis is drawn "
      << "at. A pose metres away is the wrong part of the room, which is what "
      << "global localisation exists to get right.";
    EXPECT_NEAR(result->pose.y, truth.y, cells * resolution);

    EXPECT_LE(headingError(result->pose.t, truth.t),
      headingBound(measured, resolution, cells))
      << "truth pose " << s << " faces " << truth.t << " radians and the "
      << "search reported " << result->pose.t << ". The bound is " << cells
      << " cells seen at the mean range of this scan, a heading wrong by dt "
      << "moving what the sensor would see at range r sideways by r * dt.";
  }
}

INSTANTIATE_TEST_SUITE_P(
  AtFiveCentimetreCells, LocaliserOnAFineGrid,
  testing::ValuesIn(kFineGrids), gridCaseSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverEveryGrid, LocaliserOnEveryGrid,
  testing::ValuesIn(kEveryGrid), gridCaseSuffix);
