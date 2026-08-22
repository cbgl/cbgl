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
 * **The matcher's five refusals, as a run through this library meets them.** A
 * survivor standing off free space is reached by hypotheses inside the wall, a
 * first cast that met nothing by a map with nothing standing in it, and a spent
 * recovery budget by a translation bound smaller than the cell the refinement
 * has to cross. The remaining two, a scan too short and a scan carrying nothing
 * usable, cannot reach the matcher through this library at all, because the
 * library decides both before a hypothesis is drawn. They are pinned against
 * the matcher's own copies of the same two rules by `ScanRefusalSeam` and, from
 * the other side of the boundary, by
 * `AScanTheSeamHasNoBusinessRefusingIsRefusedByNeitherLayer`.
 * **A measurement that carries no information.** A scan reading the sensor's
 * maximum range on every ray is refused by nothing and answered coherently,
 * while saying nothing at all about where the sensor was. What the metric ranks
 * by in that case is how much space a pose can see, which is pinned so that a
 * later decision to treat it differently is a decision rather than an accident.
 * **What the density is set to.** Fifteen hypothesis positions per square metre
 * of free space, which is what the package ships. The room is the same at both
 * cell sizes and so is its free area, so the same density draws the same number
 * of hypotheses at either resolution and the two cases differ in the map's
 * resolution and in nothing else.
 * **The other refinement.** Every case above refines a survivor by the matcher,
 * which is what a parameter set carrying no settings for the iterative closest
 * point solver asks for. Four cases carry a set for it instead, which is the
 * whole of how the second refinement is chosen, and they are what walks the
 * half of a run that lies downstream of the sift a second time: a pose
 * recovered from a hypothesis set the localiser drew, a hypothesis put three
 * cells along and two across from the truth and walked back onto it, the same
 * scan localised three times over for an answer that repeats to the last bit,
 * and a run whose every survivor that refinement refused. That last one is
 * where the two counts a result carries are told apart, a refusal of one
 * refinement being no fact at all about the other.
 * @remark Nothing here refines from two threads. The solver the second
 * refinement is built on keeps its context stack, its counters and every matrix
 * it has allocated in process globals and guards none of it, so two refinements
 * running at once can be handed each other's matrices. Its own header says so
 * of any two calls anywhere in the process rather than only of two on one
 * refiner.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <fstream>
#include <limits>
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
using cbgl::IcpError;
using cbgl::IcpParameters;
using cbgl::Localiser;
using cbgl::Parameters;
using cbgl::Point;
using cbgl::Pose;
using cbgl::Result;
using cbgl::ScoredPose;
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
 * @brief The minimum range the fixture's scans report from, nothing in them
 * being shorter than nought.
 * @details Handed to every localisation below. The cases that refine with the
 * matcher do not read it: the sift, the matcher and the selection each judge a
 * reading by the matcher's own test of it, which knows no lower bound. The
 * cases that refine by iterative closest point do read it, that refinement
 * marking a reading under it unusable rather than treating it as a distance.
 */
const double kRangeMin = 0.0;

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
 * @brief One scan both this library and the matcher have to refuse, and the
 * reason each of them has to give for it.
 * @details `rays` rays, every one of them `range`. The four ways a lidar reports
 * a ray it could not trust are one case each, since each is a separate encoding
 * of the same thing and either layer could stop recognising one of them without
 * the other noticing, and the length boundary is one more.
 */
struct SeamCase
{
  const char* label{""};
  double range{0.0};
  std::size_t rays{0};
  Error::Reason library{Error::Reason::scan_entirely_invalid};
  CorrectionError matcher{CorrectionError::scan_entirely_invalid};
};

const SeamCase kSeamCases[] = {
  SeamCase{.label = "every_ray_zero", .range = 0.0, .rays = kRayCount,
    .library = Error::Reason::scan_entirely_invalid,
    .matcher = CorrectionError::scan_entirely_invalid},
  SeamCase{.label = "every_ray_infinite",
    .range = std::numeric_limits<double>::infinity(), .rays = kRayCount,
    .library = Error::Reason::scan_entirely_invalid,
    .matcher = CorrectionError::scan_entirely_invalid},
  SeamCase{.label = "every_ray_not_a_number",
    .range = std::numeric_limits<double>::quiet_NaN(), .rays = kRayCount,
    .library = Error::Reason::scan_entirely_invalid,
    .matcher = CorrectionError::scan_entirely_invalid},
  SeamCase{.label = "every_ray_negative", .range = -1.0, .rays = kRayCount,
    .library = Error::Reason::scan_entirely_invalid,
    .matcher = CorrectionError::scan_entirely_invalid},
  SeamCase{.label = "one_ray_short_of_the_size_asked_for", .range = 1.0,
    .rays = kRayCount - 1U,
    .library = Error::Reason::scan_too_short,
    .matcher = CorrectionError::scan_too_short}};

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

class ScanRefusalSeam : public testing::TestWithParam<SeamCase>
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

/**
 * @brief The same parameters, refining every survivor by iterative closest
 * point instead of by the matcher.
 * @details The settings are that refinement's own defaults, which are what the
 * node declares where it is chosen. Filling the field is the whole of how a
 * refinement is chosen, so this differs from `parametersOf` in one member and
 * in nothing else, and a case run through both compares two refinements of one
 * search rather than two searches.
 */
Parameters parametersRefiningByIcp(const unsigned int top_k_caers)
{
  Parameters parameters = parametersOf(top_k_caers);
  parameters.icp = IcpParameters{};

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

std::string seamCaseSuffix(const testing::TestParamInfo<SeamCase>& info)
{
  return info.param.label;
}

/**
 * @brief How much range a cast carries in total, which is the only thing a
 * measurement entirely at its maximum range can rank a candidate by.
 */
double totalOf(const std::vector<double>& ranges)
{
  return std::accumulate(ranges.begin(), ranges.end(), 0.0);
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

/**
 * @brief The other refinement walks a hypothesis standing off the truth back
 * onto it.
 * @details One hypothesis is handed in, three cells along and two cells across
 * from where the measurement was cast, so it stands `sqrt(3 * 3 + 2 * 2)`
 * cells away, which is a little over three and a half of them. The answer is
 * held to the two cells every other case here holds one to, and the case
 * asserts before it starts that the two cells are fewer than the three and a
 * half: a refinement that handed its estimate straight back would otherwise
 * pass this.
 * The heading of the hypothesis is the truth's, which decides nothing. The
 * search scores that one position at `da` orientations whatever heading it was
 * handed, so the candidate the refinement is given carries a heading of the
 * search's own choosing rather than the one written here.
 * @remark The refinement runs one survivor at a time and is asked to here as
 * well, the solver underneath keeping its whole working state in process
 * globals and guarding none of it. Nothing in this file refines from a second
 * thread.
 */
TEST(Localiser, AHypothesisDisplacedFromTheTruthIsWalkedBackByTheIcpRefinement)
{
  const GridFixture fixture = readGrid("pillar_room_grid_fine.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no pose to cast the measurement from.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y));

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  const double cell = map->resolution();

  const Pose displaced{
    .x = truth.x + 3.0 * cell, .y = truth.y - 2.0 * cell, .t = truth.t};

  ASSERT_TRUE(map->isFree(displaced.x, displaced.y))
    << "the hypothesis has to stand somewhere the map calls free, or what the "
    << "refinement is handed is a cast taken from inside a wall and the case "
    << "is about that instead.";

  const double displacement =
    std::hypot(displaced.x - truth.x, displaced.y - truth.y);

  const double bound = 2.0 * cell;

  ASSERT_GT(displacement, bound)
    << "the hypothesis stands " << displacement << " m from the truth and the "
    << "answer is held to " << bound << " m. The first has to be the larger "
    << "of the two, or a refinement that returned the estimate it was given "
    << "would satisfy this case without refining anything.";

  const Localiser localiser(
    parametersRefiningByIcp(kTopK), map, freeSpaceOfMap(*map));

  const std::vector<Pose> hypotheses{displaced};

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);

  ASSERT_TRUE(result.has_value())
    << "the one hypothesis handed in stands three cells along and two across "
    << "from where the measurement was cast, which is well inside every bound "
    << "the refinement's own settings allow a correction to reach.";

  EXPECT_EQ(result->num_candidates, hypotheses.size() * kOrientations)
    << "one hypothesis at " << kOrientations << " orientations is "
    << kOrientations << " candidates, and choosing the other refinement "
    << "changes what a survivor is refined by rather than what is searched.";

  EXPECT_TRUE(result->refusals.empty())
    << "the matcher was never asked to refine anything here, so a count under "
    << "its reasons would name a stage that did not run.";

  EXPECT_NEAR(result->pose.x, truth.x, bound);
  EXPECT_NEAR(result->pose.y, truth.y, bound);

  EXPECT_LT(headingError(result->pose.t, truth.t),
    headingBound(measured, cell, 2.0))
    << "a heading wrong by dt moves what the sensor would see at range r "
    << "sideways by r * dt, so the same two cells as an angle is what the "
    << "heading is held to.";
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
    localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);

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
    localiser.localise(measured, kRangeMin, kRangeMax);

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
    localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);

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
 * @brief A measurement carrying nothing but its own maximum range is answered,
 * and what the answer is ranked by is how much space a pose can see.
 * @details
 * A scan every ray of which reads the sensor's maximum range is a scan that met
 * nothing at all. Nothing refuses it: the maximum range is a valid reading per
 * `fsm_localisation::isValidRange`, so the scan is neither unusable nor short,
 * and a cast taken inside a bounded room is never entirely at that maximum
 * either, so the metric grades every candidate rather than ranking it behind
 * everything that scored.
 * What the metric then computes, ray by ray, is `range_max - cast_i`, since a
 * cast inside the room is shorter than the maximum everywhere. Summed, that is
 * `n * range_max - sum(cast_i)`. The measurement enters the expression only
 * through its own maximum, which every candidate shares, so the ranking is by
 * the total of the cast and by nothing else: the position that sees the most
 * space wins, wherever the sensor actually stood. Rotating a cast does not
 * change its total, so all `da` orientations of that position tie, and the
 * survivors are `top_k_caers` of them chosen by nothing.
 * The run is therefore answered, coherently, from a measurement that says
 * nothing whatever about where the sensor was. That is not a defect: no
 * requirement asks this library to notice a measurement carrying no
 * information, and the pose it reports does stand on free space of the map. It
 * is pinned here so that a later decision to refuse such a run, or to report it
 * as anything other than an ordinary answer, is a decision somebody made rather
 * than a change nobody noticed.
 * @remark The hypothesis set is handed in as every free cell of the map rather
 * than drawn, so that the cell the ranking has to pick is a cell the search was
 * certainly offered, and so that the winner can be computed here from the casts
 * themselves.
 */
TEST(Localiser, AMeasurementEntirelyAtItsMaximumRangeIsRankedByTheSpaceSeen)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const FreeSpace free_space = freeSpaceOfMap(*map);
  ASSERT_FALSE(free_space.cells.empty());

  const std::vector<double> measured(kRayCount, kRangeMax);

  ASSERT_TRUE(std::ranges::all_of(measured, fsm_localisation::isValidRange))
    << "every ray of this measurement reads the maximum range, which is a "
    << "usable reading rather than one of the three ways a lidar reports a ray "
    << "it could not trust. Without that the case would be about an unusable "
    << "scan and not about an uninformative one.";

  std::vector<Pose> hypotheses;
  for (const Point& cell : free_space.cells)
    hypotheses.push_back(Pose{.x = cell.x, .y = cell.y, .t = 0.0});

  const double tolerance =
    1024.0 * std::numeric_limits<double>::epsilon() * kRayCount * kRangeMax;

  Pose best_seen{};
  double most_space = -1.0;
  double next_most_space = -1.0;

  for (const Pose& hypothesis : hypotheses)
  {
    const std::vector<double> cast =
      map->ranges(hypothesis, kRayCount, kRangeMax);

    ASSERT_TRUE(std::ranges::all_of(cast, [](const double range)
        { return range > 0.0 && range < kRangeMax; }))
      << "a cast from the free cell at (" << hypothesis.x << ", "
      << hypothesis.y << ") carries a ray at nothing or a ray at the maximum "
      << "range, so the score below is not the sum of range_max - cast_i that "
      << "this case computes it as.";

    const double space = totalOf(cast);

    if (space > most_space)
    {
      next_most_space = most_space;
      most_space = space;
      best_seen = hypothesis;
    }
    else if (space > next_most_space)
      next_most_space = space;
  }

  ASSERT_GT(most_space - next_most_space, tolerance)
    << "two free cells of this map see the same total to within " << tolerance
    << " metres summed over " << kRayCount << " rays, so which of them a "
    << "ranking by that total prefers is a coin toss and this case has no "
    << "single cell to hold the survivors to.";

  const Localiser localiser(parametersOf(kTopK), map, free_space);

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);

  ASSERT_TRUE(result.has_value())
    << "a measurement entirely at its own maximum range is answered today, "
    << "rather than refused. If that has deliberately changed, this case is "
    << "what has to be rewritten to say so.";

  EXPECT_TRUE(map->isFree(result->pose.x, result->pose.y))
    << "the answer stands at (" << result->pose.x << ", " << result->pose.y
    << "), which this map does not call free. An uninformative answer is still "
    << "expected to be a coherent one.";

  ASSERT_EQ(result->survivors.size(), kTopK);

  for (std::size_t s = 0; s < result->survivors.size(); s++)
  {
    const ScoredPose& survivor = result->survivors[s];

    EXPECT_EQ(survivor.pose.x, best_seen.x)
      << "survivor " << s << " stands at (" << survivor.pose.x << ", "
      << survivor.pose.y << ") where the free cell that sees the most space is "
      << "(" << best_seen.x << ", " << best_seen.y << "), which sees "
      << most_space << " metres of range against " << next_most_space
      << " for the next best. A measurement entirely at the maximum range "
      << "scores a candidate by the total of its cast alone, so every survivor "
      << "has to stand at that one cell.";
    EXPECT_EQ(survivor.pose.y, best_seen.y);

    const std::vector<double> cast =
      map->ranges(survivor.pose, kRayCount, kRangeMax);

    const double closed_form = kRayCount * kRangeMax - totalOf(cast);

    EXPECT_NEAR(survivor.score, closed_form, tolerance)
      << "survivor " << s << " scored " << survivor.score << " where summing "
      << "range_max - cast_i over the " << kRayCount << " rays of a fresh cast "
      << "at the same pose gives " << closed_form << ". The measurement is the "
      << "maximum range on every ray, so the two are the same sum in two "
      << "orderings and may differ only by the rounding of that.";

    EXPECT_NEAR(survivor.score, result->survivors.front().score, tolerance)
      << "survivor " << s << " scored " << survivor.score << " and the first "
      << "survivor scored " << result->survivors.front().score << ". The "
      << kTopK << " survivors are " << kTopK << " orientations of one position, "
      << "and rotating a cast does not change its total, so the sift cannot "
      << "prefer any of them to any other. That the answer is one of them is "
      << "arbitrary, and that is the whole of what this case records.";
  }
}

/**
 * @brief An empty hypothesis set is refused for being empty.
 * @details The four argument entry point exists so a caller can hand in a
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
    localiser.localise(measured, kRangeMin, kRangeMax, none);

  ASSERT_FALSE(refused.has_value())
    << "there is no hypothesis to score, so there is nothing to rank, nothing "
    << "to refine and no pose to report.";

  EXPECT_EQ(refused.error().reason, Error::Reason::no_hypotheses_given)
    << "the map offers " << free_space.cells.size() << " free cells, so a "
    << "refusal that named the map as having none would be a false sentence "
    << "about a healthy map. What was empty is the set the caller handed in.";

  const std::vector<Pose> one{truth};

  EXPECT_TRUE(
    localiser.localise(measured, kRangeMin, kRangeMax, one).has_value())
    << "the same localiser, the same map and the same scan answer from a set "
    << "of one pose, so the refusal above is about the set being empty and "
    << "about nothing else in the run.";
}

/**
 * @brief A pose is recovered through the other refinement, from a hypothesis
 * set the localiser drew itself.
 * @details The same room, the same measurement and the same two cell bound as
 * the case that recovers a pose from a scan the map itself cast. What differs
 * is one member of the parameter set, which sends every survivor to the
 * iterative closest point refinement instead of to the matcher, and the whole
 * of the run downstream of the sift therefore travels a second path that
 * nothing else here walks.
 * The bound is two cells for the reason it is two cells everywhere else in
 * this file: one for what a map of this cell size can support at all, a ray
 * stopping at a cell, and another for the half cell diagonal between the truth
 * and the centre of the cell the hypothesis was drawn at.
 * @remark What the matcher's refusal count says is asserted to be nothing. One
 * refinement runs per localisation and the other one did not run, so a count
 * under the matcher's reasons here would be a fact about a stage that was
 * never asked.
 */
TEST(Localiser, APoseIsRecoveredByTheIcpRefinementFromTheLocalisersOwnDraw)
{
  const GridFixture fixture = readGrid("pillar_room_grid_fine.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no pose to cast the measurement from.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y));

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  const Localiser localiser(
    parametersRefiningByIcp(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMin, kRangeMax);

  ASSERT_TRUE(result.has_value())
    << "the measurement is the cast the map itself gives at a pose the map "
    << "calls free, and the search draws its own hypotheses over that map at a "
    << "pinned seed. A run of it that produces no pose at all has lost the "
    << "answer somewhere between the sift and the selection.";

  EXPECT_TRUE(result->refusals.empty())
    << "the matcher refined nothing here and a count under its reasons would "
    << "name a stage that never ran. What this run refused, if it refused "
    << "anything, belongs under the other refinement's own reasons.";

  const double bound = 2.0 * map->resolution();

  EXPECT_NEAR(result->pose.x, truth.x, bound);
  EXPECT_NEAR(result->pose.y, truth.y, bound);

  EXPECT_LT(headingError(result->pose.t, truth.t),
    headingBound(measured, map->resolution(), 2.0))
    << "the same two cells as an angle, a heading wrong by dt moving what the "
    << "sensor would see at range r sideways by r * dt.";
}

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
    localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);

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
 * @brief A scan carrying no rays at all is refused, and refused for its length.
 * @details The ROS 1 build read the first range of a scan before establishing
 * that the scan carried one, so a message with an empty range vector read
 * before the start of it. Nothing here indexes a scan: the node copies the
 * whole vector and the library asks only for its size. This case pins the
 * degenerate end of that, an empty scan being simply the shortest scan there
 * is, so it is refused by the same rule and under the same reason as any other
 * scan too short to reduce.
 * @remark The neighbouring case refuses a scan of a tenth of the rays the
 * matcher asks for. That one shows the rule works; this one shows it does not
 * stop working at zero, which is where an index would have been read.
 */
TEST(Localiser, AScanCarryingNoRaysAtAllIsRefused)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::vector<double> no_rays;

  const std::expected<Result, Error> result =
    localiser.localise(no_rays, kRangeMin, kRangeMax);

  ASSERT_FALSE(result.has_value())
    << "the matcher is set to " << kRayCount << " rays and the scan carries "
    << "none, so there is nothing to reduce and nothing to score against.";

  EXPECT_EQ(result.error().reason, Error::Reason::scan_too_short)
    << "an empty scan is the shortest scan there is, so it is refused for its "
    << "length rather than for carrying no usable range: the second would "
    << "suggest the rays were there and unusable.";
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
    localiser.localise(measured, kRangeMin, kRangeMax);

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

TEST(Localiser, AScanShorterThanTheMatcherAsksForIsRefused)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::vector<double> too_short(kRayCount / 36U, 1.0);

  const std::expected<Result, Error> result =
    localiser.localise(too_short, kRangeMin, kRangeMax);

  ASSERT_FALSE(result.has_value())
    << "the matcher is set to " << kRayCount << " rays and the scan carries "
    << too_short.size() << ", so it cannot be reduced to the size every stage "
    << "of the run works at.";

  EXPECT_EQ(result.error().reason, Error::Reason::scan_too_short);
}

/**
 * @brief The other side of the seam: a scan neither layer is entitled to refuse.
 * @details The two rules the seam is made of are each a boundary, and a copy of
 * a rule can be wrong by refusing too much as easily as by refusing too little.
 * Exactly `size_scan` rays is the shortest scan that is not too short, and one
 * usable ray is the least a scan can carry and still carry something, so this is
 * the scan that stands one step inside both rules at once. Neither layer may
 * call it short and neither may call it unusable, whatever else either of them
 * then makes of it.
 * @remark What the run goes on to report is not asserted. A scan of one usable
 * ray is interpolated into a scan of one range repeated, which describes a
 * circular room and not this one, so where the search puts the sensor is not
 * something this case has any business pinning.
 */
TEST(Localiser, AScanTheSeamHasNoBusinessRefusingIsRefusedByNeitherLayer)
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

  std::vector<double> ranges(parameters.matcher.size_scan, 0.0);
  ranges[ranges.size() / 2] = 1.0;

  ASSERT_EQ(std::ranges::count_if(ranges, fsm_localisation::isValidRange), 1)
    << "the scan must carry exactly one usable range, that being one more than "
    << "the entirely unusable scan the seam does refuse.";

  const Corrector corrector(parameters.matcher, map);
  const Localiser localiser(parameters, map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> library =
    localiser.localise(ranges, kRangeMin, kRangeMax);

  const bool library_called_it_short = !library.has_value() &&
    library.error().reason == Error::Reason::scan_too_short;

  EXPECT_FALSE(library_called_it_short)
    << "the scan carries " << ranges.size() << " rays, which is exactly the "
    << parameters.matcher.size_scan << " the matcher asks for, and this library "
    << "refused it as shorter than that. The rule is fewer than, not fewer than "
    << "or equal to.";

  const bool library_called_it_unusable = !library.has_value() &&
    library.error().reason == Error::Reason::scan_entirely_invalid;

  EXPECT_FALSE(library_called_it_unusable)
    << "one ray of the scan carries a usable range, so the scan does not carry "
    << "nothing usable, and this library refused it as though it did. A copy of "
    << "a rule that refuses one step too much throws away a scan a lidar can "
    << "really produce.";

  const std::expected<fsm_localisation::CorrectionResult, CorrectionError>
    matcher = corrector.correct(ranges, truth, kRangeMax);

  const bool matcher_called_it_short = !matcher.has_value() &&
    matcher.error() == CorrectionError::scan_too_short;

  EXPECT_FALSE(matcher_called_it_short)
    << "the matcher refused a scan of exactly the " << parameters.matcher.size_scan
    << " rays it asks for as too short, so the two layers no longer agree on "
    << "where the boundary of that rule is.";

  const bool matcher_called_it_unusable = !matcher.has_value() &&
    matcher.error() == CorrectionError::scan_entirely_invalid;

  EXPECT_FALSE(matcher_called_it_unusable)
    << "the matcher refused a scan carrying one usable range as carrying none, "
    << "so the two layers no longer agree on what a usable range is.";
}

TEST(Localiser, AScanWithNoUsableRangeIsRefused)
{
  const GridFixture fixture = readGrid("pillar_room_grid_coarse.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const Localiser localiser(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::vector<double> unusable(kRayCount, 0.0);

  const std::expected<Result, Error> result =
    localiser.localise(unusable, kRangeMin, kRangeMax);

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
    localiser.localise(measured, kRangeMin, kRangeMax);
  const std::expected<Result, Error> second =
    localiser.localise(measured, kRangeMin, kRangeMax);

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
    second_localiser.localise(measured, kRangeMin, kRangeMax);

  ASSERT_TRUE(elsewhere.has_value());

  EXPECT_EQ(elsewhere->pose.x, first->pose.x)
    << "a second localiser built from the same parameters and the same map "
    << "answered differently, so a run cannot be repeated from its own "
    << "configuration alone.";
  EXPECT_EQ(elsewhere->pose.y, first->pose.y);
  EXPECT_EQ(elsewhere->pose.t, first->pose.t);
}

/**
 * @brief The other refinement repeats itself, over a call and over a localiser.
 * @details The solver underneath keeps its context stack, the counters that go
 * with it and every matrix it has ever allocated in process globals, and reuses
 * them from one call to the next. Its own header says none of that shows in the
 * answers. This is what holds it to that from the level above: ten survivors
 * are refined by it in the first run and ten more in the second, so the second
 * run's calls all follow calls that have already been made, and the two answers
 * are required to agree to the last bit rather than to a tolerance.
 * A third run is made through a localiser of its own, built from the same
 * parameters over the same map, so that what is being repeated is the
 * configuration rather than one object's history.
 * @remark The neighbouring case asks the same of the matcher. The two are kept
 * apart because a refinement carrying process wide state is a different claim
 * from one carrying none, and a single case over both would say which of them
 * broke it only by which line failed.
 */
TEST(Localiser, EveryLocalisationRefinedByIcpOfTheSameScanGivesTheSameAnswer)
{
  const GridFixture fixture = readGrid("pillar_room_grid_fine.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no pose to cast the measurement from.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y));

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  const std::vector<Pose> hypotheses{truth};

  const Localiser localiser(
    parametersRefiningByIcp(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> first =
    localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);
  const std::expected<Result, Error> second =
    localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->pose.x, second->pose.x)
    << "the same scan localised twice by the same localiser gave "
    << first->pose.x << " and then " << second->pose.x << ". The solver "
    << "underneath carries every matrix it has allocated from one call to the "
    << "next, and a difference here is that state reaching an answer.";
  EXPECT_EQ(first->pose.y, second->pose.y);
  EXPECT_EQ(first->pose.t, second->pose.t);

  const Localiser second_localiser(
    parametersRefiningByIcp(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> elsewhere =
    second_localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);

  ASSERT_TRUE(elsewhere.has_value());

  EXPECT_EQ(elsewhere->pose.x, first->pose.x)
    << "a second localiser built from the same parameters and the same map "
    << "answered differently, so a run through this refinement cannot be "
    << "repeated from its own configuration alone.";
  EXPECT_EQ(elsewhere->pose.y, first->pose.y);
  EXPECT_EQ(elsewhere->pose.t, first->pose.t);
}

/**
 * @brief A run whose every survivor the other refinement refused is refused,
 * and the count is kept under that refinement's own reasons.
 * @details The reference casts nothing from anywhere and calls every position
 * free, which is what leaves the measurement perfectly good and the cast it is
 * to be matched against carrying not one usable point. The solver refuses such
 * a pair on its own account, which is the one refusal of the six that is the
 * solver's rather than the library's, and every survivor earns it.
 * **What the two counts are for.** The two refinements refuse for reasons that
 * do not correspond, so a run keeps a count per refinement rather than one
 * count between them. One refinement runs per localisation, so at most one of
 * the two can carry anything, and this case asserts both halves of that: ten
 * under the reason the refinement that ran gave, and nothing at all under the
 * matcher's, which was never asked. A count put in the wrong map would read as
 * a matcher that refused ten survivors of a run it took no part in.
 * **Why ten.** One hypothesis at `da` orientations is thirty two candidates and
 * `top_k_caers` of them are refined, which is ten, so ten survivors reach the
 * refinement and every one of them is refused once.
 */
TEST(Localiser, EverySurvivorRefusedByTheIcpRefinementIsCountedUnderItsReasons)
{
  const std::shared_ptr<const FSM::GridReference> reference =
    std::make_shared<const FreeEverywhereCastingNothing>();

  const Pose standing{.x = 1.0, .y = 2.0, .t = 0.3};

  std::vector<double> measured(kRayCount);
  for (unsigned int i = 0; i < kRayCount; i++)
    measured[i] = 2.0 + std::sin(3.0 * i * 2*M_PI / kRayCount);

  ASSERT_TRUE(std::ranges::all_of(measured,
      [](const double range) { return range > 0.0; }))
    << "the measurement must be one the refinement can carry every ray of into "
    << "a match, so that what it refuses is the cast and not the scan.";

  FreeSpace free_space;
  free_space.resolution = 0.05;
  free_space.cells.push_back(Point{.x = standing.x, .y = standing.y});

  const Localiser localiser(
    parametersRefiningByIcp(kTopK), reference, free_space);

  const std::vector<Pose> hypotheses{standing};

  const std::expected<Result, Error> result =
    localiser.localise(measured, kRangeMin, kRangeMax, hypotheses);

  ASSERT_FALSE(result.has_value())
    << "every survivor was refused by the refinement, so not one corrected "
    << "pose came back and there is nothing to select between.";

  EXPECT_EQ(result.error().reason, Error::Reason::every_survivor_refused);

  EXPECT_TRUE(result.error().refusals.empty())
    << "the matcher took no part in this run and its own count of refusals "
    << "must therefore be empty. A refusal of one refinement counted under the "
    << "other's reasons is not the same fact written differently.";

  const std::map<IcpError, unsigned int>& icp_refusals =
    result.error().icp_refusals;

  ASSERT_EQ(icp_refusals.size(), 1u)
    << "every survivor was refused for the same thing, a reference with no "
    << "usable point in it, so exactly one reason may be counted.";

  ASSERT_EQ(icp_refusals.count(IcpError::matching_failed), 1u)
    << "the scan handed in is good and its length is one the solver takes, so "
    << "the five refusals the library settles before the solver is reached are "
    << "all passed and what is left is the solver's own.";

  const std::size_t candidates = hypotheses.size() * kOrientations;
  const std::size_t survivors = std::min<std::size_t>(kTopK, candidates);

  EXPECT_EQ(icp_refusals.at(IcpError::matching_failed), survivors)
    << "the refinement was handed " << survivors << " survivors, being the "
    << "smaller of " << kTopK << " and the " << candidates << " candidates "
    << "one hypothesis at " << kOrientations << " orientations yields, and "
    << "every one of them was refused, so every one of them must be counted.";
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
    localiser.localise(measured, kRangeMin, kRangeMax, in_the_wall);

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
    localiser.localise(measured, kRangeMin, kRangeMax);

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

/**
 * @brief The scan's near bound reaches the other refinement and reaches
 * nothing else.
 * @details The two refinements read a different set of the arguments a
 * localisation is given, and this is the one argument they disagree about. The
 * sift, the matcher and the selection each judge a reading by the matcher's own
 * test of it, which knows no lower bound at all, so a run refined by the
 * matcher gives the same answer whatever near bound it is handed. The other
 * refinement marks a reading under that bound unusable rather than treating it
 * as a distance, and a bound above every reading of a scan therefore leaves it
 * nothing to match.
 * The bound is not written down. It is the longest reading the scan carries
 * plus a metre, computed in the case from the scan itself, so it is above every
 * reading by construction on any room and at any resolution.
 * **What this case is really for.** Every other case here that refines by
 * iterative closest point would also pass were the refinement chosen wrongly
 * and the matcher run in its place, both refinements recovering the same pose
 * from the same room. This one cannot: the same map, the same measurement, the
 * same hypothesis and the same near bound go to both, and one answers while the
 * other refuses. So it is what holds the choice of refinement to the one field
 * that makes it, rather than only holding what happens afterwards.
 * @remark The refusal is `scan_entirely_invalid` rather than the solver's own,
 * and it is settled before the map is cast from at all. The reason matters:
 * a host reading it is told the scan it sent was unusable at the bound it
 * declared, which is a thing to correct, rather than that a match failed, which
 * is not.
 */
TEST(Localiser, TheNearBoundIsReadByTheIcpRefinementAndByNoOtherStage)
{
  const GridFixture fixture = readGrid("pillar_room_grid_fine.csv");
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no pose to cast the measurement from.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y));

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  const double furthest = *std::ranges::max_element(measured);
  const double above_every_reading = furthest + 1.0;

  ASSERT_LT(furthest, above_every_reading)
    << "the near bound this case hands in has to stand above the longest "
    << "reading the scan carries, or the refinement is left something to match "
    << "and the case is about nothing.";

  const std::vector<Pose> hypotheses{truth};

  const Localiser by_the_matcher(
    parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> at_no_bound =
    by_the_matcher.localise(measured, kRangeMin, kRangeMax, hypotheses);

  const std::expected<Result, Error> at_a_bound_above_everything =
    by_the_matcher.localise(measured, above_every_reading, kRangeMax,
      hypotheses);

  ASSERT_TRUE(at_no_bound.has_value());
  ASSERT_TRUE(at_a_bound_above_everything.has_value())
    << "a run refined by the matcher never reads the near bound, so raising it "
    << "above every reading of the scan cannot stop it answering.";

  EXPECT_EQ(at_a_bound_above_everything->pose.x, at_no_bound->pose.x)
    << "the same run refined by the matcher at two different near bounds gave "
    << "two different answers, so the bound reached a stage that is documented "
    << "as never reading it.";
  EXPECT_EQ(at_a_bound_above_everything->pose.y, at_no_bound->pose.y);
  EXPECT_EQ(at_a_bound_above_everything->pose.t, at_no_bound->pose.t);

  const Localiser by_icp(
    parametersRefiningByIcp(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> refused =
    by_icp.localise(measured, above_every_reading, kRangeMax, hypotheses);

  ASSERT_FALSE(refused.has_value())
    << "every reading of the scan sits under the near bound it was handed, so "
    << "the other refinement carries not one of them into a match as a "
    << "distance and has nothing left to refine any survivor against.";

  EXPECT_EQ(refused.error().reason, Error::Reason::every_survivor_refused);

  EXPECT_TRUE(refused.error().refusals.empty())
    << "the matcher took no part in the run that was refused, so a count under "
    << "its reasons would name a stage that never ran.";

  const std::map<IcpError, unsigned int>& icp_refusals =
    refused.error().icp_refusals;

  ASSERT_EQ(icp_refusals.count(IcpError::scan_entirely_invalid), 1u)
    << "the scan is refused for carrying no usable reading, which is settled "
    << "before the map is cast from, rather than for a match the solver could "
    << "not make.";

  const std::size_t candidates = hypotheses.size() * kOrientations;
  const std::size_t survivors = std::min<std::size_t>(kTopK, candidates);

  EXPECT_EQ(icp_refusals.at(IcpError::scan_entirely_invalid), survivors)
    << "the refinement was handed " << survivors << " survivors, being the "
    << "smaller of " << kTopK << " and the " << candidates << " candidates "
    << "one hypothesis at " << kOrientations << " orientations yields, and "
    << "each of them was refused over the same unusable scan.";
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
      localiser.localise(scans[s], kRangeMin, kRangeMax);

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
      localiser.localise(measured, kRangeMin, kRangeMax);

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

/**
 * @brief A refinement bounded to less than the cell it has to cross spends its
 * whole recovery budget, and the run says so.
 * @details
 * This is the fifth and last of the matcher's refusals that a run through this
 * library can meet. Four of the five are reached elsewhere in this file and in
 * `test_parameters`: a scan too short and a scan carrying nothing usable are
 * pinned on both sides of the seam by `ScanRefusalSeam`, a hypothesis standing
 * off free space by `EverySurvivorRefusedByTheMatcherRefusesTheRun`, and a first
 * cast that met nothing by
 * `AMapWithNothingInRangeIsRefusedBeforeAnUnscorablePoseCanWin`.
 * **Why a bound below the cell size is the honest way to reach it.** A survivor
 * of the sift stands at the centre of a free cell, so the truth can be half a
 * cell diagonal away from it, which is `resolution * sqrt(2) / 2`. The
 * translation stage abandons a candidate that has moved further than `xy_bound`
 * from where the stage began and treats that as a failure, and a failure is what
 * starts a recovery. A bound of a quarter of that half diagonal therefore cannot
 * reach the truth from where the sift left off, however good the measurement is:
 * every attempt fails, every failure spends a recovery, and the budget runs out.
 * Nothing about the room, the map or the scan is contrived to produce it. The
 * scan is the cast the map itself gives at a pose of the fixture, which every
 * other case in this file recovers from, and the only change is a translation
 * bound a user could set by hand without noticing it was smaller than a cell.
 * **What is asserted, and what is left alone.** Whether the run ends in a
 * refusal or in an answer is not asserted, because it depends on how many of the
 * ten survivors happen to run out first, and both endings were seen while this
 * was written. What is asserted is the reason: every survivor the matcher turned
 * down was turned down for a spent budget and for nothing else. The control is
 * the same room, the same scan and the same seed at the bound this package
 * ships, which refuses no survivor at all, so the case is about the bound rather
 * than about the room.
 */
TEST_P(LocaliserOnEveryGrid, ATranslationBoundBelowACellSpendsTheRecoveryBudget)
{
  const GridCase grid = GetParam();

  const GridFixture fixture = readGrid(grid.file);
  const std::shared_ptr<const OccupancyGridMap> map = mapOf(fixture);
  ASSERT_NE(map, nullptr);

  const std::vector<Pose> room_poses = readTruth(kTruthFixture);
  ASSERT_FALSE(room_poses.empty())
    << "the truth fixture " << kTruthFixture << " named no pose, so there is "
    << "no pose to cast the measurement from.";

  const Pose truth = worldFromRoom(*map, fixture.origin_yaw, room_poses[0]);
  ASSERT_TRUE(map->isFree(truth.x, truth.y));

  const std::vector<double> measured = map->ranges(truth, kRayCount, kRangeMax);

  const double resolution = map->resolution();
  const double half_cell_diagonal = resolution * std::sqrt(2.0) / 2.0;

  const Localiser generous(parametersOf(kTopK), map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> control =
    generous.localise(measured, kRangeMin, kRangeMax);

  ASSERT_TRUE(control.has_value())
    << "the control run, at the bound this package ships, produced no pose on "
    << "the " << grid.label << " grid, so the room and the scan are the "
    << "difficulty rather than the bound.";

  EXPECT_TRUE(control->refusals.empty())
    << "the control refused a survivor at the shipped bound of "
    << parametersOf(kTopK).matcher.xy_bound << " metres, so what the tight "
    << "bound below causes is not the whole of what is being measured.";

  Parameters bounded = parametersOf(kTopK);
  bounded.matcher.xy_bound = half_cell_diagonal / 4.0;

  ASSERT_TRUE(cbgl::validate(bounded).empty())
    << "a translation bound of " << bounded.matcher.xy_bound << " metres has to "
    << "be a bound the parameters accept, or this case is about a refused "
    << "configuration and not about a spent budget: " << cbgl::validate(bounded);

  const Localiser tight(bounded, map, freeSpaceOfMap(*map));

  const std::expected<Result, Error> result =
    tight.localise(measured, kRangeMin, kRangeMax);

  const std::map<CorrectionError, unsigned int>& refusals =
    result.has_value() ? result->refusals : result.error().refusals;

  unsigned int counted = 0U;
  for (const auto& refusal : refusals)
    counted += refusal.second;

  ASSERT_GT(counted, 0u)
    << "a refinement bounded to " << bounded.matcher.xy_bound << " metres on "
    << "the " << grid.label << " grid, whose cells are " << resolution
    << " metres and whose survivors therefore stand up to "
    << half_cell_diagonal << " metres from the truth, refused nothing at all. "
    << "The translation stage cannot have reached the truth from where the sift "
    << "left off, so it either was not bounded or the failure it met was not "
    << "counted.";

  EXPECT_LE(counted, kTopK)
    << "the refinement was handed at most " << kTopK << " survivors and "
    << counted << " refusals were counted, so a survivor was counted twice.";

  for (const auto& refusal : refusals)
    EXPECT_EQ(refusal.first, CorrectionError::recovery_budget_exhausted)
      << "a survivor was refused " << refusal.second << " times for something "
      << "other than a spent recovery budget on the " << grid.label << " grid. "
      << "The measurement is this map's own cast from a free cell, so nothing "
      << "here is short, unusable, off free space or out of range: the only "
      << "thing the tight bound can cause is a translation that fails, "
      << "recovers, and runs out of recoveries.";
}

/**
 * @brief The two scan refusals this library decides are the matcher's own
 * refusals under another name, and must stay that way.
 * @details A scan shorter than `size_scan`, and a scan carrying no usable range,
 * are both refused here before a hypothesis is drawn, and both are refused again
 * by `fsm_localisation::Corrector::correct` when a survivor reaches it. The
 * duplication is deliberate: each is required to be a distinct reason of this
 * library's own, and deferring to the matcher would report both as every
 * survivor having been refused, which names the refinement for a fault in the
 * measurement.
 * Two definitions of one rule can part company. This pins them: the very scan
 * this library refuses is handed to the matcher, and the matcher has to refuse it
 * for its own equivalent reason. If the matcher ever changes what either means,
 * this goes red rather than the two quietly disagreeing.
 * **Every flavour, not one of each.** A lidar reports a ray it could not trust
 * as zero, as infinity or as not-a-number, and a range below zero is no distance
 * at all either. Each is a separate encoding of the same thing, and either layer
 * could stop recognising one of them while still recognising the rest, so each is
 * a case here rather than one standing in for all four. The length rule gets the
 * boundary itself, one ray short of what the matcher asks for, rather than a scan
 * so short that a rule off by one would still refuse it.
 * The scan neither layer may refuse is
 * `AScanTheSeamHasNoBusinessRefusingIsRefusedByNeitherLayer`, which is the same
 * seam approached from inside.
 */
TEST_P(ScanRefusalSeam, AScanRefusedHereIsRefusedByTheMatcherForTheSameThing)
{
  const SeamCase seam = GetParam();

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

  const std::vector<double> ranges(seam.rays, seam.range);

  const std::expected<Result, Error> library =
    localiser.localise(ranges, kRangeMin, kRangeMax);

  ASSERT_FALSE(library.has_value())
    << "a scan of " << seam.rays << " rays, each of them " << seam.range
    << ", is one this library is required to refuse before a hypothesis is "
    << "drawn, and it produced an answer instead.";

  EXPECT_EQ(library.error().reason, seam.library)
    << "the " << seam.label << " scan was refused for another of this library's "
    << "reasons than the one it is about, so a host reading the sentence is "
    << "sent to look at the wrong thing.";

  const std::expected<fsm_localisation::CorrectionResult, CorrectionError>
    matcher = corrector.correct(ranges, truth, kRangeMax);

  ASSERT_FALSE(matcher.has_value())
    << "this library refuses the " << seam.label << " scan of " << seam.rays
    << " rays and the matcher accepted it. The two no longer mean the same thing "
    << "by the rule, and the earlier of the two checks is now guessing on the "
    << "matcher's behalf.";

  EXPECT_EQ(matcher.error(), seam.matcher)
    << "the matcher refused the same scan for a different reason of its own, so "
    << "what this library reports about the " << seam.label << " scan is no "
    << "longer what the matcher would have called it.";
}

INSTANTIATE_TEST_SUITE_P(
  AtFiveCentimetreCells, LocaliserOnAFineGrid,
  testing::ValuesIn(kFineGrids), gridCaseSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverEveryFlavourTheSeamDecides, ScanRefusalSeam,
  testing::ValuesIn(kSeamCases), seamCaseSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverEveryGrid, LocaliserOnEveryGrid,
  testing::ValuesIn(kEveryGrid), gridCaseSuffix);
