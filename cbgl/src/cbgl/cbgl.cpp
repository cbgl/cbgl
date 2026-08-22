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
 * @brief `cbgl::caer`, `cbgl::candidatesOf`, `cbgl::validate` and
 * `cbgl::Localiser`: the whole of the algorithm.
 */
#include "cbgl/cbgl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <format>
#include <limits>
#include <random>
#include <thread>
#include <utility>

namespace cbgl
{
namespace
{
/**
 * @brief What refining a whole set of survivors amounts to: the corrected pose
 * of every survivor the refinement accepted, and how many it refused, counted
 * by the reason it gave.
 * @details Two counts rather than one because the two refinements refuse for
 * reasons that do not correspond and neither reason set is the other written
 * differently. One refinement runs per localisation, so at most one of the two
 * is ever filled.
 */
struct Refinement
{
  std::vector<Pose> corrected;

  std::map<IcpError, unsigned int> icp_refusals;

  std::map<fsm_localisation::CorrectionError, unsigned int> refusals;
};

/**
 * @brief The corrected pose of every survivor the matcher accepted, and how
 * many it refused, by reason.
 * @details Refinement is not asked to run concurrently, `top_k_caers` being
 * small enough that there is nothing to gain from splitting it the way the
 * sift is.
 */
Refinement correctedPosesOf(std::span<const ScoredPose> survivors,
  std::span<const double> scan, const double range_max,
  const fsm_localisation::Corrector& corrector)
{
  Refinement refinement;

  for (const ScoredPose& survivor : survivors)
  {
    const std::expected<fsm_localisation::CorrectionResult,
      fsm_localisation::CorrectionError> result =
      corrector.correct(scan, survivor.pose, range_max);

    if (result.has_value())
      refinement.corrected.push_back(result->corrected);
    else
      refinement.refusals[result.error()]++;
  }

  return refinement;
}

/**
 * @brief The corrected pose of every survivor the iterative closest point
 * refinement accepted, and how many it refused, by reason.
 * @details Serial as well, and not for the reason the overload above is. That
 * one is serial because splitting it would buy nothing. This one is serial
 * because it cannot be split at all: the solver underneath keeps its context
 * stack, the counters that go with it and every matrix it has allocated in
 * process globals and guards none of it, so two refinements running at once
 * can be handed each other's matrices. `IcpRefiner`'s own header says so, of
 * any two calls anywhere in the process and not only of two on one refiner.
 * @remark What the refinement reports beyond the corrected pose, its error and
 * its cycle count, is dropped here. It says nothing on its own and means
 * something only against the same figures from another candidate of the same
 * scan, and the selection that follows ranks the corrected poses by the same
 * metric the sift ranked them by rather than by anything a refinement said
 * about itself.
 */
Refinement correctedPosesOf(std::span<const ScoredPose> survivors,
  std::span<const double> scan, const double range_min, const double range_max,
  const double aperture, const IcpRefiner& refiner)
{
  Refinement refinement;

  for (const ScoredPose& survivor : survivors)
  {
    const std::expected<IcpResult, IcpError> result =
      refiner.refine(scan, survivor.pose, range_min, range_max, aperture);

    if (result.has_value())
      refinement.corrected.push_back(result->corrected);
    else
      refinement.icp_refusals[result.error()]++;
  }

  return refinement;
}

/**
 * @brief The matcher's own encoding of "windowed" or "angular", from the
 * string a parameter set carries. `validate` has already refused anything
 * else, so there is no third case to report here.
 */
FSM::RaySearch raySearchOf(const std::string& ray_search)
{
  return ray_search == "windowed" ? FSM::RaySearch::windowed : FSM::RaySearch::angular;
}

/**
 * @brief How many rays a cast must carry for a scan of `rays` rays spanning
 * `aperture` radians to be read against a full turn of map.
 * @details A sensor that sees all the way round is its own answer. One that
 * does not is served by casting the whole turn at the sensor's own angular
 * resolution and comparing only the part of it the sensor could have seen,
 * which is what `prefixOf` and the window in `candidatesOf` then do. Casting
 * the sensor's own ray count over a full turn instead would compare a fraction
 * of a turn of measurement against the whole of one, which is not a worse
 * answer but no answer at all.
 * @remark The rounding is away from zero, which is what the ROS 1 build's own
 * sift does and what the harness that shapes a narrower scan does. All three
 * have to agree on where a window starts or the rays they align are not the
 * same rays.
 */
std::size_t panoramicRaysOf(const std::size_t rays, const double aperture)
{
  if (rays == 0 || !(aperture > 0.0))
    return rays;

  const double turn =
    static_cast<double>(rays) * 2.0 * M_PI / aperture;

  const std::size_t panoramic =
    static_cast<std::size_t>(std::llround(turn));

  return panoramic < rays ? rays : panoramic;
}

/**
 * @brief How many rays of a panoramic cast sit before the window keeps any.
 * @details The window is centred, so what is dropped is split evenly between
 * the two ends and an odd remainder puts the extra ray at the near end.
 */
std::size_t prefixOf(const std::size_t panoramic, const std::size_t rays)
{
  if (panoramic <= rays)
    return 0;

  return static_cast<std::size_t>(
    std::llround(static_cast<double>(panoramic - rays) / 2.0));
}

/**
 * @brief The scan every stage of a run works from: exactly `size_scan` rays,
 * or the whole of `ranges` where `size_scan` is zero.
 * @details Deciding a scan too short or entirely invalid here, before a single
 * hypothesis is drawn, is what keeps those two reasons from being reported as
 * "every survivor refused" once the same scan reaches the matcher survivor by
 * survivor. The reduction itself is the matcher's own, called directly rather
 * than copied, so that a scan reduced here and a scan reduced again inside
 * `fsm_localisation::Corrector::correct` agree once their lengths already do.
 */
std::expected<std::vector<double>, Error::Reason> reducedRanges(
  std::span<const double> ranges, const Parameters& parameters)
{
  const std::size_t target = parameters.matcher.size_scan == 0
    ? ranges.size() : parameters.matcher.size_scan;

  if (ranges.size() < target)
    return std::unexpected(Error::Reason::scan_too_short);

  std::vector<double> scan(ranges.begin(), ranges.end());
  std::ranges::replace_if(scan,
    [](const double range) { return !fsm_localisation::isValidRange(range); }, 0.0);

  if (std::ranges::none_of(scan, fsm_localisation::isValidRange))
    return std::unexpected(Error::Reason::scan_entirely_invalid);

  if (scan.size() == target)
    return scan;

  return FSM::Utils::subsampleScan(FSM::DatasetUtils::interpolateRanges(scan),
    target, raySearchOf(parameters.matcher.ray_search));
}

/**
 * @brief Every candidate of every hypothesis, scored, unranked.
 * @details Split by hypothesis index into `num_threads` contiguous chunks,
 * each thread writing only into the slots its own chunk owns. The ranking is
 * left to the caller, computed once the whole of this vector is settled, so
 * that it cannot depend on how the work was divided.
 * Casting concurrently from several threads into the same map is safe because
 * `fsm_localisation::OccupancyGridMap` guarantees it of its own `ranges`, the
 * three borrowed casters being built with their cache switched off; see that
 * package's own note beside `_USE_LRU_CACHE`.
 * Nothing a thread's own chunk throws is allowed to leave that thread: an
 * exception escaping a `std::thread`'s body calls `std::terminate`, as does
 * unwinding a vector of still joinable threads, so every chunk's exception is
 * caught where it happens, every thread is joined regardless of whether
 * spawning the rest succeeded, and only then is the first exception any chunk
 * caught rethrown here, on the thread that called this function.
 * @remark The empty check below is not defending a case `Localiser::localise`
 * still reaches, since it already refuses an empty hypothesis set before this
 * is called. It is what keeps `threads` off zero, which is what keeps the
 * division that follows safe.
 */
std::vector<ScoredPose> scoredCandidatesOf(std::span<const Pose> hypotheses,
  std::span<const double> measured, const double aperture,
  const double range_max, const Parameters& parameters,
  const FSM::GridReference& map)
{
  std::vector<ScoredPose> scored(hypotheses.size() * parameters.da);

  if (hypotheses.empty())
    return scored;

  const std::size_t rays = measured.size();
  const unsigned int num_rays =
    static_cast<unsigned int>(panoramicRaysOf(rays, aperture));

  const auto scoreOne = [&](const std::size_t hypothesis)
  {
    const std::vector<double> cast =
      map.ranges(hypotheses[hypothesis], num_rays, range_max);

    const std::vector<Candidate> candidates =
      candidatesOf(hypotheses[hypothesis], cast, parameters.da, rays);

    for (unsigned int k = 0U; k < parameters.da; k++)
      scored[hypothesis * parameters.da + k] = ScoredPose{
        .pose = candidates[k].pose,
        .score = caer(measured, candidates[k].ranges, range_max)};
  };

  const std::size_t threads = std::min<std::size_t>(hypotheses.size(),
    parameters.num_threads != 0U
      ? parameters.num_threads
      : std::max<unsigned int>(1U, std::thread::hardware_concurrency()));

  const std::size_t chunk = hypotheses.size() / threads;
  const std::size_t remainder = hypotheses.size() % threads;

  std::vector<std::thread> workers;
  std::vector<std::exception_ptr> failures(threads);
  std::size_t begin = 0;

  try
  {
    for (std::size_t t = 0; t < threads; t++)
    {
      const std::size_t end = begin + chunk + (t < remainder ? 1 : 0);

      workers.emplace_back([begin, end, t, &scoreOne, &failures]()
        {
          try
          {
            for (std::size_t i = begin; i < end; i++)
              scoreOne(i);
          }
          catch (...)
          {
            failures[t] = std::current_exception();
          }
        });

      begin = end;
    }
  }
  catch (...)
  {
    for (std::thread& worker : workers)
      worker.join();

    throw;
  }

  for (std::thread& worker : workers)
    worker.join();

  for (const std::exception_ptr& failure : failures)
    if (failure)
      std::rethrow_exception(failure);

  return scored;
}

}

/**
 * @details The cast being entirely at the measurement's maximum range is a
 * separate condition from a ray either side calling invalid: a range equal to
 * that maximum is a valid reading on its own, per
 * `fsm_localisation::isValidRange`, and only ranks behind everything because
 * the cast as a whole met nothing. This mirrors the same test
 * `fsm_localisation::Corrector::correct` makes of a first cast, right down to
 * the `>=`.
 */
double caer(std::span<const double> measured, std::span<const double> cast,
  const double range_max) noexcept
{
  if (std::ranges::all_of(cast,
      [range_max](const double range) { return range >= range_max; }))
    return std::numeric_limits<double>::infinity();

  const std::size_t rays = std::min(measured.size(), cast.size());

  double sum = 0.0;
  bool contributed = false;

  for (std::size_t i = 0; i < rays; i++)
  {
    if (!fsm_localisation::isValidRange(measured[i]) ||
        !fsm_localisation::isValidRange(cast[i]))
      continue;

    sum += std::abs(measured[i] - cast[i]);
    contributed = true;
  }

  return contributed ? sum : std::numeric_limits<double>::infinity();
}

std::vector<Candidate> candidatesOf(const Pose& pose,
  std::span<const double> cast, const unsigned int da,
  const std::size_t rays)
{
  const std::size_t n = cast.size();
  const std::size_t kept = rays == 0 ? n : rays;
  const std::size_t prefix = prefixOf(n, kept);

  std::vector<Candidate> candidates(da);

  for (unsigned int k = 0U; k < da; k++)
  {
    const long shift = n == 0
      ? 0L : std::lround(static_cast<double>(k) * n / da);

    const double heading = n == 0
      ? pose.t
      : FSM::Utils::wrapAngle(pose.t +
          2.0 * M_PI * static_cast<double>(shift) / static_cast<double>(n));

    std::vector<double> rotated(kept);
    for (std::size_t i = 0; i < kept; i++)
      rotated[i] = cast[(static_cast<std::size_t>(shift) + prefix + i) % n];

    candidates[k] = Candidate{
      .pose = Pose{.x = pose.x, .y = pose.y, .t = heading},
      .ranges = std::move(rotated)};
  }

  return candidates;
}

std::string validate(const Parameters& parameters)
{
  if (parameters.da == 0U)
    return std::format("da must be greater than zero, got {}", parameters.da);

  if (!(parameters.dl > 0.0))
    return std::format("dl must be greater than zero, got {}", parameters.dl);

  if (parameters.top_k_caers == 0U)
    return std::format("top_k_caers must be greater than zero, got {}",
      parameters.top_k_caers);

  return fsm_localisation::validate(parameters.matcher);
}

/**
 * @details The matcher is built whichever refinement was asked for, its
 * settings carrying the reduction every stage of a run works from as well as
 * the refinement's own. The iterative closest point refinement is built beside
 * it only where the settings carry a set for it, and it is built in the body
 * rather than in the list above because a refinement that is there sometimes
 * cannot be written there: one can be neither copied nor moved, deliberately,
 * so there is no one expression that yields either a refinement or nothing for
 * the list to take.
 */
Localiser::Localiser(const Parameters& parameters,
  std::shared_ptr<const FSM::GridReference> map,
  FreeSpace free_space)
: corrector_(parameters.matcher, map),
  free_space_(std::move(free_space)),
  map_(std::move(map)),
  parameters_(parameters)
{
  if (parameters_.icp.has_value())
    icp_refiner_.emplace(*parameters_.icp, map_);
}

/**
 * @details A fresh generator is seeded here, on every call, rather than kept
 * as a member: nothing survives a call, so the same localiser asked twice
 * must draw the same set at a pinned seed and no run may find its draw
 * depending on whichever call came before it.
 */
std::vector<Pose> Localiser::disperse() const
{
  if (free_space_.cells.empty())
    return {};

  const double free_area = static_cast<double>(free_space_.cells.size()) *
    free_space_.resolution * free_space_.resolution;

  const std::size_t count = static_cast<std::size_t>(
    std::max<long>(1L, std::lround(parameters_.dl * free_area)));

  std::mt19937 generator(parameters_.rng_seed != 0U
    ? parameters_.rng_seed : std::random_device{}());

  std::uniform_int_distribution<std::size_t> cell(
    0, free_space_.cells.size() - 1);
  std::uniform_real_distribution<double> heading(-M_PI, M_PI);

  std::vector<Pose> poses;
  poses.reserve(count);

  for (std::size_t i = 0; i < count; i++)
  {
    const Point& position = free_space_.cells[cell(generator)];
    poses.push_back(Pose{.x = position.x, .y = position.y, .t = heading(generator)});
  }

  return poses;
}

/**
 * @details The map's free space is checked before the scan is even reduced,
 * since it is a property of the map rather than of the scan and answering it
 * costs nothing. An empty hypothesis set is refused separately and by its own
 * reason, since the four argument overload exists precisely so a caller can
 * hand one in, and reporting it as the map having no free space would be
 * false on a map that plainly has some. The scan is then reduced exactly
 * once, by `reducedRanges`, and that single reduced scan is what the sift,
 * the refinement and the selection all work from.
 * Which refinement runs is settled once, here, rather than survivor by
 * survivor, and it is settled by whether one was built at construction.
 */
std::expected<Result, Error> Localiser::localise(std::span<const double> ranges,
  const double range_min, const double range_max,
  std::span<const Pose> hypotheses, const double aperture) const
{
  if (free_space_.cells.empty())
    return std::unexpected(Error{
      .reason = Error::Reason::map_has_no_free_space,
      .icp_refusals = {}, .refusals = {}});

  if (hypotheses.empty())
    return std::unexpected(Error{
      .reason = Error::Reason::no_hypotheses_given,
      .icp_refusals = {}, .refusals = {}});

  const std::chrono::steady_clock::time_point started =
    std::chrono::steady_clock::now();

  const std::expected<std::vector<double>, Error::Reason> reduced =
    reducedRanges(ranges, parameters_);

  if (!reduced.has_value())
    return std::unexpected(Error{
      .reason = reduced.error(), .icp_refusals = {}, .refusals = {}});

  std::vector<ScoredPose> scored = scoredCandidatesOf(
    hypotheses, *reduced, aperture, range_max, parameters_, *map_);

  const std::size_t num_candidates = scored.size();
  const std::size_t kept =
    std::min<std::size_t>(parameters_.top_k_caers, scored.size());

  std::ranges::nth_element(scored, scored.begin() + kept, {}, &ScoredPose::score);
  scored.resize(kept);
  std::ranges::sort(scored, {}, &ScoredPose::score);

  const Refinement refinement = icp_refiner_.has_value()
    ? correctedPosesOf(
        scored, *reduced, range_min, range_max, aperture, *icp_refiner_)
    : correctedPosesOf(scored, *reduced, range_max, corrector_);

  if (refinement.corrected.empty())
    return std::unexpected(Error{
      .reason = Error::Reason::every_survivor_refused,
      .icp_refusals = refinement.icp_refusals,
      .refusals = refinement.refusals});

  std::vector<ScoredPose> rescored;
  rescored.reserve(refinement.corrected.size());

  const std::size_t panoramic = panoramicRaysOf(reduced->size(), aperture);
  const std::size_t prefix = prefixOf(panoramic, reduced->size());

  for (const Pose& pose : refinement.corrected)
  {
    const std::vector<double> cast = map_->ranges(
      pose, static_cast<unsigned int>(panoramic), range_max);

    const std::span<const double> seen(cast.data() + prefix, reduced->size());

    rescored.push_back(ScoredPose{
      .pose = pose, .score = caer(*reduced, seen, range_max)});
  }

  const auto winner = std::ranges::min_element(rescored, {}, &ScoredPose::score);

  if (std::isinf(winner->score))
    return std::unexpected(Error{
      .reason = Error::Reason::no_scorable_pose,
      .icp_refusals = refinement.icp_refusals,
      .refusals = refinement.refusals});

  return Result{
    .pose = winner->pose,
    .execution_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count(),
    .num_candidates = num_candidates,
    .survivors = std::move(scored),
    .icp_refusals = refinement.icp_refusals,
    .refusals = refinement.refusals};
}

std::expected<Result, Error> Localiser::localise(std::span<const double> ranges,
  const double range_min, const double range_max, const double aperture) const
{
  const std::vector<Pose> hypotheses = disperse();
  return localise(ranges, range_min, range_max, hypotheses, aperture);
}

}
