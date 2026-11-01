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
 */
struct Refinement
{
  std::vector<Pose> corrected;

  std::map<fsm_localisation::CorrectionError, unsigned int> refusals;
};

/**
 * @brief The corrected pose of every survivor the matcher accepted, and how
 * many it refused, by reason.
 * @details Refinement is not asked to run concurrently, `top_k_caers` being
 * small enough that there is nothing to gain from splitting it the way the
 * sift is.
 * `scan` is what `reducedRanges` gives, so its first ray lies where the window
 * compares ray 0, `FSM::windowStartOf` of its ray count and `aperture`, and
 * not at the sensor's `angle_min`. That is the angle the matcher is told its
 * first ray lies at.
 */
Refinement correctedPosesOf(std::span<const ScoredPose> survivors,
  std::span<const double> scan, const double range_max, const double aperture,
  const fsm_localisation::Corrector& corrector)
{
  Refinement refinement;

  const double first_ray_angle = FSM::windowStartOf(scan.size(), aperture);

  for (const ScoredPose& survivor : survivors)
  {
    const std::expected<fsm_localisation::CorrectionResult,
      fsm_localisation::CorrectionError> result = corrector.correct(
        scan, survivor.pose, range_max, aperture, first_ray_angle);

    if (result.has_value())
      refinement.corrected.push_back(result->corrected);
    else
      refinement.refusals[result.error()]++;
  }

  return refinement;
}

/**
 * @brief The sift's own best candidate, returned without refining anything.
 * @details `scored` is already sorted by the same metric the selection would
 * rank a refined pose by, so the cheapest of them is the winner and no cast is
 * taken to find that out. A candidate that met nothing scores infinity and is
 * refused here as it would be after a refinement.
 * @remark The two refusal maps are empty because nothing was asked to refine.
 * A caller reading them cannot tell that from a run where every refinement
 * succeeded, which is why the node says at start up which refinement it holds.
 * @remark `scored` must not be empty. `validate` refuses a `top_k_caers` of
 * nought, which is what guarantees it, and a caller reaching `Localiser`
 * without passing through that refusal has undefined behaviour here where the
 * refined paths would report every survivor refused.
 */
std::expected<Result, Error> unrefinedResult(std::vector<ScoredPose> scored,
  const std::size_t num_candidates,
  const std::chrono::steady_clock::time_point started)
{
  if (std::isinf(scored.front().score))
    return std::unexpected(Error{
      .reason = Error::Reason::no_scorable_pose,
      .refusals = {}});

  const Pose winner = scored.front().pose;

  return Result{
    .pose = winner,
    .execution_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count(),
    .num_candidates = num_candidates,
    .survivors = std::move(scored),
    .refusals = {}};
}

/**
 * @brief The scan every stage of a run works from: exactly `size_scan` rays,
 * or as many rays as `ranges` holds where `size_scan` is zero.
 * @details Deciding a scan too short or entirely invalid here, before a single
 * hypothesis is drawn, is what keeps those two reasons from being reported as
 * "every survivor refused" once the same scan reaches the matcher survivor by
 * survivor.
 * Reduced ray `j` holds the scan read at the angle the window compares that
 * ray at, `-pi + (prefix + j) * 2 * pi / panoramic` from the heading, with
 * `panoramic` and `prefix` from `FSM::panoramicRaysOf` and
 * `FSM::windowPrefixOf`, the window that the sift, the selection and the
 * refinement in `fsm_localisation` all take. The sensor's ray `k` lies at
 * `angle_min + k * aperture / n` for a scan of `n` rays, so that angle falls
 * between two of the sensor's rays, and the range is interpolated linearly
 * between them. No reduced ray between the sensor's first and last rays is
 * therefore compared at an angle other than the one its range was read at,
 * whatever the ray count and the aperture.
 * A scan reduced to fewer rays has its missing readings filled first, by the
 * matcher's own `FSM::DatasetUtils::interpolateRanges`, as the reduction
 * always filled them. A reduced ray read beside a filled reading blends its
 * valid neighbour with the value that fill gave, which is what the reduction
 * did before, and such a reduced scan carries no zero.
 * A scan read at its own length is not filled. A ray of it read from a
 * missing reading, one lying on that reading or between it and a neighbour,
 * is left missing, nought, as a scan returned as it is keeps one. The sift
 * skips it as it skips the gap of a scan whose rays lie on the angles they
 * are compared at, and the matcher fills both alike. A gap of `m` rays
 * between two measured readings therefore leaves missing the reduced rays
 * that lie strictly between those two, `m + 1` of the sensor's spacings
 * apart: at most `ceil((m + 1) * r)` of them, `r` being the sensor's
 * spacing over the compared spacing. Where the two agree, `r` is 1 and the
 * gap grows by one ray at most. Where they do not, `r` is the compared rays a
 * turn holds over the sensor's rays it holds, which differ by a ray at most,
 * and the gap grows by two rays at most. 500 rays `2 * pi / 999.6` apart are
 * compared `2 * pi / 1000` apart, `r` is 1.0004, and a gap of 40 of them
 * leaves 41 or 42 reduced rays missing. A gap at an end ray of a narrower
 * scan also leaves missing every reduced ray compared beyond that end, each
 * being given the end ray's range.
 * A scan covering a whole turn, one whose ray count `FSM::panoramicRaysOf`
 * leaves as it is, has no first or last ray in that sense. Its last ray and
 * its first are neighbours a ray apart where the turn closes, as any two
 * neighbours are, so a reduced ray compared between them is interpolated
 * between them.
 * 360 rays a degree apart from -179.5 degrees compare reduced ray 0 at -180,
 * and it reads the mean of the rays at 179.5 and -179.5. No reduced ray of a
 * whole turn is therefore compared at an angle other than the one its range
 * was read at.
 * A reduced ray of a narrower scan compared beyond the sensor's first or last
 * ray is given the range of that end ray. Reducing 1073 rays a quarter of a
 * degree apart to 269 compares reduced ray 0 at -134.127 degrees, where the
 * first ray lies at -134.000, and no other reduced ray falls outside. Its
 * misplacement is 0.127 degrees on one ray of 269, which is 0.0005 degrees
 * over the scan. Leaving it missing would not keep it out of the refinement:
 * `fsm_localisation::Corrector::correct` fills a zero again from the rays
 * either side of it, which for the first ray are the second and the last, so
 * the refinement would read a range the sift never saw.
 * @remark A scan already the length asked for, which every scan is where
 * `size_scan` is zero, is read at its compared angles unless its ray `j`
 * already lies where the window compares ray `j`. Such a scan is returned as it
 * is, missing readings and all, and compared ray for ray where the window lays
 * it. How far ray `j` lies from where it is compared grows linearly with `j`,
 * so only the first and the last ray are checked, each to within `2 * pi` times
 * the epsilon of a 32 bit float, 7.5e-7 radians. Those of a whole turn are
 * checked with whole turns taken off, a ray turned by a whole turn lying
 * where it lay. A scan carrying its angles as 32 bit floats, as a
 * `sensor_msgs` scan does, has its first angle rounded by at most `pi` times
 * half that epsilon and its last ray moved by at most three times as much,
 * both inside it. The 1073 rays a quarter of a degree apart from -134 degrees
 * that a real robot's sensor reports lie at most 1.4e-7 radians off, and a
 * whole turn from -pi or from pi lies off by nothing but that rounding. A
 * centred scan of 1072 rays at that spacing lies half a ray, 2.2e-3 radians,
 * off, and is read at the angles its rays are compared at.
 */
std::expected<std::vector<double>, Error::Reason> reducedRanges(
  std::span<const double> ranges, const Parameters& parameters,
  const double aperture, const double angle_min)
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

  const double increment = aperture / static_cast<double>(scan.size());

  const std::size_t panoramic = FSM::panoramicRaysOf(target, aperture);
  const std::size_t prefix = FSM::windowPrefixOf(panoramic, target);

  const auto positionOf = [&](const std::size_t j)
  {
    const double compared = -M_PI + 2.0 * M_PI *
      static_cast<double>(prefix + j) / static_cast<double>(panoramic);

    return (compared - angle_min) / increment;
  };

  const double slack =
    2.0 * M_PI * std::numeric_limits<float>::epsilon() / increment;
  const double last = static_cast<double>(scan.size() - 1);
  const double rays = static_cast<double>(scan.size());
  const bool whole_turn =
    FSM::panoramicRaysOf(scan.size(), aperture) == scan.size();

  const auto wrappedOffset = [&](const double offset)
  {
    return whole_turn ? offset - rays * std::round(offset / rays) : offset;
  };

  if (scan.size() == target &&
      std::abs(wrappedOffset(positionOf(0))) <= slack &&
      std::abs(wrappedOffset(positionOf(target - 1) - last)) <= slack)
    return scan;

  const std::vector<double> readings = scan.size() == target
    ? scan : FSM::DatasetUtils::interpolateRanges(scan);

  std::vector<double> reduced(target);

  for (std::size_t j = 0; j < target; j++)
  {
    const double position = whole_turn
      ? positionOf(j) : std::clamp(positionOf(j), 0.0, last);

    const double floored = std::floor(position);
    const std::size_t below = static_cast<std::size_t>(
      floored - rays * std::floor(floored / rays));
    const std::size_t above = whole_turn
      ? (below + 1) % scan.size() : std::min(below + 1, scan.size() - 1);
    const double weight = position - floored;

    reduced[j] = fsm_localisation::isValidRange(readings[below]) &&
        (weight == 0.0 || fsm_localisation::isValidRange(readings[above]))
      ? readings[below] + weight * (readings[above] - readings[below]) : 0.0;
  }

  return reduced;
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
    static_cast<unsigned int>(FSM::panoramicRaysOf(rays, aperture));

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
  const std::size_t prefix = FSM::windowPrefixOf(n, kept);

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
 * reason, since the five argument overload exists precisely so a caller can
 * hand one in, and reporting it as the map having no free space would be
 * false on a map that plainly has some. The scan is then reduced exactly
 * once, by `reducedRanges`, and that single reduced scan is what the sift,
 * the refinement and the selection all work from.
 */
std::expected<Result, Error> Localiser::localise(std::span<const double> ranges,
  const double range_max, std::span<const Pose> hypotheses,
  const double aperture, const double angle_min) const
{
  if (free_space_.cells.empty())
    return std::unexpected(Error{
      .reason = Error::Reason::map_has_no_free_space,
      .refusals = {}});

  if (hypotheses.empty())
    return std::unexpected(Error{
      .reason = Error::Reason::no_hypotheses_given,
      .refusals = {}});

  const std::chrono::steady_clock::time_point started =
    std::chrono::steady_clock::now();

  const std::expected<std::vector<double>, Error::Reason> reduced =
    reducedRanges(ranges, parameters_, aperture, angle_min);

  if (!reduced.has_value())
    return std::unexpected(Error{
      .reason = reduced.error(), .refusals = {}});

  std::vector<ScoredPose> scored = scoredCandidatesOf(
    hypotheses, *reduced, aperture, range_max, parameters_, *map_);

  const std::size_t num_candidates = scored.size();
  const std::size_t kept =
    std::min<std::size_t>(parameters_.top_k_caers, scored.size());

  std::ranges::nth_element(scored, scored.begin() + kept, {}, &ScoredPose::score);
  scored.resize(kept);
  std::ranges::sort(scored, {}, &ScoredPose::score);

  if (!parameters_.refine)
    return unrefinedResult(scored, num_candidates, started);

  const Refinement refinement =
    correctedPosesOf(scored, *reduced, range_max, aperture, corrector_);

  if (refinement.corrected.empty())
    return std::unexpected(Error{
      .reason = Error::Reason::every_survivor_refused,
      .refusals = refinement.refusals});

  std::vector<ScoredPose> rescored;
  rescored.reserve(refinement.corrected.size());

  const std::size_t panoramic = FSM::panoramicRaysOf(reduced->size(), aperture);
  const std::size_t prefix = FSM::windowPrefixOf(panoramic, reduced->size());

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
      .refusals = refinement.refusals});

  return Result{
    .pose = winner->pose,
    .execution_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count(),
    .num_candidates = num_candidates,
    .survivors = std::move(scored),
    .refusals = refinement.refusals};
}

std::expected<Result, Error> Localiser::localise(std::span<const double> ranges,
  const double range_max, const double aperture, const double angle_min) const
{
  const std::vector<Pose> hypotheses = disperse();
  return localise(ranges, range_max, hypotheses, aperture, angle_min);
}

}
