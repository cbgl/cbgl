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
 * @brief `cbgl::IcpRefiner` and `cbgl::validate`.
 * @remark The solver's own headers are included last and only here. They
 * define a macro named `restrict` and define it to nothing under C++, and that
 * macro escapes into whatever is compiled alongside them. Keeping them out of
 * `icp_refinement.hpp` keeps it out of everything that refines a pose, and
 * including them after everything else keeps it out of the rest of this file.
 */
#include "cbgl/icp_refinement.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <csm/csm.h>

namespace cbgl
{
namespace
{
/**
 * @brief The fewest and the most rays the solver will accept in a scan.
 * @details Its own bounds, checked inside it before anything else and answered
 * with a refusal that says only that the match failed. They are checked here
 * as well so that a host is told which bound it crossed.
 */
const int kFewestRays = 10;
const int kMostRays = 10000;

/**
 * @brief The reading at which the solver stops treating a distance as one.
 * @details A scan carrying a reading of this or more, marked usable, is
 * refused whole rather than in part, so such a reading is marked unusable on
 * the way in.
 */
const double kLargestUsableReading = 100.0;

/**
 * @brief Release of a scan in the solver's own structure.
 * @details A type rather than a function pointer, so that the handle below is
 * the width of a bare pointer and the release can be inlined into the place it
 * happens.
 */
struct LaserDataRelease
{
  void operator()(const LDP ldp) const { ld_free(ldp); }
};

/**
 * @brief A scan in the solver's own structure, freed wherever it goes out of
 * scope.
 */
using LaserDataHandle = std::unique_ptr<laser_data, LaserDataRelease>;

/**
 * @brief A solver result that releases the three covariance matrices the
 * solver may have hung on it, wherever it goes out of scope.
 * @details The solver allocates those three and hands them over without ever
 * taking them back, so whoever holds the result owns them. Owning them through
 * a holder rather than releasing them by hand is what stops a path added later
 * from returning between the call and the release, which is a leak that shows
 * up nowhere: the setting that allocates them is off in every settings set
 * this ships with, so no test can be made to run that release.
 * The result starts zeroed, so that a solver that never writes those three
 * leaves nothing to release rather than whatever the stack held.
 */
class MatcherResult
{
public:
  MatcherResult() { std::memset(&result_, 0, sizeof(result_)); }

  ~MatcherResult()
  {
    gsl_matrix_free(result_.cov_x_m);
    gsl_matrix_free(result_.dx_dy1_m);
    gsl_matrix_free(result_.dx_dy2_m);
  }

  MatcherResult(const MatcherResult&) = delete;
  MatcherResult& operator=(const MatcherResult&) = delete;
  MatcherResult(MatcherResult&&) = delete;
  MatcherResult& operator=(MatcherResult&&) = delete;

  sm_result& get() { return result_; }

private:
  sm_result result_;
};

/**
 * @brief Ranges as the solver's own structure, at the angles a panoramic cast
 * of the same length sits at.
 * @details Ray `i` is placed at `-pi + i * 2 * pi / n`, which is where the ray
 * caster puts its own ray `i` once the pose it cast from is taken out of it.
 * That is what lets a measured scan and a cast of the same length be handed to
 * the solver as they stand, with no resampling between them.
 * A reading is carried into the match as a distance only where it is finite,
 * within the scan's own two bounds, under a hundred metres, and not exactly
 * nought. A reading outside the bounds is clipped to the bound it passed and
 * marked unusable, one that is not a number is put at the far bound and marked
 * unusable, and an exact nought stays nought and is marked unusable, a nought
 * being how a driver says it had nothing to report rather than how it says the
 * wall is here.
 * A reading at or above a hundred metres keeps its value and is marked
 * unusable. The solver refuses whole any scan offering it a usable reading
 * that far out, so a lidar whose own far bound is a hundred metres or more
 * would otherwise mark that bound usable and have every refinement of it
 * refused. Losing those rays costs the match nothing a ray meeting nothing was
 * going to give it.
 */
LaserDataHandle laserDataOf(const std::span<const double> ranges,
  const double range_min, const double range_max)
{
  const int num_rays = static_cast<int>(ranges.size());

  LaserDataHandle ldp(ld_alloc_new(num_rays));

  for (int i = 0; i < num_rays; i++)
  {
    const double range = ranges[static_cast<std::size_t>(i)];

    if (!std::isfinite(range))
    {
      ldp->readings[i] = range_max;
      ldp->valid[i] = 0;
    }
    else if (range < range_min)
    {
      ldp->readings[i] = range_min;
      ldp->valid[i] = 0;
    }
    else if (range > range_max)
    {
      ldp->readings[i] = range_max;
      ldp->valid[i] = 0;
    }
    else if (range >= kLargestUsableReading)
    {
      ldp->readings[i] = range;
      ldp->valid[i] = 0;
    }
    else
    {
      ldp->readings[i] = range;
      ldp->valid[i] = 1;
    }

    if (range == 0.0)
    {
      ldp->readings[i] = 0.0;
      ldp->valid[i] = 0;
    }

    ldp->cluster[i] = -1;
    ldp->theta[i] = -M_PI + i * 2.0 * M_PI / num_rays;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[num_rays - 1];

  for (int i = 0; i < 3; i++)
  {
    ldp->estimate[i] = 0.0;
    ldp->odometry[i] = 0.0;
    ldp->true_pose[i] = 0.0;
  }

  return ldp;
}

/**
 * @brief The settings as the solver takes them.
 * @details Zeroed whole before a field of it is written, so that the fields
 * this does not speak for start from nought rather than from whatever the
 * stack held. Three of them are written by the caller afterwards: the two
 * scans and the first guess.
 * The pose of the sensor on the robot stays at nought. The solver uses it only
 * to turn odometry into a first estimate, and there is no odometry here.
 */
sm_params matcherInputOf(const IcpParameters& parameters,
  const double range_min, const double range_max)
{
  sm_params input;
  std::memset(&input, 0, sizeof(input));

  input.clustering_threshold = parameters.clustering_threshold;
  input.debug_verify_tricks = parameters.debug_verify_tricks;
  input.do_alpha_test = parameters.do_alpha_test;
  input.do_alpha_test_thresholdDeg = parameters.do_alpha_test_thresholdDeg;
  input.do_compute_covariance = parameters.do_compute_covariance;
  input.do_visibility_test = parameters.do_visibility_test;
  input.epsilon_theta = parameters.epsilon_theta;
  input.epsilon_xy = parameters.epsilon_xy;
  input.max_angular_correction_deg = parameters.max_angular_correction_deg;
  input.max_correspondence_dist = parameters.max_correspondence_dist;
  input.max_iterations = parameters.max_iterations;
  input.max_linear_correction = parameters.max_linear_correction;
  input.orientation_neighbourhood = parameters.orientation_neighbourhood;
  input.outliers_adaptive_mult = parameters.outliers_adaptive_mult;
  input.outliers_adaptive_order = parameters.outliers_adaptive_order;
  input.outliers_maxPerc = parameters.outliers_maxPerc;
  input.outliers_remove_doubles = parameters.outliers_remove_doubles;
  input.restart = parameters.restart;
  input.restart_dt = parameters.restart_dt;
  input.restart_dtheta = parameters.restart_dtheta;
  input.restart_threshold_mean_error = parameters.restart_threshold_mean_error;
  input.sigma = parameters.sigma;
  input.use_corr_tricks = parameters.use_corr_tricks;
  input.use_ml_weights = parameters.use_ml_weights;
  input.use_point_to_line_distance = parameters.use_point_to_line_distance;
  input.use_sigma_weights = parameters.use_sigma_weights;

  input.laser[0] = 0.0;
  input.laser[1] = 0.0;
  input.laser[2] = 0.0;

  input.min_reading = range_min;
  input.max_reading = range_max;

  return input;
}

/**
 * @brief An angle put back on the interval from minus pi to pi.
 */
double wrapped(const double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

}

/**
 * @details Every setting with a range is checked against it, and the two that
 * are not simply a range are checked here as well.
 * `orientation_neighbourhood` is capped at the most rays the solver will take
 * in a scan. The solver allocates twice its value for every ray of every scan,
 * so a value above that only enlarges an allocation that can already reach
 * every ray there is, and a value large enough to fail that allocation throws
 * out through a stack the solver does not unwind, holding one level of its
 * context stack for good. Enough of those and it stops the process.
 * `use_sigma_weights` is refused where it is on. The field it weights by is
 * per reading and nothing here fills it, so it stays not a number from the
 * moment the scan is allocated: the solver notices, declines to weight
 * anything, and says so once for every correspondence of every cycle.
 */
std::string validate(const IcpParameters& parameters)
{
  if (!(parameters.clustering_threshold > 0.0))
    return std::format("clustering_threshold must be greater than zero, got {}",
      parameters.clustering_threshold);

  if (!(parameters.do_alpha_test_thresholdDeg >= 0.0))
    return std::format(
      "do_alpha_test_thresholdDeg must not be negative, got {}",
      parameters.do_alpha_test_thresholdDeg);

  if (!(parameters.epsilon_theta >= 0.0))
    return std::format("epsilon_theta must not be negative, got {}",
      parameters.epsilon_theta);

  if (!(parameters.epsilon_xy >= 0.0))
    return std::format("epsilon_xy must not be negative, got {}",
      parameters.epsilon_xy);

  if (!(parameters.max_angular_correction_deg >= 0.0))
    return std::format(
      "max_angular_correction_deg must not be negative, got {}",
      parameters.max_angular_correction_deg);

  if (!(parameters.max_correspondence_dist > 0.0))
    return std::format(
      "max_correspondence_dist must be greater than zero, got {}",
      parameters.max_correspondence_dist);

  if (parameters.max_iterations <= 0)
    return std::format("max_iterations must be greater than zero, got {}",
      parameters.max_iterations);

  if (!(parameters.max_linear_correction >= 0.0))
    return std::format("max_linear_correction must not be negative, got {}",
      parameters.max_linear_correction);

  if (parameters.orientation_neighbourhood <= 0
    || parameters.orientation_neighbourhood > kMostRays)
  {
    return std::format(
      "orientation_neighbourhood must be between one and {}, got {}",
      kMostRays, parameters.orientation_neighbourhood);
  }

  if (!(parameters.outliers_adaptive_mult >= 0.0))
    return std::format("outliers_adaptive_mult must not be negative, got {}",
      parameters.outliers_adaptive_mult);

  if (!(parameters.outliers_adaptive_order > 0.0)
    || parameters.outliers_adaptive_order > 1.0)
  {
    return std::format(
      "outliers_adaptive_order must be greater than zero and at most one, "
      "got {}", parameters.outliers_adaptive_order);
  }

  if (!(parameters.outliers_maxPerc > 0.0) || parameters.outliers_maxPerc > 1.0)
    return std::format(
      "outliers_maxPerc must be greater than zero and at most one, got {}",
      parameters.outliers_maxPerc);

  if (!(parameters.restart_dt >= 0.0))
    return std::format("restart_dt must not be negative, got {}",
      parameters.restart_dt);

  if (!(parameters.restart_dtheta >= 0.0))
    return std::format("restart_dtheta must not be negative, got {}",
      parameters.restart_dtheta);

  if (!(parameters.restart_threshold_mean_error >= 0.0))
    return std::format(
      "restart_threshold_mean_error must not be negative, got {}",
      parameters.restart_threshold_mean_error);

  if (!(parameters.sigma > 0.0))
    return std::format("sigma must be greater than zero, got {}",
      parameters.sigma);

  if (parameters.use_sigma_weights)
    return "use_sigma_weights must be off, got true, because nothing fills the "
      "per reading deviation it weights each correspondence by";

  return {};
}

IcpRefiner::IcpRefiner(const IcpParameters& parameters,
  std::shared_ptr<const FSM::GridReference> map)
: map_(std::move(map)),
  parameters_(parameters)
{
}

/**
 * @details One cast, at the estimate, of exactly as many rays as the scan was
 * handed with. That cast is the reference the solver works against from its
 * first cycle to its last, and the map is not read again for the rest of the
 * call. The alternative, casting again wherever the solver moves the scan to,
 * is what `fsm_localisation::Corrector` does and is deliberately not what
 * happens here.
 * The solver is given both scans in the sensor's frame with the reference
 * sitting at the origin and a first guess of nothing, and it answers with the
 * pose of the measured scan's frame expressed in the reference scan's frame.
 * The reference was cast at the estimate, so that frame is the estimate's own,
 * and the answer composes straight onto the estimate: the answer's translation
 * turned by the estimate's heading and added to the estimate's position, the
 * answer's rotation added to the estimate's heading. Nothing is sandwiched
 * between a pair of sensor to base transforms, as the ROS 1 build does,
 * because a sensor pose went in and a sensor pose comes out.
 * Every refusal the scan itself earns is settled before the map is touched, a
 * cast being the expensive half of a refinement and a refusal costing nothing.
 * The cast is then checked for having come back at the length it was asked
 * for: the caster's own interface promises no length, and a cast of another
 * length would put the two scans at different angular spacings and give a
 * wrong answer with nothing at all to show for it.
 * The answer is checked for being a number before it is handed back. The
 * solver can report success and still hand over an answer that is not one, and
 * a pose of not a number spreads into whatever a host does with it, so it is
 * refused here as a failed match rather than passed on.
 * The three covariance matrices the solver may hang on its result are released
 * by the holder that owns it, whatever the solver said and whether or not it
 * was asked to compute them. The ROS 1 build releases the previous call's
 * before it starts the next one, which comes to the same thing there because
 * it keeps one result across every call and here because the result does not
 * outlive the call.
 */
std::expected<IcpResult, IcpError> IcpRefiner::refine(
  const std::span<const double> ranges, const Pose& estimate,
  const double range_min, const double range_max) const
{
  if (ranges.empty())
    return std::unexpected(IcpError::scan_empty);

  if (std::cmp_less(ranges.size(), kFewestRays))
    return std::unexpected(IcpError::scan_too_short);

  if (std::cmp_greater(ranges.size(), kMostRays))
    return std::unexpected(IcpError::scan_too_long);

  const LaserDataHandle measured = laserDataOf(ranges, range_min, range_max);

  const std::span<const int> validity(measured->valid,
    static_cast<std::size_t>(measured->nrays));

  if (std::ranges::none_of(validity, [](const int valid) { return valid == 1; }))
    return std::unexpected(IcpError::scan_entirely_invalid);

  const std::vector<double> cast = map_->ranges(estimate,
    static_cast<unsigned int>(ranges.size()), range_max);

  if (cast.size() != ranges.size())
    return std::unexpected(IcpError::cast_length_mismatch);

  const LaserDataHandle reference = laserDataOf(cast, range_min, range_max);

  sm_params input = matcherInputOf(parameters_, range_min, range_max);
  input.laser_ref = reference.get();
  input.laser_sens = measured.get();
  input.first_guess[0] = 0.0;
  input.first_guess[1] = 0.0;
  input.first_guess[2] = 0.0;

  MatcherResult holder;
  sm_result& output = holder.get();

  sm_icp(&input, &output);

  if (output.valid != 1)
    return std::unexpected(IcpError::matching_failed);

  const IcpResult result{
    Pose{
      estimate.x + output.x[0] * std::cos(estimate.t)
        - output.x[1] * std::sin(estimate.t),
      estimate.y + output.x[0] * std::sin(estimate.t)
        + output.x[1] * std::cos(estimate.t),
      wrapped(estimate.t + output.x[2])},
    output.error,
    output.iterations,
    output.nvalid};

  if (!std::isfinite(result.corrected.x) || !std::isfinite(result.corrected.y)
    || !std::isfinite(result.corrected.t) || !std::isfinite(result.error))
  {
    return std::unexpected(IcpError::matching_failed);
  }

  return result;
}

}
