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

#ifndef CBGL__ICP_REFINEMENT_HPP_
#define CBGL__ICP_REFINEMENT_HPP_

#include <cmath>
#include <expected>
#include <memory>
#include <span>
#include <string>

#include <fsm_localisation/fsm_core.hpp>

namespace cbgl
{
/**
 * @brief A planar pose.
 * The ray caster carries the same type, so this is an alias rather than a
 * second declaration, and it names the very type `cbgl.hpp` names by the same
 * word.
 */
using Pose = FSM::Pose;

/**
 * @brief Everything the iterative closest point solver needs, and nothing
 * about ROS.
 * @details Every setting the ROS 1 build reads for this solver and that this
 * solver still carries, under that build's own name and with its own code
 * fallback, so a host that says nothing gets what that code gets when told
 * nothing. A setting the solver takes as nought or one is a `bool` here.
 * @remark The ray count is not here. It is the length of the scan handed to
 * `IcpRefiner::refine`, and the cast is taken to the same length.
 */
struct IcpParameters
{
  double clustering_threshold{0.25};

  bool debug_verify_tricks{false};

  bool do_alpha_test{false};

  double do_alpha_test_thresholdDeg{20.0};

  bool do_compute_covariance{false};

  bool do_visibility_test{false};

  double epsilon_theta{0.000001};

  double epsilon_xy{0.000001};

  double max_angular_correction_deg{45.0};

  double max_correspondence_dist{0.3};

  int max_iterations{10};

  double max_linear_correction{0.50};

  int orientation_neighbourhood{20};

  double outliers_adaptive_mult{2.0};

  double outliers_adaptive_order{0.7};

  double outliers_maxPerc{0.90};

  bool outliers_remove_doubles{true};

  bool restart{false};

  double restart_dt{1.0};

  double restart_dtheta{0.1};

  double restart_threshold_mean_error{0.01};

  double sigma{0.010};

  bool use_corr_tricks{true};

  bool use_ml_weights{false};

  bool use_point_to_line_distance{true};

  bool use_sigma_weights{false};
};

/**
 * @brief What one successful refinement yields.
 * @details `corrected` is where the sensor is now thought to have been.
 * `error` is the solver's total correspondence error, `iterations` how many
 * cycles it spent, and `num_valid` how many correspondences it was still
 * holding when it stopped. The last three say nothing on their own and mean
 * something compared against the same three from another candidate of the same
 * scan, which is what a host reports them for.
 */
struct IcpResult
{
  Pose corrected;

  double error{0.0};

  int iterations{0};

  int num_valid{0};
};

/**
 * @brief Why an estimate was not refined.
 * @details `matching_failed` is the solver's own refusal, which it gives for a
 * pair of scans it could not hold enough correspondences between, and which
 * the ROS 1 build answers by keeping the estimate it started from. It also
 * covers an answer that came back as not a number, which is no more usable
 * than no answer at all. Every other reason is settled here, before the solver
 * is reached, so that a host can tell a scan it should not have sent from a
 * match that could not be made. What a host does with a refusal is the host's
 * to decide, so the estimate is not handed back here dressed as an answer.
 * @remark `scan_too_short` and `scan_too_long` are the solver's own ray count
 * bounds, ten and ten thousand, refused up front rather than left to arrive as
 * a failed match a host cannot read a cause out of.
 * @remark `cast_length_mismatch` is a map that answered a request for `n`
 * ranges with some other number of them. The two scans would then sit at
 * different angular spacings and the answer would be wrong with nothing at all
 * to show for it, which is the one way this could be quietly wrong.
 */
enum class IcpError
{
  cast_length_mismatch,

  matching_failed,

  scan_empty,

  scan_entirely_invalid,

  scan_too_long,

  scan_too_short,
};

/**
 * @brief Validate a settings set.
 * @return an empty string when the settings are usable, otherwise a sentence
 * naming the offending setting and its value.
 */
std::string validate(const IcpParameters& parameters);

/**
 * @brief Refinement of a pose estimate against a map, by iterative closest
 * point between the measured scan and one cast taken where the estimate says.
 * @details One cast, taken once, at the estimate. The solver then walks the
 * measured scan onto that frozen cast. `fsm_localisation::Corrector` casts
 * afresh at every pose it tries; this consults the map once and the rest of
 * the work is between two scans.
 * The map is borrowed through `FSM::GridReference`, which keeps this free of
 * any ROS header, and is held for the life of the refiner, so replacing it
 * means a second refiner.
 * The accuracy is bounded by the map: a ray stops at a cell, so a five
 * centimetre grid cannot support a better answer than five centimetres.
 * @remark `refine` must not be called concurrently, on this refiner or any
 * other in the process. The solver keeps its context stack, its counters and
 * every matrix it has allocated in process globals and guards none of it, so
 * two calls at once can be handed each other's matrices.
 * @remark The solver linked against must carry the correction to its neighbour
 * gathering bounds. Without it the downward walk takes one step more than its
 * buffer holds, which at the shipped neighbourhood of twenty writes a forty
 * first entry into a buffer of forty. The path is entered whenever
 * `do_alpha_test` is on, which is how the ROS 1 settings ship it.
 */
class IcpRefiner
{
public:
  /**
   * @brief Construct a refiner over a settings set and the map to refine
   * against.
   * @remark `map` must not be null. Every refinement dereferences it and
   * nothing checks it.
   */
  IcpRefiner(const IcpParameters& parameters,
    std::shared_ptr<const FSM::GridReference> map);

  IcpRefiner(const IcpRefiner&) = delete;
  IcpRefiner& operator=(const IcpRefiner&) = delete;
  IcpRefiner(IcpRefiner&&) = delete;
  IcpRefiner& operator=(IcpRefiner&&) = delete;

  /**
   * @brief Refine one pose estimate against the map.
   * @param ranges the measured scan, already reduced to the number of rays the
   * match is to run at. Ray `i` of either scan sits at
   * `-aperture / 2 + i * aperture / n` in the sensor's own frame, so the two
   * share one angle array by construction.
   * @param estimate where the sensor is thought to have been when that scan
   * was taken, in the frame the map is given in.
   * @param range_min the scan's own minimum range. A reading under it is
   * carried into the match as unusable rather than as a distance.
   * @param range_max the scan's own maximum range, which is what a ray meeting
   * nothing reports. The map has never seen either bound, so both are given
   * per refinement rather than held.
   * @param aperture how much of a turn the scan covers, in radians. A whole
   * turn is the default. A narrower sensor is served by casting the whole turn
   * at the sensor's own angular resolution and matching against the middle
   * window of that cast; the cast is still taken once, at the estimate.
   * @remark A reading at or above a hundred metres is marked unusable rather
   * than carried in as a distance. The solver refuses outright any scan that
   * offers it one, so a lidar whose far bound is that high would otherwise
   * have every refinement refused with nothing to read a cause from.
   * @remark The solver writes to standard error whenever it refuses, in colour
   * where the terminal allows, and carries no setting that silences it.
   * @remark The pose is a **sensor** pose, in and out. The ROS 1 build refines
   * a robot pose and so wraps the answer between the sensor to base
   * transforms; there is nothing to wrap it between here.
   */
  std::expected<IcpResult, IcpError> refine(std::span<const double> ranges,
    const Pose& estimate, double range_min, double range_max,
    double aperture = 2.0 * M_PI) const;

  const IcpParameters& parameters() const { return parameters_; }

private:
  const std::shared_ptr<const FSM::GridReference> map_;
  const IcpParameters parameters_;
};

}

#endif
