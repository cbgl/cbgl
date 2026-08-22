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
 * @details
 * Every setting the ROS 1 build reads for this solver and that this solver
 * still carries is here, under the name that build reads it by and with the
 * value that build falls back to when it is not given one. The settings file
 * that build ships overrides most of them; the defaults below are the code's
 * own, so that a host that says nothing gets what the ROS 1 code gets when it
 * is told nothing.
 * A setting the solver takes as nought or one is a `bool` here rather than an
 * `int`, since that is what it means and what a host would declare it as. The
 * two spellings with a capital in them keep it, the name being the point.
 * @remark The number of rays a match runs at is not here. It is the length of
 * the scan handed to `IcpRefiner::refine`, which takes the cast to that same
 * length and refuses a cast that comes back at any other, so the two scans
 * cannot disagree about it.
 * @remark `orientation_neighbourhood` is bounded above as well as below. The
 * solver turns it into an allocation of twice its size, once for every ray of
 * every scan, and an allocation it cannot satisfy throws out through a stack
 * the solver does not unwind, leaving one level of its own context stack held
 * for the rest of the process.
 * @remark `use_sigma_weights` is refused where it is on. It weights each
 * correspondence by a per reading deviation that nothing here fills, so it
 * cannot do what its name says and the solver complains once per
 * correspondence instead of weighting anything.
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
 * @details
 * One cast, taken once, at the estimate. The solver then walks the measured
 * scan onto that one frozen cast and reports how far it had to move it. This
 * is the whole of the difference between this and
 * `fsm_localisation::Corrector`, which casts afresh at every pose it tries:
 * there the map is consulted throughout, here it is consulted once and the
 * rest of the work is between two scans. A refinement that cast again per
 * iteration would be a different algorithm wearing this one's name.
 * Nothing this library holds survives a refinement, and two refinements in
 * either order give the same two answers. The solver underneath is another
 * matter: it keeps a context stack, the counters that go with it and every
 * matrix it has ever allocated in process globals, and reuses them from one
 * call to the next. None of that shows in the answers, which is why the two
 * sentences above still stand, and all of it is why the first remark below
 * is not optional.
 * The map is borrowed through `FSM::GridReference`, which is what keeps this
 * file and its implementation free of any ROS header: a host reads an
 * occupancy grid message into something that stands behind that interface and
 * hands it over. The shared pointer is held for the life of the refiner, so a
 * host replacing the map constructs a second refiner rather than editing this
 * one's.
 * The accuracy an answer can reach is bounded by the map and not by the
 * solver. A ray stops at a cell, so a map of five centimetre cells cannot
 * support an answer better than five centimetres however good the match is.
 * @remark `refine` must not be called concurrently, on this refiner or on any
 * other one in the same process. The state the solver keeps is process wide
 * rather than per instance and nothing guards it, so two calls running at once
 * can be handed each other's matrices. Refine candidates one at a time.
 * @remark The solver this is linked against must carry the correction to its
 * neighbour gathering bounds. Without it the downward walk takes one step more
 * than its buffer holds, and at a neighbourhood of twenty, which is the value
 * shipped, that is a forty first entry written into a buffer of forty. The
 * path is entered whenever `do_alpha_test` is on, which is how the ROS 1
 * settings ship it, so it is the ordinary configuration that overruns rather
 * than an unusual corner of it.
 */
class IcpRefiner
{
public:
  /**
   * @brief Construct a refiner over a settings set and the map to refine
   * against.
   * @remark `map` must not be null. Every refinement dereferences it and
   * nothing checks it, a map being something a host settles once at
   * construction rather than something a single refinement could refuse and be
   * asked again about.
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
   * match is to run at. The cast is taken to that same number, so ray `i` of
   * either scan sits at `-pi + i * 2 * pi / n` in the sensor's own frame and
   * the two share one angle array by construction.
   * @param estimate where the sensor is thought to have been when that scan
   * was taken, in the frame the map is given in.
   * @param range_min the scan's own minimum range. A reading under it is
   * carried into the match as unusable rather than as a distance.
   * @param range_max the scan's own maximum range, which is what a ray meeting
   * nothing reports. The map has never seen either bound, so both are given
   * per refinement rather than held. A `range_max` of a hundred metres or more
   * is accepted, but no reading at or above a hundred is carried into the match
   * as a distance, because the solver refuses outright any scan that offers it
   * one. A lidar whose far bound is that high marks its own far bound usable,
   * and every refinement of it would otherwise be refused with nothing to read
   * a cause out of. Such a reading is marked unusable, exactly as a reading
   * past the far bound already is, and the rest of the scan is matched as it
   * stands.
   * @remark The solver writes to standard error on its own account whenever it
   * refuses, in terminal colour where the terminal looks capable of it, and
   * carries no setting that silences it. A host refining many candidates should
   * expect output on its error stream that its own logger never passed through.
   * @remark The pose is a **sensor** pose, in and out. The ROS 1 build refines
   * a robot pose and so wraps the solver's answer between the sensor to base
   * transforms before applying it. There is nothing to wrap it between here.
   */
  std::expected<IcpResult, IcpError> refine(std::span<const double> ranges,
    const Pose& estimate, double range_min, double range_max) const;

  const IcpParameters& parameters() const { return parameters_; }

private:
  const std::shared_ptr<const FSM::GridReference> map_;
  const IcpParameters parameters_;
};

}

#endif
