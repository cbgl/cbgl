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

#ifndef CBGL__CBGL_HPP_
#define CBGL__CBGL_HPP_

#include <cmath>
#include <cstddef>
#include <expected>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <fsm_localisation/fsm_localisation.hpp>

#include "cbgl/icp_refinement.hpp"

namespace cbgl
{
/**
 * @brief A planar pose.
 * The matcher carries the same type, so this is an alias rather than a second
 * declaration: the two layers hand poses to each other unconverted.
 */
using Pose = fsm_localisation::Pose;

/**
 * @brief A position in the frame the map's origin is given in.
 * @remark `fsm_localisation::WorldPoint` says the same thing and is not used
 * here, because reaching it would put an occupancy grid message in front of a
 * library that must not know what a message is.
 */
struct Point
{
  double x{0.0};
  double y{0.0};
};

/**
 * @brief The free space a map offers: the world position of every free cell,
 * and the resolution those cells were drawn at.
 * @details The free area a hypothesis count is proportional to is the number of
 * cells times the square of the resolution, so both halves are needed and
 * neither is recoverable from the other.
 */
struct FreeSpace
{
  std::vector<Point> cells;

  double resolution{0.0};
};

/**
 * @brief Everything the algorithm needs, and nothing about ROS.
 * @details `dl` is how many hypothesis positions are drawn per square metre of
 * free space, `da` how many candidate orientations each of those positions is
 * scored at, `top_k_caers` how many of the scored candidates are refined,
 * `num_threads` the concurrency of the scoring where zero means what the
 * hardware offers, and `rng_seed` the seed of the draw where zero means
 * hardware entropy.
 * @remark The matcher's own settings are embedded rather than restated, so that
 * one struct carries them and one validator refuses them. The occupancy
 * threshold a map is built at and the name of the caster it is built with are
 * among them, since the matcher already carries both.
 * @remark `icp` is what chooses the refinement. Settings present refine every
 * survivor by iterative closest point and settings absent refine it by the
 * matcher, so the choice and the settings that go with it are one field and
 * cannot contradict each other the way a pair of switches could. Absent is the
 * default, and a host that says nothing refines as this package always has.
 * @remark `validate` below does not judge `icp`, though it judges `matcher`.
 * `validate(const IcpParameters&)` judges it, and a host that fills the field
 * calls that itself before it builds a localiser, so that a setting the solver
 * cannot use is refused at start up by name and by value rather than
 * complained about on the error stream once for every reading of every
 * candidate.
 */
struct Parameters
{
  unsigned int da{32};

  double dl{15.0};

  std::optional<IcpParameters> icp;

  fsm_localisation::Parameters matcher;

  unsigned int num_threads{0};

  unsigned int rng_seed{0};

  unsigned int top_k_caers{10};
};

/**
 * @brief One of the `da` orientations a single panoramic cast yields, and the
 * ranges that orientation would see.
 */
struct Candidate
{
  Pose pose;
  std::vector<double> ranges;
};

/**
 * @brief A hypothesis and what the metric made of it. Lower scores better.
 */
struct ScoredPose
{
  Pose pose;
  double score{0.0};
};

/**
 * @brief What one successful localisation yields.
 * @details `pose` is where the **sensor** was, not the robot: the sensor is
 * what casts and the sensor is what the map's free space applies to, so
 * deriving the robot's pose from it is the host's last step and not this
 * library's. `num_candidates` is how many candidates the metric scored,
 * `survivors` the cheapest of them in ranking order, and `refusals` how many
 * survivors the matcher refused, counted by the reason it gave. The number that
 * survived the sift is `survivors.size()`, which is smaller than `top_k_caers`
 * asked for wherever the map offered fewer candidates than that.
 * @remark `icp_refusals` is the same count for the other refinement. One
 * refinement runs per localisation, so at most one of the two maps is ever
 * filled, and they are two maps rather than one because the two refinements
 * refuse for reasons that do not correspond. A count under one heading is not
 * the same fact under the other and is not put there.
 */
struct Result
{
  Pose pose;

  double execution_time{0.0};

  std::size_t num_candidates{0};

  std::vector<ScoredPose> survivors;

  std::map<IcpError, unsigned int> icp_refusals;

  std::map<fsm_localisation::CorrectionError, unsigned int> refusals;
};

/**
 * @brief Why no pose was produced.
 * @details Only the refusals this library decides are here. A host refuses a
 * run of its own accord too, for want of a map, a scan, or a place to put the
 * answer, and says so in its own words.
 * @remark `refusals` is filled for `Reason::every_survivor_refused` and for
 * `Reason::no_scorable_pose`, where a run can still have refused some
 * survivors before scoring the rest, and is empty for the remaining reasons.
 * It carries the count per reason the matcher gave, so that a host can say
 * which way the refinement went wrong rather than only that it did.
 * @remark `icp_refusals` carries the same count for the other refinement,
 * under its own reasons and for the same two cases. At most one of the two is
 * ever filled, one refinement running per localisation.
 */
struct Error
{
  enum class Reason
  {
    every_survivor_refused,

    map_has_no_free_space,

    /**
     * @remark An extension beyond the written requirements. The four
     * argument `Localiser::localise` exists so a caller can hand in a
     * hypothesis set of its own, and an empty one is neither the map having
     * no free space nor a draw this library made badly. Raised only there,
     * before a single candidate is scored, and never by the overload that
     * disperses its own set.
     */
    no_hypotheses_given,

    /**
     * @remark An extension beyond the written requirements, and not
     * reachable through an occupancy grid: the matcher's translation stage
     * tests every step against `FSM::Utils::isPositionInMap`, which for a
     * grid reference is the grid's own `isFree`, so a step that would leave
     * free space or leave the grid triggers a recovery instead of being
     * accepted, and a corrected pose is therefore always on free space of
     * the same map, where a cast in a bounded room always meets a wall.
     * `Localiser` is built around `FSM::GridReference` rather than an
     * occupancy grid by name, though, so a caller may hand in something
     * else, and this is what stops such a reference from producing a
     * confident answer out of a cast that met nothing.
     */
    no_scorable_pose,

    scan_entirely_invalid,

    scan_too_short,
  };

  Reason reason{Reason::map_has_no_free_space};

  std::map<IcpError, unsigned int> icp_refusals;

  std::map<fsm_localisation::CorrectionError, unsigned int> refusals;
};

/**
 * @brief The cumulative absolute error per ray between a measurement and a cast.
 * @details The absolute difference between the two, summed over the rays where
 * both are greater than zero. It is not divided by the number of rays that
 * contributed, because the measurement is the same for every candidate a
 * ranking compares. Where `measured` and `cast` differ in length, only the
 * shorter's own number of rays is compared, deliberately, rather than looping
 * to a length neither is known to carry.
 * @param range_max the measurement's own maximum range, which is what a cast
 * ray meeting nothing reports.
 * @return the error, or infinity where the cast met nothing at all or shares no
 * contributing ray with the measurement. Infinity is what ranks such a
 * candidate behind every candidate that scored anything.
 */
double caer(std::span<const double> measured, std::span<const double> cast,
  double range_max) noexcept;

/**
 * @brief The `da` candidates one panoramic cast from `pose` yields.
 * @details A cast over a full turn taken at one orientation is a cast at any
 * other orientation of the same position, read from a different starting
 * sample. Candidate `k` is therefore the cast rotated by `round(k * n / da)`
 * samples, where `n` is the ray count, and it is labelled with the orientation
 * that rotation actually amounts to rather than with the one `k` alone would
 * suggest. The two differ, by up to half of a candidate's spacing, wherever
 * `da` does not divide `n`.
 * @remark This is the whole of the saving the algorithm rests on: `da`
 * orientations of a position cost one cast between them.
 * @param rays how many of the rotated cast a candidate keeps, counted from the
 * middle of it. Zero keeps the whole of it, which is what a sensor that sees
 * all the way round wants and is the only thing this had before a narrower one
 * was supported. A sensor that sees less is served by casting the whole turn at
 * its own angular resolution, which is more rays than the sensor has, and
 * keeping the middle window: the rotation is still over a full turn, so the
 * orientation a candidate is labelled with is unaffected, and only which part
 * of the turn is compared changes.
 */
std::vector<Candidate> candidatesOf(const Pose& pose,
  std::span<const double> cast, unsigned int da, std::size_t rays = 0);

/**
 * @brief Validate a parameter set.
 * @return an empty string when the parameters are usable, otherwise a sentence
 * naming the offending parameter and its value. The matcher's own settings are
 * refused by the matcher's own validator, not by a second copy of it.
 */
std::string validate(const Parameters& parameters);

/**
 * @brief One-shot global localisation of a panoramic 2D lidar in an occupancy
 * grid.
 * @details
 * Where the matcher corrects an estimate that is already roughly right, this
 * needs no estimate at all. It disperses hypothesis poses over the map's free
 * space, casts one panoramic scan per hypothesis position and scores every
 * orientation that cast yields against the measurement, refines the cheapest
 * few by handing them to the matcher, and returns whichever of the refined
 * poses the same score likes best.
 * Nothing survives a call. Two localisations in either order give the same two
 * answers, and a localisation is repeatable exactly by pinning `rng_seed`.
 * The map is borrowed through `FSM::GridReference`, which is what keeps this
 * file and its implementation free of any ROS header: a host reads an occupancy
 * grid message into something that stands behind that interface and hands it
 * over. The shared pointer is held for the life of the localiser, so a host
 * replacing the map constructs a second localiser rather than editing this
 * one's.
 * The accuracy an answer can reach is bounded by the map and not by the
 * algorithm. A ray stops at a cell, so a map of five centimetre cells cannot
 * support an answer better than five centimetres however good the search is.
 * One of two refinements is built at construction and it is the settings that
 * say which: the matcher, which is what this package has always refined with,
 * or the iterative closest point refinement, which is held beside it and
 * constructed only where it is asked for. Nothing stands over the two of them.
 * A run branches once, before it walks the survivors, and a run that refines
 * with the matcher costs no more at run time than it did before the other one
 * existed.
 */
class Localiser
{
public:
  Localiser(const Parameters& parameters,
    std::shared_ptr<const FSM::GridReference> map,
    FreeSpace free_space);

  Localiser(const Localiser&) = delete;
  Localiser& operator=(const Localiser&) = delete;
  Localiser(Localiser&&) = delete;
  Localiser& operator=(Localiser&&) = delete;

  /**
   * @brief Draw the hypothesis set the search would start from.
   * @details `round(dl * free_area)` positions, each the centre of a free cell
   * chosen uniformly at random, each carrying an orientation drawn uniformly on
   * `[-pi, pi)`. At least one position is drawn wherever the map offers a free
   * cell, and none where it offers none.
   * @remark The poses are **sensor** poses. Drawing robot poses over free cells
   * and offsetting to the sensor afterwards can put the sensor, which is what
   * casts, on a blocked cell.
   */
  std::vector<Pose> disperse() const;

  /**
   * @brief Localise from a hypothesis set of the caller's own.
   * @param ranges the scan as the sensor reported it, unfilled and unreduced.
   * @param range_min the scan's own minimum range, under which a reading is
   * not a distance. It reaches the iterative closest point refinement, which
   * marks a reading under it unusable, and it reaches nothing else: the sift,
   * the matcher and the selection each judge a reading by the matcher's own
   * test of it, which knows no lower bound. So a localisation that refines
   * with the matcher gives the same answer whatever this is.
   * @param range_max the scan's own maximum range, which is what a ray meeting
   * nothing reports. The map has never seen either bound, so both are given
   * per localisation rather than held.
   * @param aperture how much of a turn the scan covers, in radians. A sensor
   * that sees all the way round is the default and is what this package has
   * always localised. A narrower one is served by casting the whole turn at the
   * sensor's own angular resolution, which is more rays than the sensor
   * carries, and comparing only the middle window of that cast. The rotation
   * that yields a candidate is still over the whole turn, so what a candidate
   * is labelled with does not change; only how much of the turn is compared
   * does.
   * @remark A narrower sensor is supported here and is not supported by the
   * node above, which still refuses one. Restoring that is a separate piece of
   * work and this is the half of it that belongs to the algorithm.
   * @remark Handing the set in rather than drawing it is what lets a test start
   * from a hypothesis set whose answer is known, and what lets two builds be
   * compared on everything downstream of the draw.
   */
  std::expected<Result, Error> localise(std::span<const double> ranges,
    double range_min, double range_max,
    std::span<const Pose> hypotheses,
    double aperture = 2.0 * M_PI) const;

  /**
   * @brief Localise from a hypothesis set of this localiser's own drawing.
   * @details `disperse` and then the localisation above, which is the whole of
   * the difference between the two.
   */
  std::expected<Result, Error> localise(std::span<const double> ranges,
    double range_min, double range_max,
    double aperture = 2.0 * M_PI) const;

private:
  const fsm_localisation::Corrector corrector_;
  const FreeSpace free_space_;

  /**
   * @brief The other refinement, where the settings asked for it, and nothing
   * where they did not.
   * @remark Held rather than pointed at, so that a localisation that refines
   * with the matcher reaches no allocation and no indirection this adds.
   */
  std::optional<IcpRefiner> icp_refiner_;

  const std::shared_ptr<const FSM::GridReference> map_;
  const Parameters parameters_;
};

}

#endif
