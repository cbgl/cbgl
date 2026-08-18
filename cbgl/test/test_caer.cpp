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
 * @brief `cbgl::caer`: the metric every ranking in this algorithm is decided
 * by, and the four things about it that are easy to get wrong.
 * @details
 * The metric is the cumulative absolute error per ray between a measurement
 * and a cast: the absolute difference between the two, summed over the rays
 * where both are greater than zero.
 * **A ray either side calls invalid contributes nothing.** A lidar reports a
 * ray it could not trust as a range that is not greater than zero, and a cast
 * that met nothing reports the same. Counting such a ray would score the
 * absence of a measurement as though it were a disagreement of the whole
 * range.
 * **A cast that met nothing at all is worst.** So is a cast that shares no
 * contributing ray with the measurement. Both are answers the metric cannot
 * grade, and both must rank behind every candidate that scored anything at
 * all, which is what infinity does and what a zero or a large finite number
 * would not.
 * **It is not divided by the number of contributing rays.** The measurement is
 * the same for every candidate a ranking compares, so dividing changes the
 * ranking rather than merely rescaling it: it would reward a candidate for
 * agreeing with fewer rays. `TheScoreIsNotDividedByTheNumberOfContributingRays`
 * is the case that says so, by holding two casts that disagree by the same
 * amount on every contributing ray against each other and requiring the one
 * with twice as many contributing rays to score twice as much.
 * **It is compared over the rays the two spans share.** The ROS 1 build looped
 * over both to a length neither was known to have. The cast and the measurement
 * are the same length by construction in a run, but the metric is a public
 * function and a caller may hand it any two spans, so taking the shorter of the
 * two is what keeps that safe.
 * Every expected value below is written out as the arithmetic that produces
 * it, in the case that uses it, so that a reader can add the terms up by hand
 * rather than trust a number.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "cbgl/cbgl.hpp"

namespace
{
const double kRangeMax = 10.0;

/**
 * @brief A range a lidar or a caster reports for a ray it has nothing to say
 * about. Both of these fail the metric's own test of being greater than zero.
 */
class CaerInvalidRange : public testing::TestWithParam<double>
{
};

std::string invalidRangeSuffix(const testing::TestParamInfo<double>& info)
{
  return info.param == 0.0 ? "zero" : "negative";
}

}

TEST(Caer, ACastEntirelyAtTheMeasurementsMaximumRangeIsRankedBehindAnythingThatScored)
{
  const std::vector<double> measured{3.0, 4.0, 5.0, 6.0};
  const std::vector<double> met_nothing(measured.size(), kRangeMax);
  const std::vector<double> met_something{
    measured[0] + 1.0, measured[1] + 1.0, measured[2] + 1.0, measured[3] + 1.0};

  const double nothing = cbgl::caer(measured, met_nothing, kRangeMax);
  const double something = cbgl::caer(measured, met_something, kRangeMax);

  EXPECT_EQ(something, 4.0 * 1.0)
    << "a cast a whole metre beyond the measurement on each of its four rays "
    << "scores four metres, which is the score the case below is held "
    << "against.";

  EXPECT_TRUE(std::isinf(nothing))
    << "a cast whose every ray came back at the measurement's own maximum "
    << "range met nothing at all, so there is nothing to grade it on. It must "
    << "score infinity rather than the " << nothing << " it scored.";

  EXPECT_GT(nothing, something)
    << "a candidate that met nothing must rank behind one that met something, "
    << "however badly the second agreed with the measurement.";
}

TEST(Caer, ACastSharingNoContributingRayWithTheMeasurementIsRankedBehindAnythingThatScored)
{
  const std::vector<double> measured{3.0, 0.0, 5.0, 0.0};
  const std::vector<double> disjoint{0.0, 4.0, 0.0, 6.0};
  const std::vector<double> overlapping{3.5, 4.0, 0.0, 6.0};

  const double shared_nothing = cbgl::caer(measured, disjoint, kRangeMax);
  const double shared_one = cbgl::caer(measured, overlapping, kRangeMax);

  EXPECT_EQ(shared_one, std::abs(3.5 - 3.0))
    << "only ray zero is greater than zero on both sides, so the score is "
    << "that one ray's own disagreement and nothing else.";

  EXPECT_TRUE(std::isinf(shared_nothing))
    << "the two vectors are valid on opposite rays, so not one ray "
    << "contributes and there is nothing to grade the cast on. It must score "
    << "infinity rather than the " << shared_nothing << " it scored, which is "
    << "what a sum over an empty set would give and what would rank this "
    << "candidate ahead of every candidate that agreed with the measurement.";

  EXPECT_GT(shared_nothing, shared_one)
    << "a candidate sharing no contributing ray must rank behind one sharing "
    << "a single ray.";
}

TEST(Caer, ACastThatAgreesEverywhereScoresNothing)
{
  const std::vector<double> measured{1.25, 2.5, 3.75, 5.0};

  EXPECT_EQ(cbgl::caer(measured, measured, kRangeMax), 0.0)
    << "a cast identical to the measurement disagrees with it on no ray, so "
    << "the sum of the disagreements is zero. This is the score the winner of "
    << "a search over a map the measurement was cast from should approach, so "
    << "a metric that could not reach zero would have no floor to rank "
    << "against.";
}

/**
 * @brief A measurement and a cast of different lengths are compared over the
 * rays they share, and neither is read past its own end.
 * @details The ROS 1 build looped over both vectors to a length neither of them
 * was known to have, so a cast shorter than the measurement was read past its
 * end. The two are the same length by construction here, the cast being taken
 * at the measurement's own ray count, but the metric is a public function and a
 * caller may hand it any two spans. Comparing over the shorter of the two is
 * what makes that safe, and it is what this case pins.
 */
TEST(Caer, AMeasurementAndACastOfDifferentLengthsAreComparedOverTheShorter)
{
  const std::vector<double> measured{2.0, 4.0, 6.0, 8.0};
  const std::vector<double> short_cast{2.5, 3.0};

  const double over_the_shared_rays =
    std::abs(2.5 - 2.0) + std::abs(3.0 - 4.0);

  EXPECT_EQ(cbgl::caer(measured, short_cast, kRangeMax), over_the_shared_rays)
    << "the cast carries " << short_cast.size() << " rays against the "
    << "measurement's " << measured.size() << ", so only the first "
    << short_cast.size() << " can be compared and the score is "
    << over_the_shared_rays << ". Reading to the measurement's length instead "
    << "would walk off the end of the cast, which is the defect this replaces.";

  EXPECT_EQ(cbgl::caer(short_cast, measured, kRangeMax), over_the_shared_rays)
    << "the same two spans the other way round share the same rays, so they "
    << "must score the same. A metric that took its ray count from whichever "
    << "span came first would disagree with itself here.";

  EXPECT_EQ(cbgl::caer(measured, std::span<const double>{}, kRangeMax),
    std::numeric_limits<double>::infinity())
    << "a cast of no rays at all shares no contributing ray with the "
    << "measurement, so it ranks behind every cast that scored anything.";

  EXPECT_EQ(cbgl::caer(std::span<const double>{}, measured, kRangeMax),
    std::numeric_limits<double>::infinity())
    << "a measurement of no rays at all contributes nothing either, and the "
    << "same reasoning applies to it.";
}

TEST(Caer, EveryContributingRayAddsItsOwnAbsoluteDifference)
{
  const std::vector<double> measured{2.0, 4.0, 6.0, 8.0};
  const std::vector<double> cast{2.5, 3.0, 6.0, 9.75};

  const double expected =
    std::abs(2.5 - 2.0) + std::abs(3.0 - 4.0) + std::abs(6.0 - 6.0) +
    std::abs(9.75 - 8.0);

  EXPECT_EQ(cbgl::caer(measured, cast, kRangeMax), expected)
    << "the score is the sum of the four rays' own absolute differences, "
    << "0.5 + 1.0 + 0.0 + 1.75, which is " << expected << ". A ray that "
    << "overshoots and a ray that undershoots by the same amount must cost "
    << "the same, so a signed sum would cancel the two and score this cast at "
    << "1.25 instead.";
}

TEST(Caer, TheScoreIsNotDividedByTheNumberOfContributingRays)
{
  const double error_per_ray = 1.5;

  const std::vector<double> measured{2.0, 2.0, 2.0, 2.0};
  const std::vector<double> four_contributing{
    2.0 + error_per_ray, 2.0 + error_per_ray,
    2.0 + error_per_ray, 2.0 + error_per_ray};
  const std::vector<double> two_contributing{
    2.0 + error_per_ray, 2.0 + error_per_ray, 0.0, 0.0};

  const double over_four = cbgl::caer(measured, four_contributing, kRangeMax);
  const double over_two = cbgl::caer(measured, two_contributing, kRangeMax);

  EXPECT_EQ(over_four, 4.0 * error_per_ray)
    << "four rays disagreeing by " << error_per_ray << " each sum to "
    << 4.0 * error_per_ray << ". A metric divided by the number of "
    << "contributing rays would report " << error_per_ray << " instead.";

  EXPECT_EQ(over_two, 2.0 * error_per_ray)
    << "two contributing rays disagreeing by " << error_per_ray << " each sum "
    << "to " << 2.0 * error_per_ray << ".";

  EXPECT_EQ(over_four, 2.0 * over_two)
    << "the same disagreement on twice as many rays must cost twice as much. "
    << "Divided by the number of contributing rays the two would score the "
    << "same, and a candidate agreeing with half the measurement would rank "
    << "level with one agreeing with all of it.";
}

TEST_P(CaerInvalidRange, ARayTheCastCallsInvalidContributesNothing)
{
  const double invalid = GetParam();

  const std::vector<double> measured{2.0, 4.0, 6.0};
  const std::vector<double> cast{2.5, invalid, 6.5};

  const double expected = std::abs(2.5 - 2.0) + std::abs(6.5 - 6.0);

  EXPECT_EQ(cbgl::caer(measured, cast, kRangeMax), expected)
    << "ray one of the cast came back at " << invalid << ", which is not "
    << "greater than zero, so the cast has nothing to say about it. The score "
    << "is rays zero and two alone, 0.5 + 0.5, which is " << expected << ". "
    << "Counting ray one would charge the cast "
    << std::abs(invalid - measured[1]) << " for a ray it never measured.";
}

TEST_P(CaerInvalidRange, ARayTheMeasurementCallsInvalidContributesNothing)
{
  const double invalid = GetParam();

  const std::vector<double> measured{2.0, invalid, 6.0};
  const std::vector<double> cast{2.5, 4.0, 6.5};

  const double expected = std::abs(2.5 - 2.0) + std::abs(6.5 - 6.0);

  EXPECT_EQ(cbgl::caer(measured, cast, kRangeMax), expected)
    << "ray one of the measurement is " << invalid << ", which is not greater "
    << "than zero, so the sensor had nothing to report on it. The score is "
    << "rays zero and two alone, 0.5 + 0.5, which is " << expected << ". "
    << "A scan carrying a handful of such rays is ordinary, and charging the "
    << "cast for each of them would rank candidates by where the sensor "
    << "failed rather than by where it was.";
}

INSTANTIATE_TEST_SUITE_P(
  NotGreaterThanZero, CaerInvalidRange,
  testing::Values(0.0, -0.5), invalidRangeSuffix);
