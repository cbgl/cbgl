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
 * @brief `cbgl::validate`: which parameter sets are usable, and what a refusal
 * has to say for itself.
 * @details
 * A parameter outside its permitted range is refused before a run starts, with
 * a sentence naming the setting and the value it was given, because that
 * sentence is the whole of what an operator reading a log has to work from. A
 * refusal that named neither would leave them to find the offending line by
 * bisection.
 * **The settings this library owns.** `dl` must be greater than zero, `da` and
 * `top_k_caers` must be at least one. A density of nothing draws no hypotheses,
 * no orientations scores nothing, and no survivors refines nothing: all three
 * refuse a run in a way that looks like a defect rather than a setting.
 * `rng_seed` and `num_threads` have no bad value at all, zero meaning hardware
 * entropy in the first and as many threads as the hardware offers in the
 * second.
 * **The matcher's settings are the matcher's business.** They are embedded
 * rather than restated, and they are judged by
 * `fsm_localisation::validate` rather than by a second copy of it. The cases
 * below assert that by requiring the two answers to be the same string, which
 * is the strongest form of "there is no second copy": a copy would have to
 * reproduce every sentence word for word to pass, and the first sentence either
 * side gained would fail.
 * **What is not asserted.** Nothing here fixes the wording of a refusal beyond
 * the setting's name and the value appearing in it. The sentences are for people
 * and are free to be improved.
 */

#include <gtest/gtest.h>

#include <functional>
#include <limits>
#include <string>

#include "cbgl/cbgl.hpp"

namespace
{
using cbgl::Parameters;

/**
 * @brief A setting put outside its permitted range, and the two things the
 * refusal has to name.
 */
struct Refusal
{
  const char* label{""};
  std::function<void(Parameters&)> apply;
  const char* setting{""};
  const char* value{""};
};

/**
 * @brief A setting put at a value that is permitted, however unusual it looks.
 */
struct Acceptance
{
  const char* label{""};
  std::function<void(Parameters&)> apply;
};

class AcceptedParameters : public testing::TestWithParam<Acceptance>
{
};

class MatcherRefusal : public testing::TestWithParam<Refusal>
{
};

class RefusedParameters : public testing::TestWithParam<Refusal>
{
};

std::string acceptanceSuffix(const testing::TestParamInfo<Acceptance>& info)
{
  return info.param.label;
}

std::string refusalSuffix(const testing::TestParamInfo<Refusal>& info)
{
  return info.param.label;
}

Parameters shippedParameters()
{
  Parameters parameters;

  parameters.dl = 15.0;
  parameters.da = 32U;
  parameters.top_k_caers = 10U;
  parameters.rng_seed = 0U;
  parameters.num_threads = 0U;

  parameters.matcher.size_scan = 360;
  parameters.matcher.min_magnification_size = 0U;
  parameters.matcher.max_magnification_size = 3U;
  parameters.matcher.num_iterations = 3U;
  parameters.matcher.xy_bound = 1.2;
  parameters.matcher.t_bound = 0.786;
  parameters.matcher.max_counter = 200U;
  parameters.matcher.max_recoveries = 10U;
  parameters.matcher.ray_search = "angular";
  parameters.matcher.enforce_early_gearup = true;
  parameters.matcher.enforce_translation_bound = true;
  parameters.matcher.recover_on_failed_sift = true;
  parameters.matcher.occupancy_threshold = 65;
  parameters.matcher.map_scan_method = "ray_marching";

  return parameters;
}

}

/**
 * @brief An unknown caster name is judged here exactly as the matcher judges
 * it, which today means it is not judged at all.
 * @details Both sides of the comparison below are the empty string, and the case
 * cannot tell the difference between the matcher having judged the name and been
 * content with it and nobody having judged it at all. Nobody has: the matcher's
 * validator carries no rule for `map_scan_method`, so an unknown name passes
 * every validator this library defers to, and it is refused later, when a host
 * parses the name into a casting method and finds no match for it.
 * The assertion stays as an equality all the same, because what is required
 * here is the deferral rather than a refusal. A sentence of this library's own
 * devising, for this setting or any other of the matcher's, would be the second
 * copy of a validator that the embedding exists to prevent. Refusing the name
 * here instead would contradict that deferral, so this case is deliberately not
 * the place where the gap is closed.
 * @remark That the gap exists at all is worth a reader knowing: a setting with
 * a written set of permitted values is not held to it by any validator, only by
 * the parse that follows.
 */
TEST(Validate, AnUnknownMapScanMethodIsJudgedInTheMatchersOwnWords)
{
  Parameters parameters;
  parameters.matcher.map_scan_method = "vanilla";

  EXPECT_EQ(cbgl::validate(parameters),
    fsm_localisation::validate(parameters.matcher))
    << "the name of the caster a map is built with is one of the matcher's own "
    << "settings, so whatever the matcher makes of an unknown one is what this "
    << "library must report. Answering differently means a second validator "
    << "has been written for the embedded half of the parameters, and two "
    << "validators disagree the moment one of them is changed.";
}

TEST(Validate, TheDefaultParametersAreUsable)
{
  EXPECT_EQ(cbgl::validate(Parameters{}), "")
    << "a default constructed parameter set has to be usable, or a host that "
    << "declares a parameter it has no opinion about cannot start. It was "
    << "refused with: " << cbgl::validate(Parameters{});
}

TEST(Validate, TheShippedParametersAreUsable)
{
  const Parameters parameters = shippedParameters();

  EXPECT_EQ(cbgl::validate(parameters), "")
    << "these are the values this package's own configuration file ships, "
    << "which are the values the ROS 1 package was run with and the values "
    << "every comparison between the two builds is measured at. A refusal here "
    << "means the shipped configuration cannot be launched. It was refused "
    << "with: " << cbgl::validate(parameters);
}

TEST_P(AcceptedParameters, ASettingInsideItsRangeIsAccepted)
{
  Parameters parameters;
  GetParam().apply(parameters);

  EXPECT_EQ(cbgl::validate(parameters), "")
    << "the case " << GetParam().label << " puts a setting at a value that is "
    << "permitted, and it was refused with: " << cbgl::validate(parameters)
    << ". A validator that refuses a usable value is as costly as one that "
    << "accepts an unusable one: it stops a run that would have answered.";
}

TEST_P(MatcherRefusal, AnEmbeddedSettingIsRefusedInTheMatchersOwnWords)
{
  const Refusal refusal = GetParam();

  Parameters parameters;
  refusal.apply(parameters);

  const std::string from_matcher = fsm_localisation::validate(parameters.matcher);

  ASSERT_FALSE(from_matcher.empty())
    << "the matcher itself has to refuse " << refusal.setting << " here, or "
    << "this case is measuring nothing. Its validator accepted the value.";

  EXPECT_EQ(cbgl::validate(parameters), from_matcher)
    << "the matcher's own validator refuses this with \"" << from_matcher
    << "\" and this library reported \"" << cbgl::validate(parameters)
    << "\". The matcher's settings are embedded rather than restated so that "
    << "one validator judges them; a sentence of this library's own devising "
    << "means a second copy has been written, and the two will disagree the "
    << "first time either is changed.";
}

TEST_P(RefusedParameters, ASettingOutsideItsRangeIsRefusedNamingTheSettingAndTheValue)
{
  const Refusal refusal = GetParam();

  Parameters parameters;
  refusal.apply(parameters);

  const std::string sentence = cbgl::validate(parameters);

  ASSERT_FALSE(sentence.empty())
    << refusal.setting << " was given " << refusal.value << ", which is "
    << "outside its permitted range, and the parameter set was accepted. The "
    << "run would then start and fail somewhere further in, where the cause is "
    << "no longer legible.";

  EXPECT_NE(sentence.find(refusal.setting), std::string::npos)
    << "the refusal must name the setting it is about. It said \"" << sentence
    << "\", which does not contain \"" << refusal.setting << "\", so an "
    << "operator reading a log cannot tell which of the parameters to correct.";

  EXPECT_NE(sentence.find(refusal.value), std::string::npos)
    << "the refusal must name the value it was given. It said \"" << sentence
    << "\", which does not contain \"" << refusal.value << "\", so an operator "
    << "cannot tell whether the value they meant to set is the value that "
    << "arrived.";
}

INSTANTIATE_TEST_SUITE_P(
  OverTheAlgorithmsOwnSettings, RefusedParameters,
  testing::Values(
    Refusal{.label = "da_of_none",
      .apply = [](Parameters& p) { p.da = 0U; },
      .setting = "da", .value = "0"},
    Refusal{.label = "dl_of_none",
      .apply = [](Parameters& p) { p.dl = 0.0; },
      .setting = "dl", .value = "0"},
    Refusal{.label = "dl_below_none",
      .apply = [](Parameters& p) { p.dl = -1.5; },
      .setting = "dl", .value = "-1.5"},
    Refusal{.label = "dl_not_a_number",
      .apply = [](Parameters& p)
        { p.dl = std::numeric_limits<double>::quiet_NaN(); },
      .setting = "dl", .value = "nan"},
    Refusal{.label = "top_k_caers_of_none",
      .apply = [](Parameters& p) { p.top_k_caers = 0U; },
      .setting = "top_k_caers", .value = "0"}),
  refusalSuffix);

INSTANTIATE_TEST_SUITE_P(
  OverTheMatchersOwnSettings, MatcherRefusal,
  testing::Values(
    Refusal{.label = "max_counter_of_none",
      .apply = [](Parameters& p) { p.matcher.max_counter = 0U; },
      .setting = "max_counter", .value = "0"},
    Refusal{.label = "magnification_range_inverted",
      .apply = [](Parameters& p)
        {
          p.matcher.min_magnification_size = 4U;
          p.matcher.max_magnification_size = 3U;
        },
      .setting = "max_magnification_size", .value = "3"},
    Refusal{.label = "num_iterations_of_none",
      .apply = [](Parameters& p) { p.matcher.num_iterations = 0U; },
      .setting = "num_iterations", .value = "0"},
    Refusal{.label = "occupancy_threshold_above_a_hundred",
      .apply = [](Parameters& p) { p.matcher.occupancy_threshold = 101; },
      .setting = "occupancy_threshold", .value = "101"},
    Refusal{.label = "occupancy_threshold_below_none",
      .apply = [](Parameters& p) { p.matcher.occupancy_threshold = -1; },
      .setting = "occupancy_threshold", .value = "-1"},
    Refusal{.label = "ray_search_unknown",
      .apply = [](Parameters& p) { p.matcher.ray_search = "spiral"; },
      .setting = "ray_search", .value = "spiral"},
    Refusal{.label = "t_bound_below_none",
      .apply = [](Parameters& p) { p.matcher.t_bound = -0.1; },
      .setting = "t_bound", .value = "-0.1"},
    Refusal{.label = "xy_bound_below_none",
      .apply = [](Parameters& p) { p.matcher.xy_bound = -0.2; },
      .setting = "xy_bound", .value = "-0.2"},
    Refusal{.label = "xy_bound_not_finite",
      .apply = [](Parameters& p)
        { p.matcher.xy_bound = std::numeric_limits<double>::infinity(); },
      .setting = "xy_bound", .value = "inf"}),
  refusalSuffix);

INSTANTIATE_TEST_SUITE_P(
  AtTheEdgesOfEveryRange, AcceptedParameters,
  testing::Values(
    Acceptance{.label = "da_of_one",
      .apply = [](Parameters& p) { p.da = 1U; }},
    Acceptance{.label = "dl_barely_above_none",
      .apply = [](Parameters& p) { p.dl = 1e-9; }},
    Acceptance{.label = "dl_of_a_million",
      .apply = [](Parameters& p) { p.dl = 1e6; }},
    Acceptance{.label = "every_ray_the_scan_carries",
      .apply = [](Parameters& p) { p.matcher.size_scan = 0; }},
    Acceptance{.label = "num_threads_of_the_hardwares_choosing",
      .apply = [](Parameters& p) { p.num_threads = 0U; }},
    Acceptance{.label = "num_threads_of_many",
      .apply = [](Parameters& p) { p.num_threads = 1024U; }},
    Acceptance{.label = "occupancy_threshold_at_none",
      .apply = [](Parameters& p) { p.matcher.occupancy_threshold = 0; }},
    Acceptance{.label = "occupancy_threshold_at_a_hundred",
      .apply = [](Parameters& p) { p.matcher.occupancy_threshold = 100; }},
    Acceptance{.label = "ray_search_windowed",
      .apply = [](Parameters& p) { p.matcher.ray_search = "windowed"; }},
    Acceptance{.label = "rng_seed_from_hardware_entropy",
      .apply = [](Parameters& p) { p.rng_seed = 0U; }},
    Acceptance{.label = "rng_seed_pinned",
      .apply = [](Parameters& p) { p.rng_seed = 20260818U; }},
    Acceptance{.label = "top_k_caers_of_one",
      .apply = [](Parameters& p) { p.top_k_caers = 1U; }}),
  acceptanceSuffix);
