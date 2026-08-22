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
 * @brief Which settings the node declares, what it declares them with, and
 * whether the readme still says the same thing.
 * @details
 * The node is constructed here rather than launched, so a setting's default is
 * read back through the parameter interface instead of being inferred from what
 * a running node does with it. What that buys is completeness: every setting the
 * node offers is checked, including the ones no behaviour test could reach
 * without a rig of its own.
 * **Three separate properties.** That every documented setting is declared with
 * the documented default. That nothing else is declared, so a setting cannot be
 * added to the code without being added to the documentation in the same
 * breath. And that a name the node never declared does not become a parameter
 * just because somebody passed it in, which is what makes the second property
 * enforceable at all: with overrides declared automatically, a misspelt setting
 * would appear beside the real ones and be read by nobody.
 * **Reading a setting that was never declared.** There is no assertion for it
 * and there does not need to be one. `rclcpp` throws on a read of an undeclared
 * parameter, so a node that reads one cannot be constructed, and every case
 * below constructs it.
 * **The types are part of the contract.** A density of hypotheses per square
 * metre is a real number, an occupancy threshold is a whole percentage and a
 * broadcast switch is a boolean. Declaring a real number as an integer would
 * refuse `dl: 0.5` at launch for no reason a reader of the documentation could
 * anticipate, so the type is asserted alongside the value.
 * @remark No setting is declared with a `ParameterDescriptor` carrying a range,
 * deliberately, and this is the place to say why so that nobody adds one as an
 * improvement. A range in a descriptor makes `rclcpp` reject an out of range
 * override itself, with a sentence of its own that names the setting but not the
 * value it was given. The refusals of this package name both, because a value
 * that was not the value somebody meant to set is the commonest reason a
 * configuration is wrong, and the two validators are where that wording lives.
 * A descriptor range would take the refusal out of their hands and silently
 * weaken what the launch level cases require of it.
 * @remark The last case reads the readme's own parameter table and holds it to
 * the declared defaults, so that the code and the document cannot drift apart.
 * It skips while the readme carries no such table, and the documentation stage
 * that writes one is what turns it on. It is a skip rather than a silence: a
 * skipped case is reported by name every time the suite runs.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "cbgl/cbgl_interface.hpp"

namespace
{
using cbgl::Interface;

/**
 * @brief One setting the node is required to declare, and the value it is
 * required to declare it with.
 */
struct Setting
{
  const char* name{""};
  rclcpp::ParameterValue value;
};

/**
 * @brief What a parameter of the empty string is spelled as in a table meant
 * for people, where an empty cell is indistinguishable from a missing one.
 */
const char* const kEmptyInProse = "empty";

/**
 * @brief What `pi / 4` is spelled as in the same table, once every space is
 * taken out of it.
 */
const char* const kQuarterTurnInProse = "pi/4";

std::set<std::string> declaredNames(const rclcpp::Node& node)
{
  std::set<std::string> names;

  for (const std::string& name : node.list_parameters({}, 0).names)
    names.insert(name);

  return names;
}

/**
 * @brief The parameters `rclcpp` declares for every node, whatever that node
 * does.
 * @details Six of them: the four quality of service overrides for the parameter
 * event publisher every node carries, the switch for the type description
 * service, and the simulated clock switch. Not one is this package's, not one
 * appears in any documentation of this package's settings, and not one can be
 * got rid of.
 * This was established rather than assumed. A bare `rclcpp::Node`, built and
 * run in the same environment this case runs in, declares exactly these six and
 * nothing else. Starting it with the parameter event publisher switched off and
 * the parameter services switched off removes none of them, and
 * `undeclare_parameter` refuses each one: the first five because they are read
 * only, the clock switch because it is statically typed. So a node that declares
 * these six has declared nothing of its own, and allowing them here is not a
 * concession, it is the floor.
 * @remark They are named one at a time on purpose. Allowing anything that
 * begins `qos_overrides.` would be shorter and would hide the very thing this
 * case is here to catch: a transform listener and a transform broadcaster left
 * on their default subscription options ask for overrides on both transform
 * topics as well, which is eleven further names under that same prefix that no
 * documentation mentions. Those eleven are gone, and this case is what keeps
 * them gone.
 */
std::set<std::string> everyNameRclcppDeclares()
{
  return {
    "qos_overrides./parameter_events.publisher.depth",
    "qos_overrides./parameter_events.publisher.durability",
    "qos_overrides./parameter_events.publisher.history",
    "qos_overrides./parameter_events.publisher.reliability",
    "start_type_description_service",
    "use_sim_time"};
}

/**
 * @brief Every setting the node offers, with the default the documentation
 * gives for it.
 * @details The order is the order the documentation presents them in: the
 * frames, topics and transport first, then the algorithm's own settings, then
 * the matcher's, then the covariance the answer is published with.
 */
std::vector<Setting> everySetting()
{
  return {
    Setting{.name = "global_frame_id",
      .value = rclcpp::ParameterValue(std::string("map"))},
    Setting{.name = "robot_frame_id",
      .value = rclcpp::ParameterValue(std::string("base_footprint"))},
    Setting{.name = "odom_frame_id",
      .value = rclcpp::ParameterValue(std::string("odom"))},
    Setting{.name = "tf_broadcast", .value = rclcpp::ParameterValue(false)},
    Setting{.name = "tf_lookup_timeout", .value = rclcpp::ParameterValue(0.1)},
    Setting{.name = "scan_topic",
      .value = rclcpp::ParameterValue(std::string("/scan"))},
    Setting{.name = "map_topic",
      .value = rclcpp::ParameterValue(std::string("/map"))},
    Setting{.name = "map_file",
      .value = rclcpp::ParameterValue(std::string(""))},
    Setting{.name = "output_pose_topic",
      .value = rclcpp::ParameterValue(std::string("/initialpose"))},
    Setting{.name = "hypotheses_topic",
      .value = rclcpp::ParameterValue(std::string(""))},
    Setting{.name = "top_hypotheses_topic",
      .value = rclcpp::ParameterValue(std::string(""))},
    Setting{.name = "map_scan_topic",
      .value = rclcpp::ParameterValue(std::string(""))},
    Setting{.name = "scan_qos_reliability",
      .value = rclcpp::ParameterValue(std::string("reliable"))},
    Setting{.name = "scan_qos_depth", .value = rclcpp::ParameterValue(1)},

    Setting{.name = "dl", .value = rclcpp::ParameterValue(15.0)},
    Setting{.name = "da", .value = rclcpp::ParameterValue(32)},
    Setting{.name = "top_k_caers", .value = rclcpp::ParameterValue(10)},
    Setting{.name = "rng_seed", .value = rclcpp::ParameterValue(0)},
    Setting{.name = "num_threads", .value = rclcpp::ParameterValue(0)},
    Setting{.name = "occupancy_threshold", .value = rclcpp::ParameterValue(65)},
    Setting{.name = "map_scan_method",
      .value = rclcpp::ParameterValue(std::string("ray_marching"))},

    Setting{.name = "size_scan", .value = rclcpp::ParameterValue(0)},
    Setting{.name = "min_magnification_size",
      .value = rclcpp::ParameterValue(0)},
    Setting{.name = "max_magnification_size",
      .value = rclcpp::ParameterValue(3)},
    Setting{.name = "num_iterations", .value = rclcpp::ParameterValue(2)},
    Setting{.name = "xy_bound", .value = rclcpp::ParameterValue(0.2)},
    Setting{.name = "t_bound", .value = rclcpp::ParameterValue(M_PI / 4)},
    Setting{.name = "max_counter", .value = rclcpp::ParameterValue(200)},
    Setting{.name = "max_recoveries", .value = rclcpp::ParameterValue(10)},
    Setting{.name = "ray_search",
      .value = rclcpp::ParameterValue(std::string("angular"))},
    Setting{.name = "enforce_early_gearup",
      .value = rclcpp::ParameterValue(false)},
    Setting{.name = "enforce_translation_bound",
      .value = rclcpp::ParameterValue(false)},
    Setting{.name = "recover_on_failed_sift",
      .value = rclcpp::ParameterValue(false)},

    Setting{.name = "initial_cov_xx", .value = rclcpp::ParameterValue(0.05)},
    Setting{.name = "initial_cov_yy", .value = rclcpp::ParameterValue(0.05)},
    Setting{.name = "initial_cov_aa", .value = rclcpp::ParameterValue(0.06854)}};
}

void expectDeclaredWith(const rclcpp::Node& node, const Setting& setting)
{
  ASSERT_TRUE(node.has_parameter(setting.name))
    << setting.name << " is documented as a setting of this node and the node "
    << "does not declare it. Nothing can set it, and a configuration file that "
    << "names it is ignored without a word.";

  const rclcpp::ParameterValue declared =
    node.get_parameter(setting.name).get_parameter_value();

  ASSERT_EQ(declared.get_type(), setting.value.get_type())
    << setting.name << " is declared as a "
    << rclcpp::to_string(declared.get_type()) << " and is documented as a "
    << rclcpp::to_string(setting.value.get_type()) << ". A value of the "
    << "documented kind is then refused at launch by the type check rather "
    << "than by anything a reader of the documentation could have anticipated.";

  EXPECT_EQ(declared, setting.value)
    << setting.name << " defaults to " << rclcpp::to_string(declared)
    << " and is documented as defaulting to "
    << rclcpp::to_string(setting.value) << ".";
}

/**
 * @brief A cell of a table meant for people, reduced to something comparable:
 * no spaces, no backticks, no capitals.
 */
std::string normalise(const std::string& cell)
{
  std::string reduced;

  for (const char character : cell)
    if (character != ' ' && character != '`' && character != '\t')
      reduced.push_back(static_cast<char>(std::tolower(character)));

  return reduced;
}

/**
 * @brief The default every parameter row of the readme's parameter tables
 * states, by the name that row names.
 * @details Only rows of a table whose header carries both a `Parameter` column
 * and a `Default` column are read, because the readme's topic and service
 * tables name the very same settings in their own first column and say
 * something else entirely in their last.
 * @return the rows found, which is empty wherever the readme carries no such
 * table at all.
 */
std::map<std::string, std::string> readmeDefaults(const std::string& path)
{
  std::ifstream file(path);
  std::map<std::string, std::string> defaults;

  if (!file.good())
    return defaults;

  std::size_t column_of_default = 0;
  bool inside_a_parameter_table = false;
  std::string line;

  while (std::getline(file, line))
  {
    if (line.empty() || line.front() != '|')
    {
      inside_a_parameter_table = false;
      continue;
    }

    std::vector<std::string> cells;
    std::istringstream fields(line);
    std::string cell;

    while (std::getline(fields, cell, '|'))
      cells.push_back(normalise(cell));

    if (!cells.empty() && cells.front().empty())
      cells.erase(cells.begin());

    if (cells.size() < 2
      || cells.front().find_first_not_of('-') == std::string::npos)
      continue;

    if (cells.front() == "parameter")
    {
      const auto found = std::find(cells.begin(), cells.end(), "default");
      inside_a_parameter_table = found != cells.end();

      if (inside_a_parameter_table)
        column_of_default =
          static_cast<std::size_t>(std::distance(cells.begin(), found));

      continue;
    }

    if (inside_a_parameter_table && column_of_default < cells.size())
      defaults[cells.front()] = cells[column_of_default];
  }

  return defaults;
}

std::string readmePath()
{
  return std::string(CBGL_README);
}

}

TEST(NodeParameters, AnUndocumentedOverrideIsNotTurnedIntoAParameter)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("scan_topics", std::string("/scan"))});

  std::shared_ptr<Interface> node;
  ASSERT_NO_THROW(node = std::make_shared<Interface>(options));

  EXPECT_FALSE(node->has_parameter("scan_topics"))
    << "a name this node never declared became a parameter because it was "
    << "passed in, so a misspelt setting is accepted in silence and read by "
    << "nobody. It also makes the count of declared parameters a count of what "
    << "was passed in rather than of what the node offers.";
}

TEST(NodeParameters, EverySettingIsDeclaredWithItsDocumentedDefault)
{
  std::shared_ptr<Interface> node;
  ASSERT_NO_THROW(node = std::make_shared<Interface>(rclcpp::NodeOptions()))
    << "the node has to come up from its documented defaults alone, or none of "
    << "them is a default.";

  for (const Setting& setting : everySetting())
    expectDeclaredWith(*node, setting);
}

/**
 * @brief Nothing is declared that no documentation of this package mentions.
 * @details Every name the node declares has to be either a setting of this
 * package, documented alongside the rest, or one of the six `rclcpp` gives every
 * node whether it is wanted or not. That second list is fixed and exhaustive, so
 * one name of the node's own beyond its documented settings still turns this
 * red, which is the whole point of the case.
 */
TEST(NodeParameters, NoSettingIsDeclaredThatIsNotDocumented)
{
  std::shared_ptr<Interface> node;
  ASSERT_NO_THROW(node = std::make_shared<Interface>(rclcpp::NodeOptions()));

  std::set<std::string> allowed = everyNameRclcppDeclares();

  for (const Setting& setting : everySetting())
    allowed.insert(setting.name);

  for (const std::string& declared : declaredNames(*node))
    EXPECT_TRUE(allowed.contains(declared))
      << declared << " is declared by the node and appears in no parameter "
      << "table, and it is not one of the six every node carries whether it "
      << "wants them or not. A setting nobody documented is a setting nobody "
      << "can be expected to set, and it will be the one the next operator "
      << "needs.";
}

TEST(NodeParameters, TheNodeAnswersToTheNameItsServiceIsAddressedBy)
{
  std::shared_ptr<Interface> node;
  ASSERT_NO_THROW(node = std::make_shared<Interface>(rclcpp::NodeOptions()));

  EXPECT_STREQ(node->get_name(), "cbgl")
    << "the service is node relative, so the node's own default name is half "
    << "of the address a caller uses. It came up as " << node->get_name()
    << ", which puts the service somewhere no documented caller looks.";
}

/**
 * @brief The readme's parameter table, held to the declared defaults.
 * @details Skipped while the readme carries no parameter table, which is the
 * state of it until the documentation is written. The stage that writes it turns
 * this on and nothing else has to change here.
 * @remark A default is compared after every space, backtick and capital has
 * been taken out of the cell, so a table is free to format itself. Two
 * spellings are understood beyond the value itself: `empty` for a setting whose
 * default is the empty string, an empty cell being indistinguishable from a
 * missing one, and `pi / 4` for a quarter turn.
 */
TEST(NodeParameters, TheReadmesParameterTableStatesTheDeclaredDefaults)
{
  const std::map<std::string, std::string> stated =
    readmeDefaults(readmePath());

  if (stated.empty())
    GTEST_SKIP()
      << readmePath() << " carries no table with a Parameter column and a "
      << "Default column, so there is nothing yet to hold the declared "
      << "defaults to. The documentation that writes one enables this case.";

  std::shared_ptr<Interface> node;
  ASSERT_NO_THROW(node = std::make_shared<Interface>(rclcpp::NodeOptions()));

  for (const Setting& setting : everySetting())
  {
    const auto row = stated.find(setting.name);

    ASSERT_NE(row, stated.end())
      << "the readme's parameter table does not mention " << setting.name
      << ", which the node declares. A reader cannot set what the "
      << "documentation does not name.";

    const std::string& cell = row->second;
    const rclcpp::Parameter declared = node->get_parameter(setting.name);

    switch (declared.get_type())
    {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        EXPECT_EQ(cell, declared.as_bool() ? "true" : "false")
          << "the readme says " << setting.name << " defaults to \"" << cell
          << "\" and it defaults to "
          << (declared.as_bool() ? "true" : "false") << ".";
        break;

      case rclcpp::ParameterType::PARAMETER_INTEGER:
        EXPECT_EQ(cell, std::to_string(declared.as_int()))
          << "the readme says " << setting.name << " defaults to \"" << cell
          << "\" and it defaults to " << declared.as_int() << ".";
        break;

      case rclcpp::ParameterType::PARAMETER_DOUBLE:
      {
        const double documented = cell == kQuarterTurnInProse
          ? M_PI / 4 : std::strtod(cell.c_str(), nullptr);

        EXPECT_NEAR(documented, declared.as_double(), 1e-9)
          << "the readme says " << setting.name << " defaults to \"" << cell
          << "\" and it defaults to " << declared.as_double()
          << ". A number is read from the cell as it stands, with \""
          << kQuarterTurnInProse << "\" understood as a quarter turn.";
        break;
      }

      default:
        EXPECT_EQ(cell == kEmptyInProse ? std::string() : cell,
          normalise(declared.as_string()))
          << "the readme says " << setting.name << " defaults to \"" << cell
          << "\" and it defaults to \"" << declared.as_string() << "\". A "
          << "default of the empty string is spelled \"" << kEmptyInProse
          << "\", an empty cell being indistinguishable from a missing one.";
        break;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
