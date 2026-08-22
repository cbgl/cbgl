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
 * @brief `cbgl::Interface`.
 */
#include "cbgl/cbgl_interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <optional>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "cbgl/free_space.hpp"
#include "cbgl/map_file.hpp"

namespace cbgl
{
namespace
{
/**
 * @brief What is wrong with a scan's own angles, for a matcher that reads ray
 * zero at minus pi and reads the rays as spanning exactly one turn.
 * @details This package localises panoramic sensors and nothing else, and the
 * angles a scan carries are the only place it says whether it is one. A scan
 * whose first ray sits somewhere else is a legal scan and a wrong answer here,
 * rotated by whatever the two conventions differ by, so it is refused rather
 * than read as though it agreed. A ray of slack is allowed on both, which is
 * more than the rounding of a float and less than a ray.
 * @return an empty string when the scan is panoramic and begins where it is
 * read as beginning, otherwise a sentence naming the field and the value it
 * carried.
 */
std::string anglesProblemOf(const sensor_msgs::msg::LaserScan& scan)
{
  const double rays = static_cast<double>(scan.ranges.size());
  const double increment = scan.angle_increment;
  const double slack = std::abs(increment) > 0.0
    ? std::abs(increment) : 2.0 * M_PI / std::max(1.0, rays);

  if (std::abs(FSM::Utils::wrapAngle(scan.angle_min + M_PI)) > slack)
  {
    return std::format("the scan carries angle_min {}, where every ray of it "
      "is read from -pi onwards. This package localises panoramic scans only",
      scan.angle_min);
  }

  if (std::abs(rays * increment - 2.0 * M_PI) > slack)
  {
    return std::format("the scan carries {} ranges at angle_increment {}, "
      "which spans {} radians rather than a full turn. This package localises "
      "panoramic scans only", scan.ranges.size(), increment, rays * increment);
  }

  return {};
}

/**
 * @brief `b`, which is expressed in the frame `a` places, expressed in the
 * frame `a` is expressed in.
 */
Pose compose(const Pose& a, const Pose& b)
{
  const double cos_t = std::cos(a.t);
  const double sin_t = std::sin(a.t);

  return Pose{
    a.x + cos_t * b.x - sin_t * b.y,
    a.y + sin_t * b.x + cos_t * b.y,
    FSM::Utils::wrapAngle(a.t + b.t)};
}

/**
 * @brief The whole count a parameter carries.
 * @details Every count this node reads is held unsigned, and a negative integer
 * cast to unsigned is an enormous positive one that no validator downstream can
 * tell from a deliberate setting. So a negative value is refused here, in the
 * same words as every other refusal, rather than turned into a run nobody asked
 * for.
 */
unsigned int countOf(const rclcpp::Node& node, const std::string& name)
{
  const std::int64_t value = node.get_parameter(name).as_int();

  if (value < 0)
  {
    const std::string problem =
      std::format("{} must not be negative, got {}", name, value);
    RCLCPP_FATAL(node.get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  return static_cast<unsigned int>(value);
}

/**
 * @brief The pose that undoes `a`: where the frame `a` is expressed in sits,
 * expressed in the frame `a` places.
 */
Pose invert(const Pose& a)
{
  const double cos_t = std::cos(a.t);
  const double sin_t = std::sin(a.t);

  return Pose{
    -(cos_t * a.x + sin_t * a.y),
    -(-sin_t * a.x + cos_t * a.y),
    FSM::Utils::wrapAngle(-a.t)};
}

/**
 * @brief A descriptor marking a setting read only.
 * @details Every setting is read once, at construction, and nothing reads one
 * again. A value set while the node runs would therefore be accepted and would
 * change nothing, which is a setting that has quietly stopped meaning what it
 * says. Read only turns such a change into a refusal instead.
 */
rcl_interfaces::msg::ParameterDescriptor readOnly()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  return descriptor;
}

/**
 * @brief Holds the flag that refuses a second run raised for as long as one run
 * lasts.
 * @details Lowering the flag on the way out of the scope is what makes the
 * invariant hold whichever way a run ends, an exception included. The sift
 * rethrows what one of its threads caught on the thread that called it, so a
 * run that ran out of memory on a large map arrives here, and a flag left
 * raised by such a run would refuse every request made afterwards.
 */
class RunFlag
{
public:
  RunFlag(std::mutex& mutex, bool& running)
  : mutex_(mutex), running_(running)
  {
  }

  ~RunFlag()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }

  RunFlag(const RunFlag&) = delete;
  RunFlag& operator=(const RunFlag&) = delete;
  RunFlag(RunFlag&&) = delete;
  RunFlag& operator=(RunFlag&&) = delete;

private:
  std::mutex& mutex_;
  bool& running_;
};

/**
 * @brief The quality of service a scan is subscribed at.
 * @return the profile, or `std::nullopt` where `reliability` is neither
 * `"reliable"` nor `"best_effort"`. Nothing else in this package judges that
 * name, so the two spellings are known here and nowhere else.
 */
std::optional<rclcpp::QoS> scanQos(const std::string& reliability,
  const unsigned int depth)
{
  rclcpp::QoS qos(static_cast<std::size_t>(depth));

  if (reliability == "reliable")
    return qos.reliable();

  if (reliability == "best_effort")
    return qos.best_effort();

  return std::nullopt;
}

/**
 * @brief What the matcher's refusal of one survivor amounts to, as a phrase a
 * count can be put in front of.
 */
std::string sentenceOf(const fsm_localisation::CorrectionError error)
{
  switch (error)
  {
    case fsm_localisation::CorrectionError::scan_too_short:
      return "a scan carrying fewer ranges than size_scan asks to match at";
    case fsm_localisation::CorrectionError::scan_entirely_invalid:
      return "a scan carrying no usable range";
    case fsm_localisation::CorrectionError::estimate_not_in_free_space:
      return "a survivor standing outside the map or on a blocked cell";
    case fsm_localisation::CorrectionError::first_cast_entirely_at_max_range:
      return "a survivor from which every cast ray leaves the map";
    case fsm_localisation::CorrectionError::recovery_budget_exhausted:
      return "a match that spent its whole recovery budget without settling";
  }

  return "a refusal the matcher did not name";
}

/**
 * @brief Why the library produced no pose, in words a caller can act on.
 * @details The two reasons that carry per survivor counts say how many
 * survivors went each way, since which way the refinement failed is what tells
 * a badly bounded search from a map the sensor cannot see anything in.
 */
std::string sentenceOf(const Error& error)
{
  std::string counted;

  for (const auto& [reason, count] : error.refusals)
  {
    counted += std::format("{}{} for {}", counted.empty() ? "" : ", ", count,
      sentenceOf(reason));
  }

  const std::string counts = counted.empty() ? "" : ": " + counted;

  switch (error.reason)
  {
    case Error::Reason::every_survivor_refused:
      return "the matcher refused every survivor of the sift" + counts;
    case Error::Reason::map_has_no_free_space:
      return "the map carries no free cell, so there is nowhere to place a "
        "hypothesis";
    case Error::Reason::no_hypotheses_given:
      return "no hypothesis was drawn to search";
    case Error::Reason::no_scorable_pose:
      return "the map casts nothing from any refined pose, so none of them can "
        "be scored" + counts;
    case Error::Reason::scan_entirely_invalid:
      return "no range in the scan is usable";
    case Error::Reason::scan_too_short:
      return "the scan carries fewer ranges than size_scan asks to work at";
  }

  return "the localisation failed for an unrecognised reason";
}

std::optional<double> yawOf(const geometry_msgs::msg::Quaternion& orientation)
{
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    return std::nullopt;
  }

  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;

  constexpr double kMinimumQuaternionNormSquared = 1e-9;

  if (norm_squared < kMinimumQuaternionNormSquared)
    return std::nullopt;

  const tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  return FSM::Utils::wrapAngle(yaw);
}

}

/**
 * @details Every refusal a setting can earn is decided here, before a
 * subscription, a publisher or a service exists, so that the node either comes
 * up usable or does not come up at all.
 * Which of the two map routes is taken is decided here too, and once only. A
 * named `map_topic` is subscribed and `map_file` is then ignored with a word
 * said about it, because a topic can deliver a second map and a file cannot. An
 * empty `map_topic` and a named `map_file` read that file now, and a file that
 * cannot be read or cannot be localised in is a refusal to start rather than a
 * node standing by for a map that will never arrive. Both empty is refused
 * outright.
 * The transform listener and the transform broadcaster are handed plain
 * subscription and publisher options rather than their own defaults, which ask
 * for quality of service overrides and would declare eleven parameters on this
 * node under names no documentation of its settings mentions.
 * The diagnostic sink takes a copy of the logger rather than the node itself,
 * since the sink it is installed in outlives any one node and a process is free
 * to construct more than one.
 */
Interface::Interface(const rclcpp::NodeOptions& options)
: rclcpp::Node("cbgl", options)
{
  declareParameters();

  parameters_ = readParameters();

  if (const std::string problem = validate(parameters_); !problem.empty())
  {
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  const std::optional<fsm_localisation::CastingMethod> method =
    fsm_localisation::castingMethodFromName(parameters_.matcher.map_scan_method);

  if (!method.has_value())
  {
    const std::string problem = std::format(
      "map_scan_method must be \"ray_marching\", \"bresenham\" or \"cddt\", "
      "got \"{}\"", parameters_.matcher.map_scan_method);
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  casting_method_ = *method;

  const std::string reliability =
    get_parameter("scan_qos_reliability").as_string();

  const unsigned int scan_qos_depth = countOf(*this, "scan_qos_depth");

  if (scan_qos_depth == 0U)
  {
    const std::string problem = std::format(
      "scan_qos_depth must be greater than zero, got {}", scan_qos_depth);
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  const std::optional<rclcpp::QoS> scan_qos = scanQos(reliability,
    scan_qos_depth);

  if (!scan_qos.has_value())
  {
    const std::string problem = std::format(
      "scan_qos_reliability must be \"reliable\" or \"best_effort\", got "
      "\"{}\"", reliability);
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  const double timeout = get_parameter("tf_lookup_timeout").as_double();

  if (!std::isfinite(timeout) || timeout < 0.0)
  {
    const std::string problem = std::format(
      "tf_lookup_timeout must be a finite number and must not be negative, "
      "got {}", timeout);
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  tf_lookup_timeout_ = rclcpp::Duration::from_seconds(timeout);

  fixed_frame_id_ = get_parameter("fixed_frame_id").as_string();
  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();

  for (const auto& [name, value] : {
      std::pair<std::string, std::string>{"fixed_frame_id", fixed_frame_id_},
      {"odom_frame_id", odom_frame_id_},
      {"base_frame_id", base_frame_id_}})
  {
    if (value.empty() || value.front() == '/')
    {
      const std::string problem = std::format(
        "{} must name a frame and must not begin with a slash, got \"{}\"",
        name, value);
      RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
      throw std::invalid_argument(problem);
    }
  }

  map_topic_ = get_parameter("map_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();

  const std::string map_file = get_parameter("map_file").as_string();

  const std::string output_pose_topic =
    get_parameter("output_pose_topic").as_string();

  for (const auto& [name, value] : {
      std::pair<std::string, std::string>{"output_pose_topic",
        output_pose_topic},
      {"scan_topic", scan_topic_}})
  {
    if (value.empty())
    {
      const std::string problem = std::format("{} must name a topic", name);
      RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
      throw std::invalid_argument(problem);
    }
  }

  if (map_topic_.empty() && map_file.empty())
  {
    const std::string problem = "one of map_topic and map_file is required, "
      "and both are empty: a map reaches this node either on a topic or from a "
      "map description file on disk, and it cannot localise in no map at all";
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  if (!map_topic_.empty() && !map_file.empty())
  {
    RCLCPP_WARN(get_logger(),
      "map_topic \"%s\" and map_file \"%s\" are both set. The topic wins and "
      "the file is ignored, since a topic can deliver a second map and a file "
      "cannot. Empty map_topic to localise in the file instead.",
      map_topic_.c_str(), map_file.c_str());
  }

  initial_cov_aa_ = get_parameter("initial_cov_aa").as_double();
  initial_cov_xx_ = get_parameter("initial_cov_xx").as_double();
  initial_cov_yy_ = get_parameter("initial_cov_yy").as_double();

  tf_broadcast_ = get_parameter("tf_broadcast").as_bool();

  fsm_localisation::setDiagnosticSink(
    [logger = get_logger()](const std::string& message)
    {
      RCLCPP_INFO(logger, "%s", message.c_str());
    });

  if (map_topic_.empty())
  {
    const std::expected<nav_msgs::msg::OccupancyGrid, std::string> described =
      mapFromFile(map_file);

    if (!described.has_value())
    {
      RCLCPP_FATAL(get_logger(), "Refusing to start: %s",
        described.error().c_str());
      throw std::invalid_argument(described.error());
    }

    nav_msgs::msg::OccupancyGrid map = *described;
    map.header.frame_id = fixed_frame_id_;

    if (const std::string problem = takeMap(map); !problem.empty())
    {
      const std::string refusal = std::format(
        "the map described by {} cannot be localised in: {}", map_file,
        problem);
      RCLCPP_FATAL(get_logger(), "Refusing to start: %s", refusal.c_str());
      throw std::invalid_argument(refusal);
    }
  }

  transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *transform_buffer_, this, true, tf2_ros::DynamicListenerQoS(),
    tf2_ros::StaticListenerQoS(), rclcpp::SubscriptionOptions(),
    rclcpp::SubscriptionOptions());

  if (tf_broadcast_)
  {
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(
      *this, tf2_ros::DynamicBroadcasterQoS(), rclcpp::PublisherOptions());
  }

  pose_publisher_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_pose_topic, 1);

  const std::string hypotheses_topic =
    get_parameter("hypotheses_topic").as_string();

  if (!hypotheses_topic.empty())
  {
    hypotheses_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      hypotheses_topic, 1);
  }

  const std::string top_hypotheses_topic =
    get_parameter("top_hypotheses_topic").as_string();

  if (!top_hypotheses_topic.empty())
  {
    top_hypotheses_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      top_hypotheses_topic, 1);
  }

  const std::string map_scan_topic =
    get_parameter("map_scan_topic").as_string();

  if (!map_scan_topic.empty())
  {
    map_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      map_scan_topic, 1);
  }

  if (!map_topic_.empty())
  {
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
      {
        mapCallback(std::move(map));
      });
  }

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, *scan_qos,
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
    {
      scanCallback(std::move(scan));
    });

  service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  global_localisation_service_ = create_service<std_srvs::srv::Trigger>(
    "~/global_localisation",
    std::bind(&Interface::globalLocalisation, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  RCLCPP_INFO(get_logger(),
    "Standing by. To localise issue ros2 service call "
    "%s/global_localisation std_srvs/srv/Trigger", get_name());
}

void Interface::declareParameters()
{
  declare_parameter("fixed_frame_id", "map", readOnly());
  declare_parameter("base_frame_id", "base_footprint", readOnly());
  declare_parameter("odom_frame_id", "odom", readOnly());
  declare_parameter("tf_broadcast", false, readOnly());
  declare_parameter("tf_lookup_timeout", 0.1, readOnly());

  declare_parameter("scan_topic", "/scan", readOnly());
  declare_parameter("map_topic", "/map", readOnly());
  declare_parameter("map_file", "", readOnly());
  declare_parameter("output_pose_topic", "/initialpose", readOnly());
  declare_parameter("hypotheses_topic", "", readOnly());
  declare_parameter("top_hypotheses_topic", "", readOnly());
  declare_parameter("map_scan_topic", "", readOnly());

  declare_parameter("scan_qos_reliability", "reliable", readOnly());
  declare_parameter("scan_qos_depth", 1, readOnly());

  declare_parameter("dl", 15.0, readOnly());
  declare_parameter("da", 32, readOnly());
  declare_parameter("top_k_caers", 10, readOnly());
  declare_parameter("rng_seed", 0, readOnly());
  declare_parameter("num_threads", 0, readOnly());
  declare_parameter("occupancy_threshold", 65, readOnly());
  declare_parameter("map_scan_method", "ray_marching", readOnly());

  declare_parameter("size_scan", 0, readOnly());
  declare_parameter("min_magnification_size", 0, readOnly());
  declare_parameter("max_magnification_size", 3, readOnly());
  declare_parameter("num_iterations", 2, readOnly());
  declare_parameter("xy_bound", 0.2, readOnly());
  declare_parameter("t_bound", M_PI / 4, readOnly());
  declare_parameter("max_counter", 200, readOnly());
  declare_parameter("max_recoveries", 10, readOnly());
  declare_parameter("ray_search", "angular", readOnly());
  declare_parameter("enforce_early_gearup", false, readOnly());
  declare_parameter("enforce_translation_bound", false, readOnly());
  declare_parameter("recover_on_failed_sift", false, readOnly());

  declare_parameter("initial_cov_xx", 0.05, readOnly());
  declare_parameter("initial_cov_yy", 0.05, readOnly());
  declare_parameter("initial_cov_aa", 0.06854, readOnly());
}

/**
 * @details The flag that refuses a second run is raised here and lowered by the
 * guard on the way out, so that the run itself has one way in and one way out
 * whichever way it ends. The service group is reentrant, which is what lets a
 * second request be answered at once instead of waiting behind the run it is
 * asking about.
 * @remark A run that throws is answered rather than left silent. Nothing else
 * about this node fails without a sentence, and a run that ran out of memory on
 * a large map is exactly the failure whose sentence somebody needs.
 */
void Interface::globalLocalisation(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  {
    const std::lock_guard<std::mutex> lock(held_mutex_);

    if (running_)
    {
      response->success = false;
      response->message = "a localisation is already under way; this request "
        "is refused rather than queued behind it";
      RCLCPP_ERROR(get_logger(), "No localisation: %s",
        response->message.c_str());
      return;
    }

    running_ = true;
  }

  const RunFlag flag(held_mutex_, running_);

  try
  {
    localiseOnce(*response);
  }
  catch (const std::exception& problem)
  {
    response->success = false;
    response->message = std::format("the localisation did not finish: {}",
      problem.what());
    RCLCPP_ERROR(get_logger(), "No localisation: %s",
      response->message.c_str());
  }
}

/**
 * @details The hypothesis set is drawn here and handed to the library, rather
 * than left to the library to draw for itself, because the set that is
 * published has to be the set that was searched.
 * Both placements the answer needs are read from the transform tree before the
 * search begins, at the stamp the scan carries. Reading the odometry frame's
 * placement afterwards would ask a tree that keeps ten seconds of history for a
 * stamp a long run has left behind, and a run on a real building at the shipped
 * hypothesis density takes longer than that.
 * @remark The refusal for want of a map names the topic because that is the
 * only route it can happen on. A node reading its map from a file holds that
 * map from the end of its own construction, or it does not exist.
 */
void Interface::localiseOnce(std_srvs::srv::Trigger::Response& outcome)
{
  const auto refuse = [this, &outcome](const std::string& problem)
    {
      outcome.success = false;
      outcome.message = problem;
      RCLCPP_ERROR(get_logger(), "No localisation: %s", problem.c_str());
    };

  std::shared_ptr<const Localiser> localiser;
  std::shared_ptr<const fsm_localisation::OccupancyGridMap> map;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    localiser = localiser_;
    map = map_;
    scan = latest_scan_;
  }

  if (!localiser)
  {
    refuse("no map has arrived on " + map_topic_);
    return;
  }

  if (!scan)
  {
    refuse("no scan has arrived on " + scan_topic_);
    return;
  }

  if (const std::string problem = anglesProblemOf(*scan); !problem.empty())
  {
    refuse(problem);
    return;
  }

  const rclcpp::Time stamp(scan->header.stamp);

  const std::expected<Pose, std::string> placement =
    sensorOnRobot(scan->header.frame_id, stamp);

  if (!placement.has_value())
  {
    refuse(placement.error());
    return;
  }

  std::optional<Pose> odometry;

  if (tf_broadcast_)
  {
    const std::expected<Pose, std::string> odometry_placement =
      placementOf(odom_frame_id_, base_frame_id_, stamp);

    if (odometry_placement.has_value())
      odometry = *odometry_placement;
    else
      RCLCPP_WARN(get_logger(), "Broadcasting nothing for this answer: %s",
        odometry_placement.error().c_str());
  }

  const std::vector<double> ranges(scan->ranges.begin(), scan->ranges.end());
  const std::vector<Pose> hypotheses = localiser->disperse();

  const std::expected<Result, Error> result =
    localiser->localise(ranges, scan->range_max, hypotheses);

  if (!result.has_value())
  {
    refuse(sentenceOf(result.error()));
    return;
  }

  if (result->survivors.size() < parameters_.top_k_caers)
  {
    RCLCPP_WARN(get_logger(),
      "The map offered %zu candidates where top_k_caers asks for %u, so every "
      "one of them was refined.", result->num_candidates,
      parameters_.top_k_caers);
  }

  const Pose robot = compose(result->pose, invert(*placement));

  publishPose(robot, stamp);

  if (hypotheses_publisher_)
    publishHypotheses(hypotheses, stamp);

  if (top_hypotheses_publisher_)
    publishTopHypotheses(result->survivors, stamp);

  if (map_scan_publisher_)
    publishMapScan(*map, *scan, result->pose, stamp);

  if (odometry.has_value())
    publishTransform(robot, *odometry, stamp);

  unsigned int refused = 0U;

  for (const auto& [reason, count] : result->refusals)
    refused += count;

  outcome.success = true;
  outcome.message = std::format(
    "localised in {:.0f} ms: {} candidates scored, {} survivors handed to the "
    "matcher, {} of them refused", 1000.0 * result->execution_time,
    result->num_candidates, result->survivors.size(), refused);

  RCLCPP_INFO(get_logger(), "%s", outcome.message.c_str());
}

/**
 * @details A map that arrives on the topic and is refused is said out loud and
 * dropped, and whatever map the node already held it keeps. A run under way
 * finishes against the map it started with either way.
 * @remark Taking a map allocates: the blocked bitmap, the free space, and the
 * casting table where that caster is configured. On a large map any of those
 * can fail, and an exception leaving a callback takes the executor and the
 * whole node with it. The service path guards the same hazard on the same map
 * for the same reason, so this guards it too: a map that cannot be taken is a
 * map to drop, not a node to lose.
 */
void Interface::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
{
  try
  {
    if (const std::string problem = takeMap(*map); !problem.empty())
      RCLCPP_ERROR(get_logger(), "Ignoring this map: %s", problem.c_str());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Ignoring this map: %s", ex.what());
  }
}

/**
 * @details The lookup is made at the stamp the scan carries rather than at the
 * clock's idea of now, so a frame that has moved since the scan is placed where
 * it was then. It carries a deadline for the same reason every lookup here
 * does: a tree that has not caught up yet is worth waiting a moment for, and a
 * tree
 * that never carries the frame at all must not hold this thread for good.
 */
std::expected<Pose, std::string> Interface::placementOf(
  const std::string& frame_id, const std::string& in_frame_id,
  const rclcpp::Time& stamp) const
{
  geometry_msgs::msg::TransformStamped placement;

  try
  {
    placement = transform_buffer_->lookupTransform(in_frame_id, frame_id, stamp,
      tf_lookup_timeout_);
  }
  catch (const tf2::TransformException& exception)
  {
    return std::unexpected(std::format(
      "the transform tree does not carry \"{}\" against \"{}\" at the "
      "scan's own stamp within {} s: {}", in_frame_id, frame_id,
      tf_lookup_timeout_.seconds(), exception.what()));
  }

  const std::optional<double> yaw = yawOf(placement.transform.rotation);

  if (!yaw.has_value())
  {
    return std::unexpected(std::format(
      "the placement of \"{}\" in \"{}\" carries no heading: a quaternion "
      "must be finite and must have a length to be normalised by", frame_id,
      in_frame_id));
  }

  return Pose{
    placement.transform.translation.x,
    placement.transform.translation.y,
    *yaw};
}

void Interface::publishHypotheses(const std::vector<Pose>& hypotheses,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseArray message;
  message.header.stamp = stamp;
  message.header.frame_id = fixed_frame_id_;
  message.poses.reserve(hypotheses.size());

  for (const Pose& hypothesis : hypotheses)
    message.poses.push_back(retypePose(hypothesis));

  hypotheses_publisher_->publish(message);
}

/**
 * @details The cast is as long as the scan the run was reduced onto, which is
 * what makes it something to lay over the measurement, and it carries its own
 * angular spacing for the same reason: a run reduced onto fewer rays than the
 * sensor reported is a cast of fewer rays over the same full turn.
 * @remark The map is handed in rather than read from the node, so that a map
 * arriving while a run was under way cannot have this cast taken from it. What
 * is published is a cast from the map the answer was found in.
 */
void Interface::publishMapScan(const fsm_localisation::OccupancyGridMap& map,
  const sensor_msgs::msg::LaserScan& scan, const Pose& sensor,
  const rclcpp::Time& stamp)
{
  const std::size_t rays = parameters_.matcher.size_scan == 0
    ? scan.ranges.size() : parameters_.matcher.size_scan;

  const std::vector<double> ranges =
    map.ranges(sensor, static_cast<unsigned int>(rays), scan.range_max);

  sensor_msgs::msg::LaserScan message;
  message.header.stamp = stamp;
  message.header.frame_id = scan.header.frame_id;
  message.angle_min = scan.angle_min;
  message.angle_increment =
    static_cast<float>(2.0 * M_PI / static_cast<double>(rays));
  message.angle_max = message.angle_min +
    static_cast<float>(rays - 1) * message.angle_increment;
  message.range_min = scan.range_min;
  message.range_max = scan.range_max;

  message.ranges.assign(ranges.begin(), ranges.end());

  map_scan_publisher_->publish(message);
}

/**
 * @details The covariance is not measured and is not an estimate of this
 * package's own uncertainty. It is what a consumer such as amcl is seeded with,
 * and it is published as it was configured.
 */
void Interface::publishPose(const Pose& robot, const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = fixed_frame_id_;
  message.pose.pose = retypePose(robot);

  std::array<double, 36> covariance{};
  covariance[0] = initial_cov_xx_;
  covariance[7] = initial_cov_yy_;
  covariance[35] = initial_cov_aa_;
  message.pose.covariance = covariance;

  pose_publisher_->publish(message);
}

void Interface::publishTopHypotheses(const std::vector<ScoredPose>& survivors,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseArray message;
  message.header.stamp = stamp;
  message.header.frame_id = fixed_frame_id_;
  message.poses.reserve(survivors.size());

  for (const ScoredPose& survivor : survivors)
    message.poses.push_back(retypePose(survivor.pose));

  top_hypotheses_publisher_->publish(message);
}

/**
 * @details The robot's pose in the global frame composed with the odometry
 * frame's pose in the robot frame, which is the transform that places the
 * answer in a tree that already carries an odometry frame.
 * @remark The odometry placement is read when the run begins and handed in
 * here, not read again now. A tree keeps a window of history, and a run of a
 * real building takes long enough to leave the scan's own stamp outside it.
 */
void Interface::publishTransform(const Pose& robot, const Pose& odom_in_robot,
  const rclcpp::Time& stamp)
{
  const Pose odom_in_global = compose(robot, odom_in_robot);

  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = fixed_frame_id_;
  message.child_frame_id = odom_frame_id_;

  message.transform.translation.x = odom_in_global.x;
  message.transform.translation.y = odom_in_global.y;
  message.transform.translation.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, odom_in_global.t);
  quaternion.normalize();
  message.transform.rotation.x = quaternion.x();
  message.transform.rotation.y = quaternion.y();
  message.transform.rotation.z = quaternion.z();
  message.transform.rotation.w = quaternion.w();

  transform_broadcaster_->sendTransform(message);
}

/**
 * @details `rng_seed` is set in both places from the one declared parameter:
 * the library's own field, which governs the dispersal draw, and the matcher's
 * embedded field, which governs the recovery jitter. One parameter, two fields,
 * so that pinning it pins the whole of a run rather than half of it.
 * @remark The occupancy threshold and the caster's name are read into the
 * embedded matcher settings, which is where the matcher's own validator judges
 * them, and are declared under the names the documentation gives them.
 */
Parameters Interface::readParameters()
{
  Parameters parameters;

  parameters.da = countOf(*this, "da");
  parameters.dl = get_parameter("dl").as_double();
  parameters.num_threads = countOf(*this, "num_threads");
  parameters.rng_seed = countOf(*this, "rng_seed");
  parameters.top_k_caers = countOf(*this, "top_k_caers");

  parameters.matcher.size_scan = countOf(*this, "size_scan");
  parameters.matcher.num_iterations = countOf(*this, "num_iterations");
  parameters.matcher.xy_bound = get_parameter("xy_bound").as_double();
  parameters.matcher.t_bound = get_parameter("t_bound").as_double();
  parameters.matcher.max_counter = countOf(*this, "max_counter");
  parameters.matcher.min_magnification_size =
    countOf(*this, "min_magnification_size");
  parameters.matcher.max_magnification_size =
    countOf(*this, "max_magnification_size");
  parameters.matcher.max_recoveries = countOf(*this, "max_recoveries");
  parameters.matcher.rng_seed = parameters.rng_seed;
  parameters.matcher.ray_search = get_parameter("ray_search").as_string();

  parameters.matcher.enforce_early_gearup =
    get_parameter("enforce_early_gearup").as_bool();
  parameters.matcher.enforce_translation_bound =
    get_parameter("enforce_translation_bound").as_bool();
  parameters.matcher.recover_on_failed_sift =
    get_parameter("recover_on_failed_sift").as_bool();

  parameters.matcher.occupancy_threshold =
    static_cast<int>(get_parameter("occupancy_threshold").as_int());
  parameters.matcher.map_scan_method =
    get_parameter("map_scan_method").as_string();

  return parameters;
}

geometry_msgs::msg::Pose Interface::retypePose(const Pose& pose) const
{
  geometry_msgs::msg::Pose message;

  message.position.x = pose.x;
  message.position.y = pose.y;
  message.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, pose.t);
  quaternion.normalize();
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();

  return message;
}

void Interface::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  const std::lock_guard<std::mutex> lock(held_mutex_);
  latest_scan_ = std::move(scan);
}

/**
 * @details The robot's frame against the frame the scan message itself names.
 * Nothing has to be told twice what the scan already says, so there is no
 * parameter for the sensor's frame and there is not to be one.
 */
std::expected<Pose, std::string> Interface::sensorOnRobot(
  const std::string& sensor_frame_id, const rclcpp::Time& stamp) const
{
  const std::expected<Pose, std::string> placement =
    placementOf(sensor_frame_id, base_frame_id_, stamp);

  if (placement.has_value())
    return placement;

  return std::unexpected("the sensor's placement on the robot cannot be read, "
    + sensor_frame_id + " being the frame the scan itself names: "
    + placement.error());
}

/**
 * @details Building the map, walking its free space and constructing the
 * localiser over both happen here, off the lock, and only the swap of the two
 * finished objects is taken under it. A run that is already under way holds its
 * own map and its own localiser and is unaffected by either.
 * @remark What is refused, and by whom, is not restated here. The map's own
 * builder judges the message and names the field it refused, and this hands that
 * sentence back to whichever of the two callers asked, so that a topic can log
 * it and a start up can die of it.
 */
std::string Interface::takeMap(const nav_msgs::msg::OccupancyGrid& map)
{
  const std::expected<std::shared_ptr<
    const fsm_localisation::OccupancyGridMap>, std::string> built =
    fsm_localisation::OccupancyGridMap::build(map,
      parameters_.matcher.occupancy_threshold, casting_method_);

  if (!built.has_value())
    return built.error();

  FreeSpace free_space = freeSpaceOf(**built);
  const std::size_t free_cells = free_space.cells.size();

  /* A map with nowhere to stand is refused here rather than at the first
   * request. The builder judges the message's shape and says nothing about
   * what is in it, so a grid whose every cell is blocked passes it and the
   * node comes up looking healthy, only to refuse every request afterwards
   * for a reason that names neither the map nor where it came from. Refusing
   * it at the point the map arrives means a start up from a file dies of it
   * with the file named, which is what this package's own documentation
   * promises, and a map on a topic is dropped with the same sentence while
   * whatever map the node already held it keeps. */
  if (free_cells == 0U)
  {
    return "the map carries no free cell, so there is nowhere in it to place "
      "a hypothesis and no request against it could ever be answered";
  }

  const auto localiser =
    std::make_shared<const Localiser>(parameters_, *built,
      std::move(free_space));

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    map_ = *built;
    localiser_ = localiser;
  }

  if (map.header.frame_id != fixed_frame_id_)
  {
    RCLCPP_WARN(get_logger(),
      "This map is labelled \"%s\" and every answer from it will be labelled "
      "\"%s\", which is what fixed_frame_id says. An answer's coordinates are "
      "the map's own, so the two names have to mean the same frame.",
      map.header.frame_id.c_str(), fixed_frame_id_.c_str());
  }

  RCLCPP_INFO(get_logger(),
    "Map taken: %u by %u cells at %.3f m, %zu of them free, cast by %s.",
    map.info.width, map.info.height, map.info.resolution, free_cells,
    std::string(fsm_localisation::nameOfCastingMethod(casting_method_)).c_str());

  return {};
}

}
