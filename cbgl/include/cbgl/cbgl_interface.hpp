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

#ifndef CBGL__CBGL_INTERFACE_HPP_
#define CBGL__CBGL_INTERFACE_HPP_

#include <expected>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <fsm_localisation/occupancy_grid_map.hpp>

#include "cbgl/cbgl.hpp"

namespace cbgl
{
/**
 * @brief Everything about this node that is a ROS concern.
 * @details
 * Owns the parameters, the topics, the service and the transform broadcast, and
 * delegates every computation to `Localiser`. Its name defaults to `cbgl`.
 * Launching it puts it in stand-by: it holds the newest scan and the newest map
 * and computes nothing at all until its service is called. One call performs
 * one whole localisation and returns when that localisation is done, which can
 * be seconds later, because a caller asking for a global answer is asking for
 * exactly that. A second call arriving while one is under way is refused rather
 * than queued.
 * The map reaches it one of two ways, chosen once at start up: on a topic, or
 * from a map description file on disk. A file is read before anything else this
 * node owns exists, so a node configured that way either comes up holding its
 * map or does not come up at all, and it subscribes to no map topic whatsoever.
 */
class Interface : public rclcpp::Node
{
public:
  explicit Interface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  
  void declareParameters();

  /**
   * @brief The service: one localisation, unless one is already under way.
   * @details The answer carries a success flag and a sentence, and the sentence
   * says why wherever the answer is no. No pose is published for a refused run.
   */
  void globalLocalisation(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief One localisation, from the map and the scan held at the moment it
   * begins, and every publication that localisation produces.
   * @details Held apart from the service so that the flag which refuses a
   * second run is raised and lowered in one place, whichever way a run ends.
   */
  void localiseOnce(std_srvs::srv::Trigger::Response& outcome);

  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);

  /**
   * @brief Where one frame sits in another, read from the transform tree.
   * @return the placement, or a sentence naming both frames and saying the tree
   * does not carry one against the other.
   */
  std::expected<Pose, std::string> placementOf(const std::string& frame_id,
    const std::string& in_frame_id, const rclcpp::Time& stamp) const;

  void publishHypotheses(const std::vector<Pose>& hypotheses,
    const rclcpp::Time& stamp);
  void publishMapScan(const fsm_localisation::OccupancyGridMap& map,
    const sensor_msgs::msg::LaserScan& scan, const Pose& sensor,
    const rclcpp::Time& stamp);
  void publishPose(const Pose& robot, const rclcpp::Time& stamp);
  void publishTopHypotheses(const std::vector<ScoredPose>& survivors,
    const rclcpp::Time& stamp);
  /**
   * @brief Broadcast the global frame against the odometry frame.
   * @param odom_in_robot the odometry frame's pose in the robot frame, read
   * from the tree when the run began rather than when it ended.
   */
  void publishTransform(const Pose& robot, const Pose& odom_in_robot,
    const rclcpp::Time& stamp);

  
  Parameters readParameters();

  geometry_msgs::msg::Pose retypePose(const Pose& pose) const;

  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  /**
   * @brief Where the sensor sits on the robot, read from the transform tree.
   * @details Looked up as `base_frame_id` against the frame the scan message
   * itself names, at the stamp that message carries. Nothing has to be told
   * twice what the scan already says.
   * @return the placement, or a sentence saying the tree does not carry it.
   */
  std::expected<Pose, std::string> sensorOnRobot(
    const std::string& sensor_frame_id, const rclcpp::Time& stamp) const;

  /**
   * @brief Take a map as the one this node localises in, or say why it cannot
   * be taken.
   * @details The one place a map becomes a localiser, whether it arrived on the
   * topic or was read from a file at start up. The two sources cannot diverge
   * because there is nothing for them to diverge in.
   * @return an empty string where the map was taken, otherwise a sentence
   * naming what about it was refused.
   */
  std::string takeMap(const nav_msgs::msg::OccupancyGrid& map);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    hypotheses_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr map_scan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    top_hypotheses_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    global_localisation_service_;
  rclcpp::CallbackGroup::SharedPtr service_group_;

  std::unique_ptr<tf2_ros::Buffer> transform_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  Parameters parameters_;

  fsm_localisation::CastingMethod casting_method_{
    fsm_localisation::CastingMethod::ray_marching};

  std::string fixed_frame_id_;
  std::string map_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string scan_topic_;

  double initial_cov_aa_{0.06854};
  double initial_cov_xx_{0.05};
  double initial_cov_yy_{0.05};

  bool tf_broadcast_{false};
  rclcpp::Duration tf_lookup_timeout_{0, 0};

  /**
   * @brief Guards the four members below: the localiser and the map one run
   * works from, the newest scan, and whether a run is under way.
   * @remark The last of the four is the least obvious. It is what refuses a
   * second request rather than queueing it, so a run that ended without
   * lowering it would refuse every request made afterwards.
   */
  mutable std::mutex held_mutex_;
  std::shared_ptr<const Localiser> localiser_;
  std::shared_ptr<const fsm_localisation::OccupancyGridMap> map_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  bool running_{false};
};

}

#endif
