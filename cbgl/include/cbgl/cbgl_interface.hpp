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
 */
class Interface : public rclcpp::Node
{
public:
  explicit Interface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void declareParameters();

  /**
   * @brief One localisation, from the map and the scan held at the moment of
   * the call.
   * @details The service answers with a success flag and a sentence, and the
   * sentence says why wherever the answer is no. No pose is published for a
   * refused run.
   */
  void globalLocalisation(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);

  void publishHypotheses(const std::vector<Pose>& hypotheses,
    const rclcpp::Time& stamp);
  void publishMapScan(const sensor_msgs::msg::LaserScan& scan,
    const Pose& sensor, const rclcpp::Time& stamp);
  void publishPose(const Pose& robot, const rclcpp::Time& stamp);
  void publishTopHypotheses(const std::vector<ScoredPose>& survivors,
    const rclcpp::Time& stamp);
  void publishTransform(const Pose& robot, const rclcpp::Time& stamp);

  Parameters readParameters();

  geometry_msgs::msg::Pose retypePose(const Pose& pose) const;

  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  /**
   * @brief Where the sensor sits on the robot, read from the transform tree.
   * @details Looked up as `robot_frame_id` against the frame the scan message
   * itself names, at the stamp that message carries. Nothing has to be told
   * twice what the scan already says.
   * @return the placement, or a sentence saying the tree does not carry it.
   */
  std::expected<Pose, std::string> sensorOnRobot(
    const std::string& sensor_frame_id, const rclcpp::Time& stamp) const;

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

  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string robot_frame_id_;

  double initial_cov_aa_{0.06854};
  double initial_cov_xx_{0.05};
  double initial_cov_yy_{0.05};

  bool tf_broadcast_{false};
  rclcpp::Duration tf_lookup_timeout_{0, 0};

  mutable std::mutex held_mutex_;
  std::shared_ptr<const Localiser> localiser_;
  std::shared_ptr<const fsm_localisation::OccupancyGridMap> map_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  bool running_{false};
};

}

#endif
