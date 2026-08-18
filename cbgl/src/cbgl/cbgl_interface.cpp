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

#include <stdexcept>
#include <string>

namespace cbgl
{
namespace
{
/**
 * @brief Refuse a call into an entry point that carries no implementation yet.
 * @details A node that came up, offered its service and refused every call for
 * a reason of its own would be indistinguishable from a working one that never
 * saw a map. Failing at construction is what keeps the difference visible.
 */
[[noreturn]] void refuseUnimplemented(const std::string& entry_point)
{
  throw std::logic_error(entry_point + " carries no implementation yet");
}
}

Interface::Interface(const rclcpp::NodeOptions& options)
: rclcpp::Node("cbgl", options)
{
  refuseUnimplemented("cbgl::Interface::Interface");
}

void Interface::declareParameters()
{
  refuseUnimplemented("cbgl::Interface::declareParameters");
}

void Interface::globalLocalisation(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response>)
{
  refuseUnimplemented("cbgl::Interface::globalLocalisation");
}

void Interface::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr)
{
  refuseUnimplemented("cbgl::Interface::mapCallback");
}

void Interface::publishHypotheses(const std::vector<Pose>&,
  const rclcpp::Time&)
{
  refuseUnimplemented("cbgl::Interface::publishHypotheses");
}

void Interface::publishMapScan(const sensor_msgs::msg::LaserScan&, const Pose&,
  const rclcpp::Time&)
{
  refuseUnimplemented("cbgl::Interface::publishMapScan");
}

void Interface::publishPose(const Pose&, const rclcpp::Time&)
{
  refuseUnimplemented("cbgl::Interface::publishPose");
}

void Interface::publishTopHypotheses(const std::vector<ScoredPose>&,
  const rclcpp::Time&)
{
  refuseUnimplemented("cbgl::Interface::publishTopHypotheses");
}

void Interface::publishTransform(const Pose&, const rclcpp::Time&)
{
  refuseUnimplemented("cbgl::Interface::publishTransform");
}

Parameters Interface::readParameters()
{
  refuseUnimplemented("cbgl::Interface::readParameters");
}

geometry_msgs::msg::Pose Interface::retypePose(const Pose&) const
{
  refuseUnimplemented("cbgl::Interface::retypePose");
}

void Interface::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr)
{
  refuseUnimplemented("cbgl::Interface::scanCallback");
}

std::expected<Pose, std::string> Interface::sensorOnRobot(const std::string&,
  const rclcpp::Time&) const
{
  refuseUnimplemented("cbgl::Interface::sensorOnRobot");
}

}
