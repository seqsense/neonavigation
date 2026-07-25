/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
 */

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace trajectory_tracker
{
namespace
{
float dist2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
}  // namespace

class RecorderNode : public rclcpp::Node
{
public:
  explicit RecorderNode(const rclcpp::NodeOptions & options);

private:
  void clearPath(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  void cbTimer();

  std::string frame_robot_;
  std::string frame_global_;
  double dist_interval_;
  double ang_interval_;
  bool store_time_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srs_clear_path_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  nav_msgs::msg::Path path_;
};

RecorderNode::RecorderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_recorder", options)
{
  frame_robot_ = this->declare_parameter("frame_robot", std::string("base_link"));
  frame_global_ = this->declare_parameter("frame_global", std::string("map"));
  dist_interval_ = this->declare_parameter("dist_interval", 0.3);
  ang_interval_ = this->declare_parameter("ang_interval", 1.0);
  store_time_ = this->declare_parameter("store_time", false);

  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

  pub_path_ =
    this->create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(10).transient_local());
  srs_clear_path_ = this->create_service<std_srvs::srv::Empty>(
    "~/clear_path",
    std::bind(&RecorderNode::clearPath, this, std::placeholders::_1, std::placeholders::_2));

  path_.header.frame_id = frame_global_;
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / 50.0),
    std::bind(&RecorderNode::cbTimer, this));
}

void RecorderNode::clearPath(
  const std::shared_ptr<std_srvs::srv::Empty::Request> /* req */,
  std::shared_ptr<std_srvs::srv::Empty::Response> /* res */)
{
  path_.poses.clear();
}

void RecorderNode::cbTimer()
{
  tf2::TimePoint stamp = tf2::TimePointZero;
  rclcpp::Time now(0, 0, RCL_ROS_TIME);
  if (store_time_) {
    now = this->now();
    stamp = tf2_ros::fromMsg(builtin_interfaces::msg::Time(now));
  }
  tf2::Stamped<tf2::Transform> transform;
  try {
    tf2::fromMsg(
      tfbuf_->lookupTransform(frame_global_, frame_robot_, stamp, tf2::durationFromSec(0.2)),
      transform);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "TF exception: %s", e.what());
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  tf2::Quaternion q;
  transform.getBasis().getRotation(q);
  pose.pose.orientation = tf2::toMsg(q);
  const tf2::Vector3 origin = transform.getOrigin();
  pose.pose.position.x = origin.x();
  pose.pose.position.y = origin.y();
  pose.pose.position.z = origin.z();
  pose.header.frame_id = frame_global_;
  pose.header.stamp = now;

  path_.header.stamp = now;

  if (path_.poses.size() == 0) {
    path_.poses.push_back(pose);
    pub_path_->publish(path_);
  } else if (dist2d(path_.poses.back().pose.position, pose.pose.position) > dist_interval_) {
    path_.poses.push_back(pose);
    pub_path_->publish(path_);
  }
}
}  // namespace trajectory_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<trajectory_tracker::RecorderNode>(options));
  rclcpp::shutdown();
  return 0;
}
