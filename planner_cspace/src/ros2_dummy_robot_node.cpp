/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace planner_cspace
{
// ROS 2 interface of the dummy_robot test node.
//
// The ROS 1 node integrated the commanded velocity in a 100 Hz `while
// (ros::ok())` loop; on ROS 2 the same body runs from a 100 Hz wall timer so
// the node can also be loaded as a component. The published topics, the
// broadcast odom -> base_link transform and the parameters are unchanged.
class DummyRobotNode : public rclcpp::Node
{
public:
  explicit DummyRobotNode(const rclcpp::NodeOptions & options);

private:
  double x_;
  double y_;
  double yaw_;
  float v_;
  float w_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;

  void cbTwist(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);
  void cbInit(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void cbTimer();
};

DummyRobotNode::DummyRobotNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dummy_robot", options), v_(0.0f), w_(0.0f)
{
  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
  tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  x_ = this->declare_parameter("initial_x", 0.0);
  y_ = this->declare_parameter("initial_y", 0.0);
  yaw_ = this->declare_parameter("initial_yaw", 0.0);

  // ROS 1 latched this publisher; transient_local is the ROS 2 equivalent.
  pub_odom_ =
    this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1).transient_local());
  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&DummyRobotNode::cbTwist, this, std::placeholders::_1));
  sub_init_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&DummyRobotNode::cbInit, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(0.01),
    std::bind(&DummyRobotNode::cbTimer, this));
}

void DummyRobotNode::cbTwist(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  v_ = msg->linear.x;
  w_ = msg->angular.z;
}

void DummyRobotNode::cbInit(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  geometry_msgs::msg::PoseStamped pose_in;
  geometry_msgs::msg::PoseStamped pose_out;
  pose_in.header = msg->header;
  pose_in.pose = msg->pose.pose;
  try {
    const geometry_msgs::msg::TransformStamped trans = tfbuf_->lookupTransform(
      "odom", pose_in.header.frame_id, pose_in.header.stamp, rclcpp::Duration::from_seconds(1.0));
    tf2::doTransform(pose_in, pose_out, trans);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    return;
  }

  x_ = pose_out.pose.position.x;
  y_ = pose_out.pose.position.y;
  yaw_ = tf2::getYaw(pose_out.pose.orientation);
  v_ = 0;
  w_ = 0;
}

void DummyRobotNode::cbTimer()
{
  const float dt = 0.01;
  const rclcpp::Time current_time = this->now();

  yaw_ += w_ * dt;
  x_ += cosf(yaw_) * v_ * dt;
  y_ += sinf(yaw_) * v_ * dt;

  geometry_msgs::msg::TransformStamped trans;
  trans.header.stamp = current_time;
  trans.header.frame_id = "odom";
  trans.child_frame_id = "base_link";
  trans.transform.translation = tf2::toMsg(tf2::Vector3(x_, y_, 0.0));
  trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
  tfb_->sendTransform(trans);

  auto odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->header.frame_id = "odom";
  odom->header.stamp = current_time;
  odom->child_frame_id = "base_link";
  odom->pose.pose.position.x = x_;
  odom->pose.pose.position.y = y_;
  odom->pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
  odom->twist.twist.linear.x = v_;
  odom->twist.twist.angular.z = w_;
  pub_odom_->publish(std::move(odom));
}
}  // namespace planner_cspace

RCLCPP_COMPONENTS_REGISTER_NODE(planner_cspace::DummyRobotNode)
