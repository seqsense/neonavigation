/*
 * Copyright (c) 2023-2025, the neonavigation authors
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

// ROS 2 port of test/src/test_navigate_remember.cpp (ROS 1
// navigation_remember_rostest.test), driven by
// test_navigate_remember_launch.py.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_cspace/ros2_test_helpers.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

tf2::TimePoint toTf2Time(const rclcpp::Time & t)
{
  return tf2::TimePoint(std::chrono::nanoseconds(t.nanoseconds()));
}

std::string statusText(const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr & msg)
{
  if (!msg) {
    return "nullptr";
  }
  return "(status: " + std::to_string(msg->status) + ", error: " + std::to_string(msg->error) + ")";
}

class NavigateWithRememberUpdates : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr planner_status_;
  costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr costmap_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr sub_costmap_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_forget_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_patrol_nodes_;
  std::vector<tf2::Stamped<tf2::Transform>> traj_;
  std::string test_scope_;
  bool enable_crowd_mode_;

  void SetUp() override
  {
    traj_.clear();
    planner_status_ = nullptr;
    costmap_ = nullptr;
    path_ = nullptr;
    test_scope_ = "[" + std::to_string(getpid()) + "] ";

    node_ = rclcpp::Node::make_shared("test_navigate_remember");
    enable_crowd_mode_ = node_->declare_parameter("enable_crowd_mode", false);
    tfbuf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

    sub_costmap_ = node_->create_subscription<costmap_cspace_msgs::msg::CSpace3D>(
      "/costmap", latchedQos(),
      [this](const costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr msg) { costmap_ = msg; });
    sub_status_ = node_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_3d/status", rclcpp::QoS(10).transient_local(),
      [this](const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr msg) {
        if (
          !planner_status_ || planner_status_->status != msg->status ||
          planner_status_->error != msg->error) {
          std::cerr << test_scope_ << " Status updated. " << statusText(msg) << std::endl;
        }
        planner_status_ = msg;
      });
    sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/path", latchedQos(),
      [this](const nav_msgs::msg::Path::ConstSharedPtr msg) { path_ = msg; });
    srv_forget_ = node_->create_client<std_srvs::srv::Empty>("/forget_planning_cost");
    pub_initial_pose_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", latchedQos());
    pub_patrol_nodes_ = node_->create_publisher<nav_msgs::msg::Path>("/patrol_nodes", latchedQos());

    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(60),
      [this] {
        return sub_costmap_->get_publisher_count() > 0 && sub_status_->get_publisher_count() > 0 &&
               pub_initial_pose_->get_subscription_count() > 0 &&
               pub_patrol_nodes_->get_subscription_count() > 0 && srv_forget_->service_is_ready();
      }))
      << test_scope_ << "Initialization timeout";

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = 2.1;
    pose.pose.pose.position.y = 3.0;
    pose.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
    pub_initial_pose_->publish(pose);

    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30),
      [this] {
        return tfbuf_->canTransform(
          "map", "base_link", toTf2Time(node_->now()), tf2::durationFromSec(0.5));
      }))
      << test_scope_ << "SetUp: transform timeout";

    ASSERT_TRUE(
      spinUntil(node_, std::chrono::seconds(30), [this] { return static_cast<bool>(costmap_); }))
      << test_scope_ << "Initial costmap timeout";

    auto future =
      srv_forget_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    spinUntil(node_, std::chrono::seconds(10), [&future] {
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });

    sleepAndSpin(1.0);
  }

  void TearDown() override
  {
    if (!node_) {
      return;
    }
    // Clear goal so that planner_3d does not have to terminate a still running
    // action goal while it is being shut down.
    pub_patrol_nodes_->publish(nav_msgs::msg::Path());
    sleepAndSpin(2.0);
  }

  void sleepAndSpin(const double sec)
  {
    spinUntil(
      node_, std::chrono::nanoseconds(static_cast<int64_t>(sec * 1e9)), [] { return false; },
      nullptr, std::chrono::milliseconds(10));
  }

  tf2::Stamped<tf2::Transform> lookupRobotTrans(const rclcpp::Time & now)
  {
    const geometry_msgs::msg::TransformStamped trans_tmp =
      tfbuf_->lookupTransform("map", "base_link", toTf2Time(now), tf2::durationFromSec(0.5));
    tf2::Stamped<tf2::Transform> trans;
    tf2::fromMsg(trans_tmp, trans);
    traj_.push_back(trans);
    return trans;
  }

  void dumpRobotTrajectory()
  {
    double x_prev(0), y_prev(0);
    tf2::Quaternion rot_prev(0, 0, 0, 1);

    std::cerr << test_scope_ << traj_.size() << " points recorded" << std::endl;
    for (const auto & t : traj_) {
      const double x = t.getOrigin().getX();
      const double y = t.getOrigin().getY();
      const tf2::Quaternion rot = t.getRotation();
      const double yaw_diff = rot.angleShortestPath(rot_prev);
      if (std::abs(x - x_prev) > 0.1 || std::abs(y - y_prev) > 0.1 || std::abs(yaw_diff) > 0.2) {
        x_prev = x;
        y_prev = y;
        rot_prev = rot;
        std::cerr << x << " " << y << " " << tf2::getYaw(rot) << std::endl;
      }
    }
  }

  nav_msgs::msg::Path goalPath() const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.poses.resize(1);
    path.poses[0].header.frame_id = path.header.frame_id;
    path.poses[0].pose.position.x = 1.5;
    path.poses[0].pose.position.y = 5.6;
    path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
    return path;
  }

  // Drives to the goal of `path`, optionally triggering a temporary escape on
  // every iteration (the CrowdEscape variant of the ROS 1 test).
  void navigateToGoal(const nav_msgs::msg::Path & path, const bool force_temporary_escape)
  {
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_trigger;
    if (force_temporary_escape) {
      pub_trigger = node_->create_publisher<std_msgs::msg::Empty>(
        "/planner_3d/temporary_escape", rclcpp::QoS(1));
    }

    pub_patrol_nodes_->publish(path);

    tf2::Transform goal;
    tf2::fromMsg(path.poses.back().pose, goal);

    rclcpp::WallRate wait(force_temporary_escape ? 2.0 : 10.0);
    const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(180);
    while (rclcpp::ok()) {
      if (pub_trigger) {
        pub_trigger->publish(std_msgs::msg::Empty());
      }

      rclcpp::spin_some(node_);
      wait.sleep();

      const rclcpp::Time now = node_->now();
      if (now > deadline) {
        dumpRobotTrajectory();
        FAIL() << test_scope_ << "Navigation timeout. status: " << statusText(planner_status_);
        return;
      }

      tf2::Stamped<tf2::Transform> trans;
      try {
        trans = lookupRobotTrans(now);
      } catch (const tf2::TransformException & e) {
        std::cerr << test_scope_ << e.what() << std::endl;
        continue;
      }

      const auto goal_rel = trans.inverse() * goal;
      if (
        goal_rel.getOrigin().length() < 0.2 &&
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2 && planner_status_ &&
        planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DONE) {
        std::cerr << test_scope_ << "Navigation success." << std::endl;
        return;
      }
    }
    FAIL() << test_scope_ << "rclcpp is not ok";
  }
};

TEST_F(NavigateWithRememberUpdates, Navigate)
{
  rclcpp::spin_some(node_);
  sleepAndSpin(0.2);
  navigateToGoal(goalPath(), false);
  ASSERT_FALSE(::testing::Test::HasFailure());
  sleepAndSpin(2.0);
}

TEST_F(NavigateWithRememberUpdates, CrowdEscape)
{
  if (!enable_crowd_mode_) {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(node_);
  sleepAndSpin(0.2);
  navigateToGoal(goalPath(), true);
}
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
