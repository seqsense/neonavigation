/*
 * Copyright (c) 2019, the neonavigation authors
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

// ROS 2 port of test/src/test_navigate_boundary.cpp (ROS 1
// navigation_boundary_rostest.test), driven by
// test_navigate_boundary_launch.py.
//
// The launch description starts planner_3d_debug, the variant of the interface
// node built with -DDEBUG so that the assert()s guarding the grid map accesses
// stay enabled; the point of the test is that no assertion fires while the
// robot pose is swept far outside the map.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_cspace/ros2_test_helpers.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

class NavigateBoundary : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr move_base_;
  planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr status_;
  nav_msgs::msg::Path::ConstSharedPtr path_;

  void publishTransform(const double x, const double y)
  {
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = node_->now();
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = x;
    trans.transform.translation.y = y;
    trans.transform.rotation.w = 1.0;
    tfb_->sendTransform(trans);
  }

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_navigate_boundary");
    tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    move_base_ = rclcpp_action::create_client<NavigateToPose>(node_, "move_base");
    sub_status_ = node_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_3d/status", latchedQos(100),
      [this](const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr msg) { status_ = msg; });
    sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/path", latchedQos(),
      [this](const nav_msgs::msg::Path::ConstSharedPtr msg) { path_ = msg; });

    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30), [this] { return move_base_->action_server_is_ready(); }))
      << "Failed to connect move_base action";

    // Keep the robot pose alive while the planner is waiting for its map.
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30), [this] { return static_cast<bool>(status_); },
      [this] { publishTransform(1.0, 0.6); }, std::chrono::milliseconds(100)))
      << "planner_3d didn't start";

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.orientation.w = 1;
    goal.pose.pose.position.x = 1.4;
    goal.pose.pose.position.y = 0.6;
    auto future = move_base_->async_send_goal(goal);
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(10),
      [&future] { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
      [this] { publishTransform(1.0, 0.6); }, std::chrono::milliseconds(100)))
      << "move_base didn't accept the goal";
    ASSERT_TRUE(static_cast<bool>(future.get())) << "move_base rejected the goal";
  }

  void TearDown() override
  {
    // actionlib aborted the goal on its own once the client process was gone;
    // rclcpp_action has no such liveliness detection, so the goal is cancelled
    // explicitly. Leaving it running makes planner_3d terminate a still active
    // goal handle while it is being torn down at SIGINT.
    if (!move_base_) {
      return;
    }
    auto future = move_base_->async_cancel_all_goals();
    spinUntil(node_, std::chrono::seconds(5), [&future] {
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
    spinUntil(node_, std::chrono::seconds(1), [] { return false; });
  }

  // Sweeps the robot pose over and far outside the map and requires that the
  // planner keeps answering with a path and a status for every pose.
  void scanStartPositions(const std::function<void()> & on_spin)
  {
    // map width/height is 32px * 0.1m = 3.2m
    for (double x = -10; x < 13; x += 2.0) {
      for (double y = -10; y < 13; y += 2.0) {
        publishTransform(x, y);

        path_ = nullptr;
        status_ = nullptr;
        const bool received = spinUntil(
          node_, std::chrono::seconds(10), [this] { return path_ && status_; },
          [this, x, y, &on_spin] {
            publishTransform(x, y);
            if (on_spin) {
              on_spin();
            }
          },
          std::chrono::milliseconds(50));
        // Planner must publish at least empty path if alive.
        ASSERT_TRUE(static_cast<bool>(path_)) << "x: " << x << ", y: " << y;
        // Planner status must be published even if robot is outside of the map.
        ASSERT_TRUE(static_cast<bool>(status_)) << "x: " << x << ", y: " << y;
        ASSERT_TRUE(received);
      }
    }
  }
};

TEST_F(NavigateBoundary, StartPositionScan) { scanStartPositions(nullptr); }

TEST_F(NavigateBoundary, StartPositionScanWithTemporaryEscape)
{
  auto pub_trigger =
    node_->create_publisher<std_msgs::msg::Empty>("/planner_3d/temporary_escape", rclcpp::QoS(1));
  scanStartPositions([&pub_trigger] { pub_trigger->publish(std_msgs::msg::Empty()); });
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
