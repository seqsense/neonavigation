/*
 * Copyright (c) 2020, the neonavigation authors
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

// ROS 2 port of test/src/test_costmap_watchdog.cpp (ROS 1
// costmap_watchdog_rostest.test), driven by test_costmap_watchdog_launch.py.
//
// The goal is published on "goal_pose" (the ROS 2 name of the ROS 1
// "move_base_simple/goal" topic, which the rostest remapped to "goal").

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_cspace/ros2_test_helpers.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

class Planner3D : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>::SharedPtr pub_cost_update_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diag_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr sub_costmap_;

  planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr status_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_;
  costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr costmap_;
  int cnt_;

  void SetUp() override
  {
    cnt_ = 0;
    node_ = rclcpp::Node::make_shared("test_costmap_watchdog");
    pub_goal_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", latchedQos());
    pub_cost_update_ = node_->create_publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>(
      "/costmap_update", latchedQos());
    sub_status_ = node_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_3d/status", latchedQos(),
      [this](const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr msg) {
        status_ = msg;
        ++cnt_;
      });
    sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/path", latchedQos(),
      [this](const nav_msgs::msg::Path::ConstSharedPtr msg) { path_ = msg; });
    sub_diag_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) { diag_ = msg; });
    sub_costmap_ = node_->create_subscription<costmap_cspace_msgs::msg::CSpace3D>(
      "/costmap", latchedQos(),
      [this](const costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr msg) { costmap_ = msg; });
  }

  // ROS 1 relied on a fixed 0.5 s sleep to make sure planner_3d had the map
  // before the goal arrived. On ROS 2 the discovery delay is not bounded, so
  // the same precondition is waited for explicitly.
  void waitPlannerReady()
  {
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(20),
      [this] {
        return costmap_ && status_ && pub_goal_->get_subscription_count() > 0 &&
               pub_cost_update_->get_subscription_count() > 0;
      }))
      << "planner_3d/costmap_3d are not up";
    cnt_ = 0;
  }

  costmap_cspace_msgs::msg::CSpace3DUpdate emptyUpdate() const
  {
    costmap_cspace_msgs::msg::CSpace3DUpdate update;
    update.header.stamp = node_->now();
    update.header.frame_id = "map";
    update.width = update.height = update.angle = 0;
    return update;
  }
};

TEST_F(Planner3D, CostmapWatchdog)
{
  waitPlannerReady();

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 1.9;
  goal.pose.position.y = 2.8;
  goal.pose.orientation.w = 1.0;
  pub_goal_->publish(goal);

  rclcpp::WallRate rate(10.0);
  while (rclcpp::ok()) {
    // cnt increments in 5 Hz at maximum
    if (cnt_ == 0 || cnt_ > 8) {
      pub_cost_update_->publish(emptyUpdate());
    }

    rclcpp::spin_some(node_);
    rate.sleep();

    if (!status_) continue;

    if (5 < cnt_ && cnt_ < 8) {
      ASSERT_EQ(status_->error, planner_cspace_msgs::msg::PlannerStatus::DATA_MISSING);

      ASSERT_TRUE(static_cast<bool>(path_));
      ASSERT_EQ(path_->poses.size(), 0u);

      ASSERT_TRUE(static_cast<bool>(diag_));
      ASSERT_EQ(diag_->status.size(), 1u);
      ASSERT_EQ(diag_->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      ASSERT_NE(diag_->status[0].message.find("missing"), std::string::npos);
    } else if (10 < cnt_ && cnt_ < 13) {
      ASSERT_EQ(status_->status, planner_cspace_msgs::msg::PlannerStatus::DOING);
      ASSERT_EQ(status_->error, planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);

      ASSERT_EQ(diag_->status.size(), 1u);
      ASSERT_EQ(diag_->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
      ASSERT_NE(diag_->status[0].message.find("well"), std::string::npos);
    } else if (cnt_ >= 13) {
      return;
    }
  }
  ASSERT_TRUE(rclcpp::ok());
}

TEST_F(Planner3D, CostmapTimeoutOnFinishing)
{
  waitPlannerReady();

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 2.55;
  goal.pose.position.y = 0.45;
  goal.pose.orientation.w = std::sin(0.09);
  goal.pose.orientation.z = std::cos(0.09);
  pub_goal_->publish(goal);

  const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(5.0);
  rclcpp::WallRate rate(10.0);
  while (rclcpp::ok()) {
    pub_cost_update_->publish(emptyUpdate());

    rclcpp::spin_some(node_);
    rate.sleep();
    if (status_ && status_->status == planner_cspace_msgs::msg::PlannerStatus::FINISHING) break;

    ASSERT_LT(node_->now(), deadline)
      << "Planner didn't enter FINISHING state: " << (status_ ? status_->status : -1);
  }
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    rate.sleep();
    if (status_->error == planner_cspace_msgs::msg::PlannerStatus::DATA_MISSING) break;

    ASSERT_EQ(status_->status, planner_cspace_msgs::msg::PlannerStatus::FINISHING)
      << "Wrong test condition";
    ASSERT_LT(node_->now(), deadline)
      << "Planner didn't enter DATA_MISSING state" << status_->error;
  }
  path_ = nullptr;
  while (rclcpp::ok()) {
    pub_cost_update_->publish(emptyUpdate());

    rclcpp::spin_some(node_);
    rate.sleep();
    if (path_) break;

    ASSERT_EQ(status_->status, planner_cspace_msgs::msg::PlannerStatus::FINISHING)
      << "Wrong test condition";
    ASSERT_LT(node_->now(), deadline) << "No path was published";
  }
  ASSERT_TRUE(rclcpp::ok());
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
