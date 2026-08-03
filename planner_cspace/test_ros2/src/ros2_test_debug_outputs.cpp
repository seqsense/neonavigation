/*
 * Copyright (c) 2018, the neonavigation authors
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

// ROS 2 port of test/src/test_debug_outputs.cpp (ROS 1
// debug_outputs_rostest.test), driven by test_debug_outputs_launch.py.
//
// The ROS 1 launch file kept a `rostopic pub -l` process alive to hold the
// goal; here the goal is published by the test itself once planner_3d has
// subscribed, which also removes the dependency on the rostopic package.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "neonavigation_metrics_msgs/msg/metrics.hpp"
#include "planner_cspace/ros2_test_helpers.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace
{
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

class DebugOutputsTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    cnt_planner_ready_ = 0;
    cnt_path_ = 0;
    cnt_hysteresis_ = 0;
    cnt_remembered_ = 0;
    cnt_distance_ = 0;

    node_ = rclcpp::Node::make_shared("test_debug_outputs");
    // planner_3d only generates the debug maps while somebody subscribes to
    // them, so the subscriptions are created before the goal is sent.
    sub_status_ = node_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_3d/status", latchedQos(),
      [this](const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr msg) {
        if (
          msg->error == planner_cspace_msgs::msg::PlannerStatus::GOING_WELL &&
          msg->status == planner_cspace_msgs::msg::PlannerStatus::DOING) {
          ++cnt_planner_ready_;
        }
      });
    sub_metrics_ = node_->create_subscription<neonavigation_metrics_msgs::msg::Metrics>(
      "/planner_3d/metrics", rclcpp::QoS(1),
      [this](const neonavigation_metrics_msgs::msg::Metrics::ConstSharedPtr msg) {
        metrics_ = msg;
      });
    sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/path", latchedQos(), [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
        if (msg->poses.size() > 0) {
          ++cnt_path_;
          path_ = msg;
        }
      });
    sub_hysteresis_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/planner_3d/hysteresis_map", latchedQos(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        ++cnt_hysteresis_;
        map_hysteresis_ = msg;
      });
    sub_remembered_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/planner_3d/remembered_map", latchedQos(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        ++cnt_remembered_;
        map_remembered_ = msg;
      });
    sub_distance_ = node_->create_subscription<sensor_msgs::msg::PointCloud>(
      "/planner_3d/distance_map", latchedQos(),
      [this](const sensor_msgs::msg::PointCloud::ConstSharedPtr msg) {
        ++cnt_distance_;
        map_distance_ = msg;
      });
    // The ROS 1 launch file held the goal with a latched `rostopic pub -l`.
    // planner_3d subscribes "goal_pose" with a volatile QoS, so the goal has
    // to be published after the subscription has been matched.
    pub_goal_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", latchedQos());
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30), [this] { return pub_goal_->get_subscription_count() > 0; }))
      << "planner_3d is not up";

    // Same goal as test/data/goal_debug_outputs.yaml.
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 0.525;
    goal.pose.orientation.z = 1.0;
    goal.pose.orientation.w = 0.0;

    // Wait planner. The goal is re-sent until the first path comes back, but
    // not afterwards: re-setting the goal would drop the hysteresis the
    // Hysteresis test is about.
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(60), [this] { return cnt_planner_ready_ > 5 && cnt_path_ > 5; },
      [this, &goal] {
        if (cnt_path_ == 0) {
          pub_goal_->publish(goal);
        }
      },
      std::chrono::milliseconds(100)))
      << "planner_3d didn't start planning";

    map_hysteresis_ = nullptr;
    map_remembered_ = nullptr;
    map_distance_ = nullptr;
    cnt_hysteresis_ = 0;
    cnt_remembered_ = 0;
    cnt_distance_ = 0;

    // Wait receiving the messages
    // First hysteresis map doesn't have previous path information.
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30),
      [this] { return cnt_hysteresis_ > 2 && cnt_remembered_ > 2; }))
      << "debug maps are not published";
  }

  void showMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
  {
    std::cerr << std::hex;
    for (size_t y = 0; y < msg->info.height; ++y) {
      for (size_t x = 0; x < msg->info.width; ++x) {
        std::cerr << (msg->data[x + y * msg->info.width] / 11);
      }
      std::cerr << std::endl;
    }
    std::cerr << std::dec;

    std::streamsize ss = std::cerr.precision();
    std::cerr << "path:" << std::endl << std::setprecision(3);
    for (const auto & p : path_->poses) {
      std::cerr << p.pose.position.x << ", " << p.pose.position.y << ", " << std::endl;
    }

    std::cerr << std::setprecision(ss);
  }

protected:
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_hysteresis_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_remembered_;
  sensor_msgs::msg::PointCloud::ConstSharedPtr map_distance_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  neonavigation_metrics_msgs::msg::Metrics::ConstSharedPtr metrics_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_hysteresis_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_remembered_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_distance_;
  rclcpp::Subscription<neonavigation_metrics_msgs::msg::Metrics>::SharedPtr sub_metrics_;
  int cnt_planner_ready_;
  int cnt_path_;
  int cnt_hysteresis_;
  int cnt_remembered_;
  int cnt_distance_;
};

struct PositionAndValue
{
  int x;
  int y;
  char value;
};

TEST_F(DebugOutputsTest, Hysteresis)
{
  ASSERT_TRUE(static_cast<bool>(map_hysteresis_));

  // Robot is at (25, 4) and goal is at (10, 4)
  const PositionAndValue data_set[] = {
    {25, 4, 0},   {20, 4, 0},   {15, 4, 0},   {10, 4, 0},   {25, 1, 100}, {20, 1, 100},
    {15, 1, 100}, {10, 1, 100}, {25, 7, 100}, {20, 7, 100}, {15, 7, 100}, {10, 7, 100},
  };
  for (auto data : data_set) {
    const size_t addr = data.x + data.y * map_hysteresis_->info.width;
    EXPECT_EQ(data.value, map_hysteresis_->data[addr]) << "x: " << data.x << ", y: " << data.y;
  }
  if (::testing::Test::HasFailure()) {
    showMap(map_hysteresis_);
  }
}

TEST_F(DebugOutputsTest, Remembered)
{
  ASSERT_TRUE(static_cast<bool>(map_remembered_));

  // Robot is at (25, 5) and obstacles are placed at
  // (0, 0)-(31, 0) and (18, 10)-(31, 10).
  // Costmap is expanded by 1 grid.
  const PositionAndValue data_set[] = {
    {17, 0, 100},   // occupied
    {17, 1, 100},   // expanded
    {17, 2, 0},     // free
    {17, 8, 0},     // free
    {17, 9, 100},   // expanded
    {17, 10, 100},  // occupied
  };
  for (auto data : data_set) {
    const size_t addr = data.x + data.y * map_remembered_->info.width;
    EXPECT_EQ(data.value, map_remembered_->data[addr]) << "x: " << data.x << ", y: " << data.y;
  }
  if (::testing::Test::HasFailure()) {
    showMap(map_remembered_);
  }
}

TEST_F(DebugOutputsTest, Distance)
{
  ASSERT_TRUE(spinUntil(
    node_, std::chrono::seconds(30), [this] { return static_cast<bool>(map_distance_); }));

  // Robot is at (2.5, 0.5) and goal is at (1.0, 0.5)
  for (size_t i = 0; i < map_distance_->points.size(); ++i) {
    const float x = map_distance_->points[i].x;
    const float y = map_distance_->points[i].y;
    const float d = map_distance_->channels[0].values[i];
    if (std::abs(y - 0.5) < 0.05) {
      const float dist_from_goal = std::abs(x - 1.0);
      ASSERT_NEAR(dist_from_goal, d, 0.15);
    }
  }
}

TEST_F(DebugOutputsTest, Metrics)
{
  metrics_ = nullptr;
  ASSERT_TRUE(
    spinUntil(node_, std::chrono::seconds(10), [this] { return static_cast<bool>(metrics_); }));
  ASSERT_NE(0u, metrics_->data.size());
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
