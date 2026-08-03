/*
 * Copyright (c) 2022, the neonavigation authors
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

// ROS 2 port of test/src/test_planner_3d_map_size.cpp (ROS 1
// planner_3d_map_size_rostest.test), driven by
// test_planner_3d_map_size_launch.py.
//
// Feeds planner_3d costmap updates whose region does not fit in the costmap it
// was given and checks that the node survives them, i.e. that it keeps
// publishing ~/status afterwards.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "planner_cspace/ros2_test_helpers.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

class Planner3DMapSize : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;
  rclcpp::Publisher<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr pub_map_;
  rclcpp::Publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>::SharedPtr pub_map_update_;
  size_t cnt_status_;

  Planner3DMapSize() : cnt_status_(0)
  {
    node_ = rclcpp::Node::make_shared("test_planner_cspace_map_size");
    // ~/status is latched (transient_local) on the ROS 2 node, so the
    // subscription has to request the same durability.
    sub_status_ = node_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_3d/status", latchedQos(100),
      [this](const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr &) { ++cnt_status_; });
    pub_map_ =
      node_->create_publisher<costmap_cspace_msgs::msg::CSpace3D>("/costmap", latchedQos());
    pub_map_update_ = node_->create_publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>(
      "/costmap_update", latchedQos());
  }

  void SetUp() override
  {
    // Wait planner
    ASSERT_TRUE(waitStatus(std::chrono::seconds(10))) << "planner_3d is not up";
  }

  bool waitStatus(const std::chrono::nanoseconds timeout)
  {
    return spinUntil(node_, timeout, [this] { return cnt_status_ > 5; });
  }

  void sleepAndSpin(const double sec)
  {
    spinUntil(
      node_, std::chrono::nanoseconds(static_cast<int64_t>(sec * 1e9)), [] { return false; },
      nullptr, std::chrono::milliseconds(10));
  }

  costmap_cspace_msgs::msg::CSpace3D generateCSpace3DMsg(
    const rclcpp::Time & stamp, const size_t w, const size_t h, const size_t angle)
  {
    costmap_cspace_msgs::msg::CSpace3D msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "msg";
    msg.info.width = w;
    msg.info.height = h;
    msg.info.angle = angle;
    msg.info.linear_resolution = 0.1;
    msg.info.angular_resolution = M_PI * 2 / angle;
    msg.info.origin.orientation.w = 1;
    msg.data.resize(msg.info.width * msg.info.height * msg.info.angle);
    for (auto & c : msg.data) {
      c = 100;
    }
    return msg;
  }

  costmap_cspace_msgs::msg::CSpace3DUpdate generateCSpace3DUpdateMsg(
    const rclcpp::Time & stamp, const size_t x, const size_t y, const size_t yaw, const size_t w,
    const size_t h, const size_t angle)
  {
    costmap_cspace_msgs::msg::CSpace3DUpdate msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "msg";
    msg.x = x;
    msg.y = y;
    msg.yaw = yaw;
    msg.width = w;
    msg.height = h;
    msg.angle = angle;
    msg.data.resize(msg.width * msg.height * msg.angle);
    for (auto & c : msg.data) {
      c = 100;
    }
    return msg;
  }

  // Common body of the out-of-range tests: publish a costmap, then an update
  // of the given geometry, and require that the planner is still alive.
  void publishAndExpectAlive(
    const size_t x, const size_t y, const size_t yaw, const size_t w, const size_t h,
    const size_t angle)
  {
    const rclcpp::Time now = node_->now();
    pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
    sleepAndSpin(0.1);
    pub_map_update_->publish(generateCSpace3DUpdateMsg(now, x, y, yaw, w, h, angle));
    sleepAndSpin(0.5);
    cnt_status_ = 0;
    ASSERT_TRUE(waitStatus(std::chrono::seconds(5)));
  }
};

TEST_F(Planner3DMapSize, OutOfRangeX) { publishAndExpectAlive(0, 0, 0, 0x81, 0x80, 4); }

TEST_F(Planner3DMapSize, OutOfRangeY) { publishAndExpectAlive(0, 0, 0, 0x80, 0x81, 4); }

TEST_F(Planner3DMapSize, OutOfRangeAngle) { publishAndExpectAlive(0, 0, 0, 0x80, 0x80, 8); }

TEST_F(Planner3DMapSize, OutOfRangeAll) { publishAndExpectAlive(0, 0, 0, 0x81, 0x81, 8); }

TEST_F(Planner3DMapSize, ZeroSizeUpdate) { publishAndExpectAlive(1, 1, 0, 0, 0, 0); }

TEST_F(Planner3DMapSize, IllOrderedUpdate)
{
  const rclcpp::Time now = node_->now();
  const rclcpp::Time next = now + rclcpp::Duration::from_seconds(0.1);

  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  sleepAndSpin(0.1);

  pub_map_update_->publish(generateCSpace3DUpdateMsg(next, 0, 0, 0, 0x81, 0x81, 8));
  sleepAndSpin(0.1);

  pub_map_->publish(generateCSpace3DMsg(next, 0x81, 0x81, 8));
  sleepAndSpin(0.5);

  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(std::chrono::seconds(5)));
}

TEST_F(Planner3DMapSize, IllOrderedUpdateShrink)
{
  const rclcpp::Time now = node_->now();
  const rclcpp::Time next = now + rclcpp::Duration::from_seconds(0.1);

  pub_map_->publish(generateCSpace3DMsg(now, 0x80, 0x80, 4));
  sleepAndSpin(0.1);

  pub_map_update_->publish(generateCSpace3DUpdateMsg(next, 0, 0, 0, 0x81, 0x81, 8));
  sleepAndSpin(0.1);

  pub_map_->publish(generateCSpace3DMsg(next, 0x40, 0x40, 4));
  sleepAndSpin(0.5);

  cnt_status_ = 0;
  ASSERT_TRUE(waitStatus(std::chrono::seconds(5)));
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
