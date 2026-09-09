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

// ROS 2 port of test/src/test_planner_2dof_serial_joints.cpp (ROS 1
// planner_2dof_serial_joints_rostest.test), driven by
// test_planner_2dof_serial_joints_launch.py.
//
// The only interface difference is that the per-group settings are given as
// flat "group0.<key>" parameters instead of the ROS 1 "~/group0/<key>" nested
// namespace; that mapping is exercised by the launch description.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>

#include "planner_cspace/ros2_test_helpers.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

class Planner2DOFSerialJoints : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_state_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_cmd_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_plan_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;

  trajectory_msgs::msg::JointTrajectory::ConstSharedPtr planned_;
  planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr status_;
  rclcpp::Time started_;

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_planner_2dof_serial_joints");
    started_ = node_->now();
    pub_state_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", latchedQos());
    pub_cmd_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/trajectory_in", latchedQos());
    sub_plan_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory", latchedQos(),
      [this](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) { planned_ = msg; });
    // The node's status topic is latched (transient_local), so a fresh
    // subscription immediately replays the status left over by the previous
    // test case. Messages stamped before this fixture was created are dropped
    // so that each test only sees its own results.
    sub_status_ = node_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_2dof_serial_joints/group0/status", latchedQos(),
      [this](const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr msg) {
        if (rclcpp::Time(msg->header.stamp, RCL_ROS_TIME) >= started_) {
          status_ = msg;
        }
      });
  }

  sensor_msgs::msg::JointState currentState() const
  {
    sensor_msgs::msg::JointState s;
    s.name.push_back("front");
    s.position.push_back(-1.57);
    s.name.push_back("rear");
    s.position.push_back(0.0);
    return s;
  }

  trajectory_msgs::msg::JointTrajectory command(const double front, const double rear) const
  {
    trajectory_msgs::msg::JointTrajectory cmd;
    cmd.joint_names.push_back("front");
    cmd.joint_names.push_back("rear");
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.push_back(front);
    p.positions.push_back(rear);
    cmd.points.push_back(p);
    return cmd;
  }
};

TEST_F(Planner2DOFSerialJoints, Plan)
{
  const sensor_msgs::msg::JointState s = currentState();
  const trajectory_msgs::msg::JointTrajectory cmd = command(-4.71, 0.0);

  int cnt = 0;
  ASSERT_TRUE(spinUntil(
    node_, std::chrono::seconds(30),
    [this, &cnt] {
      if (planned_ && status_) {
        ++cnt;
      }
      return cnt > 5;
    },
    [this, &s, &cmd] {
      pub_state_->publish(s);
      pub_cmd_->publish(cmd);
    },
    std::chrono::seconds(1)))
    << "Timeout";

  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::DOING, status_->status);
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::GOING_WELL, status_->error);

  ASSERT_EQ(2u, planned_->joint_names.size());
  ASSERT_EQ("front", planned_->joint_names[0]);
  ASSERT_EQ("rear", planned_->joint_names[1]);

  // collision at front=-3.14, rear=0.0 must be avoided.
  const float fc = -3.14;
  const float rc = 0.0;
  for (int i = 1; i < static_cast<int>(planned_->points.size()); ++i) {
    const trajectory_msgs::msg::JointTrajectoryPoint & p0 = planned_->points[i - 1];
    const trajectory_msgs::msg::JointTrajectoryPoint & p1 = planned_->points[i];
    ASSERT_EQ(2u, p0.positions.size());
    ASSERT_EQ(2u, p1.positions.size());

    const float f0 = p0.positions[0];
    const float r0 = p0.positions[1];
    const float f1 = p1.positions[0];
    const float r1 = p1.positions[1];

    ASSERT_LT(-6.28, f0);
    ASSERT_GT(0.0, f0);
    ASSERT_LT(-3.14, r0);
    ASSERT_GT(3.14, r0);
    ASSERT_LT(-6.28, f1);
    ASSERT_GT(0.0, f1);
    ASSERT_LT(-3.14, r1);
    ASSERT_GT(3.14, r1);

    const float front_diff = f1 - f0;
    const float rear_diff = r1 - r0;

    const float d = std::abs(rear_diff * fc - front_diff * rc + f1 * r0 - r1 * f0) /
                    std::hypot(front_diff, rear_diff);
    ASSERT_GT(d, 0.15);

    ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::DOING, status_->status);
    ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::GOING_WELL, status_->error);
  }
}

TEST_F(Planner2DOFSerialJoints, NoPath)
{
  const sensor_msgs::msg::JointState s = currentState();
  // Collided state
  const trajectory_msgs::msg::JointTrajectory cmd = command(3.14, 0.0);

  int cnt = 0;
  ASSERT_TRUE(spinUntil(
    node_, std::chrono::seconds(30),
    [this, &cnt] {
      if (status_) {
        ++cnt;
      }
      return cnt > 5;
    },
    [this, &s, &cmd] {
      pub_state_->publish(s);
      pub_cmd_->publish(cmd);
    },
    std::chrono::seconds(1)))
    << "Timeout";

  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::DOING, status_->status);
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND, status_->error);
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
