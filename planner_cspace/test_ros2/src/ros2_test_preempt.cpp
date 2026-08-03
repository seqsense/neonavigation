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

// ROS 2 port of test/src/test_preempt.cpp (ROS 1 preempt_rostest.test),
// driven by test_preempt_launch.py.
//
// ROS 1 had a single PREEMPTED terminal state covering both "the client
// cancelled" and "a newer goal replaced this one". rclcpp_action separates
// them, so this test covers both explicitly:
//
//   * Preempt          - cancel request  -> ResultCode::CANCELED
//   * PreemptByNewGoal - superseded goal -> ResultCode::ABORTED with
//                        "Preempted." in NavigateToPose::Result::error_msg

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "planner_cspace/ros2_action_test_base.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class PreemptTest : public ActionTestBase<NavigateToPose, ACTION_TOPIC_MOVE_BASE>
{
protected:
  NavigateToPose::Goal createGoalInFree()
  {
    NavigateToPose::Goal goal;
    goal.pose.header.stamp = node_->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 1.24;
    goal.pose.pose.position.y = 0.65;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    return goal;
  }
};

TEST_F(PreemptTest, Preempt)
{
  ASSERT_TRUE(sendGoal(createGoalInFree())) << "Action didn't respond " << statusString();
  ASSERT_TRUE(goalHandle()) << "Action rejected the goal " << statusString();
  ASSERT_TRUE(isActive()) << "Action didn't get active: " << resultString() << statusString();

  ASSERT_TRUE(cancelAndWait(std::chrono::seconds(30)))
    << "Action didn't get inactive: " << resultString() << statusString();

  ASSERT_TRUE(planner_status_);

  // actionlib reported PREEMPTED here; rclcpp_action reports CANCELED because
  // the goal was terminated by an accepted cancel request.
  ASSERT_EQ(rclcpp_action::ResultCode::CANCELED, resultCode()) << resultString();
  ASSERT_EQ("Preempted.", result()->result->error_msg);

  ASSERT_TRUE(planner_cspace_testing::spinUntil(
    node_, std::chrono::seconds(10),
    [this] { return planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DONE; }))
    << "Planner didn't stop: " << statusString();
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::GOING_WELL, planner_status_->error);
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::DONE, planner_status_->status);
}

TEST_F(PreemptTest, PreemptByNewGoal)
{
  ASSERT_TRUE(sendGoal(createGoalInFree())) << "Action didn't respond " << statusString();
  ASSERT_TRUE(goalHandle()) << "Action rejected the goal " << statusString();
  ASSERT_TRUE(isActive()) << "Action didn't get active: " << resultString() << statusString();

  // Keep the first goal's session so that its result can still be inspected
  // after the next goal has taken over.
  const auto first = session();

  ASSERT_TRUE(sendGoal(createGoalInFree())) << "Action didn't respond " << statusString();
  ASSERT_TRUE(goalHandle()) << "Action rejected the second goal " << statusString();

  ASSERT_TRUE(waitResult(std::chrono::seconds(30), first))
    << "The superseded goal was not terminated " << statusString();

  // ROS 2 has no terminal state for "replaced by a newer goal", so the node
  // aborts the old goal and puts the ROS 1 status text into error_msg.
  ASSERT_EQ(rclcpp_action::ResultCode::ABORTED, first->result->code);
  ASSERT_EQ("Preempted.", first->result->result->error_msg);

  // The new goal keeps running.
  ASSERT_TRUE(cancelAndWait(std::chrono::seconds(30)))
    << "Action didn't get inactive: " << resultString() << statusString();
  ASSERT_EQ(rclcpp_action::ResultCode::CANCELED, resultCode()) << resultString();
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
