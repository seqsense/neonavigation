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

// ROS 2 port of test/src/test_abort.cpp (ROS 1 abort_rostest.test), driven by
// test_abort_launch.py.

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

class AbortTest : public ActionTestBase<NavigateToPose, ACTION_TOPIC_MOVE_BASE>
{
protected:
  NavigateToPose::Goal createGoalInRock()
  {
    NavigateToPose::Goal goal;
    goal.pose.header.stamp = node_->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 1.19;
    goal.pose.pose.position.y = 1.90;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    return goal;
  }
  NavigateToPose::Goal createGoalInFree()
  {
    NavigateToPose::Goal goal;
    goal.pose.header.stamp = node_->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 2.1;
    goal.pose.pose.position.y = 0.45;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 1.0;
    goal.pose.pose.orientation.w = 0.0;
    return goal;
  }
};

TEST_F(AbortTest, AbortByGoalInRock)
{
  // Send a goal which is in Rock
  ASSERT_TRUE(sendGoal(createGoalInRock())) << "Action didn't respond " << statusString();
  ASSERT_TRUE(goalHandle()) << "Action rejected the goal " << statusString();

  // Try to replan until the planner gives up (max_retry_num)
  ASSERT_TRUE(waitResult(std::chrono::seconds(30)))
    << "Action didn't get inactive: " << resultString() << " " << statusString();

  ASSERT_TRUE(planner_status_);

  // Abort after exceeding max_retry_num
  ASSERT_EQ(rclcpp_action::ResultCode::ABORTED, resultCode()) << resultString();
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND, planner_status_->error);

  // Send another goal which is not in Rock
  ASSERT_TRUE(sendGoal(createGoalInFree())) << "Action didn't respond " << statusString();
  ASSERT_TRUE(goalHandle()) << "Action rejected the goal " << statusString();
  ASSERT_TRUE(waitResult(std::chrono::seconds(60)))
    << "Action didn't get inactive: " << resultString() << " " << statusString();

  // Succeed
  ASSERT_EQ(rclcpp_action::ResultCode::SUCCEEDED, resultCode()) << resultString();
  ASSERT_EQ(planner_cspace_msgs::msg::PlannerStatus::GOING_WELL, planner_status_->error);
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
