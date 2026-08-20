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

// ROS 2 port of test/src/test_tolerant_action.cpp (ROS 1
// tolerant_action_rostest.test), driven by test_tolerant_action_launch.py.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "planner_cspace/ros2_action_test_base.h"
#include "planner_cspace_msgs/action/move_with_tolerance.hpp"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace
{
using MoveWithTolerance = planner_cspace_msgs::action::MoveWithTolerance;

class TolerantActionTest : public ActionTestBase<MoveWithTolerance, ACTION_TOPIC_TOLERANT_MOVE>
{
protected:
  MoveWithTolerance::Goal createGoalInFree()
  {
    MoveWithTolerance::Goal goal;
    goal.target_pose.header.stamp = node_->now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 2.1;
    goal.target_pose.pose.position.y = 0.45;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;
    goal.continuous_movement_mode = true;
    goal.goal_tolerance_ang = 0.1;
    goal.goal_tolerance_ang_finish = 0.05;
    goal.goal_tolerance_lin = 0.2;
    return goal;
  }

  double getDistBetweenRobotAndGoal(const MoveWithTolerance::Goal & goal)
  {
    try {
      const geometry_msgs::msg::TransformStamped map_to_robot =
        tfbuf_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));
      return std::hypot(
        map_to_robot.transform.translation.x - goal.target_pose.pose.position.x,
        map_to_robot.transform.translation.y - goal.target_pose.pose.position.y);
    } catch (const std::exception &) {
      return std::numeric_limits<double>::max();
    }
  }
};

TEST_F(TolerantActionTest, GoalWithTolerance)
{
  const MoveWithTolerance::Goal goal = createGoalInFree();
  ASSERT_TRUE(sendGoal(goal)) << "Action didn't respond " << statusString();
  ASSERT_TRUE(goalHandle()) << "Action rejected the goal " << statusString();

  ASSERT_TRUE(waitResult(std::chrono::seconds(60)))
    << "Action didn't finish: " << resultString() << " " << statusString();
  ASSERT_EQ(rclcpp_action::ResultCode::SUCCEEDED, resultCode()) << resultString();

  const double dist_to_goal = getDistBetweenRobotAndGoal(goal);
  // distance_remains is less than updated goal_tolerance_lin
  // (set in planner_cspace_msgs::action::MoveWithTolerance::Goal).
  EXPECT_LT(dist_to_goal, goal.goal_tolerance_lin);
  // distance_remains is greater than default goal_tolerance_lin
  // (set in actionlib_common.py).
  EXPECT_GT(dist_to_goal, 0.05);
  // Navigation still continues after the action client succeeded.
  ASSERT_TRUE(planner_status_);
  EXPECT_EQ(planner_status_->status, planner_cspace_msgs::msg::PlannerStatus::DOING);

  ASSERT_TRUE(
    planner_cspace_testing::spinUntil(
      node_, std::chrono::seconds(60),
      [this] { return planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DONE; }))
    << "Navigation didn't finish: " << resultString() << " " << statusString();
  EXPECT_LT(getDistBetweenRobotAndGoal(goal), 0.05);
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
