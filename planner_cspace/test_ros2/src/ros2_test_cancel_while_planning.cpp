/*
 * Copyright (c) 2026, the neonavigation authors
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

// Driven by test_cancel_while_planning_launch.py.
//
// Checks that planner_3d accepts a cancel request while it is in the middle of
// a long search, instead of after the search, and that the goal is canceled
// once the search is over.
//
// The costmap makes the search long: a thick wall splits the map, and its only
// gap is free at headings of +-90 deg only. The goal behind the wall is
// reachable on the 2D cost estimation (which takes the cheapest heading of
// each cell) but not for a robot that moves along its heading, so the search
// explores the whole reachable side before it gives up. The launch file sets
// local_range so that the whole map is searched in 3D, and freq_min so that
// the search logs "Search timed out" while it is running.

// ament_auto links planner_3d_debug_component into every executable of this
// package, and with it its public DEBUG definition, which would expand inside
// rcl_interfaces::msg::Log::DEBUG.
#undef DEBUG

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "planner_cspace/ros2_test_helpers.h"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

constexpr int WIDTH = 100;
constexpr int HEIGHT = 120;
constexpr int ANGLE = 16;
constexpr double RESOLUTION = 0.1;
constexpr int WALL_X_MIN = 50;
constexpr int WALL_X_MAX = 59;
constexpr int GAP_Y_MIN = 55;
constexpr int GAP_Y_MAX = 64;
constexpr std::chrono::milliseconds SPIN_INTERVAL(5);

class CancelWhilePlanningTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_cancel_while_planning");
    pub_map_ =
      node_->create_publisher<costmap_cspace_msgs::msg::CSpace3D>("/costmap", latchedQos());
    sub_rosout_ = node_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100), [this](const rcl_interfaces::msg::Log::ConstSharedPtr msg) {
        if (msg->name == "planner_3d" && msg->msg.rfind("Search timed out", 0) == 0) {
          ++search_progress_;
        }
      });
    tfb_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/move_base");

    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = node_->now();
    trans.header.frame_id = "map";
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = 2.0;
    trans.transform.translation.y = 6.0;
    trans.transform.rotation.w = 1.0;
    tfb_->sendTransform(trans);
    pub_map_->publish(generateMap());

    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30), [this] { return action_client_->action_server_is_ready(); },
      nullptr, SPIN_INTERVAL))
      << "move_base action server is not up";
    // The poses planIsAvailable() plans between are free on this map as well.
    srv_plan_ = node_->create_client<nav_msgs::srv::GetPlan>("/planner_3d/make_plan");
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30),
      [this] { return planner_cspace_testing::planIsAvailable(node_, srv_plan_); }, nullptr,
      std::chrono::milliseconds(500)))
      << "planner_3d didn't receive the map";
  }

  costmap_cspace_msgs::msg::CSpace3D generateMap() const
  {
    costmap_cspace_msgs::msg::CSpace3D msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "map";
    msg.info.width = WIDTH;
    msg.info.height = HEIGHT;
    msg.info.angle = ANGLE;
    msg.info.linear_resolution = RESOLUTION;
    msg.info.angular_resolution = M_PI * 2 / ANGLE;
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(WIDTH * HEIGHT * ANGLE, 0);
    for (int a = 0; a < ANGLE; ++a) {
      const bool sideways = (a == ANGLE / 4 || a == ANGLE * 3 / 4);
      for (int y = 0; y < HEIGHT; ++y) {
        const bool gap = sideways && GAP_Y_MIN <= y && y <= GAP_Y_MAX;
        for (int x = WALL_X_MIN; x <= WALL_X_MAX; ++x) {
          msg.data[(a * HEIGHT + y) * WIDTH + x] = gap ? 0 : 100;
        }
      }
    }
    return msg;
  }

  NavigateToPose::Goal goalBehindWall() const
  {
    NavigateToPose::Goal goal;
    goal.pose.header.stamp = node_->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 8.0;
    goal.pose.pose.position.y = 6.0;
    goal.pose.pose.orientation.w = 1.0;
    return goal;
  }

  template <typename T>
  bool waitFuture(std::shared_future<T> & future, const std::chrono::nanoseconds timeout)
  {
    return spinUntil(
      node_, timeout,
      [&future] { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
      nullptr, SPIN_INTERVAL);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr pub_map_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_rosout_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfb_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_;
  int search_progress_ = 0;
};

TEST_F(CancelWhilePlanningTest, CancelIsAcceptedDuringSearch)
{
  std::shared_ptr<GoalHandle::WrappedResult> result;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.result_callback = [&result](const GoalHandle::WrappedResult & r) {
    result = std::make_shared<GoalHandle::WrappedResult>(r);
  };
  auto goal_future = action_client_->async_send_goal(goalBehindWall(), options);
  ASSERT_TRUE(waitFuture(goal_future, std::chrono::seconds(10))) << "Goal was not answered";
  const GoalHandle::SharedPtr goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle) << "Goal was rejected";

  ASSERT_TRUE(spinUntil(
    node_, std::chrono::seconds(30), [this] { return search_progress_ > 0; }, nullptr,
    SPIN_INTERVAL))
    << "planner_3d did not start a long search";

  auto cancel_future = action_client_->async_cancel_goal(goal_handle);
  ASSERT_TRUE(waitFuture(cancel_future, std::chrono::seconds(30))) << "Cancel was not answered";
  ASSERT_EQ(action_msgs::srv::CancelGoal::Response::ERROR_NONE, cancel_future.get()->return_code);
  const int progress_at_cancel_accepted = search_progress_;

  ASSERT_TRUE(spinUntil(
    node_, std::chrono::seconds(60), [&result] { return static_cast<bool>(result); }, nullptr,
    SPIN_INTERVAL))
    << "Goal was not finished after the cancel";
  EXPECT_EQ(rclcpp_action::ResultCode::CANCELED, result->code);

  // The search which was running when the cancel was sent kept reporting its
  // progress after the cancel had been accepted.
  EXPECT_GT(search_progress_, progress_at_cancel_accepted)
    << "The cancel was accepted only after the search had finished";
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
