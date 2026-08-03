/*
 * Copyright (c) 2023, the neonavigation authors
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

// ROS 2 port of test/src/test_dynamic_parameter_change.cpp (ROS 1
// dynamic_parameter_change_rostest.test), driven by
// test_dynamic_parameter_change_launch.py.
//
// dynamic_reconfigure::Client<Planner3DConfig> is replaced by
// rclcpp::AsyncParametersClient. The two differ in one way that matters here:
// setConfiguration() pushed the *whole* config, so every value the previous
// test case had changed went back to its default, while set_parameters() only
// touches the names it is given. defaultParameters() below therefore lists
// every parameter these tests modify, and each test re-sends that whole list
// before applying its own overrides.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_cspace/ros2_action_test_base.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

// Default of the planner_3d "freq" parameter.
constexpr double kDefaultFreq = 4.0;

class DynamicParameterChangeTest : public ActionTestBase<NavigateToPose, ACTION_TOPIC_MOVE_BASE>
{
public:
  void SetUp() final
  {
    path_ = nullptr;
    path_received_count_ = 0;
    last_path_received_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    setUpNode();
    tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/planner_3d");
    sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/path", latchedQos(), [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
        path_ = msg;
        ++path_received_count_;
        last_path_received_time_ = node_->now();
      });
    pub_map_overlay_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_overlay", latchedQos());
    // not actually used
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", latchedQos());

    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30),
      [this] {
        return sub_path_->get_publisher_count() > 0 &&
               pub_map_overlay_->get_subscription_count() > 0 && params_client_->service_is_ready();
      }))
      << "planner_3d/costmap_3d are not up";

    map_overlay_.header.frame_id = "map";
    map_overlay_.info.resolution = 0.1;
    map_overlay_.info.width = 32;
    map_overlay_.info.height = 32;
    map_overlay_.info.origin.position.x = 0.0;
    map_overlay_.info.origin.position.y = 0.0;
    map_overlay_.info.origin.position.z = 0.0;
    map_overlay_.info.origin.orientation.x = 0.0;
    map_overlay_.info.origin.orientation.y = 0.0;
    map_overlay_.info.origin.orientation.z = 0.0;
    map_overlay_.info.origin.orientation.w = 1.0;
    map_overlay_.data.resize(map_overlay_.info.width * map_overlay_.info.height, 0);
    publishMapAndRobot(0, 0, 0);

    waitPlannerReady();

    ASSERT_TRUE(setParameters(defaultParameters()));
  }

  void TearDown() final
  {
    if (!action_client_) {
      return;
    }
    auto future = action_client_->async_cancel_all_goals();
    spinUntil(node_, std::chrono::seconds(5), [&future] {
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
    spinUntil(node_, std::chrono::milliseconds(500), [] { return false; });
  }

protected:
  // Counterpart of Planner3DConfig::__getDefault__() plus the overrides the
  // ROS 1 fixture applied on top of it. Also pins every parameter which one of
  // the tests below changes, so that re-sending this list restores the state
  // setConfiguration() used to restore implicitly.
  std::vector<rclcpp::Parameter> defaultParameters() const
  {
    return {
      rclcpp::Parameter("max_retry_num", 5),
      rclcpp::Parameter("tolerance_range", 0.0),
      rclcpp::Parameter("temporary_escape", false),
      rclcpp::Parameter("goal_tolerance_lin", 0.05),
      rclcpp::Parameter("cost_in_place_turn", 3.0),
      rclcpp::Parameter("min_curve_radius", 0.4),
      rclcpp::Parameter("max_vel", 0.3),
      rclcpp::Parameter("max_ang_vel", 1.0),
      rclcpp::Parameter("freq", kDefaultFreq),
      rclcpp::Parameter("keep_a_part_of_previous_path", false),
      rclcpp::Parameter("dist_stop_to_previous_path", 0.1),
      rclcpp::Parameter("trigger_plan_by_costmap_update", false),
      rclcpp::Parameter("costmap_watchdog", 0.0),
    };
  }

  bool setParameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    auto future = params_client_->set_parameters(parameters);
    if (!spinUntil(node_, std::chrono::seconds(10), [&future] {
          return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        })) {
      return false;
    }
    for (const auto & result : future.get()) {
      if (!result.successful) {
        return false;
      }
    }
    return true;
  }

  NavigateToPose::Goal createGoalInFree()
  {
    NavigateToPose::Goal goal;
    goal.pose.header.stamp = node_->now();
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = 1.25;
    goal.pose.pose.position.y = 1.05;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = std::sin(M_PI / 4);
    goal.pose.pose.orientation.w = std::cos(M_PI / 4);
    return goal;
  }

  bool isPathIncludingCurves() const
  {
    for (size_t i = 0; i + 1 < path_->poses.size(); ++i) {
      const auto & pose1 = path_->poses[i];
      const auto & pose2 = path_->poses[i + 1];
      const double distance = std::hypot(
        pose1.pose.position.x - pose2.pose.position.x,
        pose1.pose.position.y - pose2.pose.position.y);
      const double yaw_diff =
        std::abs(tf2::getYaw(pose1.pose.orientation) - tf2::getYaw(pose2.pose.orientation));
      if (distance > 1.0e-3 && yaw_diff > 1.0e-3) {
        return true;
      }
    }
    return false;
  }

  ::testing::AssertionResult comparePath(
    const nav_msgs::msg::Path & path1, const nav_msgs::msg::Path & path2)
  {
    if (path1.poses.size() != path2.poses.size()) {
      return ::testing::AssertionFailure()
             << "Path size different: " << path1.poses.size() << " != " << path2.poses.size();
    }
    for (size_t i = 0; i < path1.poses.size(); ++i) {
      const geometry_msgs::msg::Point & pos1 = path1.poses[i].pose.position;
      const geometry_msgs::msg::Point & pos2 = path2.poses[i].pose.position;
      if (std::abs(pos1.x - pos2.x) > 1.0e-6) {
        return ::testing::AssertionFailure()
               << "X different at #" << i << ": " << pos1.x << " != " << pos2.x;
      }
      if (std::abs(pos1.y - pos2.y) > 1.0e-6) {
        return ::testing::AssertionFailure()
               << "Y different at #" << i << ": " << pos1.y << " != " << pos2.y;
      }
      const double yaw1 = tf2::getYaw(path1.poses[i].pose.orientation);
      const double yaw2 = tf2::getYaw(path2.poses[i].pose.orientation);
      if (std::abs(yaw1 - yaw2) > 1.0e-6) {
        return ::testing::AssertionFailure()
               << "Yaw different at #" << i << ": " << yaw1 << " != " << yaw2;
      }
    }
    return ::testing::AssertionSuccess();
  }

  void sendGoalAndWaitForPath()
  {
    ASSERT_TRUE(sendGoal(createGoalInFree())) << "Action didn't respond " << statusString();
    ASSERT_TRUE(goalHandle()) << "Action rejected the goal " << statusString();

    spinSome();  // Flush message buffer
    path_ = nullptr;

    const rclcpp::Time start_time = node_->now();
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(10),
      [this, start_time] {
        return path_ && rclcpp::Time(path_->header.stamp, RCL_ROS_TIME) > start_time &&
               path_->poses.size() > 0;
      },
      nullptr, std::chrono::milliseconds(100)))
      << "Failed to plan: " << resultString() << statusString();
  }

  void publishMapAndRobot(const double x, const double y, const double yaw)
  {
    const rclcpp::Time current_time = node_->now();
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = current_time;
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";
    trans.transform.translation = tf2::toMsg(tf2::Vector3(x, y, 0.0));
    trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    tfb_->sendTransform(trans);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = current_time;
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    pub_odom_->publish(odom);

    pub_map_overlay_->publish(map_overlay_);
  }

  void sleepAndSpin(const double sec)
  {
    spinUntil(
      node_, std::chrono::nanoseconds(static_cast<int64_t>(sec * 1e9)), [] { return false; },
      nullptr, std::chrono::milliseconds(10));
  }

  double getAveragePathInterval(const rclcpp::Duration & costmap_publishing_interval)
  {
    publishMapAndRobot(2.55, 0.45, M_PI);
    sleepAndSpin(0.3);
    EXPECT_TRUE(sendGoal(createGoalInFree()));

    last_path_received_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publishMapAndRobot(2.55, 0.45, M_PI);
    rclcpp::Time last_costmap_publishing_time = node_->now();
    const auto republish = [this, &last_costmap_publishing_time, &costmap_publishing_interval] {
      if ((node_->now() - last_costmap_publishing_time) > costmap_publishing_interval) {
        publishMapAndRobot(2.55, 0.45, M_PI);
        last_costmap_publishing_time = node_->now();
      }
    };
    EXPECT_TRUE(spinUntil(
      node_, std::chrono::seconds(30),
      [this] { return last_path_received_time_ != rclcpp::Time(0, 0, RCL_ROS_TIME); }, republish,
      std::chrono::milliseconds(10)));

    const rclcpp::Time initial_path_received_time = last_path_received_time_;
    const int prev_path_received_count = path_received_count_;
    EXPECT_TRUE(spinUntil(
      node_, std::chrono::seconds(60),
      [this, prev_path_received_count] {
        return path_received_count_ >= prev_path_received_count + 10;
      },
      republish, std::chrono::milliseconds(10)));

    return (last_path_received_time_ - initial_path_received_time).seconds() /
           (path_received_count_ - prev_path_received_count);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  std::shared_ptr<rclcpp::AsyncParametersClient> params_client_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_overlay_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  nav_msgs::msg::OccupancyGrid map_overlay_;
  int path_received_count_;
  rclcpp::Time last_path_received_time_;
};

TEST_F(DynamicParameterChangeTest, DisableCurves)
{
  publishMapAndRobot(2.55, 0.45, M_PI);
  sleepAndSpin(0.5);

  sendGoalAndWaitForPath();
  // The default path is including curves.
  EXPECT_TRUE(isPathIncludingCurves());

  // Large min_curve_radius disables curves.
  auto config = defaultParameters();
  config.push_back(rclcpp::Parameter("min_curve_radius", 10.0));
  ASSERT_TRUE(setParameters(config));

  sendGoalAndWaitForPath();
  EXPECT_FALSE(isPathIncludingCurves());
}

TEST_F(DynamicParameterChangeTest, StartPosePrediction)
{
  publishMapAndRobot(1.65, 0.65, M_PI);
  sleepAndSpin(0.5);
  sendGoalAndWaitForPath();
  const nav_msgs::msg::Path initial_path = *path_;

  // The path is changed to keep distance from the obstacle.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 100;
  publishMapAndRobot(1.65, 0.65, M_PI);
  sleepAndSpin(0.5);
  sendGoalAndWaitForPath();
  EXPECT_FALSE(comparePath(initial_path, *path_));

  // Enable start pose prediction.
  action_client_->async_cancel_all_goals();
  sleepAndSpin(0.5);
  auto config = defaultParameters();
  config.push_back(rclcpp::Parameter("keep_a_part_of_previous_path", true));
  config.push_back(rclcpp::Parameter("dist_stop_to_previous_path", 0.1));
  ASSERT_TRUE(setParameters(config));

  // No obstacle and the path is same as the first one.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 0;
  publishMapAndRobot(1.65, 0.65, M_PI);
  sleepAndSpin(0.5);
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(initial_path, *path_));

  // The path does not change as a part of the previous path is kept and it is
  // not possible to keep distance from the obstacle.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 100;
  publishMapAndRobot(1.65, 0.65, M_PI);
  sleepAndSpin(0.5);
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(initial_path, *path_));

  // It is expected that the robot reaches the goal during the path planning.
  action_client_->async_cancel_all_goals();
  sleepAndSpin(0.5);
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 0;
  publishMapAndRobot(1.25, 0.95, M_PI / 2);
  sleepAndSpin(0.5);
  sendGoalAndWaitForPath();
  const nav_msgs::msg::Path short_path = *path_;
  // In the second path planning after cancel, the expected start pose is same
  // as the goal.
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(short_path, *path_));
}

TEST_F(DynamicParameterChangeTest, TriggerPlanByCostmapUpdate)
{
  publishMapAndRobot(2.55, 0.45, M_PI);
  sleepAndSpin(0.5);
  sendGoalAndWaitForPath();

  const rclcpp::Duration costmap_publishing_interval = rclcpp::Duration::from_seconds(0.1);
  // The path planning frequency is 4.0 Hz (Designated by the "freq" parameter)
  const double default_interval = getAveragePathInterval(costmap_publishing_interval);
  EXPECT_NEAR(default_interval, 1.0 / kDefaultFreq, (1.0 / kDefaultFreq) * 0.1);

  constexpr double kCostmapWatchdog = 0.5;
  auto config = defaultParameters();
  config.push_back(rclcpp::Parameter("trigger_plan_by_costmap_update", true));
  config.push_back(rclcpp::Parameter("costmap_watchdog", kCostmapWatchdog));
  ASSERT_TRUE(setParameters(config));

  // The path planning is triggered by the callback of CSpace3DUpdate, so its
  // frequency is same as the frequency of CSpace3DUpdate (10 Hz).
  const double interval_triggered_by_costmap = getAveragePathInterval(costmap_publishing_interval);
  EXPECT_NEAR(
    interval_triggered_by_costmap, costmap_publishing_interval.seconds(),
    costmap_publishing_interval.seconds() * 0.1);

  // The path planning is triggered by costmap_watchdog (0.5 seconds) when
  // CSpace3DUpdate is not published.
  const double interval_triggered_by_watchdog =
    getAveragePathInterval(rclcpp::Duration::from_seconds(100));
  EXPECT_NEAR(interval_triggered_by_watchdog, kCostmapWatchdog, kCostmapWatchdog * 0.1);
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
