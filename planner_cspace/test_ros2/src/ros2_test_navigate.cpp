/*
 * Copyright (c) 2018-2025, the neonavigation authors
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

// ROS 2 port of test/src/test_navigate.cpp (ROS 1 navigation_rostest.test),
// driven by test_navigate_launch.py. The launch description is instantiated
// several times with different arguments, mirroring the ROS 1 add_rostest
// matrix (antialias_start / fast_map_update / with_tolerance /
// enable_crowd_mode).
//
// The `~/make_plan` service is the one place where the ROS 2 interface cannot
// mirror ROS 1: a ROS 2 service cannot report a failure, so the node answers a
// failed planning request with an empty plan. Where ROS 1 asserted
// `!srv_plan.call(...)`, this test asserts that the returned plan is empty.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "planner_cspace/ros2_test_helpers.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_tracker_msgs/msg/path_with_velocity.hpp"

namespace
{
using planner_cspace_testing::latchedQos;
using planner_cspace_testing::spinUntil;

tf2::TimePoint toTf2Time(const rclcpp::Time & t)
{
  return tf2::TimePoint(std::chrono::nanoseconds(t.nanoseconds()));
}

std::string statusText(const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr & msg)
{
  if (!msg) {
    return "nullptr";
  }
  return "(status: " + std::to_string(msg->status) + ", error: " + std::to_string(msg->error) + ")";
}

class Navigate : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_local_;
  planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr planner_status_;
  costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr costmap_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  trajectory_tracker_msgs::msg::PathWithVelocity::ConstSharedPtr path_vel_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_local_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr sub_costmap_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<trajectory_tracker_msgs::msg::PathWithVelocity>::SharedPtr sub_path_vel_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_forget_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_local_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_patrol_nodes_;
  size_t local_map_apply_cnt_;
  std::vector<tf2::Stamped<tf2::Transform>> traj_;
  std::string test_scope_;
  bool enable_crowd_mode_;

  void SetUp() override
  {
    local_map_apply_cnt_ = 0;
    traj_.clear();
    map_ = nullptr;
    map_local_ = nullptr;
    planner_status_ = nullptr;
    costmap_ = nullptr;
    path_ = nullptr;
    path_vel_ = nullptr;

    node_ = rclcpp::Node::make_shared("test_navigate");
    enable_crowd_mode_ = node_->declare_parameter("enable_crowd_mode", false);
    tfbuf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

    test_scope_ = "[" + std::to_string(getpid()) + "/" +
                  ::testing::UnitTest::GetInstance()->current_test_info()->name() + "] ";

    sub_map_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map_global", latchedQos(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map_ = msg; });
    sub_map_local_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map_local", latchedQos(), [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        if (map_local_) {
          return;
        }
        map_local_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
      });
    sub_costmap_ = node_->create_subscription<costmap_cspace_msgs::msg::CSpace3D>(
      "/costmap", latchedQos(),
      [this](const costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr msg) { costmap_ = msg; });
    sub_status_ = node_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_3d/status", rclcpp::QoS(10).transient_local(),
      [this](const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr msg) {
        if (
          !planner_status_ || planner_status_->status != msg->status ||
          planner_status_->error != msg->error) {
          std::cerr << test_scope_ << " Status updated. " << statusText(msg) << std::endl;
        }
        planner_status_ = msg;
      });
    sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/path", latchedQos(),
      [this](const nav_msgs::msg::Path::ConstSharedPtr msg) { path_ = msg; });
    sub_path_vel_ = node_->create_subscription<trajectory_tracker_msgs::msg::PathWithVelocity>(
      "/path_velocity", latchedQos(),
      [this](const trajectory_tracker_msgs::msg::PathWithVelocity::ConstSharedPtr msg) {
        path_vel_ = msg;
      });
    srv_forget_ = node_->create_client<std_srvs::srv::Empty>("/forget_planning_cost");
    srv_plan_ = node_->create_client<nav_msgs::srv::GetPlan>("/planner_3d/make_plan");
    pub_map_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latchedQos());
    pub_map_local_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/overlay", latchedQos());
    pub_initial_pose_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", latchedQos());
    pub_patrol_nodes_ = node_->create_publisher<nav_msgs::msg::Path>("/patrol_nodes", latchedQos());

    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(60),
      [this] {
        return sub_map_->get_publisher_count() > 0 && sub_map_local_->get_publisher_count() > 0 &&
               sub_costmap_->get_publisher_count() > 0 && sub_status_->get_publisher_count() > 0 &&
               pub_map_->get_subscription_count() > 0 &&
               pub_map_local_->get_subscription_count() > 0 &&
               pub_initial_pose_->get_subscription_count() > 0 &&
               pub_patrol_nodes_->get_subscription_count() > 0;
      }))
      << test_scope_ << "Initialization timeout";
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30), [this] { return srv_forget_->service_is_ready(); }))
      << test_scope_ << "forget_planning_cost is not available";

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = 2.5;
    pose.pose.pose.position.y = 0.45;
    // yaw = pi. The ROS 2 message default of geometry_msgs/Quaternion is w = 1
    // (ROS 1 default constructed it to all zeros), so w is zeroed explicitly.
    pose.pose.pose.orientation.z = 1.0;
    pose.pose.pose.orientation.w = 0.0;
    pub_initial_pose_->publish(pose);

    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30),
      [this] {
        return tfbuf_->canTransform(
          "map", "base_link", toTf2Time(node_->now()), tf2::durationFromSec(0.5));
      }))
      << test_scope_ << "Initial transform timeout";

    // ROS 1 published the initial pose once and relied on the sleeps that
    // follow. Every test case here starts from wherever the previous one left
    // the robot, so the reset is waited for explicitly instead: without it a
    // test can start with the robot already sitting on its goal.
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(30),
      [this] {
        try {
          const auto trans = tfbuf_->lookupTransform("map", "base_link", tf2::TimePointZero);
          return std::hypot(
                   trans.transform.translation.x - 2.5, trans.transform.translation.y - 0.45) <
                 0.05;
        } catch (const tf2::TransformException &) {
          return false;
        }
      },
      [this, &pose] { pub_initial_pose_->publish(pose); }, std::chrono::milliseconds(200)))
      << test_scope_ << "dummy_robot didn't apply the initial pose";

    ASSERT_TRUE(
      spinUntil(node_, std::chrono::seconds(30), [this] { return static_cast<bool>(map_); }))
      << test_scope_ << "Initial map timeout";
    pub_map_->publish(*map_);
    std::cerr << test_scope_ << " Map applied." << std::endl;

    ASSERT_TRUE(
      spinUntil(node_, std::chrono::seconds(30), [this] { return static_cast<bool>(map_local_); }))
      << test_scope_ << "Initial local map timeout";

    ASSERT_TRUE(
      spinUntil(node_, std::chrono::seconds(30), [this] { return static_cast<bool>(costmap_); }))
      << test_scope_ << "Initial costmap timeout";

    auto future =
      srv_forget_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    spinUntil(node_, std::chrono::seconds(10), [&future] {
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });

    sleepAndSpin(1.0);
  }

  void TearDown() override
  {
    // Clear goal
    if (!node_) {
      return;
    }
    pub_patrol_nodes_->publish(nav_msgs::msg::Path());
    sleepAndSpin(2.0);
  }

  void sleepAndSpin(const double sec)
  {
    spinUntil(
      node_, std::chrono::nanoseconds(static_cast<int64_t>(sec * 1e9)), [] { return false; },
      nullptr, std::chrono::milliseconds(10));
  }

  void pubMapLocal()
  {
    if (!map_local_) {
      return;
    }
    pub_map_local_->publish(*map_local_);
    if ((local_map_apply_cnt_++) % 30 == 0) {
      int num_occupied = 0;
      for (const auto & c : map_local_->data) {
        if (c == 100) {
          num_occupied++;
        }
      }
      std::cerr << test_scope_ << " Local map applied. occupied grids:" << num_occupied
                << std::endl;
    }
  }

  tf2::Stamped<tf2::Transform> lookupRobotTrans(const rclcpp::Time & now)
  {
    const geometry_msgs::msg::TransformStamped trans_tmp =
      tfbuf_->lookupTransform("map", "base_link", toTf2Time(now), tf2::durationFromSec(0.5));
    tf2::Stamped<tf2::Transform> trans;
    tf2::fromMsg(trans_tmp, trans);
    traj_.push_back(trans);
    return trans;
  }

  void dumpRobotTrajectory()
  {
    std::cerr << test_scope_ << traj_.size() << " points recorded" << std::endl;
    for (const auto & t : traj_) {
      std::cerr << t.getOrigin().getX() << " " << t.getOrigin().getY() << " "
                << tf2::getYaw(t.getRotation()) << std::endl;
    }
  }

  // Checks that the map cells around the robot are not occupied.
  void assertNoCollision(const tf2::Stamped<tf2::Transform> & trans, const bool check_local)
  {
    for (int x = -2; x <= 2; ++x) {
      for (int y = -1; y <= 1; ++y) {
        const tf2::Vector3 pos =
          trans * tf2::Vector3(x * map_->info.resolution, y * map_->info.resolution, 0);
        const int map_x = pos.x() / map_->info.resolution;
        const int map_y = pos.y() / map_->info.resolution;
        const size_t addr = map_x + map_y * map_->info.width;
        ASSERT_LT(addr, map_->data.size());
        ASSERT_LT(map_x, static_cast<int>(map_->info.width));
        ASSERT_LT(map_y, static_cast<int>(map_->info.height));
        ASSERT_GE(map_x, 0);
        ASSERT_GE(map_y, 0);
        ASSERT_NE(map_->data[addr], 100);
        if (check_local) {
          ASSERT_NE(map_local_->data[addr], 100);
        }
      }
    }
  }

  void waitForPlannerStatus(const std::string & name, const int expected_error)
  {
    rclcpp::spin_some(node_);
    ASSERT_TRUE(static_cast<bool>(map_));
    ASSERT_TRUE(static_cast<bool>(map_local_));
    pubMapLocal();
    sleepAndSpin(0.2);

    const bool reached = spinUntil(
      node_, std::chrono::seconds(20),
      [this, expected_error] {
        return planner_status_ && planner_status_->error == expected_error;
      },
      [this] { pubMapLocal(); }, std::chrono::milliseconds(100));
    if (!reached) {
      dumpRobotTrajectory();
      FAIL() << test_scope_ << "/" << name << ": Navigation timeout." << std::endl
             << "status: " << statusText(planner_status_) << " (expected: " << expected_error
             << ")";
    }
  }

  // Publishes a patrol path and waits until planner_3d has actually started
  // planning for it.
  //
  // patrol only sends an action goal when it receives a new non-empty path,
  // and the result callback of the goal cancelled by the previous TearDown can
  // race with that, leaving patrol at the end of its (new) path without ever
  // sending a goal. actionlib hid this because its callbacks were delivered on
  // the same queue as the subscription; the path is therefore re-published
  // until the planner reports that it is working on it.
  void publishPatrolGoal(
    const nav_msgs::msg::Path & path, const std::function<void()> & on_spin = nullptr)
  {
    int cnt = 0;
    ASSERT_TRUE(spinUntil(
      node_, std::chrono::seconds(60),
      [this] {
        return planner_status_ &&
               planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DOING;
      },
      [this, &path, &cnt, &on_spin] {
        if (cnt % 30 == 0) {
          pub_patrol_nodes_->publish(path);
        }
        ++cnt;
        if (on_spin) {
          on_spin();
        }
      },
      std::chrono::milliseconds(100)))
      << test_scope_ << "planner_3d didn't start planning";
  }

  // Common body of the "drive to the goal" tests.
  void navigateTo(
    const nav_msgs::msg::Path & path, const double timeout_sec, const bool publish_local_map,
    const bool check_local_map, const bool require_done = false)
  {
    ASSERT_NO_FATAL_FAILURE(publishPatrolGoal(path, [this, publish_local_map] {
      if (publish_local_map) {
        pubMapLocal();
      }
    }));

    tf2::Transform goal;
    tf2::fromMsg(path.poses.back().pose, goal);

    const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_sec);
    rclcpp::WallRate wait(10.0);
    while (rclcpp::ok()) {
      if (publish_local_map) {
        pubMapLocal();
      }
      rclcpp::spin_some(node_);
      wait.sleep();

      const rclcpp::Time now = node_->now();
      if (now > deadline) {
        dumpRobotTrajectory();
        FAIL() << test_scope_ << "Navigation timeout." << std::endl
               << "status: " << statusText(planner_status_);
        return;
      }

      tf2::Stamped<tf2::Transform> trans;
      try {
        trans = lookupRobotTrans(now);
      } catch (const tf2::TransformException & e) {
        std::cerr << test_scope_ << e.what() << std::endl;
        continue;
      }

      const auto goal_rel = trans.inverse() * goal;
      const bool arrived =
        goal_rel.getOrigin().length() < 0.2 && std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2;
      if (
        arrived &&
        (!require_done || (planner_status_ && planner_status_->status ==
                                                planner_cspace_msgs::msg::PlannerStatus::DONE))) {
        std::cerr << test_scope_ << "Navigation success." << std::endl;
        return;
      }

      ASSERT_NO_FATAL_FAILURE(assertNoCollision(trans, check_local_map));
    }
    FAIL() << test_scope_ << "rclcpp is not ok";
  }

  nav_msgs::msg::Path makePath(const std::vector<std::array<double, 3>> & poses)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (const auto & p : poses) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), p[2]));
      path.poses.push_back(pose);
    }
    return path;
  }

  nav_msgs::srv::GetPlan::Response::SharedPtr callMakePlan(
    const nav_msgs::srv::GetPlan::Request::SharedPtr & req)
  {
    auto future = srv_plan_->async_send_request(req);
    if (!spinUntil(node_, std::chrono::seconds(20), [&future] {
          return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        })) {
      srv_plan_->remove_pending_request(future);
      return nullptr;
    }
    return future.get();
  }
};

TEST_F(Navigate, Navigate)
{
  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));

  navigateTo(makePath({{1.7, 2.8, -3.14}, {1.9, 2.8, -1.57}}), 90.0, false, false);
}

TEST_F(Navigate, NavigateWithLocalMap)
{
  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));
  pubMapLocal();
  sleepAndSpin(0.2);

  navigateTo(makePath({{1.7, 2.8, -3.14}}), 90.0, true, true);
}

TEST_F(Navigate, GlobalPlan)
{
  ASSERT_TRUE(
    spinUntil(node_, std::chrono::seconds(30), [this] { return srv_plan_->service_is_ready(); }));

  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));

  auto req = std::make_shared<nav_msgs::srv::GetPlan::Request>();
  req->tolerance = 0.0;
  req->start.header.frame_id = "map";
  req->start.pose.position.x = 1.95;
  req->start.pose.position.y = 0.45;
  req->start.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 3.14));

  req->goal.header.frame_id = "map";
  req->goal.pose.position.x = 1.25;
  req->goal.pose.position.y = 2.15;
  req->goal.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  // Planning fails as (12, 21, 0) is in rock. ROS 1 reported this by returning
  // false from the service; ROS 2 services cannot fail, so the node answers
  // with an empty plan.
  auto res = callMakePlan(req);
  ASSERT_TRUE(static_cast<bool>(res));
  ASSERT_TRUE(res->plan.poses.empty());

  // Goal grid is moved to (12, 22, 0).
  req->tolerance = 0.1;
  res = callMakePlan(req);
  ASSERT_TRUE(static_cast<bool>(res));
  ASSERT_FALSE(res->plan.poses.empty());
  EXPECT_NEAR(1.25, res->plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(2.25, res->plan.poses.back().pose.position.y, 1.0e-5);

  // Goal grid is moved to (12, 23, 0). This is because cost of (12, 22, 0) is
  // larger than 50.
  req->tolerance = 0.2f;
  res = callMakePlan(req);
  ASSERT_TRUE(static_cast<bool>(res));
  ASSERT_FALSE(res->plan.poses.empty());
  EXPECT_NEAR(1.25, res->plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(2.35, res->plan.poses.back().pose.position.y, 1.0e-5);

  req->tolerance = 0.0;
  req->goal.header.frame_id = "map";
  req->goal.pose.position.x = 1.85;
  req->goal.pose.position.y = 2.75;
  req->goal.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -1.57));
  res = callMakePlan(req);
  ASSERT_TRUE(static_cast<bool>(res));
  ASSERT_FALSE(res->plan.poses.empty());

  EXPECT_NEAR(req->start.pose.position.x, res->plan.poses.front().pose.position.x, 1.0e-5);
  EXPECT_NEAR(req->start.pose.position.y, res->plan.poses.front().pose.position.y, 1.0e-5);
  EXPECT_NEAR(req->goal.pose.position.x, res->plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(req->goal.pose.position.y, res->plan.poses.back().pose.position.y, 1.0e-5);

  for (const geometry_msgs::msg::PoseStamped & p : res->plan.poses) {
    const int map_x = p.pose.position.x / map_->info.resolution;
    const int map_y = p.pose.position.y / map_->info.resolution;
    const size_t addr = map_x + map_y * map_->info.width;
    ASSERT_LT(addr, map_->data.size());
    ASSERT_LT(map_x, static_cast<int>(map_->info.width));
    ASSERT_LT(map_y, static_cast<int>(map_->info.height));
    ASSERT_GE(map_x, 0);
    ASSERT_GE(map_y, 0);
    ASSERT_NE(map_->data[addr], 100);
  }
}

TEST_F(Navigate, RobotIsInRockOnSetGoal)
{
  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));
  pubMapLocal();
  sleepAndSpin(0.2);

  nav_msgs::msg::Path path = makePath({{1.19, 1.90, 0.0}});
  pub_patrol_nodes_->publish(path);

  const bool found = spinUntil(
    node_, std::chrono::seconds(20),
    [this] {
      return planner_status_ &&
             planner_status_->error == planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
    },
    [this] { pubMapLocal(); }, std::chrono::milliseconds(100));
  if (!found) {
    dumpRobotTrajectory();
    FAIL() << test_scope_ << "Navigation timeout. status: " << statusText(planner_status_);
  }
}

TEST_F(Navigate, GoalIsInRockRecovered)
{
  if (enable_crowd_mode_) {
    GTEST_SKIP() << "enable_crowd_mode is set";
  }

  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  for (int x = 10; x <= 16; ++x) {
    for (int y = 22; y <= 26; ++y) {
      map_local_->data[x + y * map_local_->info.width] = 100;
    }
  }
  pubMapLocal();

  pub_patrol_nodes_->publish(makePath({{1.25, 2.55, 0.0}}));

  waitForPlannerStatus("Got stuck", planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND);
  ASSERT_FALSE(::testing::Test::HasFailure());

  for (int x = 10; x <= 16; ++x) {
    for (int y = 22; y <= 26; ++y) {
      map_local_->data[x + y * map_local_->info.width] = 0;
    }
  }
  waitForPlannerStatus("Stuck recovered", planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);
}

TEST_F(Navigate, RobotIsInRockOnRecovered)
{
  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  for (int x = 21; x <= 28; ++x) {
    for (int y = 2; y <= 8; ++y) {
      map_local_->data[x + y * map_local_->info.width] = 100;
    }
  }
  pubMapLocal();

  pub_patrol_nodes_->publish(makePath({{1.25, 2.55, 0.0}}));

  waitForPlannerStatus("Got stuck", planner_cspace_msgs::msg::PlannerStatus::IN_ROCK);
  ASSERT_FALSE(::testing::Test::HasFailure());

  for (int x = 21; x <= 28; ++x) {
    for (int y = 2; y <= 8; ++y) {
      map_local_->data[x + y * map_local_->info.width] = 0;
    }
  }
  waitForPlannerStatus("Stuck recovered", planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);
}

TEST_F(Navigate, CrowdEscapeOnSurrounded)
{
  if (!enable_crowd_mode_) {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  const nav_msgs::msg::Path path = makePath({{1.6, 2.2, 1.57}});
  ASSERT_NO_FATAL_FAILURE(publishPatrolGoal(path, [this] { pubMapLocal(); }));

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::WallRate wait(10.0);
  const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(90);
  while (rclcpp::ok()) {
    pubMapLocal();
    rclcpp::spin_some(node_);
    wait.sleep();

    const rclcpp::Time now = node_->now();
    if (now > deadline) {
      dumpRobotTrajectory();
      FAIL() << test_scope_ << "Navigation timeout. status: " << statusText(planner_status_);
      return;
    }

    tf2::Stamped<tf2::Transform> trans;
    try {
      trans = lookupRobotTrans(now);
    } catch (const tf2::TransformException & e) {
      std::cerr << test_scope_ << e.what() << std::endl;
      continue;
    }

    ASSERT_TRUE(static_cast<bool>(planner_status_));
    EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);

    const auto goal_rel = trans.inverse() * goal;
    if (
      goal_rel.getOrigin().length() < 0.2 && std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2 &&
      planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DONE) {
      std::cerr << test_scope_ << "Navigation success." << std::endl;
      return;
    }

    const size_t rx = trans.getOrigin().x() / map_->info.resolution;
    const size_t ry = trans.getOrigin().y() / map_->info.resolution;
    const size_t data_size = map_local_->data.size();
    map_local_->data.clear();
    map_local_->data.resize(data_size, 0);
    const int bs = 6;
    for (int i = -bs; i <= bs; ++i) {
      map_local_->data[std::min((rx + i) + (ry - bs) * map_local_->info.width, data_size - 1)] =
        100;
      map_local_->data[std::min((rx + i) + (ry + bs) * map_local_->info.width, data_size - 1)] =
        100;
      map_local_->data[std::min((rx - bs) + (ry + i) * map_local_->info.width, data_size - 1)] =
        100;
      map_local_->data[std::min((rx + bs) + (ry + i) * map_local_->info.width, data_size - 1)] =
        100;
    }
  }
}

TEST_F(Navigate, CrowdEscapeOnPathNotFound)
{
  if (!enable_crowd_mode_) {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  ASSERT_NO_FATAL_FAILURE(
    publishPatrolGoal(makePath({{1.6, 2.2, 1.57}}), [this] { pubMapLocal(); }));

  rclcpp::WallRate wait(10.0);
  bool unreachable = false;
  const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(90);
  rclcpp::Time check_until = deadline;
  while (rclcpp::ok()) {
    for (size_t x = 0; x < map_local_->info.width; ++x) {
      const size_t y = 1.1 / map_->info.resolution;
      map_local_->data[x + y * map_local_->info.width] = 100;
    }
    pubMapLocal();

    rclcpp::spin_some(node_);
    wait.sleep();

    const rclcpp::Time now = node_->now();
    if (now > deadline) {
      dumpRobotTrajectory();
      FAIL() << test_scope_ << "Navigation timeout. status: " << statusText(planner_status_);
      return;
    }
    if (!planner_status_) {
      continue;
    }

    if (
      planner_status_->error == planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND &&
      !unreachable) {
      unreachable = true;
      // Check another 2 seconds that state is not changed
      check_until = now + rclcpp::Duration::from_seconds(2);
    }
    if (unreachable) {
      EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND);
    }
    if (now > check_until) {
      return;
    }
  }
}

TEST_F(Navigate, CrowdEscapeOnGoalIsInRock)
{
  if (!enable_crowd_mode_) {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  // (0, 0, 1, 0), i.e. yaw = pi. Unlike ROS 1, the ROS 2 message default of
  // geometry_msgs/Quaternion is w = 1, so w has to be zeroed explicitly: a
  // non-unit quaternion is interpreted by planner_3d as "clear the goal".
  nav_msgs::msg::Path path = makePath({{1.5, 0.45, 0.0}});
  path.poses[0].pose.orientation.x = 0.0;
  path.poses[0].pose.orientation.y = 0.0;
  path.poses[0].pose.orientation.z = 1.0;
  path.poses[0].pose.orientation.w = 0.0;
  // The goal has to be reachable when it is set; the cells around it are only
  // blocked by the loop below.
  pubMapLocal();
  ASSERT_NO_FATAL_FAILURE(publishPatrolGoal(path, [this] { pubMapLocal(); }));

  rclcpp::WallRate wait(10.0);
  bool unreachable = false;
  const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(90);
  rclcpp::Time check_until = deadline;
  while (rclcpp::ok()) {
    const int gx = path.poses[0].pose.position.x / map_->info.resolution;
    const int gy = path.poses[0].pose.position.y / map_->info.resolution;
    for (int x = gx - 2; x <= gx + 2; ++x) {
      for (int y = gy - 2; y <= gy + 2; ++y) {
        map_local_->data[x + y * map_local_->info.width] = 100;
      }
    }
    pubMapLocal();

    rclcpp::spin_some(node_);
    wait.sleep();

    const rclcpp::Time now = node_->now();
    if (now > deadline) {
      dumpRobotTrajectory();
      FAIL() << test_scope_ << "Navigation timeout. status: " << statusText(planner_status_);
      return;
    }
    if (!planner_status_) {
      continue;
    }

    if (
      planner_status_->error == planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND &&
      !unreachable) {
      unreachable = true;
      // Check another 2 seconds that state is not changed
      check_until = now + rclcpp::Duration::from_seconds(2);
    }
    if (unreachable) {
      EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND);
    }
    if (now > check_until) {
      return;
    }
  }
}

TEST_F(Navigate, CrowdEscapeButNoValidTemporaryGoal)
{
  if (!enable_crowd_mode_) {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  // (0, 0, 1, 0), i.e. yaw = pi. Unlike ROS 1, the ROS 2 message default of
  // geometry_msgs/Quaternion is w = 1, so w has to be zeroed explicitly: a
  // non-unit quaternion is interpreted by planner_3d as "clear the goal".
  nav_msgs::msg::Path path = makePath({{1.5, 0.45, 0.0}});
  path.poses[0].pose.orientation.x = 0.0;
  path.poses[0].pose.orientation.y = 0.0;
  path.poses[0].pose.orientation.z = 1.0;
  path.poses[0].pose.orientation.w = 0.0;
  // The goal has to be reachable when it is set; the loop below floods the
  // whole map with cost 60 and blocks the goal.
  pubMapLocal();
  ASSERT_NO_FATAL_FAILURE(publishPatrolGoal(path, [this] { pubMapLocal(); }));

  rclcpp::WallRate wait(10.0);
  const rclcpp::Time check_until = node_->now() + rclcpp::Duration::from_seconds(5);
  int cnt_planning = 0;
  while (rclcpp::ok()) {
    const int gx = path.poses[0].pose.position.x / map_->info.resolution;
    const int gy = path.poses[0].pose.position.y / map_->info.resolution;
    map_local_->data.clear();
    map_local_->data.resize(map_local_->info.width * map_local_->info.height, 60);
    for (int x = gx - 2; x <= gx + 2; ++x) {
      for (int y = gy - 2; y <= gy + 2; ++y) {
        map_local_->data[x + y * map_local_->info.width] = 100;
      }
    }
    pubMapLocal();

    rclcpp::spin_some(node_);
    wait.sleep();

    if (
      planner_status_ &&
      planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DOING) {
      if (cnt_planning > 1) {
        // Temporary goal is selected from cells with cost<50 by default.
        // No temporary goal should be selected on this map.
        EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND);
      }
      cnt_planning++;
    }
    if (node_->now() > check_until) {
      EXPECT_GT(cnt_planning, 2);
      return;
    }
  }
}

TEST_F(Navigate, ForceTemporaryEscape)
{
  if (!enable_crowd_mode_) {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  auto pub_trigger =
    node_->create_publisher<std_msgs::msg::Empty>("/planner_3d/temporary_escape", rclcpp::QoS(1));

  rclcpp::spin_some(node_);
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  const nav_msgs::msg::Path path = makePath({{1.6, 2.2, 1.57}});
  ASSERT_NO_FATAL_FAILURE(publishPatrolGoal(path, [this] { pubMapLocal(); }));

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  rclcpp::WallRate wait(2.0);
  const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(90);
  while (rclcpp::ok()) {
    const size_t data_size = map_local_->data.size();
    map_local_->data.clear();
    map_local_->data.resize(data_size, 0);
    pubMapLocal();

    pub_trigger->publish(std_msgs::msg::Empty());

    rclcpp::spin_some(node_);
    wait.sleep();

    const rclcpp::Time now = node_->now();
    if (now > deadline) {
      dumpRobotTrajectory();
      FAIL() << test_scope_ << "Navigation timeout. status: " << statusText(planner_status_);
      return;
    }

    tf2::Stamped<tf2::Transform> trans;
    try {
      trans = lookupRobotTrans(now);
    } catch (const tf2::TransformException & e) {
      std::cerr << test_scope_ << e.what() << std::endl;
      continue;
    }

    ASSERT_TRUE(static_cast<bool>(planner_status_));
    EXPECT_EQ(planner_status_->error, planner_cspace_msgs::msg::PlannerStatus::GOING_WELL);

    const auto goal_rel = trans.inverse() * goal;
    if (
      goal_rel.getOrigin().length() < 0.2 && std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2 &&
      planner_status_->status == planner_cspace_msgs::msg::PlannerStatus::DONE) {
      std::cerr << test_scope_ << "Navigation success." << std::endl;
      return;
    }
  }
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
