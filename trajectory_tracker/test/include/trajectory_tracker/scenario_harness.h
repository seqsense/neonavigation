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

#ifndef TRAJECTORY_TRACKER_SCENARIO_HARNESS_H
#define TRAJECTORY_TRACKER_SCENARIO_HARNESS_H

// In-process harness for the trajectory_tracker scenarios.
//
// The ROS 1 rostests and the ROS 2 launch tests drive the tracker as a running
// node: a node under test, a test node that integrates cmd_vel into a pose and
// republishes TF, and (on ROS 2) a process publishing an accelerated /clock.
// That makes every scenario depend on process scheduling and on DDS delivery,
// which is what made them slow and occasionally flaky.
//
// TrackerController takes the robot transform and dt as arguments and returns
// the command and status, so the same scenarios can run in one process against
// a virtual clock: no DDS, no /clock, no TF listener, and no wall-clock waits.
// The kinematic model and the integration order are copied from the node tests
// so that the numbers stay comparable.

#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "trajectory_tracker/tracker_controller.h"
#include "trajectory_tracker_msgs/msg/path_with_velocity.hpp"
#include "trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp"

namespace trajectory_tracker_testing
{

// The ROS 1 build reaches rclcpp through sq_ros1_rclcpp_compat, whose header
// only exists there; tf2_ros::Buffer takes no clock on that side.
#if __has_include(<sq_ros1_compat/logger.hpp>)
#define TRAJECTORY_TRACKER_TEST_ROS1 1
#endif

inline std::unique_ptr<tf2_ros::Buffer> makeBuffer()
{
#ifdef TRAJECTORY_TRACKER_TEST_ROS1
  return std::make_unique<tf2_ros::Buffer>();
#else
  return std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
#endif
}

// The controller timestamps its transform-age check against the process clock,
// which on ROS 1 has to be initialized explicitly when there is no node around.
// sq_ros1_rclcpp_compat's gtest main already does it; calling it again is cheap.
inline void initTimeSource()
{
#ifdef TRAJECTORY_TRACKER_TEST_ROS1
  ros::Time::init();
#endif
}

// Path builders matching the node tests' publishPath/publishPathVelocity: xy+yaw
// triples, optionally with a per-pose velocity.
inline nav_msgs::msg::Path makePath(const std::vector<Eigen::Vector3d> & poses)
{
  nav_msgs::msg::Path path;
  for (const Eigen::Vector3d & p : poses) {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    path.poses.push_back(pose);
  }
  return path;
}

inline trajectory_tracker_msgs::msg::PathWithVelocity makePathWithVelocity(
  const std::vector<Eigen::Vector4d> & poses)
{
  trajectory_tracker_msgs::msg::PathWithVelocity path;
  for (const Eigen::Vector4d & p : poses) {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));
    trajectory_tracker_msgs::msg::PoseStampedWithVelocity pose;
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose.linear_velocity.x = p[3];
    path.poses.push_back(pose);
  }
  return path;
}

class Scenario
{
public:
  struct Options
  {
    // Control period, as the node's "hz" parameter.
    double hz = 50.0;
    // Fed to TrackerController::control() as the odometry twist, mirroring the
    // node's odometry path. False means the timer path (no odometry).
    bool use_odom = false;
    // Delay of that odometry feedback, as the node tests' "odom_delay".
    double odom_delay = 0.0;
    // Mirrors the node test's cap on the integration step.
    double dt_cap = 0.1;
    std::string frame_odom = "odom";
    std::string frame_robot = "base_link";
    // The scenarios publish their paths in this frame; the harness keeps a
    // static identity transform to frame_odom, as the node tests effectively do.
    std::string frame_path = "map";
  };

  struct State
  {
    Eigen::Vector2d pos{0.0, 0.0};
    double yaw = 0.0;
    geometry_msgs::msg::Twist cmd_vel;
    trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
    // Simulated seconds since the scenario started.
    double time = 0.0;
    int steps = 0;
  };

  // The single-argument form is defined after the class: a default argument
  // cannot name a nested class whose member initializers are not complete yet.
  explicit Scenario(const trajectory_tracker::TrackerController::Parameters & params);

  Scenario(const trajectory_tracker::TrackerController::Parameters & params, Options options)
  : options_(std::move(options)), buffer_(makeBuffer())
  {
    initTimeSource();
    controller_ = std::make_unique<trajectory_tracker::TrackerController>(
      *buffer_, rclcpp::get_logger("trajectory_tracker_scenario"));
    controller_->setFrames(options_.frame_robot, options_.frame_odom);
    controller_->setParameters(params);
    publishStaticPathTransform();
  }

  void initState(const Eigen::Vector2d & pos, const double yaw)
  {
    state_ = State();
    state_.pos = pos;
    state_.yaw = yaw;
  }

  template <typename MSG_TYPE>
  void setPath(MSG_TYPE path)
  {
    path.header.frame_id = options_.frame_path;
    path.header.stamp = stamp(state_.time);
    for (auto & pose : path.poses) {
      pose.header = path.header;
    }
    controller_->setPath(path);
  }

  void setSpeed(const double speed) { controller_->setSpeed(speed); }

  // Reports a velocity the robot does not actually have, which is how the
  // overshoot scenarios check the goal velocity tolerances.
  void setReportedVelocity(const double linear, const double angular)
  {
    reported_vel_ = std::make_pair(linear, angular);
  }

  // Keeps the robot where it is while still running control cycles.
  void freezePose(const bool frozen) { pose_frozen_ = frozen; }

  // One control cycle: command the controller, then move the robot with it.
  const State & step()
  {
    const double dt = std::min(1.0 / options_.hz, options_.dt_cap);

    tf2::Stamped<tf2::Transform> odom_to_robot;
    odom_to_robot.frame_id_ = options_.frame_odom;
    odom_to_robot.stamp_ =
      tf2::TimePoint(std::chrono::nanoseconds(stamp(state_.time).nanoseconds()));
    odom_to_robot.setData(
      tf2::Transform(
        tf2::Quaternion(tf2::Vector3(0, 0, 1), state_.yaw),
        tf2::Vector3(state_.pos.x(), state_.pos.y(), 0.0)));

    double odom_lin = 0.0;
    double odom_ang = 0.0;
    if (reported_vel_) {
      odom_lin = reported_vel_->first;
      odom_ang = reported_vel_->second;
    } else if (options_.use_odom) {
      // The node feeds back measured odometry, which lags the command.
      const size_t delay_steps =
        static_cast<size_t>(std::lround(options_.odom_delay * options_.hz));
      cmd_history_.push_back(state_.cmd_vel);
      if (cmd_history_.size() > delay_steps + 1) {
        cmd_history_.erase(cmd_history_.begin());
      }
      odom_lin = cmd_history_.front().linear.x;
      odom_ang = cmd_history_.front().angular.z;
    }
    const auto out =
      controller_->control(odom_to_robot, Eigen::Vector3d(0, 0, 0), odom_lin, odom_ang, dt);

    if (!pose_frozen_) {
      // Integration order copied from the node tests' cmd_vel callback.
      state_.yaw += out.cmd_vel.angular.z * dt;
      state_.pos +=
        Eigen::Vector2d(std::cos(state_.yaw), std::sin(state_.yaw)) * out.cmd_vel.linear.x * dt;
    }
    state_.cmd_vel = out.cmd_vel;
    state_.status = out.status;
    state_.time += dt;
    ++state_.steps;
    return state_;
  }

  // Steps until `stop` returns true or the simulated budget is spent. Every
  // step is offered to `watch`, which is where a scenario asserts that nothing
  // went wrong on the way (overshoot, wrong direction, ...).
  bool runUntil(
    const double budget_sec, const std::function<bool(const State &)> & stop,
    const std::function<void(const State &)> & watch = nullptr)
  {
    const double deadline = state_.time + budget_sec;
    while (state_.time <= deadline) {
      step();
      if (watch) {
        watch(state_);
      }
      if (stop(state_)) {
        return true;
      }
    }
    return false;
  }

  bool runUntilGoal(
    const double budget_sec, const std::function<void(const State &)> & watch = nullptr)
  {
    return runUntil(
      budget_sec,
      [](const State & s) {
        return s.status.status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL;
      },
      watch);
  }

  // Keeps stepping after the goal, which is how the node tests looked for a
  // late overshoot.
  void settle(const int steps)
  {
    for (int i = 0; i < steps; ++i) {
      step();
    }
  }

  const State & state() const { return state_; }
  double distanceRemains() const { return state_.status.distance_remains; }
  double angleRemains() const { return state_.status.angle_remains; }

private:
  rclcpp::Time stamp(const double t) const
  {
    return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
  }

  void publishStaticPathTransform()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = options_.frame_path;
    tf.child_frame_id = options_.frame_odom;
    tf.transform.rotation.w = 1.0;
    buffer_->setTransform(tf, "scenario_harness", true);
  }

  Options options_;
  bool pose_frozen_ = false;
  std::optional<std::pair<double, double>> reported_vel_;
  std::vector<geometry_msgs::msg::Twist> cmd_history_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<trajectory_tracker::TrackerController> controller_;
  State state_;
};

inline Scenario::Scenario(const trajectory_tracker::TrackerController::Parameters & params)
: Scenario(params, Options())
{
}

}  // namespace trajectory_tracker_testing

#endif  // TRAJECTORY_TRACKER_SCENARIO_HARNESS_H
