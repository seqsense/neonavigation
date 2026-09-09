/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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

#ifndef TRAJECTORY_TRACKER__TRACKER_CONTROLLER_H_
#define TRAJECTORY_TRACKER__TRACKER_CONTROLLER_H_

#include <cmath>
#include <limits>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "trajectory_tracker/basic_control.h"
#include "trajectory_tracker/eigen_line.h"
#include "trajectory_tracker/path2d.h"
#include "trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp"

namespace trajectory_tracker
{
// TrackerController holds the pure trajectory-following control logic.
// It has no knowledge of ROS nodes, publishers, subscribers, parameters or
// dynamic_reconfigure; those belong to the interface layer (TrackerNode).
// Message types, TF, time and logging use the ROS 2 (rclcpp) surface, which on
// ROS 1 is provided by sq_ros1_rclcpp_compat so the same source builds on both.
class TrackerController
{
public:
  // Tunable parameters. The interface layer fills this from
  // dynamic_reconfigure and pushes it via setParameters().
  struct Parameters
  {
    double look_forward = 0.0;
    double curv_forward = 0.0;
    double k[3] = {0.0, 0.0, 0.0};
    double gain_at_vel = 0.0;
    double d_lim = 0.0;
    double d_stop = 0.0;
    double rotate_ang = 0.0;
    double vel[2] = {0.0, 0.0};
    double acc[2] = {0.0, 0.0};
    double acc_toc[2] = {0.0, 0.0};
    int path_step = 1;
    double goal_tolerance_dist = 0.0;
    double goal_tolerance_ang = 0.0;
    double stop_tolerance_dist = 0.0;
    double stop_tolerance_ang = 0.0;
    double no_pos_cntl_dist = 0.0;
    double min_track_path = 0.0;
    bool allow_backward = false;
    bool limit_vel_by_avel = false;
    bool check_old_path = false;
    double epsilon = 1.0e-6;
    bool use_time_optimal_control = false;
    double time_optimal_control_future_gain = 0.0;
    double k_ang_rotation = 0.0;
    double k_avel_rotation = 0.0;
    double goal_tolerance_lin_vel = 0.0;
    double goal_tolerance_ang_vel = 0.0;
  };

  // Result of a single control step. The interface layer publishes both.
  struct ControlOutput
  {
    geometry_msgs::msg::Twist cmd_vel;
    trajectory_tracker_msgs::msg::TrajectoryTrackerStatus status;
  };

  TrackerController(tf2_ros::Buffer & tfbuf, const rclcpp::Logger & logger);

  void setParameters(const Parameters & params);

  void setFrames(const std::string & frame_robot, const std::string & frame_odom);
  const std::string & frameRobot() const { return frame_robot_; }
  const std::string & frameOdom() const { return frame_odom_; }
  void setFrameRobot(const std::string & frame_robot) { frame_robot_ = frame_robot; }
  void setFrameOdom(const std::string & frame_odom) { frame_odom_ = frame_odom; }

  const std_msgs::msg::Header & pathHeader() const { return path_header_; }

  // Update the internal path from a nav_msgs::msg::Path or
  // trajectory_tracker_msgs::msg::PathWithVelocity message.
  template <typename MSG_TYPE>
  void setPath(const MSG_TYPE & msg)
  {
    path_header_ = msg.header;
    is_path_updated_ = true;
    path_step_done_ = 0;
    path_.fromMsg(msg, epsilon_);
    for (const auto & path_pose : path_) {
      if (std::isfinite(path_pose.velocity_) && path_pose.velocity_ < -0.0) {
        rclcpp::Clock clock(RCL_ROS_TIME);
        RCLCPP_ERROR_THROTTLE(logger_, clock, 1000, "path_velocity.velocity.x must be positive");
        path_.clear();
        return;
      }
    }
  }

  // Override the reference linear velocity (speed topic).
  void setSpeed(const double speed) { vel_[0] = speed; }

  // Reset the velocity/acceleration limiters.
  void resetLimiters()
  {
    v_lim_.clear();
    w_lim_.clear();
  }

  // Run one control step and return the command velocity and status.
  ControlOutput control(
    const tf2::Stamped<tf2::Transform> & odom_to_robot, const Eigen::Vector3d & prediction_offset,
    const double odom_linear_vel, const double odom_angular_vel, const double dt);

private:
  std::string frame_robot_;
  std::string frame_odom_;
  double look_forward_;
  double curv_forward_;
  double k_[3];
  double gain_at_vel_;
  double d_lim_;
  double d_stop_;
  double vel_[2];
  double acc_[2];
  double acc_toc_[2];
  trajectory_tracker::VelAccLimitter v_lim_;
  trajectory_tracker::VelAccLimitter w_lim_;
  double rotate_ang_;
  double goal_tolerance_dist_;
  double goal_tolerance_ang_;
  double stop_tolerance_dist_;
  double stop_tolerance_ang_;
  double no_pos_cntl_dist_;
  double min_track_path_;
  int path_step_;
  int path_step_done_;
  bool allow_backward_;
  bool limit_vel_by_avel_;
  bool check_old_path_;
  double epsilon_;
  bool use_time_optimal_control_;
  double time_optimal_control_future_gain_;
  double k_ang_rotation_;
  double k_avel_rotation_;
  double goal_tolerance_lin_vel_;
  double goal_tolerance_ang_vel_;

  tf2_ros::Buffer & tfbuf_;
  rclcpp::Logger logger_;

  trajectory_tracker::Path2D path_;
  std_msgs::msg::Header path_header_;
  bool is_path_updated_;

  struct TrackingResult
  {
    explicit TrackingResult(const int s)
    : status(s),
      distance_remains(0.0),
      angle_remains(0.0),
      distance_remains_raw(0.0),
      angle_remains_raw(0.0),
      turning_in_place(false),
      signed_local_distance(0.0),
      distance_from_target(0.0),
      target_linear_vel(0.0),
      tracking_point_x(0.0),
      tracking_point_y(0.0),
      tracking_point_curv(0.0),
      path_step_done(0)
    {
    }

    int status;  // same as trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::status
    double distance_remains;
    double angle_remains;
    double distance_remains_raw;  // remained distance without prediction
    double angle_remains_raw;
    bool turning_in_place;
    double signed_local_distance;
    double distance_from_target;
    double target_linear_vel;
    double tracking_point_x;
    double tracking_point_y;
    double tracking_point_curv;
    int path_step_done;
  };

  TrackingResult getTrackingResult(
    const tf2::Stamped<tf2::Transform> &, const Eigen::Vector3d &, const double,
    const double) const;
};
}  // namespace trajectory_tracker

#endif  // TRAJECTORY_TRACKER__TRACKER_CONTROLLER_H_
