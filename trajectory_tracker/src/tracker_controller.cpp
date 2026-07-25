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

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
   Original idea of the implemented control scheme was published on:
   S. Iida, S. Yuta, "Vehicle command system and trajectory control for
   autonomous mobile robots," in Proceedings of the 1991 IEEE/RSJ
   International Workshop on Intelligent Robots and Systems (IROS),
   1991, pp. 212-217.
 */

#include "trajectory_tracker/tracker_controller.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "trajectory_tracker/basic_control.h"
#include "trajectory_tracker/eigen_line.h"
#include "trajectory_tracker/path2d.h"
#include "trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp"

namespace trajectory_tracker
{
TrackerController::TrackerController(tf2_ros::Buffer & tfbuf, const rclcpp::Logger & logger)
: look_forward_(0.0),
  curv_forward_(0.0),
  k_{0.0, 0.0, 0.0},
  gain_at_vel_(0.0),
  d_lim_(0.0),
  d_stop_(0.0),
  vel_{0.0, 0.0},
  acc_{0.0, 0.0},
  acc_toc_{0.0, 0.0},
  rotate_ang_(0.0),
  goal_tolerance_dist_(0.0),
  goal_tolerance_ang_(0.0),
  stop_tolerance_dist_(0.0),
  stop_tolerance_ang_(0.0),
  no_pos_cntl_dist_(0.0),
  min_track_path_(0.0),
  path_step_(1),
  path_step_done_(0),
  allow_backward_(false),
  limit_vel_by_avel_(false),
  check_old_path_(false),
  epsilon_(1.0e-6),
  use_time_optimal_control_(false),
  time_optimal_control_future_gain_(0.0),
  k_ang_rotation_(0.0),
  k_avel_rotation_(0.0),
  goal_tolerance_lin_vel_(0.0),
  goal_tolerance_ang_vel_(0.0),
  tfbuf_(tfbuf),
  logger_(logger),
  is_path_updated_(false)
{
}

void TrackerController::setFrames(const std::string & frame_robot, const std::string & frame_odom)
{
  frame_robot_ = frame_robot;
  frame_odom_ = frame_odom;
}

void TrackerController::setParameters(const Parameters & params)
{
  look_forward_ = params.look_forward;
  curv_forward_ = params.curv_forward;
  k_[0] = params.k[0];
  k_[1] = params.k[1];
  k_[2] = params.k[2];
  gain_at_vel_ = params.gain_at_vel;
  d_lim_ = params.d_lim;
  d_stop_ = params.d_stop;
  rotate_ang_ = params.rotate_ang;
  vel_[0] = params.vel[0];
  vel_[1] = params.vel[1];
  acc_[0] = params.acc[0];
  acc_[1] = params.acc[1];
  acc_toc_[0] = params.acc_toc[0];
  acc_toc_[1] = params.acc_toc[1];
  path_step_ = params.path_step;
  goal_tolerance_dist_ = params.goal_tolerance_dist;
  goal_tolerance_ang_ = params.goal_tolerance_ang;
  stop_tolerance_dist_ = params.stop_tolerance_dist;
  stop_tolerance_ang_ = params.stop_tolerance_ang;
  no_pos_cntl_dist_ = params.no_pos_cntl_dist;
  min_track_path_ = params.min_track_path;
  allow_backward_ = params.allow_backward;
  limit_vel_by_avel_ = params.limit_vel_by_avel;
  check_old_path_ = params.check_old_path;
  epsilon_ = params.epsilon;
  use_time_optimal_control_ = params.use_time_optimal_control;
  time_optimal_control_future_gain_ = params.time_optimal_control_future_gain;
  k_ang_rotation_ = params.k_ang_rotation;
  k_avel_rotation_ = params.k_avel_rotation;
  goal_tolerance_lin_vel_ = params.goal_tolerance_lin_vel;
  goal_tolerance_ang_vel_ = params.goal_tolerance_ang_vel;
}

TrackerController::ControlOutput TrackerController::control(
  const tf2::Stamped<tf2::Transform> & odom_to_robot, const Eigen::Vector3d & prediction_offset,
  const double odom_linear_vel, const double odom_angular_vel, const double dt)
{
  ControlOutput output;
  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus & status = output.status;
  geometry_msgs::msg::Twist & cmd_vel = output.cmd_vel;
  status.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  status.path_header = path_header_;
  if (is_path_updated_) {
    // Call getTrackingResult to update path_step_done_.
    const TrackingResult initial_tracking_result =
      getTrackingResult(odom_to_robot, prediction_offset, odom_linear_vel, odom_angular_vel);
    path_step_done_ = initial_tracking_result.path_step_done;
    is_path_updated_ = false;
  }
  const TrackingResult tracking_result =
    getTrackingResult(odom_to_robot, prediction_offset, odom_linear_vel, odom_angular_vel);
  switch (tracking_result.status) {
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH:
    case trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH: {
      v_lim_.clear();
      w_lim_.clear();
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      break;
    }
    default: {
      if (tracking_result.turning_in_place) {
        v_lim_.set(0.0, tracking_result.target_linear_vel, acc_[0], dt);

        if (use_time_optimal_control_) {
          const double expected_angle_remains =
            tracking_result.angle_remains + w_lim_.get() * dt * time_optimal_control_future_gain_;
          w_lim_.set(
            trajectory_tracker::timeOptimalControl(expected_angle_remains, acc_toc_[1]), vel_[1],
            acc_[1], dt);
        } else {
          const double wvel_increment =
            (-tracking_result.angle_remains * k_ang_rotation_ - w_lim_.get() * k_avel_rotation_) *
            dt;
          w_lim_.increment(wvel_increment, vel_[1], acc_[1], dt);
        }
        RCLCPP_DEBUG(
          logger_, "trajectory_tracker: angular residual %0.3f, angular vel %0.3f",
          tracking_result.angle_remains, w_lim_.get());
      } else {
        v_lim_.set(
          trajectory_tracker::timeOptimalControl(
            tracking_result.signed_local_distance, acc_toc_[0]),
          tracking_result.target_linear_vel, acc_[0], dt);

        float wref = std::abs(v_lim_.get()) * tracking_result.tracking_point_curv;

        if (limit_vel_by_avel_ && std::abs(wref) > vel_[1]) {
          v_lim_.set(
            std::copysign(1.0, v_lim_.get()) *
              std::abs(vel_[1] / tracking_result.tracking_point_curv),
            tracking_result.target_linear_vel, acc_[0], dt);
          wref = std::copysign(1.0, wref) * vel_[1];
        }

        const double k_ang = (gain_at_vel_ == 0.0)
                               ? (k_[1])
                               : (k_[1] * tracking_result.target_linear_vel / gain_at_vel_);
        const double dist_diff = tracking_result.distance_from_target;
        const double angle_diff = tracking_result.angle_remains;
        const double wvel_diff = w_lim_.get() - wref;
        w_lim_.increment(
          dt * (-dist_diff * k_[0] - angle_diff * k_ang - wvel_diff * k_[2]), vel_[1], acc_[1], dt);

        RCLCPP_DEBUG(
          logger_,
          "trajectory_tracker: distance residual %0.3f, angular residual %0.3f, ang vel residual "
          "%0.3f"
          ", v_lim %0.3f, w_lim %0.3f signed_local_distance %0.3f, k_ang %0.3f",
          dist_diff, angle_diff, wvel_diff, v_lim_.get(), w_lim_.get(),
          tracking_result.signed_local_distance, k_ang);
      }
      if (
        std::abs(tracking_result.distance_remains) < stop_tolerance_dist_ &&
        std::abs(tracking_result.angle_remains) < stop_tolerance_ang_ &&
        std::abs(tracking_result.distance_remains_raw) < stop_tolerance_dist_ &&
        std::abs(tracking_result.angle_remains_raw) < stop_tolerance_ang_) {
        v_lim_.clear();
        w_lim_.clear();
      }
      cmd_vel.linear.x = v_lim_.get();
      cmd_vel.angular.z = w_lim_.get();
      path_step_done_ = tracking_result.path_step_done;
      break;
    }
  }
  status.status = tracking_result.status;
  status.distance_remains = tracking_result.distance_remains;
  status.angle_remains = tracking_result.angle_remains;
  return output;
}

TrackerController::TrackingResult TrackerController::getTrackingResult(
  const tf2::Stamped<tf2::Transform> & odom_to_robot, const Eigen::Vector3d & prediction_offset,
  const double odom_linear_vel, const double odom_angular_vel) const
{
  if (path_header_.frame_id.size() == 0 || path_.size() == 0) {
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }
  // Transform
  trajectory_tracker::Path2D lpath;
  double transform_delay = 0;
  try {
    tf2::Stamped<tf2::Transform> path_to_odom;
    const auto path_to_odom_msg =
      tfbuf_.lookupTransform(path_header_.frame_id, frame_odom_, rclcpp::Time(0, 0, RCL_ROS_TIME));
    tf2::fromMsg(path_to_odom_msg, path_to_odom);
    const rclcpp::Time path_to_odom_stamp(path_to_odom_msg.header.stamp);
    const tf2::Transform path_to_robot = path_to_odom * odom_to_robot;
    transform_delay = (rclcpp::Clock(RCL_ROS_TIME).now() - path_to_odom_stamp).seconds();
    if (std::abs(transform_delay) > 0.1 && check_old_path_) {
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_ERROR_THROTTLE(
        logger_, clock, 1000, "Timestamp of the transform is too old %f %f",
        rclcpp::Clock(RCL_ROS_TIME).now().seconds(), path_to_odom_stamp.seconds());
    }
    const float robot_yaw = tf2::getYaw(path_to_robot.getRotation());
    const Eigen::Transform<double, 2, Eigen::TransformTraits::AffineCompact> path_to_robot_2d =
      Eigen::Translation2d(
        Eigen::Vector2d(path_to_robot.getOrigin().x(), path_to_robot.getOrigin().y())) *
      Eigen::Rotation2Dd(robot_yaw);
    const auto robot_to_path_2d = path_to_robot_2d.inverse();

    for (size_t i = 0; i < path_.size(); i += path_step_)
      lpath.push_back(trajectory_tracker::Pose2D(
        robot_to_path_2d * path_[i].pos_, -robot_yaw + path_[i].yaw_, path_[i].velocity_));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(logger_, "TF exception: %s", e.what());
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }

  const Eigen::Vector2d origin_raw = prediction_offset.head<2>();
  const float yaw_raw = prediction_offset[2];

  const float yaw_predicted = w_lim_.get() * look_forward_ / 2;
  const Eigen::Vector2d origin = Eigen::Vector2d(std::cos(yaw_predicted), std::sin(yaw_predicted)) *
                                 v_lim_.get() * look_forward_;

  const double path_length = lpath.length();

  // Find nearest line strip
  const trajectory_tracker::Path2D::ConstIterator it_local_goal =
    lpath.findLocalGoal(lpath.cbegin() + path_step_done_, lpath.cend(), allow_backward_);

  const float max_search_range = (path_step_done_ > 0) ? 1.0 : 0.0;
  const trajectory_tracker::Path2D::ConstIterator it_nearest = lpath.findNearest(
    lpath.cbegin() + path_step_done_, it_local_goal, origin, max_search_range, epsilon_);

  if (it_nearest == lpath.end()) {
    return TrackingResult(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH);
  }

  const int i_nearest = std::distance(lpath.cbegin(), it_nearest);
  const int i_nearest_prev = std::max(0, i_nearest - 1);
  const int i_local_goal = std::distance(lpath.cbegin(), it_local_goal);

  const Eigen::Vector2d pos_on_line =
    trajectory_tracker::projection2d(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin);
  const Eigen::Vector2d pos_on_line_raw =
    trajectory_tracker::projection2d(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin_raw);

  const float linear_vel =
    std::isnan(lpath[i_nearest].velocity_) ? vel_[0] : lpath[i_nearest].velocity_;

  // Remained distance to the local goal
  float remain_local =
    lpath.remainedDistance(lpath.cbegin(), it_nearest, it_local_goal, pos_on_line);
  // Remained distance to the final goal
  float distance_remains =
    lpath.remainedDistance(lpath.cbegin(), it_nearest, lpath.cend(), pos_on_line);
  float distance_remains_raw =
    lpath.remainedDistance(lpath.cbegin(), it_nearest, lpath.cend(), pos_on_line_raw);
  if (path_length < no_pos_cntl_dist_) distance_remains = distance_remains_raw = remain_local = 0;

  // Signed distance error
  const float dist_err =
    trajectory_tracker::lineDistance(lpath[i_nearest_prev].pos_, lpath[i_nearest].pos_, origin);

  // Angular error
  const Eigen::Vector2d vec = lpath[i_nearest].pos_ - lpath[i_nearest_prev].pos_;
  float angle_remains = -atan2(vec[1], vec[0]);
  const float angle_pose = allow_backward_ ? lpath[i_nearest].yaw_ : -angle_remains;
  float sign_vel = 1.0;
  if (
    std::cos(-angle_remains) * std::cos(angle_pose) +
      std::sin(-angle_remains) * std::sin(angle_pose) <
    0) {
    sign_vel = -1.0;
    angle_remains = angle_remains + M_PI;
  }
  angle_remains = trajectory_tracker::angleNormalized(angle_remains);

  // Curvature
  const float curv = lpath.getCurvature(it_nearest, it_local_goal, pos_on_line, curv_forward_);

  RCLCPP_DEBUG(
    logger_,
    "trajectory_tracker: nearest: %d, local goal: %d, done: %d, goal: %lu, remain: %0.3f, "
    "remain_local: %0.3f",
    i_nearest, i_local_goal, path_step_done_, lpath.size(), distance_remains, remain_local);

  bool arrive_local_goal(false);
  bool in_place_turning = (vec[1] == 0.0 && vec[0] == 0.0);

  TrackingResult result(trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING);

  // Stop and rotate
  const bool large_angle_error =
    std::abs(rotate_ang_) < M_PI && std::cos(rotate_ang_) > std::cos(angle_remains);
  if (
    large_angle_error || std::abs(remain_local) < stop_tolerance_dist_ ||
    path_length < min_track_path_ || in_place_turning) {
    if (large_angle_error) {
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_INFO_THROTTLE(
        logger_, clock, 1000, "Stop and rotate due to large angular error: %0.3f", angle_remains);
    }

    if (
      path_length < min_track_path_ || std::abs(remain_local) < stop_tolerance_dist_ ||
      in_place_turning) {
      angle_remains = trajectory_tracker::angleNormalized(-(it_local_goal - 1)->yaw_);
      if (it_local_goal != lpath.end()) arrive_local_goal = true;
    }
    if (path_length < stop_tolerance_dist_ || in_place_turning)
      distance_remains = distance_remains_raw = 0.0;

    result.turning_in_place = true;
    result.target_linear_vel = linear_vel;
    result.distance_remains = distance_remains;
    result.distance_remains_raw = distance_remains_raw;
    result.angle_remains = angle_remains;
  } else {
    // Too far from given path
    float dist_from_path = dist_err;
    if (i_nearest == 0)
      dist_from_path = -(lpath[i_nearest].pos_ - origin).norm();
    else if (i_nearest + 1 >= static_cast<int>(path_.size()))
      dist_from_path = -(lpath[i_nearest].pos_ - origin).norm();
    if (std::abs(dist_from_path) > d_stop_) {
      result.distance_remains = distance_remains;
      result.distance_remains_raw = distance_remains_raw;
      result.angle_remains = angle_remains;
      result.angle_remains_raw = angle_remains + yaw_raw;
      result.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FAR_FROM_PATH;
      return result;
    }

    // Path following control
    result.turning_in_place = false;
    result.target_linear_vel = linear_vel;
    result.distance_remains = distance_remains;
    result.distance_remains_raw = distance_remains_raw;
    result.angle_remains = angle_remains;
    result.angle_remains_raw = angle_remains + yaw_raw;
    result.distance_from_target = trajectory_tracker::clip(dist_err, d_lim_);
    result.signed_local_distance = -remain_local * sign_vel;
    result.tracking_point_curv = curv;
    result.tracking_point_x = pos_on_line[0];
    result.tracking_point_y = pos_on_line[1];
  }

  if (
    std::abs(result.distance_remains) < goal_tolerance_dist_ &&
    std::abs(result.angle_remains) < goal_tolerance_ang_ &&
    std::abs(result.distance_remains_raw) < goal_tolerance_dist_ &&
    std::abs(result.angle_remains_raw) < goal_tolerance_ang_ &&
    (goal_tolerance_lin_vel_ == 0.0 || std::abs(odom_linear_vel) < goal_tolerance_lin_vel_) &&
    (goal_tolerance_ang_vel_ == 0.0 || std::abs(odom_angular_vel) < goal_tolerance_ang_vel_) &&
    it_local_goal == lpath.end()) {
    result.status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL;
  }

  if (arrive_local_goal)
    result.path_step_done = i_local_goal;
  else
    result.path_step_done = std::max(path_step_done_, i_nearest - 1);

  return result;
}
}  // namespace trajectory_tracker
