/*
 * Copyright (c) 2014-2019, the neonavigation authors
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

#ifndef TRACK_ODOMETRY__TRACK_ODOMETRY_H_
#define TRACK_ODOMETRY__TRACK_ODOMETRY_H_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.h"
#include "track_odometry/kalman_filter1.h"

namespace track_odometry
{
// Tunable parameters filled by the interface layer (from ROS parameters) and
// pushed into the logic via setParameters().
struct TrackOdometryParams
{
  // Initial value of the base_link frame id. In "without_odom" mode this is the
  // configured base_link_id; otherwise it is empty and resolved from odometry.
  std::string base_link_id;
  // When non-empty, overrides base_link_id and disables resolving it from the
  // incoming odometry child_frame_id.
  std::string base_link_id_overwrite;
  double z_filter_timeconst = -1.0;
  double tf_tolerance = 0.01;
  bool use_kf = true;
  bool negative_slip = false;
  bool debug = false;
  // sigma_odom [rad/s]: standard deviation of odometry angular vel on straight running
  double sigma_odom = 0.005;
  // sigma_predict [sigma/second]: prediction sigma of kalman filter
  double sigma_predict = 0.5;
  // predict_filter_tc [sec.]: LPF time-constant to forget estimated slip ratio
  double predict_filter_tc = 1.0;
};

// IMU/odometry Kalman fusion logic separated from the ROS interface.
// Message types, time and logging use the ROS 2 (rclcpp) surface, which on
// ROS 1 is provided by sq_ros1_rclcpp_compat so the same source builds on both;
// node handles, publishers, subscribers, message_filters and parameter access
// stay in the interface layer.
class TrackOdometry
{
public:
  // Result of processing a single odometry update. When valid is true the
  // interface layer publishes odom and, if configured, broadcasts transform.
  struct OdomResult
  {
    bool valid = false;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::TransformStamped transform;
  };

  TrackOdometry(tf2_ros::Buffer & tf_buffer, const rclcpp::Logger & logger);

  void setParameters(const TrackOdometryParams & params);

  // Transform the incoming IMU into the base_link frame and cache it.
  void processImu(const std::shared_ptr<const sensor_msgs::msg::Imu> & msg);

  // Fuse the cached IMU with the incoming odometry and return the corrected
  // odometry together with the corresponding transform.
  OdomResult processOdom(const std::shared_ptr<const nav_msgs::msg::Odometry> & msg);

  // Override the z component of the previous odometry pose (reset_z topic).
  void resetZ(const double z);

private:
  tf2_ros::Buffer & tf_buffer_;
  rclcpp::Logger logger_;

  TrackOdometryParams params_;
  double z_filter_timeconst_;

  std::string base_link_id_;

  nav_msgs::msg::Odometry odom_prev_;
  nav_msgs::msg::Odometry odomraw_prev_;
  sensor_msgs::msg::Imu imu_;

  KalmanFilter1 slip_;
  double dist_;

  bool has_imu_;
  bool has_odom_;
};
}  // namespace track_odometry

#endif  // TRACK_ODOMETRY__TRACK_ODOMETRY_H_
