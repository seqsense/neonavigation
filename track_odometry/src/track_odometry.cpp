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

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <track_odometry/track_odometry.h>

namespace track_odometry
{
namespace
{
Eigen::Vector3d toEigen(const geometry_msgs::Point& a)
{
  return Eigen::Vector3d(a.x, a.y, a.z);
}
Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion& a)
{
  return Eigen::Quaterniond(a.w, a.x, a.y, a.z);
}
geometry_msgs::Point toPoint(const Eigen::Vector3d& a)
{
  geometry_msgs::Point b;
  b.x = a.x();
  b.y = a.y();
  b.z = a.z();
  return b;
}
geometry_msgs::Vector3 toVector3(const Eigen::Vector3d& a)
{
  geometry_msgs::Vector3 b;
  b.x = a.x();
  b.y = a.y();
  b.z = a.z();
  return b;
}
}  // namespace

TrackOdometry::TrackOdometry(tf2_ros::Buffer& tf_buffer)
  : tf_buffer_(tf_buffer)
  , z_filter_timeconst_(-1.0)
  , dist_(0.0)
  , has_imu_(false)
  , has_odom_(false)
{
  slip_.set(0.0, 0.1);
}

void TrackOdometry::setParameters(const TrackOdometryParams& params)
{
  params_ = params;
  z_filter_timeconst_ = params.z_filter_timeconst;

  base_link_id_ = params.base_link_id;
  if (params.base_link_id_overwrite.size() > 0)
  {
    base_link_id_ = params.base_link_id_overwrite;
  }
}

void TrackOdometry::resetZ(const double z)
{
  odom_prev_.pose.pose.position.z = z;
}

void TrackOdometry::processImu(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (base_link_id_.size() == 0)
  {
    ROS_ERROR("base_link id is not specified.");
    return;
  }

  imu_.header = msg->header;
  try
  {
    geometry_msgs::TransformStamped trans = tf_buffer_.lookupTransform(
        base_link_id_, msg->header.frame_id, ros::Time(0), ros::Duration(0.1));

    geometry_msgs::Vector3Stamped vin, vout;
    vin.header = imu_.header;
    vin.header.stamp = ros::Time(0);
    vin.vector = msg->linear_acceleration;
    tf2::doTransform(vin, vout, trans);
    imu_.linear_acceleration = vout.vector;

    vin.header = imu_.header;
    vin.header.stamp = ros::Time(0);
    vin.vector = msg->angular_velocity;
    tf2::doTransform(vin, vout, trans);
    imu_.angular_velocity = vout.vector;

    tf2::Stamped<tf2::Quaternion> qin, qout;
    geometry_msgs::QuaternionStamped qmin, qmout;
    qmin.header = imu_.header;
    qmin.quaternion = msg->orientation;
    tf2::fromMsg(qmin, qin);

    auto axis = qin.getAxis();
    auto angle = qin.getAngle();
    geometry_msgs::Vector3Stamped axis2;
    geometry_msgs::Vector3Stamped axis1;
    axis1.vector = tf2::toMsg(axis);
    axis1.header.stamp = ros::Time(0);
    axis1.header.frame_id = qin.frame_id_;
    tf2::doTransform(axis1, axis2, trans);

    tf2::fromMsg(axis2.vector, axis);
    qout.setData(tf2::Quaternion(axis, angle));
    qout.stamp_ = qin.stamp_;
    qout.frame_id_ = base_link_id_;

    qmout = tf2::toMsg(qout);
    imu_.orientation = qmout.quaternion;
    // ROS_INFO("%0.3f %s -> %0.3f %s",
    //   tf2::getYaw(qmin.quaternion), qmin.header.frame_id.c_str(),
    //   tf2::getYaw(qmout.quaternion), qmout.header.frame_id.c_str());

    has_imu_ = true;
  }
  catch (tf2::TransformException& e)
  {
    ROS_ERROR("%s", e.what());
    has_imu_ = false;
    return;
  }
}

TrackOdometry::OdomResult TrackOdometry::processOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  OdomResult result;
  nav_msgs::Odometry odom = *msg;
  if (has_odom_)
  {
    const double dt = (odom.header.stamp - odomraw_prev_.header.stamp).toSec();
    if (params_.base_link_id_overwrite.size() == 0)
    {
      base_link_id_ = odom.child_frame_id;
    }

    if (!has_imu_)
    {
      ROS_ERROR_THROTTLE(1.0, "IMU data not received");
      return result;
    }

    double slip_ratio = 1.0;
    odom.header.stamp += ros::Duration(params_.tf_tolerance);
    odom.twist.twist.angular = imu_.angular_velocity;
    odom.pose.pose.orientation = imu_.orientation;

    double w_imu = imu_.angular_velocity.z;
    const double w_odom = msg->twist.twist.angular.z;

    if (w_imu * w_odom < 0 && !params_.negative_slip)
      w_imu = w_odom;

    slip_.predict(-slip_.x_ * dt * params_.predict_filter_tc, dt * params_.sigma_predict);
    if (std::abs(w_odom) > params_.sigma_odom * 3)
    {
      // non-kf mode: calculate slip_ratio if angular vel < 3*sigma
      slip_ratio = w_imu / w_odom;
    }

    const double slip_ratio_per_angvel =
        (w_odom - w_imu) / (w_odom * std::abs(w_odom));
    double slip_ratio_per_angvel_sigma =
        params_.sigma_odom * std::abs(2.0 * w_odom * params_.sigma_odom /
                                      std::pow(w_odom * w_odom - params_.sigma_odom * params_.sigma_odom, 2));
    if (std::abs(w_odom) < params_.sigma_odom)
      slip_ratio_per_angvel_sigma = std::numeric_limits<double>::infinity();

    slip_.measure(slip_ratio_per_angvel, slip_ratio_per_angvel_sigma);
    // printf("%0.5f %0.5f %0.5f   %0.5f %0.5f  %0.5f\n",
    //   slip_ratio_per_angvel, slip_ratio_sigma, slip_ratio_per_angvel_sigma,
    //   slip_.x_, slip_.sigma_, msg->twist.twist.angular.z);

    if (params_.debug)
    {
      printf("%0.3f %0.3f  %0.3f  %0.3f %0.3f  %0.3f  %0.3f\n",
             imu_.angular_velocity.z,
             msg->twist.twist.angular.z,
             slip_ratio,
             slip_.x_, slip_.sigma_,
             odom.twist.twist.linear.x, dist_);
    }
    dist_ += odom.twist.twist.linear.x * dt;

    const Eigen::Vector3d diff = toEigen(msg->pose.pose.position) - toEigen(odomraw_prev_.pose.pose.position);
    Eigen::Vector3d v =
        toEigen(odom.pose.pose.orientation) * toEigen(msg->pose.pose.orientation).inverse() * diff;
    if (params_.use_kf)
      v *= 1.0 - slip_.x_;
    else
      v *= slip_ratio;

    odom.pose.pose.position = toPoint(toEigen(odom_prev_.pose.pose.position) + v);
    if (z_filter_timeconst_ > 0)
      odom.pose.pose.position.z *= 1.0 - (dt / z_filter_timeconst_);

    odom.child_frame_id = base_link_id_;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odom.header;
    odom_trans.child_frame_id = base_link_id_;
    odom_trans.transform.translation = toVector3(toEigen(odom.pose.pose.position));
    odom_trans.transform.rotation = odom.pose.pose.orientation;

    result.valid = true;
    result.odom = odom;
    result.transform = odom_trans;
  }
  odomraw_prev_ = *msg;
  odom_prev_ = odom;
  has_odom_ = true;
  return result;
}
}  // namespace track_odometry
