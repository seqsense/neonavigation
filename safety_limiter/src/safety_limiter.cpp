/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

#include "safety_limiter/safety_limiter.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_ros/buffer.h"

namespace safety_limiter
{
pcl::PointXYZ operator-(const pcl::PointXYZ & a, const pcl::PointXYZ & b)
{
  auto c = a;
  c.x -= b.x;
  c.y -= b.y;
  c.z -= b.z;
  return c;
}
pcl::PointXYZ operator+(const pcl::PointXYZ & a, const pcl::PointXYZ & b)
{
  auto c = a;
  c.x += b.x;
  c.y += b.y;
  c.z += b.z;
  return c;
}
pcl::PointXYZ operator*(const pcl::PointXYZ & a, const float & b)
{
  auto c = a;
  c.x *= b;
  c.y *= b;
  c.z *= b;
  return c;
}

SafetyLimiter::SafetyLimiter(tf2_ros::Buffer & tfbuf, const rclcpp::Logger & logger)
: tfbuf_(tfbuf),
  logger_(logger),
  tmax_(0.0),
  footprint_radius_(0.0),
  has_collision_at_now_(false),
  stuck_started_since_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
}

void SafetyLimiter::setParameters(const Parameters & params)
{
  params_ = params;

  tmax_ = 0.0;
  for (int i = 0; i < 2; i++) {
    auto t = params_.vel[i] / params_.acc[i];
    if (tmax_ < t) tmax_ = t;
  }
  tmax_ *= 1.5;
  tmax_ += std::max(params_.d_margin / params_.vel[0], params_.yaw_margin / params_.vel[1]);
}

void SafetyLimiter::setFootprint(const polygon & footprint, const float footprint_radius)
{
  footprint_p_ = footprint;
  footprint_radius_ = footprint_radius;
}

void SafetyLimiter::setBaseFrame(const std::string & base_frame_id)
{
  base_frame_id_ = base_frame_id;
}

SafetyLimiter::PredictResult SafetyLimiter::predict(
  const geometry_msgs::msg::Twist & twist, const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  PredictResult result;

  if (cloud->size() == 0) {
    if (params_.allow_empty_cloud) {
      result.r_lim = 1.0;
      return result;
    }
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "safety_limiter: Empty pointcloud passed.");
    result.r_lim = 0.0;
    return result;
  }

  const rclcpp::Time cloud_stamp = pcl_conversions::fromPCL(cloud->header.stamp);
  const bool can_transform =
    tfbuf_.canTransform(base_frame_id_, cloud->header.frame_id, cloud_stamp);
  rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  if (can_transform) stamp = cloud_stamp;

  geometry_msgs::msg::TransformStamped fixed_to_base;
  try {
    fixed_to_base = tfbuf_.lookupTransform(base_frame_id_, cloud->header.frame_id, stamp);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "safety_limiter: Transform failed: %s", e.what());
    result.r_lim = 0.0;
    return result;
  }

  const Eigen::Affine3f fixed_to_base_eigen =
    Eigen::Translation3f(
      fixed_to_base.transform.translation.x, fixed_to_base.transform.translation.y,
      fixed_to_base.transform.translation.z) *
    Eigen::Quaternionf(
      fixed_to_base.transform.rotation.w, fixed_to_base.transform.rotation.x,
      fixed_to_base.transform.rotation.y, fixed_to_base.transform.rotation.z);
  pcl::transformPointCloud(*cloud, *cloud, fixed_to_base_eigen);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> ds;
  ds.setInputCloud(cloud);
  ds.setLeafSize(params_.downsample_grid, params_.downsample_grid, params_.downsample_grid);
  ds.filter(*pc);

  auto filter_z = [this](pcl::PointXYZ & p) {
    if (p.z < this->params_.z_range[0] || this->params_.z_range[1] < p.z) return true;
    p.z = 0.0;
    return false;
  };
  pc->erase(std::remove_if(pc->points.begin(), pc->points.end(), filter_z), pc->points.end());

  if (pc->size() == 0) {
    if (params_.allow_empty_cloud) {
      result.r_lim = 1.0;
      return result;
    }
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "safety_limiter: Empty pointcloud passed.");
    result.r_lim = 0.0;
    return result;
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(pc);

  Eigen::Affine3f move;
  Eigen::Affine3f move_inv;
  Eigen::Affine3f motion =
    Eigen::AngleAxisf(-twist.angular.z * params_.dt, Eigen::Vector3f::UnitZ()) *
    Eigen::Translation3f(
      Eigen::Vector3f(-twist.linear.x * params_.dt, -twist.linear.y * params_.dt, 0.0));
  Eigen::Affine3f motion_inv =
    Eigen::Translation3f(
      Eigen::Vector3f(twist.linear.x * params_.dt, twist.linear.y * params_.dt, 0.0)) *
    Eigen::AngleAxisf(twist.angular.z * params_.dt, Eigen::Vector3f::UnitZ());
  move.setIdentity();
  move_inv.setIdentity();
  sensor_msgs::msg::PointCloud & col_points = result.collision_points;
  col_points.header.frame_id = base_frame_id_;
  col_points.header.stamp = clock_->now();

  float d_col = 0;
  float yaw_col = 0;
  bool has_collision = false;
  float d_escape_remain = 0;
  float yaw_escape_remain = 0;
  has_collision_at_now_ = false;
  const double linear_vel = std::hypot(twist.linear.x, twist.linear.y);

  for (float t = 0; t < tmax_; t += params_.dt) {
    if (t != 0) {
      d_col += linear_vel * params_.dt;
      d_escape_remain -= linear_vel * params_.dt;
      yaw_col += twist.angular.z * params_.dt;
      yaw_escape_remain -= std::abs(twist.angular.z) * params_.dt;
      move = move * motion;
      move_inv = move_inv * motion_inv;
    }

    pcl::PointXYZ center;
    center = pcl::transformPoint(center, move_inv);
    std::vector<int> indices;
    std::vector<float> dist;
    const int num = kdtree.radiusSearch(center, footprint_radius_, indices, dist);
    if (num == 0) continue;

    bool colliding = false;
    for (auto & i : indices) {
      auto & p = pc->points[i];
      auto point = pcl::transformPoint(p, move);
      vec v(point.x, point.y);
      if (footprint_p_.inside(v)) {
        geometry_msgs::msg::Point32 pos;
        pos.x = p.x;
        pos.y = p.y;
        pos.z = p.z;
        col_points.points.push_back(pos);
        colliding = true;
        break;
      }
    }
    if (colliding) {
      d_col -= linear_vel * params_.dt;
      yaw_col -= twist.angular.z * params_.dt;
      if (t == 0) {
        // The robot is already in collision.
        // Allow movement under d_escape_ and yaw_escape_
        d_escape_remain = params_.d_escape;
        yaw_escape_remain = params_.yaw_escape;
        has_collision_at_now_ = true;
      }
      if (d_escape_remain <= 0 || yaw_escape_remain <= 0) {
        if (has_collision_at_now_) {
          // It's not possible to escape from collision; stop completely.
          d_col = yaw_col = 0;
        }

        has_collision = true;
        break;
      }
    }
  }
  result.has_collision_points = true;

  if (has_collision_at_now_) {
    if (stuck_started_since_ == rclcpp::Time(0, 0, RCL_ROS_TIME))
      stuck_started_since_ = clock_->now();
  } else {
    if (stuck_started_since_ != rclcpp::Time(0, 0, RCL_ROS_TIME))
      stuck_started_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  if (!has_collision) {
    result.r_lim = 1.0;
    return result;
  }

  // delay compensation:
  //   solve for d_compensated: d_compensated = d - delay * sqrt(2 * acc * d_compensated)
  //     d_compensated = d + acc * delay^2 - sqrt((acc * delay^2)^2 + 2 * d * acc * delay^2)

  const float delay = 1.0 * (1.0 / params_.hz) + params_.dt;
  const float acc_dtsq[2] = {
    static_cast<float>(params_.acc[0] * std::pow(delay, 2)),
    static_cast<float>(params_.acc[1] * std::pow(delay, 2)),
  };

  d_col = std::max<float>(
    0.0, std::abs(d_col) - params_.d_margin + acc_dtsq[0] -
           std::sqrt(std::pow(acc_dtsq[0], 2) + 2 * acc_dtsq[0] * std::abs(d_col)));
  yaw_col = std::max<float>(
    0.0, std::abs(yaw_col) - params_.yaw_margin + acc_dtsq[1] -
           std::sqrt(std::pow(acc_dtsq[1], 2) + 2 * acc_dtsq[1] * std::abs(yaw_col)));

  float d_r = std::sqrt(std::abs(2 * params_.acc[0] * d_col)) / linear_vel;
  float yaw_r = std::sqrt(std::abs(2 * params_.acc[1] * yaw_col)) / std::abs(twist.angular.z);
  if (!std::isfinite(d_r)) d_r = 1.0;
  if (!std::isfinite(yaw_r)) yaw_r = 1.0;

  result.r_lim = std::min(d_r, yaw_r);
  return result;
}
}  // namespace safety_limiter
