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

#ifndef SAFETY_LIMITER__SAFETY_LIMITER_H_
#define SAFETY_LIMITER__SAFETY_LIMITER_H_

#include <cassert>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_ros/buffer.h"

namespace safety_limiter
{
pcl::PointXYZ operator-(const pcl::PointXYZ & a, const pcl::PointXYZ & b);
pcl::PointXYZ operator+(const pcl::PointXYZ & a, const pcl::PointXYZ & b);
pcl::PointXYZ operator*(const pcl::PointXYZ & a, const float & b);

// Minimal 2D vector helper used by the footprint polygon geometry.
class vec
{
public:
  float c[2];
  vec(const float x, const float y)
  {
    c[0] = x;
    c[1] = y;
  }
  vec() { c[0] = c[1] = 0.0; }
  float & operator[](const int & i)
  {
    assert(i < 2);
    return c[i];
  }
  const float & operator[](const int & i) const
  {
    assert(i < 2);
    return c[i];
  }
  vec operator-(const vec & a) const
  {
    vec out = *this;
    out[0] -= a[0];
    out[1] -= a[1];
    return out;
  }
  float cross(const vec & a) const { return (*this)[0] * a[1] - (*this)[1] * a[0]; }
  float dot(const vec & a) const { return (*this)[0] * a[0] + (*this)[1] * a[1]; }
  float dist(const vec & a) const { return std::hypot((*this)[0] - a[0], (*this)[1] - a[1]); }
  float dist_line(const vec & a, const vec & b) const
  {
    return (b - a).cross((*this) - a) / b.dist(a);
  }
  float dist_linestrip(const vec & a, const vec & b) const
  {
    if ((b - a).dot((*this) - a) <= 0) return this->dist(a);
    if ((a - b).dot((*this) - b) <= 0) return this->dist(b);
    return std::abs(this->dist_line(a, b));
  }
};

// Footprint polygon used for collision checking.
class polygon
{
public:
  std::vector<vec> v;
  void move(const float & x, const float & y, const float & yaw)
  {
    const float cos_v = cosf(yaw);
    const float sin_v = sinf(yaw);
    for (auto & p : v) {
      const auto tmp = p;
      p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
      p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
    }
  }
  bool inside(const vec & a) const
  {
    int cn = 0;
    for (size_t i = 0; i < v.size() - 1; i++) {
      auto & v1 = v[i];
      auto & v2 = v[i + 1];
      if ((v1[1] <= a[1] && a[1] < v2[1]) || (v2[1] <= a[1] && a[1] < v1[1])) {
        float lx;
        lx = v1[0] + (v2[0] - v1[0]) * (a[1] - v1[1]) / (v2[1] - v1[1]);
        if (a[0] < lx) cn++;
      }
    }
    return ((cn & 1) == 1);
  }
  float dist(const vec & a) const
  {
    float dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < v.size() - 1; i++) {
      auto & v1 = v[i];
      auto & v2 = v[i + 1];
      auto d = a.dist_linestrip(v1, v2);
      if (d < dist) dist = d;
    }
    return dist;
  }
};

// SafetyLimiter holds the pure collision-prediction logic.
// It has no knowledge of ROS nodes, publishers, subscribers, parameters or
// dynamic_reconfigure; those belong to the interface layer (SafetyLimiterNode).
// Message types, PCL, TF, time and logging use the ROS 2 (rclcpp) surface, which
// on ROS 1 is provided by the sq_ros1_rclcpp_compat shim so the same source
// builds for both ROS 1 and ROS 2.
class SafetyLimiter
{
public:
  // Tunable parameters. The interface layer fills this from
  // dynamic_reconfigure and pushes it via setParameters().
  struct Parameters
  {
    double vel[2] = {0.0, 0.0};
    double acc[2] = {0.0, 0.0};
    double dt = 0.0;
    double d_margin = 0.0;
    double d_escape = 0.0;
    double yaw_margin = 0.0;
    double yaw_escape = 0.0;
    double z_range[2] = {0.0, 0.0};
    double downsample_grid = 0.0;
    double hz = 0.0;
    bool allow_empty_cloud = false;
  };

  // Result of a single prediction step. The interface layer decides how to act
  // on r_lim and, when has_collision_points is true, publishes collision_points.
  struct PredictResult
  {
    double r_lim = 1.0;
    bool has_collision_points = false;
    sensor_msgs::msg::PointCloud collision_points;
  };

  SafetyLimiter(tf2_ros::Buffer & tfbuf, const rclcpp::Logger & logger);

  void setParameters(const Parameters & params);
  void setFootprint(const polygon & footprint, const float footprint_radius);
  void setBaseFrame(const std::string & base_frame_id);

  float footprintRadius() const { return footprint_radius_; }
  bool hasCollisionAtNow() const { return has_collision_at_now_; }
  rclcpp::Time stuckStartedSince() const { return stuck_started_since_; }

  // Run one collision-prediction step against the accumulated cloud and return
  // the allowed velocity ratio together with the colliding points.
  // Note: cloud is transformed in place, matching the original node behavior.
  PredictResult predict(
    const geometry_msgs::msg::Twist & twist, const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

private:
  tf2_ros::Buffer & tfbuf_;
  rclcpp::Logger logger_;

  Parameters params_;
  double tmax_;
  std::string base_frame_id_;
  polygon footprint_p_;
  float footprint_radius_;

  bool has_collision_at_now_;
  rclcpp::Time stuck_started_since_;
};
}  // namespace safety_limiter

#endif  // SAFETY_LIMITER__SAFETY_LIMITER_H_
