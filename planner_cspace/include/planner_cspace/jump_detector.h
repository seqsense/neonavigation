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

#ifndef PLANNER_CSPACE__JUMP_DETECTOR_H_
#define PLANNER_CSPACE__JUMP_DETECTOR_H_

#include <cmath>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace planner_cspace
{
namespace planner_3d
{
class JumpDetector
{
protected:
  tf2_ros::Buffer & tfbuf_;
  tf2::Transform base_trans_prev_;
  tf2::Transform map_trans_prev_;

  std::string jump_detect_frame_;
  std::string map_frame_;

  double pos_jump_;
  double yaw_jump_;

  bool init_;

  rclcpp::Logger logger_;

public:
  JumpDetector(tf2_ros::Buffer & tfbuf, const rclcpp::Logger & logger)
  : tfbuf_(tfbuf),
    base_trans_prev_(tf2::Quaternion(0, 0, 0, 1)),
    map_trans_prev_(tf2::Quaternion(0, 0, 0, 1)),
    init_(false),
    logger_(logger)
  {
  }
  void setMapFrame(const std::string & frame_id) { map_frame_ = frame_id; }
  void setBaseFrame(const std::string & frame_id) { jump_detect_frame_ = frame_id; }
  void setThresholds(const double pos_jump, const double yaw_jump)
  {
    pos_jump_ = pos_jump;
    yaw_jump_ = yaw_jump;
  }
  bool detectJump()
  {
    tf2::Stamped<tf2::Transform> base_trans;
    tf2::Stamped<tf2::Transform> map_trans;
    try {
      geometry_msgs::msg::TransformStamped base_trans_tmp =
        tfbuf_.lookupTransform(jump_detect_frame_, "base_link", rclcpp::Time(0, 0, RCL_ROS_TIME));
      geometry_msgs::msg::TransformStamped map_trans_tmp =
        tfbuf_.lookupTransform(map_frame_, "base_link", rclcpp::Time(0, 0, RCL_ROS_TIME));
      tf2::fromMsg(base_trans_tmp, base_trans);
      tf2::fromMsg(map_trans_tmp, map_trans);
    } catch (tf2::TransformException & e) {
      return false;
    }
    const auto diff =
      map_trans.inverse() * map_trans_prev_ * (base_trans_prev_.inverse() * base_trans);
    base_trans_prev_ = base_trans;
    map_trans_prev_ = map_trans;

    if (!init_) {
      init_ = true;
      return false;
    }

    const auto pos_diff = diff.getOrigin().length();
    const auto yaw_diff = tf2::getYaw(diff.getRotation());

    if (pos_diff > pos_jump_ || std::abs(yaw_diff) > yaw_jump_) {
      RCLCPP_ERROR(
        logger_, "Position jumped (%0.3f/%0.3f, %0.3f/%0.3f); clearing history", pos_diff,
        pos_jump_, yaw_diff, yaw_jump_);
      return true;
    }
    return false;
  }
};
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE__JUMP_DETECTOR_H_
