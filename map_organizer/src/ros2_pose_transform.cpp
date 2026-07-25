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

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace map_organizer
{
class PoseTransformNode : public rclcpp::Node
{
public:
  explicit PoseTransformNode(const rclcpp::NodeOptions & options);

private:
  void cbPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);

  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::string to_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
};

PoseTransformNode::PoseTransformNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pose_transform", options)
{
  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_in", 1, std::bind(&PoseTransformNode::cbPose, this, std::placeholders::_1));
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_out", 1);
  to_ = this->declare_parameter("to_frame", std::string("map"));
}

void PoseTransformNode::cbPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  try {
    geometry_msgs::msg::PoseStamped in;
    geometry_msgs::msg::PoseStamped out;
    in.header = msg->header;
    in.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    in.pose = msg->pose.pose;
    geometry_msgs::msg::TransformStamped trans = tfbuf_->lookupTransform(
      to_, msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
    tf2::doTransform(in, out, trans);
    auto out_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>(*msg);
    out_msg->header = out.header;
    out_msg->pose.pose = out.pose;
    pub_pose_->publish(std::move(out_msg));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "pose_transform: %s", e.what());
  }
}
}  // namespace map_organizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<map_organizer::PoseTransformNode>(options));
  rclcpp::shutdown();
  return 0;
}
