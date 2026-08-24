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

#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "track_odometry/track_odometry.h"

namespace track_odometry
{
class TrackOdometryNode : public rclcpp::Node
{
private:
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> sub_imu_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_reset_z_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string base_link_id_;
  std::string odom_id_;

  bool without_odom_;
  bool publish_tf_;

  TrackOdometry track_odometry_;

  void cbResetZ(const std_msgs::msg::Float32::ConstSharedPtr & msg)
  {
    track_odometry_.resetZ(msg->data);
  }
  void cbOdomImu(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg)
  {
    RCLCPP_DEBUG(
      this->get_logger(), "Synchronized timestamp: odom %0.3f, imu %0.3f",
      rclcpp::Time(odom_msg->header.stamp).seconds(),
      rclcpp::Time(imu_msg->header.stamp).seconds());
    cbImu(imu_msg);
    cbOdom(odom_msg);
  }
  void cbImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg) { track_odometry_.processImu(msg); }
  void cbOdom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    const TrackOdometry::OdomResult result = track_odometry_.processOdom(msg);
    if (result.valid) {
      auto out = std::make_unique<nav_msgs::msg::Odometry>(result.odom);
      pub_odom_->publish(std::move(out));
      if (publish_tf_) {
        tf_broadcaster_.sendTransform(result.transform);
      }
    }
  }
  void cbTimer()
  {
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = this->now();
    odom->header.frame_id = odom_id_;
    odom->child_frame_id = base_link_id_;
    odom->pose.pose.orientation.w = 1.0;
    cbOdom(odom);
  }

public:
  explicit TrackOdometryNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("track_odometry", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(*this),
    track_odometry_(tf_buffer_, this->get_logger())
  {
    // Time is read through the node's clock so the logic follows /clock
    // when use_sim_time is set.
    track_odometry_.setClock(this->get_clock());

    TrackOdometryParams params;

    // enable_tcp_no_delay is a ROS 1 transport hint with no ROS 2 QoS
    // equivalent; declared for parameter-surface parity only.
    this->declare_parameter("enable_tcp_no_delay", true);

    without_odom_ = this->declare_parameter("without_odom", false);

    rmw_qos_profile_t sub_qos = rmw_qos_profile_default;
    sub_qos.depth = 50;

    if (without_odom_) {
      sub_imu_raw_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(64),
        std::bind(&TrackOdometryNode::cbImu, this, std::placeholders::_1));
      params.base_link_id = this->declare_parameter("base_link_id", std::string("base_link"));
      odom_id_ = this->declare_parameter("odom_id", std::string("odom"));
      base_link_id_ = params.base_link_id;
    } else {
      sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
        this, "odom_raw", sub_qos);
      sub_imu_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
        this, "imu/data", sub_qos);

      const int sync_window = this->declare_parameter("sync_window", 50);
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(sync_window), *sub_odom_, *sub_imu_);
      sync_->registerCallback(
        std::bind(
          &TrackOdometryNode::cbOdomImu, this, std::placeholders::_1, std::placeholders::_2));

      params.base_link_id_overwrite = this->declare_parameter("base_link_id", std::string(""));
    }

    sub_reset_z_ = this->create_subscription<std_msgs::msg::Float32>(
      "reset_odometry_z", rclcpp::QoS(1),
      std::bind(&TrackOdometryNode::cbResetZ, this, std::placeholders::_1));
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(8));

    // z_filter is a deprecated exponential-filter alpha; z_filter_timeconst (in
    // seconds) supersedes it. A NaN sentinel detects whether z_filter was set.
    const double z_filter =
      this->declare_parameter("z_filter", std::numeric_limits<double>::quiet_NaN());
    if (!std::isnan(z_filter)) {
      params.z_filter_timeconst = -1.0;
      const double odom_freq = 100.0;
      if (0.0 < z_filter && z_filter < 1.0) {
        params.z_filter_timeconst = (1.0 / odom_freq) / (1.0 - z_filter);
      }
      RCLCPP_ERROR(
        this->get_logger(),
        "track_odometry: ~z_filter parameter (exponential filter (1 - alpha) value) is deprecated. "
        "Use ~z_filter_timeconst (in seconds) instead. "
        "Treated as z_filter_timeconst=%0.6f. (negative value means disabled)",
        params.z_filter_timeconst);
    } else {
      params.z_filter_timeconst = this->declare_parameter("z_filter_timeconst", -1.0);
    }
    params.tf_tolerance = this->declare_parameter("tf_tolerance", 0.01);
    params.use_kf = this->declare_parameter("use_kf", true);
    params.negative_slip = this->declare_parameter("enable_negative_slip", false);
    params.debug = this->declare_parameter("debug", false);
    publish_tf_ = this->declare_parameter("publish_tf", true);

    if (params.base_link_id_overwrite.size() > 0) {
      base_link_id_ = params.base_link_id_overwrite;
    }

    // sigma_odom [rad/s]: standard deviation of odometry angular vel on straight running
    params.sigma_odom = this->declare_parameter("sigma_odom", 0.005);
    // sigma_predict [sigma/second]: prediction sigma of kalman filter
    params.sigma_predict = this->declare_parameter("sigma_predict", 0.5);
    // predict_filter_tc [sec.]: LPF time-constant to forget estimated slip ratio
    params.predict_filter_tc = this->declare_parameter("predict_filter_tc", 1.0);

    track_odometry_.setParameters(params);

    if (without_odom_) {
      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / 50.0), std::bind(&TrackOdometryNode::cbTimer, this));
    }
  }
};
}  // namespace track_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(track_odometry::TrackOdometryNode)
