/*
 * Copyright (c) 2018-2019, the neonavigation authors
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

// ROS 2 integration test mirroring the ROS 1 track_odometry_rostest.test. It is
// driven by test_track_odometry_launch.py, which starts one track_odometry node
// per configuration (global, with/without z-filter, old/new parameter names).
// This gtest binary owns its rclcpp context (rclcpp::init in main) and is
// started as a plain node by the launch harness, which checks its exit code
// after shutdown.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class TrackOdometryTest : public ::testing::TestWithParam<const char *>
{
protected:
  std::vector<nav_msgs::msg::Odometry> odom_msg_buffer_;

public:
  void initializeNode(const std::string & ns)
  {
    const std::string node_ns = ns.empty() ? "" : ("/" + ns);
    node_ = std::make_shared<rclcpp::Node>("test_track_odometry", node_ns);
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom_raw", rclcpp::QoS(10));
    pub_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(10));
    sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10), std::bind(&TrackOdometryTest::cbOdom, this, std::placeholders::_1));
  }
  bool initializeTrackOdometry(nav_msgs::msg::Odometry & odom_raw, sensor_msgs::msg::Imu & imu)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::WallRate rate(100.0);
    odom_ = nullptr;
    for (int i = 0; i < 1000 && rclcpp::ok(); ++i) {
      const rclcpp::Time now = node_->now();
      odom_raw.header.stamp = now;
      imu.header.stamp = now + rclcpp::Duration::from_seconds(0.0001);
      pub_odom_->publish(odom_raw);
      pub_imu_->publish(imu);
      rate.sleep();
      odom_.reset();
      rclcpp::spin_some(node_);
      if (odom_ && i > 50) break;
    }
    return static_cast<bool>(odom_);
  }
  bool run(
    nav_msgs::msg::Odometry & odom_raw, sensor_msgs::msg::Imu & imu, const double dt,
    const int steps)
  {
    rclcpp::WallRate rate(1.0 / dt);
    int cnt(0);

    while (rclcpp::ok()) {
      tf2::Quaternion quat_odom;
      tf2::fromMsg(odom_raw.pose.pose.orientation, quat_odom);
      odom_raw.pose.pose.orientation = tf2::toMsg(
        quat_odom *
        tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), dt * odom_raw.twist.twist.angular.z));
      tf2::Quaternion quat_imu;
      tf2::fromMsg(imu.orientation, quat_imu);
      imu.orientation = tf2::toMsg(
        quat_imu * tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), dt * imu.angular_velocity.z));

      const double yaw = tf2::getYaw(odom_raw.pose.pose.orientation);
      odom_raw.pose.pose.position.x += cos(yaw) * dt * odom_raw.twist.twist.linear.x;
      odom_raw.pose.pose.position.y += sin(yaw) * dt * odom_raw.twist.twist.linear.x;

      stepAndPublish(odom_raw, imu, dt);

      rate.sleep();
      rclcpp::spin_some(node_);
      if (++cnt >= steps) break;
    }
    flushOdomMsgs();
    return rclcpp::ok();
  }
  void stepAndPublish(
    nav_msgs::msg::Odometry & odom_raw, sensor_msgs::msg::Imu & imu, const double dt)
  {
    odom_raw.header.stamp =
      rclcpp::Time(odom_raw.header.stamp) + rclcpp::Duration::from_seconds(dt);
    imu.header.stamp = rclcpp::Time(imu.header.stamp) + rclcpp::Duration::from_seconds(dt);
    pub_imu_->publish(imu);

    // Buffer odom message to add delay and jitter.
    // Send odometry in half rate of IMU.
    ++odom_cnt_;
    if (odom_cnt_ % 2 == 0) odom_msg_buffer_.push_back(odom_raw);
    if (odom_msg_buffer_.size() > 10) {
      flushOdomMsgs();
    }
  }
  void flushOdomMsgs()
  {
    for (nav_msgs::msg::Odometry & o : odom_msg_buffer_) {
      pub_odom_->publish(o);
    }
    odom_msg_buffer_.clear();
  }
  void waitAndSpinOnce()
  {
    nav_msgs::msg::Odometry::ConstSharedPtr odom_prev = odom_;
    while (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(node_);
      if (odom_prev == odom_) {
        // no more new messages
        return;
      }
      odom_prev = odom_;
    }
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  size_t odom_cnt_;

  void cbOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; }
};

TEST_F(TrackOdometryTest, OdomImuFusion)
{
  initializeNode("");

  const double dt = 0.01;
  const int steps = 200;

  nav_msgs::msg::Odometry odom_raw;
  odom_raw.header.frame_id = "odom";
  odom_raw.pose.pose.orientation.w = 1;

  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "base_link";
  imu.orientation.w = 1;
  imu.linear_acceleration.z = 9.8;

  ASSERT_TRUE(initializeTrackOdometry(odom_raw, imu));

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  odom_raw.twist.twist.linear.x = 0.0;
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.x, 1.0, 1e-3);
  ASSERT_NEAR(odom_->pose.pose.position.y, 0.0, 1e-3);
  ASSERT_NEAR(odom_->pose.pose.position.z, 0.0, 1e-3);
  ASSERT_NEAR(tf2::getYaw(odom_->pose.pose.orientation), 0.0, 1e-3);

  // Turn 90 degrees with 10% of odometry errors
  imu.angular_velocity.z = M_PI * 0.25;
  odom_raw.twist.twist.angular.z = imu.angular_velocity.z * 1.1;  // Odometry with error
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  imu.angular_velocity.z = 0;
  odom_raw.twist.twist.angular.z = 0;
  imu.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), M_PI / 2));
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.x, 1.0, 1e-2);
  ASSERT_NEAR(odom_->pose.pose.position.y, 0.0, 1e-2);
  ASSERT_NEAR(odom_->pose.pose.position.z, 0.0, 1e-2);
  ASSERT_NEAR(tf2::getYaw(odom_->pose.pose.orientation), M_PI / 2, 1e-2);

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  odom_raw.twist.twist.linear.x = 0.0;
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.x, 1.0, 5e-2);
  ASSERT_NEAR(odom_->pose.pose.position.y, 1.0, 5e-2);
  ASSERT_NEAR(odom_->pose.pose.position.z, 0.0, 5e-2);
  ASSERT_NEAR(tf2::getYaw(odom_->pose.pose.orientation), M_PI / 2, 1e-2);
}

TEST_P(TrackOdometryTest, ZFilterOff)
{
  const std::string ns_postfix(GetParam());
  initializeNode("no_z_filter" + ns_postfix);

  const double dt = 0.01;
  const int steps = 200;

  nav_msgs::msg::Odometry odom_raw;
  odom_raw.header.frame_id = "odom";
  odom_raw.pose.pose.orientation.w = 1;

  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "base_link";
  imu.orientation.y = sin(-M_PI / 4);
  imu.orientation.w = cos(-M_PI / 4);
  imu.linear_acceleration.x = -9.8;

  ASSERT_TRUE(initializeTrackOdometry(odom_raw, imu));

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  odom_raw.twist.twist.linear.x = 0.0;
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.x, 0.0, 1e-3);
  ASSERT_NEAR(odom_->pose.pose.position.y, 0.0, 1e-3);
  ASSERT_NEAR(odom_->pose.pose.position.z, 1.0, 1e-3);
}

TEST_P(TrackOdometryTest, ZFilterOn)
{
  const std::string ns_postfix(GetParam());
  initializeNode("z_filter" + ns_postfix);

  const double dt = 0.01;
  const int steps = 200;

  nav_msgs::msg::Odometry odom_raw;
  odom_raw.header.frame_id = "odom";
  odom_raw.pose.pose.orientation.w = 1;

  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "base_link";
  imu.orientation.y = sin(-M_PI / 4);
  imu.orientation.w = cos(-M_PI / 4);
  imu.linear_acceleration.x = -9.8;

  ASSERT_TRUE(initializeTrackOdometry(odom_raw, imu));

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  odom_raw.twist.twist.linear.x = 0.0;
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.z, 1.0 - 1.0 / M_E, 5e-2);
}

INSTANTIATE_TEST_SUITE_P(
  TrackOdometryTestInstance, TrackOdometryTest, ::testing::Values("", "_old_param"));

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
