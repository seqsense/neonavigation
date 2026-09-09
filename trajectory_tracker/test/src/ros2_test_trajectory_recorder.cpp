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

// ROS 2 counterpart of trajectory_recorder_rostest.test: the recorder turns the
// map -> base_link transform into a path and drops it again on request.
//
// The ROS 1 test counted the callbacks it got with a queue of one; here the
// path topic is transient_local with a depth of ten, so every update arrives
// and the test waits for the path to have the expected length instead.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
const tf2::Transform kPoints[] = {
  tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0)),
  tf2::Transform(tf2::Quaternion(0, 0, 1, 0), tf2::Vector3(2, 0, 0)),
  tf2::Transform(tf2::Quaternion(0, 0, 0, -1), tf2::Vector3(3, 5, 0)),
  tf2::Transform(tf2::Quaternion(0, 0, -1, 0), tf2::Vector3(-1, 5, 1)),
};
const size_t kLen = sizeof(kPoints) / sizeof(tf2::Transform);

// q and -q are the same rotation, and nothing on the way through tf2 promises
// which of the two comes back.
void expectSameRotation(
  const geometry_msgs::msg::Quaternion & actual, const tf2::Quaternion & expected)
{
  const double dot = actual.x * expected.x() + actual.y * expected.y() + actual.z * expected.z() +
                     actual.w * expected.w();
  EXPECT_NEAR(std::abs(dot), 1.0, 1.0e-6);
}

class TrajectoryRecorderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_trajectory_recorder");
    sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
      "path", rclcpp::QoS(10).transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) { path_ = msg; });
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    client_ = node_->create_client<std_srvs::srv::Empty>("/trajectory_recorder/clear_path");
  }

  void spinFor(const double sec)
  {
    const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(sec);
    rclcpp::Rate rate(100);
    while (rclcpp::ok() && node_->now() < deadline) {
      rclcpp::spin_some(node_);
      rate.sleep();
    }
  }

  // Runs until the recorder has published a path of the given length, or fails.
  void waitForPathSize(const size_t size, const double timeout_sec)
  {
    const rclcpp::Time deadline = node_->now() + rclcpp::Duration::from_seconds(timeout_sec);
    rclcpp::Rate rate(100);
    while (rclcpp::ok() && node_->now() < deadline) {
      rclcpp::spin_some(node_);
      if (path_ && path_->poses.size() == size) {
        return;
      }
      rate.sleep();
    }
    FAIL() << "Timed out waiting for a path of " << size << " poses; got "
           << (path_ ? std::to_string(path_->poses.size()) : std::string("no path"));
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  nav_msgs::msg::Path::SharedPtr path_;
};
}  // namespace

TEST_F(TrajectoryRecorderTest, TfToPath)
{
  spinFor(1.0);
  for (const auto & p : kPoints) {
    for (size_t i = 0; i < 3; ++i) {
      geometry_msgs::msg::TransformStamped trans;
      trans.header.frame_id = "map";
      trans.header.stamp = node_->now() + rclcpp::Duration::from_seconds(0.1);
      trans.child_frame_id = "base_link";
      trans.transform = tf2::toMsg(p);
      tfb_->sendTransform(trans);
      spinFor(0.1);
    }
  }
  ASSERT_NO_FATAL_FAILURE(waitForPathSize(kLen, 5.0));

  for (size_t i = 0; i < kLen; ++i) {
    EXPECT_NEAR(path_->poses[i].pose.position.x, kPoints[i].getOrigin().x(), 1.0e-9);
    EXPECT_NEAR(path_->poses[i].pose.position.y, kPoints[i].getOrigin().y(), 1.0e-9);
    EXPECT_NEAR(path_->poses[i].pose.position.z, kPoints[i].getOrigin().z(), 1.0e-9);
    expectSameRotation(path_->poses[i].pose.orientation, kPoints[i].getRotation());
  }

  ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(5)));
  auto future = client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);

  // The next timer tick starts the path over from wherever the robot is now.
  path_.reset();
  ASSERT_NO_FATAL_FAILURE(waitForPathSize(1, 5.0));
  EXPECT_NEAR(path_->poses.back().pose.position.x, kPoints[kLen - 1].getOrigin().x(), 1.0e-9);
  EXPECT_NEAR(path_->poses.back().pose.position.y, kPoints[kLen - 1].getOrigin().y(), 1.0e-9);
  EXPECT_NEAR(path_->poses.back().pose.position.z, kPoints[kLen - 1].getOrigin().z(), 1.0e-9);
  expectSameRotation(path_->poses.back().pose.orientation, kPoints[kLen - 1].getRotation());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  const int ret = RUN_ALL_TESTS();
  // Not optional: musl keeps the context's globals alive until process exit,
  // where they are torn down in an order that crashes. Returning from main
  // with the context still initialized segfaults after every test has passed.
  rclcpp::shutdown();
  return ret;
}
