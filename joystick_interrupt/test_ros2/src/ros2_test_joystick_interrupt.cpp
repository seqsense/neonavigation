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

// ROS 2 integration test mirroring the ROS 1 joystick_interrupt_rostest.test.
// It is driven by test_joystick_interrupt_launch.py, which starts the
// joystick_interrupt node (default and omni configurations) and the
// joystick_mux node. This gtest binary owns its rclcpp context (rclcpp::init in
// main) and is started as a plain node by the launch harness, which checks its
// exit code after shutdown.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"

class JoystickInterruptTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_;

  void cbCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg) { cmd_vel_ = msg; }

public:
  explicit JoystickInterruptTest(const std::string & cmd_vel_topic = "cmd_vel")
  {
    node_ = std::make_shared<rclcpp::Node>("test_joystick_interrupt");
    pub_cmd_vel_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_input", rclcpp::QoS(1));
    pub_joy_ = node_->create_publisher<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(1));
    sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(1),
      std::bind(&JoystickInterruptTest::cbCmdVel, this, std::placeholders::_1));

    rclcpp::WallRate wait(10.0);
    for (size_t i = 0; i < 100 && rclcpp::ok(); ++i) {
      wait.sleep();
      rclcpp::spin_some(node_);
      if (i > 5 && pub_cmd_vel_->get_subscription_count() > 0) break;
    }
  }
  void publishCmdVel(const float lin, const float ang)
  {
    geometry_msgs::msg::Twist cmd_vel_out;
    cmd_vel_out.linear.x = lin;
    cmd_vel_out.angular.z = ang;
    pub_cmd_vel_->publish(cmd_vel_out);
  }
  void publishJoy(
    const int button, const int high_speed, const float lin0, const float ang0, const float lin1,
    const float ang1)
  {
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = node_->now();
    joy.buttons.resize(2);
    joy.buttons[0] = button;
    joy.buttons[1] = high_speed;
    joy.axes.resize(6);
    joy.axes[0] = lin0;
    joy.axes[1] = ang0;
    joy.axes[2] = lin1;
    joy.axes[3] = ang1;
    joy.axes[4] = 0;
    joy.axes[5] = 0;
    pub_joy_->publish(joy);
  }
};

TEST_F(JoystickInterruptTest, NoInterrupt)
{
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::WallRate rate(20.0);
  for (size_t i = 0; i < 25; ++i) {
    publishCmdVel(0.1, 0.2);
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(0, 0, 1, 1, 0, 0);
    else if (i < 15)
      publishJoy(0, 0, 0, 0, 1, 1);
    else if (i < 20)
      publishJoy(0, 1, 1, 1, 0, 0);
    else
      publishJoy(0, 1, 0, 0, 1, 1);

    rate.sleep();
    rclcpp::spin_some(node_);
    if (i < 3) continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    ASSERT_NEAR(cmd_vel_->linear.x, 0.1, 1e-3);
    ASSERT_NEAR(cmd_vel_->angular.z, 0.2, 1e-3);
  }
}

TEST_F(JoystickInterruptTest, Interrupt)
{
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::WallRate rate(20.0);
  for (size_t i = 0; i < 25; ++i) {
    publishCmdVel(0.1, 0.2);
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(1, 0, 1, 0, 0, 0);
    else if (i < 15)
      publishJoy(1, 0, 0, 0, 1, 0);
    else if (i < 20)
      publishJoy(1, 0, 0, 1, 0, 0);
    else
      publishJoy(1, 0, 0, 0, 0, 1);

    rate.sleep();
    rclcpp::spin_some(node_);
    if (i < 3) continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    if (i < 5) {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.1, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.2, 1e-3);
    } else if (i < 15) {
      ASSERT_NEAR(cmd_vel_->linear.x, 1.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.0, 1e-3);
    } else {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 1.0, 1e-3);
    }
  }
}

TEST_F(JoystickInterruptTest, InterruptNoTwistInput)
{
  rclcpp::sleep_for(std::chrono::seconds(1));
  // make sure the internal state of the joystick interrupt node
  // (i.e. last_input_twist_) is set back to a zero twist.
  publishCmdVel(0, 0);
  rclcpp::WallRate rate(20.0);
  for (size_t i = 0; i < 20; ++i) {
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(1, 0, 1, 0.5, 0, 0);
    else if (i < 15)
      publishJoy(0, 0, 1, 0, 0, 0);
    else
      publishJoy(0, 0, 0, 0.5, 0, 0);

    rate.sleep();
    rclcpp::spin_some(node_);
    if (i < 3) continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    if (i < 5) {
      ASSERT_NEAR(cmd_vel_->linear.x, 0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0, 1e-3);
    } else if (i < 10) {
      ASSERT_NEAR(cmd_vel_->linear.x, 1.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.5, 1e-3);
    } else {
      ASSERT_NEAR(cmd_vel_->linear.x, 0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0, 1e-3);
    }
  }
}

TEST_F(JoystickInterruptTest, InterruptHighSpeed)
{
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::WallRate rate(20.0);
  for (size_t i = 0; i < 25; ++i) {
    publishCmdVel(0.1, 0.2);
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(1, 1, 1, 0, 0, 0);
    else if (i < 15)
      publishJoy(1, 1, 0, 0, 1, 0);
    else if (i < 20)
      publishJoy(1, 1, 0, 1, 0, 0);
    else
      publishJoy(1, 1, 0, 0, 0, 1);

    rate.sleep();
    rclcpp::spin_some(node_);
    if (i < 3) continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    if (i < 5) {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.1, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.2, 1e-3);
    } else if (i < 15) {
      ASSERT_NEAR(cmd_vel_->linear.x, 2.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.0, 1e-3);
    } else {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 2.0, 1e-3);
    }
  }
}

class JoystickInterruptOmniTest : public JoystickInterruptTest
{
public:
  JoystickInterruptOmniTest() : JoystickInterruptTest("cmd_vel_omni") {}
  void publishJoy(
    const int button, const int high_speed, const float lin0, const float lin_y0, const float ang0,
    const float lin1, const float lin_y1, const float ang1)
  {
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = node_->now();
    joy.buttons.resize(2);
    joy.buttons[0] = button;
    joy.buttons[1] = high_speed;
    joy.axes.resize(6);
    joy.axes[0] = lin0;
    joy.axes[1] = ang0;
    joy.axes[2] = lin1;
    joy.axes[3] = ang1;
    joy.axes[4] = lin_y0;
    joy.axes[5] = lin_y1;
    pub_joy_->publish(joy);
  }
};

TEST_F(JoystickInterruptOmniTest, Interrupt)
{
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::WallRate rate(20.0);
  for (size_t i = 0; i < 25; ++i) {
    publishCmdVel(0.1, 0.2);
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(1, 0, 0.5, 0, 0, 0, 0, 0);
    else if (i < 15)
      publishJoy(1, 0, 0.5, 0, 0, -1, 0, 0);
    else if (i < 20)
      publishJoy(1, 0, 0, 0.3, 0, 0, 0, 0);
    else if (i < 25)
      publishJoy(1, 0, 0, 0.3, 0, 0, 1, 0);
    else if (i < 30)
      publishJoy(1, 0, 0, 0, -0.7, 0, 0, 0);
    else
      publishJoy(1, 0, 0, 0, -0.7, 0, 0, 1);

    rate.sleep();
    rclcpp::spin_some(node_);
    if (i < 3) continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    if (i < 5) {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.1, 1e-3);
      ASSERT_NEAR(cmd_vel_->linear.y, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.2, 1e-3);
    } else if (i < 10) {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.5, 1e-3);
      ASSERT_NEAR(cmd_vel_->linear.y, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.0, 1e-3);
    } else if (i < 15) {
      ASSERT_NEAR(cmd_vel_->linear.x, -1.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->linear.y, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.0, 1e-3);
    } else if (i < 20) {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->linear.y, 0.15, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.0, 1e-3);
    } else if (i < 25) {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->linear.y, 0.5, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.0, 1e-3);
    } else if (i < 30) {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->linear.y, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, -0.7, 1e-3);
    } else {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->linear.y, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 1.0, 1e-3);
    }
  }
}

class JoystickMuxTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub2_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;

  std_msgs::msg::Int32::ConstSharedPtr msg_;

  void cbMsg(const std_msgs::msg::Int32::ConstSharedPtr msg) { msg_ = msg; }

public:
  JoystickMuxTest()
  {
    node_ = std::make_shared<rclcpp::Node>("test_joystick_mux");
    pub1_ = node_->create_publisher<std_msgs::msg::Int32>("mux_input0", rclcpp::QoS(1));
    pub2_ = node_->create_publisher<std_msgs::msg::Int32>("mux_input1", rclcpp::QoS(1));
    pub_joy_ = node_->create_publisher<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(1));
    sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "mux_output", rclcpp::QoS(1),
      std::bind(&JoystickMuxTest::cbMsg, this, std::placeholders::_1));

    rclcpp::WallRate wait(10.0);
    for (size_t i = 0; i < 100 && rclcpp::ok(); ++i) {
      wait.sleep();
      rclcpp::spin_some(node_);
      if (i > 5 && pub1_->get_subscription_count() > 0) break;
    }
  }
  // The ROS 2 joystick_mux discovers the input message type from the graph and
  // creates its generic subscription lazily (topic_tools::ShapeShifter
  // equivalent). Unlike the ROS 1 node whose subscription exists from
  // construction, an input message published before that discovery is dropped.
  // Keep publishing while waiting so the mux advertises mux_output.
  void waitPublisher()
  {
    rclcpp::WallRate wait(10.0);
    for (size_t i = 0; i < 100 && rclcpp::ok(); ++i) {
      publish1(0);
      publish2(0);
      wait.sleep();
      rclcpp::spin_some(node_);
      if (i > 5 && sub_->get_publisher_count() > 0) break;
    }
  }
  void publish1(const int32_t v)
  {
    std_msgs::msg::Int32 msg_out;
    msg_out.data = v;
    pub1_->publish(msg_out);
  }
  void publish2(const int32_t v)
  {
    std_msgs::msg::Int32 msg_out;
    msg_out.data = v;
    pub2_->publish(msg_out);
  }
  void publishJoy(const int button)
  {
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = node_->now();
    joy.buttons.resize(1);
    joy.buttons[0] = button;
    pub_joy_->publish(joy);
  }
};

TEST_F(JoystickMuxTest, Interrupt)
{
  publish1(0);
  publish2(0);
  waitPublisher();
  for (int btn = 0; btn < 2; ++btn) {
    publishJoy(btn);
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::WallRate rate(20.0);
    for (int i = 0; i < 15; ++i) {
      publishJoy(btn);
      publish1(i);
      publish2(-i);

      rate.sleep();
      rclcpp::spin_some(node_);

      if (i < 5) continue;

      ASSERT_TRUE(static_cast<bool>(msg_)) << "button: " << btn;
      if (btn) {
        ASSERT_NEAR(-i, msg_->data, 2) << "button: " << btn;
      } else {
        ASSERT_NEAR(i, msg_->data, 2) << "button:" << btn;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
