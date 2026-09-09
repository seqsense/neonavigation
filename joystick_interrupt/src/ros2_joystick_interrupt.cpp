/*
 * Copyright (c) 2015-2018, the neonavigation authors
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
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

namespace joystick_interrupt
{
class JoystickInterrupt : public rclcpp::Node
{
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_int_;
  double linear_vel_;
  double angular_vel_;
  double linear_y_vel_;
  double timeout_;
  double linear_high_speed_ratio_;
  double angular_high_speed_ratio_;
  int linear_axis_;
  int angular_axis_;
  int linear_axis2_;
  int angular_axis2_;
  int linear_y_axis_;
  int linear_y_axis2_;
  int interrupt_button_;
  int high_speed_button_;
  bool use_sim_time_;
  rclcpp::Time last_joy_msg_;
  geometry_msgs::msg::Twist last_input_twist_;

  float getAxisValue(
    const sensor_msgs::msg::Joy::ConstSharedPtr & msg, const int axis,
    const std::string & axis_name) const
  {
    if (axis < 0) {
      return 0.0;
    }
    if (static_cast<size_t>(axis) >= msg->axes.size()) {
      RCLCPP_ERROR(
        this->get_logger(), "Out of range: number of axis (%lu) must be greater than %s (%d).",
        msg->axes.size(), axis_name.c_str(), axis);
      return 0.0;
    }
    return msg->axes[axis];
  }

  float getJoyValue(
    const sensor_msgs::msg::Joy::ConstSharedPtr & msg, const int axis, const int axis2,
    const std::string & axis_name) const
  {
    const float value = getAxisValue(msg, axis, axis_name);
    const float value2 = getAxisValue(msg, axis2, axis_name + "2");
    return (std::abs(value2) > std::abs(value)) ? value2 : value;
  }

  void cbJoy(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    if (static_cast<size_t>(interrupt_button_) >= msg->buttons.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Out of range: number of buttons (%lu) must be greater than interrupt_button (%d).",
        msg->buttons.size(), interrupt_button_);
      last_joy_msg_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }
    if (!msg->buttons[interrupt_button_]) {
      if (last_joy_msg_ != rclcpp::Time(0, 0, RCL_ROS_TIME)) {
        auto out = std::make_unique<geometry_msgs::msg::Twist>(last_input_twist_);
        pub_twist_->publish(std::move(out));
      }
      last_joy_msg_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }

    last_joy_msg_ = this->now();

    float lin_x = getJoyValue(msg, linear_axis_, linear_axis2_, "linear_axis");
    float lin_y = getJoyValue(msg, linear_y_axis_, linear_y_axis2_, "linear_y_axis");
    float ang = getJoyValue(msg, angular_axis_, angular_axis2_, "angular_axis");

    if (high_speed_button_ >= 0) {
      if (static_cast<size_t>(high_speed_button_) < msg->buttons.size()) {
        if (msg->buttons[high_speed_button_]) {
          lin_x *= linear_high_speed_ratio_;
          lin_y *= linear_high_speed_ratio_;
          ang *= angular_high_speed_ratio_;
        }
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Out of range: number of buttons (%lu) must be greater than high_speed_button (%d).",
          msg->buttons.size(), high_speed_button_);
      }
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = lin_x * linear_vel_;
    cmd_vel->linear.y = lin_y * linear_y_vel_;
    cmd_vel->linear.z = 0.0;
    cmd_vel->angular.z = ang * angular_vel_;
    cmd_vel->angular.x = cmd_vel->angular.y = 0.0;
    pub_twist_->publish(std::move(cmd_vel));
  }
  void cbTwist(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
  {
    last_input_twist_ = *msg;
    auto status = std::make_unique<std_msgs::msg::Bool>();
    if (
      this->now() - last_joy_msg_ > rclcpp::Duration::from_seconds(timeout_) ||
      (use_sim_time_ && last_joy_msg_ == rclcpp::Time(0, 0, RCL_ROS_TIME))) {
      auto out = std::make_unique<geometry_msgs::msg::Twist>(last_input_twist_);
      pub_twist_->publish(std::move(out));
      status->data = true;
    } else {
      status->data = false;
    }
    pub_int_->publish(std::move(status));
  }

public:
  explicit JoystickInterrupt(const rclcpp::NodeOptions & options)
  : rclcpp::Node("joystick_interrupt", options), last_joy_msg_(0, 0, RCL_ROS_TIME)
  {
    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::QoS(1), std::bind(&JoystickInterrupt::cbJoy, this, std::placeholders::_1));
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_input", rclcpp::QoS(1),
      std::bind(&JoystickInterrupt::cbTwist, this, std::placeholders::_1));
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(2));
    pub_int_ = this->create_publisher<std_msgs::msg::Bool>("~/interrupt_status", rclcpp::QoS(2));

    linear_vel_ = this->declare_parameter("linear_vel", 0.5);
    angular_vel_ = this->declare_parameter("angular_vel", 0.8);
    linear_axis_ = this->declare_parameter("linear_axis", 1);
    angular_axis_ = this->declare_parameter("angular_axis", 0);
    linear_axis2_ = this->declare_parameter("linear_axis2", -1);
    angular_axis2_ = this->declare_parameter("angular_axis2", -1);
    interrupt_button_ = this->declare_parameter("interrupt_button", 6);
    high_speed_button_ = this->declare_parameter("high_speed_button", -1);
    linear_high_speed_ratio_ = this->declare_parameter("linear_high_speed_ratio", 1.3);
    angular_high_speed_ratio_ = this->declare_parameter("angular_high_speed_ratio", 1.1);
    timeout_ = this->declare_parameter("timeout", 0.5);
    linear_y_vel_ = this->declare_parameter("linear_y_vel", 0.0);
    linear_y_axis_ = this->declare_parameter("linear_y_axis", -1);
    linear_y_axis2_ = this->declare_parameter("linear_y_axis2", -1);

    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();

    last_joy_msg_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    if (interrupt_button_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "interrupt_button must be grater than -1.");
      throw std::runtime_error("interrupt_button must be grater than -1.");
    }
    if (linear_axis_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "linear_axis must be grater than -1.");
      throw std::runtime_error("linear_axis must be grater than -1.");
    }
    if (angular_axis_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "angular_axis must be grater than -1.");
      throw std::runtime_error("angular_axis must be grater than -1.");
    }
  }
};
}  // namespace joystick_interrupt

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_interrupt::JoystickInterrupt)
