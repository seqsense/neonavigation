/*
 * Copyright (c) 2015-2020, the neonavigation authors
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
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joystick_interrupt
{
// ROS 2 port of the ROS 1 topic_tools::ShapeShifter based joystick_mux.
//
// The ROS 1 node used topic_tools::ShapeShifter to relay an input topic of an
// arbitrary message type. ROS 2 provides rclcpp::GenericSubscription /
// rclcpp::GenericPublisher for the same "any type" behaviour, but both require
// the concrete message type name at creation time. The ShapeShifter learns the
// type lazily from the first received message; the generic API cannot, so the
// message type is instead discovered from the graph (get_publishers_info_by_
// topic) inside the periodic timer and the generic subscription / publisher are
// created once the type becomes known.
class JoystickMux : public rclcpp::Node
{
private:
  static constexpr size_t kNumInputs = 2;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> sub_topics_;
  std::vector<std::string> input_topics_;
  std::vector<std::string> input_types_;
  rclcpp::GenericPublisher::SharedPtr pub_topic_;
  std::string output_type_;
  rclcpp::TimerBase::SharedPtr timer_;
  double timeout_;
  int interrupt_button_;
  rclcpp::Time last_joy_msg_;
  bool advertised_;
  int selected_;

  void cbJoy(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    if (static_cast<size_t>(interrupt_button_) >= msg->buttons.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Out of range: number of buttons (%lu) must be greater than interrupt_button (%d).",
        msg->buttons.size(), interrupt_button_);
      return;
    }

    last_joy_msg_ = this->now();
    if (msg->buttons[interrupt_button_]) {
      selected_ = 1;
    } else {
      selected_ = 0;
    }
  }
  void cbTopic(const std::shared_ptr<const rclcpp::SerializedMessage> & msg, int id)
  {
    if (selected_ == id) {
      if (!advertised_) {
        advertised_ = true;
        output_type_ = input_types_[id];
        pub_topic_ = this->create_generic_publisher("mux_output", output_type_, rclcpp::QoS(1));
      }
      pub_topic_->publish(*msg);
    }
  }
  void cbTimer()
  {
    // Lazily create the generic subscriptions once the publisher type of each
    // input topic is available on the graph (ShapeShifter type-discovery
    // equivalent).
    for (size_t i = 0; i < kNumInputs; ++i) {
      if (sub_topics_[i]) {
        continue;
      }
      const auto infos = this->get_publishers_info_by_topic(input_topics_[i]);
      if (infos.empty()) {
        continue;
      }
      input_types_[i] = infos.front().topic_type();
      const int id = static_cast<int>(i);
      sub_topics_[i] = this->create_generic_subscription(
        input_topics_[i], input_types_[i], rclcpp::QoS(1),
        [this, id](const std::shared_ptr<const rclcpp::SerializedMessage> & msg) {
          cbTopic(msg, id);
        });
      RCLCPP_INFO(
        this->get_logger(), "Subscribed to mux input %d (%s) as %s", id, input_topics_[i].c_str(),
        input_types_[i].c_str());
    }

    if (this->now() - last_joy_msg_ > rclcpp::Duration::from_seconds(timeout_)) {
      selected_ = 0;
    }
  }

public:
  explicit JoystickMux(const rclcpp::NodeOptions & options)
  : rclcpp::Node("joystick_mux", options),
    input_topics_{"mux_input0", "mux_input1"},
    input_types_(kNumInputs),
    last_joy_msg_(0, 0, RCL_ROS_TIME),
    advertised_(false),
    selected_(0)
  {
    sub_topics_.resize(kNumInputs);

    sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::QoS(1), std::bind(&JoystickMux::cbJoy, this, std::placeholders::_1));

    interrupt_button_ = this->declare_parameter("interrupt_button", 5);
    timeout_ = this->declare_parameter("timeout", 0.5);
    last_joy_msg_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(0.1), std::bind(&JoystickMux::cbTimer, this));
  }
};
}  // namespace joystick_interrupt

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_interrupt::JoystickMux)
