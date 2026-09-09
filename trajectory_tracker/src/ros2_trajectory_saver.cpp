/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
 */

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

namespace trajectory_tracker
{
class SaverNode : public rclcpp::Node
{
public:
  explicit SaverNode(const rclcpp::NodeOptions & options);

  bool saved() const { return saved_; }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

  std::string filename_;
  bool saved_;
  void cbPath(const nav_msgs::msg::Path::ConstSharedPtr & msg);
};

SaverNode::SaverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_saver", options), saved_(false)
{
  filename_ = this->declare_parameter("file", std::string("a.path"));

  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(10).transient_local(),
    std::bind(&SaverNode::cbPath, this, std::placeholders::_1));
}

void SaverNode::cbPath(const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
  if (saved_) return;
  std::ofstream ofs(filename_.c_str(), std::ios::binary);

  if (!ofs) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open %s", filename_.c_str());
    return;
  }

  rclcpp::Serialization<nav_msgs::msg::Path> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(msg.get(), &serialized);

  const auto & rcl_handle = serialized.get_rcl_serialized_message();
  RCLCPP_INFO(this->get_logger(), "Size: %d", static_cast<int>(rcl_handle.buffer_length));
  ofs.write(reinterpret_cast<const char *>(rcl_handle.buffer), rcl_handle.buffer_length);

  saved_ = true;
}
}  // namespace trajectory_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<trajectory_tracker::SaverNode>(options);
  RCLCPP_INFO(node->get_logger(), "Waiting for the path");
  rclcpp::Rate loop_rate(5);
  while (rclcpp::ok() && !node->saved()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  RCLCPP_INFO(node->get_logger(), "Path saved");
  rclcpp::shutdown();
  return 0;
}
