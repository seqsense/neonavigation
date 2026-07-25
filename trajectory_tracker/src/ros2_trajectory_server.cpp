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

#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "trajectory_tracker/filter.h"
#include "trajectory_tracker_msgs/msg/trajectory_server_status.hpp"
#include "trajectory_tracker_msgs/srv/change_path.hpp"

namespace trajectory_tracker
{
class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<trajectory_tracker_msgs::msg::TrajectoryServerStatus>::SharedPtr pub_status_;
  rclcpp::Service<trajectory_tracker_msgs::srv::ChangePath>::SharedPtr srv_change_path_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;
  std::string filename_;
  int32_t path_id_;
  double hz_;
  double filter_step_;
  std::vector<uint8_t> buffer_;

  bool loadFile();
  void change(
    const std::shared_ptr<trajectory_tracker_msgs::srv::ChangePath::Request> req,
    std::shared_ptr<trajectory_tracker_msgs::srv::ChangePath::Response> res);
  void cbTimer();
};

ServerNode::ServerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_server", options), path_id_(0)
{
  filename_ = this->declare_parameter("file", std::string("a.path"));
  hz_ = this->declare_parameter("hz", 5.0);
  filter_step_ = this->declare_parameter("filter_step", 0.0);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(2).transient_local());
  pub_status_ =
    this->create_publisher<trajectory_tracker_msgs::msg::TrajectoryServerStatus>("~/status", 2);
  srv_change_path_ = this->create_service<trajectory_tracker_msgs::srv::ChangePath>(
    "ChangePath",
    std::bind(&ServerNode::change, this, std::placeholders::_1, std::placeholders::_2));

  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / hz_),
    std::bind(&ServerNode::cbTimer, this));
}

bool ServerNode::loadFile()
{
  std::ifstream ifs(filename_.c_str(), std::ios::binary);
  if (ifs.good()) {
    ifs.seekg(0, ifs.end);
    const std::streamsize serial_size = ifs.tellg();
    ifs.seekg(0, ifs.beg);
    buffer_.resize(serial_size);
    ifs.read(reinterpret_cast<char *>(buffer_.data()), serial_size);
    return true;
  }
  return false;
}

void ServerNode::change(
  const std::shared_ptr<trajectory_tracker_msgs::srv::ChangePath::Request> req,
  std::shared_ptr<trajectory_tracker_msgs::srv::ChangePath::Response> res)
{
  filename_ = req->filename;
  path_id_ = req->id;
  res->success = false;

  if (loadFile()) {
    res->success = true;
    rclcpp::SerializedMessage serialized(buffer_.size());
    auto & rcl_handle = serialized.get_rcl_serialized_message();
    std::memcpy(rcl_handle.buffer, buffer_.data(), buffer_.size());
    rcl_handle.buffer_length = buffer_.size();
    rclcpp::Serialization<nav_msgs::msg::Path> serializer;
    serializer.deserialize_message(&serialized, &path_);
    path_.header.stamp = this->now();
    if (filter_step_ > 0 && path_.poses.size() > 0) {
      trajectory_tracker::Filter lpf_x(
        trajectory_tracker::Filter::FILTER_LPF, filter_step_, path_.poses[0].pose.position.x);
      trajectory_tracker::Filter lpf_y(
        trajectory_tracker::Filter::FILTER_LPF, filter_step_, path_.poses[0].pose.position.y);
      for (size_t i = 0; i < path_.poses.size(); i++) {
        path_.poses[i].pose.position.x = lpf_x.in(path_.poses[i].pose.position.x);
        path_.poses[i].pose.position.y = lpf_y.in(path_.poses[i].pose.position.y);
      }
    }
    pub_path_->publish(path_);
  } else {
    filename_ = "";
    path_.poses.clear();
    path_.header.frame_id = "map";
  }
}

void ServerNode::cbTimer()
{
  trajectory_tracker_msgs::msg::TrajectoryServerStatus status;
  status.header = path_.header;
  status.filename = filename_;
  status.id = path_id_;
  pub_status_->publish(status);
}
}  // namespace trajectory_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<trajectory_tracker::ServerNode>(options));
  rclcpp::shutdown();
  return 0;
}
