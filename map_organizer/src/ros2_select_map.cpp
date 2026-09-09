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
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "map_organizer_msgs/msg/occupancy_grid_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace map_organizer
{
class SelectMapNode : public rclcpp::Node
{
public:
  explicit SelectMapNode(const rclcpp::NodeOptions & options);

private:
  void cbMaps(const map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr & msg);
  void cbFloor(const std_msgs::msg::Int32::ConstSharedPtr & msg);
  void cbTimer();

  rclcpp::Subscription<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr sub_maps_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_floor_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;

  map_organizer_msgs::msg::OccupancyGridArray maps_;
  std::vector<nav_msgs::msg::MapMetaData> orig_mapinfos_;
  int floor_cur_;
  int floor_prev_;
  geometry_msgs::msg::TransformStamped trans_;
};

SelectMapNode::SelectMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("select_map", options), floor_cur_(0), floor_prev_(-1)
{
  sub_maps_ = this->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>(
    "maps", rclcpp::QoS(1).transient_local(),
    std::bind(&SelectMapNode::cbMaps, this, std::placeholders::_1));
  sub_floor_ = this->create_subscription<std_msgs::msg::Int32>(
    "floor", 1, std::bind(&SelectMapNode::cbFloor, this, std::placeholders::_1));
  pub_map_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());

  tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  trans_.header.frame_id = "map_ground";
  trans_.child_frame_id = "map";
  trans_.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 0.0));

  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(0.1),
    std::bind(&SelectMapNode::cbTimer, this));
}

void SelectMapNode::cbMaps(const map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr & msg)
{
  RCLCPP_INFO(this->get_logger(), "Map array received");
  maps_ = *msg;
  orig_mapinfos_.clear();
  for (auto & map : maps_.maps) {
    orig_mapinfos_.push_back(map.info);
    map.info.origin.position.z = 0.0;
  }
}

void SelectMapNode::cbFloor(const std_msgs::msg::Int32::ConstSharedPtr & msg)
{
  floor_cur_ = msg->data;
}

void SelectMapNode::cbTimer()
{
  if (maps_.maps.size() == 0) {
    return;
  }

  if (floor_cur_ != floor_prev_) {
    if (floor_cur_ >= 0 && floor_cur_ < static_cast<int>(maps_.maps.size())) {
      auto map = std::make_unique<nav_msgs::msg::OccupancyGrid>(maps_.maps[floor_cur_]);
      pub_map_->publish(std::move(map));
      trans_.transform.translation.z = orig_mapinfos_[floor_cur_].origin.position.z;
    } else {
      RCLCPP_INFO(this->get_logger(), "Floor out of range");
    }
    floor_prev_ = floor_cur_;
  }
  trans_.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.15);
  tfb_->sendTransform(trans_);
}
}  // namespace map_organizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<map_organizer::SelectMapNode>(options));
  rclcpp::shutdown();
  return 0;
}
