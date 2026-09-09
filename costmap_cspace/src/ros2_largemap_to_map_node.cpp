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

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace costmap_cspace
{
// ROS 2 port of the largemap_to_map node. The node is small and entirely made
// of ROS interface code, so it is kept as a direct ROS 2 counterpart of
// src/ros1_largemap_to_map.cpp instead of being split into a logic class.
class LargeMapToMapNode : public rclcpp::Node
{
public:
  explicit LargeMapToMapNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_largemap_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr large_map_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  std::string robot_frame_;

  int width_;
  bool round_local_map_;
  bool simulate_occlusion_;
  bool simulate_surrounded_;
  std::map<size_t, std::vector<size_t>> occlusion_table_;

  void cbTimer();
  void cbLargeMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void publishMap();
};

LargeMapToMapNode::LargeMapToMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("largemap_to_map", options)
{
  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

  robot_frame_ = this->declare_parameter("robot_frame", std::string("base_link"));

  // ROS 1 latched this publisher; transient_local is the ROS 2 equivalent.
  pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map_local", rclcpp::QoS(1).transient_local());
  sub_largemap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(2).transient_local(),
    std::bind(&LargeMapToMapNode::cbLargeMap, this, std::placeholders::_1));

  width_ = static_cast<int>(this->declare_parameter("width", 30));
  round_local_map_ = this->declare_parameter("round_local_map", false);
  simulate_occlusion_ = this->declare_parameter("simulate_occlusion", false);
  simulate_surrounded_ = this->declare_parameter("simulate_surrounded", false);

  for (size_t addr = 0; addr < static_cast<size_t>(width_ * width_); ++addr) {
    const int ux = addr % width_;
    const int uy = addr / width_;
    const float x = ux - width_ / 2.0;
    const float y = uy - width_ / 2.0;
    const float l = std::sqrt(x * x + y * y);
    for (float r = 1.0; r < l - 1.0; r += 0.5) {
      const float x2 = x * r / l;
      const float y2 = y * r / l;
      const int ux2 = x2 + width_ / 2.0;
      const int uy2 = y2 + width_ / 2.0;
      const int addr2 = uy2 * width_ + ux2;
      occlusion_table_[addr2].push_back(addr);
    }
  }
  for (auto & cell : occlusion_table_) {
    std::sort(cell.second.begin(), cell.second.end());
    cell.second.erase(std::unique(cell.second.begin(), cell.second.end()), cell.second.end());
    const auto self_it = std::find(cell.second.begin(), cell.second.end(), cell.first);
    if (self_it != cell.second.end()) {
      cell.second.erase(self_it);
    }
  }

  const double hz = this->declare_parameter("hz", 1.0);
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / hz),
    std::bind(&LargeMapToMapNode::cbTimer, this));
}

void LargeMapToMapNode::cbTimer() { publishMap(); }

void LargeMapToMapNode::cbLargeMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  large_map_ = msg;
}

void LargeMapToMapNode::publishMap()
{
  if (!large_map_) {
    return;
  }
  tf2::Stamped<tf2::Transform> trans;
  try {
    tf2::fromMsg(
      tfbuf_->lookupTransform(large_map_->header.frame_id, robot_frame_, tf2::TimePointZero),
      trans);
  } catch (const tf2::TransformException &) {
    return;
  }

  auto map = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  map->header.frame_id = large_map_->header.frame_id;
  map->header.stamp = this->now();
  map->info = large_map_->info;
  map->info.width = width_;
  map->info.height = width_;

  const auto pos = trans.getOrigin();
  const float x = static_cast<int>(pos.x() / map->info.resolution) * map->info.resolution;
  const float y = static_cast<int>(pos.y() / map->info.resolution) * map->info.resolution;
  map->info.origin.position.x = x - map->info.width * map->info.resolution * 0.5;
  map->info.origin.position.y = y - map->info.height * map->info.resolution * 0.5;
  map->info.origin.position.z = 0.0;
  map->info.origin.orientation.w = 1.0;
  map->data.resize(width_ * width_);

  const int gx = std::lround(
    (map->info.origin.position.x - large_map_->info.origin.position.x) / map->info.resolution);
  const int gy = std::lround(
    (map->info.origin.position.y - large_map_->info.origin.position.y) / map->info.resolution);
  const float half_width = width_ / 2.0;

  for (int iy = gy; iy < gy + width_; ++iy) {
    for (int ix = gx; ix < gx + width_; ++ix) {
      const int lx = ix - gx;
      const int ly = iy - gy;
      const size_t addr = ly * width_ + lx;
      const size_t addr_large = iy * large_map_->info.width + ix;
      const float r_sq = std::pow(lx - half_width, 2) + std::pow(ly - half_width, 2);
      if (
        simulate_surrounded_ && r_sq <= std::pow(half_width, 2) &&
        std::pow(half_width - 2, 2) <= r_sq) {
        map->data[addr] = 100;
      } else if (round_local_map_ && r_sq > std::pow(half_width, 2)) {
        map->data[addr] = -1;
      } else if (
        ix < 0 || iy < 0 || ix >= static_cast<int>(large_map_->info.width) ||
        iy >= static_cast<int>(large_map_->info.height)) {
        map->data[addr] = -1;
      } else {
        map->data[addr] = large_map_->data[addr_large];
      }
    }
  }
  if (simulate_occlusion_) {
    for (size_t addr = 0; addr < static_cast<size_t>(width_ * width_); ++addr) {
      if (map->data[addr] == 100) {
        for (auto a : occlusion_table_[addr]) {
          map->data[a] = -1;
        }
      }
    }
  }

  pub_map_->publish(std::move(map));
}
}  // namespace costmap_cspace

RCLCPP_COMPONENTS_REGISTER_NODE(costmap_cspace::LargeMapToMapNode)
