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

#include <map>
#include <memory>
#include <string>

#include "map_organizer/pointcloud_to_maps.h"
#include "map_organizer_msgs/msg/occupancy_grid_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace map_organizer
{
class PointcloudToMapsNode : public rclcpp::Node
{
public:
  explicit PointcloudToMapsNode(const rclcpp::NodeOptions & options);

private:
  void cbPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  // Latched publishers are declared with transient_local durability so that
  // late-joining subscribers receive the most recently produced maps (the
  // ROS 1 node advertised these topics with latch=true).
  rclcpp::Publisher<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr pub_map_array_;
  std::map<std::string, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pub_maps_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
};

PointcloudToMapsNode::PointcloudToMapsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_maps", options)
{
  this->declare_parameter("grid", 0.05);
  this->declare_parameter("points_thresh_rate", 0.5);
  this->declare_parameter("robot_height", 1.0);
  this->declare_parameter("floor_height", 0.1);
  this->declare_parameter("floor_tolerance", 0.2);
  this->declare_parameter("min_floor_area", 100.0);
  this->declare_parameter("floor_area_thresh_rate", 0.8);

  pub_map_array_ = this->create_publisher<map_organizer_msgs::msg::OccupancyGridArray>(
    "maps", rclcpp::QoS(1).transient_local());
  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mapcloud", 1, std::bind(&PointcloudToMapsNode::cbPoints, this, std::placeholders::_1));
}

void PointcloudToMapsNode::cbPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *pc);

  map_organizer::PointcloudToMaps::Config config;
  config.grid = this->get_parameter("grid").as_double();
  config.points_thresh_rate = this->get_parameter("points_thresh_rate").as_double();
  config.robot_height = this->get_parameter("robot_height").as_double();
  config.floor_height = this->get_parameter("floor_height").as_double();
  config.floor_tolerance = this->get_parameter("floor_tolerance").as_double();
  config.min_floor_area = this->get_parameter("min_floor_area").as_double();
  config.floor_area_thresh_rate = this->get_parameter("floor_area_thresh_rate").as_double();

  map_organizer::PointcloudToMaps p2m(config, this->get_logger());
  auto map_array = std::make_unique<map_organizer_msgs::msg::OccupancyGridArray>(
    p2m.generateMaps(*pc, msg->header));

  for (size_t i = 0; i < map_array->maps.size(); ++i) {
    const std::string name = "~/map" + std::to_string(i);
    auto it = pub_maps_.find(name);
    if (it == pub_maps_.end()) {
      it = pub_maps_
             .emplace(
               name, this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                       name, rclcpp::QoS(1).transient_local()))
             .first;
    }
    auto map = std::make_unique<nav_msgs::msg::OccupancyGrid>(map_array->maps[i]);
    it->second->publish(std::move(map));
  }
  pub_map_array_->publish(std::move(map_array));
}
}  // namespace map_organizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(map_organizer::PointcloudToMapsNode)
