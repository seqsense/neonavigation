/*
 * Copyright (c) 2025, the neonavigation authors
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

// Test-only replacement for the ROS 1 `map_server` node used by the rostests
// of this package.
//
// nav2_map_server's map_server is a lifecycle node which does not activate on
// its own, so using it from launch_testing would mean driving the lifecycle
// transitions from every test description and racing the nodes which have to
// see the map before they start. Reusing nav2_map_server's map_io library from
// a plain node keeps the map format (the very same YAML/PGM files the ROS 1
// tests use) while behaving exactly like the ROS 1 latched publisher.

#include <chrono>
#include <memory>
#include <string>

#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("map_publisher");

  const std::string yaml_filename = node->declare_parameter("yaml_filename", std::string(""));
  const std::string topic_name = node->declare_parameter("topic_name", std::string("map"));
  const std::string frame_id = node->declare_parameter("frame_id", std::string("map"));

  nav_msgs::msg::OccupancyGrid map;
  if (nav2_map_server::loadMapFromYaml(yaml_filename, map) != nav2_map_server::LOAD_MAP_SUCCESS) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load map: %s", yaml_filename.c_str());
    rclcpp::shutdown();
    return 1;
  }
  map.header.frame_id = frame_id;

  // The ROS 1 map_server latched the map; transient_local is the ROS 2
  // equivalent and matches the subscriptions of costmap_cspace/planner_cspace.
  auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name, rclcpp::QoS(1).transient_local());
  map.header.stamp = node->now();
  pub->publish(map);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
