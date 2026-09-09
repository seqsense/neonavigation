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

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "map_organizer_msgs/msg/occupancy_grid_array.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace map_organizer
{
class TieMapNode : public rclcpp::Node
{
public:
  explicit TieMapNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<map_organizer_msgs::msg::OccupancyGridArray>::SharedPtr pub_map_array_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> pub_map_;
};

TieMapNode::TieMapNode(const rclcpp::NodeOptions & options) : rclcpp::Node("tie_maps", options)
{
  pub_map_array_ = this->create_publisher<map_organizer_msgs::msg::OccupancyGridArray>(
    "maps", rclcpp::QoS(1).transient_local());

  const std::string files_str = this->declare_parameter("map_files", std::string(""));
  const std::string frame_id = this->declare_parameter("frame_id", std::string("map"));

  map_organizer_msgs::msg::OccupancyGridArray maps;

  int i = 0;
  std::string file;
  std::stringstream ss(files_str);
  while (std::getline(ss, file, ',')) {
    // nav2_map_server parses the standard map YAML fields (resolution, origin,
    // negate, thresholds, mode, image path) and loads the image into an
    // OccupancyGrid.
    nav2_map_server::LoadParameters load_params;
    try {
      load_params = nav2_map_server::loadMapYaml(file);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map YAML %s: %s", file.c_str(), e.what());
      rclcpp::shutdown();
      return;
    }

    // The "height" field is a map_organizer extension and is not part of the
    // standard map metadata, so it is parsed separately here.
    double height = 0.0;
    try {
      std::ifstream fin(file);
      YAML::Node doc = YAML::Load(fin);
      if (doc["height"]) {
        height = doc["height"].as<double>();
      }
    } catch (const YAML::Exception &) {
      height = 0.0;
    }

    nav_msgs::msg::OccupancyGrid map;
    try {
      nav2_map_server::loadMapFromFile(load_params, map);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to load map image for %s: %s", file.c_str(), e.what());
      rclcpp::shutdown();
      return;
    }

    map.info.origin.position.z = height;
    map.info.map_load_time = this->now();
    map.header.frame_id = frame_id;
    map.header.stamp = this->now();
    RCLCPP_INFO(
      this->get_logger(), "Read a %d X %d map @ %.3lf m/cell", map.info.width, map.info.height,
      map.info.resolution);

    maps.maps.push_back(map);
    auto pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map" + std::to_string(i), rclcpp::QoS(1).transient_local());
    auto map_ptr = std::make_unique<nav_msgs::msg::OccupancyGrid>(map);
    pub->publish(std::move(map_ptr));
    pub_map_.push_back(pub);
    i++;
  }
  auto maps_ptr = std::make_unique<map_organizer_msgs::msg::OccupancyGridArray>(maps);
  pub_map_array_->publish(std::move(maps_ptr));
}
}  // namespace map_organizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<map_organizer::TieMapNode>(options));
  rclcpp::shutdown();
  return 0;
}
