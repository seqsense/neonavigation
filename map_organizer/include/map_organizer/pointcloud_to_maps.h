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

#ifndef MAP_ORGANIZER__POINTCLOUD_TO_MAPS_H_
#define MAP_ORGANIZER__POINTCLOUD_TO_MAPS_H_

#include "map_organizer_msgs/msg/occupancy_grid_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace map_organizer
{
// PointcloudToMaps holds the ROS-independent algorithm that slices a 3D point
// cloud into a stack of 2D occupancy grids (one per detected floor level).
// It intentionally depends only on message types, so it can be unit tested
// without a running ROS node.
class PointcloudToMaps
{
public:
  // Algorithm parameters. Lengths marked "[m]" are given in meters and are
  // converted to grid cells internally using grid.
  struct Config
  {
    double grid = 0.05;                   // grid resolution [m/cell]
    double points_thresh_rate = 0.5;      // ratio of the busiest layer used as
                                          // the per-layer point count threshold
    double robot_height = 1.0;            // [m]
    double floor_height = 0.1;            // [m]
    double floor_tolerance = 0.2;         // [m]
    double min_floor_area = 100.0;        // [m^2]
    double floor_area_thresh_rate = 0.8;  // ratio of the largest runnable area
  };

  explicit PointcloudToMaps(const rclcpp::Logger & logger) : logger_(logger) {}
  PointcloudToMaps(const Config & config, const rclcpp::Logger & logger)
  : config_(config), logger_(logger)
  {
  }

  void setConfig(const Config & config) { config_ = config; }
  const Config & config() const { return config_; }

  // Generate the layered occupancy grids from a point cloud. The header is
  // copied into every produced map. The returned array contains one map per
  // accepted floor, in ascending height order.
  map_organizer_msgs::msg::OccupancyGridArray generateMaps(
    const pcl::PointCloud<pcl::PointXYZ> & pc, const std_msgs::msg::Header & header) const;

private:
  Config config_;
  rclcpp::Logger logger_;
};
}  // namespace map_organizer

#endif  // MAP_ORGANIZER__POINTCLOUD_TO_MAPS_H_
