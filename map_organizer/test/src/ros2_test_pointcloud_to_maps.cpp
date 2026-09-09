/*
 * Copyright (c) 2019, the neonavigation authors
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

// Pure unit test for the ROS-independent PointcloudToMaps::generateMaps logic.
//
// The ROS 1 pointcloud_to_maps_rostest.test drove the same algorithm through a
// running node (publish a PointCloud2 on "mapcloud", subscribe to "maps"). Since
// the algorithm is now hybridized into pointcloud_to_maps_core, this test calls
// generateMaps() directly with the same synthetic cloud and node parameters,
// which exercises identical numeric behaviour without launch/timing overhead.

#include <gtest/gtest.h>

#include <vector>

#include "map_organizer/pointcloud_to_maps.h"
#include "map_organizer_msgs/msg/occupancy_grid_array.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

pcl::PointCloud<pcl::PointXYZ> generateMapCloud()
{
  pcl::PointCloud<pcl::PointXYZ> pc;
  for (float x = 0.025; x < 1.0; x += 0.05) {
    for (float y = 0.025; y < 1.0; y += 0.05) {
      pc.push_back(pcl::PointXYZ(x, y, 0.0));
    }
  }
  for (float z = 0.05; z < 0.5; z += 0.05) {
    pc.push_back(pcl::PointXYZ(0.425, 0.425, z));
    pc.push_back(pcl::PointXYZ(0.575, 0.425, z));
    pc.push_back(pcl::PointXYZ(0.425, 0.575, z));
    pc.push_back(pcl::PointXYZ(0.575, 0.575, z));
  }
  for (float x = 0.425; x < 0.6; x += 0.05) {
    for (float y = 0.425; y < 0.6; y += 0.05) {
      pc.push_back(pcl::PointXYZ(x, y, 0.5));
    }
  }
  for (float x = 0.225; x < 0.8; x += 0.05) {
    for (float y = 0.225; y < 0.8; y += 0.05) {
      pc.push_back(pcl::PointXYZ(x, y, 2.0));
    }
  }
  for (float x = 0.225; x < 0.8; x += 0.05) {
    pc.push_back(pcl::PointXYZ(x, 0.225, 2.5));
    pc.push_back(pcl::PointXYZ(x, 0.775, 2.5));
    pc.push_back(pcl::PointXYZ(0.225, x, 2.5));
    pc.push_back(pcl::PointXYZ(0.775, x, 2.5));
  }
  return pc;
}

TEST(PointcloudToMaps, Convert)
{
  // Same parameters as pointcloud_to_maps_rostest.test.
  map_organizer::PointcloudToMaps::Config config;
  config.min_floor_area = 0.2;
  config.points_thresh_rate = 0.0;
  config.floor_area_thresh_rate = 0.0;

  map_organizer::PointcloudToMaps p2m(config, rclcpp::get_logger("test_pointcloud_to_maps"));

  std_msgs::msg::Header header;
  header.frame_id = "map";
  const map_organizer_msgs::msg::OccupancyGridArray maps =
    p2m.generateMaps(generateMapCloud(), header);

  ASSERT_EQ(2u, maps.maps.size());
  for (int i = 0; i < 2; ++i) {
    ASSERT_EQ("map", maps.maps[i].header.frame_id);
    ASSERT_EQ(20u, maps.maps[i].info.width);
    ASSERT_EQ(20u, maps.maps[i].info.height);
  }
  ASSERT_NEAR(0.0, maps.maps[0].info.origin.position.z, 0.05);
  for (int u = 0; u < 20; ++u) {
    for (int v = 0; v < 20; ++v) {
      if (8 <= u && u < 12 && 8 <= v && v < 12) {
        ASSERT_EQ(100, maps.maps[0].data[v * 20 + u]) << u << ", " << v;
      } else {
        ASSERT_EQ(0, maps.maps[0].data[v * 20 + u]) << u << ", " << v;
      }
    }
  }
  ASSERT_NEAR(2.0, maps.maps[1].info.origin.position.z, 0.05);
  for (int u = 0; u < 20; ++u) {
    for (int v = 0; v < 20; ++v) {
      if (4 <= u && u < 16 && 4 <= v && v < 16) {
        if (4 == u || u == 15 || 4 == v || v == 15) {
          ASSERT_EQ(100, maps.maps[1].data[v * 20 + u]) << u << ", " << v;
        } else {
          ASSERT_EQ(0, maps.maps[1].data[v * 20 + u]) << u << ", " << v;
        }
      } else {
        ASSERT_EQ(-1, maps.maps[1].data[v * 20 + u]) << u << ", " << v;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
