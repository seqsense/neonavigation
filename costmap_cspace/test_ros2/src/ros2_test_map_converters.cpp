/*
 * Copyright (c) 2014-2018, the neonavigation authors
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

// ROS 2 integration test of the three map converter interface nodes
// (laserscan_to_map, pointcloud2_to_map and largemap_to_map), driven by
// test_map_converters_launch.py.
//
// These nodes had no ROS 1 rostest, so the test pins down their main pub/sub
// path after the port: an input scan / point cloud / large map plus the robot
// transform must produce a local occupancy grid centred on the robot with the
// obstacles at the expected cells.
//
// All three nodes are configured with a 1.0 m resolution and an 8x8 (4x4 for
// largemap_to_map) local map, and the launch file provides an identity
// map -> base_link transform, so the local map spans [-4, 4) m ([-2, 2) m)
// around the origin and the expected cell indices can be written down
// directly.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
constexpr double kResolution = 1.0;

rclcpp::QoS latchedQos(const size_t depth) { return rclcpp::QoS(depth).transient_local(); }

class MapConverterTestBase : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_local_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_local_;

  explicit MapConverterTestBase(const std::string & map_local_topic)
  {
    node_ = std::make_shared<rclcpp::Node>("test_map_converters");
    sub_map_local_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_local_topic, latchedQos(1),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map_local_ = msg; });
  }

  bool waitUntil(
    const std::function<bool()> & pred, const std::function<void()> & tick = nullptr,
    const double timeout_sec = 20.0)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node_);
      if (pred()) {
        return true;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      if (tick) {
        tick();
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  int8_t cellOf(const unsigned int x, const unsigned int y) const
  {
    return map_local_->data[x + y * map_local_->info.width];
  }

  // Geometry shared by laserscan_to_map and pointcloud2_to_map: an 8x8 grid of
  // 1.0 m cells centred on the robot standing at the origin of "map".
  void expectLocalMapGeometry(const unsigned int width) const
  {
    EXPECT_EQ("map", map_local_->header.frame_id);
    EXPECT_EQ(width, map_local_->info.width);
    EXPECT_EQ(width, map_local_->info.height);
    EXPECT_NEAR(kResolution, map_local_->info.resolution, 1e-6);
    EXPECT_NEAR(-0.5 * width * kResolution, map_local_->info.origin.position.x, 1e-6);
    EXPECT_NEAR(-0.5 * width * kResolution, map_local_->info.origin.position.y, 1e-6);
    EXPECT_EQ(width * width, map_local_->data.size());
  }
};

class LaserscanToMapTest : public MapConverterTestBase
{
protected:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;

  LaserscanToMapTest() : MapConverterTestBase("/laserscan_map_local")
  {
    pub_scan_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("/test_scan", 2);
  }

  void publishScan()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "base_link";
    scan.header.stamp = node_->now();
    scan.angle_min = -0.5;
    scan.angle_increment = 0.6;
    scan.angle_max = scan.angle_min + 2 * scan.angle_increment;
    scan.range_min = 0.1;
    scan.range_max = 10.0;
    scan.ranges.assign(3, 2.5);
    pub_scan_->publish(scan);
  }
};

class Pointcloud2ToMapTest : public MapConverterTestBase
{
protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;

  Pointcloud2ToMapTest() : MapConverterTestBase("/pointcloud_map_local")
  {
    pub_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/test_cloud", 2);
  }

  void publishCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "base_link";
    cloud.header.stamp = node_->now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(2);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    const float xs[2] = {2.5f, -1.4f};
    const float ys[2] = {-1.2f, 1.6f};
    for (size_t i = 0; i < 2; ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = xs[i];
      *iter_y = ys[i];
      *iter_z = 0.0f;
    }
    pub_cloud_->publish(cloud);
  }
};

class LargemapToMapTest : public MapConverterTestBase
{
protected:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_large_map_;

  LargemapToMapTest() : MapConverterTestBase("/largemap_map_local")
  {
    pub_large_map_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/large_map", latchedQos(2));
  }

  void publishLargeMap()
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = node_->now();
    map.info.resolution = kResolution;
    map.info.width = 20;
    map.info.height = 20;
    map.info.origin.position.x = -10.0;
    map.info.origin.position.y = -10.0;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(20 * 20, 0);
    map.data[10 + 11 * 20] = 100;
    pub_large_map_->publish(map);
  }
};

TEST_F(LaserscanToMapTest, ScanIsProjectedToLocalMap)
{
  // The three rays of the scan (range 2.5 m at -0.5, 0.1 and 0.7 rad) hit the
  // cells listed below once projected into "map" and shifted by the map origin
  // at (-4, -4).
  ASSERT_TRUE(waitUntil(
    [this] { return map_local_ && map_local_->info.width == 8 && cellOf(6, 4) == 100; },
    [this] { publishScan(); }));

  expectLocalMapGeometry(8);
  EXPECT_EQ(100, cellOf(6, 2));
  EXPECT_EQ(100, cellOf(6, 4));
  EXPECT_EQ(100, cellOf(5, 5));
  EXPECT_EQ(0, cellOf(0, 0));
  EXPECT_EQ(0, cellOf(4, 4));
}

TEST_F(Pointcloud2ToMapTest, CloudIsProjectedToLocalMap)
{
  ASSERT_TRUE(waitUntil(
    [this] { return map_local_ && map_local_->info.width == 8 && cellOf(6, 2) == 100; },
    [this] { publishCloud(); }));

  expectLocalMapGeometry(8);
  // (2.5, -1.2) and (-1.4, 1.6) in "map" with the origin at (-4, -4).
  EXPECT_EQ(100, cellOf(6, 2));
  EXPECT_EQ(100, cellOf(2, 5));
  EXPECT_EQ(0, cellOf(0, 0));
  EXPECT_EQ(0, cellOf(4, 4));
}

TEST_F(LargemapToMapTest, LargeMapIsCroppedAroundRobot)
{
  // The 4x4 local map starts at (-2, -2), i.e. at cell (8, 8) of the 20x20
  // large map whose origin is (-10, -10).
  ASSERT_TRUE(waitUntil(
    [this] { return map_local_ && map_local_->info.width == 4 && cellOf(2, 3) == 100; },
    [this] { publishLargeMap(); }));

  expectLocalMapGeometry(4);
  EXPECT_EQ(100, cellOf(2, 3));
  for (unsigned int y = 0; y < 4; ++y) {
    for (unsigned int x = 0; x < 4; ++x) {
      if (x == 2 && y == 3) {
        continue;
      }
      EXPECT_EQ(0, cellOf(x, y)) << "x: " << x << ", y: " << y;
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
