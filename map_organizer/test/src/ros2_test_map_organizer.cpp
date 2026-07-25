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

// Integration test mirroring the ROS 1 map_organizer_rostest.test. It is driven
// by test_map_organizer_launch.py, which starts tie_maps (loads the maps from
// test/data), save_maps (re-serialises them to a temporary prefix), a second
// tie_maps in the "saved" namespace (reloads the saved files) and select_map.

#include <gtest/gtest.h>

#include <memory>

#include "map_organizer_msgs/msg/occupancy_grid_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace
{
// Latched (transient_local) subscription QoS so late-joining subscribers pick
// up the maps published once at node startup.
rclcpp::QoS latchedQos() { return rclcpp::QoS(1).transient_local(); }

void validateMap0(const nav_msgs::msg::OccupancyGrid & map, const double z)
{
  ASSERT_EQ("map_ground", map.header.frame_id);
  ASSERT_EQ(2u, map.info.width);
  ASSERT_EQ(4u, map.data.size());
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.y);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.z);
  ASSERT_FLOAT_EQ(1.0, map.info.origin.orientation.w);
  ASSERT_FLOAT_EQ(0.1, map.info.resolution);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.position.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.position.y);
  ASSERT_FLOAT_EQ(z, map.info.origin.position.z);
  ASSERT_EQ(100, map.data[0]);
  ASSERT_EQ(0, map.data[1]);
  ASSERT_EQ(100, map.data[2]);
  ASSERT_EQ(100, map.data[3]);
}
void validateMap1(const nav_msgs::msg::OccupancyGrid & map, const double z)
{
  ASSERT_EQ("map_ground", map.header.frame_id);
  ASSERT_EQ(2u, map.info.width);
  ASSERT_EQ(4u, map.data.size());
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.y);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.z);
  ASSERT_FLOAT_EQ(1.0, map.info.origin.orientation.w);
  ASSERT_FLOAT_EQ(0.2, map.info.resolution);
  ASSERT_FLOAT_EQ(0.1, map.info.origin.position.x);
  ASSERT_FLOAT_EQ(0.1, map.info.origin.position.y);
  ASSERT_FLOAT_EQ(z, map.info.origin.position.z);
  ASSERT_EQ(100, map.data[0]);
  ASSERT_EQ(0, map.data[1]);
  ASSERT_EQ(0, map.data[2]);
  ASSERT_EQ(100, map.data[3]);
}
}  // namespace

TEST(MapOrganizer, MapArray)
{
  auto node = std::make_shared<rclcpp::Node>("test_map_organizer_map_array");

  map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr maps;
  auto sub = node->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>(
    "maps", latchedQos(),
    [&maps](const map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr msg) { maps = msg; });

  rclcpp::WallRate rate(10.0);
  for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
    if (maps) break;
  }
  ASSERT_TRUE(static_cast<bool>(maps));
  ASSERT_EQ(2u, maps->maps.size());

  validateMap0(maps->maps[0], 1.0);
  validateMap1(maps->maps[1], 10.0);
}

TEST(MapOrganizer, Maps)
{
  auto node = std::make_shared<rclcpp::Node>("test_map_organizer_maps");

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map[2];
  auto sub0 = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map0", latchedQos(),
    [&map](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map[0] = msg; });
  auto sub1 = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map1", latchedQos(),
    [&map](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map[1] = msg; });

  rclcpp::WallRate rate(10.0);
  for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
    if (map[0] && map[1]) break;
  }
  for (int i = 0; i < 2; ++i) {
    ASSERT_TRUE(static_cast<bool>(map[i]));
  }
  validateMap0(*(map[0]), 1.0);
  validateMap1(*(map[1]), 10.0);
}

TEST(MapOrganizer, SelectMap)
{
  auto node = std::make_shared<rclcpp::Node>("test_map_organizer_select_map");

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", latchedQos(),
    [&map](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map = msg; });
  auto pub = node->create_publisher<std_msgs::msg::Int32>("floor", 1);

  rclcpp::WallRate rate(10.0);
  for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
    if (node->count_subscribers("floor") > 0 && map) break;
  }
  ASSERT_GT(node->count_subscribers("floor"), 0u);
  ASSERT_TRUE(static_cast<bool>(map));
  validateMap0(*map, 0.0);

  std_msgs::msg::Int32 floor;
  floor.data = 2;  // invalid floor must be ignored
  pub->publish(floor);

  map = nullptr;
  for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
    if (map) break;
  }
  ASSERT_FALSE(static_cast<bool>(map));

  floor.data = 1;
  pub->publish(floor);

  map = nullptr;
  for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
    if (map) break;
  }
  ASSERT_TRUE(static_cast<bool>(map));
  validateMap1(*map, 0.0);
}

TEST(MapOrganizer, SavedMapArray)
{
  auto node = std::make_shared<rclcpp::Node>("test_map_organizer_saved");

  map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr maps;
  auto sub = node->create_subscription<map_organizer_msgs::msg::OccupancyGridArray>(
    "saved/maps", latchedQos(),
    [&maps](const map_organizer_msgs::msg::OccupancyGridArray::ConstSharedPtr msg) { maps = msg; });

  rclcpp::WallRate rate(10.0);
  for (int i = 0; i < 200 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
    if (maps) break;
  }
  ASSERT_TRUE(static_cast<bool>(maps));
  ASSERT_EQ(2u, maps->maps.size());

  validateMap0(maps->maps[0], 1.0);
  validateMap1(maps->maps[1], 10.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
