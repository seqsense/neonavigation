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

// ROS 2 integration test of the costmap_3d interface node, driven by
// test_costmap_3d_launch.py.
//
// The ROS 1 node read the layer chain from a nested XmlRpc structure while the
// ROS 2 node reads it from a flat string array plus "layer.<name>.<key>" /
// "static_layer.<name>.<key>" parameters. There was no rostest covering the
// node, so this test pins down that the flattened representation builds the
// very same layer tree as the ROS 1 one:
//
//   * the root footprint comes from the flat [x0, y0, x1, y1, ...] array
//     (checked through the published ~/footprint polygon),
//   * every entry of "layers" becomes an overlay layer subscribing a topic
//     named after it and feeding /costmap_update,
//   * every entry of "static_layers" becomes a layer in front of the static
//     output, so its overlay maps re-publish /costmap,
//   * omitting "layers" keeps the single-layer backward compatible mode with
//     its "map_overlay" subscription and "overlay_mode" parameter.
//
// The launch file configures a 1.0 m resolution map and footprints smaller
// than half a cell with no expansion/spread, so the resulting C-space is
// identical to the input occupancy grid and the expected costs can be written
// down cell by cell.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr unsigned int kWidth = 8;
constexpr unsigned int kHeight = 8;
constexpr unsigned int kAngle = 4;
constexpr double kResolution = 1.0;
// Half width of the square footprint given to every layer by the launch file.
constexpr double kFootprintHalfWidth = 0.4;

// Occupancy grid matching the geometry the costmap nodes are configured with,
// with a single occupied cell so that each publication can be told apart.
nav_msgs::msg::OccupancyGrid makeGrid(const unsigned int x, const unsigned int y)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = kResolution;
  grid.info.width = kWidth;
  grid.info.height = kHeight;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(kWidth * kHeight, 0);
  grid.data[x + y * kWidth] = 100;
  return grid;
}

int8_t costOf(
  const costmap_cspace_msgs::msg::CSpace3D & map, const unsigned int x, const unsigned int y,
  const unsigned int yaw)
{
  return map.data[(yaw * map.info.height + y) * map.info.width + x];
}

int8_t costOf(
  const costmap_cspace_msgs::msg::CSpace3DUpdate & update, const unsigned int x,
  const unsigned int y, const unsigned int yaw)
{
  return update
    .data[((yaw - update.yaw) * update.height + (y - update.y)) * update.width + (x - update.x)];
}

rclcpp::QoS latchedQos(const size_t depth) { return rclcpp::QoS(depth).transient_local(); }

// Base of the per-node fixtures. Everything is addressed with absolute topic
// names so that each fixture can talk to the node running in its own
// namespace.
class Costmap3dTestBase : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr sub_costmap_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>::SharedPtr sub_costmap_update_;

  costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr costmap_;
  costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr costmap_update_;

  explicit Costmap3dTestBase(const std::string & ns) : ns_(ns)
  {
    node_ = std::make_shared<rclcpp::Node>("test_costmap_3d");
    pub_map_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(topic("map"), latchedQos(1));
    sub_costmap_ = node_->create_subscription<costmap_cspace_msgs::msg::CSpace3D>(
      topic("costmap"), latchedQos(1),
      [this](const costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr msg) { costmap_ = msg; });
    sub_costmap_update_ = node_->create_subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>(
      topic("costmap_update"), latchedQos(1),
      [this](const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr msg) {
        costmap_update_ = msg;
      });
  }

  std::string topic(const std::string & name) const { return "/" + ns_ + "/" + name; }

  // Spins until the predicate holds, re-running "tick" on every iteration so
  // that periodically re-published inputs can be driven from the caller.
  bool waitUntil(
    const std::function<bool()> & pred, const std::function<void()> & tick = nullptr,
    const double timeout_sec = 10.0)
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

  // Publishes the base map until the node answers with a costmap containing
  // the expected obstacle. The node may still be starting up, so the map is
  // re-published on every iteration.
  void setUpBaseMap(const unsigned int x, const unsigned int y)
  {
    costmap_.reset();
    ASSERT_TRUE(waitUntil(
      [this, x, y] { return costmap_ && costOf(*costmap_, x, y, 0) == 100; },
      [this, x, y] { pub_map_->publish(makeGrid(x, y)); }));
  }

private:
  std::string ns_;
};

// costmap_3d configured with an explicit static_layers/layers chain.
class Costmap3dLayeredTest : public Costmap3dTestBase
{
protected:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_overlay_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_static_overlay_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_footprint_;

  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint_;

  Costmap3dLayeredTest() : Costmap3dTestBase("layered")
  {
    // The overlay topics are named after the layer names configured through
    // the flattened "layers" / "static_layers" parameters.
    pub_overlay_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>(topic("overlay"), latchedQos(1));
    pub_static_overlay_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>(topic("static_overlay"), latchedQos(1));
    sub_footprint_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
      topic("costmap_3d/footprint"), latchedQos(2),
      [this](const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) { footprint_ = msg; });
  }
};

// costmap_3d without "layers": the backward compatible single-layer mode.
class Costmap3dCompatTest : public Costmap3dTestBase
{
protected:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_overlay_;

  Costmap3dCompatTest() : Costmap3dTestBase("compat")
  {
    pub_map_overlay_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>(topic("map_overlay"), latchedQos(1));
  }
};

TEST_F(Costmap3dLayeredTest, BaseMapIsExpandedToCSpace)
{
  ASSERT_NO_FATAL_FAILURE(setUpBaseMap(3, 3));

  EXPECT_EQ(kWidth, costmap_->info.width);
  EXPECT_EQ(kHeight, costmap_->info.height);
  // The "ang_resolution" parameter defines the number of angular grids.
  EXPECT_EQ(kAngle, costmap_->info.angle);
  EXPECT_NEAR(kResolution, costmap_->info.linear_resolution, 1e-6);
  ASSERT_EQ(kWidth * kHeight * kAngle, costmap_->data.size());

  for (unsigned int yaw = 0; yaw < kAngle; ++yaw) {
    EXPECT_EQ(100, costOf(*costmap_, 3, 3, yaw)) << "yaw: " << yaw;
    EXPECT_EQ(0, costOf(*costmap_, 0, 0, yaw)) << "yaw: " << yaw;
    EXPECT_EQ(0, costOf(*costmap_, 5, 5, yaw)) << "yaw: " << yaw;
  }
}

TEST_F(Costmap3dLayeredTest, FootprintParameterIsFlattenedArray)
{
  // The ROS 1 node took an XmlRpc array of [x, y] pairs; the ROS 2 node takes
  // the same polygon as a flat double array. Both end up in the same footprint
  // polygon, which the node publishes on ~/footprint.
  ASSERT_TRUE(waitUntil([this] { return static_cast<bool>(footprint_); }));

  const auto & points = footprint_->polygon.points;
  // Polygon::toMsg() closes the ring twice (once in the constructor and once
  // while converting), so a 4-vertex footprint yields 6 points.
  ASSERT_EQ(6u, points.size());
  const double h = kFootprintHalfWidth;
  EXPECT_NEAR(h, points[0].x, 1e-6);
  EXPECT_NEAR(-h, points[0].y, 1e-6);
  EXPECT_NEAR(h, points[1].x, 1e-6);
  EXPECT_NEAR(h, points[1].y, 1e-6);
  EXPECT_NEAR(-h, points[2].x, 1e-6);
  EXPECT_NEAR(h, points[2].y, 1e-6);
  EXPECT_NEAR(-h, points[3].x, 1e-6);
  EXPECT_NEAR(-h, points[3].y, 1e-6);
}

TEST_F(Costmap3dLayeredTest, OverlayLayerUpdatesCostmap)
{
  ASSERT_NO_FATAL_FAILURE(setUpBaseMap(3, 3));

  costmap_update_.reset();
  ASSERT_TRUE(waitUntil(
    [this] {
      return costmap_update_ && costmap_update_->width == kWidth &&
             costmap_update_->height == kHeight && costOf(*costmap_update_, 5, 5, 0) == 100;
    },
    [this] { pub_overlay_->publish(makeGrid(5, 5)); }));

  EXPECT_EQ(0u, costmap_update_->x);
  EXPECT_EQ(0u, costmap_update_->y);
  EXPECT_EQ(0u, costmap_update_->yaw);
  EXPECT_EQ(kAngle, costmap_update_->angle);
  for (unsigned int yaw = 0; yaw < kAngle; ++yaw) {
    // The overlay is merged on top of the base map in MAX mode.
    EXPECT_EQ(100, costOf(*costmap_update_, 5, 5, yaw)) << "yaw: " << yaw;
    EXPECT_EQ(100, costOf(*costmap_update_, 3, 3, yaw)) << "yaw: " << yaw;
    EXPECT_EQ(0, costOf(*costmap_update_, 0, 0, yaw)) << "yaw: " << yaw;
  }
}

TEST_F(Costmap3dLayeredTest, StaticLayerRepublishesCostmap)
{
  ASSERT_NO_FATAL_FAILURE(setUpBaseMap(3, 3));

  // A layer listed in "static_layers" sits in front of the static output, so
  // its overlay maps re-publish the whole costmap instead of an update.
  costmap_.reset();
  ASSERT_TRUE(waitUntil(
    [this] { return costmap_ && costOf(*costmap_, 6, 1, 0) == 100; },
    [this] { pub_static_overlay_->publish(makeGrid(6, 1)); }));

  for (unsigned int yaw = 0; yaw < kAngle; ++yaw) {
    EXPECT_EQ(100, costOf(*costmap_, 6, 1, yaw)) << "yaw: " << yaw;
    EXPECT_EQ(100, costOf(*costmap_, 3, 3, yaw)) << "yaw: " << yaw;
    EXPECT_EQ(0, costOf(*costmap_, 0, 0, yaw)) << "yaw: " << yaw;
  }
}

TEST_F(Costmap3dCompatTest, SingleLayerModeUsesMapOverlayTopic)
{
  ASSERT_NO_FATAL_FAILURE(setUpBaseMap(3, 3));

  EXPECT_EQ(kAngle, costmap_->info.angle);

  costmap_update_.reset();
  ASSERT_TRUE(waitUntil(
    [this] {
      return costmap_update_ && costmap_update_->width == kWidth &&
             costmap_update_->height == kHeight && costOf(*costmap_update_, 5, 5, 0) == 100;
    },
    [this] { pub_map_overlay_->publish(makeGrid(5, 5)); }));

  for (unsigned int yaw = 0; yaw < kAngle; ++yaw) {
    EXPECT_EQ(100, costOf(*costmap_update_, 5, 5, yaw)) << "yaw: " << yaw;
    EXPECT_EQ(100, costOf(*costmap_update_, 3, 3, yaw)) << "yaw: " << yaw;
    EXPECT_EQ(0, costOf(*costmap_update_, 0, 0, yaw)) << "yaw: " << yaw;
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
