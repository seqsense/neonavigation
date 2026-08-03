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

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "costmap_cspace/pointcloud_accumulator.h"
#include "laser_geometry/laser_geometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace costmap_cspace
{
// ROS 2 port of the laserscan_to_map node. The node is small and entirely
// made of ROS interface code, so it is kept as a direct ROS 2 counterpart of
// src/ros1_laserscan_to_map.cpp instead of being split into a logic class.
class LaserscanToMapNode : public rclcpp::Node
{
public:
  explicit LaserscanToMapNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

  nav_msgs::msg::OccupancyGrid map_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  laser_geometry::LaserProjection projector_;
  rclcpp::Time published_;
  rclcpp::Duration publish_interval_;

  double z_min_;
  double z_max_;
  std::string global_frame_;
  std::string robot_frame_;

  unsigned int width_;
  unsigned int height_;
  float origin_x_;
  float origin_y_;

  PointcloudAccumulator<sensor_msgs::msg::PointCloud2> accum_;

  void cbScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);
};

LaserscanToMapNode::LaserscanToMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("laserscan_to_map", options),
  published_(0, 0, RCL_ROS_TIME),
  publish_interval_(0, 0),
  origin_x_(0),
  origin_y_(0)
{
  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

  z_min_ = this->declare_parameter("z_min", std::numeric_limits<double>::lowest());
  z_max_ = this->declare_parameter("z_max", std::numeric_limits<double>::max());
  global_frame_ = this->declare_parameter("global_frame", std::string("map"));
  robot_frame_ = this->declare_parameter("robot_frame", std::string("base_link"));

  const double accum_duration = this->declare_parameter("accum_duration", 1.0);
  accum_.reset(rclcpp::Duration::from_seconds(accum_duration));

  // ROS 1 latched this publisher; transient_local is the ROS 2 equivalent.
  pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map_local", rclcpp::QoS(1).transient_local());
  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 2, std::bind(&LaserscanToMapNode::cbScan, this, std::placeholders::_1));

  const int width_param = static_cast<int>(this->declare_parameter("width", 30));
  height_ = width_ = width_param;
  map_.header.frame_id = global_frame_;

  const double resolution = this->declare_parameter("resolution", 0.1);
  map_.info.resolution = resolution;
  map_.info.width = width_;
  map_.info.height = height_;
  map_.data.resize(map_.info.width * map_.info.height);

  const double hz = this->declare_parameter("hz", 1.0);
  publish_interval_ = rclcpp::Duration::from_seconds(1.0 / hz);
}

void LaserscanToMapNode::cbScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::msg::PointCloud2 cloud_global;
  projector_.projectLaser(*scan, cloud);
  try {
    const geometry_msgs::msg::TransformStamped trans = tfbuf_->lookupTransform(
      global_frame_, cloud.header.frame_id, cloud.header.stamp,
      rclcpp::Duration::from_seconds(0.5));
    tf2::doTransform(cloud, cloud_global, trans);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
  }
  accum_.push(PointcloudAccumulator<sensor_msgs::msg::PointCloud2>::Points(
    cloud_global, cloud_global.header.stamp));

  const rclcpp::Time now(scan->header.stamp, RCL_ROS_TIME);
  if (published_ + publish_interval_ > now) {
    return;
  }
  published_ = now;

  float robot_z;
  try {
    tf2::Stamped<tf2::Transform> trans;
    tf2::fromMsg(tfbuf_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero), trans);

    const auto pos = trans.getOrigin();
    const float x = static_cast<int>(pos.x() / map_.info.resolution) * map_.info.resolution;
    const float y = static_cast<int>(pos.y() / map_.info.resolution) * map_.info.resolution;
    map_.info.origin.position.x = x - map_.info.width * map_.info.resolution * 0.5;
    map_.info.origin.position.y = y - map_.info.height * map_.info.resolution * 0.5;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    origin_x_ = x - width_ * map_.info.resolution * 0.5;
    origin_y_ = y - height_ * map_.info.resolution * 0.5;
    robot_z = pos.z();
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    return;
  }
  for (auto & cell : map_.data) {
    cell = 0;
  }

  for (auto & pc : accum_) {
    auto itr_x = sensor_msgs::PointCloud2ConstIterator<float>(pc, "x");
    auto itr_y = sensor_msgs::PointCloud2ConstIterator<float>(pc, "y");
    auto itr_z = sensor_msgs::PointCloud2ConstIterator<float>(pc, "z");
    // itr_z is intentionally not advanced here: the ROS 1 node does the same,
    // so the z filter is evaluated against the first point of the cloud. Kept
    // as-is to preserve the externally observable behaviour of the node.
    for (; itr_x != itr_x.end(); ++itr_x, ++itr_y) {
      if (*itr_z - robot_z < z_min_ || z_max_ < *itr_z - robot_z) {
        continue;
      }
      const unsigned int x =
        static_cast<int>((*itr_x - map_.info.origin.position.x) / map_.info.resolution);
      const unsigned int y =
        static_cast<int>((*itr_y - map_.info.origin.position.y) / map_.info.resolution);
      if (x >= map_.info.width || y >= map_.info.height) {
        continue;
      }
      map_.data[x + y * map_.info.width] = 100;
    }
  }

  auto out = std::make_unique<nav_msgs::msg::OccupancyGrid>(map_);
  pub_map_->publish(std::move(out));
}
}  // namespace costmap_cspace

RCLCPP_COMPONENTS_REGISTER_NODE(costmap_cspace::LaserscanToMapNode)
