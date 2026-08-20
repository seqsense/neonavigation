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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "costmap_cspace/pointcloud_accumulator.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace costmap_cspace
{
// ROS 2 port of the pointcloud2_to_map node. The node is small and entirely
// made of ROS interface code, so it is kept as a direct ROS 2 counterpart of
// src/ros1_pointcloud2_to_map.cpp instead of being split into a logic class.
class Pointcloud2ToMapNode : public rclcpp::Node
{
public:
  explicit Pointcloud2ToMapNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_single_;

  nav_msgs::msg::OccupancyGrid map_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
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

  std::vector<PointcloudAccumulator<sensor_msgs::msg::PointCloud2>> accums_;

  void cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud, const bool singleshot);
};

Pointcloud2ToMapNode::Pointcloud2ToMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud2_to_map", options),
  published_(0, 0, RCL_ROS_TIME),
  publish_interval_(0, 0),
  origin_x_(0),
  origin_y_(0),
  accums_(2)
{
  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

  z_min_ = this->declare_parameter("z_min", 0.1);
  z_max_ = this->declare_parameter("z_max", 1.0);
  global_frame_ = this->declare_parameter("global_frame", std::string("map"));
  robot_frame_ = this->declare_parameter("robot_frame", std::string("base_link"));

  const double accum_duration = this->declare_parameter("accum_duration", 1.0);
  accums_[0].reset(rclcpp::Duration::from_seconds(accum_duration));
  accums_[1].reset(rclcpp::Duration(0, 0));

  // ROS 1 latched this publisher; transient_local is the ROS 2 equivalent.
  pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map_local", rclcpp::QoS(1).transient_local());
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", 100,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) { cbCloud(msg, false); });
  sub_cloud_single_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_singleshot", 100,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) { cbCloud(msg, true); });

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

void Pointcloud2ToMapNode::cbCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud, const bool singleshot)
{
  sensor_msgs::msg::PointCloud2 cloud_global;
  geometry_msgs::msg::TransformStamped trans;
  try {
    trans = tfbuf_->lookupTransform(
      global_frame_, cloud->header.frame_id, cloud->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    return;
  }
  tf2::doTransform(*cloud, cloud_global, trans);

  const int buffer = singleshot ? 1 : 0;
  accums_[buffer].push(
    PointcloudAccumulator<sensor_msgs::msg::PointCloud2>::Points(
      cloud_global, cloud_global.header.stamp));

  const rclcpp::Time now(cloud->header.stamp, RCL_ROS_TIME);
  if (published_ + publish_interval_ > now) {
    return;
  }
  published_ = now;

  float robot_z;
  try {
    tf2::Stamped<tf2::Transform> robot_trans;
    tf2::fromMsg(
      tfbuf_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero), robot_trans);

    const auto pos = robot_trans.getOrigin();
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

  for (auto & accum : accums_) {
    for (auto & pc : accum) {
      sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (*iter_z - robot_z < z_min_ || z_max_ < *iter_z - robot_z) {
          continue;
        }
        const unsigned int x =
          static_cast<int>((*iter_x - map_.info.origin.position.x) / map_.info.resolution);
        const unsigned int y =
          static_cast<int>((*iter_y - map_.info.origin.position.y) / map_.info.resolution);
        if (x >= map_.info.width || y >= map_.info.height) {
          continue;
        }
        map_.data[x + y * map_.info.width] = 100;
      }
    }
  }

  auto out = std::make_unique<nav_msgs::msg::OccupancyGrid>(map_);
  pub_map_->publish(std::move(out));
}
}  // namespace costmap_cspace

RCLCPP_COMPONENTS_REGISTER_NODE(costmap_cspace::Pointcloud2ToMapNode)
