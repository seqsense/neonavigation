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

#ifndef COSTMAP_CSPACE__COSTMAP_3D_HANDLER_H_
#define COSTMAP_CSPACE__COSTMAP_3D_HANDLER_H_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "costmap_cspace/costmap_3d.h"
#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace costmap_cspace
{
// Converts the textual representation of the overlay mode into the enum.
// Throws std::runtime_error when the string is not a known mode.
MapOverlayMode getMapOverlayModeFromString(const std::string & overlay_mode_str);

// ROS-neutral description of one overlay layer of the costmap chain.
// "name" is also used by the ROS interface layer as the name of the topic
// the overlay map of this layer is received on.
struct Costmap3dLayerSpec
{
  std::string name;
  std::string type;
  MapOverlayMode overlay_mode = MapOverlayMode::MAX;
  Costmap3dLayerConfig config;
};

// ROS-neutral description of the whole costmap layer chain.
struct Costmap3dConfig
{
  int ang_resolution = 16;
  Polygon footprint;
  float linear_expand = 0.2f;
  float linear_spread = 0.5f;
  int linear_spread_min_cost = 0;
  std::vector<Costmap3dLayerSpec> static_layers;
  std::vector<Costmap3dLayerSpec> layers;
};

// Builds and drives the costmap layer chain.
// This class holds no ROS node, publisher, subscriber nor parameter access;
// the ROS interface layer feeds it with incoming messages and publishes the
// results handed back through the output callbacks.
class Costmap3dHandler
{
public:
  using Ptr = std::shared_ptr<Costmap3dHandler>;

  // An overlay layer of the chain together with the name it was configured
  // with, in the order they were added.
  struct OverlayLayer
  {
    std::string name;
    Costmap3dLayerBase::Ptr layer;
  };

  using StaticOutputCallback = std::function<bool(const CSpace3DMsg::Ptr &)>;
  using UpdateOutputCallback = std::function<bool(
    const CSpace3DMsg::Ptr &, const std::shared_ptr<costmap_cspace_msgs::msg::CSpace3DUpdate> &)>;

  Costmap3dHandler(const Costmap3dConfig & config, const rclcpp::Logger & logger);

  void setStaticOutputCallback(StaticOutputCallback cb);
  void setUpdateOutputCallback(UpdateOutputCallback cb);

  const std::vector<OverlayLayer> & getOverlayLayers() const { return overlay_layers_; }
  const geometry_msgs::msg::PolygonStamped & getFootprintMsg() const { return footprint_msg_; }
  Costmap3dLayerFootprint::Ptr getRootLayer() const { return root_layer_; }

  // Sets the base (static) map of the chain and flushes the overlay maps
  // which arrived before the base map was available.
  void setBaseMap(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid> & msg);
  // Applies an overlay map to the given layer. The map is buffered when the
  // base map has not been received yet.
  void processMapOverlay(
    const std::shared_ptr<const nav_msgs::msg::OccupancyGrid> & msg,
    const Costmap3dLayerBase::Ptr & layer);

  // Builds the point cloud visualizing the occupied cells of the costmap.
  sensor_msgs::msg::PointCloud generateDebugPointCloud(
    const costmap_cspace_msgs::msg::CSpace3D & map);

  // The clock the logic reads time from. On ROS 2 a bare rclcpp::Clock never
  // subscribes to /clock, so under use_sim_time it silently is wall time; the
  // interface node hands in its own clock instead. The default keeps the ROS 1
  // build right on its own, where the compat rclcpp::Clock is ros::Time.
  void setClock(const rclcpp::Clock::SharedPtr & clock) { clock_ = clock; }

protected:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  Costmap3d::Ptr costmap_;
  Costmap3dLayerFootprint::Ptr root_layer_;
  Costmap3dStaticLayerOutput::Ptr static_output_layer_;
  Costmap3dUpdateLayerOutput::Ptr update_output_layer_;
  std::vector<OverlayLayer> overlay_layers_;
  std::vector<
    std::pair<std::shared_ptr<const nav_msgs::msg::OccupancyGrid>, Costmap3dLayerBase::Ptr>>
    map_buffer_;
  geometry_msgs::msg::PolygonStamped footprint_msg_;

  void addOverlayLayer(const Costmap3dLayerSpec & spec);
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE__COSTMAP_3D_HANDLER_H_
