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

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include "rclcpp/rclcpp.hpp"

#include "costmap_cspace/costmap_3d_handler.h"

namespace costmap_cspace
{
MapOverlayMode getMapOverlayModeFromString(const std::string& overlay_mode_str)
{
  if (overlay_mode_str == "overwrite")
  {
    return MapOverlayMode::OVERWRITE;
  }
  else if (overlay_mode_str == "max")
  {
    return MapOverlayMode::MAX;
  }
  RCLCPP_ERROR(
      rclcpp::get_logger("costmap_cspace"),
      "Unknown overlay_mode \"%s\"", overlay_mode_str.c_str());
  throw std::runtime_error("Unknown overlay_mode.");
}

Costmap3dHandler::Costmap3dHandler(const Costmap3dConfig& config, const rclcpp::Logger& logger)
  : logger_(logger)
{
  costmap_.reset(new Costmap3d(config.ang_resolution));

  root_layer_ = costmap_->addRootLayer<Costmap3dLayerFootprint>();
  root_layer_->setLogger(logger_);
  root_layer_->setExpansion(
      config.linear_expand, config.linear_spread, config.linear_spread_min_cost);
  root_layer_->setFootprint(config.footprint);
  footprint_msg_ = config.footprint.toMsg();

  for (const Costmap3dLayerSpec& spec : config.static_layers)
  {
    RCLCPP_INFO(logger_, "New static layer: %s", spec.name.c_str());
    addOverlayLayer(spec);
  }

  static_output_layer_ = costmap_->addLayer<Costmap3dStaticLayerOutput>();
  static_output_layer_->setLogger(logger_);

  for (const Costmap3dLayerSpec& spec : config.layers)
  {
    RCLCPP_INFO(logger_, "New layer: %s", spec.name.c_str());
    addOverlayLayer(spec);
  }

  update_output_layer_ = costmap_->addLayer<Costmap3dUpdateLayerOutput>();
  update_output_layer_->setLogger(logger_);
}

void Costmap3dHandler::addOverlayLayer(const Costmap3dLayerSpec& spec)
{
  if (spec.type.empty())
  {
    RCLCPP_ERROR(logger_, "Layer type is not specified.");
    throw std::runtime_error("Layer type is not specified.");
  }
  Costmap3dLayerBase::Ptr layer = Costmap3dLayerClassLoader::loadClass(spec.type);
  layer->setLogger(logger_);
  costmap_->addLayer(layer, spec.overlay_mode);
  layer->loadConfig(spec.config);

  overlay_layers_.push_back(OverlayLayer{spec.name, layer});
}

void Costmap3dHandler::setStaticOutputCallback(StaticOutputCallback cb)
{
  if (!cb)
    return;
  static_output_layer_->setHandler(cb);
}

void Costmap3dHandler::setUpdateOutputCallback(UpdateOutputCallback cb)
{
  if (!cb)
    return;
  update_output_layer_->setHandler(cb);
}

void Costmap3dHandler::setBaseMap(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid>& msg)
{
  if (root_layer_->getAngularGrid() <= 0)
  {
    RCLCPP_ERROR(logger_, "ang_resolution is not set.");
    std::runtime_error("ang_resolution is not set.");
  }
  RCLCPP_INFO(logger_, "2D costmap received");

  root_layer_->setBaseMap(msg);
  RCLCPP_DEBUG(logger_, "C-Space costmap generated");

  if (map_buffer_.size() > 0)
  {
    const size_t buffered = map_buffer_.size();
    // processMapOverlay() may push the map back to the buffer, so iterate on a copy.
    const auto map_buffer = map_buffer_;
    map_buffer_.clear();
    for (const auto& map : map_buffer)
      processMapOverlay(map.first, map.second);
    RCLCPP_INFO(logger_, "%ld buffered costmaps processed", buffered);
  }
}

void Costmap3dHandler::processMapOverlay(
    const std::shared_ptr<const nav_msgs::msg::OccupancyGrid>& msg,
    const Costmap3dLayerBase::Ptr& layer)
{
  RCLCPP_DEBUG(logger_, "Overlay 2D costmap received");

  auto map_msg = layer->getMap();
  if (map_msg->info.width < 1 ||
      map_msg->info.height < 1)
  {
    map_buffer_.push_back(
        std::pair<std::shared_ptr<const nav_msgs::msg::OccupancyGrid>,
                  Costmap3dLayerBase::Ptr>(msg, layer));
    return;
  }

  layer->processMapOverlay(msg, true);
  RCLCPP_DEBUG(logger_, "C-Space costmap updated");
}

sensor_msgs::msg::PointCloud Costmap3dHandler::generateDebugPointCloud(
    const costmap_cspace_msgs::msg::CSpace3D& map)
{
  sensor_msgs::msg::PointCloud pc;
  pc.header = map.header;
  pc.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  for (size_t yaw = 0; yaw < map.info.angle; yaw++)
  {
    for (unsigned int i = 0; i < map.info.width * map.info.height; i++)
    {
      int gx = i % map.info.width;
      int gy = i / map.info.width;
      if (map.data[i + yaw * map.info.width * map.info.height] < 100)
        continue;
      geometry_msgs::msg::Point32 p;
      p.x = gx * map.info.linear_resolution + map.info.origin.position.x;
      p.y = gy * map.info.linear_resolution + map.info.origin.position.y;
      p.z = yaw * 0.1;
      pc.points.push_back(p);
    }
  }
  return pc;
}
}  // namespace costmap_cspace
