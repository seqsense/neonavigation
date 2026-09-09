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
#include <vector>

#include "costmap_cspace/costmap_3d_handler.h"
#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace costmap_cspace
{
// ROS 2 interface layer of the 3-DOF configuration space costmap.
//
// The layer chain itself lives in the ROS-independent Costmap3dHandler; this
// node only translates ROS 2 parameters into Costmap3dConfig and wires the
// handler to publishers, subscribers and the footprint timer.
//
// Parameter representation
// ------------------------
// ROS 1 read the nested layer configuration through XmlRpc::XmlRpcValue.
// ROS 2 parameters are strictly flat and typed, so the nested structures are
// flattened as follows (see also README.md):
//
//   * footprint: an XmlRpc array of [x, y] pairs on ROS 1 becomes a flat
//     double array [x0, y0, x1, y1, ...] with at least three vertices.
//   * layers / static_layers: an XmlRpc array of dictionaries on ROS 1 becomes
//     a string array holding the ordered layer names, plus one parameter per
//     setting under the "layer.<name>." / "static_layer.<name>." prefix.
//
//   ROS 1                              ROS 2
//   ---------------------------------  ------------------------------------
//   footprint:                         footprint: [0.2, -0.1, 0.2, 0.1,
//     [[0.2, -0.1], [0.2, 0.1],                    -0.2, 0.1, -0.2, -0.1]
//      [-0.2, 0.1], [-0.2, -0.1]]
//   layers:                            layers: ["overlay"]
//   - name: overlay                    layer:
//     type: Costmap3dLayerFootprint      overlay:
//     overlay_mode: max                    type: Costmap3dLayerFootprint
//                                          overlay_mode: max
//
// The per-layer keys are the same as on ROS 1: type, overlay_mode, footprint
// (flat double array), linear_expand, linear_spread, linear_spread_min_cost,
// keep_unknown and unknown_cost. The externally visible behaviour (topic
// names, one subscription per layer named after the layer, published
// costmap/costmap_update/footprint/debug) is unchanged.
class Costmap3DOFNode : public rclcpp::Node
{
public:
  explicit Costmap3DOFNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> sub_map_overlay_;
  rclcpp::Publisher<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr pub_costmap_;
  rclcpp::Publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>::SharedPtr pub_costmap_update_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_footprint_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_footprint_;

  std::unique_ptr<Costmap3dHandler> handler_;
  geometry_msgs::msg::PolygonStamped footprint_msg_;

  void cbMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void cbMapOverlay(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg,
    const Costmap3dLayerBase::Ptr & layer);
  bool cbUpdateStatic(const CSpace3DMsg::Ptr & map);
  bool cbUpdate(
    const CSpace3DMsg::Ptr & map,
    const std::shared_ptr<costmap_cspace_msgs::msg::CSpace3DUpdate> & update);
  void publishDebug(const costmap_cspace_msgs::msg::CSpace3D & map);
  void cbPublishFootprint();

  Costmap3dConfig loadConfig();
  Polygon parseFootprint(const std::vector<double> & footprint_flat) const;
  bool hasOverride(const std::string & name) const;
  std::vector<Costmap3dLayerSpec> loadLayers(
    const std::string & param_name, const std::string & prefix, const Polygon & default_footprint,
    const std::string & kind);
};

Costmap3DOFNode::Costmap3DOFNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("costmap_3d", options)
{
  // ROS 1 latched all four publishers; the ROS 2 equivalent is transient_local
  // durability, which subscribers must request as well to get the last sample.
  pub_costmap_ = this->create_publisher<costmap_cspace_msgs::msg::CSpace3D>(
    "costmap", rclcpp::QoS(1).transient_local());
  pub_costmap_update_ = this->create_publisher<costmap_cspace_msgs::msg::CSpace3DUpdate>(
    "costmap_update", rclcpp::QoS(1).transient_local());
  pub_footprint_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "~/footprint", rclcpp::QoS(2).transient_local());
  pub_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
    "~/debug", rclcpp::QoS(1).transient_local());

  handler_ = std::make_unique<Costmap3dHandler>(loadConfig(), this->get_logger());
  // Time is read through the node's clock so the logic follows /clock
  // when use_sim_time is set.
  handler_->setClock(this->get_clock());
  handler_->setStaticOutputCallback(
    [this](const CSpace3DMsg::Ptr & map) { return cbUpdateStatic(map); });
  handler_->setUpdateOutputCallback(
    [this](
      const CSpace3DMsg::Ptr & map,
      const std::shared_ptr<costmap_cspace_msgs::msg::CSpace3DUpdate> & update) {
      return cbUpdate(map, update);
    });

  sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local(),
    std::bind(&Costmap3DOFNode::cbMap, this, std::placeholders::_1));
  for (const Costmap3dHandler::OverlayLayer & overlay : handler_->getOverlayLayers()) {
    // One subscription per overlay layer, on a topic named after the layer.
    const Costmap3dLayerBase::Ptr layer = overlay.layer;
    sub_map_overlay_.push_back(this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      overlay.name, rclcpp::QoS(1).transient_local(),
      [this, layer](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg) {
        cbMapOverlay(msg, layer);
      }));
  }

  footprint_msg_ = handler_->getFootprintMsg();
  timer_footprint_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(1.0),
    std::bind(&Costmap3DOFNode::cbPublishFootprint, this));
}

void Costmap3DOFNode::cbMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  handler_->setBaseMap(msg);
}

void Costmap3DOFNode::cbMapOverlay(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg, const Costmap3dLayerBase::Ptr & layer)
{
  handler_->processMapOverlay(msg, layer);
}

bool Costmap3DOFNode::cbUpdateStatic(const CSpace3DMsg::Ptr & map)
{
  publishDebug(*map);
  // CSpace3DMsg derives from CSpace3D and is owned by the layer, so the
  // published message is a sliced copy.
  auto out = std::make_unique<costmap_cspace_msgs::msg::CSpace3D>(
    static_cast<const costmap_cspace_msgs::msg::CSpace3D &>(*map));
  pub_costmap_->publish(std::move(out));
  return true;
}

bool Costmap3DOFNode::cbUpdate(
  const CSpace3DMsg::Ptr & map,
  const std::shared_ptr<costmap_cspace_msgs::msg::CSpace3DUpdate> & update)
{
  if (update) {
    publishDebug(*map);
    const bool empty_region = (update->width * update->height * update->angle == 0);
    // The update message is freshly generated for this callback and not
    // retained by the layer, so its payload can be moved out.
    auto out = std::make_unique<costmap_cspace_msgs::msg::CSpace3DUpdate>(std::move(*update));
    pub_costmap_update_->publish(std::move(out));
    if (empty_region) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Updated region of the costmap is empty. "
        "The position may be out-of-boundary, or input map is wrong.");
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Failed to update the costmap.");
  }
  return true;
}

void Costmap3DOFNode::publishDebug(const costmap_cspace_msgs::msg::CSpace3D & map)
{
  if (pub_debug_->get_subscription_count() == 0) {
    return;
  }
  auto pc = std::make_unique<sensor_msgs::msg::PointCloud>(handler_->generateDebugPointCloud(map));
  pub_debug_->publish(std::move(pc));
}

void Costmap3DOFNode::cbPublishFootprint()
{
  auto footprint = std::make_unique<geometry_msgs::msg::PolygonStamped>(footprint_msg_);
  footprint->header.stamp = this->now();
  pub_footprint_->publish(std::move(footprint));
}

bool Costmap3DOFNode::hasOverride(const std::string & name) const
{
  for (const auto & param_override : this->get_node_options().parameter_overrides()) {
    if (param_override.get_name() == name) {
      return true;
    }
  }
  return false;
}

Polygon Costmap3DOFNode::parseFootprint(const std::vector<double> & footprint_flat) const
{
  if (footprint_flat.size() % 2 != 0 || footprint_flat.size() < 6) {
    throw std::runtime_error(
      "Invalid footprint. It must be a flat [x0, y0, x1, y1, ...] array "
      "with at least three vertices.");
  }
  PolygonPoints points;
  for (size_t i = 0; i < footprint_flat.size(); i += 2) {
    PolygonPoints::value_type point;
    point[0] = footprint_flat[i];
    point[1] = footprint_flat[i + 1];
    points.push_back(point);
  }
  return Polygon(points);
}

std::vector<Costmap3dLayerSpec> Costmap3DOFNode::loadLayers(
  const std::string & param_name, const std::string & prefix, const Polygon & default_footprint,
  const std::string & kind)
{
  const std::vector<std::string> names =
    this->declare_parameter(param_name, std::vector<std::string>());
  if (names.empty()) {
    // An explicitly given but empty layer list is a configuration mistake; the
    // ROS 1 node aborted with the same migration hint in that case.
    if (hasOverride(param_name)) {
      RCLCPP_FATAL(
        this->get_logger(), "%s parameter must contain at least one layer config.",
        param_name.c_str());
      RCLCPP_ERROR(
        this->get_logger(),
        "Migration from old version:\n"
        "---  # ROS 1\n"
        "%s:\n"
        "  - name: YOUR_LAYER_NAME\n"
        "    type: LAYER_TYPE\n"
        "    parameters: values\n"
        "---  # ROS 2\n"
        "%s: [\"YOUR_LAYER_NAME\"]\n"
        "%s:\n"
        "  YOUR_LAYER_NAME:\n"
        "    type: LAYER_TYPE\n"
        "    parameters: values\n"
        "---\n",
        param_name.c_str(), param_name.c_str(), prefix.c_str());
      throw std::runtime_error(param_name + " parameter must contain at least one layer config.");
    }
    return std::vector<Costmap3dLayerSpec>();
  }

  std::vector<Costmap3dLayerSpec> specs;
  for (const std::string & name : names) {
    Costmap3dLayerSpec spec;
    spec.name = name;

    const std::string p = prefix + "." + name + ".";
    spec.type = this->declare_parameter(p + "type", std::string(""));

    const std::string overlay_mode_str =
      this->declare_parameter(p + "overlay_mode", std::string(""));
    if (!overlay_mode_str.empty()) {
      spec.overlay_mode = getMapOverlayModeFromString(overlay_mode_str);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "overlay_mode of the %s is not specified. Using MAX mode.",
        kind.c_str());
    }

    const std::vector<double> footprint_flat =
      this->declare_parameter(p + "footprint", std::vector<double>());
    if (!footprint_flat.empty()) {
      spec.config.footprint = parseFootprint(footprint_flat);
    } else {
      spec.config.footprint = default_footprint;
    }

    // The remaining keys keep the layer defaults when they are not given, so
    // the declared default mirrors Costmap3dLayerConfig.
    const Costmap3dLayerConfig defaults;
    spec.config.linear_expand =
      this->declare_parameter(p + "linear_expand", defaults.linear_expand);
    spec.config.linear_spread =
      this->declare_parameter(p + "linear_spread", defaults.linear_spread);
    spec.config.linear_spread_min_cost = static_cast<int>(
      this->declare_parameter(p + "linear_spread_min_cost", defaults.linear_spread_min_cost));
    spec.config.keep_unknown = this->declare_parameter(p + "keep_unknown", defaults.keep_unknown);
    spec.config.unknown_cost =
      static_cast<int>(this->declare_parameter(p + "unknown_cost", defaults.unknown_cost));

    specs.push_back(spec);
  }
  return specs;
}

Costmap3dConfig Costmap3DOFNode::loadConfig()
{
  Costmap3dConfig config;

  config.ang_resolution = static_cast<int>(this->declare_parameter("ang_resolution", 16));

  const std::vector<double> footprint_flat =
    this->declare_parameter("footprint", std::vector<double>());
  if (footprint_flat.empty()) {
    RCLCPP_FATAL(this->get_logger(), "Footprint doesn't specified");
    throw std::runtime_error("Footprint doesn't specified.");
  }
  try {
    config.footprint = parseFootprint(footprint_flat);
  } catch (const std::exception &) {
    RCLCPP_FATAL(this->get_logger(), "Invalid footprint");
    throw;
  }

  config.linear_expand = static_cast<float>(this->declare_parameter("linear_expand", 0.2));
  config.linear_spread = static_cast<float>(this->declare_parameter("linear_spread", 0.5));
  config.linear_spread_min_cost =
    static_cast<int>(this->declare_parameter("linear_spread_min_cost", 0));

  config.static_layers =
    loadLayers("static_layers", "static_layer", config.footprint, "static layer");
  config.layers = loadLayers("layers", "layer", config.footprint, "layer");

  if (config.layers.empty()) {
    // Single layer mode for backward-compatibility.
    const std::string overlay_mode_str =
      this->declare_parameter("overlay_mode", std::string("max"));
    Costmap3dLayerSpec spec;
    spec.overlay_mode = getMapOverlayModeFromString(overlay_mode_str);
    RCLCPP_INFO(this->get_logger(), "costmap_3d: %s mode", overlay_mode_str.c_str());

    spec.name = "map_overlay";
    spec.type = "Costmap3dLayerFootprint";
    spec.config.footprint = config.footprint;
    spec.config.linear_expand = config.linear_expand;
    spec.config.linear_spread = config.linear_spread;
    config.layers.push_back(spec);
  }

  return config;
}
}  // namespace costmap_cspace

RCLCPP_COMPONENTS_REGISTER_NODE(costmap_cspace::Costmap3DOFNode)
