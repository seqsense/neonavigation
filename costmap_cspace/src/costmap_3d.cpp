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

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <xmlrpcpp/XmlRpcException.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>

#include <costmap_cspace/costmap_3d_handler.h>
#include <neonavigation_common/compatibility.h>

namespace
{
// Converts the XmlRpc representation of the "footprint" parameter, an array of
// [x, y] pairs, into the ROS-neutral polygon used by the costmap logic.
costmap_cspace::Polygon parseFootprint(XmlRpc::XmlRpcValue footprint_xml)
{
  if (footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
  {
    throw std::runtime_error("Invalid footprint xml.");
  }

  costmap_cspace::PolygonPoints points;
  for (int i = 0; i < footprint_xml.size(); i++)
  {
    try
    {
      costmap_cspace::PolygonPoints::value_type point;
      point[0] = static_cast<double>(footprint_xml[i][0]);
      point[1] = static_cast<double>(footprint_xml[i][1]);
      points.push_back(point);
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      throw std::runtime_error("Invalid footprint xml." + e.getMessage());
    }
  }
  return costmap_cspace::Polygon(points);
}

// Converts the XmlRpc representation of one entry of the "layers" or
// "static_layers" parameter into the ROS-neutral layer spec.
costmap_cspace::Costmap3dLayerSpec parseLayer(
    XmlRpc::XmlRpcValue layer_xml,
    const costmap_cspace::Polygon& default_footprint,
    const std::string& kind)
{
  costmap_cspace::Costmap3dLayerSpec spec;
  spec.name = static_cast<std::string>(layer_xml["name"]);

  if (layer_xml["overlay_mode"].getType() == XmlRpc::XmlRpcValue::TypeString)
    spec.overlay_mode = costmap_cspace::getMapOverlayModeFromString(
        static_cast<std::string>(layer_xml["overlay_mode"]));
  else
    ROS_WARN("overlay_mode of the %s is not specified. Using MAX mode.", kind.c_str());

  if (layer_xml["type"].getType() == XmlRpc::XmlRpcValue::TypeString)
    spec.type = static_cast<std::string>(layer_xml["type"]);

  if (layer_xml.hasMember("footprint"))
    spec.config.footprint = parseFootprint(layer_xml["footprint"]);
  else
    spec.config.footprint = default_footprint;

  if (layer_xml.hasMember("linear_expand"))
    spec.config.linear_expand = static_cast<double>(layer_xml["linear_expand"]);
  if (layer_xml.hasMember("linear_spread"))
    spec.config.linear_spread = static_cast<double>(layer_xml["linear_spread"]);
  if (layer_xml.hasMember("linear_spread_min_cost"))
    spec.config.linear_spread_min_cost = static_cast<int>(layer_xml["linear_spread_min_cost"]);
  if (layer_xml.hasMember("keep_unknown"))
    spec.config.keep_unknown = static_cast<bool>(layer_xml["keep_unknown"]);
  if (layer_xml.hasMember("unknown_cost"))
    spec.config.unknown_cost = static_cast<int>(layer_xml["unknown_cost"]);

  return spec;
}

std::vector<costmap_cspace::Costmap3dLayerSpec> parseLayers(
    XmlRpc::XmlRpcValue layers_xml,
    const costmap_cspace::Polygon& default_footprint,
    const std::string& param_name,
    const std::string& kind)
{
  if (layers_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || layers_xml.size() < 1)
  {
    ROS_FATAL("%s parameter must contain at least one layer config.", param_name.c_str());
    ROS_ERROR(
        "Migration from old version:\n"
        "---  # Old\n"
        "%s:\n"
        "  YOUR_LAYER_NAME:\n"
        "    type: LAYER_TYPE\n"
        "    parameters: values\n"
        "---  # New\n"
        "%s:\n"
        "  - name: YOUR_LAYER_NAME\n"
        "    type: LAYER_TYPE\n"
        "    parameters: values\n"
        "---\n",
        param_name.c_str(), param_name.c_str());
    throw std::runtime_error("layers parameter must contain at least one layer config.");
  }

  std::vector<costmap_cspace::Costmap3dLayerSpec> specs;
  for (int i = 0; i < layers_xml.size(); ++i)
  {
    specs.push_back(parseLayer(layers_xml[i], default_footprint, kind));
  }
  return specs;
}
}  // namespace

class Costmap3DOFNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_map_;
  std::vector<ros::Subscriber> sub_map_overlay_;
  ros::Publisher pub_costmap_;
  ros::Publisher pub_costmap_update_;
  ros::Publisher pub_footprint_;
  ros::Publisher pub_debug_;
  ros::Timer timer_footprint_;

  std::unique_ptr<costmap_cspace::Costmap3dHandler> handler_;

  void cbMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    handler_->setBaseMap(msg);
  }
  void cbMapOverlay(
      const nav_msgs::OccupancyGrid::ConstPtr& msg,
      const costmap_cspace::Costmap3dLayerBase::Ptr layer)
  {
    handler_->processMapOverlay(msg, layer);
  }
  bool cbUpdateStatic(
      const costmap_cspace::CSpace3DMsg::Ptr& map)
  {
    publishDebug(*map);
    pub_costmap_.publish<costmap_cspace_msgs::CSpace3D>(*map);
    return true;
  }
  bool cbUpdate(
      const costmap_cspace::CSpace3DMsg::Ptr& map,
      const costmap_cspace_msgs::CSpace3DUpdate::Ptr& update)
  {
    if (update)
    {
      publishDebug(*map);
      pub_costmap_update_.publish(*update);
      if (update->width * update->height * update->angle == 0)
      {
        ROS_WARN_THROTTLE(
            5, "Updated region of the costmap is empty. "
               "The position may be out-of-boundary, or input map is wrong.");
      }
    }
    else
    {
      ROS_WARN_THROTTLE(5, "Failed to update the costmap.");
    }
    return true;
  }
  void publishDebug(const costmap_cspace_msgs::CSpace3D& map)
  {
    if (pub_debug_.getNumSubscribers() == 0)
      return;
    pub_debug_.publish(costmap_cspace::Costmap3dHandler::generateDebugPointCloud(map));
  }
  void cbPublishFootprint(const ros::TimerEvent& /* event */, const geometry_msgs::PolygonStamped msg)
  {
    auto footprint = msg;
    footprint.header.stamp = ros::Time::now();
    pub_footprint_.publish(footprint);
  }

  costmap_cspace::Costmap3dConfig loadConfig()
  {
    costmap_cspace::Costmap3dConfig config;

    pnh_.param("ang_resolution", config.ang_resolution, 16);

    XmlRpc::XmlRpcValue footprint_xml;
    if (!pnh_.hasParam("footprint"))
    {
      ROS_FATAL("Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified.");
    }
    pnh_.getParam("footprint", footprint_xml);
    try
    {
      config.footprint = parseFootprint(footprint_xml);
    }
    catch (const std::exception& e)
    {
      ROS_FATAL("Invalid footprint");
      throw e;
    }

    pnh_.param("linear_expand", config.linear_expand, 0.2f);
    pnh_.param("linear_spread", config.linear_spread, 0.5f);
    pnh_.param("linear_spread_min_cost", config.linear_spread_min_cost, 0);

    if (pnh_.hasParam("static_layers"))
    {
      XmlRpc::XmlRpcValue layers_xml;
      pnh_.getParam("static_layers", layers_xml);
      config.static_layers = parseLayers(
          layers_xml, config.footprint, "static_layers", "static layer");
    }

    if (pnh_.hasParam("layers"))
    {
      XmlRpc::XmlRpcValue layers_xml;
      pnh_.getParam("layers", layers_xml);
      config.layers = parseLayers(layers_xml, config.footprint, "layers", "layer");
    }
    else
    {
      // Single layer mode for backward-compatibility
      std::string overlay_mode_str;
      pnh_.param("overlay_mode", overlay_mode_str, std::string("max"));
      costmap_cspace::Costmap3dLayerSpec spec;
      spec.overlay_mode = costmap_cspace::getMapOverlayModeFromString(overlay_mode_str);
      ROS_INFO("costmap_3d: %s mode", overlay_mode_str.c_str());

      spec.name = "map_overlay";
      spec.type = "Costmap3dLayerFootprint";
      spec.config.footprint = config.footprint;
      spec.config.linear_expand = config.linear_expand;
      spec.config.linear_spread = config.linear_spread;
      config.layers.push_back(spec);
    }

    return config;
  }

public:
  Costmap3DOFNode()
    : nh_()
    , pnh_("~")
  {
    neonavigation_common::compat::checkCompatMode();
    pub_costmap_ = neonavigation_common::compat::advertise<costmap_cspace_msgs::CSpace3D>(
        nh_, "costmap",
        pnh_, "costmap", 1, true);
    pub_costmap_update_ = neonavigation_common::compat::advertise<costmap_cspace_msgs::CSpace3DUpdate>(
        nh_, "costmap_update",
        pnh_, "costmap_update", 1, true);
    pub_footprint_ = pnh_.advertise<geometry_msgs::PolygonStamped>("footprint", 2, true);
    pub_debug_ = pnh_.advertise<sensor_msgs::PointCloud>("debug", 1, true);

    handler_.reset(new costmap_cspace::Costmap3dHandler(loadConfig()));
    handler_->setStaticOutputCallback(
        boost::bind(&Costmap3DOFNode::cbUpdateStatic, this, _1));
    handler_->setUpdateOutputCallback(
        boost::bind(&Costmap3DOFNode::cbUpdate, this, _1, _2));

    sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
        "map", 1,
        boost::bind(&Costmap3DOFNode::cbMap, this, _1));
    for (const costmap_cspace::Costmap3dHandler::OverlayLayer& overlay : handler_->getOverlayLayers())
    {
      sub_map_overlay_.push_back(nh_.subscribe<nav_msgs::OccupancyGrid>(
          overlay.name, 1,
          boost::bind(&Costmap3DOFNode::cbMapOverlay, this, _1, overlay.layer)));
    }

    const geometry_msgs::PolygonStamped footprint_msg = handler_->getFootprintMsg();
    timer_footprint_ = nh_.createTimer(
        ros::Duration(1.0),
        boost::bind(&Costmap3DOFNode::cbPublishFootprint, this, _1, footprint_msg));
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "costmap_3d");

  Costmap3DOFNode cm;
  ros::spin();

  return 0;
}
