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

#include <map_organizer/pointcloud_to_maps.h>
#include <map_organizer_msgs/OccupancyGridArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <neonavigation_common/compatibility.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <map>
#include <sq_ros1_compat/logger.hpp>
#include <string>

class PointcloudToMapsNode
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  std::map<std::string, ros::Publisher> pub_maps_;
  ros::Publisher pub_map_array_;
  ros::Subscriber sub_points_;

public:
  PointcloudToMapsNode() : pnh_("~"), nh_()
  {
    neonavigation_common::compat::checkCompatMode();
    sub_points_ = neonavigation_common::compat::subscribe(
      nh_, "mapcloud", pnh_, "map_cloud", 1, &PointcloudToMapsNode::cbPoints, this);
    pub_map_array_ = nh_.advertise<map_organizer_msgs::OccupancyGridArray>("maps", 1, true);
  }
  void cbPoints(const sensor_msgs::PointCloud2::Ptr & msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pc);

    map_organizer::PointcloudToMaps::Config config;
    pnh_.param("grid", config.grid, 0.05);
    pnh_.param("points_thresh_rate", config.points_thresh_rate, 0.5);
    pnh_.param("robot_height", config.robot_height, 1.0);
    pnh_.param("floor_height", config.floor_height, 0.1);
    pnh_.param("floor_tolerance", config.floor_tolerance, 0.2);
    pnh_.param("min_floor_area", config.min_floor_area, 100.0);
    pnh_.param("floor_area_thresh_rate", config.floor_area_thresh_rate, 0.8);

    map_organizer::PointcloudToMaps p2m(config, sq_ros1_compat::get_logger("pointcloud_to_maps"));
    const map_organizer_msgs::OccupancyGridArray map_array = p2m.generateMaps(*pc, msg->header);

    for (size_t i = 0; i < map_array.maps.size(); ++i) {
      const std::string name = "map" + std::to_string(i);
      pub_maps_[name] = pnh_.advertise<nav_msgs::OccupancyGrid>(name, 1, true);
      pub_maps_[name].publish(map_array.maps[i]);
    }
    pub_map_array_.publish(map_array);
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pointcloud_to_maps");

  PointcloudToMapsNode p2m;
  ros::spin();

  return 0;
}
