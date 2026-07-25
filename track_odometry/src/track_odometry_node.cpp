/*
 * Copyright (c) 2014-2019, the neonavigation authors
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

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <track_odometry/track_odometry.h>

#include <neonavigation_common/compatibility.h>

class TrackOdometryNode
{
private:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu>;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_imu_raw_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Imu>> sub_imu_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  ros::Subscriber sub_reset_z_;
  ros::Publisher pub_odom_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string base_link_id_;
  std::string odom_id_;

  bool without_odom_;
  bool publish_tf_;

  track_odometry::TrackOdometry track_odometry_;

  void cbResetZ(const std_msgs::Float32::Ptr& msg)
  {
    track_odometry_.resetZ(msg->data);
  }
  void cbOdomImu(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    ROS_DEBUG(
        "Synchronized timestamp: odom %0.3f, imu %0.3f",
        odom_msg->header.stamp.toSec(),
        imu_msg->header.stamp.toSec());
    cbImu(imu_msg);
    cbOdom(odom_msg);
  }
  void cbImu(const sensor_msgs::Imu::ConstPtr& msg)
  {
    track_odometry_.processImu(msg);
  }
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
  {
    const track_odometry::TrackOdometry::OdomResult result = track_odometry_.processOdom(msg);
    if (result.valid)
    {
      pub_odom_.publish(result.odom);
      if (publish_tf_)
        tf_broadcaster_.sendTransform(result.transform);
    }
  }

public:
  TrackOdometryNode()
    : nh_()
    , pnh_("~")
    , tf_listener_(tf_buffer_)
    , track_odometry_(tf_buffer_)
  {
    neonavigation_common::compat::checkCompatMode();

    track_odometry::TrackOdometryParams params;

    bool enable_tcp_no_delay;
    pnh_.param("enable_tcp_no_delay", enable_tcp_no_delay, true);
    const ros::TransportHints transport_hints =
        enable_tcp_no_delay ? ros::TransportHints().reliable().tcpNoDelay(true) : ros::TransportHints();

    pnh_.param("without_odom", without_odom_, false);
    if (without_odom_)
    {
      sub_imu_raw_ = neonavigation_common::compat::subscribe(
          nh_, "imu/data",
          nh_, "imu", 64, &TrackOdometryNode::cbImu, this);
      pnh_.param("base_link_id", params.base_link_id, std::string("base_link"));
      pnh_.param("odom_id", odom_id_, std::string("odom"));
      base_link_id_ = params.base_link_id;
    }
    else
    {
      sub_odom_.reset(
          new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odom_raw", 50, transport_hints));
      if (neonavigation_common::compat::getCompat() == neonavigation_common::compat::current_level)
      {
        sub_imu_.reset(
            new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "imu/data", 50, transport_hints));
      }
      else
      {
        sub_imu_.reset(
            new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "imu", 50, transport_hints));
      }

      int sync_window;
      pnh_.param("sync_window", sync_window, 50);
      sync_.reset(
          new message_filters::Synchronizer<SyncPolicy>(
              SyncPolicy(sync_window), *sub_odom_, *sub_imu_));
      sync_->registerCallback(boost::bind(&TrackOdometryNode::cbOdomImu, this, _1, _2));

      pnh_.param("base_link_id", params.base_link_id_overwrite, std::string(""));
    }

    sub_reset_z_ = neonavigation_common::compat::subscribe(
        nh_, "reset_odometry_z",
        pnh_, "reset_z", 1, &TrackOdometryNode::cbResetZ, this);
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 8);

    if (pnh_.hasParam("z_filter"))
    {
      params.z_filter_timeconst = -1.0;
      double z_filter;
      if (pnh_.getParam("z_filter", z_filter))
      {
        const double odom_freq = 100.0;
        if (0.0 < z_filter && z_filter < 1.0)
          params.z_filter_timeconst = (1.0 / odom_freq) / (1.0 - z_filter);
      }
      ROS_ERROR(
          "track_odometry: ~z_filter parameter (exponential filter (1 - alpha) value) is deprecated. "
          "Use ~z_filter_timeconst (in seconds) instead. "
          "Treated as z_filter_timeconst=%0.6f. (negative value means disabled)",
          params.z_filter_timeconst);
    }
    else
    {
      pnh_.param("z_filter_timeconst", params.z_filter_timeconst, -1.0);
    }
    pnh_.param("tf_tolerance", params.tf_tolerance, 0.01);
    pnh_.param("use_kf", params.use_kf, true);
    pnh_.param("enable_negative_slip", params.negative_slip, false);
    pnh_.param("debug", params.debug, false);
    pnh_.param("publish_tf", publish_tf_, true);

    if (params.base_link_id_overwrite.size() > 0)
    {
      base_link_id_ = params.base_link_id_overwrite;
    }

    // sigma_odom [rad/s]: standard deviation of odometry angular vel on straight running
    pnh_.param("sigma_odom", params.sigma_odom, 0.005);
    // sigma_predict [sigma/second]: prediction sigma of kalman filter
    pnh_.param("sigma_predict", params.sigma_predict, 0.5);
    // predict_filter_tc [sec.]: LPF time-constant to forget estimated slip ratio
    pnh_.param("predict_filter_tc", params.predict_filter_tc, 1.0);

    track_odometry_.setParameters(params);
  }
  void cbTimer(const ros::TimerEvent& /* event */)
  {
    nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
    odom->header.stamp = ros::Time::now();
    odom->header.frame_id = odom_id_;
    odom->child_frame_id = base_link_id_;
    odom->pose.pose.orientation.w = 1.0;
    cbOdom(odom);
  }
  void spin()
  {
    if (!without_odom_)
    {
      ros::spin();
    }
    else
    {
      ros::Timer timer = nh_.createTimer(
          ros::Duration(1.0 / 50.0), &TrackOdometryNode::cbTimer, this);
      ros::spin();
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "track_odometry");

  TrackOdometryNode odom;

  odom.spin();

  return 0;
}
