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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <safety_limiter_msgs/SafetyLimiterStatus.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <neonavigation_common/compatibility.h>

#include <safety_limiter/SafetyLimiterConfig.h>
#include <safety_limiter/safety_limiter.h>

namespace safety_limiter
{
bool XmlRpc_isNumber(XmlRpc::XmlRpcValue& value)
{
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
         value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
}

class SafetyLimiterNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_status_;
  ros::Subscriber sub_twist_;
  std::vector<ros::Subscriber> sub_clouds_;
  ros::Subscriber sub_disable_;
  ros::Subscriber sub_watchdog_;
  ros::Timer watchdog_timer_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  boost::recursive_mutex parameter_server_mutex_;
  std::unique_ptr<dynamic_reconfigure::Server<SafetyLimiterConfig>> parameter_server_;

  SafetyLimiter limiter_;

  geometry_msgs::Twist twist_;
  ros::Time last_cloud_stamp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accum_;
  bool cloud_clear_;
  double hz_;
  double timeout_;
  double disable_timeout_;
  double r_lim_;
  double max_values_[2];
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  ros::Time last_disable_cmd_;
  ros::Duration hold_;
  ros::Time hold_off_;
  ros::Duration watchdog_interval_;

  bool watchdog_stop_;
  bool has_cloud_;
  bool has_twist_;

  constexpr static float EPSILON = 1e-6;

  diagnostic_updater::Updater diag_updater_;

public:
  SafetyLimiterNode()
    : nh_()
    , pnh_("~")
    , tfl_(tfbuf_)
    , limiter_(tfbuf_)
    , cloud_accum_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloud_clear_(false)
    , last_disable_cmd_(0)
    , watchdog_stop_(false)
    , has_cloud_(false)
    , has_twist_(true)
  {
    neonavigation_common::compat::checkCompatMode();
    pub_twist_ = neonavigation_common::compat::advertise<geometry_msgs::Twist>(
        nh_, "cmd_vel",
        pnh_, "cmd_vel_out", 1, true);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud>("collision", 1, true);
    pub_status_ = pnh_.advertise<safety_limiter_msgs::SafetyLimiterStatus>("status", 1, true);
    sub_twist_ = neonavigation_common::compat::subscribe(
        nh_, "cmd_vel_in",
        pnh_, "cmd_vel_in", 1, &SafetyLimiterNode::cbTwist, this);
    sub_disable_ = neonavigation_common::compat::subscribe(
        nh_, "disable_safety",
        pnh_, "disable", 1, &SafetyLimiterNode::cbDisable, this);
    sub_watchdog_ = neonavigation_common::compat::subscribe(
        nh_, "watchdog_reset",
        pnh_, "watchdog_reset", 1, &SafetyLimiterNode::cbWatchdogReset, this);

    int num_input_clouds;
    pnh_.param("num_input_clouds", num_input_clouds, 1);
    if (num_input_clouds == 1)
    {
      sub_clouds_.push_back(neonavigation_common::compat::subscribe(
          nh_, "cloud",
          pnh_, "cloud", 1, &SafetyLimiterNode::cbCloud, this));
    }
    else
    {
      for (int i = 0; i < num_input_clouds; ++i)
      {
        sub_clouds_.push_back(nh_.subscribe(
            "cloud" + std::to_string(i), 1, &SafetyLimiterNode::cbCloud, this));
      }
    }

    if (pnh_.hasParam("t_margin"))
      ROS_WARN("safety_limiter: t_margin parameter is obsolated. Use d_margin and yaw_margin instead.");
    pnh_.param("base_frame", base_frame_id_, std::string("base_link"));
    pnh_.param("fixed_frame", fixed_frame_id_, std::string("odom"));
    limiter_.setBaseFrame(base_frame_id_);
    double watchdog_interval_d;
    pnh_.param("watchdog_interval", watchdog_interval_d, 0.0);
    watchdog_interval_ = ros::Duration(watchdog_interval_d);
    pnh_.param("max_linear_vel", max_values_[0], std::numeric_limits<double>::infinity());
    pnh_.param("max_angular_vel", max_values_[1], std::numeric_limits<double>::infinity());

    parameter_server_.reset(
        new dynamic_reconfigure::Server<SafetyLimiterConfig>(parameter_server_mutex_, pnh_));
    parameter_server_->setCallback(boost::bind(&SafetyLimiterNode::cbParameter, this, _1, _2));

    XmlRpc::XmlRpcValue footprint_xml;
    if (!pnh_.hasParam("footprint"))
    {
      ROS_FATAL("Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified");
    }
    pnh_.getParam("footprint", footprint_xml);
    if (footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
    {
      ROS_FATAL("Invalid footprint");
      throw std::runtime_error("Invalid footprint");
    }
    polygon footprint_p;
    float footprint_radius = 0;
    for (int i = 0; i < footprint_xml.size(); i++)
    {
      if (!XmlRpc_isNumber(footprint_xml[i][0]) ||
          !XmlRpc_isNumber(footprint_xml[i][1]))
      {
        ROS_FATAL("Invalid footprint value");
        throw std::runtime_error("Invalid footprint value");
      }

      vec v;
      v[0] = static_cast<double>(footprint_xml[i][0]);
      v[1] = static_cast<double>(footprint_xml[i][1]);
      footprint_p.v.push_back(v);

      const float dist = std::hypot(v[0], v[1]);
      if (dist > footprint_radius)
        footprint_radius = dist;
    }
    footprint_p.v.push_back(footprint_p.v.front());
    limiter_.setFootprint(footprint_p, footprint_radius);
    ROS_INFO("footprint radius: %0.3f", footprint_radius);

    diag_updater_.setHardwareID("none");
    diag_updater_.add("Collision", this, &SafetyLimiterNode::diagnoseCollision);
  }
  void spin()
  {
    ros::Timer predict_timer =
        nh_.createTimer(ros::Duration(1.0 / hz_), &SafetyLimiterNode::cbPredictTimer, this);

    if (watchdog_interval_ != ros::Duration(0.0))
    {
      watchdog_timer_ =
          nh_.createTimer(watchdog_interval_, &SafetyLimiterNode::cbWatchdogTimer, this);
    }

    ros::spin();
  }

protected:
  void cbWatchdogReset(const std_msgs::Empty::ConstPtr& /* msg */)
  {
    watchdog_timer_.setPeriod(watchdog_interval_, true);
    watchdog_stop_ = false;
  }
  void cbWatchdogTimer(const ros::TimerEvent& /* event */)
  {
    ROS_WARN_THROTTLE(1.0, "safety_limiter: Watchdog timed-out");
    watchdog_stop_ = true;
    r_lim_ = 0;
    geometry_msgs::Twist cmd_vel;
    pub_twist_.publish(cmd_vel);

    diag_updater_.force_update();
  }
  void cbPredictTimer(const ros::TimerEvent& /* event */)
  {
    if (!has_twist_)
      return;
    if (!has_cloud_)
      return;

    if (ros::Time::now() - last_cloud_stamp_ > ros::Duration(timeout_))
    {
      ROS_WARN_THROTTLE(1.0, "safety_limiter: PointCloud timed-out");
      geometry_msgs::Twist cmd_vel;
      pub_twist_.publish(cmd_vel);

      cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      has_cloud_ = false;
      r_lim_ = 0;

      diag_updater_.force_update();
      return;
    }

    ros::Time now = ros::Time::now();
    const SafetyLimiter::PredictResult result = limiter_.predict(twist_, cloud_accum_);
    if (result.has_collision_points)
      pub_cloud_.publish(result.collision_points);
    const double r_lim_current = result.r_lim;

    if (r_lim_current < r_lim_)
      r_lim_ = r_lim_current;

    if (r_lim_current < 1.0)
      hold_off_ = now + hold_;

    cloud_clear_ = true;

    diag_updater_.force_update();
  }
  void cbParameter(const SafetyLimiterConfig& config, const uint32_t /* level */)
  {
    boost::recursive_mutex::scoped_lock lock(parameter_server_mutex_);
    hz_ = config.freq;
    timeout_ = config.cloud_timeout;
    disable_timeout_ = config.disable_timeout;
    max_values_[0] = config.max_linear_vel;
    max_values_[1] = config.max_angular_vel;
    hold_ = ros::Duration(std::max(config.hold, 1.0 / hz_));

    SafetyLimiter::Parameters params;
    params.vel[0] = config.lin_vel;
    params.acc[0] = config.lin_acc;
    params.vel[1] = config.ang_vel;
    params.acc[1] = config.ang_acc;
    params.z_range[0] = config.z_range_min;
    params.z_range[1] = config.z_range_max;
    params.dt = config.dt;
    params.d_margin = config.d_margin;
    params.d_escape = config.d_escape;
    params.yaw_margin = config.yaw_margin;
    params.yaw_escape = config.yaw_escape;
    params.downsample_grid = config.downsample_grid;
    params.hz = config.freq;
    params.allow_empty_cloud = config.allow_empty_cloud;
    limiter_.setParameters(params);

    r_lim_ = 1.0;
  }

  geometry_msgs::Twist
  limit(const geometry_msgs::Twist& in)
  {
    auto out = in;
    if (r_lim_ < 1.0 - EPSILON)
    {
      out.linear.x *= r_lim_;
      out.linear.y *= r_lim_;
      out.angular.z *= r_lim_;
      if (std::abs(in.linear.x - out.linear.x) > EPSILON ||
          std::abs(in.linear.y - out.linear.y) > EPSILON ||
          std::abs(in.angular.z - out.angular.z) > EPSILON)
      {
        ROS_WARN_THROTTLE(
            1.0, "safety_limiter: (%0.2f, %0.2f, %0.2f)->(%0.2f, %0.2f, %0.2f)",
            in.linear.x, in.linear.y, in.angular.z,
            out.linear.x, out.linear.y, out.angular.z);
      }
    }
    return out;
  }

  geometry_msgs::Twist
  limitMaxVelocities(const geometry_msgs::Twist& in)
  {
    auto out = in;
    if (max_values_[0] <= 0.0)
    {
      out.linear.x = 0;
      out.linear.y = 0;
    }
    else
    {
      const double out_linear_vel = std::hypot(out.linear.x, out.linear.y);
      if (out_linear_vel > max_values_[0])
      {
        const double vel_ratio = max_values_[0] / out_linear_vel;
        out.linear.x *= vel_ratio;
        out.linear.y *= vel_ratio;
      }
    }
    out.angular.z = (out.angular.z > 0) ?
                        std::min(out.angular.z, max_values_[1]) :
                        std::max(out.angular.z, -max_values_[1]);

    return out;
  }

  void cbTwist(const geometry_msgs::Twist::ConstPtr& msg)
  {
    ros::Time now = ros::Time::now();

    twist_ = *msg;
    has_twist_ = true;

    if (now - last_disable_cmd_ < ros::Duration(disable_timeout_))
    {
      pub_twist_.publish(limitMaxVelocities(twist_));
    }
    else if (!has_cloud_ || watchdog_stop_)
    {
      geometry_msgs::Twist cmd_vel;
      pub_twist_.publish(cmd_vel);
    }
    else
    {
      geometry_msgs::Twist cmd_vel = limitMaxVelocities(limit(twist_));
      pub_twist_.publish(cmd_vel);

      if (now > hold_off_)
        r_lim_ = 1.0;
    }
  }

  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    const bool can_transform = tfbuf_.canTransform(
        fixed_frame_id_, msg->header.frame_id, msg->header.stamp);
    const ros::Time stamp =
        can_transform ? msg->header.stamp : ros::Time(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fixed(new pcl::PointCloud<pcl::PointXYZ>());
    if (!msg->data.empty())
    {
      sensor_msgs::PointCloud2 cloud_msg_fixed;
      try
      {
        const geometry_msgs::TransformStamped cloud_to_fixed =
            tfbuf_.lookupTransform(fixed_frame_id_, msg->header.frame_id, stamp);
        tf2::doTransform(*msg, cloud_msg_fixed, cloud_to_fixed);
      }
      catch (tf2::TransformException& e)
      {
        ROS_WARN_THROTTLE(1.0, "safety_limiter: Transform failed: %s", e.what());
        return;
      }

      cloud_fixed->header.frame_id = fixed_frame_id_;
      pcl::fromROSMsg(cloud_msg_fixed, *cloud_fixed);
    }

    if (cloud_clear_)
    {
      cloud_clear_ = false;
      cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    *cloud_accum_ += *cloud_fixed;
    cloud_accum_->header.frame_id = fixed_frame_id_;
    last_cloud_stamp_ = msg->header.stamp;
    has_cloud_ = true;
  }
  void cbDisable(const std_msgs::Bool::ConstPtr& msg)
  {
    if (msg->data)
    {
      last_disable_cmd_ = ros::Time::now();
    }
  }

  void diagnoseCollision(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    safety_limiter_msgs::SafetyLimiterStatus status_msg;

    if (!has_cloud_ || watchdog_stop_)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Stopped due to data timeout.");
    }
    else if (r_lim_ == 1.0)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
    else if (r_lim_ < EPSILON)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                   (limiter_.hasCollisionAtNow()) ?
                       "Cannot escape from collision." :
                       "Trying to avoid collision, but cannot move anymore.");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   (limiter_.hasCollisionAtNow()) ?
                       "Escaping from collision." :
                       "Reducing velocity to avoid collision.");
    }
    stat.addf("Velocity Limit Ratio", "%.2f", r_lim_);
    stat.add("Pointcloud Availability", has_cloud_ ? "true" : "false");
    stat.add("Watchdog Timeout", watchdog_stop_ ? "true" : "false");

    status_msg.limit_ratio = r_lim_;
    status_msg.is_cloud_available = has_cloud_;
    status_msg.has_watchdog_timed_out = watchdog_stop_;
    status_msg.stuck_started_since = limiter_.stuckStartedSince();

    pub_status_.publish(status_msg);
  }
};

}  // namespace safety_limiter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_limiter");

  safety_limiter::SafetyLimiterNode limiter;
  limiter.spin();

  return 0;
}
