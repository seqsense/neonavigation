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

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "safety_limiter/safety_limiter.h"
#include "safety_limiter_msgs/msg/safety_limiter_status.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace safety_limiter
{
// ROS 2 interface layer for the collision-prevention motion limiter.
// The ROS-independent prediction logic lives in SafetyLimiter; this node wires
// it to ROS 2 publishers, subscribers, timers, TF and parameters. The tunables
// that were served by dynamic_reconfigure on ROS 1 are exposed here through
// declare_parameter() plus a post-set parameter callback. The footprint, read
// from an XmlRpc array on ROS 1, is provided here as a flat [x0, y0, x1, y1, ...]
// double array parameter (ROS 2 parameters cannot hold nested arrays).
class SafetyLimiterNode : public rclcpp::Node
{
public:
  explicit SafetyLimiterNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_cloud_;
  rclcpp::Publisher<safety_limiter_msgs::msg::SafetyLimiterStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_clouds_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_disable_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_watchdog_;
  rclcpp::TimerBase::SharedPtr predict_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  std::unique_ptr<SafetyLimiter> limiter_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  geometry_msgs::msg::Twist twist_;
  rclcpp::Time last_cloud_stamp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accum_;
  bool cloud_clear_;
  double hz_;
  double timeout_;
  double disable_timeout_;
  double r_lim_;
  double max_values_[2];
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  rclcpp::Time last_disable_cmd_;
  rclcpp::Duration hold_;
  rclcpp::Time hold_off_;
  double watchdog_interval_d_;

  bool watchdog_stop_;
  bool has_cloud_;
  bool has_twist_;

  constexpr static float EPSILON = 1e-6;

  diagnostic_updater::Updater diag_updater_;

  void declareDynamicParameters();
  void updateParameters(const std::vector<rclcpp::Parameter> & changed = {});
  void setFootprint();

  void cbWatchdogReset(const std_msgs::msg::Empty::ConstSharedPtr & msg);
  void cbWatchdogTimer();
  void cbPredictTimer();
  geometry_msgs::msg::Twist limit(const geometry_msgs::msg::Twist & in);
  geometry_msgs::msg::Twist limitMaxVelocities(const geometry_msgs::msg::Twist & in);
  void cbTwist(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);
  void cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void cbDisable(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void diagnoseCollision(diagnostic_updater::DiagnosticStatusWrapper & stat);
};

SafetyLimiterNode::SafetyLimiterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("safety_limiter", options),
  last_cloud_stamp_(0, 0, RCL_ROS_TIME),
  cloud_accum_(new pcl::PointCloud<pcl::PointXYZ>),
  cloud_clear_(false),
  hz_(6.0),
  r_lim_(1.0),
  last_disable_cmd_(0, 0, RCL_ROS_TIME),
  hold_(0, 0),
  hold_off_(0, 0, RCL_ROS_TIME),
  watchdog_stop_(false),
  has_cloud_(false),
  has_twist_(true),
  diag_updater_(this)
{
  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
  limiter_ = std::make_unique<SafetyLimiter>(*tfbuf_, this->get_logger());
  // Time is read through the node's clock so the logic follows /clock
  // when use_sim_time is set.
  limiter_->setClock(this->get_clock());

  pub_twist_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1).transient_local());
  pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
    "collision", rclcpp::QoS(1).transient_local());
  pub_status_ = this->create_publisher<safety_limiter_msgs::msg::SafetyLimiterStatus>(
    "~/status", rclcpp::QoS(1).transient_local());

  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 1, std::bind(&SafetyLimiterNode::cbTwist, this, std::placeholders::_1));
  sub_disable_ = this->create_subscription<std_msgs::msg::Bool>(
    "disable_safety", 1, std::bind(&SafetyLimiterNode::cbDisable, this, std::placeholders::_1));
  sub_watchdog_ = this->create_subscription<std_msgs::msg::Empty>(
    "watchdog_reset", 1,
    std::bind(&SafetyLimiterNode::cbWatchdogReset, this, std::placeholders::_1));

  const int num_input_clouds = static_cast<int>(this->declare_parameter("num_input_clouds", 1));
  if (num_input_clouds == 1) {
    sub_clouds_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", 1, std::bind(&SafetyLimiterNode::cbCloud, this, std::placeholders::_1)));
  } else {
    for (int i = 0; i < num_input_clouds; ++i) {
      sub_clouds_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud" + std::to_string(i), 1,
        std::bind(&SafetyLimiterNode::cbCloud, this, std::placeholders::_1)));
    }
  }

  // Obsolete parameter warning, mirroring the ROS 1 node. t_margin is never
  // declared, so a user-provided value only shows up in the parameter overrides.
  for (const auto & param_override : this->get_node_options().parameter_overrides()) {
    if (param_override.get_name() == "t_margin") {
      RCLCPP_WARN(
        this->get_logger(),
        "safety_limiter: t_margin parameter is obsolated. Use d_margin and yaw_margin instead.");
      break;
    }
  }

  base_frame_id_ = this->declare_parameter("base_frame", std::string("base_link"));
  fixed_frame_id_ = this->declare_parameter("fixed_frame", std::string("odom"));
  limiter_->setBaseFrame(base_frame_id_);
  watchdog_interval_d_ = this->declare_parameter("watchdog_interval", 0.0);

  setFootprint();

  declareDynamicParameters();
  updateParameters();
  param_callback_handle_ =
    this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
      // This node registers no other callback, so nothing downstream can
      // reject the change after the state has been updated here.
      updateParameters(params);
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  diag_updater_.setHardwareID("none");
  diag_updater_.add("Collision", this, &SafetyLimiterNode::diagnoseCollision);

  predict_timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / hz_),
    std::bind(&SafetyLimiterNode::cbPredictTimer, this));

  if (watchdog_interval_d_ != 0.0) {
    watchdog_timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(watchdog_interval_d_),
      std::bind(&SafetyLimiterNode::cbWatchdogTimer, this));
  }
}

void SafetyLimiterNode::setFootprint()
{
  // ROS 1 read the footprint from an XmlRpc array of [x, y] pairs. ROS 2
  // parameters cannot hold nested arrays, so the footprint is provided as a
  // flat double array: [x0, y0, x1, y1, ...] with at least three vertices.
  const std::vector<double> footprint_flat =
    this->declare_parameter("footprint", std::vector<double>());
  if (footprint_flat.empty()) {
    RCLCPP_FATAL(this->get_logger(), "Footprint doesn't specified");
    throw std::runtime_error("Footprint doesn't specified");
  }
  if (footprint_flat.size() % 2 != 0 || footprint_flat.size() < 6) {
    RCLCPP_FATAL(this->get_logger(), "Invalid footprint");
    throw std::runtime_error("Invalid footprint");
  }

  polygon footprint_p;
  float footprint_radius = 0;
  for (size_t i = 0; i < footprint_flat.size(); i += 2) {
    vec v;
    v[0] = footprint_flat[i];
    v[1] = footprint_flat[i + 1];
    footprint_p.v.push_back(v);

    const float dist = std::hypot(v[0], v[1]);
    if (dist > footprint_radius) footprint_radius = dist;
  }
  footprint_p.v.push_back(footprint_p.v.front());
  limiter_->setFootprint(footprint_p, footprint_radius);
  RCLCPP_INFO(this->get_logger(), "footprint radius: %0.3f", footprint_radius);
}

void SafetyLimiterNode::declareDynamicParameters()
{
  this->declare_parameter("freq", 6.0);
  this->declare_parameter("cloud_timeout", 0.8);
  this->declare_parameter("disable_timeout", 0.1);
  this->declare_parameter("lin_vel", 0.5);
  this->declare_parameter("lin_acc", 1.0);
  this->declare_parameter("max_linear_vel", 10.0);
  this->declare_parameter("ang_vel", 0.8);
  this->declare_parameter("ang_acc", 1.6);
  this->declare_parameter("max_angular_vel", 10.0);
  this->declare_parameter("z_range_min", 0.0);
  this->declare_parameter("z_range_max", 0.5);
  this->declare_parameter("dt", 0.1);
  this->declare_parameter("d_margin", 0.2);
  this->declare_parameter("d_escape", 0.05);
  this->declare_parameter("yaw_margin", 0.2);
  this->declare_parameter("yaw_escape", 0.05);
  this->declare_parameter("downsample_grid", 0.05);
  this->declare_parameter("hold", 0.0);
  this->declare_parameter("allow_empty_cloud", false);
}

void SafetyLimiterNode::updateParameters(const std::vector<rclcpp::Parameter> & changed)
{
  // Runs from an on-set callback, i.e. before the new values reach the node's
  // parameter store, because humble's rclcpp has no post-set callback. Read
  // the values that are about to be applied first, and fall back to the store
  // for every parameter the change does not touch.
  const auto param = [this, &changed](const std::string & name) {
    for (const auto & p : changed) {
      if (p.get_name() == name) {
        return p;
      }
    }
    return this->get_parameter(name);
  };
  hz_ = param("freq").as_double();
  timeout_ = param("cloud_timeout").as_double();
  disable_timeout_ = param("disable_timeout").as_double();
  max_values_[0] = param("max_linear_vel").as_double();
  max_values_[1] = param("max_angular_vel").as_double();
  hold_ = rclcpp::Duration::from_seconds(std::max(param("hold").as_double(), 1.0 / hz_));

  SafetyLimiter::Parameters params;
  params.vel[0] = param("lin_vel").as_double();
  params.acc[0] = param("lin_acc").as_double();
  params.vel[1] = param("ang_vel").as_double();
  params.acc[1] = param("ang_acc").as_double();
  params.z_range[0] = param("z_range_min").as_double();
  params.z_range[1] = param("z_range_max").as_double();
  params.dt = param("dt").as_double();
  params.d_margin = param("d_margin").as_double();
  params.d_escape = param("d_escape").as_double();
  params.yaw_margin = param("yaw_margin").as_double();
  params.yaw_escape = param("yaw_escape").as_double();
  params.downsample_grid = param("downsample_grid").as_double();
  params.hz = hz_;
  params.allow_empty_cloud = param("allow_empty_cloud").as_bool();
  limiter_->setParameters(params);

  r_lim_ = 1.0;
}

void SafetyLimiterNode::cbWatchdogReset(const std_msgs::msg::Empty::ConstSharedPtr & /* msg */)
{
  if (watchdog_timer_) {
    watchdog_timer_->reset();
  }
  watchdog_stop_ = false;
}

void SafetyLimiterNode::cbWatchdogTimer()
{
  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Watchdog timed-out");
  watchdog_stop_ = true;
  r_lim_ = 0;
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  pub_twist_->publish(std::move(cmd_vel));

  diag_updater_.force_update();
}

void SafetyLimiterNode::cbPredictTimer()
{
  if (!has_twist_) return;
  if (!has_cloud_) return;

  if (this->now() - last_cloud_stamp_ > rclcpp::Duration::from_seconds(timeout_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "safety_limiter: PointCloud timed-out");
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    pub_twist_->publish(std::move(cmd_vel));

    cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    has_cloud_ = false;
    r_lim_ = 0;

    diag_updater_.force_update();
    return;
  }

  const rclcpp::Time now = this->now();
  const SafetyLimiter::PredictResult result = limiter_->predict(twist_, cloud_accum_);
  if (result.has_collision_points) {
    auto col_points = std::make_unique<sensor_msgs::msg::PointCloud>(result.collision_points);
    pub_cloud_->publish(std::move(col_points));
  }
  const double r_lim_current = result.r_lim;

  if (r_lim_current < r_lim_) r_lim_ = r_lim_current;

  if (r_lim_current < 1.0) hold_off_ = now + hold_;

  cloud_clear_ = true;

  diag_updater_.force_update();
}

geometry_msgs::msg::Twist SafetyLimiterNode::limit(const geometry_msgs::msg::Twist & in)
{
  auto out = in;
  if (r_lim_ < 1.0 - EPSILON) {
    out.linear.x *= r_lim_;
    out.linear.y *= r_lim_;
    out.angular.z *= r_lim_;
    if (
      std::abs(in.linear.x - out.linear.x) > EPSILON ||
      std::abs(in.linear.y - out.linear.y) > EPSILON ||
      std::abs(in.angular.z - out.angular.z) > EPSILON) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "safety_limiter: (%0.2f, %0.2f, %0.2f)->(%0.2f, %0.2f, %0.2f)", in.linear.x, in.linear.y,
        in.angular.z, out.linear.x, out.linear.y, out.angular.z);
    }
  }
  return out;
}

geometry_msgs::msg::Twist SafetyLimiterNode::limitMaxVelocities(
  const geometry_msgs::msg::Twist & in)
{
  auto out = in;
  if (max_values_[0] <= 0.0) {
    out.linear.x = 0;
    out.linear.y = 0;
  } else {
    const double out_linear_vel = std::hypot(out.linear.x, out.linear.y);
    if (out_linear_vel > max_values_[0]) {
      const double vel_ratio = max_values_[0] / out_linear_vel;
      out.linear.x *= vel_ratio;
      out.linear.y *= vel_ratio;
    }
  }
  out.angular.z = (out.angular.z > 0) ? std::min(out.angular.z, max_values_[1])
                                      : std::max(out.angular.z, -max_values_[1]);

  return out;
}

void SafetyLimiterNode::cbTwist(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  const rclcpp::Time now = this->now();

  twist_ = *msg;
  has_twist_ = true;

  if (now - last_disable_cmd_ < rclcpp::Duration::from_seconds(disable_timeout_)) {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(limitMaxVelocities(twist_));
    pub_twist_->publish(std::move(cmd_vel));
  } else if (!has_cloud_ || watchdog_stop_) {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    pub_twist_->publish(std::move(cmd_vel));
  } else {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(limitMaxVelocities(limit(twist_)));
    pub_twist_->publish(std::move(cmd_vel));

    if (now > hold_off_) r_lim_ = 1.0;
  }
}

void SafetyLimiterNode::cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const bool can_transform =
    tfbuf_->canTransform(fixed_frame_id_, msg->header.frame_id, msg->header.stamp);
  const rclcpp::Time stamp = can_transform ? rclcpp::Time(msg->header.stamp) : rclcpp::Time(0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fixed(new pcl::PointCloud<pcl::PointXYZ>());
  if (!msg->data.empty()) {
    sensor_msgs::msg::PointCloud2 cloud_msg_fixed;
    try {
      const geometry_msgs::msg::TransformStamped cloud_to_fixed =
        tfbuf_->lookupTransform(fixed_frame_id_, msg->header.frame_id, stamp);
      tf2::doTransform(*msg, cloud_msg_fixed, cloud_to_fixed);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "safety_limiter: Transform failed: %s",
        e.what());
      return;
    }

    cloud_fixed->header.frame_id = fixed_frame_id_;
    pcl::fromROSMsg(cloud_msg_fixed, *cloud_fixed);
  }

  if (cloud_clear_) {
    cloud_clear_ = false;
    cloud_accum_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }
  *cloud_accum_ += *cloud_fixed;
  cloud_accum_->header.frame_id = fixed_frame_id_;
  last_cloud_stamp_ = msg->header.stamp;
  has_cloud_ = true;
}

void SafetyLimiterNode::cbDisable(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  if (msg->data) {
    last_disable_cmd_ = this->now();
  }
}

void SafetyLimiterNode::diagnoseCollision(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  auto status_msg = std::make_unique<safety_limiter_msgs::msg::SafetyLimiterStatus>();

  if (!has_cloud_ || watchdog_stop_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Stopped due to data timeout.");
  } else if (r_lim_ == 1.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  } else if (r_lim_ < EPSILON) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      (limiter_->hasCollisionAtNow()) ? "Cannot escape from collision."
                                      : "Trying to avoid collision, but cannot move anymore.");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK, (limiter_->hasCollisionAtNow())
                                                    ? "Escaping from collision."
                                                    : "Reducing velocity to avoid collision.");
  }
  stat.addf("Velocity Limit Ratio", "%.2f", r_lim_);
  stat.add("Pointcloud Availability", has_cloud_ ? "true" : "false");
  stat.add("Watchdog Timeout", watchdog_stop_ ? "true" : "false");

  status_msg->limit_ratio = r_lim_;
  status_msg->is_cloud_available = has_cloud_;
  status_msg->has_watchdog_timed_out = watchdog_stop_;
  status_msg->stuck_started_since = limiter_->stuckStartedSince();

  pub_status_->publish(std::move(status_msg));
}
}  // namespace safety_limiter

RCLCPP_COMPONENTS_REGISTER_NODE(safety_limiter::SafetyLimiterNode)
