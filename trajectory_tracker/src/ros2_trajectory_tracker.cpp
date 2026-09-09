/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
   Original idea of the implemented control scheme was published on:
   S. Iida, S. Yuta, "Vehicle command system and trajectory control for
   autonomous mobile robots," in Proceedings of the 1991 IEEE/RSJ
   International Workshop on Intelligent Robots and Systems (IROS),
   1991, pp. 212-217.
 */

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_tracker/tracker_controller.h"
#include "trajectory_tracker_msgs/msg/path_with_velocity.hpp"
#include "trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp"

namespace trajectory_tracker
{
// ROS 2 interface layer for the trajectory following controller.
// Holds the ROS-independent control logic in TrackerController and connects it
// to ROS 2 publishers, subscribers, parameters and timers. The dynamic
// parameters that were served by dynamic_reconfigure on ROS 1 are provided here
// through declare_parameter() plus a post-set parameter callback.
class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options);
  ~TrackerNode() override;

private:
  double hz_;
  double max_dt_;
  double odom_timeout_sec_;
  bool use_odom_;
  bool predict_odom_;
  rclcpp::Time prev_odom_stamp_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<trajectory_tracker_msgs::msg::PathWithVelocity>::SharedPtr
    sub_path_velocity_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tracking_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr odom_timeout_timer_;

  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  std::unique_ptr<trajectory_tracker::TrackerController> controller_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  void declareDynamicParameters();
  void updateParameters(const std::vector<rclcpp::Parameter> & changed = {});

  template <typename MSG_TYPE>
  void cbPath(const typename MSG_TYPE::ConstSharedPtr & msg);
  void cbSpeed(const std_msgs::msg::Float32::ConstSharedPtr & msg);
  void cbOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void cbTimer();
  void cbOdomTimeout();
  void publish(const trajectory_tracker::TrackerController::ControlOutput & output);
};

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("trajectory_tracker", options), prev_odom_stamp_(0, 0, RCL_ROS_TIME)
{
  tfbuf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
  controller_ =
    std::make_unique<trajectory_tracker::TrackerController>(*tfbuf_, this->get_logger());

  const std::string frame_robot = this->declare_parameter("frame_robot", std::string("base_link"));
  const std::string frame_odom = this->declare_parameter("frame_odom", std::string("odom"));
  controller_->setFrames(frame_robot, frame_odom);

  hz_ = this->declare_parameter("hz", 50.0);
  use_odom_ = this->declare_parameter("use_odom", false);
  predict_odom_ = this->declare_parameter("predict_odom", true);
  max_dt_ = this->declare_parameter("max_dt", 0.1);
  odom_timeout_sec_ = this->declare_parameter("odom_timeout_sec", 0.1);

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

  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", 2, std::bind(&TrackerNode::cbPath<nav_msgs::msg::Path>, this, std::placeholders::_1));
  sub_path_velocity_ = this->create_subscription<trajectory_tracker_msgs::msg::PathWithVelocity>(
    "path_velocity", 2,
    std::bind(
      &TrackerNode::cbPath<trajectory_tracker_msgs::msg::PathWithVelocity>, this,
      std::placeholders::_1));
  sub_vel_ = this->create_subscription<std_msgs::msg::Float32>(
    "speed", 20, std::bind(&TrackerNode::cbSpeed, this, std::placeholders::_1));

  pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pub_status_ = this->create_publisher<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>(
    "~/status", rclcpp::QoS(10).transient_local());
  pub_tracking_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/tracking", rclcpp::QoS(10).transient_local());

  if (use_odom_) {
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&TrackerNode::cbOdometry, this, std::placeholders::_1));
  } else {
    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / hz_),
      std::bind(&TrackerNode::cbTimer, this));
  }
}

TrackerNode::~TrackerNode()
{
  if (pub_vel_) {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = 0.0;
    cmd_vel->angular.z = 0.0;
    pub_vel_->publish(std::move(cmd_vel));
  }
}

void TrackerNode::declareDynamicParameters()
{
  this->declare_parameter("look_forward", 0.5);
  this->declare_parameter("curv_forward", 0.5);
  this->declare_parameter("k_dist", 1.0);
  this->declare_parameter("k_ang", 1.0);
  this->declare_parameter("k_avel", 1.0);
  this->declare_parameter("gain_at_vel", 0.0);
  this->declare_parameter("dist_lim", 0.5);
  this->declare_parameter("dist_stop", 2.0);
  this->declare_parameter("rotate_ang", 0.78539816339);
  this->declare_parameter("max_vel", 0.5);
  this->declare_parameter("max_angvel", 1.0);
  this->declare_parameter("max_acc", 1.0);
  this->declare_parameter("max_angacc", 2.0);
  this->declare_parameter("acc_toc_factor", 0.9);
  this->declare_parameter("angacc_toc_factor", 0.9);
  this->declare_parameter("path_step", 1);
  this->declare_parameter("goal_tolerance_dist", 0.2);
  this->declare_parameter("goal_tolerance_ang", 0.1);
  this->declare_parameter("stop_tolerance_dist", 0.1);
  this->declare_parameter("stop_tolerance_ang", 0.05);
  this->declare_parameter("no_position_control_dist", 0.0);
  this->declare_parameter("min_tracking_path", 0.0);
  this->declare_parameter("allow_backward", true);
  this->declare_parameter("limit_vel_by_avel", false);
  this->declare_parameter("check_old_path", false);
  this->declare_parameter("epsilon", 0.001);
  this->declare_parameter("use_time_optimal_control", true);
  this->declare_parameter("time_optimal_control_future_gain", 1.5);
  this->declare_parameter("k_ang_rotation", 1.0);
  this->declare_parameter("k_avel_rotation", 1.0);
  this->declare_parameter("goal_tolerance_lin_vel", 0.0);
  this->declare_parameter("goal_tolerance_ang_vel", 0.0);
}

void TrackerNode::updateParameters(const std::vector<rclcpp::Parameter> & changed)
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
  trajectory_tracker::TrackerController::Parameters params;
  params.look_forward = param("look_forward").as_double();
  params.curv_forward = param("curv_forward").as_double();
  params.k[0] = param("k_dist").as_double();
  params.k[1] = param("k_ang").as_double();
  params.k[2] = param("k_avel").as_double();
  params.gain_at_vel = param("gain_at_vel").as_double();
  params.d_lim = param("dist_lim").as_double();
  params.d_stop = param("dist_stop").as_double();
  params.rotate_ang = param("rotate_ang").as_double();
  params.vel[0] = param("max_vel").as_double();
  params.vel[1] = param("max_angvel").as_double();
  params.acc[0] = param("max_acc").as_double();
  params.acc[1] = param("max_angacc").as_double();
  params.acc_toc[0] = params.acc[0] * param("acc_toc_factor").as_double();
  params.acc_toc[1] = params.acc[1] * param("angacc_toc_factor").as_double();
  params.path_step = static_cast<int>(param("path_step").as_int());
  params.goal_tolerance_dist = param("goal_tolerance_dist").as_double();
  params.goal_tolerance_ang = param("goal_tolerance_ang").as_double();
  params.stop_tolerance_dist = param("stop_tolerance_dist").as_double();
  params.stop_tolerance_ang = param("stop_tolerance_ang").as_double();
  params.no_pos_cntl_dist = param("no_position_control_dist").as_double();
  params.min_track_path = param("min_tracking_path").as_double();
  params.allow_backward = param("allow_backward").as_bool();
  params.limit_vel_by_avel = param("limit_vel_by_avel").as_bool();
  params.check_old_path = param("check_old_path").as_bool();
  params.epsilon = param("epsilon").as_double();
  params.use_time_optimal_control = param("use_time_optimal_control").as_bool();
  params.time_optimal_control_future_gain = param("time_optimal_control_future_gain").as_double();
  params.k_ang_rotation = param("k_ang_rotation").as_double();
  params.k_avel_rotation = param("k_avel_rotation").as_double();
  params.goal_tolerance_lin_vel = param("goal_tolerance_lin_vel").as_double();
  params.goal_tolerance_ang_vel = param("goal_tolerance_ang_vel").as_double();
  controller_->setParameters(params);
}

void TrackerNode::cbSpeed(const std_msgs::msg::Float32::ConstSharedPtr & msg)
{
  controller_->setSpeed(msg->data);
}

template <typename MSG_TYPE>
void TrackerNode::cbPath(const typename MSG_TYPE::ConstSharedPtr & msg)
{
  controller_->setPath(*msg);
}

void TrackerNode::publish(const trajectory_tracker::TrackerController::ControlOutput & output)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(output.cmd_vel);
  pub_vel_->publish(std::move(cmd_vel));
  auto status =
    std::make_unique<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>(output.status);
  pub_status_->publish(std::move(status));
}

void TrackerNode::cbOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & odom)
{
  if (odom->header.frame_id != controller_->frameOdom()) {
    RCLCPP_WARN(
      this->get_logger(), "frame_odom is invalid. Update from \"%s\" to \"%s\"",
      controller_->frameOdom().c_str(), odom->header.frame_id.c_str());
    controller_->setFrameOdom(odom->header.frame_id);
  }
  if (odom->child_frame_id != controller_->frameRobot()) {
    RCLCPP_WARN(
      this->get_logger(), "frame_robot is invalid. Update from \"%s\" to \"%s\"",
      controller_->frameRobot().c_str(), odom->child_frame_id.c_str());
    controller_->setFrameRobot(odom->child_frame_id);
  }
  if (odom_timeout_sec_ != 0.0) {
    if (odom_timeout_timer_) {
      odom_timeout_timer_->cancel();
    }
    odom_timeout_timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(odom_timeout_sec_),
      std::bind(&TrackerNode::cbOdomTimeout, this));
  }

  const rclcpp::Time odom_stamp(odom->header.stamp);
  if (prev_odom_stamp_.nanoseconds() != 0) {
    const double dt = std::min(max_dt_, (odom_stamp - prev_odom_stamp_).seconds());
    nav_msgs::msg::Odometry odom_compensated = *odom;
    Eigen::Vector3d prediction_offset(0, 0, 0);
    if (predict_odom_) {
      const double predict_dt =
        std::max(0.0, std::min(max_dt_, (this->now() - odom_stamp).seconds()));
      tf2::Transform trans;
      const tf2::Quaternion rotation(
        tf2::Vector3(0, 0, 1), odom->twist.twist.angular.z * predict_dt);
      const tf2::Vector3 translation(odom->twist.twist.linear.x * predict_dt, 0, 0);

      prediction_offset[0] = odom->twist.twist.linear.x * predict_dt;
      prediction_offset[2] = odom->twist.twist.angular.z * predict_dt;

      tf2::fromMsg(odom->pose.pose, trans);
      trans.setOrigin(trans.getOrigin() + tf2::Transform(trans.getRotation()) * translation);
      trans.setRotation(trans.getRotation() * rotation);
      tf2::toMsg(trans, odom_compensated.pose.pose);
    }

    tf2::Transform odom_to_robot;
    tf2::fromMsg(odom_compensated.pose.pose, odom_to_robot);
    const tf2::Stamped<tf2::Transform> odom_to_robot_stamped(
      odom_to_robot, tf2_ros::fromMsg(odom_stamp), odom->header.frame_id);
    const trajectory_tracker::TrackerController::ControlOutput output = controller_->control(
      odom_to_robot_stamped, prediction_offset, odom->twist.twist.linear.x,
      odom->twist.twist.angular.z, dt);
    publish(output);
  }
  prev_odom_stamp_ = odom_stamp;
}

void TrackerNode::cbTimer()
{
  try {
    tf2::Stamped<tf2::Transform> transform;
    tf2::fromMsg(
      tfbuf_->lookupTransform(
        controller_->frameOdom(), controller_->frameRobot(), tf2::TimePointZero),
      transform);
    const trajectory_tracker::TrackerController::ControlOutput output =
      controller_->control(transform, Eigen::Vector3d(0, 0, 0), 0, 0, 1.0 / hz_);
    publish(output);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "TF exception: %s", e.what());
    auto status = std::make_unique<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>();
    status->header.stamp = this->now();
    status->distance_remains = 0.0;
    status->angle_remains = 0.0;
    status->path_header = controller_->pathHeader();
    status->status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
    pub_status_->publish(std::move(status));
    return;
  }
}

void TrackerNode::cbOdomTimeout()
{
  RCLCPP_WARN(
    this->get_logger(), "Odometry timeout. Last odometry stamp: %f", prev_odom_stamp_.seconds());
  controller_->resetLimiters();
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = 0.0;
  cmd_vel->angular.z = 0.0;
  pub_vel_->publish(std::move(cmd_vel));

  auto status = std::make_unique<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>();
  status->header.stamp = this->now();
  status->distance_remains = 0.0;
  status->angle_remains = 0.0;
  status->path_header = controller_->pathHeader();
  status->status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::NO_PATH;
  pub_status_->publish(std::move(status));
}
}  // namespace trajectory_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_tracker::TrackerNode)
