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
#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <neonavigation_common/compatibility.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/TrajectoryTrackerStatus.h>

#include <trajectory_tracker/TrajectoryTrackerConfig.h>
#include <trajectory_tracker/tracker_controller.h>

namespace trajectory_tracker
{
class TrackerNode
{
public:
  TrackerNode();
  ~TrackerNode();
  void spin();

private:
  std::string topic_path_;
  std::string topic_cmd_vel_;
  double hz_;
  double max_dt_;

  ros::Subscriber sub_path_;
  ros::Subscriber sub_path_velocity_;
  ros::Subscriber sub_vel_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_vel_;
  ros::Publisher pub_status_;
  ros::Publisher pub_tracking_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  ros::Timer odom_timeout_timer_;
  double odom_timeout_sec_;

  trajectory_tracker::TrackerController controller_;

  mutable boost::recursive_mutex parameter_server_mutex_;
  dynamic_reconfigure::Server<TrajectoryTrackerConfig> parameter_server_;

  bool use_odom_;
  bool predict_odom_;
  ros::Time prev_odom_stamp_;

  template <typename MSG_TYPE>
  void cbPath(const typename MSG_TYPE::ConstPtr&);
  void cbSpeed(const std_msgs::Float32::ConstPtr&);
  void cbOdometry(const nav_msgs::Odometry::ConstPtr&);
  void cbTimer(const ros::TimerEvent&);
  void cbOdomTimeout(const ros::TimerEvent&);
  void cbParameter(const TrajectoryTrackerConfig& config, const uint32_t /* level */);
};

TrackerNode::TrackerNode()
  : nh_()
  , pnh_("~")
  , tfl_(tfbuf_)
  , controller_(tfbuf_)
{
  neonavigation_common::compat::checkCompatMode();
  std::string frame_robot;
  std::string frame_odom;
  pnh_.param("frame_robot", frame_robot, std::string("base_link"));
  pnh_.param("frame_odom", frame_odom, std::string("odom"));
  controller_.setFrames(frame_robot, frame_odom);
  neonavigation_common::compat::deprecatedParam(pnh_, "path", topic_path_, std::string("path"));
  neonavigation_common::compat::deprecatedParam(pnh_, "cmd_vel", topic_cmd_vel_, std::string("cmd_vel"));
  pnh_.param("hz", hz_, 50.0);
  pnh_.param("use_odom", use_odom_, false);
  pnh_.param("predict_odom", predict_odom_, true);
  pnh_.param("max_dt", max_dt_, 0.1);
  pnh_.param("odom_timeout_sec", odom_timeout_sec_, 0.1);

  sub_path_ = neonavigation_common::compat::subscribe<nav_msgs::Path>(
      nh_, "path",
      pnh_, topic_path_, 2,
      boost::bind(&TrackerNode::cbPath<nav_msgs::Path>, this, _1));
  sub_path_velocity_ = nh_.subscribe<trajectory_tracker_msgs::PathWithVelocity>(
      "path_velocity", 2,
      boost::bind(&TrackerNode::cbPath<trajectory_tracker_msgs::PathWithVelocity>, this, _1));
  sub_vel_ = neonavigation_common::compat::subscribe(
      nh_, "speed",
      pnh_, "speed", 20, &TrackerNode::cbSpeed, this);
  pub_vel_ = neonavigation_common::compat::advertise<geometry_msgs::Twist>(
      nh_, "cmd_vel",
      pnh_, topic_cmd_vel_, 10);
  pub_status_ = pnh_.advertise<trajectory_tracker_msgs::TrajectoryTrackerStatus>("status", 10, true);
  pub_tracking_ = pnh_.advertise<geometry_msgs::PoseStamped>("tracking", 10, true);
  if (use_odom_)
  {
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &TrackerNode::cbOdometry, this,
                                                  ros::TransportHints().reliable().tcpNoDelay(true));
  }

  boost::recursive_mutex::scoped_lock lock(parameter_server_mutex_);
  parameter_server_.setCallback(boost::bind(&TrackerNode::cbParameter, this, _1, _2));
}

void TrackerNode::cbParameter(const TrajectoryTrackerConfig& config, const uint32_t /* level */)
{
  boost::recursive_mutex::scoped_lock lock(parameter_server_mutex_);
  trajectory_tracker::TrackerController::Parameters params;
  params.look_forward = config.look_forward;
  params.curv_forward = config.curv_forward;
  params.k[0] = config.k_dist;
  params.k[1] = config.k_ang;
  params.k[2] = config.k_avel;
  params.gain_at_vel = config.gain_at_vel;
  params.d_lim = config.dist_lim;
  params.d_stop = config.dist_stop;
  params.rotate_ang = config.rotate_ang;
  params.vel[0] = config.max_vel;
  params.vel[1] = config.max_angvel;
  params.acc[0] = config.max_acc;
  params.acc[1] = config.max_angacc;
  params.acc_toc[0] = params.acc[0] * config.acc_toc_factor;
  params.acc_toc[1] = params.acc[1] * config.angacc_toc_factor;
  params.path_step = config.path_step;
  params.goal_tolerance_dist = config.goal_tolerance_dist;
  params.goal_tolerance_ang = config.goal_tolerance_ang;
  params.stop_tolerance_dist = config.stop_tolerance_dist;
  params.stop_tolerance_ang = config.stop_tolerance_ang;
  params.no_pos_cntl_dist = config.no_position_control_dist;
  params.min_track_path = config.min_tracking_path;
  params.allow_backward = config.allow_backward;
  params.limit_vel_by_avel = config.limit_vel_by_avel;
  params.check_old_path = config.check_old_path;
  params.epsilon = config.epsilon;
  params.use_time_optimal_control = config.use_time_optimal_control;
  params.time_optimal_control_future_gain = config.time_optimal_control_future_gain;
  params.k_ang_rotation = config.k_ang_rotation;
  params.k_avel_rotation = config.k_avel_rotation;
  params.goal_tolerance_lin_vel = config.goal_tolerance_lin_vel;
  params.goal_tolerance_ang_vel = config.goal_tolerance_ang_vel;
  controller_.setParameters(params);
}

TrackerNode::~TrackerNode()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub_vel_.publish(cmd_vel);
}

void TrackerNode::cbSpeed(const std_msgs::Float32::ConstPtr& msg)
{
  controller_.setSpeed(msg->data);
}

template <typename MSG_TYPE>
void TrackerNode::cbPath(const typename MSG_TYPE::ConstPtr& msg)
{
  controller_.setPath(*msg);
}

void TrackerNode::cbOdometry(const nav_msgs::Odometry::ConstPtr& odom)
{
  if (odom->header.frame_id != controller_.frameOdom())
  {
    ROS_WARN("frame_odom is invalid. Update from \"%s\" to \"%s\"",
             controller_.frameOdom().c_str(), odom->header.frame_id.c_str());
    controller_.setFrameOdom(odom->header.frame_id);
  }
  if (odom->child_frame_id != controller_.frameRobot())
  {
    ROS_WARN("frame_robot is invalid. Update from \"%s\" to \"%s\"",
             controller_.frameRobot().c_str(), odom->child_frame_id.c_str());
    controller_.setFrameRobot(odom->child_frame_id);
  }
  if (odom_timeout_sec_ != 0.0)
  {
    if (odom_timeout_timer_.isValid())
    {
      odom_timeout_timer_.setPeriod(ros::Duration(odom_timeout_sec_), true);
    }
    else
    {
      odom_timeout_timer_ =
          nh_.createTimer(ros::Duration(odom_timeout_sec_), &TrackerNode::cbOdomTimeout, this, true, true);
    }
  }

  if (prev_odom_stamp_ != ros::Time())
  {
    const double dt = std::min(max_dt_, (odom->header.stamp - prev_odom_stamp_).toSec());
    nav_msgs::Odometry odom_compensated = *odom;
    Eigen::Vector3d prediction_offset(0, 0, 0);
    if (predict_odom_)
    {
      const double predict_dt = std::max(0.0, std::min(max_dt_, (ros::Time::now() - odom->header.stamp).toSec()));
      tf2::Transform trans;
      const tf2::Quaternion rotation(tf2::Vector3(0, 0, 1), odom->twist.twist.angular.z * predict_dt);
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
    const tf2::Stamped<tf2::Transform> odom_to_robot_stamped(odom_to_robot, odom->header.stamp, odom->header.frame_id);
    const trajectory_tracker::TrackerController::ControlOutput output = controller_.control(
        odom_to_robot_stamped, prediction_offset, odom->twist.twist.linear.x, odom->twist.twist.angular.z, dt);
    pub_vel_.publish(output.cmd_vel);
    pub_status_.publish(output.status);
  }
  prev_odom_stamp_ = odom->header.stamp;
}

void TrackerNode::cbTimer(const ros::TimerEvent& /* event */)
{
  try
  {
    tf2::Stamped<tf2::Transform> transform;
    tf2::fromMsg(
        tfbuf_.lookupTransform(controller_.frameOdom(), controller_.frameRobot(), ros::Time(0)), transform);
    const trajectory_tracker::TrackerController::ControlOutput output =
        controller_.control(transform, Eigen::Vector3d(0, 0, 0), 0, 0, 1.0 / hz_);
    pub_vel_.publish(output.cmd_vel);
    pub_status_.publish(output.status);
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN_THROTTLE(1, "TF exception: %s", e.what());
    trajectory_tracker_msgs::TrajectoryTrackerStatus status;
    status.header.stamp = ros::Time::now();
    status.distance_remains = 0.0;
    status.angle_remains = 0.0;
    status.path_header = controller_.pathHeader();
    status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }
}

void TrackerNode::cbOdomTimeout(const ros::TimerEvent& /* event */)
{
  ROS_WARN_STREAM("Odometry timeout. Last odometry stamp: " << prev_odom_stamp_);
  controller_.resetLimiters();
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  pub_vel_.publish(cmd_vel);

  trajectory_tracker_msgs::TrajectoryTrackerStatus status;
  status.header.stamp = ros::Time::now();
  status.distance_remains = 0.0;
  status.angle_remains = 0.0;
  status.path_header = controller_.pathHeader();
  status.status = trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH;
  pub_status_.publish(status);
}

void TrackerNode::spin()
{
  ros::Timer timer;
  if (!use_odom_)
  {
    timer = nh_.createTimer(ros::Duration(1.0 / hz_), &TrackerNode::cbTimer, this);
  }
  ros::spin();
}
}  // namespace trajectory_tracker

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_tracker");
  trajectory_tracker::TrackerNode track;
  track.spin();

  return 0;
}
