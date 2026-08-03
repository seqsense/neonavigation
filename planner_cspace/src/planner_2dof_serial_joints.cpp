/*
 * Copyright (c) 2014-2025, the neonavigation authors
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

#include <neonavigation_common/compatibility.h>
#include <planner_cspace/planner_2dof_serial_joints/planner_core.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <algorithm>
#include <memory>
#include <sq_ros1_compat/logger.hpp>
#include <string>
#include <utility>
#include <vector>

namespace planner_cspace
{
namespace planner_2dof_serial_joints
{
// ROS interface of the planner_2dof_serial_joints node. All of the trajectory
// planning is delegated to Planner2dofSerialJointsCore.
class Planner2dofSerialJointsNode
{
public:
  using Ptr = std::shared_ptr<Planner2dofSerialJointsNode>;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher pub_status_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_trajectory_;
  ros::Subscriber sub_joint_;

  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  Planner2dofSerialJointsCore planner_;

  std::string group_;

  std::pair<ros::Duration, std::pair<float, float>> cmd_prev_;
  trajectory_msgs::JointTrajectory traj_prev_;
  int id_[2];

  ros::Time replan_prev_;
  ros::Duration replan_interval_;
  bool has_joint_states_;

  void cbJoint(const sensor_msgs::JointState::ConstPtr & msg)
  {
    int id[2] = {-1, -1};
    for (size_t i = 0; i < msg->name.size(); i++) {
      if (msg->name[i].compare(planner_.linkName(0)) == 0)
        id[0] = i;
      else if (msg->name[i].compare(planner_.linkName(1)) == 0)
        id[1] = i;
    }
    if (id[0] == -1 || id[1] == -1) {
      ROS_ERROR("joint_state does not contain link group %s.", group_.c_str());
      return;
    }
    planner_.setCurrentAngles(msg->position[id[0]], msg->position[id[1]]);
    has_joint_states_ = true;

    if (
      (replan_prev_ + replan_interval_ < ros::Time::now() || replan_prev_ == ros::Time(0)) &&
      replan_interval_ > ros::Duration(0)) {
      replan();
    }
  }
  void cbTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr & msg)
  {
    id_[0] = -1;
    id_[1] = -1;
    for (size_t i = 0; i < msg->joint_names.size(); i++) {
      if (msg->joint_names[i].compare(planner_.linkName(0)) == 0)
        id_[0] = i;
      else if (msg->joint_names[i].compare(planner_.linkName(1)) == 0)
        id_[1] = i;
    }
    if (id_[0] == -1 || id_[1] == -1) {
      ROS_ERROR("joint_trajectory does not contains link group %s.", group_.c_str());
      return;
    }
    if (msg->points.size() != 1) {
      ROS_ERROR("single trajectory point required.");
    }
    decltype(cmd_prev_) cmd;
    cmd.first = msg->points[0].time_from_start;
    cmd.second.first = msg->points[0].positions[id_[0]];
    cmd.second.second = msg->points[0].positions[id_[1]];
    if (cmd_prev_ == cmd) return;
    cmd_prev_ = cmd;
    traj_prev_ = *msg;
    planner_.invalidateAvgVel();

    replan();
  }
  void replan()
  {
    if (!has_joint_states_) return;

    replan_prev_ = ros::Time::now();
    if (id_[0] == -1 || id_[1] == -1) return;

    trajectory_msgs::JointTrajectory out;
    planner_.replan(
      static_cast<float>(traj_prev_.points[0].positions[id_[0]]),
      static_cast<float>(traj_prev_.points[0].positions[id_[1]]),
      traj_prev_.points[0].time_from_start, traj_prev_.header, out);
    pub_trajectory_.publish(out);
    if (planner_.takeReplanTimerReset()) replan_prev_ = ros::Time(0);

    planner_.setStatusStamp(ros::Time::now());
    pub_status_.publish(planner_.status());
  }

public:
  explicit Planner2dofSerialJointsNode(const std::string group_name)
  : nh_(),
    pnh_("~"),
    tfl_(tfbuf_),
    planner_(sq_ros1_compat::get_logger("planner_2dof_serial_joints")),
    has_joint_states_(false)
  {
    neonavigation_common::compat::checkCompatMode();
    group_ = group_name;
    ros::NodeHandle nh_group("~/" + group_);

    pub_trajectory_ = neonavigation_common::compat::advertise<trajectory_msgs::JointTrajectory>(
      nh_, "joint_trajectory", pnh_, "trajectory_out", 1, true);
    sub_trajectory_ = neonavigation_common::compat::subscribe(
      nh_, "trajectory_in", pnh_, "trajectory_in", 1, &Planner2dofSerialJointsNode::cbTrajectory,
      this);
    sub_joint_ = neonavigation_common::compat::subscribe(
      nh_, "joint_states", pnh_, "joint", 1, &Planner2dofSerialJointsNode::cbJoint, this);

    pub_status_ = nh_group.advertise<planner_cspace_msgs::PlannerStatus>("status", 1, true);

    Planner2dofSerialJointsCore::Config config;
    config.group_name = group_;
    nh_group.param("resolution", config.resolution, 128);
    pnh_.param("debug_aa", config.debug_aa, false);

    double interval;
    pnh_.param("replan_interval", interval, 0.2);
    replan_interval_ = ros::Duration(interval);
    replan_prev_ = ros::Time(0);
    config.replan_interval = replan_interval_;

    nh_group.param("queue_size_limit", config.queue_size_limit, 0);

    nh_group.param("link0_name", config.links[0].name, std::string("link0"));
    nh_group.param("link0_joint_radius", config.links[0].joint_radius, 0.07f);
    nh_group.param("link0_end_radius", config.links[0].end_radius, 0.07f);
    nh_group.param("link0_length", config.links[0].length, 0.135f);
    nh_group.param("link0_x", config.links[0].x, 0.22f);
    nh_group.param("link0_y", config.links[0].y, 0.0f);
    nh_group.param("link0_th", config.links[0].th, 0.0f);
    nh_group.param("link0_gain_th", config.links[0].gain_th, -1.0f);
    nh_group.param("link0_vmax", config.links[0].vmax, 0.5f);
    nh_group.param("link1_name", config.links[1].name, std::string("link1"));
    nh_group.param("link1_joint_radius", config.links[1].joint_radius, 0.07f);
    nh_group.param("link1_end_radius", config.links[1].end_radius, 0.07f);
    nh_group.param("link1_length", config.links[1].length, 0.27f);
    nh_group.param("link1_x", config.links[1].x, -0.22f);
    nh_group.param("link1_y", config.links[1].y, 0.0f);
    nh_group.param("link1_th", config.links[1].th, 0.0f);
    nh_group.param("link1_gain_th", config.links[1].gain_th, 1.0f);
    nh_group.param("link1_vmax", config.links[1].vmax, 0.5f);

    id_[0] = -1;
    id_[1] = -1;

    nh_group.param("link0_coef", config.links[0].coef, 1.0f);
    nh_group.param("link1_coef", config.links[1].coef, 1.5f);

    nh_group.param("weight_cost", config.weight_cost, 4.0f);
    nh_group.param("expand", config.expand, 0.1f);

    std::string point_vel_mode;
    nh_group.param("point_vel_mode", point_vel_mode, std::string("prev"));
    std::transform(point_vel_mode.begin(), point_vel_mode.end(), point_vel_mode.begin(), ::tolower);
    if (point_vel_mode.compare("prev") == 0)
      config.point_vel = Planner2dofSerialJointsCore::PointVelMode::VEL_PREV;
    else if (point_vel_mode.compare("next") == 0)
      config.point_vel = Planner2dofSerialJointsCore::PointVelMode::VEL_NEXT;
    else if (point_vel_mode.compare("avg") == 0)
      config.point_vel = Planner2dofSerialJointsCore::PointVelMode::VEL_AVG;
    else
      ROS_ERROR("point_vel_mode must be prev/next/avg");

    nh_group.param("range", config.range, 8);
    nh_group.param("num_threads", config.num_threads, 1);

    planner_.initialize(config);
  }
};
}  // namespace planner_2dof_serial_joints
}  // namespace planner_cspace

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "planner_2dof_serial_joints");
  ros::NodeHandle pnh("~");

  std::vector<planner_cspace::planner_2dof_serial_joints::Planner2dofSerialJointsNode::Ptr> jys;
  int n;
  pnh.param("num_groups", n, 1);
  for (int i = 0; i < n; i++) {
    std::string name;
    pnh.param(
      "group" + std::to_string(i) + "_name", name, std::string("group") + std::to_string(i));
    planner_cspace::planner_2dof_serial_joints::Planner2dofSerialJointsNode::Ptr jy;

    jy.reset(new planner_cspace::planner_2dof_serial_joints::Planner2dofSerialJointsNode(name));
    jys.push_back(jy);
  }

  ros::spin();

  return 0;
}
