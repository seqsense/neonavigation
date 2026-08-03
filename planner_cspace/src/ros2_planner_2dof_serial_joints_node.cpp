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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planner_cspace/planner_2dof_serial_joints/planner_core.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace planner_cspace
{
namespace planner_2dof_serial_joints
{
// ROS 2 interface of the planner_2dof_serial_joints node. All of the
// trajectory planning is delegated to Planner2dofSerialJointsCore, one
// instance per link group.
//
// Differences to the ROS 1 node
// -----------------------------
// * ROS 1 instantiated one Planner2dofSerialJointsNode object per link group,
//   each holding its own node handles but sharing the same topics. A ROS 2
//   component is a single node, so this node owns one JointGroup per group and
//   dispatches the shared subscriptions to all of them. The externally visible
//   topics are unchanged.
// * ROS 1 read the per-group settings from the nested private namespace
//   `~/<group>/<key>` (e.g. `~/group0/link0_name`). ROS 2 parameters are flat,
//   so the same settings are given as dot separated names `<group>.<key>`
//   (e.g. `group0.link0_name`). See README.md.
class Planner2dofSerialJointsNode : public rclcpp::Node
{
public:
  explicit Planner2dofSerialJointsNode(const rclcpp::NodeOptions & options);

private:
  // Per link group state: the planning core plus the interface-side bookkeeping
  // which the ROS 1 node kept in each Planner2dofSerialJointsNode instance.
  class JointGroup
  {
  public:
    using Ptr = std::shared_ptr<JointGroup>;

    JointGroup(const std::string & name, const rclcpp::Logger & logger)
    : planner(logger),
      group(name),
      cmd_prev(rclcpp::Duration(0, 0), std::pair<float, float>(0.0f, 0.0f)),
      replan_prev(0, 0, RCL_ROS_TIME),
      replan_interval(0, 0),
      has_joint_states(false)
    {
      id[0] = -1;
      id[1] = -1;
    }

    Planner2dofSerialJointsCore planner;
    std::string group;
    rclcpp::Publisher<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr pub_status;

    std::pair<rclcpp::Duration, std::pair<float, float>> cmd_prev;
    trajectory_msgs::msg::JointTrajectory traj_prev;
    int id[2];

    rclcpp::Time replan_prev;
    rclcpp::Duration replan_interval;
    bool has_joint_states;
  };

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;

  std::vector<JointGroup::Ptr> groups_;

  JointGroup::Ptr createGroup(const std::string & name, bool debug_aa, double replan_interval);
  void cbJoint(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);
  void cbTrajectory(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr & msg);
  void replan(const JointGroup::Ptr & g);
};

Planner2dofSerialJointsNode::Planner2dofSerialJointsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("planner_2dof_serial_joints", options)
{
  // ROS 1 latched this publisher; transient_local is the ROS 2 equivalent.
  pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory", rclcpp::QoS(1).transient_local());
  sub_trajectory_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "trajectory_in", 1,
    std::bind(&Planner2dofSerialJointsNode::cbTrajectory, this, std::placeholders::_1));
  sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 1,
    std::bind(&Planner2dofSerialJointsNode::cbJoint, this, std::placeholders::_1));

  const bool debug_aa = this->declare_parameter("debug_aa", false);
  const double replan_interval = this->declare_parameter("replan_interval", 0.2);

  const int num_groups = static_cast<int>(this->declare_parameter("num_groups", 1));
  for (int i = 0; i < num_groups; i++) {
    const std::string name = this->declare_parameter(
      "group" + std::to_string(i) + "_name", std::string("group") + std::to_string(i));
    groups_.push_back(createGroup(name, debug_aa, replan_interval));
  }
}

Planner2dofSerialJointsNode::JointGroup::Ptr Planner2dofSerialJointsNode::createGroup(
  const std::string & name, const bool debug_aa, const double replan_interval)
{
  auto g = std::make_shared<JointGroup>(name, this->get_logger());

  // ROS 1 published this on the nested private handle `~/<group>`.
  g->pub_status = this->create_publisher<planner_cspace_msgs::msg::PlannerStatus>(
    "~/" + name + "/status", rclcpp::QoS(1).transient_local());

  // Flattened counterpart of the ROS 1 `~/<group>/<key>` namespace.
  const std::string p = name + ".";

  Planner2dofSerialJointsCore::Config config;
  config.group_name = name;
  config.resolution = static_cast<int>(this->declare_parameter(p + "resolution", 128));
  config.debug_aa = debug_aa;

  g->replan_interval = rclcpp::Duration::from_seconds(replan_interval);
  g->replan_prev = rclcpp::Time(0, 0, RCL_ROS_TIME);
  config.replan_interval = g->replan_interval;

  config.queue_size_limit = static_cast<int>(this->declare_parameter(p + "queue_size_limit", 0));

  config.links[0].name = this->declare_parameter(p + "link0_name", std::string("link0"));
  config.links[0].joint_radius = this->declare_parameter(p + "link0_joint_radius", 0.07);
  config.links[0].end_radius = this->declare_parameter(p + "link0_end_radius", 0.07);
  config.links[0].length = this->declare_parameter(p + "link0_length", 0.135);
  config.links[0].x = this->declare_parameter(p + "link0_x", 0.22);
  config.links[0].y = this->declare_parameter(p + "link0_y", 0.0);
  config.links[0].th = this->declare_parameter(p + "link0_th", 0.0);
  config.links[0].gain_th = this->declare_parameter(p + "link0_gain_th", -1.0);
  config.links[0].vmax = this->declare_parameter(p + "link0_vmax", 0.5);
  config.links[1].name = this->declare_parameter(p + "link1_name", std::string("link1"));
  config.links[1].joint_radius = this->declare_parameter(p + "link1_joint_radius", 0.07);
  config.links[1].end_radius = this->declare_parameter(p + "link1_end_radius", 0.07);
  config.links[1].length = this->declare_parameter(p + "link1_length", 0.27);
  config.links[1].x = this->declare_parameter(p + "link1_x", -0.22);
  config.links[1].y = this->declare_parameter(p + "link1_y", 0.0);
  config.links[1].th = this->declare_parameter(p + "link1_th", 0.0);
  config.links[1].gain_th = this->declare_parameter(p + "link1_gain_th", 1.0);
  config.links[1].vmax = this->declare_parameter(p + "link1_vmax", 0.5);

  config.links[0].coef = this->declare_parameter(p + "link0_coef", 1.0);
  config.links[1].coef = this->declare_parameter(p + "link1_coef", 1.5);

  config.weight_cost = this->declare_parameter(p + "weight_cost", 4.0);
  config.expand = this->declare_parameter(p + "expand", 0.1);

  std::string point_vel_mode = this->declare_parameter(p + "point_vel_mode", std::string("prev"));
  std::transform(point_vel_mode.begin(), point_vel_mode.end(), point_vel_mode.begin(), ::tolower);
  if (point_vel_mode.compare("prev") == 0) {
    config.point_vel = Planner2dofSerialJointsCore::PointVelMode::VEL_PREV;
  } else if (point_vel_mode.compare("next") == 0) {
    config.point_vel = Planner2dofSerialJointsCore::PointVelMode::VEL_NEXT;
  } else if (point_vel_mode.compare("avg") == 0) {
    config.point_vel = Planner2dofSerialJointsCore::PointVelMode::VEL_AVG;
  } else {
    RCLCPP_ERROR(this->get_logger(), "point_vel_mode must be prev/next/avg");
  }

  config.range = static_cast<int>(this->declare_parameter(p + "range", 8));
  config.num_threads = static_cast<int>(this->declare_parameter(p + "num_threads", 1));

  g->planner.initialize(config);
  return g;
}

void Planner2dofSerialJointsNode::cbJoint(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  for (const JointGroup::Ptr & g : groups_) {
    int id[2] = {-1, -1};
    for (size_t i = 0; i < msg->name.size(); i++) {
      if (msg->name[i].compare(g->planner.linkName(0)) == 0) {
        id[0] = i;
      } else if (msg->name[i].compare(g->planner.linkName(1)) == 0) {
        id[1] = i;
      }
    }
    if (id[0] == -1 || id[1] == -1) {
      RCLCPP_ERROR(
        this->get_logger(), "joint_state does not contain link group %s.", g->group.c_str());
      continue;
    }
    g->planner.setCurrentAngles(msg->position[id[0]], msg->position[id[1]]);
    g->has_joint_states = true;

    if (
      (g->replan_prev + g->replan_interval < this->now() ||
       g->replan_prev == rclcpp::Time(0, 0, RCL_ROS_TIME)) &&
      g->replan_interval > rclcpp::Duration(0, 0)) {
      replan(g);
    }
  }
}

void Planner2dofSerialJointsNode::cbTrajectory(
  const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr & msg)
{
  for (const JointGroup::Ptr & g : groups_) {
    g->id[0] = -1;
    g->id[1] = -1;
    for (size_t i = 0; i < msg->joint_names.size(); i++) {
      if (msg->joint_names[i].compare(g->planner.linkName(0)) == 0) {
        g->id[0] = i;
      } else if (msg->joint_names[i].compare(g->planner.linkName(1)) == 0) {
        g->id[1] = i;
      }
    }
    if (g->id[0] == -1 || g->id[1] == -1) {
      RCLCPP_ERROR(
        this->get_logger(), "joint_trajectory does not contains link group %s.", g->group.c_str());
      continue;
    }
    if (msg->points.size() != 1) {
      RCLCPP_ERROR(this->get_logger(), "single trajectory point required.");
    }
    std::pair<rclcpp::Duration, std::pair<float, float>> cmd(
      rclcpp::Duration(msg->points[0].time_from_start),
      std::pair<float, float>(
        msg->points[0].positions[g->id[0]], msg->points[0].positions[g->id[1]]));
    if (g->cmd_prev == cmd) {
      continue;
    }
    g->cmd_prev = cmd;
    g->traj_prev = *msg;
    g->planner.invalidateAvgVel();

    replan(g);
  }
}

void Planner2dofSerialJointsNode::replan(const JointGroup::Ptr & g)
{
  if (!g->has_joint_states) {
    return;
  }

  g->replan_prev = this->now();
  if (g->id[0] == -1 || g->id[1] == -1) {
    return;
  }

  auto out = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
  g->planner.replan(
    static_cast<float>(g->traj_prev.points[0].positions[g->id[0]]),
    static_cast<float>(g->traj_prev.points[0].positions[g->id[1]]),
    rclcpp::Duration(g->traj_prev.points[0].time_from_start), g->traj_prev.header, *out);
  pub_trajectory_->publish(std::move(out));
  if (g->planner.takeReplanTimerReset()) {
    g->replan_prev = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  g->planner.setStatusStamp(this->now());
  auto status = std::make_unique<planner_cspace_msgs::msg::PlannerStatus>(g->planner.status());
  g->pub_status->publish(std::move(status));
}
}  // namespace planner_2dof_serial_joints
}  // namespace planner_cspace

RCLCPP_COMPONENTS_REGISTER_NODE(
  planner_cspace::planner_2dof_serial_joints::Planner2dofSerialJointsNode)
