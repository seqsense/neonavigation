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

#ifndef PLANNER_CSPACE__PLANNER_2DOF_SERIAL_JOINTS__PLANNER_CORE_H_
#define PLANNER_CSPACE__PLANNER_2DOF_SERIAL_JOINTS__PLANNER_CORE_H_

#include <cmath>
#include <list>
#include <string>

#include "planner_cspace/grid_astar.h"
#include "planner_cspace/planner_2dof_serial_joints/grid_astar_model.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace planner_cspace
{
namespace planner_2dof_serial_joints
{
// Planner2dofSerialJointsCore holds the trajectory planning logic for a pair
// of serially connected joints. It has no knowledge of node handles,
// publishers, subscribers or parameters; the interface layer
// (Planner2dofSerialJointsNode in src/planner_2dof_serial_joints.cpp) reads
// the parameters, feeds the joint states in and publishes the results.
class Planner2dofSerialJointsCore
{
public:
  using Astar = GridAstar<2, 0>;

  enum class PointVelMode
  {
    VEL_PREV,
    VEL_NEXT,
    VEL_AVG,
  };

  class LinkBody
  {
  protected:
    class Vec3dof
    {
    public:
      float x_;
      float y_;
      float th_;

      float dist(const Vec3dof & b) { return std::hypot(b.x_ - x_, b.y_ - y_); }
    };

  public:
    float radius_[2];
    float vmax_;
    float length_;
    std::string name_;
    Vec3dof origin_;
    Vec3dof gain_;
    float current_th_;

    LinkBody() : radius_{0.07f, 0.07f}, vmax_(0.5f), length_(0.0f), current_th_(0.0f)
    {
      origin_.x_ = 0.0;
      origin_.y_ = 0.0;
      origin_.th_ = 0.0;
      gain_.x_ = 1.0;
      gain_.y_ = 1.0;
      gain_.th_ = 1.0;
    }
    Vec3dof end(const float th) const
    {
      Vec3dof e = origin_;
      e.x_ += cosf(e.th_ + th * gain_.th_) * length_;
      e.y_ += sinf(e.th_ + th * gain_.th_) * length_;
      e.th_ += th;
      return e;
    }
    bool isCollide(const LinkBody b, const float th0, const float th1)
    {
      auto end0 = end(th0);
      auto end1 = b.end(th1);
      auto & end0r = radius_[1];
      auto & end1r = b.radius_[1];
      auto & origin0 = origin_;
      auto & origin1 = b.origin_;
      auto & origin0r = radius_[0];
      auto & origin1r = b.radius_[0];

      if (end0.dist(end1) < end0r + end1r) return true;
      if (end0.dist(origin1) < end0r + origin1r) return true;
      if (end1.dist(origin0) < end1r + origin0r) return true;

      // add side collision

      return false;
    }
  };

  // Parameters of a single link.
  struct LinkConfig
  {
    std::string name;
    float joint_radius = 0.07f;
    float end_radius = 0.07f;
    float length = 0.135f;
    float x = 0.0f;
    float y = 0.0f;
    float th = 0.0f;
    float gain_th = 1.0f;
    float vmax = 0.5f;
    float coef = 1.0f;
  };

  struct Config
  {
    std::string group_name;
    int resolution = 128;
    int range = 8;
    int queue_size_limit = 0;
    int num_threads = 1;
    float weight_cost = 4.0f;
    float expand = 0.1f;
    PointVelMode point_vel = PointVelMode::VEL_PREV;
    bool debug_aa = false;
    rclcpp::Duration replan_interval = rclcpp::Duration::from_seconds(0.2);
    LinkConfig links[2];
  };

  explicit Planner2dofSerialJointsCore(const rclcpp::Logger & logger);

  // Builds the collision map and the search model. Must be called once
  // before any other method.
  void initialize(const Config & config);

  void setCurrentAngles(const float th0, const float th1);
  // Forces recalculation of the average velocity on the next plan.
  void invalidateAvgVel();

  const std::string & linkName(const size_t i) const { return links_[i].name_; }

  // Plans a trajectory from the current joint angles to the given target.
  // The trajectory to be published is always stored to out; when no path is
  // found it holds a single point which keeps the current angles.
  bool replan(
    const float target0, const float target1, const rclcpp::Duration & time_from_start,
    const std_msgs::msg::Header & header, trajectory_msgs::msg::JointTrajectory & out);

  // Returns true once when the replan interval timer should be reset.
  bool takeReplanTimerReset();

  const planner_cspace_msgs::msg::PlannerStatus & status() const { return status_; }
  void setStatusStamp(const rclcpp::Time & stamp) { status_.header.stamp = stamp; }

protected:
  rclcpp::Logger logger_;

  void grid2Metric(const int t0, const int t1, float & gt0, float & gt1) const;
  void metric2Grid(int & t0, int & t1, const float gt0, const float gt1) const;
  void grid2Metric(const Astar::Vec t, Astar::Vecf & gt) const;
  void metric2Grid(Astar::Vec & t, const Astar::Vecf gt) const;
  bool makePlan(const Astar::Vecf sg, const Astar::Vecf eg, std::list<Astar::Vecf> & path);
  bool cbProgress(const std::list<Astar::Vec> & path_grid, const SearchStats & stats);
  trajectory_msgs::msg::JointTrajectory buildTrajectory(
    const std::list<Astar::Vecf> & path, const rclcpp::Duration & time_from_start,
    const std_msgs::msg::Header & header);
  trajectory_msgs::msg::JointTrajectory buildStayTrajectory(
    const std_msgs::msg::Header & header) const;

  Astar as_;
  Astar::Gridmap<char, 0x40> cm_;
  GridAstarModel2DoFSerialJoint::Ptr model_;

  int resolution_;
  float avg_vel_;
  PointVelMode point_vel_;
  std::string group_;
  bool debug_aa_;
  rclcpp::Duration replan_interval_;
  bool reset_replan_timer_;

  LinkBody links_[2];

  planner_cspace_msgs::msg::PlannerStatus status_;
};
}  // namespace planner_2dof_serial_joints
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE__PLANNER_2DOF_SERIAL_JOINTS__PLANNER_CORE_H_
