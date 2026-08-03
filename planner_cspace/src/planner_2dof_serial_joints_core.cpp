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
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <list>
#include <string>
#include <vector>

#include <omp.h>

#include <ros/console.h>

#include <planner_cspace/planner_2dof_serial_joints/planner_core.h>

namespace planner_cspace
{
namespace planner_2dof_serial_joints
{
Planner2dofSerialJointsCore::Planner2dofSerialJointsCore()
  : resolution_(128)
  , avg_vel_(-1.0f)
  , point_vel_(PointVelMode::VEL_PREV)
  , debug_aa_(false)
  , replan_interval_(0)
  , reset_replan_timer_(false)
{
  status_.status = planner_cspace_msgs::PlannerStatus::DONE;
}

void Planner2dofSerialJointsCore::initialize(const Config& config)
{
  group_ = config.group_name;
  resolution_ = config.resolution;
  point_vel_ = config.point_vel;
  debug_aa_ = config.debug_aa;
  replan_interval_ = config.replan_interval;

  as_.setQueueSizeLimit(config.queue_size_limit);

  cm_.reset(Astar::Vec(resolution_ * 2, resolution_ * 2));
  as_.reset(Astar::Vec(resolution_ * 2, resolution_ * 2));
  cm_.clear(0);

  for (size_t i = 0; i < 2; ++i)
  {
    links_[i].name_ = config.links[i].name;
    links_[i].radius_[0] = config.links[i].joint_radius;
    links_[i].radius_[1] = config.links[i].end_radius;
    links_[i].length_ = config.links[i].length;
    links_[i].origin_.x_ = config.links[i].x;
    links_[i].origin_.y_ = config.links[i].y;
    links_[i].origin_.th_ = config.links[i].th;
    links_[i].gain_.th_ = config.links[i].gain_th;
    links_[i].vmax_ = config.links[i].vmax;
    links_[i].current_th_ = 0.0;
  }

  ROS_INFO("link group: %s", group_.c_str());
  ROS_INFO(" - link0: %s", links_[0].name_.c_str());
  ROS_INFO(" - link1: %s", links_[1].name_.c_str());

  Astar::Vecf euclid_cost_coef;
  euclid_cost_coef[0] = config.links[0].coef;
  euclid_cost_coef[1] = config.links[1].coef;

  CostCoeff cc;
  cc.weight_cost_ = config.weight_cost;
  cc.expand_ = config.expand;

  ROS_INFO("Resolution: %d", resolution_);
  Astar::Vec p;
  for (p[0] = 0; p[0] < resolution_ * 2; p[0]++)
  {
    for (p[1] = 0; p[1] < resolution_ * 2; p[1]++)
    {
      Astar::Vecf pf;
      grid2Metric(p, pf);

      if (links_[0].isCollide(links_[1], pf[0], pf[1]))
        cm_[p] = 100;
      // else if(pf[0] > M_PI || pf[1] > M_PI)
      //   cm_[p] = 50;
      else
        cm_[p] = 0;
    }
  }
  for (p[0] = 0; p[0] < resolution_ * 2; p[0]++)
  {
    for (p[1] = 0; p[1] < resolution_ * 2; p[1]++)
    {
      if (cm_[p] != 100)
        continue;

      Astar::Vec d;
      int range = std::lround(cc.expand_ * resolution_ / (2.0 * M_PI));
      for (d[0] = -range; d[0] <= range; d[0]++)
      {
        for (d[1] = -range; d[1] <= range; d[1]++)
        {
          Astar::Vec p2 = p + d;
          if ((unsigned int)p2[0] >= (unsigned int)resolution_ * 2 ||
              (unsigned int)p2[1] >= (unsigned int)resolution_ * 2)
            continue;
          int dist = std::max(std::abs(d[0]), abs(d[1]));
          int c = std::floor(100.0 * (range - dist) / range);
          if (cm_[p2] < c)
            cm_[p2] = c;
        }
      }
    }
  }

  model_.reset(new GridAstarModel2DoFSerialJoint(
      euclid_cost_coef,
      resolution_,
      cm_,
      cc,
      config.range));

  omp_set_num_threads(config.num_threads);
}

void Planner2dofSerialJointsCore::setCurrentAngles(const float th0, const float th1)
{
  links_[0].current_th_ = th0;
  links_[1].current_th_ = th1;
}

void Planner2dofSerialJointsCore::invalidateAvgVel()
{
  avg_vel_ = -1.0;
}

bool Planner2dofSerialJointsCore::takeReplanTimerReset()
{
  const bool reset = reset_replan_timer_;
  reset_replan_timer_ = false;
  return reset;
}

void Planner2dofSerialJointsCore::grid2Metric(
    const int t0, const int t1,
    float& gt0, float& gt1) const
{
  gt0 = (t0 - resolution_) * 2.0 * M_PI / static_cast<float>(resolution_);
  gt1 = (t1 - resolution_) * 2.0 * M_PI / static_cast<float>(resolution_);
}

void Planner2dofSerialJointsCore::metric2Grid(
    int& t0, int& t1,
    const float gt0, const float gt1) const
{
  t0 = std::lround(gt0 * resolution_ / (2.0 * M_PI)) + resolution_;
  t1 = std::lround(gt1 * resolution_ / (2.0 * M_PI)) + resolution_;
}

void Planner2dofSerialJointsCore::grid2Metric(
    const Astar::Vec t,
    Astar::Vecf& gt) const
{
  grid2Metric(t[0], t[1], gt[0], gt[1]);
}

void Planner2dofSerialJointsCore::metric2Grid(
    Astar::Vec& t,
    const Astar::Vecf gt) const
{
  metric2Grid(t[0], t[1], gt[0], gt[1]);
}

bool Planner2dofSerialJointsCore::cbProgress(
    const std::list<Astar::Vec>& /* path_grid */, const SearchStats& /* stats */)
{
  return false;
}

bool Planner2dofSerialJointsCore::makePlan(
    const Astar::Vecf sg, const Astar::Vecf eg, std::list<Astar::Vecf>& path)
{
  Astar::Vec s, e;
  metric2Grid(s, sg);
  metric2Grid(e, eg);
  ROS_INFO("Planning from (%d, %d) to (%d, %d)",
           s[0], s[1], e[0], e[1]);

  if (cm_[s] == 100)
  {
    ROS_WARN("Path plan failed (current status is in collision)");
    status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
    return false;
  }
  if (cm_[e] == 100)
  {
    ROS_WARN("Path plan failed (goal status is in collision)");
    status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
    return false;
  }
  Astar::Vec d = e - s;
  d.cycle(resolution_, resolution_);

  std::vector<Astar::VecWithCost> starts;
  starts.emplace_back(s);

  if (model_->cost(s, e, starts, e) >= model_->euclidCost(d))
  {
    path.push_back(sg);
    path.push_back(eg);
    if (s == e)
    {
      reset_replan_timer_ = true;
    }
    return true;
  }
  std::list<Astar::Vec> path_grid;
  // const auto ts = std::chrono::high_resolution_clock::now();
  float cancel = std::numeric_limits<float>::max();
  if (replan_interval_ >= ros::Duration(0))
    cancel = replan_interval_.toSec();
  if (!as_.search(
          starts, e, path_grid, model_,
          std::bind(&Planner2dofSerialJointsCore::cbProgress, this,
                    std::placeholders::_1, std::placeholders::_2),
          0, cancel, true))
  {
    ROS_WARN("Path plan failed (goal unreachable)");
    status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
    return false;
  }
  // const auto tnow = std::chrono::high_resolution_clock::now();
  // ROS_INFO("Path found (%0.3f sec.)",
  //   std::chrono::duration<float>(tnow - ts).count());

  bool first = false;
  Astar::Vec n_prev = s;
  path.push_back(sg);
  int i = 0;
  for (auto& n : path_grid)
  {
    if (!first)
    {
      first = true;
      continue;
    }
    if (i == 0)
      ROS_INFO("  next: %d, %d", n[0], n[1]);
    Astar::Vec n_diff = n - n_prev;
    n_diff.cycle(resolution_, resolution_);
    Astar::Vec n2 = n_prev + n_diff;
    n_prev = n2;

    Astar::Vecf p;
    grid2Metric(n2, p);
    path.push_back(p);
    i++;
  }
  float prec = 2.0 * M_PI / static_cast<float>(resolution_);
  Astar::Vecf egp = eg;
  if (egp[0] < 0)
    egp[0] += std::ceil(-egp[0] / M_PI * 2.0) * M_PI * 2.0;
  if (egp[1] < 0)
    egp[1] += std::ceil(-egp[1] / M_PI * 2.0) * M_PI * 2.0;
  path.back()[0] += fmod(egp[0] + prec / 2.0, prec) - prec / 2.0;
  path.back()[1] += fmod(egp[1] + prec / 2.0, prec) - prec / 2.0;

  if (debug_aa_)
  {
    Astar::Vec p;
    for (p[0] = resolution_ / 2; p[0] < resolution_ * 3 / 2; p[0]++)
    {
      for (p[1] = resolution_ / 2; p[1] < resolution_ * 3 / 2; p[1]++)
      {
        bool found = false;
        for (auto& g : path_grid)
        {
          if (g == p)
            found = true;
        }
        if (p == s)
          printf("\033[31ms\033[0m");
        else if (p == e)
          printf("\033[31me\033[0m");
        else if (found)
          printf("\033[34m*\033[0m");
        else
          printf("%d", cm_[p] / 11);
      }
      printf("\n");
    }
    printf("\n");
  }

  return true;
}

trajectory_msgs::JointTrajectory Planner2dofSerialJointsCore::buildTrajectory(
    const std::list<Astar::Vecf>& path, const ros::Duration& time_from_start,
    const std_msgs::Header& header)
{
  if (avg_vel_ < 0)
  {
    float pos_sum = 0;
    for (auto it = path.begin(); it != path.end(); it++)
    {
      auto it_next = it;
      it_next++;
      if (it_next != path.end())
      {
        float diff[2], diff_max;
        diff[0] = std::abs((*it_next)[0] - (*it)[0]);
        diff[1] = std::abs((*it_next)[1] - (*it)[1]);
        diff_max = std::max(diff[0], diff[1]);
        pos_sum += diff_max;
      }
    }
    if (time_from_start <= ros::Duration(0))
    {
      avg_vel_ = std::min(links_[0].vmax_, links_[1].vmax_);
    }
    else
    {
      avg_vel_ = pos_sum / time_from_start.toSec();
      if (avg_vel_ > links_[0].vmax_)
        avg_vel_ = links_[0].vmax_;
      if (avg_vel_ > links_[1].vmax_)
        avg_vel_ = links_[1].vmax_;
    }
  }

  trajectory_msgs::JointTrajectory out;
  out.header = header;
  out.header.stamp = ros::Time(0);
  out.joint_names.resize(2);
  out.joint_names[0] = links_[0].name_;
  out.joint_names[1] = links_[1].name_;
  float pos_sum = 0.0;
  for (auto it = path.begin(); it != path.end(); it++)
  {
    if (it == path.begin())
      continue;

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize(2);
    p.velocities.resize(2);

    auto it_prev = it;
    it_prev--;
    auto it_next = it;
    it_next++;

    float diff[2], diff_max;
    diff[0] = std::abs((*it)[0] - (*it_prev)[0]);
    diff[1] = std::abs((*it)[1] - (*it_prev)[1]);
    diff_max = std::max(diff[0], diff[1]);
    pos_sum += diff_max;

    if (it_next == path.end())
    {
      p.velocities[0] = 0.0;
      p.velocities[1] = 0.0;
    }
    else
    {
      float dir[2], dir_max;
      switch (point_vel_)
      {
        default:
        case PointVelMode::VEL_PREV:
          dir[0] = ((*it)[0] - (*it_prev)[0]);
          dir[1] = ((*it)[1] - (*it_prev)[1]);
          break;
        case PointVelMode::VEL_NEXT:
          dir[0] = ((*it_next)[0] - (*it)[0]);
          dir[1] = ((*it_next)[1] - (*it)[1]);
          break;
        case PointVelMode::VEL_AVG:
          dir[0] = ((*it_next)[0] - (*it_prev)[0]);
          dir[1] = ((*it_next)[1] - (*it_prev)[1]);
          break;
      }
      dir_max = std::max(std::abs(dir[0]), std::abs(dir[1]));
      float t = dir_max / avg_vel_;

      p.velocities[0] = dir[0] / t;
      p.velocities[1] = dir[1] / t;
    }
    p.time_from_start = ros::Duration(pos_sum / avg_vel_);
    p.positions[0] = (*it)[0];
    p.positions[1] = (*it)[1];
    out.points.push_back(p);
  }
  return out;
}

trajectory_msgs::JointTrajectory Planner2dofSerialJointsCore::buildStayTrajectory(
    const std_msgs::Header& header) const
{
  trajectory_msgs::JointTrajectory out;
  out.header = header;
  out.header.stamp = ros::Time(0);
  out.joint_names.resize(2);
  out.joint_names[0] = links_[0].name_;
  out.joint_names[1] = links_[1].name_;
  trajectory_msgs::JointTrajectoryPoint p;
  p.positions.resize(2);
  p.positions[0] = links_[0].current_th_;
  p.positions[1] = links_[1].current_th_;
  p.velocities.resize(2);
  out.points.push_back(p);
  return out;
}

bool Planner2dofSerialJointsCore::replan(
    const float target0, const float target1,
    const ros::Duration& time_from_start,
    const std_msgs::Header& header,
    trajectory_msgs::JointTrajectory& out)
{
  const Astar::Vecf start(
      links_[0].current_th_,
      links_[1].current_th_);
  const Astar::Vecf end(target0, target1);

  ROS_INFO("link %s: %0.3f, %0.3f", group_.c_str(), target0, target1);

  status_.status = planner_cspace_msgs::PlannerStatus::DOING;
  status_.error = planner_cspace_msgs::PlannerStatus::GOING_WELL;

  ROS_INFO("Start searching");
  std::list<Astar::Vecf> path;
  if (makePlan(start, end, path))
  {
    ROS_INFO("Trajectory found");
    out = buildTrajectory(path, time_from_start, header);
    return true;
  }
  out = buildStayTrajectory(header);
  ROS_WARN("Trajectory not found");
  return false;
}
}  // namespace planner_2dof_serial_joints
}  // namespace planner_cspace
