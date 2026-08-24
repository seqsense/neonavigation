/*
 * Copyright (c) 2018-2025, the neonavigation authors
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

#ifndef PLANNER_CSPACE__PLANNER_SCENARIO_HARNESS_H_
#define PLANNER_CSPACE__PLANNER_SCENARIO_HARNESS_H_

// In-process harness for the planner_3d navigation scenarios.
//
// The rostests and launch tests bring up five nodes - a map server, the
// costmap, the planner, a dummy robot and the tracker - and let them drive each
// other over DDS for two minutes per scenario. Planner3dCore takes the costmap,
// the start and the goal, and is stepped one planning cycle at a time with the
// time passed in, so the same scenarios can run in one process with time under
// the test's control.
//
// Two deliberate simplifications:
//
//  * the costmap is built here from a 2-D obstacle grid, replicated across the
//    angle layers. Inflating a footprint into a 3-D costmap is costmap_cspace's
//    job and is covered by its own tests; these scenarios vary the obstacle
//    layout instead, with a point robot.
//  * the robot follows the planned path exactly at max_vel/max_ang_vel instead
//    of being steered by trajectory_tracker. Tracking error is what
//    trajectory_tracker's own scenarios are about.
//
// What is left is the planner: whether it finds a path, how it recovers when
// the robot or the goal ends up in an obstacle, what it remembers, and what it
// reports while doing so.

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_cspace/planner_3d/planner_3d_core.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace planner_cspace_testing
{

// A 2-D obstacle grid, written as ASCII rows so a scenario's map is readable in
// the test itself. '.' is free, '#' is occupied, '?' is unknown (which the
// planner charges its unknown_cost for), digits 0-9 are costs in tens.
// The rows are written the way the map looks - top row first - and y grows
// upwards from the bottom row, which is what map_server does with an image.
class GridMap
{
public:
  // costmap_cspace turns a 2-D map into a 3-D one by inflating each angle layer
  // with the robot footprint rotated to that angle. The launch tests use a
  // 0.4 x 0.2 m footprint with linear_expand 0.1, which is what the defaults
  // here mean: a cell is blocked for an angle if placing the robot there at
  // that angle would put the footprint on an obstacle.
  GridMap(
    std::vector<std::string> rows, const double resolution = 0.1, const int angle = 16,
    const double half_length = 0.2 + 0.1, const double half_width = 0.1 + 0.1)
  : rows_(std::move(rows)),
    resolution_(resolution),
    angle_(angle),
    half_length_(half_length),
    half_width_(half_width)
  {
    std::reverse(rows_.begin(), rows_.end());
    buildFootprints();
  }

  int width() const { return static_cast<int>(rows_.empty() ? 0 : rows_.front().size()); }
  int height() const { return static_cast<int>(rows_.size()); }
  double resolution() const { return resolution_; }

  void set(const int x, const int y, const char c) { rows_[y][x] = c; }

  // Writes a filled rectangle, which is how the scenarios drop an obstacle on
  // the robot or the goal.
  void fill(const int x0, const int y0, const int x1, const int y1, const char c)
  {
    for (int y = std::max(0, y0); y <= std::min(height() - 1, y1); ++y) {
      for (int x = std::max(0, x0); x <= std::min(width() - 1, x1); ++x) {
        rows_[y][x] = c;
      }
    }
  }

  // The raw cell, before inflation. Unknown is -1, as map_server reports it.
  int8_t rawCostAt(const int x, const int y) const
  {
    const char c = rows_[y][x];
    if (c == '#') {
      return 100;
    }
    if (c == '?') {
      return -1;
    }
    if (c >= '0' && c <= '9') {
      return static_cast<int8_t>((c - '0') * 10);
    }
    return 0;
  }

  // What the planner sees for a given angle layer.
  int8_t costAt(const int x, const int y, const int a) const
  {
    const int8_t raw = rawCostAt(x, y);
    if (raw == 100) {
      return raw;
    }
    for (const auto & offset : footprints_[a]) {
      const int nx = x + offset.first;
      const int ny = y + offset.second;
      if (nx < 0 || ny < 0 || nx >= width() || ny >= height()) {
        continue;
      }
      if (rawCostAt(nx, ny) == 100) {
        return 100;
      }
    }
    return raw;
  }

  // std::shared_ptr rather than the message's own SharedPtr alias: on ROS 1 the
  // compat headers alias the ROS 2 message names onto the ROS 1 structs, which
  // carry Ptr/ConstPtr instead. Planner3dCore's own signatures spell it out the
  // same way.
  std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3D> toCSpace3D(
    const rclcpp::Time & stamp) const
  {
    auto msg = std::make_shared<costmap_cspace_msgs::msg::CSpace3D>();
    msg->header.frame_id = "map";
    msg->header.stamp = stamp;
    msg->info.width = width();
    msg->info.height = height();
    msg->info.angle = angle_;
    msg->info.linear_resolution = resolution_;
    msg->info.angular_resolution = 2.0 * M_PI / angle_;
    msg->info.origin.orientation.w = 1.0;
    msg->data.resize(static_cast<size_t>(width()) * height() * angle_);
    for (int a = 0; a < angle_; ++a) {
      for (int y = 0; y < height(); ++y) {
        for (int x = 0; x < width(); ++x) {
          msg->data[index(x, y, a)] = costAt(x, y, a);
        }
      }
    }
    return msg;
  }

  // A whole-map update, as costmap_cspace publishes after a map change.
  std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> toUpdate(
    const rclcpp::Time & stamp) const
  {
    auto msg = std::make_shared<costmap_cspace_msgs::msg::CSpace3DUpdate>();
    msg->header.frame_id = "map";
    msg->header.stamp = stamp;
    msg->x = 0;
    msg->y = 0;
    msg->yaw = 0;
    msg->width = width();
    msg->height = height();
    msg->angle = angle_;
    msg->data.resize(static_cast<size_t>(width()) * height() * angle_);
    for (int a = 0; a < angle_; ++a) {
      for (int y = 0; y < height(); ++y) {
        for (int x = 0; x < width(); ++x) {
          msg->data[index(x, y, a)] = costAt(x, y, a);
        }
      }
    }
    return msg;
  }

private:
  // Cell offsets the footprint covers, per angle layer.
  void buildFootprints()
  {
    const int reach =
      static_cast<int>(std::ceil(std::hypot(half_length_, half_width_) / resolution_));
    footprints_.resize(angle_);
    for (int a = 0; a < angle_; ++a) {
      const double yaw = 2.0 * M_PI * a / angle_;
      const double c = std::cos(yaw);
      const double sn = std::sin(yaw);
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
          const double mx = dx * resolution_;
          const double my = dy * resolution_;
          // Into the robot frame at this angle.
          const double along = c * mx + sn * my;
          const double across = -sn * mx + c * my;
          // Strict, so a cell whose centre sits exactly on the footprint edge
          // stays free: including it would block one ring more than the real
          // costmap does when it rasterises the polygon.
          if (std::abs(along) < half_length_ && std::abs(across) < half_width_) {
            footprints_[a].emplace_back(dx, dy);
          }
        }
      }
    }
  }

  size_t index(const int x, const int y, const int a) const
  {
    return static_cast<size_t>(a) * width() * height() + static_cast<size_t>(y) * width() + x;
  }

  std::vector<std::string> rows_;
  double resolution_;
  int angle_;
  double half_length_;
  double half_width_;
  std::vector<std::vector<std::pair<int, int>>> footprints_;
};

class PlannerScenario
{
public:
  struct Pose
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  PlannerScenario(
    const planner_cspace::planner_3d::Planner3dCore::Parameters & params,
    const planner_cspace::planner_3d::Planner3dCore::StaticParameters & static_params)
  : core_(rclcpp::get_logger("planner_scenario")), params_(params)
  {
    planner_cspace::planner_3d::Planner3dCore::Callbacks cb;
    cb.publish_path = [this](const nav_msgs::msg::Path & path) { path_ = path; };
    cb.publish_status = [this]() { ++status_updates_; };
    cb.goal_reached_in_continuous_mode = [this]() { ++continuous_goal_reached_; };
    core_.setCallbacks(cb);
    core_.initialize(static_params);
    core_.setParameters(params_);
  }

  void setMap(const GridMap & map)
  {
    const auto msg = map.toCSpace3D(stamp());
    const auto retained = core_.setMap(msg);
    if (retained) {
      core_.applyCostmapUpdate(retained);
    }
    core_.clearRetainedMapUpdate();
    core_.applyCostmapUpdate(map.toUpdate(stamp()));
  }

  // A map change while the planner is running, as a costmap update would be.
  void applyMapUpdate(const GridMap & map) { core_.applyCostmapUpdate(map.toUpdate(stamp())); }

  void setStart(const Pose & pose)
  {
    robot_ = pose;
    core_.setStart(toPoseStamped(pose));
  }

  void setGoal(const Pose & pose) { ASSERT_GOAL(core_.setGoal(toPoseStamped(pose))); }

  planner_cspace::planner_3d::Planner3dCore & core() { return core_; }

  // One planning cycle, followed by the robot moving along the path it got.
  planner_cspace::planner_3d::Planner3dCore::PlanCycleResult step()
  {
    const double dt = 1.0 / params_.freq;
    core_.setStart(toPoseStamped(robot_));
    auto result = planner_cspace::planner_3d::Planner3dCore::PlanCycleResult::NONE;
    const bool has_costmap = core_.preparePlanCycle(now());
    if (core_.isReadyToPlan() && has_costmap) {
      result = core_.runPlanCycle(now());
    } else if (!core_.hasGoal()) {
      core_.handleNoGoal();
    }
    core_.setStatusStamp(now());
    followPath(dt);
    time_ += dt;
    ++cycles_;
    return result;
  }

  // Steps until `stop` says so, the goal is reached, or the budget is spent.
  bool runUntil(
    const double budget_sec, const std::function<bool(PlannerScenario &)> & stop,
    const std::function<void(PlannerScenario &)> & watch = nullptr)
  {
    const double deadline = time_ + budget_sec;
    while (time_ <= deadline) {
      last_result_ = step();
      if (watch) {
        watch(*this);
      }
      if (stop(*this)) {
        return true;
      }
    }
    return false;
  }

  bool runUntilGoalReached(
    const double budget_sec, const std::function<void(PlannerScenario &)> & watch = nullptr)
  {
    return runUntil(
      budget_sec,
      [](PlannerScenario & s) {
        return s.lastResult() ==
               planner_cspace::planner_3d::Planner3dCore::PlanCycleResult::GOAL_REACHED;
      },
      watch);
  }

  const nav_msgs::msg::Path & path() const { return path_; }
  const planner_cspace_msgs::msg::PlannerStatus & status() const { return core_.status(); }
  const Pose & robot() const { return robot_; }
  double time() const { return time_; }
  int cycles() const { return cycles_; }
  int continuousGoalReached() const { return continuous_goal_reached_; }
  planner_cspace::planner_3d::Planner3dCore::PlanCycleResult lastResult() const
  {
    return last_result_;
  }

  double distanceToGoal(const Pose & goal) const
  {
    return std::hypot(robot_.x - goal.x, robot_.y - goal.y);
  }

private:
  static void ASSERT_GOAL(const planner_cspace::planner_3d::Planner3dCore::SetGoalResult result)
  {
    // The scenarios set reachable goals; a rejected one is a bug in the test.
    if (result == planner_cspace::planner_3d::Planner3dCore::SetGoalResult::REJECTED) {
      throw std::runtime_error("the planner rejected the goal");
    }
  }

  rclcpp::Time stamp() const { return now(); }

  rclcpp::Time now() const
  {
    return rclcpp::Time(static_cast<int64_t>(time_ * 1e9) + 1, RCL_ROS_TIME);
  }

  geometry_msgs::msg::PoseStamped toPoseStamped(const Pose & pose) const
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.yaw);
    msg.pose.orientation = tf2::toMsg(q);
    return msg;
  }

  // Moves at most max_vel * dt towards a pose, turning at most max_ang_vel * dt
  // onto its orientation.
  void moveTowards(const geometry_msgs::msg::Pose & target, const double dt)
  {
    const double dx = target.position.x - robot_.x;
    const double dy = target.position.y - robot_.y;
    const double dist = std::hypot(dx, dy);
    const double budget = params_.max_vel * dt;
    if (dist > 1.0e-9) {
      const double ratio = std::min(1.0, budget / dist);
      robot_.x += dx * ratio;
      robot_.y += dy * ratio;
    }
    turnTowards(tf2::getYaw(target.orientation), dt);
  }

  void turnTowards(const double target_yaw, const double dt)
  {
    const double yaw_diff = std::remainder(target_yaw - robot_.yaw, 2.0 * M_PI);
    const double yaw_budget = params_.max_ang_vel * dt;
    robot_.yaw =
      std::remainder(robot_.yaw + std::clamp(yaw_diff, -yaw_budget, yaw_budget), 2.0 * M_PI);
  }

  // Walks the robot along the planned path: project it onto the path, then
  // advance that far plus max_vel * dt along the remaining poses. Aiming at the
  // first pose instead would leave the robot oscillating around it, since the
  // path is replanned from the robot's position every cycle.
  void followPath(const double dt)
  {
    const auto & poses = path_.poses;
    if (poses.empty()) {
      return;
    }
    if (poses.size() == 1) {
      // Finishing: the planner is down to the goal pose itself, so drive
      // straight at it and turn onto its orientation.
      moveTowards(poses.front().pose, dt);
      return;
    }

    // Closest point on the polyline, as a segment index plus a ratio into it.
    size_t seg = 0;
    double seg_ratio = 0.0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < poses.size(); ++i) {
      const auto & a = poses[i].pose.position;
      const auto & b = poses[i + 1].pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len_sq = dx * dx + dy * dy;
      double t = 0.0;
      if (len_sq > 1.0e-12) {
        t = ((robot_.x - a.x) * dx + (robot_.y - a.y) * dy) / len_sq;
        t = std::clamp(t, 0.0, 1.0);
      }
      const double d = std::hypot(a.x + dx * t - robot_.x, a.y + dy * t - robot_.y);
      if (d < best) {
        best = d;
        seg = i;
        seg_ratio = t;
      }
    }

    // Advance along the path from there.
    double budget = params_.max_vel * dt;
    double x = poses[seg].pose.position.x +
               (poses[seg + 1].pose.position.x - poses[seg].pose.position.x) * seg_ratio;
    double y = poses[seg].pose.position.y +
               (poses[seg + 1].pose.position.y - poses[seg].pose.position.y) * seg_ratio;
    size_t at = seg + 1;
    while (at < poses.size() && budget > 0.0) {
      const auto & target = poses[at].pose.position;
      const double dist = std::hypot(target.x - x, target.y - y);
      if (dist <= budget) {
        x = target.x;
        y = target.y;
        budget -= dist;
        ++at;
      } else {
        x += (target.x - x) * budget / dist;
        y += (target.y - y) * budget / dist;
        budget = 0.0;
      }
    }
    robot_.x = x;
    robot_.y = y;

    const size_t heading_at = std::min(at, poses.size() - 1);
    turnTowards(tf2::getYaw(poses[heading_at].pose.orientation), dt);
  }

  planner_cspace::planner_3d::Planner3dCore core_;
  planner_cspace::planner_3d::Planner3dCore::Parameters params_;
  nav_msgs::msg::Path path_;
  Pose robot_;
  double time_ = 0.0;
  int cycles_ = 0;
  int status_updates_ = 0;
  int continuous_goal_reached_ = 0;
  planner_cspace::planner_3d::Planner3dCore::PlanCycleResult last_result_ =
    planner_cspace::planner_3d::Planner3dCore::PlanCycleResult::NONE;
};

}  // namespace planner_cspace_testing

#endif  // PLANNER_CSPACE__PLANNER_SCENARIO_HARNESS_H_
