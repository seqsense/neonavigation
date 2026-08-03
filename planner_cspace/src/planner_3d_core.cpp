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

// Make DEBUG flag taking precedence over NDEBUG flag for boundary test
#ifdef DEBUG
#ifdef NDEBUG
#undef NDEBUG
#endif
#endif

#include "planner_cspace/planner_3d/planner_3d_core.h"

#include <omp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "neonavigation_metrics_msgs/helper.h"
#include "planner_cspace/bbf.h"
#include "planner_cspace/planner_3d/grid_metric_converter.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace planner_cspace
{
namespace planner_3d
{
Planner3dCore::Planner3dCore(const rclcpp::Logger & logger)
: logger_(logger),
  bbf_costmap_(new CostmapBBFImpl()),
  cost_estim_cache_(cm_rough_, bbf_costmap_),
  cost_estim_cache_static_(cm_rough_base_, bbf_costmap_),
  arrivable_map_(cm_local_esc_, CostmapBBF::Ptr(new CostmapBBFNoOp())),
  cost_estim_cache_static_update_(
    DistanceMap::Astar::Vec(1, 1, 0), DistanceMap::Astar::Vec(0, 0, 0)),
  freq_(4.0f),
  freq_min_(2.0f),
  search_timeout_abort_(30.0f),
  search_range_(0.4f),
  antialias_start_(false),
  range_(0),
  local_range_(0),
  local_range_f_(0.0),
  longcut_range_f_(0.0),
  esc_range_(0),
  esc_range_min_(0),
  esc_angle_(0),
  esc_range_f_(0.0),
  esc_range_min_ratio_(0.0),
  tolerance_range_(0),
  tolerance_angle_(0),
  tolerance_range_f_(0.0),
  tolerance_angle_f_(0.0),
  path_interpolation_resolution_(0.5),
  grid_enumeration_resolution_(0.1),
  unknown_cost_(100),
  overwrite_cost_(false),
  relocation_acceptable_cost_(50),
  has_map_(false),
  has_goal_(false),
  has_start_(false),
  has_hysteresis_map_(false),
  cost_estim_cache_created_(false),
  remember_updates_(false),
  fast_map_update_(false),
  hist_ignore_range_f_(0.0),
  hist_ignore_range_(0),
  hist_ignore_range_max_f_(0.0),
  hist_ignore_range_max_(0),
  temporary_escape_(false),
  remember_hit_odds_(0.0f),
  remember_miss_odds_(0.0f),
  retain_last_error_status_(true),
  num_cost_estim_task_(16),
  keep_a_part_of_previous_path_(false),
  enable_crowd_mode_(false),
  robot_frame_("base_link"),
  max_retry_num_(-1),
  cc_(),
  goal_tolerance_lin_f_(0.0),
  goal_tolerance_ang_f_(0.0),
  goal_tolerance_ang_finish_(0.0),
  goal_tolerance_lin_(0),
  goal_tolerance_ang_(0),
  temporary_escape_tolerance_lin_f_(0.0),
  temporary_escape_tolerance_ang_f_(0.0),
  temporary_escape_tolerance_lin_(0),
  temporary_escape_tolerance_ang_(0),
  has_goal_tolerance_(false),
  find_best_(true),
  sw_wait_(0.0f),
  is_path_switchback_(false),
  force_goal_orientation_(true),
  escape_status_(TemporaryEscapeStatus::NOT_ESCAPING),
  cnt_stuck_(0),
  is_start_occupied_(false),
  costmap_watchdog_(0, 0),
  last_costmap_(0, 0, RCL_ROS_TIME),
  prev_map_update_x_min_(0),
  prev_map_update_x_max_(0),
  prev_map_update_y_min_(0),
  prev_map_update_y_max_(0)
{
  status_.status = planner_cspace_msgs::msg::PlannerStatus::DONE;
  start_pose_predictor_.setLogger(logger_);
}

void Planner3dCore::setCallbacks(const Callbacks & cb) { cb_ = cb; }

void Planner3dCore::initialize(const StaticParameters & p)
{
  unknown_cost_ = p.unknown_cost;
  path_interpolation_resolution_ = p.path_interpolation_resolution;
  grid_enumeration_resolution_ = p.grid_enumeration_resolution;
  if (path_interpolation_resolution_ < grid_enumeration_resolution_) {
    RCLCPP_ERROR(
      logger_,
      "path_interpolation_resolution must be greater than or equal to "
      "grid_enumeration_resolution.");
    path_interpolation_resolution_ = grid_enumeration_resolution_;
  }
  robot_frame_ = p.robot_frame;
  enable_crowd_mode_ = p.enable_crowd_mode;
  retain_last_error_status_ = p.retain_last_error_status;
  num_cost_estim_task_ = p.num_cost_estim_task;

  as_.setQueueSizeLimit(p.queue_size_limit);
  as_.setSearchTaskNum(p.num_search_task);
  omp_set_num_threads(p.num_threads);
}

void Planner3dCore::setParameters(const Parameters & p)
{
  freq_ = p.freq;
  freq_min_ = p.freq_min;
  search_timeout_abort_ = p.search_timeout_abort;
  search_range_ = p.search_range;
  antialias_start_ = p.antialias_start;
  costmap_watchdog_ = rclcpp::Duration::from_seconds(p.costmap_watchdog);

  cc_.max_vel_ = p.max_vel;
  cc_.max_ang_vel_ = p.max_ang_vel;
  cc_.min_curve_radius_ = p.min_curve_radius;
  cc_.weight_decel_ = p.weight_decel;
  cc_.weight_backward_ = p.weight_backward;
  cc_.weight_ang_vel_ = p.weight_ang_vel;
  cc_.weight_costmap_ = p.weight_costmap;
  cc_.weight_costmap_turn_ = p.weight_costmap_turn;
  cc_.weight_remembered_ = p.weight_remembered;
  cc_.in_place_turn_ = p.cost_in_place_turn;
  cc_.hysteresis_max_dist_ = p.hysteresis_max_dist;
  cc_.hysteresis_expand_ = p.hysteresis_expand;
  cc_.weight_hysteresis_ = p.weight_hysteresis;
  cc_.weight_costmap_turn_heuristics_ = p.weight_costmap_turn_heuristics;
  cc_.turn_penalty_cost_threshold_ = p.turn_penalty_cost_threshold;

  goal_tolerance_lin_f_ = p.goal_tolerance_lin;
  goal_tolerance_ang_f_ = p.goal_tolerance_ang;
  goal_tolerance_ang_finish_ = p.goal_tolerance_ang_finish;
  temporary_escape_tolerance_lin_f_ = p.temporary_escape_tolerance_lin;
  temporary_escape_tolerance_ang_f_ = p.temporary_escape_tolerance_ang;

  overwrite_cost_ = p.overwrite_cost;
  relocation_acceptable_cost_ = p.relocation_acceptable_cost;
  hist_ignore_range_f_ = p.hist_ignore_range;
  hist_ignore_range_max_f_ = p.hist_ignore_range_max;

  remember_updates_ = p.remember_updates;
  remember_hit_odds_ = bbf::probabilityToOdds(p.remember_hit_prob);
  remember_miss_odds_ = bbf::probabilityToOdds(p.remember_miss_prob);

  local_range_f_ = p.local_range;
  longcut_range_f_ = p.longcut_range;
  esc_range_f_ = p.esc_range;
  esc_range_min_ratio_ = p.esc_range_min_ratio;
  tolerance_range_f_ = p.tolerance_range;
  tolerance_angle_f_ = p.tolerance_angle;
  find_best_ = p.find_best;
  force_goal_orientation_ = p.force_goal_orientation;
  temporary_escape_ = p.temporary_escape;
  fast_map_update_ = p.fast_map_update;
  max_retry_num_ = p.max_retry_num;
  sw_wait_ = p.sw_wait;

  cost_estim_cache_.setParams(cc_, num_cost_estim_task_);
  cost_estim_cache_static_.setParams(cc_, num_cost_estim_task_);
  ec_ = Astar::Vecf(
    1.0f / cc_.max_vel_, 1.0f / cc_.max_vel_, 1.0f * cc_.weight_ang_vel_ / cc_.max_ang_vel_);

  if (map_info_.linear_resolution != 0.0 && map_info_.angular_resolution != 0.0) {
    resetGridAstarModel(false);
    const Astar::Vec size2d(
      static_cast<int>(map_info_.width), static_cast<int>(map_info_.height), 1);
    const DistanceMap::Params dmp = {
      .euclid_cost = ec_,
      .range = range_,
      .local_range = local_range_,
      .longcut_range =
        static_cast<int>(std::lround(longcut_range_f_ / map_info_.linear_resolution)),
      .size = size2d,
      .resolution = map_info_.linear_resolution,
    };
    cost_estim_cache_.init(model_, dmp);
    if (enable_crowd_mode_) {
      cost_estim_cache_static_.init(model_, dmp);
    }
  }

  keep_a_part_of_previous_path_ = p.keep_a_part_of_previous_path;
  StartPosePredictor::Config start_pose_predictor_config;
  start_pose_predictor_config.lin_vel_ = p.max_vel;
  start_pose_predictor_config.ang_vel_ = p.max_ang_vel;
  start_pose_predictor_config.dist_stop_ = p.dist_stop_to_previous_path;
  start_pose_predictor_config.prediction_sec_ = 1.0 / freq_;
  start_pose_predictor_config.switch_back_prediction_sec_ = p.sw_wait;
  if (keep_a_part_of_previous_path_) {
    // No need to wait additional times
    sw_wait_ = 1.0 / freq_;
  } else {
    sw_wait_ = p.sw_wait;
  }
  start_pose_predictor_.setConfig(start_pose_predictor_config);
}

void Planner3dCore::resetGridAstarModel(const bool force_reset)
{
  const int previous_range = range_;
  range_ = static_cast<int>(search_range_ / map_info_.linear_resolution);
  hist_ignore_range_ = std::lround(hist_ignore_range_f_ / map_info_.linear_resolution);
  hist_ignore_range_max_ = std::lround(hist_ignore_range_max_f_ / map_info_.linear_resolution);
  local_range_ = std::lround(local_range_f_ / map_info_.linear_resolution);
  esc_range_ = std::lround(esc_range_f_ / map_info_.linear_resolution);
  esc_range_min_ = std::lround(esc_range_f_ * esc_range_min_ratio_ / map_info_.linear_resolution);
  esc_angle_ = map_info_.angle / 8;
  tolerance_range_ = std::lround(tolerance_range_f_ / map_info_.linear_resolution);
  tolerance_angle_ = std::lround(tolerance_angle_f_ / map_info_.angular_resolution);
  goal_tolerance_lin_ = std::lround(goal_tolerance_lin_f_ / map_info_.linear_resolution);
  goal_tolerance_ang_ = std::lround(goal_tolerance_ang_f_ / map_info_.angular_resolution);
  temporary_escape_tolerance_lin_ =
    std::lround(temporary_escape_tolerance_lin_f_ / map_info_.linear_resolution);
  temporary_escape_tolerance_ang_ =
    std::lround(temporary_escape_tolerance_ang_f_ / map_info_.angular_resolution);
  cc_.angle_resolution_aspect_ = 2.0 / tanf(map_info_.angular_resolution);

  const bool reset_required = force_reset || (previous_range != range_);
  if (reset_required) {
    model_.reset(new GridAstarModel3D(
      map_info_, ec_, local_range_, cost_estim_cache_.gridmap(), cm_, cm_hyst_, cm_rough_, cc_,
      range_, path_interpolation_resolution_, grid_enumeration_resolution_));
  } else {
    model_->updateCostParameters(ec_, cc_, local_range_);
  }
}

Planner3dCore::Astar::Vec Planner3dCore::metric2Grid(const geometry_msgs::msg::Pose & pose) const
{
  Astar::Vec grid;
  grid_metric_converter::metric2Grid(
    map_info_, grid[0], grid[1], grid[2], pose.position.x, pose.position.y,
    tf2::getYaw(pose.orientation));
  grid.cycleUnsigned(map_info_.angle);
  return grid;
}

geometry_msgs::msg::Pose Planner3dCore::grid2MetricPose(const Astar::Vec & grid) const
{
  float x, y, yaw;
  grid_metric_converter::grid2Metric(map_info_, grid[0], grid[1], grid[2], x, y, yaw);
  geometry_msgs::msg::Pose pose;
  pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
  pose.position.x = x;
  pose.position.y = y;
  return pose;
}

geometry_msgs::msg::Point32 Planner3dCore::grid2MetricPoint(const Astar::Vec & grid) const
{
  float x, y, yaw;
  grid_metric_converter::grid2Metric(map_info_, grid[0], grid[1], grid[2], x, y, yaw);
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  return point;
}

template <class T>
DiscretePoseStatus Planner3dCore::relocateDiscretePoseIfNeededImpl(
  const T & cm, const int tolerance_range, const int tolerance_angle,
  Astar::Vec & pose_discrete) const
{
  if (!cm.validate(pose_discrete, range_)) {
    return DiscretePoseStatus::OUT_OF_MAP;
  }
  if (cm[pose_discrete] == 100) {
    if (searchAvailablePos(cm, pose_discrete, tolerance_range, tolerance_angle))
      return DiscretePoseStatus::RELOCATED;
    else
      return DiscretePoseStatus::IN_ROCK;
  }
  return DiscretePoseStatus::OK;
}

DiscretePoseStatus Planner3dCore::relocateDiscretePoseIfNeeded(
  Astar::Vec & pose_discrete, const int tolerance_range, const int tolerance_angle,
  bool use_cm_rough) const
{
  if (use_cm_rough) {
    pose_discrete[2] = 0;
    return relocateDiscretePoseIfNeededImpl(
      cm_rough_, tolerance_range, tolerance_angle, pose_discrete);
  } else {
    return relocateDiscretePoseIfNeededImpl(cm_, tolerance_range, tolerance_angle, pose_discrete);
  }
}

template <class T>
bool Planner3dCore::searchAvailablePos(
  const T & cm, Astar::Vec & s, const int xy_range, const int angle_range, int cost_acceptable,
  const int min_xy_range) const
{
  if (cost_acceptable == -1) {
    cost_acceptable = relocation_acceptable_cost_;
  }
  RCLCPP_DEBUG(logger_, "%d, %d  (%d,%d,%d)", xy_range, angle_range, s[0], s[1], s[2]);

  float range_min = std::numeric_limits<float>::max();
  Astar::Vec s_out;
  Astar::Vec d;
  for (d[2] = -angle_range; d[2] <= angle_range; d[2]++) {
    for (d[0] = -xy_range; d[0] <= xy_range; d[0]++) {
      for (d[1] = -xy_range; d[1] <= xy_range; d[1]++) {
        if (d[0] == 0 && d[1] == 0 && d[2] == 0) continue;
        if (d.sqlen() > xy_range * xy_range) continue;
        if (d.sqlen() < min_xy_range * min_xy_range) continue;

        Astar::Vec s2 = s + d;
        if (
          (unsigned int)s2[0] >= (unsigned int)map_info_.width ||
          (unsigned int)s2[1] >= (unsigned int)map_info_.height)
          continue;
        s2.cycleUnsigned(map_info_.angle);
        if (!cm_.validate(s2, range_)) continue;

        if (cm_[s2] >= cost_acceptable) continue;
        const auto cost = model_->euclidCost(d);
        if (cost < range_min) {
          range_min = cost;
          s_out = s2;
        }
      }
    }
  }

  if (range_min == std::numeric_limits<float>::max()) {
    if (cost_acceptable != 100) {
      return searchAvailablePos(cm, s, xy_range, angle_range, 100);
    }
    return false;
  }
  s = s_out;
  s.cycleUnsigned(map_info_.angle);
  RCLCPP_DEBUG(logger_, "    (%d,%d,%d)", s[0], s[1], s[2]);
  return true;
}

std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> Planner3dCore::setMap(
  const std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3D> & msg)
{
  RCLCPP_INFO(logger_, "Map received");
  RCLCPP_INFO(
    logger_, " linear_resolution %0.2f x (%dx%d) px", msg->info.linear_resolution, msg->info.width,
    msg->info.height);
  RCLCPP_INFO(
    logger_, " angular_resolution %0.2f x %d px", msg->info.angular_resolution, msg->info.angle);
  RCLCPP_INFO(
    logger_, " origin %0.3f m, %0.3f m, %0.3f rad", msg->info.origin.position.x,
    msg->info.origin.position.y, tf2::getYaw(msg->info.origin.orientation));

  // Stop robot motion until next planning step
  publishEmptyPath();

  ec_ = Astar::Vecf(
    1.0f / cc_.max_vel_, 1.0f / cc_.max_vel_, 1.0f * cc_.weight_ang_vel_ / cc_.max_ang_vel_);

  if (
    map_info_.linear_resolution != msg->info.linear_resolution ||
    map_info_.angular_resolution != msg->info.angular_resolution) {
    map_info_ = msg->info;
    resetGridAstarModel(true);
    RCLCPP_DEBUG(logger_, "Search model updated");
  } else {
    map_info_ = msg->info;
  }
  map_header_ = msg->header;

  const int size[3] = {
    static_cast<int>(map_info_.width),
    static_cast<int>(map_info_.height),
    static_cast<int>(map_info_.angle),
  };

  const Astar::Vec size3d(size[0], size[1], size[2]);
  const Astar::Vec size2d(size[0], size[1], 1);

  as_.reset(size3d);
  cm_.reset(size3d);
  cm_hyst_.reset(size3d);

  const DistanceMap::Params dmp = {
    .euclid_cost = ec_,
    .range = range_,
    .local_range = local_range_,
    .longcut_range = static_cast<int>(std::lround(longcut_range_f_ / map_info_.linear_resolution)),
    .size = size2d,
    .resolution = map_info_.linear_resolution,
  };
  cost_estim_cache_.init(model_, dmp);
  if (enable_crowd_mode_) {
    cost_estim_cache_static_.init(model_, dmp);
  }
  cm_rough_.reset(size2d);
  cm_updates_.reset(size2d);
  bbf_costmap_->reset(size2d);

  Astar::Vec p;
  for (p[0] = 0; p[0] < static_cast<int>(map_info_.width); p[0]++) {
    for (p[1] = 0; p[1] < static_cast<int>(map_info_.height); p[1]++) {
      int cost_min = 100;
      for (p[2] = 0; p[2] < static_cast<int>(map_info_.angle); p[2]++) {
        const size_t addr = ((p[2] * size[1]) + p[1]) * size[0] + p[0];
        char c = msg->data[addr];
        if (c < 0) c = unknown_cost_;
        cm_[p] = c;
        if (c < cost_min) cost_min = c;
      }
      p[2] = 0;
      cm_rough_[p] = cost_min;
    }
  }
  RCLCPP_DEBUG(logger_, "Map copied");

  cm_hyst_.clear(100);
  hyst_updated_cells_.clear();
  has_hysteresis_map_ = false;

  cm_updates_.clear(0);

  has_map_ = true;

  cm_rough_base_ = cm_rough_;
  cm_base_ = cm_;
  bbf_costmap_->clear();

  prev_map_update_x_min_ = map_info_.width;
  prev_map_update_x_max_ = 0;
  prev_map_update_y_min_ = map_info_.height;
  prev_map_update_y_max_ = 0;

  cost_estim_cache_static_update_.min[0] = map_info_.width;
  cost_estim_cache_static_update_.max[0] = 0;
  cost_estim_cache_static_update_.min[1] = map_info_.height;
  cost_estim_cache_static_update_.max[1] = 0;
  cost_estim_cache_static_update_.min[2] = 0;
  cost_estim_cache_static_update_.max[2] = 0;

  createCostEstimCache();

  if (
    map_update_retained_ &&
    rclcpp::Time(map_update_retained_->header.stamp) >= rclcpp::Time(msg->header.stamp)) {
    // The caller re-applies it and then calls clearRetainedMapUpdate().
    return map_update_retained_;
  }
  map_update_retained_ = nullptr;
  return nullptr;
}

void Planner3dCore::clearRetainedMapUpdate() { map_update_retained_ = nullptr; }

void Planner3dCore::applyCostmapUpdate(
  const std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> & msg)
{
  const auto ts_cm_init_start = std::chrono::steady_clock::now();
  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();

  const int map_update_x_min = static_cast<int>(msg->x);
  const int map_update_x_max = std::max(static_cast<int>(msg->x + msg->width) - 1, 0);
  const int map_update_y_min = static_cast<int>(msg->y);
  const int map_update_y_max = std::max(static_cast<int>(msg->y + msg->height) - 1, 0);

  if (
    static_cast<size_t>(map_update_x_max) >= map_info_.width ||
    static_cast<size_t>(map_update_y_max) >= map_info_.height || msg->angle > map_info_.angle) {
    RCLCPP_WARN(
      logger_, "Map update out of range (update range: %dx%dx%d, map range: %dx%dx%d)",
      map_update_x_max, map_update_y_max, msg->angle, map_info_.width, map_info_.height,
      map_info_.angle);
    map_update_retained_ = msg;
    return;
  }

  last_costmap_ = now;

  cm_.copy_partially(
    cm_base_, Astar::Vec(prev_map_update_x_min_, prev_map_update_y_min_, 0),
    Astar::Vec(
      prev_map_update_x_max_, prev_map_update_y_max_, static_cast<int>(map_info_.angle) - 1));
  cm_rough_.copy_partially(
    cm_rough_base_, Astar::Vec(prev_map_update_x_min_, prev_map_update_y_min_, 0),
    Astar::Vec(prev_map_update_x_max_, prev_map_update_y_max_, 0));
  cm_updates_.clear_partially(
    -1, Astar::Vec(prev_map_update_x_min_, prev_map_update_y_min_, 0),
    Astar::Vec(prev_map_update_x_max_, prev_map_update_y_max_, 0));

  // Should search 1px around the region to
  // update the costmap even if the edge of the local map is obstacle
  const int search_range_x_min =
    std::max(0, std::min(prev_map_update_x_min_, map_update_x_min) - 1);
  const int search_range_x_max = std::min(
    static_cast<int>(map_info_.width - 1), std::max(prev_map_update_x_max_, map_update_x_max) + 1);
  const int search_range_y_min =
    std::max(0, std::min(prev_map_update_y_min_, map_update_y_min) - 1);
  const int search_range_y_max = std::min(
    static_cast<int>(map_info_.height - 1), std::max(prev_map_update_y_max_, map_update_y_max) + 1);

  prev_map_update_x_min_ = map_update_x_min;
  prev_map_update_x_max_ = map_update_x_max;
  prev_map_update_y_min_ = map_update_y_min;
  prev_map_update_y_max_ = map_update_y_max;

  cost_estim_cache_static_update_.min[0] =
    std::min(cost_estim_cache_static_update_.min[0], map_update_x_min);
  cost_estim_cache_static_update_.max[0] =
    std::max(cost_estim_cache_static_update_.max[0], map_update_x_max);
  cost_estim_cache_static_update_.min[1] =
    std::min(cost_estim_cache_static_update_.min[1], map_update_y_min);
  cost_estim_cache_static_update_.max[1] =
    std::max(cost_estim_cache_static_update_.max[1], map_update_y_max);

  bool clear_hysteresis(false);

  {
    const Astar::Vec gp(
      static_cast<int>(msg->x), static_cast<int>(msg->y), static_cast<int>(msg->yaw));
    const Astar::Vec gp_rough(gp[0], gp[1], 0);
    for (Astar::Vec p(0, 0, 0); p[0] < static_cast<int>(msg->width); p[0]++) {
      for (p[1] = 0; p[1] < static_cast<int>(msg->height); p[1]++) {
        int cost_min = 100;
        for (p[2] = 0; p[2] < static_cast<int>(msg->angle); p[2]++) {
          const size_t addr = ((p[2] * msg->height) + p[1]) * msg->width + p[0];
          const char c = msg->data[addr];
          if (c < cost_min) cost_min = c;
          if (c == 100 && !clear_hysteresis && cm_hyst_[gp + p] == 0) clear_hysteresis = true;
        }
        p[2] = 0;
        cm_updates_[gp_rough + p] = cost_min;
        if (cost_min > cm_rough_[gp_rough + p]) cm_rough_[gp_rough + p] = cost_min;

        for (p[2] = 0; p[2] < static_cast<int>(msg->angle); p[2]++) {
          const size_t addr = ((p[2] * msg->height) + p[1]) * msg->width + p[0];
          const char c = msg->data[addr];
          if (overwrite_cost_) {
            if (c >= 0) cm_[gp + p] = c;
          } else {
            if (cm_[gp + p] < c) cm_[gp + p] = c;
          }
        }
      }
    }
  }
  map_update_retained_ = nullptr;
  const auto ts_cm_init_end = std::chrono::steady_clock::now();
  const float ts_cm_init_dur =
    std::chrono::duration<float>(ts_cm_init_end - ts_cm_init_start).count();
  RCLCPP_DEBUG(logger_, "Costmaps updated (%.4f)", ts_cm_init_dur);
  metrics_.data.push_back(
    neonavigation_metrics_msgs::metric("costmap_dur", ts_cm_init_dur, "second"));

  if (clear_hysteresis && has_hysteresis_map_) {
    RCLCPP_INFO(logger_, "The previous path collides to the obstacle. Clearing hysteresis map.");
    clearHysteresis();
    has_hysteresis_map_ = false;
  }

  if (!has_start_) return;

  const Astar::Vec s = metric2Grid(start_.pose);

  if (remember_updates_) {
    const auto ts = std::chrono::steady_clock::now();
    bbf_costmap_->remember(
      &cm_updates_, s, remember_hit_odds_, remember_miss_odds_, hist_ignore_range_,
      hist_ignore_range_max_);
    publishRememberedMap();
    bbf_costmap_->updateCostmap();
    const auto tnow = std::chrono::steady_clock::now();
    const float dur = std::chrono::duration<float>(tnow - ts).count();
    RCLCPP_DEBUG(logger_, "Remembered costmap updated (%0.4f sec.)", dur);
  }
  if (!has_goal_) return;

  if (!fast_map_update_) {
    createCostEstimCache(false);
    return;
  }

  const Astar::Vec e = metric2Grid(goal_.pose);

  if (cm_[e] == 100) {
    createCostEstimCache(false);
    return;
  }

  {
    const auto ts = std::chrono::steady_clock::now();
    cost_estim_cache_.update(
      s, e,
      DistanceMap::Rect(
        Astar::Vec(search_range_x_min, search_range_y_min, 0),
        Astar::Vec(search_range_x_max, search_range_y_max, 0)));
    const auto tnow = std::chrono::steady_clock::now();
    const float dur = std::chrono::duration<float>(tnow - ts).count();
    RCLCPP_DEBUG(logger_, "Cost estimation cache updated (%0.4f sec.)", dur);
    metrics_.data.push_back(
      neonavigation_metrics_msgs::metric("distance_map_update_dur", dur, "second"));
    metrics_.data.push_back(
      neonavigation_metrics_msgs::metric("distance_map_init_dur", 0.0, "second"));
  }

  const DistanceMap::DebugData dm_debug = cost_estim_cache_.getDebugData();
  if (dm_debug.has_negative_cost) {
    RCLCPP_WARN(logger_, "Negative cost value is detected. Limited to zero.");
  }
  RCLCPP_DEBUG(
    logger_, "Cost estimation cache search queue initial size: %lu, capacity: %lu",
    dm_debug.search_queue_size, dm_debug.search_queue_cap);
  publishDebug();
  return;
}

void Planner3dCore::setStart(const geometry_msgs::msg::PoseStamped & start)
{
  start_ = start;
  has_start_ = true;
}

void Planner3dCore::clearStart() { has_start_ = false; }

Planner3dCore::SetGoalResult Planner3dCore::setGoal(const geometry_msgs::msg::PoseStamped & msg)
{
  if (msg.header.frame_id != map_header_.frame_id) {
    RCLCPP_ERROR(
      logger_, "Goal [%s] pose must be in the map frame [%s].", msg.header.frame_id.c_str(),
      map_header_.frame_id.c_str());
    return SetGoalResult::REJECTED;
  }

  goal_original_ = goal_raw_ = goal_ = msg;

  const double len2 = goal_.pose.orientation.x * goal_.pose.orientation.x +
                      goal_.pose.orientation.y * goal_.pose.orientation.y +
                      goal_.pose.orientation.z * goal_.pose.orientation.z +
                      goal_.pose.orientation.w * goal_.pose.orientation.w;
  if (std::abs(len2 - 1.0) < 0.1) {
    escape_status_ = TemporaryEscapeStatus::NOT_ESCAPING;
    has_goal_ = true;
    cnt_stuck_ = 0;
    if (!createCostEstimCache()) {
      has_goal_ = false;
      return SetGoalResult::REJECTED;
    }
    status_.status = planner_cspace_msgs::msg::PlannerStatus::DOING;
    status_.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    if (cb_.publish_status) cb_.publish_status();
    return SetGoalResult::ACCEPTED;
  }
  has_goal_ = false;
  return SetGoalResult::CLEARED;
}

void Planner3dCore::clearGoal()
{
  has_goal_ = false;
  escape_status_ = TemporaryEscapeStatus::NOT_ESCAPING;
  status_.status = planner_cspace_msgs::msg::PlannerStatus::DONE;
}

void Planner3dCore::setGoalTolerance(const GoalTolerance & tolerance)
{
  has_goal_tolerance_ = true;
  goal_tolerance_ = tolerance;
}

void Planner3dCore::clearGoalTolerance() { has_goal_tolerance_ = false; }

void Planner3dCore::triggerTemporaryEscape()
{
  if (!has_map_) {
    // metric2Grid requires map_info_
    return;
  }
  updateTemporaryEscapeGoal(metric2Grid(start_.pose), false);
}

void Planner3dCore::forgetRememberedCostmap()
{
  RCLCPP_WARN(logger_, "Forgetting remembered costmap.");
  if (has_map_) bbf_costmap_->clear();
}

void Planner3dCore::clearRememberedCostmap() { bbf_costmap_->clear(); }

bool Planner3dCore::makePlanOnDemand(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  const double tolerance, nav_msgs::msg::Path & plan)
{
  if (!has_map_) {
    RCLCPP_ERROR(logger_, "make_plan service is called without map.");
    return false;
  }

  if (
    start.header.frame_id != map_header_.frame_id || goal.header.frame_id != map_header_.frame_id) {
    RCLCPP_ERROR(
      logger_, "Start [%s] and Goal [%s] poses must be in the map frame [%s].",
      start.header.frame_id.c_str(), goal.header.frame_id.c_str(), map_header_.frame_id.c_str());
    return false;
  }

  Astar::Vec s = metric2Grid(start.pose);
  Astar::Vec e = metric2Grid(goal.pose);
  RCLCPP_INFO(logger_, "Path plan from (%d, %d) to (%d, %d)", s[0], s[1], e[0], e[1]);

  const int tolerance_range = std::lround(tolerance / map_info_.linear_resolution);
  const DiscretePoseStatus start_status =
    relocateDiscretePoseIfNeeded(s, tolerance_range, tolerance_angle_, true);
  const DiscretePoseStatus goal_status =
    relocateDiscretePoseIfNeeded(e, tolerance_range, tolerance_angle_, true);
  switch (start_status) {
    case DiscretePoseStatus::OUT_OF_MAP:
      RCLCPP_ERROR(logger_, "Given start is not on the map.");
      return false;
    case DiscretePoseStatus::IN_ROCK:
      RCLCPP_ERROR(logger_, "Given start is in Rock.");
      return false;
    case DiscretePoseStatus::RELOCATED:
      RCLCPP_INFO(logger_, "Given start is moved (%d, %d)", s[0], s[1]);
      break;
    default:
      break;
  }
  switch (goal_status) {
    case DiscretePoseStatus::OUT_OF_MAP:
      RCLCPP_ERROR(logger_, "Given goal is not on the map.");
      return false;
    case DiscretePoseStatus::IN_ROCK:
      RCLCPP_ERROR(logger_, "Given goal is in Rock.");
      return false;
    case DiscretePoseStatus::RELOCATED:
      RCLCPP_INFO(logger_, "Given goal is moved (%d, %d)", e[0], e[1]);
      break;
    default:
      break;
  }

  const auto cb_progress = [](
                             const std::list<Astar::Vec> & /* path_grid */,
                             const SearchStats & /* stats */) { return true; };

  const auto ts = std::chrono::steady_clock::now();

  GridAstarModel2D::Ptr model_2d(new GridAstarModel2D(model_));

  std::list<Astar::Vec> path_grid;
  std::vector<GridAstarModel3D::VecWithCost> starts;
  starts.emplace_back(s);
  if (!as_.search(starts, e, path_grid, model_2d, cb_progress, 0, 1.0f / freq_min_, find_best_)) {
    RCLCPP_WARN(logger_, "Path plan failed (goal unreachable)");
    return false;
  }
  const auto tnow = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger_, "Path found (%0.4f sec.)", std::chrono::duration<float>(tnow - ts).count());

  nav_msgs::msg::Path path;
  path.header = map_header_;
  path.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  const std::list<Astar::Vecf> path_interpolated = model_->interpolatePath(path_grid);
  grid_metric_converter::appendGridPath2MetricPath(map_info_, path_interpolated, path);

  plan.header = map_header_;
  plan.poses.resize(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    plan.poses[i] = path.poses[i];
  }
  return true;
}

bool Planner3dCore::createCostEstimCache(const bool goal_changed)
{
  if (!has_goal_) return true;

  if (!has_map_ || !has_start_) {
    RCLCPP_ERROR(
      logger_, "Goal received, however map/goal/start are not ready. (%d/%d/%d)",
      static_cast<int>(has_map_), static_cast<int>(has_goal_), static_cast<int>(has_start_));
    return true;
  }

  cost_estim_cache_created_ = false;
  is_start_occupied_ = false;

  Astar::Vec s = metric2Grid(start_.pose);
  Astar::Vec e = metric2Grid(goal_raw_.pose);
  if (goal_changed) {
    RCLCPP_INFO(
      logger_, "New goal received. Metric: (%.3f, %.3f, %.3f), Grid: (%d, %d, %d)",
      goal_raw_.pose.position.x, goal_raw_.pose.position.y, tf2::getYaw(goal_raw_.pose.orientation),
      e[0], e[1], e[2]);
    clearHysteresis();
    has_hysteresis_map_ = false;
  }
  const DiscretePoseStatus start_pose_status =
    relocateDiscretePoseIfNeeded(s, tolerance_range_, tolerance_angle_);
  const DiscretePoseStatus goal_pose_status =
    relocateDiscretePoseIfNeeded(e, tolerance_range_, tolerance_angle_);
  switch (start_pose_status) {
    case DiscretePoseStatus::OUT_OF_MAP:
      RCLCPP_ERROR(logger_, "You are on the edge of the world.");
      return false;
    case DiscretePoseStatus::IN_ROCK:
      RCLCPP_WARN(logger_, "Oops! You are in Rock!");
      ++cnt_stuck_;
      is_start_occupied_ = true;
      return true;
    default:
      break;
  }
  switch (goal_pose_status) {
    case DiscretePoseStatus::OUT_OF_MAP:
      RCLCPP_ERROR(logger_, "Given goal is not on the map.");
      return false;
    case DiscretePoseStatus::IN_ROCK:
      if (isEscaping(escape_status_)) {
        RCLCPP_WARN(logger_, "Oops! Temporary goal is in Rock! Reverting the temporary goal.");
        goal_raw_ = goal_ = goal_original_;
        escape_status_ = TemporaryEscapeStatus::NOT_ESCAPING;
        return true;
      }
      RCLCPP_WARN(logger_, "Oops! Goal is in Rock!");
      ++cnt_stuck_;
      if (temporary_escape_ && enable_crowd_mode_) {
        updateTemporaryEscapeGoal(s);
      }
      return true;
    case DiscretePoseStatus::RELOCATED:
      goal_.pose = grid2MetricPose(e);
      RCLCPP_INFO(
        logger_, "Goal moved. Metric: (%.3f, %.3f, %.3f), Grid: (%d, %d, %d)",
        goal_.pose.position.x, goal_.pose.position.y, tf2::getYaw(goal_.pose.orientation), e[0],
        e[1], e[2]);
      break;
    default:
      const Astar::Vec e_prev = metric2Grid(goal_.pose);
      if (e[0] != e_prev[0] || e[1] != e_prev[1] || e[2] != e_prev[2]) {
        RCLCPP_INFO(
          logger_, "Goal reverted. Metric: (%.3f, %.3f, %.3f), Grid: (%d, %d, %d)",
          goal_raw_.pose.position.x, goal_raw_.pose.position.y,
          tf2::getYaw(goal_raw_.pose.orientation), e[0], e[1], e[2]);
      }
      goal_ = goal_raw_;
      break;
  }

  {
    const auto ts = std::chrono::steady_clock::now();
    cost_estim_cache_.create(s, e);
    const auto tnow = std::chrono::steady_clock::now();
    const float dur = std::chrono::duration<float>(tnow - ts).count();
    RCLCPP_DEBUG(logger_, "Cost estimation cache generated (%0.4f sec.)", dur);

    metrics_.data.push_back(
      neonavigation_metrics_msgs::metric("distance_map_init_dur", dur, "second"));
    metrics_.data.push_back(
      neonavigation_metrics_msgs::metric("distance_map_update_dur", 0.0, "second"));
  }

  publishDebug();

  cost_estim_cache_created_ = true;

  return true;
}

void Planner3dCore::clearHysteresis()
{
  for (const Astar::Vec & p : hyst_updated_cells_) {
    cm_hyst_[p] = 100;
  }
  hyst_updated_cells_.clear();
}

void Planner3dCore::publishDebug()
{
  if (cb_.publish_debug_maps) cb_.publish_debug_maps();
}

void Planner3dCore::publishRememberedMap()
{
  if (cb_.publish_remembered_map) cb_.publish_remembered_map();
}

sensor_msgs::msg::PointCloud Planner3dCore::generateDistanceMapMsg() const
{
  sensor_msgs::msg::PointCloud distance_map;
  distance_map.header = map_header_;
  distance_map.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  distance_map.channels.resize(1);
  distance_map.channels[0].name = "distance";
  distance_map.points.reserve(1024);
  distance_map.channels[0].values.reserve(1024);
  const float k_dist = map_info_.linear_resolution * cc_.max_vel_;
  for (Astar::Vec p(0, 0, 0); p[1] < cost_estim_cache_.size()[1]; p[1]++) {
    for (p[0] = 0; p[0] < cost_estim_cache_.size()[0]; p[0]++) {
      p[2] = 0;
      const float cost = cost_estim_cache_[p];
      if (cost == std::numeric_limits<float>::max()) continue;

      geometry_msgs::msg::Point32 point = grid2MetricPoint(p);
      point.z = cost / 500;
      distance_map.points.push_back(point);
      distance_map.channels[0].values.push_back(cost * k_dist);
    }
  }
  return distance_map;
}

nav_msgs::msg::OccupancyGrid Planner3dCore::generateHysteresisMapMsg() const
{
  nav_msgs::msg::OccupancyGrid hysteresis_map;
  hysteresis_map.header.frame_id = map_header_.frame_id;
  hysteresis_map.info.resolution = map_info_.linear_resolution;
  hysteresis_map.info.width = map_info_.width;
  hysteresis_map.info.height = map_info_.height;
  hysteresis_map.info.origin = map_info_.origin;
  hysteresis_map.data.resize(map_info_.width * map_info_.height, 100);

  for (Astar::Vec p(0, 0, 0); p[1] < cost_estim_cache_.size()[1]; p[1]++) {
    for (p[0] = 0; p[0] < cost_estim_cache_.size()[0]; p[0]++) {
      if (cost_estim_cache_[p] == std::numeric_limits<float>::max()) continue;

      char cost = 100;
      for (Astar::Vec p2 = p; p2[2] < static_cast<int>(map_info_.angle); ++p2[2]) {
        cost = std::min(cm_hyst_[p2], cost);
      }
      hysteresis_map.data[p[0] + p[1] * map_info_.width] = cost;
    }
  }
  return hysteresis_map;
}

nav_msgs::msg::OccupancyGrid Planner3dCore::generateRememberedMapMsg() const
{
  nav_msgs::msg::OccupancyGrid remembered_map;
  remembered_map.header.frame_id = map_header_.frame_id;
  remembered_map.info.resolution = map_info_.linear_resolution;
  remembered_map.info.width = map_info_.width;
  remembered_map.info.height = map_info_.height;
  remembered_map.info.origin = map_info_.origin;
  remembered_map.data.resize(map_info_.width * map_info_.height);

  const auto generate_pointcloud = [this, &remembered_map](
                                     const Astar::Vec & p, bbf::BinaryBayesFilter & bbf) {
    remembered_map.data[p[0] + p[1] * map_info_.width] =
      (bbf.getProbability() - bbf::MIN_PROBABILITY) * 100 /
      (bbf::MAX_PROBABILITY - bbf::MIN_PROBABILITY);
  };
  bbf_costmap_->forEach(generate_pointcloud);
  return remembered_map;
}

void Planner3dCore::publishEmptyPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = robot_frame_;
  path.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  publishPath(path);
}

void Planner3dCore::publishFinishPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_header_.frame_id;
  path.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  // Specify single pose to control only orientation
  path.poses.resize(1);
  path.poses[0].header = path.header;
  if (force_goal_orientation_)
    path.poses[0].pose = goal_raw_.pose;
  else
    path.poses[0].pose = goal_.pose;
  publishPath(path);
}

void Planner3dCore::publishPath(const nav_msgs::msg::Path & path)
{
  if (cb_.publish_path) cb_.publish_path(path);
  previous_path_ = path;
}

void Planner3dCore::publishStartAndGoalMarkers(
  const Astar::Vec & start_grid, const Astar::Vec & end_grid)
{
  if (!cb_.publish_start_and_end) return;

  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header = map_header_;
  start_pose.pose = grid2MetricPose(start_grid);
  geometry_msgs::msg::PoseStamped end_pose;
  end_pose.header = map_header_;
  end_pose.pose = grid2MetricPose(end_grid);
  cb_.publish_start_and_end(start_pose, end_pose);
}

geometry_msgs::msg::PoseStamped Planner3dCore::currentGoalStamped() const
{
  geometry_msgs::msg::PoseStamped p(goal_);
  p.header = map_header_;
  return p;
}

neonavigation_metrics_msgs::msg::Metrics Planner3dCore::collectMetrics(const rclcpp::Time & now)
{
  metrics_.header.stamp = now;
  metrics_.data.push_back(neonavigation_metrics_msgs::metric("stuck_cnt", cnt_stuck_, "count"));
  metrics_.data.push_back(neonavigation_metrics_msgs::metric("error", status_.error, "enum"));
  metrics_.data.push_back(neonavigation_metrics_msgs::metric("status", status_.status, "enum"));
  const neonavigation_metrics_msgs::msg::Metrics metrics = metrics_;
  metrics_.data.clear();
  return metrics;
}

bool Planner3dCore::preparePlanCycle(const rclcpp::Time & now)
{
  if (has_map_ && !cost_estim_cache_created_ && has_goal_) {
    createCostEstimCache();
  }
  bool has_costmap(false);
  if (costmap_watchdog_ > rclcpp::Duration(0, 0)) {
    const rclcpp::Duration costmap_delay = now - last_costmap_;
    metrics_.data.push_back(
      neonavigation_metrics_msgs::metric("costmap_delay", costmap_delay.seconds(), "second"));
    if (costmap_delay > costmap_watchdog_) {
      rclcpp::Clock clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(
        logger_, clock, 1000,
        "Navigation is stopping since the costmap is too old (costmap: %0.3f)",
        last_costmap_.seconds());
      status_.error = planner_cspace_msgs::msg::PlannerStatus::DATA_MISSING;
      publishEmptyPath();
    } else {
      has_costmap = true;
    }
  } else {
    metrics_.data.push_back(neonavigation_metrics_msgs::metric("costmap_delay", -1.0, "second"));
    has_costmap = true;
  }
  return has_costmap;
}

bool Planner3dCore::isReadyToPlan() const { return has_map_ && has_goal_ && has_start_; }

Planner3dCore::PlanCycleResult Planner3dCore::runPlanCycle(const rclcpp::Time & now)
{
  is_path_switchback_ = false;
  if (status_.status == planner_cspace_msgs::msg::PlannerStatus::FINISHING) {
    const float yaw_s = tf2::getYaw(start_.pose.orientation);
    float yaw_g = tf2::getYaw(goal_.pose.orientation);
    if (force_goal_orientation_) yaw_g = tf2::getYaw(goal_raw_.pose.orientation);

    float yaw_diff = yaw_s - yaw_g;
    if (yaw_diff > M_PI)
      yaw_diff -= M_PI * 2.0;
    else if (yaw_diff < -M_PI)
      yaw_diff += M_PI * 2.0;
    if (
      std::abs(yaw_diff) <
      (has_goal_tolerance_ ? goal_tolerance_.ang_finish : goal_tolerance_ang_finish_)) {
      status_.status = planner_cspace_msgs::msg::PlannerStatus::DONE;
      has_goal_ = false;
      // Don't publish empty path here in order a path follower
      // to minimize the error to the desired final pose
      RCLCPP_INFO(logger_, "Path plan finished");
      return PlanCycleResult::GOAL_REACHED;
    }
    publishFinishPath();
    return PlanCycleResult::NONE;
  }

  const TemporaryEscapeStatus previous_escape_status = escape_status_;
  bool skip_path_planning = false;
  if (max_retry_num_ != -1 && cnt_stuck_ > max_retry_num_) {
    status_.error = planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
    status_.status = planner_cspace_msgs::msg::PlannerStatus::DONE;
    has_goal_ = false;

    publishEmptyPath();
    RCLCPP_ERROR(logger_, "Exceeded max_retry_num:%d", max_retry_num_);
    return PlanCycleResult::ABORT_MAX_RETRY;
  } else if (!cost_estim_cache_created_) {
    skip_path_planning = true;
    if (is_start_occupied_) {
      status_.error = planner_cspace_msgs::msg::PlannerStatus::IN_ROCK;
    } else {
      status_.error = planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
    }
  } else {
    status_.error = planner_cspace_msgs::msg::PlannerStatus::GOING_WELL;
  }

  if (skip_path_planning) {
    publishEmptyPath();
    return PlanCycleResult::NONE;
  }

  nav_msgs::msg::Path path;
  path.header = map_header_;
  path.header.stamp = now;
  makePlan(start_.pose, goal_.pose, path, true);
  publishPath(path);
  if ((sw_wait_ > 0.0) && !keep_a_part_of_previous_path_) {
    const int sw_index = getSwitchIndex(path);
    is_path_switchback_ = (sw_index >= 0);
    if (is_path_switchback_) sw_pos_ = path.poses[sw_index];
  }
  if (isEscaping(escape_status_) || isEscaping(previous_escape_status)) {
    // Planner error status is obtained by escape_status_ during temporary escape.
    // TODO(at-wat): Add temporary_escape status field to planner_cspace_msgs::msg::PlannerStatus
    status_.error =
      temporaryEscapeStatus2PlannerErrorStatus(escape_status_ | previous_escape_status);
  }
  return PlanCycleResult::NONE;
}

void Planner3dCore::handleNoGoal()
{
  if (!retain_last_error_status_)
    status_.error = planner_cspace_msgs::msg::PlannerStatus::GOING_WELL;
  publishEmptyPath();
}

bool Planner3dCore::isPreviousPathBlocked() const
{
  if (previous_path_.poses.size() <= 1) return false;

  for (const auto & path_pose : previous_path_.poses) {
    if (cm_[metric2Grid(path_pose.pose)] == 100) {
      // Obstacle on the path.
      return true;
    }
  }
  return false;
}

bool Planner3dCore::checkSwitchbackArrival()
{
  if (!is_path_switchback_) return false;

  const float len = std::hypot(
    start_.pose.position.y - sw_pos_.pose.position.y,
    start_.pose.position.x - sw_pos_.pose.position.x);
  const float yaw = tf2::getYaw(start_.pose.orientation);
  const float sw_yaw = tf2::getYaw(sw_pos_.pose.orientation);
  float yaw_diff = yaw - sw_yaw;
  yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));
  if (len < goal_tolerance_lin_f_ && std::fabs(yaw_diff) < goal_tolerance_ang_f_) {
    // robot has arrived at the switchback point
    is_path_switchback_ = false;
    return true;
  }
  return false;
}

bool Planner3dCore::isPathFinishing(
  const Astar::Vec & start_grid, const Astar::Vec & end_grid) const
{
  int g_tolerance_lin, g_tolerance_ang;
  if (isEscaping(escape_status_)) {
    g_tolerance_lin = temporary_escape_tolerance_lin_;
    g_tolerance_ang = temporary_escape_tolerance_ang_;
  } else if (has_goal_tolerance_) {
    g_tolerance_lin = std::lround(goal_tolerance_.lin / map_info_.linear_resolution);
    g_tolerance_ang = std::lround(goal_tolerance_.ang / map_info_.angular_resolution);
  } else {
    g_tolerance_lin = goal_tolerance_lin_;
    g_tolerance_ang = goal_tolerance_ang_;
  }
  Astar::Vec remain = start_grid - end_grid;
  remain.cycle(map_info_.angle);
  return (
    remain.sqlen() <= g_tolerance_lin * g_tolerance_lin && std::abs(remain[2]) <= g_tolerance_ang);
}

StartPoseStatus Planner3dCore::buildStartPoses(
  const geometry_msgs::msg::Pose & start_metric, const geometry_msgs::msg::Pose & end_metric,
  std::vector<Astar::VecWithCost> & result_start_poses)
{
  result_start_poses.clear();
  start_pose_predictor_.clear();

  Astar::Vecf sf;
  grid_metric_converter::metric2Grid(
    map_info_, sf[0], sf[1], sf[2], start_metric.position.x, start_metric.position.y,
    tf2::getYaw(start_metric.orientation));
  Astar::Vec start_grid(
    static_cast<int>(std::floor(sf[0])), static_cast<int>(std::floor(sf[1])), std::lround(sf[2]));
  start_grid.cycleUnsigned(map_info_.angle);

  const Astar::Vec end_grid = metric2Grid(end_metric);

  if (!cm_.validate(start_grid, range_)) {
    RCLCPP_ERROR(logger_, "You are on the edge of the world.");
    return StartPoseStatus::START_OCCUPIED;
  }

  if (keep_a_part_of_previous_path_ && !previous_path_.poses.empty()) {
    Astar::Vec expected_start_grid;
    if (start_pose_predictor_.process(
          start_metric, cm_, map_info_, previous_path_, expected_start_grid)) {
      RCLCPP_DEBUG(
        logger_, "Start grid is moved to (%d, %d, %d) from (%d, %d, %d) by start pose predictor.",
        expected_start_grid[0], expected_start_grid[1], expected_start_grid[2], start_grid[0],
        start_grid[1], start_grid[2]);
      result_start_poses.push_back(Astar::VecWithCost(expected_start_grid));
      return isPathFinishing(start_grid, end_grid) ? StartPoseStatus::FINISHING
                                                   : StartPoseStatus::NORMAL;
    } else {
      return StartPoseStatus::START_OCCUPIED;
    }
  }

  if (antialias_start_) {
    const int x_cand[] = {0, ((sf[0] - start_grid[0]) < 0.5 ? -1 : 1)};
    const int y_cand[] = {0, ((sf[1] - start_grid[1]) < 0.5 ? -1 : 1)};
    for (const int x : x_cand) {
      for (const int y : y_cand) {
        const Astar::Vec p = start_grid + Astar::Vec(x, y, 0);
        if (!cm_.validate(p, range_)) continue;

        const Astar::Vecf subpx = sf - Astar::Vecf(p[0] + 0.5f, p[1] + 0.5f, 0.0f);
        if (subpx.sqlen() > 1.0) continue;
        if (cm_[p] > 99) continue;

        const Astar::Vecf diff = Astar::Vecf(p[0] + 0.5f, p[1] + 0.5f, 0.0f) - sf;
        const double cost =
          std::hypot(diff[0] * ec_[0], diff[1] * ec_[0]) + cm_[p] * cc_.weight_costmap_ / 100.0;
        result_start_poses.push_back(Astar::VecWithCost(p, cost));
      }
    }
  } else if (cm_[start_grid] < 100) {
    result_start_poses.push_back(Astar::VecWithCost(start_grid));
  }
  if (result_start_poses.empty()) {
    const Astar::Vec original_start_grid = start_grid;
    if (!searchAvailablePos(cm_, start_grid, tolerance_range_, tolerance_angle_)) {
      RCLCPP_WARN(logger_, "Oops! You are in Rock!");
      return StartPoseStatus::START_OCCUPIED;
    }
    RCLCPP_INFO(
      logger_, "Start grid is moved to (%d, %d, %d) from (%d, %d, %d) by relocation.",
      start_grid[0], start_grid[1], start_grid[2], original_start_grid[0], original_start_grid[1],
      original_start_grid[2]);
    result_start_poses.push_back(Astar::VecWithCost(start_grid));
  }
  for (const Astar::VecWithCost & s : result_start_poses) {
    if (isPathFinishing(s.v_, end_grid)) {
      return StartPoseStatus::FINISHING;
    }
  }
  return StartPoseStatus::NORMAL;
}

bool Planner3dCore::makePlan(
  const geometry_msgs::msg::Pose & start_metric, const geometry_msgs::msg::Pose & end_metric,
  nav_msgs::msg::Path & path, bool hyst)
{
  const Astar::Vec start_grid = metric2Grid(start_metric);
  const Astar::Vec end_grid = metric2Grid(end_metric);
  publishStartAndGoalMarkers(start_grid, end_grid);

  std::vector<Astar::VecWithCost> starts;
  switch (buildStartPoses(start_metric, end_metric, starts)) {
    case StartPoseStatus::START_OCCUPIED:
      status_.error = planner_cspace_msgs::msg::PlannerStatus::IN_ROCK;
      return false;
    case StartPoseStatus::FINISHING:
      if (isEscaping(escape_status_)) {
        goal_ = goal_raw_ = goal_original_;
        escape_status_ = TemporaryEscapeStatus::NOT_ESCAPING;
        createCostEstimCache();
        RCLCPP_INFO(logger_, "Escaped");
        return true;
      }
      if (has_goal_tolerance_ && goal_tolerance_.continuous_movement_mode) {
        RCLCPP_INFO(logger_, "Robot reached near the goal.");
        if (cb_.goal_reached_in_continuous_mode) cb_.goal_reached_in_continuous_mode();
        has_goal_tolerance_ = false;
      } else {
        status_.status = planner_cspace_msgs::msg::PlannerStatus::FINISHING;
        publishFinishPath();
        RCLCPP_INFO(logger_, "Path plan finishing");
        return true;
      }
      break;
    default:
      break;
  }

  const float initial_2dof_cost = cost_estim_cache_[Astar::Vec(start_grid[0], start_grid[1], 0)];
  // If goal gets occupied, cost_estim_cache_ is not updated to reduce
  // computational cost for clearing huge map. In this case, cm_[e] is 100.
  if (initial_2dof_cost == std::numeric_limits<float>::max() || cm_[end_grid] >= 100) {
    status_.error = planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
    RCLCPP_WARN(logger_, "Goal unreachable.");
    start_pose_predictor_.clear();
    if (temporary_escape_) {
      updateTemporaryEscapeGoal(start_grid);
    }
    return false;
  }

  const float range_limit = initial_2dof_cost - (local_range_ + range_) * ec_[0];
  const auto ts = std::chrono::steady_clock::now();
  const auto cb_progress = [this, ts, start_grid, end_grid](
                             const std::list<Astar::Vec> & /* path_grid */,
                             const SearchStats & stats) -> bool {
    const auto tnow = std::chrono::steady_clock::now();
    const auto tdiff = std::chrono::duration<float>(tnow - ts).count();
    publishEmptyPath();
    if (tdiff > search_timeout_abort_) {
      RCLCPP_ERROR(
        logger_,
        "Search aborted due to timeout. "
        "search_timeout_abort may be too small or planner_3d may have a bug: "
        "s=(%d, %d, %d), g=(%d, %d, %d), tdiff=%0.4f, "
        "num_loop=%lu, "
        "num_search_queue=%lu, "
        "num_prev_updates=%lu, "
        "num_total_updates=%lu, ",
        start_grid[0], start_grid[1], start_grid[2], end_grid[0], end_grid[1], end_grid[2], tdiff,
        stats.num_loop, stats.num_search_queue, stats.num_prev_updates, stats.num_total_updates);
      return false;
    }
    RCLCPP_WARN(logger_, "Search timed out (%0.4f sec.)", tdiff);
    return true;
  };

  model_->enableHysteresis(hyst && has_hysteresis_map_);
  std::list<Astar::Vec> path_grid;
  bool is_goal_same_as_start = false;
  for (const auto s : starts) {
    if (s.v_ == end_grid) {
      RCLCPP_DEBUG(logger_, "The start grid is the same as the end grid. Path planning skipped.");
      path_grid.push_back(end_grid);
      is_goal_same_as_start = true;
      break;
    }
  }
  if (
    !is_goal_same_as_start && !as_.search(
                                starts, end_grid, path_grid, model_, cb_progress, range_limit,
                                1.0f / freq_min_, find_best_)) {
    RCLCPP_WARN(logger_, "Path plan failed (goal unreachable)");
    status_.error = planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
    if (!find_best_) return false;
  }
  const auto tnow = std::chrono::steady_clock::now();
  const float dur = std::chrono::duration<float>(tnow - ts).count();
  RCLCPP_DEBUG(logger_, "Path found (%0.4f sec.)", dur);
  metrics_.data.push_back(neonavigation_metrics_msgs::metric("path_search_dur", dur, "second"));

  geometry_msgs::msg::PoseArray poses;
  poses.header = path.header;
  for (const auto & p : path_grid) {
    poses.poses.push_back(grid2MetricPose(p));
  }
  if (cb_.publish_path_poses) cb_.publish_path_poses(poses);
  if (!start_pose_predictor_.getPreservedPath().poses.empty()) {
    if (cb_.publish_preserved_path_poses)
      cb_.publish_preserved_path_poses(start_pose_predictor_.getPreservedPath());
  }
  const std::list<Astar::Vecf> path_interpolated = model_->interpolatePath(path_grid);
  path.poses = start_pose_predictor_.getPreservedPath().poses;
  grid_metric_converter::appendGridPath2MetricPath(map_info_, path_interpolated, path);

  if (hyst) {
    const auto ts_hyst = std::chrono::steady_clock::now();
    std::unordered_map<Astar::Vec, bool, Astar::Vec> path_points;
    const float max_dist = cc_.hysteresis_max_dist_ / map_info_.linear_resolution;
    const float expand_dist = cc_.hysteresis_expand_ / map_info_.linear_resolution;
    const int path_range = range_ + max_dist + expand_dist + 5;
    for (const Astar::Vecf & p : path_interpolated) {
      Astar::Vec d;
      for (d[0] = -path_range; d[0] <= path_range; d[0]++) {
        for (d[1] = -path_range; d[1] <= path_range; d[1]++) {
          Astar::Vec point = p + d;
          point.cycleUnsigned(map_info_.angle);
          if (
            (unsigned int)point[0] >= (unsigned int)map_info_.width ||
            (unsigned int)point[1] >= (unsigned int)map_info_.height)
            continue;
          path_points[point] = true;
        }
      }
    }

    clearHysteresis();
    for (auto & ps : path_points) {
      const Astar::Vec & p = ps.first;
      float d_min = std::numeric_limits<float>::max();
      auto it_prev = path_interpolated.cbegin();
      for (auto it = path_interpolated.cbegin(); it != path_interpolated.cend(); it++) {
        if (it != it_prev) {
          int yaw = std::lround((*it)[2]) % map_info_.angle;
          int yaw_prev = std::lround((*it_prev)[2]) % map_info_.angle;
          if (yaw < 0) yaw += map_info_.angle;
          if (yaw_prev < 0) yaw_prev += map_info_.angle;
          if (yaw == p[2] || yaw_prev == p[2]) {
            const float d = CyclicVecFloat<3, 2>(p).distLinestrip2d(*it_prev, *it);
            if (d < d_min) d_min = d;
          }
        }
        it_prev = it;
      }
      d_min = std::max(expand_dist, std::min(expand_dist + max_dist, d_min));
      cm_hyst_[p] = std::lround((d_min - expand_dist) * 100.0 / max_dist);
      hyst_updated_cells_.push_back(p);
    }
    has_hysteresis_map_ = true;
    const auto tnow_hyst = std::chrono::steady_clock::now();
    const float dur_hyst = std::chrono::duration<float>(tnow_hyst - ts_hyst).count();
    RCLCPP_DEBUG(logger_, "Hysteresis map generated (%0.4f sec.)", dur_hyst);
    metrics_.data.push_back(neonavigation_metrics_msgs::metric("hyst_map_dur", dur_hyst, "second"));
    publishDebug();
  }

  return true;
}

void Planner3dCore::updateTemporaryEscapeGoal(
  const Astar::Vec & start_grid, const bool log_on_unready)
{
  if (!has_map_ || !has_goal_ || !has_start_) {
    if (log_on_unready) {
      RCLCPP_WARN(logger_, "Not ready to update temporary escape goal");
    }
    return;
  }
  if (is_path_switchback_) {
    RCLCPP_INFO(logger_, "Skipping temporary goal update during switch back");
    return;
  }

  if (!enable_crowd_mode_) {
    // Just find available (not occupied) pose
    Astar::Vec te;
    if (!searchAvailablePos(
          cm_, te, esc_range_, esc_angle_, relocation_acceptable_cost_, esc_range_min_)) {
      RCLCPP_WARN(logger_, "No valid temporary escape goal");
      return;
    }
    escape_status_ = TemporaryEscapeStatus::ESCAPING_WITHOUT_IMPROVEMENT;
    RCLCPP_INFO(logger_, "Temporary goal (%d, %d, %d)", te[0], te[1], te[2]);
    goal_raw_.pose = grid2MetricPose(te);
    goal_ = goal_raw_;

    if (status_.status == planner_cspace_msgs::msg::PlannerStatus::FINISHING) {
      RCLCPP_INFO(
        logger_,
        "Planner was finishing but temporary escape is triggered. Clearing finishing state.");
      status_.status = planner_cspace_msgs::msg::PlannerStatus::DOING;
    }

    createCostEstimCache();
    return;
  }

  // Find available pos with minumum cost on static distance map
  {
    const Astar::Vec s(start_grid[0], start_grid[1], 0);
    if (!cm_.validate(s, esc_range_)) {
      RCLCPP_ERROR(logger_, "Crowd escape is disabled on the edge of the world.");
      return;
    }
    const Astar::Vec g_orig = metric2Grid(goal_original_.pose);

    {
      const auto ts = std::chrono::steady_clock::now();
      if (remember_updates_) {
        cost_estim_cache_static_.update(s, g_orig, cost_estim_cache_static_update_);
        cost_estim_cache_static_update_.min[0] = map_info_.width;
        cost_estim_cache_static_update_.max[0] = 0;
        cost_estim_cache_static_update_.min[1] = map_info_.height;
        cost_estim_cache_static_update_.max[1] = 0;
      } else {
        // Update without region if remember_updates is disabled.
        // Distance map will expand distance map using edges_buf if needed.
        cost_estim_cache_static_.update(
          s, g_orig, DistanceMap::Rect(Astar::Vec(1, 1, 0), Astar::Vec(0, 0, 0)));
      }
      const auto tnow = std::chrono::steady_clock::now();
      const float dur = std::chrono::duration<float>(tnow - ts).count();
      RCLCPP_DEBUG(logger_, "Cost estimation cache for static map updated (%0.4f sec.)", dur);
      metrics_.data.push_back(
        neonavigation_metrics_msgs::metric("distance_map_static_update_dur", dur, "second"));
      metrics_.data.push_back(
        neonavigation_metrics_msgs::metric("distance_map_static_init_dur", 0.0, "second"));
    }

    // Construct arraivability map
    const int local_width = esc_range_ * 2 + 1;
    const Astar::Vec local_origin = s - Astar::Vec(esc_range_, esc_range_, 0);
    const Astar::Vec local_range(local_width, local_width, 0);
    const Astar::Vec local_size(local_width + 1, local_width + 1, 1);
    const Astar::Vec local_center(esc_range_, esc_range_, 0);
    const DistanceMap::Params dmp = {
      .euclid_cost = ec_,
      .range = 0,
      .local_range = 0,
      .longcut_range = esc_range_,
      .size = local_size,
      .resolution = map_info_.linear_resolution,
    };

    cm_local_esc_.reset(local_size);
    arrivable_map_.setParams(cc_, num_cost_estim_task_);
    arrivable_map_.init(model_, dmp);
    cm_local_esc_.copy_partially(
      Astar::Vec(0, 0, 0), cm_rough_, local_origin, local_origin + local_range);
    arrivable_map_.create(local_center, local_center);

    // Find temporary goal
    float cost_min = std::numeric_limits<float>::max();
    Astar::Vec te_out;
    const int esc_range_sq = esc_range_ * esc_range_;
    const int esc_range_min_sq = esc_range_min_ * esc_range_min_;
    for (Astar::Vec d(0, -esc_range_, 0); d[1] <= esc_range_; d[1]++) {
      for (d[0] = -esc_range_; d[0] <= esc_range_; d[0]++) {
        const int sqlen = d.sqlen();
        // Too close escaping range should be checked after original goal arrivability check
        if (sqlen > esc_range_sq) {
          continue;
        }

        Astar::Vec te(start_grid[0] + d[0], start_grid[1] + d[1], 0);
        if (
          (unsigned int)te[0] >= (unsigned int)map_info_.width ||
          (unsigned int)te[1] >= (unsigned int)map_info_.height) {
          continue;
        }
        if (!cm_rough_.validate(te, range_)) {
          continue;
        }

        // Check arrivability
        if (arrivable_map_[te - local_origin] == std::numeric_limits<float>::max()) {
          continue;
        }

        if (te[0] == g_orig[0] && te[1] == g_orig[1] && cm_[g_orig] < 100) {
          // Original goal is in the temporary escape range and reachable
          RCLCPP_INFO(logger_, "Original goal is reachable. Back to the original goal.");
          goal_ = goal_raw_ = goal_original_;
          escape_status_ = TemporaryEscapeStatus::NOT_ESCAPING;
          createCostEstimCache();
          return;
        }

        if ((d[0] == 0 && d[1] == 0) || sqlen < esc_range_min_sq) {
          continue;
        }
        if (cm_rough_[te] >= relocation_acceptable_cost_) {
          continue;
        }

        const auto cost = cost_estim_cache_static_[te];
        if (cost >= cost_min) {
          continue;
        }

        // Calculate distance map gradient
        float grad[2] = {0, 0};
        for (Astar::Vec d2(0, -1, 0); d2[1] <= 1; d2[1]++) {
          for (d2[0] = -1; d2[0] <= 1; d2[0]++) {
            if (d2[0] == 0 && d2[1] == 0) {
              continue;
            }

            const auto p = te + d2;
            const auto cost2 = cost_estim_cache_static_[p];
            if (cost2 == std::numeric_limits<float>::max()) {
              continue;
            }
            const float cost_diff = cost2 - cost;
            grad[0] += -cost_diff * d2[0];
            grad[1] += -cost_diff * d2[1];
          }
        }
        if (grad[0] == 0 && grad[1] == 0) {
          continue;
        }
        const float yaw = std::atan2(grad[1], grad[0]);
        te[2] = static_cast<int>(yaw / map_info_.angular_resolution);

        te.cycleUnsigned(map_info_.angle);
        if (cm_[te] >= relocation_acceptable_cost_) {
          continue;
        }

        if (cost < cost_min) {
          cost_min = cost;
          te_out = te;
        }
      }
    }
    if (cost_min == std::numeric_limits<float>::max()) {
      RCLCPP_WARN(logger_, "No valid temporary escape goal");
      return;
    }

    escape_status_ = cost_min < cost_estim_cache_static_[s]
                       ? TemporaryEscapeStatus::ESCAPING_WITH_IMPROVEMENT
                       : TemporaryEscapeStatus::ESCAPING_WITHOUT_IMPROVEMENT;
    if (isPathFinishing(start_grid, te_out)) {
      // This temporary goal is too close and it will be immediately goes escaped state
      escape_status_ = TemporaryEscapeStatus::ESCAPING_WITHOUT_IMPROVEMENT;
    }

    RCLCPP_INFO(logger_, "Temporary goal (%d, %d, %d)", te_out[0], te_out[1], te_out[2]);
    goal_raw_.pose = grid2MetricPose(te_out);
    goal_ = goal_raw_;

    if (status_.status == planner_cspace_msgs::msg::PlannerStatus::FINISHING) {
      RCLCPP_INFO(
        logger_,
        "Planner was finishing but temporary escape is triggered. Clearing finishing state.");
      status_.status = planner_cspace_msgs::msg::PlannerStatus::DOING;
    }

    createCostEstimCache();
  }
}

int Planner3dCore::getSwitchIndex(const nav_msgs::msg::Path & path) const
{
  geometry_msgs::msg::Pose p_prev;
  bool first(true);
  bool dir_set(false);
  bool dir_prev(false);
  for (auto it = path.poses.begin(); it != path.poses.end(); ++it) {
    const auto & p = *it;
    if (!first) {
      const float x_diff = p.pose.position.x - p_prev.position.x;
      const float y_diff = p.pose.position.y - p_prev.position.y;
      const float len_sq = std::pow(y_diff, 2) + std::pow(x_diff, 2);
      if (len_sq > std::pow(0.001f, 2)) {
        const float yaw = tf2::getYaw(p.pose.orientation);
        const bool dir = (std::cos(yaw) * x_diff + std::sin(yaw) * y_diff < 0);

        if (dir_set && (dir_prev ^ dir)) {
          return std::distance(path.poses.begin(), it);
        }
        dir_prev = dir;
        dir_set = true;
      }
    }
    first = false;
    p_prev = p.pose;
  }
  // -1 means no switchback in the path
  return -1;
}
}  // namespace planner_3d
}  // namespace planner_cspace
