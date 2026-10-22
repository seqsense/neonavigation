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

#ifndef PLANNER_CSPACE__PLANNER_3D__PLANNER_3D_CORE_H_
#define PLANNER_CSPACE__PLANNER_3D__PLANNER_3D_CORE_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "costmap_cspace_msgs/msg/map_meta_data3_d.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "neonavigation_metrics_msgs/msg/metrics.hpp"
#include "planner_cspace/grid_astar.h"
#include "planner_cspace/planner_3d/costmap_bbf.h"
#include "planner_cspace/planner_3d/distance_map.h"
#include "planner_cspace/planner_3d/grid_astar_model.h"
#include "planner_cspace/planner_3d/pose_status.h"
#include "planner_cspace/planner_3d/start_pose_predictor.h"
#include "planner_cspace/planner_3d/temporary_escape.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/header.hpp"

namespace planner_cspace
{
namespace planner_3d
{
// Planner3dCore holds the path planning logic of the planner_3d node.
// It owns the costmaps, the search model, the distance maps, the goal/start
// state and the planner status, but it knows nothing about node handles,
// publishers, subscribers, services, actions or dynamic_reconfigure. Those
// belong to the interface layer (Planner3dNode in src/planner_3d.cpp), which
// pushes data in via the setters below and receives the results either as
// return values or through the injected std::function callbacks.
class Planner3dCore
{
public:
  using Astar = GridAstar<3, 2>;

  // Tunable parameters which the interface layer may update at any time
  // (they are backed by dynamic_reconfigure on the ROS 1 side).
  struct Parameters
  {
    float freq = 4.0f;
    float freq_min = 2.0f;
    float search_timeout_abort = 30.0f;
    float search_range = 0.4f;
    bool antialias_start = false;
    double costmap_watchdog = 0.0;
    float max_vel = 0.3f;
    float max_ang_vel = 0.6f;
    float min_curve_radius = 0.1f;
    float weight_decel = 50.0f;
    float weight_backward = 0.9f;
    float weight_ang_vel = 1.0f;
    float weight_costmap = 50.0f;
    float weight_costmap_turn = 0.0f;
    float weight_costmap_turn_heuristics = 100.0f;
    float weight_remembered = 1000.0f;
    float cost_in_place_turn = 30.0f;
    int turn_penalty_cost_threshold = 0;
    float hysteresis_max_dist = 0.1f;
    float hysteresis_expand = 0.1f;
    float weight_hysteresis = 5.0f;
    double goal_tolerance_lin = 0.05;
    double goal_tolerance_ang = 0.1;
    double goal_tolerance_ang_finish = 0.05;
    double temporary_escape_tolerance_lin = 0.1;
    double temporary_escape_tolerance_ang = 1.57;
    bool overwrite_cost = false;
    int relocation_acceptable_cost = 50;
    double hist_ignore_range = 0.6;
    double hist_ignore_range_max = 1.25;
    bool remember_updates = false;
    double remember_hit_prob = 0.6;
    double remember_miss_prob = 0.3;
    double local_range = 2.5;
    double longcut_range = 0.0;
    double esc_range = 0.25;
    double esc_range_min_ratio = 0.5;
    double tolerance_range = 0.25;
    double tolerance_angle = 0.0;
    bool find_best = true;
    bool force_goal_orientation = true;
    bool temporary_escape = true;
    bool fast_map_update = false;
    int max_retry_num = -1;
    float sw_wait = 2.0f;
    bool keep_a_part_of_previous_path = false;
    double dist_stop_to_previous_path = 0.1;
  };

  // Parameters which are applied only once on startup.
  struct StaticParameters
  {
    int unknown_cost = 100;
    double path_interpolation_resolution = 0.5;
    double grid_enumeration_resolution = 0.1;
    std::string robot_frame = "base_link";
    bool enable_crowd_mode = false;
    bool retain_last_error_status = true;
    int num_threads = 1;
    int num_search_task = 16;
    int num_cost_estim_task = 16;
    int queue_size_limit = 0;
  };

  // Goal tolerances given by a tolerant move goal. While this is set, it
  // takes precedence over the tolerances in Parameters. The interface layer
  // sets it when a tolerant action goal becomes active and clears it when
  // the action becomes inactive.
  struct GoalTolerance
  {
    double lin = 0.0;
    double ang = 0.0;
    double ang_finish = 0.0;
    bool continuous_movement_mode = false;
  };

  // Outputs of the logic. The interface layer maps them to topics/actions.
  struct Callbacks
  {
    // Planned path (empty path means "stop").
    std::function<void(const nav_msgs::msg::Path &)> publish_path;
    // Debug output of the raw grid path.
    std::function<void(const geometry_msgs::msg::PoseArray &)> publish_path_poses;
    // Debug output of the preserved part of the previous path.
    std::function<void(const nav_msgs::msg::Path &)> publish_preserved_path_poses;
    // Debug output of the relocated start/end grids.
    std::function<void(
      const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::PoseStamped &)>
      publish_start_and_end;
    // Distance map and hysteresis map should be published if anyone listens.
    std::function<void()> publish_debug_maps;
    // Remembered costmap should be published if anyone listens.
    std::function<void()> publish_remembered_map;
    // Planner status has been changed and should be published.
    std::function<void()> publish_status;
    // The robot reached near the goal while continuous_movement_mode is set.
    std::function<void()> goal_reached_in_continuous_mode;
  };

  enum class SetGoalResult
  {
    ACCEPTED,
    CLEARED,
    REJECTED,
  };

  // Semantics of a planning cycle which the interface layer has to reflect
  // to the action servers.
  enum class PlanCycleResult
  {
    NONE,
    GOAL_REACHED,
    ABORT_MAX_RETRY,
  };

  explicit Planner3dCore(const rclcpp::Logger & logger);

  void setCallbacks(const Callbacks & cb);
  void initialize(const StaticParameters & p);
  void setParameters(const Parameters & p);

  // --- Inputs -------------------------------------------------------------
  // Applies a new costmap. Returns the retained costmap update which should
  // be re-applied by the caller, or nullptr if there is nothing to re-apply.
  std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> setMap(
    const std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3D> & msg);
  void clearRetainedMapUpdate();
  void applyCostmapUpdate(
    const std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> & msg);
  void setStart(const geometry_msgs::msg::PoseStamped & start);
  void clearStart();
  SetGoalResult setGoal(const geometry_msgs::msg::PoseStamped & msg);
  void clearGoal();
  void setGoalTolerance(const GoalTolerance & tolerance);
  void clearGoalTolerance();
  void triggerTemporaryEscape();
  void forgetRememberedCostmap();
  void clearRememberedCostmap();

  // Plans a path between the given poses without touching the planner state.
  bool makePlanOnDemand(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    const double tolerance, nav_msgs::msg::Path & plan);

  // --- Planning cycle -----------------------------------------------------
  // Creates the pending cost estimation cache and checks the costmap age.
  // Returns false and stops the robot if the costmap is too old.
  bool preparePlanCycle(const rclcpp::Time & now);
  bool isReadyToPlan() const;
  // Runs one planning step and reports what the action servers should do.
  PlanCycleResult runPlanCycle(const rclcpp::Time & now);
  void handleNoGoal();

  // --- Replan trigger helpers --------------------------------------------
  bool isPreviousPathBlocked() const;
  // Returns true when the robot has reached the switchback pose. The
  // switchback state is cleared at the same time.
  bool checkSwitchbackArrival();
  bool isPathSwitchback() const { return is_path_switchback_; }
  float swWait() const { return sw_wait_; }
  const rclcpp::Time & lastCostmapStamp() const { return last_costmap_; }

  // --- Accessors ----------------------------------------------------------
  bool hasMap() const { return has_map_; }
  bool hasGoal() const { return has_goal_; }
  bool hasStart() const { return has_start_; }
  const geometry_msgs::msg::PoseStamped & start() const { return start_; }
  const std_msgs::msg::Header & mapHeader() const { return map_header_; }
  const planner_cspace_msgs::msg::PlannerStatus & status() const { return status_; }
  void setStatusStamp(const rclcpp::Time & stamp) { status_.header.stamp = stamp; }
  geometry_msgs::msg::PoseStamped currentGoalStamped() const;
  // Appends the per-cycle metrics and returns the whole set, clearing the
  // internal buffer.
  neonavigation_metrics_msgs::msg::Metrics collectMetrics(const rclcpp::Time & now);

  // --- Debug outputs ------------------------------------------------------
  sensor_msgs::msg::PointCloud generateDistanceMapMsg() const;
  nav_msgs::msg::OccupancyGrid generateHysteresisMapMsg() const;
  nav_msgs::msg::OccupancyGrid generateRememberedMapMsg() const;

  // The clock the logic reads time from. On ROS 2 a bare rclcpp::Clock never
  // subscribes to /clock, so under use_sim_time it silently is wall time; the
  // interface node hands in its own clock instead. The default keeps the ROS 1
  // build right on its own, where the compat rclcpp::Clock is ros::Time.
  void setClock(const rclcpp::Clock::SharedPtr & clock) { clock_ = clock; }

protected:
  Astar::Vec metric2Grid(const geometry_msgs::msg::Pose & pose) const;
  geometry_msgs::msg::Pose grid2MetricPose(const Astar::Vec & grid) const;
  geometry_msgs::msg::Point32 grid2MetricPoint(const Astar::Vec & grid) const;

  template <class T>
  DiscretePoseStatus relocateDiscretePoseIfNeededImpl(
    const T & cm, const int tolerance_range, const int tolerance_angle,
    Astar::Vec & pose_discrete) const;
  DiscretePoseStatus relocateDiscretePoseIfNeeded(
    Astar::Vec & pose_discrete, const int tolerance_range, const int tolerance_angle,
    bool use_cm_rough = false) const;
  template <class T>
  bool searchAvailablePos(
    const T & cm, Astar::Vec & s, const int xy_range, const int angle_range,
    int cost_acceptable = -1, const int min_xy_range = 0) const;

  bool createCostEstimCache(const bool goal_changed = true);
  void clearHysteresis();
  void resetGridAstarModel(const bool force_reset);
  void publishDebug();
  void publishRememberedMap();
  void publishEmptyPath();
  void publishFinishPath();
  void publishPath(const nav_msgs::msg::Path & path);
  void publishStartAndGoalMarkers(const Astar::Vec & start_grid, const Astar::Vec & end_grid);
  bool isPathFinishing(const Astar::Vec & start_grid, const Astar::Vec & end_grid) const;
  StartPoseStatus buildStartPoses(
    const geometry_msgs::msg::Pose & start_metric, const geometry_msgs::msg::Pose & end_metric,
    std::vector<Astar::VecWithCost> & result_start_poses);
  bool makePlan(
    const geometry_msgs::msg::Pose & start_metric, const geometry_msgs::msg::Pose & end_metric,
    nav_msgs::msg::Path & path, bool hyst);
  void updateTemporaryEscapeGoal(const Astar::Vec & start_grid, const bool log_on_unready = true);
  int getSwitchIndex(const nav_msgs::msg::Path & path) const;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  Callbacks cb_;

  Astar as_;
  Astar::Gridmap<char, 0x40> cm_;
  Astar::Gridmap<char, 0x80> cm_rough_;
  Astar::Gridmap<char, 0x40> cm_base_;
  Astar::Gridmap<char, 0x80> cm_rough_base_;
  Astar::Gridmap<char, 0x80> cm_hyst_;
  Astar::Gridmap<char, 0x80> cm_updates_;
  Astar::Gridmap<char, 0x80> cm_local_esc_;
  CostmapBBF::Ptr bbf_costmap_;
  DistanceMap cost_estim_cache_;
  DistanceMap cost_estim_cache_static_;
  DistanceMap arrivable_map_;
  DistanceMap::Rect cost_estim_cache_static_update_;

  GridAstarModel3D::Ptr model_;

  costmap_cspace_msgs::msg::MapMetaData3D map_info_;
  std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> map_update_retained_;
  std_msgs::msg::Header map_header_;
  float freq_;
  float freq_min_;
  float search_timeout_abort_;
  float search_range_;
  bool antialias_start_;
  int range_;
  int local_range_;
  double local_range_f_;
  double longcut_range_f_;
  int esc_range_;
  int esc_range_min_;
  int esc_angle_;
  double esc_range_f_;
  double esc_range_min_ratio_;
  int tolerance_range_;
  int tolerance_angle_;
  double tolerance_range_f_;
  double tolerance_angle_f_;
  double path_interpolation_resolution_;
  double grid_enumeration_resolution_;
  int unknown_cost_;
  bool overwrite_cost_;
  int relocation_acceptable_cost_;
  bool has_map_;
  bool has_goal_;
  bool has_start_;
  bool has_hysteresis_map_;
  std::vector<Astar::Vec> hyst_updated_cells_;
  bool cost_estim_cache_created_;
  bool remember_updates_;
  bool fast_map_update_;
  double hist_ignore_range_f_;
  int hist_ignore_range_;
  double hist_ignore_range_max_f_;
  int hist_ignore_range_max_;
  bool temporary_escape_;
  float remember_hit_odds_;
  float remember_miss_odds_;
  bool retain_last_error_status_;
  int num_cost_estim_task_;
  bool keep_a_part_of_previous_path_;
  bool enable_crowd_mode_;

  std::string robot_frame_;

  int max_retry_num_;

  // Cost weights
  CostCoeff cc_;

  geometry_msgs::msg::PoseStamped start_;
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped goal_raw_;
  geometry_msgs::msg::PoseStamped goal_original_;
  Astar::Vecf ec_;
  double goal_tolerance_lin_f_;
  double goal_tolerance_ang_f_;
  double goal_tolerance_ang_finish_;
  int goal_tolerance_lin_;
  int goal_tolerance_ang_;
  double temporary_escape_tolerance_lin_f_;
  double temporary_escape_tolerance_ang_f_;
  int temporary_escape_tolerance_lin_;
  int temporary_escape_tolerance_ang_;
  bool has_goal_tolerance_;
  GoalTolerance goal_tolerance_;

  planner_cspace_msgs::msg::PlannerStatus status_;
  neonavigation_metrics_msgs::msg::Metrics metrics_;

  bool find_best_;
  float sw_wait_;
  geometry_msgs::msg::PoseStamped sw_pos_;
  bool is_path_switchback_;

  bool force_goal_orientation_;

  TemporaryEscapeStatus escape_status_;

  int cnt_stuck_;
  bool is_start_occupied_;

  rclcpp::Duration costmap_watchdog_;
  rclcpp::Time last_costmap_;

  int prev_map_update_x_min_;
  int prev_map_update_x_max_;
  int prev_map_update_y_min_;
  int prev_map_update_y_max_;
  nav_msgs::msg::Path previous_path_;
  StartPosePredictor start_pose_predictor_;
};
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE__PLANNER_3D__PLANNER_3D_CORE_H_
