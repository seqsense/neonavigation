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

#include <actionlib/server/simple_action_server.h>
#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <neonavigation_common/compatibility.h>
#include <neonavigation_metrics_msgs/Metrics.h>
#include <planner_cspace/Planner3DConfig.h>
#include <planner_cspace/jump_detector.h>
#include <planner_cspace/planner_3d/planner_3d_core.h>
#include <planner_cspace_msgs/MoveWithToleranceAction.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/converter.h>

#include <functional>
#include <limits>
#include <memory>
#include <sq_ros1_compat/logger.hpp>
#include <sq_ros1_compat/msg_ptr.hpp>
#include <string>

namespace planner_cspace
{
namespace planner_3d
{
// Planner3dNode is the ROS interface of the planner_3d node. It owns the
// publishers, subscribers, services, action servers, dynamic_reconfigure
// server, TF and diagnostics, and delegates all path planning to
// Planner3dCore.
class Planner3dNode
{
protected:
  using Planner3DActionServer = actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>;
  using Planner3DTolerantActionServer =
    actionlib::SimpleActionServer<planner_cspace_msgs::MoveWithToleranceAction>;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_map_update_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_temporary_escape_trigger_;
  ros::Publisher pub_path_;
  ros::Publisher pub_path_velocity_;
  ros::Publisher pub_path_poses_;
  ros::Publisher pub_preserved_path_poses_;
  ros::Publisher pub_hysteresis_map_;
  ros::Publisher pub_distance_map_;
  ros::Publisher pub_remembered_map_;
  ros::Publisher pub_start_;
  ros::Publisher pub_end_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_status_;
  ros::Publisher pub_metrics_;
  ros::ServiceServer srs_forget_;
  ros::ServiceServer srs_make_plan_;

  std::shared_ptr<Planner3DActionServer> act_;
  std::shared_ptr<Planner3DTolerantActionServer> act_tolerant_;
  planner_cspace_msgs::MoveWithToleranceGoalConstPtr goal_tolerant_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  dynamic_reconfigure::Server<Planner3DConfig> parameter_server_;
  diagnostic_updater::Updater diag_updater_;
  JumpDetector jump_;
  ros::Timer no_map_update_timer_;

  Planner3dCore planner_;

  std::string robot_frame_;
  bool use_path_with_velocity_;
  float freq_;
  ros::Duration costmap_watchdog_;
  bool trigger_plan_by_costmap_update_;

  // --- Outputs of the planning logic --------------------------------------
  void publishPath(const nav_msgs::Path & path)
  {
    if (use_path_with_velocity_) {
      pub_path_velocity_.publish(
        trajectory_tracker_msgs::toPathWithVelocity(
          path, std::numeric_limits<double>::quiet_NaN()));
    } else {
      pub_path_.publish(path);
    }
  }
  void publishPathPoses(const geometry_msgs::PoseArray & poses) { pub_path_poses_.publish(poses); }
  void publishPreservedPathPoses(const nav_msgs::Path & path)
  {
    pub_preserved_path_poses_.publish(path);
  }
  void publishStartAndEnd(
    const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & end)
  {
    pub_end_.publish(end);
    pub_start_.publish(start);
  }
  void publishDebugMaps()
  {
    if (pub_distance_map_.getNumSubscribers() > 0) {
      pub_distance_map_.publish(planner_.generateDistanceMapMsg());
    }
    if (pub_hysteresis_map_.getNumSubscribers() > 0) {
      pub_hysteresis_map_.publish(planner_.generateHysteresisMapMsg());
    }
  }
  void publishRememberedMap()
  {
    if (pub_remembered_map_.getNumSubscribers() > 0) {
      pub_remembered_map_.publish(planner_.generateRememberedMapMsg());
    }
  }
  void publishStatus()
  {
    pub_status_.publish(planner_.status());
    diag_updater_.update();
  }
  void onGoalReachedInContinuousMode()
  {
    act_tolerant_->setSucceeded(
      planner_cspace_msgs::MoveWithToleranceResult(), "Goal reached (Continuous movement mode).");
    goal_tolerant_ = nullptr;
  }

  // --- Action server helpers ----------------------------------------------
  // Pushes the tolerances of the active tolerant move goal to the logic.
  void updateGoalTolerance()
  {
    if (act_tolerant_->isActive() && goal_tolerant_) {
      Planner3dCore::GoalTolerance tolerance;
      tolerance.lin = goal_tolerant_->goal_tolerance_lin;
      tolerance.ang = goal_tolerant_->goal_tolerance_ang;
      tolerance.ang_finish = goal_tolerant_->goal_tolerance_ang_finish;
      tolerance.continuous_movement_mode = goal_tolerant_->continuous_movement_mode;
      planner_.setGoalTolerance(tolerance);
    } else {
      planner_.clearGoalTolerance();
    }
  }
  void publishActionFeedback()
  {
    if (act_->isActive()) {
      move_base_msgs::MoveBaseFeedback feedback;
      feedback.base_position = planner_.start();
      act_->publishFeedback(feedback);
    }
    if (act_tolerant_->isActive()) {
      planner_cspace_msgs::MoveWithToleranceFeedback feedback;
      feedback.base_position = planner_.start();
      act_tolerant_->publishFeedback(feedback);
    }
  }

  // --- Subscriber/service/action callbacks --------------------------------
  bool cbForget(std_srvs::EmptyRequest & /* req */, std_srvs::EmptyResponse & /* res */)
  {
    planner_.forgetRememberedCostmap();
    return true;
  }
  void cbTemporaryEscape(const std_msgs::Empty::ConstPtr &)
  {
    updateGoalTolerance();
    planner_.triggerTemporaryEscape();
  }
  bool cbMakePlan(nav_msgs::GetPlan::Request & req, nav_msgs::GetPlan::Response & res)
  {
    return planner_.makePlanOnDemand(req.start, req.goal, req.tolerance, res.plan);
  }
  void cbGoal(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    if (act_->isActive() || act_tolerant_->isActive()) {
      ROS_ERROR("Setting new goal is ignored since planner_3d is proceeding the action.");
      return;
    }
    setGoal(*msg);
  }
  void cbPreempt()
  {
    ROS_WARN("Preempting the current goal.");
    if (act_->isActive()) act_->setPreempted(move_base_msgs::MoveBaseResult(), "Preempted.");

    if (act_tolerant_->isActive())
      act_tolerant_->setPreempted(planner_cspace_msgs::MoveWithToleranceResult(), "Preempted.");

    updateGoalTolerance();
    planner_.clearGoal();
  }
  bool setGoal(const geometry_msgs::PoseStamped & msg)
  {
    updateGoalTolerance();
    switch (planner_.setGoal(msg)) {
      case Planner3dCore::SetGoalResult::REJECTED:
        return false;
      case Planner3dCore::SetGoalResult::CLEARED:
        if (act_->isActive()) act_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal cleared.");
        if (act_tolerant_->isActive())
          act_tolerant_->setSucceeded(
            planner_cspace_msgs::MoveWithToleranceResult(), "Goal cleared.");
        updateGoalTolerance();
        break;
      default:
        break;
    }
    return true;
  }
  void cbAction()
  {
    if (act_tolerant_->isActive()) {
      ROS_ERROR(
        "Setting new goal is ignored since planner_3d is proceeding by tolerant_move action.");
      return;
    }

    move_base_msgs::MoveBaseGoalConstPtr goal = act_->acceptNewGoal();
    if (!setGoal(goal->target_pose))
      act_->setAborted(move_base_msgs::MoveBaseResult(), "Given goal is invalid.");
  }
  void cbTolerantAction()
  {
    if (act_->isActive()) {
      ROS_ERROR("Setting new goal is ignored since planner_3d is proceeding by move_base action.");
      return;
    }

    goal_tolerant_ = act_tolerant_->acceptNewGoal();
    if (!setGoal(goal_tolerant_->target_pose))
      act_tolerant_->setAborted(
        planner_cspace_msgs::MoveWithToleranceResult(), "Given goal is invalid.");
  }
  void cbNoMapUpdateTimer(const ros::TimerEvent & e)
  {
    planPath(e.current_real);
    no_map_update_timer_ =
      nh_.createTimer(costmap_watchdog_, &Planner3dNode::cbNoMapUpdateTimer, this, true);
  }
  void cbMapUpdate(const costmap_cspace_msgs::CSpace3DUpdate::ConstPtr & msg)
  {
    applyMapUpdate(sq_ros1_compat::to_std(msg));
  }
  void applyMapUpdate(const std::shared_ptr<const costmap_cspace_msgs::CSpace3DUpdate> & msg)
  {
    if (!planner_.hasMap()) return;
    ROS_DEBUG("Map updated");
    updateGoalTolerance();
    if (trigger_plan_by_costmap_update_) {
      no_map_update_timer_.stop();
      updateStart();
      planner_.applyCostmapUpdate(msg);
      planPath(planner_.lastCostmapStamp());
      if (costmap_watchdog_ > ros::Duration(0)) {
        no_map_update_timer_ =
          nh_.createTimer(costmap_watchdog_, &Planner3dNode::cbNoMapUpdateTimer, this, true);
      }
    } else {
      planner_.applyCostmapUpdate(msg);
    }
  }
  void cbMap(const costmap_cspace_msgs::CSpace3D::ConstPtr & msg)
  {
    updateGoalTolerance();
    const std::shared_ptr<const costmap_cspace_msgs::CSpace3DUpdate> map_update_retained =
      planner_.setMap(sq_ros1_compat::to_std(msg));
    jump_.setMapFrame(planner_.mapHeader().frame_id);
    if (map_update_retained) {
      ROS_INFO("Applying retained map update");
      applyMapUpdate(map_update_retained);
    }
    planner_.clearRetainedMapUpdate();
  }
  void cbParameter(const Planner3DConfig & config, const uint32_t /* level */)
  {
    Planner3dCore::Parameters p;
    p.freq = config.freq;
    p.freq_min = config.freq_min;
    p.search_timeout_abort = config.search_timeout_abort;
    p.search_range = config.search_range;
    p.antialias_start = config.antialias_start;
    p.costmap_watchdog = config.costmap_watchdog;
    p.max_vel = config.max_vel;
    p.max_ang_vel = config.max_ang_vel;
    p.min_curve_radius = config.min_curve_radius;
    p.weight_decel = config.weight_decel;
    p.weight_backward = config.weight_backward;
    p.weight_ang_vel = config.weight_ang_vel;
    p.weight_costmap = config.weight_costmap;
    p.weight_costmap_turn = config.weight_costmap_turn;
    p.weight_costmap_turn_heuristics = config.weight_costmap_turn_heuristics;
    p.weight_remembered = config.weight_remembered;
    p.cost_in_place_turn = config.cost_in_place_turn;
    p.turn_penalty_cost_threshold = config.turn_penalty_cost_threshold;
    p.hysteresis_max_dist = config.hysteresis_max_dist;
    p.hysteresis_expand = config.hysteresis_expand;
    p.weight_hysteresis = config.weight_hysteresis;
    p.goal_tolerance_lin = config.goal_tolerance_lin;
    p.goal_tolerance_ang = config.goal_tolerance_ang;
    p.goal_tolerance_ang_finish = config.goal_tolerance_ang_finish;
    p.temporary_escape_tolerance_lin = config.temporary_escape_tolerance_lin;
    p.temporary_escape_tolerance_ang = config.temporary_escape_tolerance_ang;
    p.overwrite_cost = config.overwrite_cost;
    p.relocation_acceptable_cost = config.relocation_acceptable_cost;
    p.hist_ignore_range = config.hist_ignore_range;
    p.hist_ignore_range_max = config.hist_ignore_range_max;
    p.remember_updates = config.remember_updates;
    p.remember_hit_prob = config.remember_hit_prob;
    p.remember_miss_prob = config.remember_miss_prob;
    p.local_range = config.local_range;
    p.longcut_range = config.longcut_range;
    p.esc_range = config.esc_range;
    p.esc_range_min_ratio = config.esc_range_min_ratio;
    p.tolerance_range = config.tolerance_range;
    p.tolerance_angle = config.tolerance_angle;
    p.find_best = config.find_best;
    p.force_goal_orientation = config.force_goal_orientation;
    p.temporary_escape = config.temporary_escape;
    p.fast_map_update = config.fast_map_update;
    p.max_retry_num = config.max_retry_num;
    p.sw_wait = config.sw_wait;
    p.keep_a_part_of_previous_path = config.keep_a_part_of_previous_path;
    p.dist_stop_to_previous_path = config.dist_stop_to_previous_path;
    planner_.setParameters(p);

    freq_ = config.freq;
    costmap_watchdog_ = ros::Duration(config.costmap_watchdog);
    trigger_plan_by_costmap_update_ = config.trigger_plan_by_costmap_update;
    no_map_update_timer_.stop();
  }
  void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const planner_cspace_msgs::PlannerStatus & status = planner_.status();
    switch (status.error) {
      case planner_cspace_msgs::PlannerStatus::GOING_WELL:
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Going well.");
        break;
      case planner_cspace_msgs::PlannerStatus::IN_ROCK:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "The robot is in rock.");
        break;
      case planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Path not found.");
        break;
      case planner_cspace_msgs::PlannerStatus::DATA_MISSING:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Required data is missing.");
        break;
      case planner_cspace_msgs::PlannerStatus::INTERNAL_ERROR:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Planner internal error.");
        break;
      default:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown error.");
        break;
    }
    stat.addf("status", "%u", status.status);
    stat.addf("error", "%u", status.error);
  }

  // Looks up the robot pose and pushes it to the planning logic.
  void updateStart()
  {
    geometry_msgs::PoseStamped start;
    start.header.frame_id = robot_frame_;
    start.header.stamp = ros::Time(0);
    start.pose.orientation.x = 0.0;
    start.pose.orientation.y = 0.0;
    start.pose.orientation.z = 0.0;
    start.pose.orientation.w = 1.0;
    start.pose.position.x = 0;
    start.pose.position.y = 0;
    start.pose.position.z = 0;
    try {
      geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform(
        planner_.mapHeader().frame_id, robot_frame_, ros::Time(), ros::Duration(0.1));
      tf2::doTransform(start, start, trans);
    } catch (tf2::TransformException & e) {
      planner_.clearStart();
      return;
    }
    planner_.setStart(start);
  }

public:
  Planner3dNode()
  : nh_(),
    pnh_("~"),
    tfl_(tfbuf_),
    parameter_server_(pnh_),
    jump_(tfbuf_, sq_ros1_compat::get_logger("planner_3d")),
    planner_(sq_ros1_compat::get_logger("planner_3d")),
    freq_(4.0f),
    costmap_watchdog_(0),
    trigger_plan_by_costmap_update_(false)
  {
    neonavigation_common::compat::checkCompatMode();
    sub_map_ = neonavigation_common::compat::subscribe(
      nh_, "costmap", pnh_, "costmap", 1, &Planner3dNode::cbMap, this);
    sub_map_update_ = neonavigation_common::compat::subscribe(
      nh_, "costmap_update", pnh_, "costmap_update", 1, &Planner3dNode::cbMapUpdate, this);
    sub_goal_ = neonavigation_common::compat::subscribe(
      nh_, "move_base_simple/goal", pnh_, "goal", 1, &Planner3dNode::cbGoal, this);
    sub_temporary_escape_trigger_ =
      pnh_.subscribe("temporary_escape", 1, &Planner3dNode::cbTemporaryEscape, this);
    pub_start_ = pnh_.advertise<geometry_msgs::PoseStamped>("path_start", 1, true);
    pub_end_ = pnh_.advertise<geometry_msgs::PoseStamped>("path_end", 1, true);
    pub_goal_ = pnh_.advertise<geometry_msgs::PoseStamped>("current_goal", 1, true);
    pub_status_ = pnh_.advertise<planner_cspace_msgs::PlannerStatus>("status", 1, true);
    pub_metrics_ = pnh_.advertise<neonavigation_metrics_msgs::Metrics>("metrics", 1, false);
    srs_forget_ = neonavigation_common::compat::advertiseService(
      nh_, "forget_planning_cost", pnh_, "forget", &Planner3dNode::cbForget, this);
    srs_make_plan_ = pnh_.advertiseService("make_plan", &Planner3dNode::cbMakePlan, this);

    // Debug outputs
    pub_distance_map_ = pnh_.advertise<sensor_msgs::PointCloud>("distance_map", 1, true);
    pub_hysteresis_map_ = pnh_.advertise<nav_msgs::OccupancyGrid>("hysteresis_map", 1, true);
    pub_remembered_map_ = pnh_.advertise<nav_msgs::OccupancyGrid>("remembered_map", 1, true);

    act_.reset(new Planner3DActionServer(ros::NodeHandle(), "move_base", false));
    act_->registerGoalCallback(boost::bind(&Planner3dNode::cbAction, this));
    act_->registerPreemptCallback(boost::bind(&Planner3dNode::cbPreempt, this));

    act_tolerant_.reset(
      new Planner3DTolerantActionServer(ros::NodeHandle(), "tolerant_move", false));
    act_tolerant_->registerGoalCallback(boost::bind(&Planner3dNode::cbTolerantAction, this));
    act_tolerant_->registerPreemptCallback(boost::bind(&Planner3dNode::cbPreempt, this));
    goal_tolerant_ = nullptr;

    pnh_.param("use_path_with_velocity", use_path_with_velocity_, false);
    if (use_path_with_velocity_) {
      pub_path_velocity_ =
        nh_.advertise<trajectory_tracker_msgs::PathWithVelocity>("path_velocity", 1, true);
    } else {
      pub_path_ =
        neonavigation_common::compat::advertise<nav_msgs::Path>(nh_, "path", pnh_, "path", 1, true);
    }
    pub_path_poses_ = pnh_.advertise<geometry_msgs::PoseArray>("path_poses", 1, true);
    pub_preserved_path_poses_ = pnh_.advertise<nav_msgs::Path>("preserved_path_poses", 1, true);

    // Parameters which are also exposed by dynamic_reconfigure are not read
    // here; cbParameter() is called with the initial values (loaded from the
    // parameter server by dynamic_reconfigure::Server) at the end of this
    // constructor.
    Planner3dCore::StaticParameters sp;
    pnh_.param("unknown_cost", sp.unknown_cost, 100);
    pnh_.param("path_interpolation_resolution", sp.path_interpolation_resolution, 0.5);
    pnh_.param("grid_enumeration_resolution", sp.grid_enumeration_resolution, 0.1);
    pnh_.param("robot_frame", sp.robot_frame, std::string("base_link"));
    pnh_.param("enable_crowd_mode", sp.enable_crowd_mode, false);
    pnh_.param("retain_last_error_status", sp.retain_last_error_status, true);
    pnh_.param("num_threads", sp.num_threads, 1);
    pnh_.param("num_search_task", sp.num_search_task, sp.num_threads * 16);
    pnh_.param("num_cost_estim_task", sp.num_cost_estim_task, sp.num_threads * 16);
    pnh_.param("queue_size_limit", sp.queue_size_limit, 0);
    robot_frame_ = sp.robot_frame;

    double pos_jump, yaw_jump;
    std::string jump_detect_frame;
    pnh_.param("pos_jump", pos_jump, 1.0);
    pnh_.param("yaw_jump", yaw_jump, 1.5);
    pnh_.param("jump_detect_frame", jump_detect_frame, std::string("base_link"));
    jump_.setBaseFrame(jump_detect_frame);
    jump_.setThresholds(pos_jump, yaw_jump);

    bool fast_map_update;
    pnh_.param("fast_map_update", fast_map_update, false);
    if (fast_map_update) {
      ROS_WARN("planner_3d: Experimental fast_map_update is enabled. ");
    }
    if (pnh_.hasParam("debug_mode")) {
      ROS_ERROR(
        "planner_3d: ~/debug_mode parameter and ~/debug topic are deprecated. "
        "Use ~/distance_map, ~/hysteresis_map, and ~/remembered_map topics instead.");
    }

    bool print_planning_duration;
    pnh_.param("print_planning_duration", print_planning_duration, false);
    if (print_planning_duration) {
      if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
      }
    }

    Planner3dCore::Callbacks cb;
    cb.publish_path = std::bind(&Planner3dNode::publishPath, this, std::placeholders::_1);
    cb.publish_path_poses =
      std::bind(&Planner3dNode::publishPathPoses, this, std::placeholders::_1);
    cb.publish_preserved_path_poses =
      std::bind(&Planner3dNode::publishPreservedPathPoses, this, std::placeholders::_1);
    cb.publish_start_and_end = std::bind(
      &Planner3dNode::publishStartAndEnd, this, std::placeholders::_1, std::placeholders::_2);
    cb.publish_debug_maps = std::bind(&Planner3dNode::publishDebugMaps, this);
    cb.publish_remembered_map = std::bind(&Planner3dNode::publishRememberedMap, this);
    cb.publish_status = std::bind(&Planner3dNode::publishStatus, this);
    cb.goal_reached_in_continuous_mode =
      std::bind(&Planner3dNode::onGoalReachedInContinuousMode, this);
    planner_.setCallbacks(cb);
    planner_.initialize(sp);

    diag_updater_.setHardwareID("none");
    diag_updater_.add("Path Planner Status", this, &Planner3dNode::diagnoseStatus);

    act_->start();
    act_tolerant_->start();

    // cbParameter() with the inital parameters will be called within setCallback().
    parameter_server_.setCallback(boost::bind(&Planner3dNode::cbParameter, this, _1, _2));
  }

  void planPath(const ros::Time & now)
  {
    updateGoalTolerance();
    const bool has_costmap = planner_.preparePlanCycle(now);

    if (planner_.isReadyToPlan() && has_costmap) {
      publishActionFeedback();
      switch (planner_.runPlanCycle(now)) {
        case Planner3dCore::PlanCycleResult::GOAL_REACHED:
          if (act_->isActive())
            act_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          if (act_tolerant_->isActive())
            act_tolerant_->setSucceeded(
              planner_cspace_msgs::MoveWithToleranceResult(), "Goal reached.");
          updateGoalTolerance();
          break;
        case Planner3dCore::PlanCycleResult::ABORT_MAX_RETRY:
          if (act_->isActive())
            act_->setAborted(move_base_msgs::MoveBaseResult(), "Goal is in Rock");
          if (act_tolerant_->isActive())
            act_tolerant_->setAborted(
              planner_cspace_msgs::MoveWithToleranceResult(), "Goal is in Rock");
          updateGoalTolerance();
          return;
        default:
          break;
      }
    } else if (!planner_.hasGoal()) {
      planner_.handleNoGoal();
    }
    pub_goal_.publish(planner_.currentGoalStamped());
    planner_.setStatusStamp(now);
    pub_status_.publish(planner_.status());
    diag_updater_.force_update();
    pub_metrics_.publish(planner_.collectMetrics(now));
  }

  void waitUntil(const ros::Time & next_replan_time)
  {
    while (ros::ok()) {
      const ros::Time prev_map_update_stamp = planner_.lastCostmapStamp();
      ros::spinOnce();
      const bool costmap_updated = planner_.lastCostmapStamp() != prev_map_update_stamp;

      if (planner_.hasMap()) {
        updateStart();

        if (jump_.detectJump()) {
          planner_.clearRememberedCostmap();
          // Robot pose jumped.
          return;
        }

        if (costmap_updated && planner_.isPreviousPathBlocked()) {
          // Obstacle on the path.
          return;
        }

        if (planner_.checkSwitchbackArrival()) {
          // robot has arrived at the switchback point
          return;
        }
      }
      if (ros::Time::now() > next_replan_time) {
        return;
      }
      ros::Duration(0.01).sleep();
    }
  }

  void spin()
  {
    ROS_DEBUG("Initialized");

    ros::Time next_replan_time = ros::Time::now();
    ros::Rate r(100);
    while (ros::ok()) {
      if (trigger_plan_by_costmap_update_) {
        if (jump_.detectJump()) {
          planner_.clearRememberedCostmap();
        }
        ros::spinOnce();
        r.sleep();
      } else {
        waitUntil(next_replan_time);
        const ros::Time now = ros::Time::now();
        planPath(now);
        if (planner_.isPathSwitchback()) {
          next_replan_time = now + ros::Duration(planner_.swWait());
          ROS_INFO(
            "Planned path has switchback. Planner will stop until: %f at the latest.",
            next_replan_time.toSec());
        } else {
          next_replan_time = now + ros::Duration(1.0 / freq_);
        }
      }
    }
  }
};
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "planner_3d");

  planner_cspace::planner_3d::Planner3dNode node;
  node.spin();

  return 0;
}
