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

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "neonavigation_metrics_msgs/msg/metrics.hpp"
#include "planner_cspace/jump_detector.h"
#include "planner_cspace/planner_3d/planner_3d_core.h"
#include "planner_cspace_msgs/action/move_with_tolerance.hpp"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_tracker_msgs/converter.h"
#include "trajectory_tracker_msgs/msg/path_with_velocity.hpp"

namespace planner_cspace
{
namespace planner_3d
{
namespace
{
// nav2_msgs/NavigateToPose gained the error_code/error_msg result fields after
// humble, whose result is just an empty message. Fill them where they exist so
// the status text that ROS 1 passed to setSucceeded/setAborted survives; on
// humble there is nowhere to put it and it is dropped.
// Overload ranking: the first overload is picked whenever the fields exist.
struct FallbackTag
{
};
struct PreferredTag : FallbackTag
{
};

template <typename ResultT, typename = decltype(std::declval<ResultT &>().error_code)>
void setResultStatus(ResultT & result, const std::string & text, PreferredTag)
{
  result.error_code = ResultT::NONE;
  result.error_msg = text;
}

template <typename ResultT>
void setResultStatus(ResultT &, const std::string &, FallbackTag)
{
}

rcl_interfaces::msg::ParameterDescriptor floatRange(const double from, const double to)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  desc.floating_point_range.push_back(range);
  return desc;
}

rcl_interfaces::msg::ParameterDescriptor intRange(const int64_t from, const int64_t to)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  desc.integer_range.push_back(range);
  return desc;
}

float pathLength(const nav_msgs::msg::Path & path)
{
  float length = 0.0f;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    length += static_cast<float>(std::hypot(b.x - a.x, b.y - a.y));
  }
  return length;
}
}  // namespace

// Planner3dNode is the ROS 2 interface of the planner_3d node. It owns the
// publishers, subscribers, services, action servers, parameters, TF and
// diagnostics, and delegates all path planning to Planner3dCore.
//
// Differences to the ROS 1 node
// -----------------------------
// * Action types. move_base_msgs does not exist on ROS 2, so the `move_base`
//   action uses nav2_msgs/action/NavigateToPose instead:
//     goal.target_pose      -> goal.pose (goal.behavior_tree is ignored)
//     feedback.base_position-> feedback.current_pose
//     result (empty)        -> result.error_code (always NONE) and
//                              result.error_msg, which carries the status text
//                              that ROS 1 passed to setSucceeded/setAborted.
//                              humble's result has neither field, so there the
//                              text is dropped (see setResultStatus).
//   feedback.navigation_time is filled with the time since the goal was
//   accepted and feedback.distance_remaining with the length of the last
//   published path; estimated_time_remaining and number_of_recoveries have no
//   counterpart in this planner and stay zero. The `tolerant_move` action keeps
//   using planner_cspace_msgs/action/MoveWithTolerance.
//
// * Preemption. actionlib preempted the running goal automatically when a new
//   one arrived, and setPreempted() reported it. rclcpp_action has neither an
//   automatic preemption nor a PREEMPTED terminal state, and canceled() may
//   only be called after a cancel request was accepted. Therefore:
//     - a goal that is superseded by a new goal on the same action server is
//       aborted with error_msg/"Preempted.";
//     - an explicitly cancelled goal is terminated with canceled() (the cancel
//       request is recorded in the cancel callback and the goal handle is
//       finished from the spin timer, because a goal handle may not be
//       terminated from within handle_cancel);
//     - a goal that arrives while the *other* action server is busy is
//       rejected in handle_goal (ROS 1 logged an error and silently left the
//       goal pending forever).
//
// * dynamic_reconfigure is replaced by plain ROS 2 parameters with ranges from
//   cfg/Planner3D.cfg plus a post-set parameter callback.
//
// * The 100 Hz `while (ros::ok())` planning loop of the ROS 1 node runs from a
//   100 Hz timer so that the node can be loaded as a component.
//
// * `move_base_simple/goal` is named `goal_pose` on ROS 2, which is what RViz 2
//   publishes for "2D Goal Pose". The neonavigation_common compatibility
//   aliases (old `~/`-relative topic names) are ROS 1 only.
class Planner3dNode : public rclcpp::Node
{
public:
  explicit Planner3dNode(const rclcpp::NodeOptions & options);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using MoveWithTolerance = planner_cspace_msgs::action::MoveWithTolerance;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using GoalHandleTolerant = rclcpp_action::ServerGoalHandle<MoveWithTolerance>;

  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3D>::SharedPtr sub_map_;
  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>::SharedPtr sub_map_update_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_temporary_escape_trigger_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<trajectory_tracker_msgs::msg::PathWithVelocity>::SharedPtr pub_path_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_path_poses_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_preserved_path_poses_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_hysteresis_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_distance_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_remembered_map_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_start_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_end_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<neonavigation_metrics_msgs::msg::Metrics>::SharedPtr pub_metrics_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srs_forget_;
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srs_make_plan_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr act_;
  rclcpp_action::Server<MoveWithTolerance>::SharedPtr act_tolerant_;
  std::shared_ptr<GoalHandleNavigate> gh_move_base_;
  std::shared_ptr<GoalHandleTolerant> gh_tolerant_;
  std::shared_ptr<const MoveWithTolerance::Goal> goal_tolerant_;
  bool cancel_requested_move_base_;
  bool cancel_requested_tolerant_;
  rclcpp::Time goal_accepted_stamp_;
  float remaining_path_length_;

  tf2_ros::Buffer tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  JumpDetector jump_;
  Planner3dCore planner_;
  diagnostic_updater::Updater diag_updater_;

  rclcpp::TimerBase::SharedPtr spin_timer_;
  rclcpp::TimerBase::SharedPtr no_map_update_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string robot_frame_;
  bool use_path_with_velocity_;
  float freq_;
  rclcpp::Duration costmap_watchdog_;
  bool trigger_plan_by_costmap_update_;
  rclcpp::Time next_replan_time_;
  rclcpp::Time prev_map_update_stamp_;

  // --- Parameters ---------------------------------------------------------
  bool hasOverride(const std::string & name) const;
  void declareDynamicParameters();
  void updateParameters(const std::vector<rclcpp::Parameter> & changed = {});

  // --- Outputs of the planning logic --------------------------------------
  void publishPath(const nav_msgs::msg::Path & path);
  void publishPathPoses(const geometry_msgs::msg::PoseArray & poses);
  void publishPreservedPathPoses(const nav_msgs::msg::Path & path);
  void publishStartAndEnd(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & end);
  void publishDebugMaps();
  void publishRememberedMap();
  void publishStatus();
  void onGoalReachedInContinuousMode();

  // --- Action server helpers ----------------------------------------------
  bool moveBaseActive() const;
  bool tolerantActive() const;
  void finishMoveBase(const bool succeeded, const std::string & text);
  void finishTolerant(const bool succeeded, const std::string & text);
  void updateGoalTolerance();
  void publishActionFeedback();
  void preemptCurrentGoals();
  void processCancelRequests();

  rclcpp_action::GoalResponse cbMoveBaseGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse cbMoveBaseCancel(const std::shared_ptr<GoalHandleNavigate> gh);
  void cbMoveBaseAccepted(const std::shared_ptr<GoalHandleNavigate> gh);
  rclcpp_action::GoalResponse cbTolerantGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveWithTolerance::Goal> goal);
  rclcpp_action::CancelResponse cbTolerantCancel(const std::shared_ptr<GoalHandleTolerant> gh);
  void cbTolerantAccepted(const std::shared_ptr<GoalHandleTolerant> gh);

  // --- Subscriber/service callbacks ---------------------------------------
  void cbForget(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  void cbTemporaryEscape(const std_msgs::msg::Empty::ConstSharedPtr & msg);
  void cbMakePlan(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> res);
  void cbGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);
  bool setGoal(const geometry_msgs::msg::PoseStamped & msg);
  void cbMapUpdate(const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr & msg);
  void applyMapUpdate(const std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> & msg);
  void cbMap(const costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr & msg);
  void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // --- Planning cycle -----------------------------------------------------
  void armNoMapUpdateTimer();
  void disarmNoMapUpdateTimer();
  void cbNoMapUpdateTimer();
  void cbSpinTimer();
  void updateStart();
  void planPath(const rclcpp::Time & now);
};

Planner3dNode::Planner3dNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("planner_3d", options),
  cancel_requested_move_base_(false),
  cancel_requested_tolerant_(false),
  goal_accepted_stamp_(0, 0, RCL_ROS_TIME),
  remaining_path_length_(0.0f),
  tfbuf_(this->get_clock()),
  jump_(tfbuf_, this->get_logger()),
  planner_(this->get_logger()),
  diag_updater_(this),
  freq_(4.0f),
  costmap_watchdog_(0, 0),
  trigger_plan_by_costmap_update_(false),
  next_replan_time_(0, 0, RCL_ROS_TIME),
  prev_map_update_stamp_(0, 0, RCL_ROS_TIME)
{
  tfl_ = std::make_shared<tf2_ros::TransformListener>(tfbuf_);

  // Time is read through the node's clock so the logic follows /clock
  // when use_sim_time is set.
  planner_.setClock(this->get_clock());

  declareDynamicParameters();

  Planner3dCore::StaticParameters sp;
  sp.unknown_cost = static_cast<int>(this->declare_parameter("unknown_cost", 100));
  sp.path_interpolation_resolution = this->declare_parameter("path_interpolation_resolution", 0.5);
  sp.grid_enumeration_resolution = this->declare_parameter("grid_enumeration_resolution", 0.1);
  sp.robot_frame = this->declare_parameter("robot_frame", std::string("base_link"));
  sp.enable_crowd_mode = this->declare_parameter("enable_crowd_mode", false);
  sp.retain_last_error_status = this->declare_parameter("retain_last_error_status", true);
  sp.num_threads = static_cast<int>(this->declare_parameter("num_threads", 1));
  sp.num_search_task =
    static_cast<int>(this->declare_parameter("num_search_task", sp.num_threads * 16));
  sp.num_cost_estim_task =
    static_cast<int>(this->declare_parameter("num_cost_estim_task", sp.num_threads * 16));
  sp.queue_size_limit = static_cast<int>(this->declare_parameter("queue_size_limit", 0));
  robot_frame_ = sp.robot_frame;

  // Declared before the parameter callback is registered so that the
  // declaration itself does not re-enter updateParameters().
  use_path_with_velocity_ = this->declare_parameter("use_path_with_velocity", false);

  const double pos_jump = this->declare_parameter("pos_jump", 1.0);
  const double yaw_jump = this->declare_parameter("yaw_jump", 1.5);
  const std::string jump_detect_frame =
    this->declare_parameter("jump_detect_frame", std::string("base_link"));
  jump_.setBaseFrame(jump_detect_frame);
  jump_.setThresholds(pos_jump, yaw_jump);

  if (this->get_parameter("fast_map_update").as_bool()) {
    RCLCPP_WARN(this->get_logger(), "planner_3d: Experimental fast_map_update is enabled. ");
  }
  // ROS 1 checked ros::NodeHandle::hasParam(); on ROS 2 an undeclared parameter
  // is only visible through the node options.
  if (hasOverride("debug_mode")) {
    RCLCPP_ERROR(
      this->get_logger(),
      "planner_3d: ~/debug_mode parameter and ~/debug topic are deprecated. "
      "Use ~/distance_map, ~/hysteresis_map, and ~/remembered_map topics instead.");
  }

  if (this->declare_parameter("print_planning_duration", false)) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }

  Planner3dCore::Callbacks cb;
  cb.publish_path = std::bind(&Planner3dNode::publishPath, this, std::placeholders::_1);
  cb.publish_path_poses = std::bind(&Planner3dNode::publishPathPoses, this, std::placeholders::_1);
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

  // Mirrors the initial dynamic_reconfigure callback of the ROS 1 node.
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

  diag_updater_.setHardwareID("none");
  diag_updater_.add("Path Planner Status", this, &Planner3dNode::diagnoseStatus);

  // ROS 1 latched every publisher but ~/metrics; transient_local is the ROS 2
  // equivalent, and subscribers have to request it as well.
  const auto latched = rclcpp::QoS(1).transient_local();
  pub_start_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/path_start", latched);
  pub_end_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/path_end", latched);
  pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_goal", latched);
  pub_status_ =
    this->create_publisher<planner_cspace_msgs::msg::PlannerStatus>("~/status", latched);
  pub_metrics_ =
    this->create_publisher<neonavigation_metrics_msgs::msg::Metrics>("~/metrics", rclcpp::QoS(1));

  // Debug outputs
  pub_distance_map_ =
    this->create_publisher<sensor_msgs::msg::PointCloud>("~/distance_map", latched);
  pub_hysteresis_map_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/hysteresis_map", latched);
  pub_remembered_map_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/remembered_map", latched);

  if (use_path_with_velocity_) {
    pub_path_velocity_ = this->create_publisher<trajectory_tracker_msgs::msg::PathWithVelocity>(
      "path_velocity", latched);
  } else {
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("path", latched);
  }
  pub_path_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("~/path_poses", latched);
  pub_preserved_path_poses_ =
    this->create_publisher<nav_msgs::msg::Path>("~/preserved_path_poses", latched);

  srs_forget_ = this->create_service<std_srvs::srv::Empty>(
    "forget_planning_cost",
    std::bind(&Planner3dNode::cbForget, this, std::placeholders::_1, std::placeholders::_2));
  srs_make_plan_ = this->create_service<nav_msgs::srv::GetPlan>(
    "~/make_plan",
    std::bind(&Planner3dNode::cbMakePlan, this, std::placeholders::_1, std::placeholders::_2));

  act_ = rclcpp_action::create_server<NavigateToPose>(
    this, "move_base",
    std::bind(&Planner3dNode::cbMoveBaseGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Planner3dNode::cbMoveBaseCancel, this, std::placeholders::_1),
    std::bind(&Planner3dNode::cbMoveBaseAccepted, this, std::placeholders::_1));
  act_tolerant_ = rclcpp_action::create_server<MoveWithTolerance>(
    this, "tolerant_move",
    std::bind(&Planner3dNode::cbTolerantGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Planner3dNode::cbTolerantCancel, this, std::placeholders::_1),
    std::bind(&Planner3dNode::cbTolerantAccepted, this, std::placeholders::_1));

  sub_map_ = this->create_subscription<costmap_cspace_msgs::msg::CSpace3D>(
    "costmap", latched, std::bind(&Planner3dNode::cbMap, this, std::placeholders::_1));
  sub_map_update_ = this->create_subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>(
    "costmap_update", latched, std::bind(&Planner3dNode::cbMapUpdate, this, std::placeholders::_1));
  sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 1, std::bind(&Planner3dNode::cbGoal, this, std::placeholders::_1));
  sub_temporary_escape_trigger_ = this->create_subscription<std_msgs::msg::Empty>(
    "~/temporary_escape", 1,
    std::bind(&Planner3dNode::cbTemporaryEscape, this, std::placeholders::_1));

  next_replan_time_ = this->now();
  // The ROS 1 node ran the planning loop at 100 Hz in main(); as a component
  // the same body is driven by a timer.
  spin_timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(0.01),
    std::bind(&Planner3dNode::cbSpinTimer, this));

  RCLCPP_DEBUG(this->get_logger(), "Initialized");
}

// --- Parameters -----------------------------------------------------------
bool Planner3dNode::hasOverride(const std::string & name) const
{
  for (const auto & param_override : this->get_node_options().parameter_overrides()) {
    if (param_override.get_name() == name) {
      return true;
    }
  }
  return false;
}

// Counterpart of cfg/Planner3D.cfg. The defaults and the ranges are the ones
// declared there; dynamic_reconfigure loaded them from the parameter server on
// ROS 1, so the ROS 2 parameter store holds exactly the same values.
void Planner3dNode::declareDynamicParameters()
{
  this->declare_parameter("freq", 4.0, floatRange(0.0, 100.0));
  this->declare_parameter("freq_min", 2.0, floatRange(0.0, 100.0));
  this->declare_parameter("search_timeout_abort", 30.0, floatRange(0.0, 100.0));
  this->declare_parameter("search_range", 0.4, floatRange(0.0, 100.0));
  this->declare_parameter("antialias_start", false);
  this->declare_parameter("costmap_watchdog", 0.0, floatRange(0.0, 100.0));
  this->declare_parameter("max_vel", 0.3, floatRange(0.0, 100.0));
  this->declare_parameter("max_ang_vel", 0.6, floatRange(0.0, 100.0));
  this->declare_parameter("min_curve_radius", 0.1, floatRange(0.0, 100.0));
  this->declare_parameter("weight_decel", 50.0, floatRange(0.0, 1000.0));
  this->declare_parameter("weight_backward", 0.9, floatRange(0.0, 1000.0));
  this->declare_parameter("weight_ang_vel", 1.0, floatRange(0.0, 1000.0));
  this->declare_parameter("weight_costmap", 50.0, floatRange(0.0, 1000.0));
  this->declare_parameter("weight_costmap_turn", 0.0, floatRange(0.0, 1000.0));
  this->declare_parameter("weight_costmap_turn_heuristics", 100.0, floatRange(0.0, 1000.0));
  this->declare_parameter("weight_remembered", 1000.0, floatRange(0.0, 1000.0));
  this->declare_parameter("cost_in_place_turn", 30.0, floatRange(0.0, 1000.0));
  this->declare_parameter("turn_penalty_cost_threshold", 0, intRange(0, 100));
  this->declare_parameter("hysteresis_max_dist", 0.1, floatRange(0.0, 10.0));
  this->declare_parameter("hysteresis_expand", 0.1, floatRange(0.0, 10.0));
  this->declare_parameter("weight_hysteresis", 5.0, floatRange(0.0, 1000.0));
  this->declare_parameter("goal_tolerance_lin", 0.05, floatRange(0.0, 10.0));
  this->declare_parameter("goal_tolerance_ang", 0.1, floatRange(0.0, M_PI));
  this->declare_parameter("goal_tolerance_ang_finish", 0.05, floatRange(0.0, M_PI));
  this->declare_parameter("overwrite_cost", false);
  this->declare_parameter("hist_ignore_range", 0.6, floatRange(0.0, 100.0));
  this->declare_parameter("hist_ignore_range_max", 1.25, floatRange(0.0, 100.0));
  this->declare_parameter("remember_updates", false);
  this->declare_parameter("remember_hit_prob", 0.6, floatRange(0.0, 1.0));
  this->declare_parameter("remember_miss_prob", 0.3, floatRange(0.0, 1.0));
  this->declare_parameter("local_range", 2.5, floatRange(0.0, 100.0));
  this->declare_parameter("longcut_range", 0.0, floatRange(0.0, 100.0));
  this->declare_parameter("esc_range", 0.25, floatRange(0.0, 100.0));
  this->declare_parameter("esc_range_min_ratio", 0.5, floatRange(0.0, 1.0));
  this->declare_parameter("tolerance_range", 0.25, floatRange(0.0, 1.0));
  this->declare_parameter("tolerance_angle", 0.0, floatRange(0.0, M_PI));
  this->declare_parameter("sw_wait", 2.0, floatRange(0.0, 100.0));
  this->declare_parameter("find_best", true);
  this->declare_parameter("force_goal_orientation", true);
  this->declare_parameter("temporary_escape", true);
  this->declare_parameter("temporary_escape_tolerance_lin", 0.1, floatRange(0.0, 10.0));
  this->declare_parameter("temporary_escape_tolerance_ang", 1.57, floatRange(0.0, M_PI));
  this->declare_parameter("fast_map_update", false);
  this->declare_parameter("max_retry_num", -1, intRange(-1, 100));
  this->declare_parameter("keep_a_part_of_previous_path", false);
  this->declare_parameter("dist_stop_to_previous_path", 0.1, floatRange(0.0, 1.0));
  this->declare_parameter("trigger_plan_by_costmap_update", false);
  this->declare_parameter("relocation_acceptable_cost", 50, intRange(0, 99));
}

void Planner3dNode::updateParameters(const std::vector<rclcpp::Parameter> & changed)
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
  Planner3dCore::Parameters p;
  p.freq = param("freq").as_double();
  p.freq_min = param("freq_min").as_double();
  p.search_timeout_abort = param("search_timeout_abort").as_double();
  p.search_range = param("search_range").as_double();
  p.antialias_start = param("antialias_start").as_bool();
  p.costmap_watchdog = param("costmap_watchdog").as_double();
  p.max_vel = param("max_vel").as_double();
  p.max_ang_vel = param("max_ang_vel").as_double();
  p.min_curve_radius = param("min_curve_radius").as_double();
  p.weight_decel = param("weight_decel").as_double();
  p.weight_backward = param("weight_backward").as_double();
  p.weight_ang_vel = param("weight_ang_vel").as_double();
  p.weight_costmap = param("weight_costmap").as_double();
  p.weight_costmap_turn = param("weight_costmap_turn").as_double();
  p.weight_costmap_turn_heuristics = param("weight_costmap_turn_heuristics").as_double();
  p.weight_remembered = param("weight_remembered").as_double();
  p.cost_in_place_turn = param("cost_in_place_turn").as_double();
  p.turn_penalty_cost_threshold = static_cast<int>(param("turn_penalty_cost_threshold").as_int());
  p.hysteresis_max_dist = param("hysteresis_max_dist").as_double();
  p.hysteresis_expand = param("hysteresis_expand").as_double();
  p.weight_hysteresis = param("weight_hysteresis").as_double();
  p.goal_tolerance_lin = param("goal_tolerance_lin").as_double();
  p.goal_tolerance_ang = param("goal_tolerance_ang").as_double();
  p.goal_tolerance_ang_finish = param("goal_tolerance_ang_finish").as_double();
  p.temporary_escape_tolerance_lin = param("temporary_escape_tolerance_lin").as_double();
  p.temporary_escape_tolerance_ang = param("temporary_escape_tolerance_ang").as_double();
  p.overwrite_cost = param("overwrite_cost").as_bool();
  p.relocation_acceptable_cost = static_cast<int>(param("relocation_acceptable_cost").as_int());
  p.hist_ignore_range = param("hist_ignore_range").as_double();
  p.hist_ignore_range_max = param("hist_ignore_range_max").as_double();
  p.remember_updates = param("remember_updates").as_bool();
  p.remember_hit_prob = param("remember_hit_prob").as_double();
  p.remember_miss_prob = param("remember_miss_prob").as_double();
  p.local_range = param("local_range").as_double();
  p.longcut_range = param("longcut_range").as_double();
  p.esc_range = param("esc_range").as_double();
  p.esc_range_min_ratio = param("esc_range_min_ratio").as_double();
  p.tolerance_range = param("tolerance_range").as_double();
  p.tolerance_angle = param("tolerance_angle").as_double();
  p.find_best = param("find_best").as_bool();
  p.force_goal_orientation = param("force_goal_orientation").as_bool();
  p.temporary_escape = param("temporary_escape").as_bool();
  p.fast_map_update = param("fast_map_update").as_bool();
  p.max_retry_num = static_cast<int>(param("max_retry_num").as_int());
  p.sw_wait = param("sw_wait").as_double();
  p.keep_a_part_of_previous_path = param("keep_a_part_of_previous_path").as_bool();
  p.dist_stop_to_previous_path = param("dist_stop_to_previous_path").as_double();
  planner_.setParameters(p);

  freq_ = p.freq;
  costmap_watchdog_ = rclcpp::Duration::from_seconds(p.costmap_watchdog);
  trigger_plan_by_costmap_update_ = param("trigger_plan_by_costmap_update").as_bool();
  // Drop the watchdog timer so that a changed costmap_watchdog takes effect on
  // the next costmap update (ROS 1 stopped its one-shot timer here).
  no_map_update_timer_.reset();
}

// --- Outputs of the planning logic ----------------------------------------
void Planner3dNode::publishPath(const nav_msgs::msg::Path & path)
{
  remaining_path_length_ = pathLength(path);
  if (use_path_with_velocity_) {
    auto out = std::make_unique<trajectory_tracker_msgs::msg::PathWithVelocity>(
      trajectory_tracker_msgs::toPathWithVelocity(path, std::numeric_limits<double>::quiet_NaN()));
    pub_path_velocity_->publish(std::move(out));
  } else {
    auto out = std::make_unique<nav_msgs::msg::Path>(path);
    pub_path_->publish(std::move(out));
  }
}

void Planner3dNode::publishPathPoses(const geometry_msgs::msg::PoseArray & poses)
{
  pub_path_poses_->publish(std::make_unique<geometry_msgs::msg::PoseArray>(poses));
}

void Planner3dNode::publishPreservedPathPoses(const nav_msgs::msg::Path & path)
{
  pub_preserved_path_poses_->publish(std::make_unique<nav_msgs::msg::Path>(path));
}

void Planner3dNode::publishStartAndEnd(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & end)
{
  pub_end_->publish(std::make_unique<geometry_msgs::msg::PoseStamped>(end));
  pub_start_->publish(std::make_unique<geometry_msgs::msg::PoseStamped>(start));
}

void Planner3dNode::publishDebugMaps()
{
  if (pub_distance_map_->get_subscription_count() > 0) {
    pub_distance_map_->publish(
      std::make_unique<sensor_msgs::msg::PointCloud>(planner_.generateDistanceMapMsg()));
  }
  if (pub_hysteresis_map_->get_subscription_count() > 0) {
    pub_hysteresis_map_->publish(
      std::make_unique<nav_msgs::msg::OccupancyGrid>(planner_.generateHysteresisMapMsg()));
  }
}

void Planner3dNode::publishRememberedMap()
{
  if (pub_remembered_map_->get_subscription_count() > 0) {
    pub_remembered_map_->publish(
      std::make_unique<nav_msgs::msg::OccupancyGrid>(planner_.generateRememberedMapMsg()));
  }
}

void Planner3dNode::publishStatus()
{
  pub_status_->publish(
    std::make_unique<planner_cspace_msgs::msg::PlannerStatus>(planner_.status()));
  // ROS 2's Updater::update() is private (it has its own periodic timer), so
  // the ROS 1 rate-limited update is expressed as a forced one here.
  diag_updater_.force_update();
}

void Planner3dNode::onGoalReachedInContinuousMode()
{
  finishTolerant(true, "Goal reached (Continuous movement mode).");
}

// --- Action server helpers ------------------------------------------------
bool Planner3dNode::moveBaseActive() const { return gh_move_base_ && gh_move_base_->is_active(); }

bool Planner3dNode::tolerantActive() const { return gh_tolerant_ && gh_tolerant_->is_active(); }

void Planner3dNode::finishMoveBase(const bool succeeded, const std::string & text)
{
  if (!moveBaseActive()) {
    return;
  }
  auto result = std::make_shared<NavigateToPose::Result>();
  setResultStatus(*result, text, PreferredTag{});
  if (succeeded) {
    gh_move_base_->succeed(result);
  } else {
    gh_move_base_->abort(result);
  }
  gh_move_base_.reset();
}

void Planner3dNode::finishTolerant(const bool succeeded, const std::string & text)
{
  if (!tolerantActive()) {
    return;
  }
  // MoveWithTolerance has an empty result; the ROS 1 status text is only logged.
  RCLCPP_DEBUG(this->get_logger(), "tolerant_move: %s", text.c_str());
  auto result = std::make_shared<MoveWithTolerance::Result>();
  if (succeeded) {
    gh_tolerant_->succeed(result);
  } else {
    gh_tolerant_->abort(result);
  }
  gh_tolerant_.reset();
  goal_tolerant_ = nullptr;
}

// Pushes the tolerances of the active tolerant move goal to the logic.
void Planner3dNode::updateGoalTolerance()
{
  if (tolerantActive() && goal_tolerant_) {
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

void Planner3dNode::publishActionFeedback()
{
  if (moveBaseActive()) {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose = planner_.start();
    feedback->navigation_time = this->now() - goal_accepted_stamp_;
    feedback->estimated_time_remaining = rclcpp::Duration(0, 0);
    feedback->number_of_recoveries = 0;
    feedback->distance_remaining = remaining_path_length_;
    gh_move_base_->publish_feedback(feedback);
  }
  if (tolerantActive()) {
    auto feedback = std::make_shared<MoveWithTolerance::Feedback>();
    feedback->base_position = planner_.start();
    gh_tolerant_->publish_feedback(feedback);
  }
}

// Counterpart of the ROS 1 preempt callback: terminates whatever goal is
// running because it is being superseded.
void Planner3dNode::preemptCurrentGoals()
{
  RCLCPP_WARN(this->get_logger(), "Preempting the current goal.");
  finishMoveBase(false, "Preempted.");
  finishTolerant(false, "Preempted.");
  updateGoalTolerance();
  planner_.clearGoal();
}

// Terminates goals whose cancellation was accepted. This cannot be done from
// within handle_cancel because the goal handle only enters the CANCELING state
// after that callback returns.
void Planner3dNode::processCancelRequests()
{
  if (cancel_requested_move_base_) {
    if (gh_move_base_ && gh_move_base_->is_canceling()) {
      cancel_requested_move_base_ = false;
      RCLCPP_WARN(this->get_logger(), "Preempting the current goal.");
      auto result = std::make_shared<NavigateToPose::Result>();
      setResultStatus(*result, "Preempted.", PreferredTag{});
      gh_move_base_->canceled(result);
      gh_move_base_.reset();
      updateGoalTolerance();
      planner_.clearGoal();
    } else if (!moveBaseActive()) {
      // The goal was already terminated by another path (preempted/aborted).
      cancel_requested_move_base_ = false;
    }
  }
  if (cancel_requested_tolerant_) {
    if (gh_tolerant_ && gh_tolerant_->is_canceling()) {
      cancel_requested_tolerant_ = false;
      RCLCPP_WARN(this->get_logger(), "Preempting the current goal.");
      gh_tolerant_->canceled(std::make_shared<MoveWithTolerance::Result>());
      gh_tolerant_.reset();
      goal_tolerant_ = nullptr;
      updateGoalTolerance();
      planner_.clearGoal();
    } else if (!tolerantActive()) {
      cancel_requested_tolerant_ = false;
    }
  }
}

rclcpp_action::GoalResponse Planner3dNode::cbMoveBaseGoal(
  const rclcpp_action::GoalUUID & /* uuid */,
  std::shared_ptr<const NavigateToPose::Goal> /* goal */)
{
  if (tolerantActive()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Setting new goal is ignored since planner_3d is proceeding by tolerant_move action.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Planner3dNode::cbMoveBaseCancel(
  const std::shared_ptr<GoalHandleNavigate> gh)
{
  if (gh_move_base_ == gh) {
    cancel_requested_move_base_ = true;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Planner3dNode::cbMoveBaseAccepted(const std::shared_ptr<GoalHandleNavigate> gh)
{
  if (moveBaseActive()) {
    preemptCurrentGoals();
  }
  gh_move_base_ = gh;
  goal_accepted_stamp_ = this->now();
  if (!setGoal(gh->get_goal()->pose)) {
    finishMoveBase(false, "Given goal is invalid.");
  }
}

rclcpp_action::GoalResponse Planner3dNode::cbTolerantGoal(
  const rclcpp_action::GoalUUID & /* uuid */,
  std::shared_ptr<const MoveWithTolerance::Goal> /* goal */)
{
  if (moveBaseActive()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Setting new goal is ignored since planner_3d is proceeding by move_base action.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Planner3dNode::cbTolerantCancel(
  const std::shared_ptr<GoalHandleTolerant> gh)
{
  if (gh_tolerant_ == gh) {
    cancel_requested_tolerant_ = true;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Planner3dNode::cbTolerantAccepted(const std::shared_ptr<GoalHandleTolerant> gh)
{
  if (tolerantActive()) {
    preemptCurrentGoals();
  }
  gh_tolerant_ = gh;
  goal_tolerant_ = gh->get_goal();
  goal_accepted_stamp_ = this->now();
  if (!setGoal(goal_tolerant_->target_pose)) {
    finishTolerant(false, "Given goal is invalid.");
  }
}

// --- Subscriber/service callbacks -----------------------------------------
void Planner3dNode::cbForget(
  const std::shared_ptr<std_srvs::srv::Empty::Request> /* req */,
  std::shared_ptr<std_srvs::srv::Empty::Response> /* res */)
{
  planner_.forgetRememberedCostmap();
}

void Planner3dNode::cbTemporaryEscape(const std_msgs::msg::Empty::ConstSharedPtr & /* msg */)
{
  updateGoalTolerance();
  planner_.triggerTemporaryEscape();
}

// ROS 1 returned false from the service callback when no plan was found; ROS 2
// services cannot report a failure, so the response simply holds an empty plan.
void Planner3dNode::cbMakePlan(
  const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
  std::shared_ptr<nav_msgs::srv::GetPlan::Response> res)
{
  planner_.makePlanOnDemand(req->start, req->goal, req->tolerance, res->plan);
}

void Planner3dNode::cbGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  if (moveBaseActive() || tolerantActive()) {
    RCLCPP_ERROR(
      this->get_logger(), "Setting new goal is ignored since planner_3d is proceeding the action.");
    return;
  }
  setGoal(*msg);
}

bool Planner3dNode::setGoal(const geometry_msgs::msg::PoseStamped & msg)
{
  updateGoalTolerance();
  switch (planner_.setGoal(msg)) {
    case Planner3dCore::SetGoalResult::REJECTED:
      return false;
    case Planner3dCore::SetGoalResult::CLEARED:
      finishMoveBase(true, "Goal cleared.");
      finishTolerant(true, "Goal cleared.");
      updateGoalTolerance();
      break;
    default:
      break;
  }
  return true;
}

void Planner3dNode::cbMapUpdate(
  const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr & msg)
{
  applyMapUpdate(msg);
}

void Planner3dNode::applyMapUpdate(
  const std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> & msg)
{
  if (!planner_.hasMap()) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Map updated");
  updateGoalTolerance();
  if (trigger_plan_by_costmap_update_) {
    disarmNoMapUpdateTimer();
    updateStart();
    planner_.applyCostmapUpdate(msg);
    planPath(planner_.lastCostmapStamp());
    armNoMapUpdateTimer();
  } else {
    planner_.applyCostmapUpdate(msg);
  }
}

void Planner3dNode::cbMap(const costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr & msg)
{
  updateGoalTolerance();
  const std::shared_ptr<const costmap_cspace_msgs::msg::CSpace3DUpdate> map_update_retained =
    planner_.setMap(msg);
  jump_.setMapFrame(planner_.mapHeader().frame_id);
  if (map_update_retained) {
    RCLCPP_INFO(this->get_logger(), "Applying retained map update");
    applyMapUpdate(map_update_retained);
  }
  planner_.clearRetainedMapUpdate();
}

void Planner3dNode::diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const planner_cspace_msgs::msg::PlannerStatus & status = planner_.status();
  switch (status.error) {
    case planner_cspace_msgs::msg::PlannerStatus::GOING_WELL:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Going well.");
      break;
    case planner_cspace_msgs::msg::PlannerStatus::IN_ROCK:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "The robot is in rock.");
      break;
    case planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Path not found.");
      break;
    case planner_cspace_msgs::msg::PlannerStatus::DATA_MISSING:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Required data is missing.");
      break;
    case planner_cspace_msgs::msg::PlannerStatus::INTERNAL_ERROR:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Planner internal error.");
      break;
    default:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown error.");
      break;
  }
  stat.addf("status", "%u", status.status);
  stat.addf("error", "%u", status.error);
}

// --- Planning cycle -------------------------------------------------------
// ROS 1 used a one-shot ros::Timer which re-created itself in its callback;
// ROS 2 timers are periodic, so the same timer is only re-armed (reset()) or
// stopped (cancel()) here.
void Planner3dNode::armNoMapUpdateTimer()
{
  if (!(costmap_watchdog_ > rclcpp::Duration(0, 0))) {
    return;
  }
  if (!no_map_update_timer_) {
    no_map_update_timer_ = rclcpp::create_timer(
      this, this->get_clock(), costmap_watchdog_,
      std::bind(&Planner3dNode::cbNoMapUpdateTimer, this));
  } else {
    no_map_update_timer_->reset();
  }
}

void Planner3dNode::disarmNoMapUpdateTimer()
{
  if (no_map_update_timer_) {
    no_map_update_timer_->cancel();
  }
}

void Planner3dNode::cbNoMapUpdateTimer()
{
  // ros::TimerEvent::current_real has no ROS 2 counterpart; now() is used.
  planPath(this->now());
}

// One iteration of the ROS 1 spin()/waitUntil() loop.
void Planner3dNode::cbSpinTimer()
{
  processCancelRequests();

  if (trigger_plan_by_costmap_update_) {
    if (jump_.detectJump()) {
      planner_.clearRememberedCostmap();
    }
    return;
  }

  const rclcpp::Time costmap_stamp = planner_.lastCostmapStamp();
  const bool costmap_updated = costmap_stamp != prev_map_update_stamp_;
  prev_map_update_stamp_ = costmap_stamp;

  bool replan_now = false;
  if (planner_.hasMap()) {
    updateStart();

    if (jump_.detectJump()) {
      planner_.clearRememberedCostmap();
      // Robot pose jumped.
      replan_now = true;
    } else if (costmap_updated && planner_.isPreviousPathBlocked()) {
      // Obstacle on the path.
      replan_now = true;
    } else if (planner_.checkSwitchbackArrival()) {
      // robot has arrived at the switchback point
      replan_now = true;
    }
  }
  if (!replan_now && this->now() <= next_replan_time_) {
    return;
  }

  const rclcpp::Time now = this->now();
  planPath(now);
  if (planner_.isPathSwitchback()) {
    next_replan_time_ = now + rclcpp::Duration::from_seconds(planner_.swWait());
    RCLCPP_INFO(
      this->get_logger(), "Planned path has switchback. Planner will stop until: %f at the latest.",
      next_replan_time_.seconds());
  } else {
    next_replan_time_ = now + rclcpp::Duration::from_seconds(1.0 / freq_);
  }
}

// Looks up the robot pose and pushes it to the planning logic.
void Planner3dNode::updateStart()
{
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = robot_frame_;
  start.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  start.pose.orientation.x = 0.0;
  start.pose.orientation.y = 0.0;
  start.pose.orientation.z = 0.0;
  start.pose.orientation.w = 1.0;
  start.pose.position.x = 0;
  start.pose.position.y = 0;
  start.pose.position.z = 0;
  try {
    const geometry_msgs::msg::TransformStamped trans = tfbuf_.lookupTransform(
      planner_.mapHeader().frame_id, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    tf2::doTransform(start, start, trans);
  } catch (const tf2::TransformException & e) {
    planner_.clearStart();
    return;
  }
  planner_.setStart(start);
}

void Planner3dNode::planPath(const rclcpp::Time & now)
{
  updateGoalTolerance();
  const bool has_costmap = planner_.preparePlanCycle(now);

  if (planner_.isReadyToPlan() && has_costmap) {
    publishActionFeedback();
    switch (planner_.runPlanCycle(now)) {
      case Planner3dCore::PlanCycleResult::GOAL_REACHED:
        finishMoveBase(true, "Goal reached.");
        finishTolerant(true, "Goal reached.");
        updateGoalTolerance();
        break;
      case Planner3dCore::PlanCycleResult::ABORT_MAX_RETRY:
        finishMoveBase(false, "Goal is in Rock");
        finishTolerant(false, "Goal is in Rock");
        updateGoalTolerance();
        return;
      default:
        break;
    }
  } else if (!planner_.hasGoal()) {
    planner_.handleNoGoal();
  }
  pub_goal_->publish(
    std::make_unique<geometry_msgs::msg::PoseStamped>(planner_.currentGoalStamped()));
  planner_.setStatusStamp(now);
  pub_status_->publish(
    std::make_unique<planner_cspace_msgs::msg::PlannerStatus>(planner_.status()));
  diag_updater_.force_update();
  pub_metrics_->publish(
    std::make_unique<neonavigation_metrics_msgs::msg::Metrics>(planner_.collectMetrics(now)));
}
}  // namespace planner_3d
}  // namespace planner_cspace

RCLCPP_COMPONENTS_REGISTER_NODE(planner_cspace::planner_3d::Planner3dNode)
