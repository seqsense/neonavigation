/*
 * Copyright (c) 2016-2017, the neonavigation authors
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

#include <cstddef>
#include <memory>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_cspace_msgs/action/move_with_tolerance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace planner_cspace
{
// ROS 2 interface of the patrol sample node.
//
// It walks through the poses of the last received path, sending one action
// goal at a time to planner_3d. The ROS 1 node used actionlib and polled
// SimpleActionClient::getState() from a 10 Hz loop; rclcpp_action has no
// equivalent polled state, so the terminal state of each goal is recorded by a
// result callback and the same 10 Hz timer reacts to it:
//
//   actionlib                          rclcpp_action
//   ---------------------------------  ------------------------------------
//   SUCCEEDED                          ResultCode::SUCCEEDED
//   ABORTED                            ResultCode::ABORTED
//   LOST (server not available)        !action_server_is_ready()
//   cancelAllGoals()                   async_cancel_all_goals()
//
// The move_base goal type is nav2_msgs/action/NavigateToPose on ROS 2
// (move_base_msgs does not exist there); its `pose` field carries what
// `target_pose` carried on ROS 1.
class PatrolActionNode : public rclcpp::Node
{
public:
  explicit PatrolActionNode(const rclcpp::NodeOptions & options);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using MoveWithTolerance = planner_cspace_msgs::action::MoveWithTolerance;

  // Terminal state of the goal which was sent last.
  enum class GoalState
  {
    NONE,
    RUNNING,
    SUCCEEDED,
    ABORTED,
    CANCELED,
  };

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr act_cli_;
  rclcpp_action::Client<MoveWithTolerance>::SharedPtr act_cli_tolerant_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;
  size_t pos_;
  GoalState state_;
  bool with_tolerance_;
  double tolerance_lin_;
  double tolerance_ang_;
  double tolerance_ang_finish_;

  void cbPath(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  bool actionServerIsReady() const;
  bool sendNextGoal();
  void cbTimer();
};

PatrolActionNode::PatrolActionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("patrol", options), pos_(0), state_(GoalState::NONE)
{
  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
    "patrol_nodes", 1, std::bind(&PatrolActionNode::cbPath, this, std::placeholders::_1));

  with_tolerance_ = this->declare_parameter("with_tolerance", false);
  tolerance_lin_ = this->declare_parameter("tolerance_lin", 0.1);
  tolerance_ang_ = this->declare_parameter("tolerance_ang", 0.1);
  tolerance_ang_finish_ = this->declare_parameter("tolerance_ang_finish", 0.05);

  if (with_tolerance_) {
    act_cli_tolerant_ = rclcpp_action::create_client<MoveWithTolerance>(this, "tolerant_move");
  } else {
    act_cli_ = rclcpp_action::create_client<NavigateToPose>(this, "move_base");
  }

  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(0.1),
    std::bind(&PatrolActionNode::cbTimer, this));
}

void PatrolActionNode::cbPath(const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
  if (path_.poses.size() > 0) {
    // Cancel previous patrol if stored
    if (with_tolerance_) {
      act_cli_tolerant_->async_cancel_all_goals();
    } else {
      act_cli_->async_cancel_all_goals();
    }
  }
  path_ = *msg;
  pos_ = 0;
  state_ = GoalState::NONE;
}

bool PatrolActionNode::actionServerIsReady() const
{
  return with_tolerance_ ? act_cli_tolerant_->action_server_is_ready()
                         : act_cli_->action_server_is_ready();
}

bool PatrolActionNode::sendNextGoal()
{
  if (path_.poses.size() <= pos_) {
    RCLCPP_WARN(this->get_logger(), "Patrol finished. Waiting next path.");
    path_.poses.clear();

    return false;
  }

  if (with_tolerance_) {
    MoveWithTolerance::Goal goal;

    goal.target_pose.header = path_.poses[pos_].header;
    goal.target_pose.header.stamp = this->now();
    goal.target_pose.pose = path_.poses[pos_].pose;
    goal.goal_tolerance_lin = tolerance_lin_;
    goal.goal_tolerance_ang = tolerance_ang_;
    goal.goal_tolerance_ang_finish = tolerance_ang_finish_;

    rclcpp_action::Client<MoveWithTolerance>::SendGoalOptions options;
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<MoveWithTolerance>::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            state_ = GoalState::SUCCEEDED;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            state_ = GoalState::CANCELED;
            break;
          default:
            state_ = GoalState::ABORTED;
            break;
        }
      };
    act_cli_tolerant_->async_send_goal(goal, options);
  } else {
    NavigateToPose::Goal goal;

    goal.pose.header = path_.poses[pos_].header;
    goal.pose.header.stamp = this->now();
    goal.pose.pose = path_.poses[pos_].pose;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            state_ = GoalState::SUCCEEDED;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            state_ = GoalState::CANCELED;
            break;
          default:
            state_ = GoalState::ABORTED;
            break;
        }
      };
    act_cli_->async_send_goal(goal, options);
  }
  state_ = GoalState::RUNNING;
  pos_++;

  return true;
}

void PatrolActionNode::cbTimer()
{
  if (path_.poses.size() == 0) {
    return;
  }

  if (!actionServerIsReady()) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Action server is not ready.");
    return;
  }

  if (pos_ == 0) {
    sendNextGoal();
    return;
  }

  switch (state_) {
    case GoalState::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action has been finished.");
      sendNextGoal();
      break;
    case GoalState::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Action has been aborted. Skipping.");
      sendNextGoal();
      break;
    default:
      break;
  }
}
}  // namespace planner_cspace

RCLCPP_COMPONENTS_REGISTER_NODE(planner_cspace::PatrolActionNode)
