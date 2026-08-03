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

#ifndef PLANNER_CSPACE__ROS2_ACTION_TEST_BASE_H_
#define PLANNER_CSPACE__ROS2_ACTION_TEST_BASE_H_

// rclcpp_action counterpart of test/include/planner_cspace/action_test_base.h.
//
// actionlib's SimpleActionClient exposed a polled goal state; rclcpp_action
// only reports the terminal state through the result future, so the state the
// ROS 1 tests polled is reconstructed here:
//
//   actionlib SimpleClientGoalState   this base
//   -------------------------------   ------------------------------------
//   ACTIVE                            goal accepted and no result yet
//   SUCCEEDED                         ResultCode::SUCCEEDED
//   ABORTED                           ResultCode::ABORTED
//   PREEMPTED (explicit cancel)       ResultCode::CANCELED
//   PREEMPTED (superseded goal)       ResultCode::ABORTED, "Preempted." text
//   REJECTED                          goal handle is null
//
// The last row is the one real semantic difference: ROS 2 has no terminal
// state which means "replaced by a newer goal", so the ROS 2 planner_3d node
// aborts the superseded goal with "Preempted." in its status text.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "nav_msgs/srv/get_plan.hpp"
#include "planner_cspace/ros2_test_helpers.h"
#include "planner_cspace_msgs/msg/planner_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

constexpr const char ACTION_TOPIC_MOVE_BASE[] = "move_base";
constexpr const char ACTION_TOPIC_TOLERANT_MOVE[] = "tolerant_move";

template <typename ACTION, char const * TOPIC>
class ActionTestBase : public ::testing::Test
{
public:
  using ActionClient = rclcpp_action::Client<ACTION>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ACTION>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  void SetUp() override
  {
    setUpNode();
    waitPlannerReady();
  }

protected:
  // SetUp() is split so that a derived fixture can create its own
  // publishers/subscriptions (and feed the planner) between the two halves,
  // which is what the ROS 1 dynamic parameter test did.
  void setUpNode()
  {
    node_ = rclcpp::Node::make_shared(std::string("test_") + TOPIC + "_client");
    tfbuf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
    action_client_ = rclcpp_action::create_client<ACTION>(node_, TOPIC);
    sub_status_ = node_->create_subscription<planner_cspace_msgs::msg::PlannerStatus>(
      "/planner_3d/status", rclcpp::QoS(10).transient_local(),
      [this](const planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr msg) {
        planner_status_ = msg;
      });
    srv_plan_ = node_->create_client<nav_msgs::srv::GetPlan>("/planner_3d/make_plan");
  }

  void waitPlannerReady()
  {
    ASSERT_TRUE(planner_cspace_testing::spinUntil(
      node_, std::chrono::seconds(30), [this] { return action_client_->action_server_is_ready(); }))
      << "Failed to connect " << TOPIC << " action";

    // ROS 1 polled ~/make_plan, which returned false until planner_3d had a
    // map. A ROS 2 service cannot report a failure, so the node answers with
    // an empty plan instead and the emptiness is what is polled here.
    ASSERT_TRUE(planner_cspace_testing::spinUntil(
      node_, std::chrono::seconds(30), [this] { return planIsAvailable(); }, nullptr,
      std::chrono::milliseconds(500)))
      << "planner_3d didn't receive map";
  }

  // Fires a ~/make_plan request between two nearby free poses and reports
  // whether a non-empty plan came back within a short timeout.
  bool planIsAvailable()
  {
    if (!srv_plan_->service_is_ready()) {
      return false;
    }
    auto req = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    req->tolerance = 10.0;
    req->start.header.frame_id = "map";
    req->start.pose.position.x = 1.24;
    req->start.pose.position.y = 0.65;
    req->start.pose.orientation.w = 1;
    req->goal.header.frame_id = "map";
    req->goal.pose.position.x = 1.25;
    req->goal.pose.position.y = 0.75;
    req->goal.pose.orientation.w = 1;
    auto future = srv_plan_->async_send_request(req);
    if (!planner_cspace_testing::spinUntil(node_, std::chrono::seconds(2), [&future] {
          return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        })) {
      srv_plan_->remove_pending_request(future);
      return false;
    }
    return !future.get()->plan.poses.empty();
  }

  // One sent goal and the terminal result which came back for it. Kept in a
  // shared_ptr so that a test can hold on to a goal after it has sent the next
  // one (the superseded-goal case of the preempt test).
  struct GoalSession
  {
    typename GoalHandle::SharedPtr handle;
    std::shared_ptr<WrappedResult> result;
  };
  using GoalSessionPtr = std::shared_ptr<GoalSession>;

  // Sends a goal and waits until the server has accepted or rejected it.
  // Returns false only on a timeout; a rejected goal leaves goalHandle() null.
  bool sendGoal(
    const typename ACTION::Goal & goal,
    const std::chrono::nanoseconds timeout = std::chrono::seconds(10))
  {
    auto session = std::make_shared<GoalSession>();
    session_ = session;

    typename ActionClient::SendGoalOptions options;
    // Setting result_callback makes the client request the result as soon as
    // the goal is accepted, which is what turns GoalSession::result into the
    // "the goal is no longer active" flag used below. The session is captured
    // by value so that a result arriving after the next goal was sent lands on
    // its own session instead of the current one.
    options.result_callback = [session](const WrappedResult & result) {
      session->result = std::make_shared<WrappedResult>(result);
    };
    auto future = action_client_->async_send_goal(goal, options);
    if (!planner_cspace_testing::spinUntil(node_, timeout, [&future] {
          return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        })) {
      return false;
    }
    session->handle = future.get();
    return true;
  }

  GoalSessionPtr session() const { return session_; }

  typename GoalHandle::SharedPtr goalHandle() const
  {
    return session_ ? session_->handle : nullptr;
  }

  // Counterpart of SimpleClientGoalState::ACTIVE.
  bool isActive() const { return session_ && session_->handle && !session_->result; }

  bool waitResult(const std::chrono::nanoseconds timeout, const GoalSessionPtr & session = nullptr)
  {
    const GoalSessionPtr s = session ? session : session_;
    return planner_cspace_testing::spinUntil(
      node_, timeout, [s] { return s && static_cast<bool>(s->result); });
  }

  // Cancels the goal repeatedly (like the ROS 1 test's cancelAllGoals() loop)
  // until the result arrives.
  bool cancelAndWait(const std::chrono::nanoseconds timeout)
  {
    const GoalSessionPtr s = session_;
    return planner_cspace_testing::spinUntil(
      node_, timeout, [s] { return s && static_cast<bool>(s->result); },
      [this, s] {
        if (s && s->handle) {
          action_client_->async_cancel_all_goals();
        }
      },
      std::chrono::milliseconds(500));
  }

  rclcpp_action::ResultCode resultCode() const
  {
    return (session_ && session_->result) ? session_->result->code
                                          : rclcpp_action::ResultCode::UNKNOWN;
  }

  std::shared_ptr<WrappedResult> result() const { return session_ ? session_->result : nullptr; }

  void spinSome() { rclcpp::spin_some(node_); }

  std::string statusString() const
  {
    if (!planner_status_) {
      return "(no status)";
    }
    return "(status: " + std::to_string(planner_status_->status) +
           ", error: " + std::to_string(planner_status_->error) + ")";
  }

  std::string resultString() const
  {
    if (!session_ || !session_->handle) {
      return "REJECTED";
    }
    if (!session_->result) {
      return "ACTIVE";
    }
    switch (session_->result->code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return "SUCCEEDED";
      case rclcpp_action::ResultCode::ABORTED:
        return "ABORTED";
      case rclcpp_action::ResultCode::CANCELED:
        return "CANCELED";
      default:
        return "UNKNOWN";
    }
  }

  rclcpp::Node::SharedPtr node_;
  typename ActionClient::SharedPtr action_client_;
  rclcpp::Subscription<planner_cspace_msgs::msg::PlannerStatus>::SharedPtr sub_status_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_;
  planner_cspace_msgs::msg::PlannerStatus::ConstSharedPtr planner_status_;
  std::unique_ptr<tf2_ros::Buffer> tfbuf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

private:
  GoalSessionPtr session_;
};

#endif  // PLANNER_CSPACE__ROS2_ACTION_TEST_BASE_H_
