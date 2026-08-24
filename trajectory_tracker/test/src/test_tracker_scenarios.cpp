/*
 * Copyright (c) 2018, the neonavigation authors
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

// Deterministic ports of the trajectory_tracker node scenarios.
//
// These are the same paths, the same kinematic model and the same tolerances as
// the ROS 1 rostest / ROS 2 launch_test versions, but the controller is driven
// directly (see scenario_harness.h): no nodes, no DDS, no simulated clock
// process. That removes the flakiness those tests had and runs the whole set in
// well under a second, so the node tests only have to prove that the interface
// layer is wired up, not that the control law works.
//
// The parameter set mirrors test/configs/test_params.yaml, so a number that
// changes here should change there too.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "trajectory_tracker/scenario_harness.h"

namespace
{
using trajectory_tracker_testing::Scenario;
using Status = trajectory_tracker_msgs::msg::TrajectoryTrackerStatus;

constexpr double kErrorLin = 0.03;
constexpr double kErrorAng = 0.03;

// test/configs/test_params.yaml, with the node's declare_parameter defaults for
// everything that file leaves out.
trajectory_tracker::TrackerController::Parameters testParameters()
{
  trajectory_tracker::TrackerController::Parameters p;
  p.look_forward = 0.0;
  p.curv_forward = 0.5;
  p.k[0] = 4.5;
  p.k[1] = 3.0;
  p.k[2] = 4.0;
  p.gain_at_vel = 1.0;
  p.d_lim = 0.5;
  p.d_stop = 2.0;
  p.rotate_ang = 0.78539816339;
  p.vel[0] = 1.0;
  p.vel[1] = 0.5;
  p.acc[0] = 2.0;
  p.acc[1] = 2.0;
  p.acc_toc[0] = 2.0 * 0.9;
  p.acc_toc[1] = 2.0 * 0.9;
  p.path_step = 1;
  p.goal_tolerance_dist = 0.005;
  p.goal_tolerance_ang = 0.005;
  p.stop_tolerance_dist = 0.002;
  p.stop_tolerance_ang = 0.002;
  p.no_pos_cntl_dist = 0.0;
  p.min_track_path = 0.0;
  p.allow_backward = true;
  p.limit_vel_by_avel = false;
  p.check_old_path = false;
  p.epsilon = 0.001;
  p.use_time_optimal_control = true;
  p.time_optimal_control_future_gain = 1.5;
  p.k_ang_rotation = 8.0;
  p.k_avel_rotation = 5.0;
  p.goal_tolerance_lin_vel = 0.0;
  p.goal_tolerance_ang_vel = 0.0;
  return p;
}

// The node tests kept sampling the pose for a while after the goal to catch a
// late overshoot; five rounds of five control cycles is what they did.
void expectSettledAt(
  Scenario & scenario, const Eigen::Vector2d & pos, const double yaw, const std::string & info = "")
{
  for (int round = 0; round < 5; ++round) {
    scenario.settle(5);
    const auto & s = scenario.state();
    ASSERT_NEAR(s.yaw, yaw, kErrorAng) << "[overshoot after goal (" << round << ")] " << info;
    ASSERT_NEAR(s.pos[0], pos[0], kErrorLin) << "[overshoot after goal (" << round << ")] " << info;
    ASSERT_NEAR(s.pos[1], pos[1], kErrorLin) << "[overshoot after goal (" << round << ")] " << info;
  }
}

std::string describe(const Scenario::State & s)
{
  std::stringstream ss;
  ss << "pos (" << s.pos[0] << ", " << s.pos[1] << "), yaw " << s.yaw << ", status "
     << static_cast<int>(s.status.status) << ", remains " << s.status.distance_remains << "/"
     << s.status.angle_remains << ", t " << s.time;
  return ss.str();
}

TEST(TrackerScenario, StraightStop)
{
  Scenario scenario(testParameters());
  scenario.initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  for (double x = 0.0; x < 0.5; x += 0.01) {
    poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
  }
  poses.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
  scenario.setPath(trajectory_tracker_testing::makePath(poses));

  ASSERT_TRUE(scenario.runUntilGoal(10.0)) << describe(scenario.state());
  expectSettledAt(scenario, Eigen::Vector2d(0.5, 0.0), 0.0);
}

TEST(TrackerScenario, StraightStopOvershoot)
{
  for (const double resolution : {0.1, 0.001, 0.0001}) {
    const std::string info = "resolution: " + std::to_string(resolution);

    Scenario scenario(testParameters());
    scenario.initState(Eigen::Vector2d(1, 0), 0);

    std::vector<Eigen::Vector3d> poses;
    for (double x = 0.0; x < 0.5 - resolution; x += 0.1) {
      poses.push_back(Eigen::Vector3d(x, 0, 0));
    }
    poses.push_back(Eigen::Vector3d(0.5 - resolution, 0, 0));
    poses.push_back(Eigen::Vector3d(0.5, 0, 0));
    scenario.setPath(trajectory_tracker_testing::makePath(poses));

    ASSERT_TRUE(scenario.runUntilGoal(10.0)) << info << " " << describe(scenario.state());
    expectSettledAt(scenario, Eigen::Vector2d(0.5, 0.0), 0.0, info);
  }
}

TEST(TrackerScenario, StraightStopConvergence)
{
  const double path_length = 2.0;
  for (const double vel : {0.05, 0.1, 0.2, 0.5, 1.0}) {
    const std::string info = "linear vel: " + std::to_string(vel);

    Scenario scenario(testParameters());
    scenario.initState(Eigen::Vector2d(0, 0.01), 0);

    std::vector<Eigen::Vector4d> poses;
    for (double x = 0.0; x < path_length; x += 0.01) {
      poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, vel));
    }
    poses.push_back(Eigen::Vector4d(path_length, 0.0, 0.0, vel));
    scenario.setPath(trajectory_tracker_testing::makePathWithVelocity(poses));

    ASSERT_TRUE(scenario.runUntilGoal(5.0 + path_length / vel))
      << info << " " << describe(scenario.state());
    expectSettledAt(scenario, Eigen::Vector2d(path_length, 0.0), 0.0, info);
  }
}

TEST(TrackerScenario, StraightVelocityChange)
{
  Scenario scenario(testParameters());
  scenario.initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector4d> poses;
  for (double x = 0.0; x < 0.6; x += 0.01) {
    poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, 0.3));
  }
  for (double x = 0.6; x < 1.5; x += 0.01) {
    poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, 0.5));
  }
  poses.push_back(Eigen::Vector4d(1.5, 0.0, 0.0, 0.5));
  scenario.setPath(trajectory_tracker_testing::makePathWithVelocity(poses));

  // The commanded speed must follow the per-pose velocity of the segment the
  // robot is on.
  const bool reached = scenario.runUntilGoal(10.0, [](const Scenario::State & s) {
    if (0.3 < s.pos[0] && s.pos[0] < 0.35) {
      ASSERT_NEAR(s.cmd_vel.linear.x, 0.3, kErrorLin) << describe(s);
    } else if (0.95 < s.pos[0] && s.pos[0] < 1.0) {
      ASSERT_NEAR(s.cmd_vel.linear.x, 0.5, kErrorLin) << describe(s);
    }
  });
  ASSERT_TRUE(reached) << describe(scenario.state());
  expectSettledAt(scenario, Eigen::Vector2d(1.5, 0.0), 0.0);
}

TEST(TrackerScenario, CurveFollow)
{
  Scenario scenario(testParameters());
  scenario.initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  Eigen::Vector3d p(0.0, 0.0, 0.0);
  for (double t = 0.0; t < 1.0; t += 0.01) {
    p += Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, 0.005);
    poses.push_back(p);
  }
  scenario.setPath(trajectory_tracker_testing::makePath(poses));

  ASSERT_TRUE(scenario.runUntilGoal(20.0)) << describe(scenario.state());
  expectSettledAt(scenario, Eigen::Vector2d(p[0], p[1]), p[2]);
}

TEST(TrackerScenario, InPlaceTurn)
{
  const double init_yaws[] = {0.0, 1.0, -2.0};
  for (const double init_yaw : init_yaws) {
    const std::vector<std::vector<float>> target_angle_array = {
      {0.5}, {-0.5}, {0.5, 0.5}, {-0.5, -0.5}};
    for (const auto & angles : target_angle_array) {
      for (const bool has_short_path : {false, true}) {
        std::stringstream info;
        info << "init_yaw: " << init_yaw << ", angles: " << angles.front() << "-" << angles.back()
             << ", has_short_path: " << has_short_path;

        Scenario scenario(testParameters());
        scenario.initState(Eigen::Vector2d(0, 0), init_yaw);

        std::vector<Eigen::Vector3d> poses;
        if (has_short_path) {
          poses.push_back(
            Eigen::Vector3d(-std::cos(init_yaw) * 0.01, std::sin(init_yaw) * 0.01, init_yaw));
        }
        for (const float ang : angles) {
          poses.push_back(Eigen::Vector3d(0.0, 0.0, init_yaw + ang));
        }
        scenario.setPath(trajectory_tracker_testing::makePath(poses));

        const double sign = std::copysign(1.0, angles.back());
        const bool reached = scenario.runUntilGoal(10.0, [&](const Scenario::State & s) {
          if (s.steps > 5) {
            ASSERT_GT(s.cmd_vel.angular.z * sign, -kErrorAng)
              << "[overshoot detected] " << info.str();
            ASSERT_LT(s.status.angle_remains * sign, kErrorAng)
              << "[overshoot detected] " << info.str();
          }
        });
        ASSERT_TRUE(reached) << info.str() << " " << describe(scenario.state());
        expectSettledAt(scenario, Eigen::Vector2d(0, 0), init_yaw + angles.back(), info.str());
      }
    }
  }
}

TEST(TrackerScenario, SwitchBack)
{
  Scenario scenario(testParameters());
  scenario.initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  Eigen::Vector3d p(0.0, 0.0, 0.0);
  for (double t = 0.0; t < 0.5; t += 0.01) {
    p += Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, 0.01);
    poses.push_back(p);
  }
  for (double t = 0.0; t < 0.5; t += 0.01) {
    p -= Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, -0.01);
    poses.push_back(p);
  }
  scenario.setPath(trajectory_tracker_testing::makePath(poses));

  ASSERT_TRUE(scenario.runUntilGoal(20.0)) << describe(scenario.state());
  expectSettledAt(scenario, Eigen::Vector2d(p[0], p[1]), p[2]);
}

TEST(TrackerScenario, FarAray)
{
  // Same path as StraightStop, but 500 m away from the origin: the tracker must
  // stay accurate where float precision starts to bite.
  const double y_pos = 500.0;

  Scenario scenario(testParameters());
  scenario.initState(Eigen::Vector2d(0.0, y_pos), 0);

  std::vector<Eigen::Vector3d> poses;
  for (double x = 0.0; x < 0.5; x += 0.01) {
    poses.push_back(Eigen::Vector3d(x, y_pos, 0.0));
  }
  poses.push_back(Eigen::Vector3d(0.5, y_pos, 0.0));
  scenario.setPath(trajectory_tracker_testing::makePath(poses));

  ASSERT_TRUE(scenario.runUntilGoal(10.0)) << describe(scenario.state());
  expectSettledAt(scenario, Eigen::Vector2d(0.5, y_pos), 0.0);
}

TEST(TrackerScenario, SwitchBackWithPathUpdate)
{
  Scenario scenario(testParameters());
  scenario.initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  std::vector<Eigen::Vector3d> poses_second_half;
  Eigen::Vector3d p(0.0, 0.0, 0.0);
  for (double t = 0.0; t < 0.5; t += 0.01) {
    p -= Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, -0.01);
    poses.push_back(p);
  }
  const Eigen::Vector2d pos_local_goal = p.head<2>();
  for (double t = 0.0; t < 1.0; t += 0.01) {
    p += Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, 0.01);
    poses.push_back(p);
    poses_second_half.push_back(p);
  }
  scenario.setPath(trajectory_tracker_testing::makePath(poses));

  // Once the robot has been sitting on the switchback point for a while, the
  // path is replaced by its second half - the same thing a global planner does
  // when the robot has finished reversing.
  int cnt_arrive_local_goal = 0;
  bool switched = false;
  const bool reached = scenario.runUntilGoal(15.0, [&](const Scenario::State & s) {
    if ((pos_local_goal - s.pos).norm() < 0.1) {
      ++cnt_arrive_local_goal;
    }
    if (!switched && cnt_arrive_local_goal > 25) {
      scenario.setPath(trajectory_tracker_testing::makePath(poses_second_half));
      switched = true;
    }
  });
  ASSERT_GT(cnt_arrive_local_goal, 25) << "failed to update path";
  ASSERT_TRUE(reached) << describe(scenario.state());
  expectSettledAt(scenario, Eigen::Vector2d(p[0], p[1]), p[2]);
}

// Ports of the node-level overshoot tests: the robot is parked on the goal but
// reports a residual velocity, and the goal velocity tolerances decide whether
// that still counts as arrived.
void runVelocityTolerance(
  const double goal_tolerance_lin_vel, const double goal_tolerance_ang_vel,
  const double reported_linear, const double reported_angular, const uint8_t expected_status)
{
  auto params = testParameters();
  params.goal_tolerance_lin_vel = goal_tolerance_lin_vel;
  params.goal_tolerance_ang_vel = goal_tolerance_ang_vel;

  Scenario::Options options;
  options.use_odom = true;
  Scenario scenario(params, options);
  scenario.initState(Eigen::Vector2d(0.5, 0.0), 0.0);

  std::vector<Eigen::Vector3d> poses;
  for (double x = 0.0; x < 0.5; x += 0.01) {
    poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
  }
  poses.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
  scenario.setPath(trajectory_tracker_testing::makePath(poses));

  scenario.setReportedVelocity(reported_linear, reported_angular);
  scenario.freezePose(true);

  // The node test only looked at the status half a second in.
  scenario.settle(50);
  EXPECT_EQ(scenario.state().status.status, expected_status) << describe(scenario.state());
}

TEST(TrackerScenario, NoVelocityToleranceWithRemainingLinearVel)
{
  runVelocityTolerance(0.0, 0.0, 0.1, 0.0, Status::GOAL);
}

TEST(TrackerScenario, NoVelocityToleranceWithRemainingAngularVel)
{
  runVelocityTolerance(0.0, 0.0, 0.0, 0.1, Status::GOAL);
}

TEST(TrackerScenario, LinearVelocityToleranceWithRemainingLinearVel)
{
  runVelocityTolerance(0.05, 0.0, 0.1, 0.0, Status::FOLLOWING);
}

TEST(TrackerScenario, LinearVelocityToleranceWithRemainingAngularVel)
{
  runVelocityTolerance(0.05, 0.0, 0.0, 0.1, Status::GOAL);
}

TEST(TrackerScenario, AngularVelocityToleranceWithRemainingLinearVel)
{
  runVelocityTolerance(0.0, 0.05, 0.1, 0.0, Status::GOAL);
}

TEST(TrackerScenario, AngularVelocityToleranceWithRemainingAngularVel)
{
  runVelocityTolerance(0.0, 0.05, 0.0, 0.1, Status::FOLLOWING);
}

// The launch tests also ran the straight and curved scenarios with delayed
// odometry feedback, with and without time-optimal control. Those are two
// parameters here rather than two more processes.
TEST(TrackerScenario, DelayedOdometryFeedback)
{
  for (const bool time_optimal : {true, false}) {
    const std::string info =
      std::string("use_time_optimal_control: ") + (time_optimal ? "true" : "false");

    auto params = testParameters();
    params.use_time_optimal_control = time_optimal;

    Scenario::Options options;
    options.use_odom = true;
    options.odom_delay = 0.04;
    Scenario scenario(params, options);
    scenario.initState(Eigen::Vector2d(0, 0), 0);

    std::vector<Eigen::Vector3d> poses;
    for (double x = 0.0; x < 0.5; x += 0.01) {
      poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
    }
    poses.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
    scenario.setPath(trajectory_tracker_testing::makePath(poses));

    ASSERT_TRUE(scenario.runUntilGoal(10.0)) << info << " " << describe(scenario.state());
    expectSettledAt(scenario, Eigen::Vector2d(0.5, 0.0), 0.0, info);
  }
}

}  // namespace
