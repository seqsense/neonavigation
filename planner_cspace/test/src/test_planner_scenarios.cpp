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

// Deterministic ports of the planner_3d navigation scenarios.
//
// The launch tests run these through five live nodes and 90-second budgets;
// here the planner is stepped directly (see planner_scenario_harness.h). The
// maps are the ones test/data/global_map.png and local_map.png hold, written
// out as ASCII so the scenario can be read without opening an image, and the
// start pose, goals and tolerances are the ones the launch tests use.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "planner_cspace/planner_scenario_harness.h"

namespace
{
using planner_cspace::planner_3d::Planner3dCore;
using planner_cspace_testing::GridMap;
using planner_cspace_testing::PlannerScenario;
using Pose = PlannerScenario::Pose;

// test/data/global_map.png, as map_server reads it: '?' is unknown.
const std::vector<std::string> kGlobalMap = {
  "#######################?????????", "#.....................#?????????",
  "#.....................#?????????", "#.....................#?????????",
  "#.....................#?????????", "#.....................#?????????",
  "#.....................#?????????", "#.....................#?????????",
  "#.....................#?????????", "#.......................????????",
  "#.........................??????", "#..........##...............????",
  "#..........##.................??", "#..........##..................#",
  "#..........##..................#", "#..............................#",
  "#..............................#", "#..............................#",
  "#..............................#", "#..............................#",
  "#..............................#", "########..........##############",
  "#..............................#", "#..............................#",
  "#..............................#", "#..............................#",
  "#..............................#", "#..............................#",
  "#..............................#", "#..............................#",
  "#..............................#", "################################",
};

// test/data/local_map.png: the extra wall the local costmap adds.
const std::vector<std::string> kLocalMap = {
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "...........#####################",
  "...........#####################", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
  "................................", "................................",
};

// The launch file's planner_3d parameters; everything else keeps the defaults
// the node declares.
Planner3dCore::Parameters testParameters()
{
  Planner3dCore::Parameters p;
  p.max_vel = 0.1f;
  p.max_ang_vel = 0.3f;
  p.goal_tolerance_lin = 0.025;
  return p;
}

Planner3dCore::StaticParameters testStaticParameters()
{
  Planner3dCore::StaticParameters p;
  p.retain_last_error_status = false;
  return p;
}

// The launch tests' initial pose and patrol waypoints, with the waypoints moved
// two cells away from the top border: this robot lands exactly on the goal
// cell, and the planner refuses to plan from a pose within its search range of
// the map edge ("You are on the edge of the world"). A real robot stops a few
// centimetres short and never sees it.
constexpr Pose kStart{2.5, 0.45, M_PI};
constexpr Pose kGoal1{1.7, 2.6, -3.14};
constexpr Pose kGoal2{1.9, 2.6, -1.57};

std::string describe(const PlannerScenario & s)
{
  std::stringstream ss;
  ss << "robot (" << s.robot().x << ", " << s.robot().y << ", " << s.robot().yaw << "), status "
     << static_cast<int>(s.status().status) << "/" << static_cast<int>(s.status().error)
     << ", path " << s.path().poses.size() << " poses, t " << s.time() << " s, " << s.cycles()
     << " cycles";
  return ss.str();
}

// Reached as the launch tests judge it: within 0.2 m and 0.2 rad of the goal.
bool arrived(const PlannerScenario & s, const Pose & goal)
{
  return s.distanceToGoal(goal) < 0.2 &&
         std::abs(std::remainder(s.robot().yaw - goal.yaw, 2.0 * M_PI)) < 0.2;
}

void navigateTo(PlannerScenario & scenario, const Pose & goal, const double budget_sec = 90.0)
{
  scenario.setGoal(goal);
  const bool reached = scenario.runUntil(budget_sec, [&goal](PlannerScenario & s) {
    return arrived(s, goal) && s.status().status == planner_cspace_msgs::msg::PlannerStatus::DONE;
  });
  ASSERT_TRUE(reached) << "did not reach the goal: " << describe(scenario);
}

TEST(PlannerScenario, Navigate)
{
  PlannerScenario scenario(testParameters(), testStaticParameters());
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);

  ASSERT_NO_FATAL_FAILURE(navigateTo(scenario, kGoal1));
  ASSERT_NO_FATAL_FAILURE(navigateTo(scenario, kGoal2));
}

TEST(PlannerScenario, NavigateWithLocalMap)
{
  PlannerScenario scenario(testParameters(), testStaticParameters());
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);

  // The local costmap keeps adding its wall while the robot drives.
  GridMap local(kLocalMap);
  scenario.setGoal(kGoal1);
  const bool reached = scenario.runUntil(
    90.0,
    [](PlannerScenario & s) {
      return arrived(s, kGoal1) &&
             s.status().status == planner_cspace_msgs::msg::PlannerStatus::DONE;
    },
    [&local](PlannerScenario & s) { s.applyMapUpdate(local); });
  ASSERT_TRUE(reached) << "did not reach the goal: " << describe(scenario);
}

// The variants the launch tests spent a matrix of processes on are parameters
// here.
TEST(PlannerScenario, NavigateWithAntialiasStart)
{
  auto params = testParameters();
  params.antialias_start = true;
  PlannerScenario scenario(params, testStaticParameters());
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);

  ASSERT_NO_FATAL_FAILURE(navigateTo(scenario, kGoal1));
}

TEST(PlannerScenario, NavigateWithAntialiasStartAndFastMapUpdate)
{
  auto params = testParameters();
  params.antialias_start = true;
  params.fast_map_update = true;
  PlannerScenario scenario(params, testStaticParameters());
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);

  ASSERT_NO_FATAL_FAILURE(navigateTo(scenario, kGoal1));
}

TEST(PlannerScenario, NavigateWithTolerance)
{
  PlannerScenario scenario(testParameters(), testStaticParameters());
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);

  Planner3dCore::GoalTolerance tolerance;
  tolerance.lin = 0.1;
  tolerance.ang = 0.2;
  tolerance.ang_finish = 0.1;
  scenario.core().setGoalTolerance(tolerance);

  ASSERT_NO_FATAL_FAILURE(navigateTo(scenario, kGoal1));
}

TEST(PlannerScenario, GlobalPlan)
{
  PlannerScenario scenario(testParameters(), testStaticParameters());
  const GridMap map(kGlobalMap);
  scenario.setMap(map);
  scenario.setStart(kStart);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position.x = 1.95;
  start.pose.position.y = 0.45;
  start.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 3.14));

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 1.25;
  goal.pose.position.y = 2.15;
  goal.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));

  // (12, 21, 0) is in rock, so planning to it exactly must fail.
  nav_msgs::msg::Path plan;
  ASSERT_FALSE(scenario.core().makePlanOnDemand(start, goal, 0.0, plan));

  // With a tolerance the goal is moved to a free cell nearby and a plan exists.
  // Which cell that is depends on the costmap, and this harness inflates the
  // footprint slightly differently from costmap_cspace, so the end of the plan
  // is checked against the tolerance rather than against an exact pose.
  ASSERT_TRUE(scenario.core().makePlanOnDemand(start, goal, 0.1, plan));
  ASSERT_FALSE(plan.poses.empty());
  EXPECT_NEAR(start.pose.position.x, plan.poses.front().pose.position.x, 1.0e-5);
  EXPECT_NEAR(start.pose.position.y, plan.poses.front().pose.position.y, 1.0e-5);
  EXPECT_NEAR(goal.pose.position.x, plan.poses.back().pose.position.x, 0.15);
  EXPECT_NEAR(goal.pose.position.y, plan.poses.back().pose.position.y, 0.15);

  // No pose on the plan may sit on an obstacle.
  for (const auto & p : plan.poses) {
    const int map_x = static_cast<int>(p.pose.position.x / map.resolution());
    const int map_y = static_cast<int>(p.pose.position.y / map.resolution());
    ASSERT_GE(map_x, 0);
    ASSERT_GE(map_y, 0);
    ASSERT_LT(map_x, map.width());
    ASSERT_LT(map_y, map.height());
    ASSERT_NE(map.rawCostAt(map_x, map_y), 100)
      << "plan crosses an obstacle at " << map_x << ", " << map_y;
  }
}

// Waits for the planner to report a particular error, the way the launch tests
// watched the status topic.
bool waitForError(PlannerScenario & scenario, const uint8_t error, const double budget_sec)
{
  return scenario.runUntil(
    budget_sec, [error](PlannerScenario & s) { return s.status().error == error; });
}

TEST(PlannerScenario, RobotIsWalledOffFromTheGoal)
{
  PlannerScenario scenario(testParameters(), testStaticParameters());
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);

  // The local costmap's wall cuts the map in two, leaving the goal unreachable.
  const GridMap local(kLocalMap);
  scenario.applyMapUpdate(local);
  scenario.setGoal({1.19, 1.90, 0.0});

  const bool reported = scenario.runUntil(
    20.0,
    [](PlannerScenario & s) {
      return s.status().error == planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND;
    },
    [&local](PlannerScenario & s) { s.applyMapUpdate(local); });
  ASSERT_TRUE(reported) << "expected PATH_NOT_FOUND: " << describe(scenario);
}

TEST(PlannerScenario, GoalIsInRockRecovered)
{
  PlannerScenario scenario(testParameters(), testStaticParameters());
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);

  // Drop an obstacle over the goal, as the launch test does through the local
  // costmap, and the planner must report that there is no path.
  GridMap blocked(kGlobalMap);
  blocked.fill(10, 22, 16, 26, '#');
  scenario.applyMapUpdate(blocked);
  scenario.setGoal({1.25, 2.55, 0.0});

  ASSERT_TRUE(waitForError(scenario, planner_cspace_msgs::msg::PlannerStatus::PATH_NOT_FOUND, 30.0))
    << "expected to get stuck: " << describe(scenario);

  // Clear it again and the planner must recover on its own.
  scenario.applyMapUpdate(GridMap(kGlobalMap));
  ASSERT_TRUE(waitForError(scenario, planner_cspace_msgs::msg::PlannerStatus::GOING_WELL, 30.0))
    << "expected to recover: " << describe(scenario);
}

// costmap_watchdog is not here on purpose: Planner3dCore stamps the costmap
// with rclcpp::Clock(RCL_ROS_TIME).now() inside applyCostmapUpdate rather than
// with the time the planning cycle is given, so the age it compares against the
// watchdog can only be produced by a real clock. test_costmap_watchdog covers
// it as a launch test.

// Crowd mode: when the goal cannot be reached the planner escapes to a
// temporary goal instead of standing still.
TEST(PlannerScenario, CrowdEscapeWhenPathIsBlocked)
{
  auto params = testParameters();
  params.temporary_escape = true;
  auto static_params = testStaticParameters();
  static_params.enable_crowd_mode = true;

  PlannerScenario scenario(params, static_params);
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);

  const GridMap local(kLocalMap);
  scenario.applyMapUpdate(local);
  scenario.setGoal({1.6, 2.2, 1.57});

  const Pose from = scenario.robot();
  const bool moved = scenario.runUntil(
    60.0,
    [&from](PlannerScenario & s) {
      // An escape shows up as the robot leaving where it started.
      return std::hypot(s.robot().x - from.x, s.robot().y - from.y) > 0.3;
    },
    [&local](PlannerScenario & s) { s.applyMapUpdate(local); });
  ASSERT_TRUE(moved) << "the robot never escaped: " << describe(scenario);
}

TEST(PlannerScenario, ForceTemporaryEscape)
{
  auto params = testParameters();
  params.temporary_escape = true;
  auto static_params = testStaticParameters();
  static_params.enable_crowd_mode = true;

  PlannerScenario scenario(params, static_params);
  scenario.setMap(GridMap(kGlobalMap));
  scenario.setStart(kStart);
  scenario.setGoal({1.6, 2.2, 1.57});

  const Pose from = scenario.robot();
  const bool moved = scenario.runUntil(
    60.0,
    [&from](PlannerScenario & s) {
      return std::hypot(s.robot().x - from.x, s.robot().y - from.y) > 0.3;
    },
    [](PlannerScenario & s) { s.core().triggerTemporaryEscape(); });
  ASSERT_TRUE(moved) << "the forced escape did not move the robot: " << describe(scenario);
}

}  // namespace
