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

#include <trajectory_tracker_test.h>

#include <algorithm>
#include <string>
#include <vector>

// Integration smoke test: the node subscribes to the path, publishes cmd_vel
// and status, follows a straight line to the goal and reports the path header
// it acted on. The control law itself is covered by
// test_trajectory_tracker_scenarios.
TEST_F(TrajectoryTrackerTest, FollowsPathThroughTheNodeInterface)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  for (double x = 0.0; x < 0.5; x += 0.01) poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
  poses.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
  waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

  ros::Rate rate(50);
  const ros::Time start = ros::Time::now();
  while (ros::ok()) {
    if (ros::Time::now() > start + ros::Duration(10.0)) {
      FAIL() << "Timeout" << std::endl
             << "Pos " << getPos() << std::endl
             << "Yaw " << getYaw() << std::endl
             << "Status " << std::endl
             << status_ << std::endl;
    }

    publishTransform();
    rate.sleep();
    ros::spinOnce();
    if (status_->status == trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL) break;
  }
  for (int j = 0; j < 5; ++j) {
    for (int i = 0; i < 5; ++i) {
      publishTransform();
      rate.sleep();
      ros::spinOnce();
    }

    // Check multiple times to assert overshoot.
    ASSERT_NEAR(getYaw(), 0.0, error_ang_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(getPos()[0], 0.5, error_lin_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(getPos()[1], 0.0, error_lin_) << "[overshoot after goal (" << j << ")] ";
  }
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
}

// The remaining scenarios - overshoot behaviour, velocity changes, curves,
// in-place turns, switchbacks, far-away paths - are covered without a node by
// test_trajectory_tracker_scenarios, which drives the same controller through
// the same paths against a virtual clock. Running them here as well only added
// runtime and, because simulated time keeps advancing whether or not the node
// gets scheduled, the occasional false failure.

void timeSource()
{
  ros::NodeHandle nh("/");
  bool use_sim_time;
  nh.param("/use_sim_time", use_sim_time, false);
  if (!use_sim_time) return;

  ros::Publisher pub = nh.advertise<rosgraph_msgs::Clock>("clock", 1);

  ros::WallRate rate(400.0);  // 400% speed
  ros::WallTime time = ros::WallTime::now();
  while (ros::ok()) {
    rosgraph_msgs::Clock clock;
    clock.clock.fromNSec(time.toNSec());
    pub.publish(clock);
    rate.sleep();
    time += ros::WallDuration(0.01);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_trajectory_tracker");

  boost::thread time_thread(timeSource);

  return RUN_ALL_TESTS();
}
