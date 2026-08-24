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

#include <algorithm>
#include <string>
#include <vector>

#include "trajectory_tracker_test_ros2.h"

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

// The simulated clock is published by a separate process at a fixed wall rate,
// so it keeps advancing even when the machine is busy and the tracker misses
// control cycles. The budgets below only exist to catch a hang, so they are
// scaled well past the nominal motion time; a loaded CI runner otherwise fails
// them while the robot is still converging.
constexpr double kTimeoutScale = 2.0;

class RosRate : public rclcpp::RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RosRate)

  explicit RosRate(double rate, rclcpp::Node & node)
  : RosRate(duration_cast<nanoseconds>(duration<double>(1.0 / rate)), node)
  {
  }
  explicit RosRate(std::chrono::nanoseconds period, rclcpp::Node & node)
  : node_(node.get_node_base_interface()), clock_(node.get_clock()), period_(period)
  {
    last_interval_ = clock_->now();
  }

  virtual bool sleep()
  {
    // Time coming into sleep
    auto now = clock_->now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_) {
      // Best thing to do is to set the next_interval to now + period
      next_interval = now + period_;
    }
    // Calculate the time to sleep
    auto time_to_sleep = next_interval - now;
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (time_to_sleep <= std::chrono::seconds(0)) {
      // If an entire cycle was missed then reset next interval.
      // This might happen if the loop took more than a cycle.
      // Or if time jumps forward.
      if (now > next_interval + period_) {
        last_interval_ = now + period_;
      }
      // Either way do not sleep and return false
      return false;
    }
    // Sleep (will get interrupted by ctrl-c, may not sleep full time)
    while (clock_->now() < next_interval) {
      rclcpp::spin_some(node_);
      clock_->sleep_for(std::chrono::milliseconds(1));
    }
    return true;
  }

  virtual bool is_steady() const { return false; }

  virtual rcl_clock_type_t get_type() const { return clock_->get_clock_type(); }

  virtual void reset() { last_interval_ = clock_->now(); }

  std::chrono::nanoseconds period() const
  {
    return std::chrono::nanoseconds(period_.nanoseconds());
  }

private:
  RCLCPP_DISABLE_COPY(RosRate)

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
};

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

  RosRate rate(50, *this);
  const rclcpp::Time start = now();
  while (rclcpp::ok()) {
    if (now() > start + rclcpp::Duration::from_seconds(kTimeoutScale * 10.0)) {
      FAIL() << "Timeout" << std::endl
             << "Pos " << pos_ << std::endl
             << "Yaw " << yaw_ << std::endl
             << "Status " << std::endl
             << status_ << std::endl;
    }
    publishTransform();
    rate.sleep();
    rclcpp::spin_some(get_node_base_interface());
    if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL) break;
  }
  for (int j = 0; j < 5; ++j) {
    for (int i = 0; i < 5; ++i) {
      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
    }

    // Check multiple times to assert overshoot.
    ASSERT_NEAR(yaw_, 0.0, error_ang_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[0], 0.5, error_lin_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[1], 0.0, error_lin_) << "[overshoot after goal (" << j << ")] ";
  }
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
}

// The remaining scenarios - overshoot behaviour, velocity changes, curves,
// in-place turns, switchbacks, far-away paths - are covered without a node by
// test_trajectory_tracker_scenarios, which drives the same controller through
// the same paths against a virtual clock. Running them here as well only added
// runtime and, because simulated time keeps advancing whether or not the node
// gets scheduled, the occasional false failure.

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  // Not optional: musl keeps the context's globals alive until process exit,
  // where they are torn down in an order that crashes. Returning from main
  // with the context still initialized segfaults after every test has passed.
  rclcpp::shutdown();
  return ret;
}
