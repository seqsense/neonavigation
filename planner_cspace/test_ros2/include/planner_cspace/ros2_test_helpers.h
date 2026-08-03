/*
 * Copyright (c) 2025, the neonavigation authors
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

#ifndef PLANNER_CSPACE__ROS2_TEST_HELPERS_H_
#define PLANNER_CSPACE__ROS2_TEST_HELPERS_H_

#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace planner_cspace_testing
{
// QoS used by every latched (transient_local) topic of the hybridized nodes.
// Both ends have to request it: a VOLATILE publisher and a TRANSIENT_LOCAL
// subscription are incompatible and silently never connect.
inline rclcpp::QoS latchedQos(const size_t depth = 1)
{
  return rclcpp::QoS(depth).transient_local();
}

// Spins `node` until `pred` becomes true or `timeout` elapses. `on_spin` is
// called on every iteration, which is where a test re-publishes its inputs.
//
// Wall time is used on purpose: the tests below drive real nodes over DDS and
// must make progress even before any /clock arrives.
inline bool spinUntil(
  const rclcpp::Node::SharedPtr & node, const std::chrono::nanoseconds timeout,
  const std::function<bool()> & pred,
  const std::function<void()> & on_spin = std::function<void()>(),
  const std::chrono::nanoseconds interval = std::chrono::milliseconds(50))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok()) {
    if (on_spin) {
      on_spin();
    }
    rclcpp::spin_some(node);
    if (pred()) {
      return true;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    rclcpp::sleep_for(interval);
  }
  return false;
}
}  // namespace planner_cspace_testing

#endif  // PLANNER_CSPACE__ROS2_TEST_HELPERS_H_
