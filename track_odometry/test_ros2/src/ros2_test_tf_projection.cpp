/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

// ROS 2 port of the ROS 1 pure gtest test_tf_projection.cpp. It exercises the
// hybridized track_odometry::projectTranslation logic directly, so it needs no
// running node and is built with ament_add_gtest. The only change from the ROS
// 1 source is tf2::TimePointZero in place of ros::Time(0) for the stamp.

#include <gtest/gtest.h>

#include <cmath>

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "track_odometry/tf_projection.h"

void testTransform(
  const tf2::Stamped<tf2::Transform> proj2base, const tf2::Stamped<tf2::Transform> targ2proj,
  const tf2::Stamped<tf2::Transform> truth)
{
  tf2::Transform result = track_odometry::projectTranslation(proj2base, targ2proj);

  const double error_x = std::abs(result.getOrigin().x() - truth.getOrigin().x());
  const double error_y = std::abs(result.getOrigin().y() - truth.getOrigin().y());
  const double error_z = std::abs(result.getOrigin().z() - truth.getOrigin().z());
  const tf2::Quaternion error_q = result.getRotation() * truth.getRotation().inverse();
  ASSERT_LT(error_x, 0.001);
  ASSERT_LT(error_y, 0.001);
  ASSERT_LT(error_z, 0.001);
  ASSERT_LT(std::abs(error_q.getAngle()), 0.001);
}

TEST(TfProjection, ProjectionTransform)
{
  testTransform(
    tf2::Stamped<tf2::Transform>(
      tf2::Transform(
        tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), M_PI / 2.0), tf2::Vector3(1.0, 3.0, 10.0)),
      tf2::TimePointZero, "map"),
    // projected: t(1.0, 3.0, 0.0), r((0.0, 0.0, 1.0), M_PI/2.0)
    tf2::Stamped<tf2::Transform>(
      tf2::Transform(
        tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), M_PI / 2.0), tf2::Vector3(1.0, 0.0, 0.5)),
      tf2::TimePointZero, "odom"),
    // rotate 90deg: (-3.0, 1.0, 0.0), r((0.0, 0.0, 1.0), M_PI)
    // offset x+1.0, z+0.5: t(-2.0, 1.0, 0.5), r((0.0, 0.0, 1.0), M_PI)
    tf2::Stamped<tf2::Transform>(
      tf2::Transform(
        tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), M_PI), tf2::Vector3(-2.0, 1.0, 0.5)),
      tf2::TimePointZero, "odom"));

  testTransform(
    tf2::Stamped<tf2::Transform>(
      tf2::Transform(
        tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 0.0), tf2::Vector3(1.0, 1.0, 100.0)),
      tf2::TimePointZero, "map"),
    // projected: t(1.0, 1.0, 0.0), r((0.0, 0.0, 1.0), 0.0)
    tf2::Stamped<tf2::Transform>(
      tf2::Transform(
        tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), M_PI / 6.0),
        tf2::Vector3(0.0, -std::sqrt(3.0) / 2.0, 3.0 / 2.0)),
      tf2::TimePointZero, "odom"),
    // rotate 30deg and offset to make it on z axis
    tf2::Stamped<tf2::Transform>(
      tf2::Transform(
        tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), M_PI / 6.0), tf2::Vector3(1.0, 0.0, 2.0)),
      tf2::TimePointZero, "odom"));
}
