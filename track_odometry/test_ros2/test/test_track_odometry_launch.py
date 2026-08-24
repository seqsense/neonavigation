"""
launch_testing description for the track_odometry integration test.

Mirrors the ROS 1 track_odometry_rostest.test: it starts one track_odometry
node per configuration (global, plus the no_z_filter / z_filter namespaces in
both the new z_filter_timeconst form and the deprecated z_filter form) and runs
the ros2_test_track_odometry gtest binary against them. The gtest binary owns
its rclcpp context (calls rclcpp::init in main), so it is started as a plain
node and its exit code is checked after shutdown.
"""

import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


def track_odometry_node(name, namespace, extra_params):
    params = {'base_link_id': 'base_link', 'use_kf': False}
    params.update(extra_params)
    return Node(
        package='track_odometry',
        executable='track_odometry',
        name=name,
        namespace=namespace,
        output='screen',
        parameters=[params],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    nodes = [
        track_odometry_node('track_odometry', '/', {'z_filter_timeconst': -1.0}),
        track_odometry_node(
            'track_odometry_no_z_filter', '/no_z_filter', {'z_filter_timeconst': -1.0}),
        track_odometry_node(
            'track_odometry_no_z_filter', '/no_z_filter_old_param', {'z_filter': 1.0}),
        track_odometry_node(
            'track_odometry_z_filter', '/z_filter', {'z_filter_timeconst': 2.0}),
        track_odometry_node(
            'track_odometry_z_filter', '/z_filter_old_param', {'z_filter': 0.995}),
    ]

    test_node = Node(
        package='track_odometry',
        executable='ros2_test_track_odometry',
        name='test_track_odometry',
        output='screen',
    )

    return LaunchDescription(
        nodes + [test_node, launch_testing.actions.ReadyToTest()]), {'test_node': test_node}


class TestTrackOdometryTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=180.0)


@launch_testing.post_shutdown_test()
class TestTrackOdometryOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
