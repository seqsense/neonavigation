"""
launch_testing description for the safety_limiter integration tests.

Mirrors the ROS 1 safety_limiter_rostest.test: it launches the safety_limiter
node with the test parameters and runs the ros2_test_safety_limiter gtest binary
against it. The gtest binary owns its rclcpp context (calls rclcpp::init in
main), so it is started as a plain node and its exit code is checked after
shutdown.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    params_file = os.path.join(
        get_package_share_directory('safety_limiter'),
        'test', 'configs', 'test_params.yaml')

    safety_limiter_node = Node(
        package='safety_limiter',
        executable='safety_limiter',
        name='safety_limiter',
        output='screen',
        parameters=[params_file],
    )
    test_node = Node(
        package='safety_limiter',
        executable='ros2_test_safety_limiter',
        name='test_safety_limiter',
        output='screen',
    )

    return LaunchDescription([
        safety_limiter_node,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestSafetyLimiterTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=160.0)


@launch_testing.post_shutdown_test()
class TestSafetyLimiterOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
