"""
launch_testing description for the safety_limiter margin integration test.

Mirrors the ROS 1 safety_limiter2_rostest.test: it launches the safety_limiter
node with the d_margin test parameters and runs the ros2_test_safety_limiter2
gtest binary against it.
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
        'test', 'configs', 'test_params2.yaml')

    safety_limiter_node = Node(
        package='safety_limiter',
        executable='safety_limiter',
        name='safety_limiter',
        output='screen',
        parameters=[params_file],
    )
    test_node = Node(
        package='safety_limiter',
        executable='ros2_test_safety_limiter2',
        name='test_safety_limiter2',
        output='screen',
    )

    return LaunchDescription([
        safety_limiter_node,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestSafetyLimiter2Terminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestSafetyLimiter2Outcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
