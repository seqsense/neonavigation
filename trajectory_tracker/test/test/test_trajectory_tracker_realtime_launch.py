"""
launch_testing description for the wall-clock trajectory_tracker tests.

Drives either the overshoot or the with_odom gtest binary (selected via the
``test_executable`` argument) against a trajectory_tracker node configured from
test_overshoot_params.yaml. These runs mirror the ROS 1
trajectory_tracker_overshoot_rostest.test / trajectory_tracker_with_odom_rostest.test
and use real time (no /clock source).
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    test_executable = LaunchConfiguration(
        'test_executable', default='ros2_test_trajectory_tracker_overshoot')

    params_file = os.path.join(
        get_package_share_directory('trajectory_tracker'),
        'test', 'configs', 'test_overshoot_params.yaml')

    trajectory_tracker_node = Node(
        package='trajectory_tracker',
        executable='trajectory_tracker',
        name='trajectory_tracker',
        output='screen',
        parameters=[params_file],
    )
    test_node = Node(
        package='trajectory_tracker',
        executable=test_executable,
        name='test_trajectory_tracker',
        output='screen',
    )

    return LaunchDescription([
        trajectory_tracker_node,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestTrajectoryTrackerTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestTrajectoryTrackerOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
