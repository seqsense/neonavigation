"""
launch_testing description for the trajectory_recorder test.

Mirrors the ROS 1 trajectory_recorder_rostest.test: a trajectory_recorder node
with its default parameters, and the gtest binary that feeds it transforms.
"""

import unittest

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
    trajectory_recorder_node = Node(
        package='trajectory_tracker',
        executable='trajectory_recorder',
        name='trajectory_recorder',
        output='screen',
    )
    test_node = Node(
        package='trajectory_tracker',
        executable='ros2_test_trajectory_recorder',
        name='test_trajectory_recorder',
        output='screen',
    )

    return LaunchDescription([
        trajectory_recorder_node,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestTrajectoryRecorderTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestTrajectoryRecorderOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
