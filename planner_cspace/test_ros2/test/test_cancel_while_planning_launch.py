"""
launch_testing description for the planner_3d cancel-while-planning test.

A bare planner_3d node fed by the gtest binary, which publishes the costmap
and the robot pose (static TF) itself. The parameters make each search long
and observable; see ros2_test_cancel_while_planning.cpp.
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
    planner_3d = Node(
        package='planner_cspace',
        executable='planner_3d',
        name='planner_3d',
        output='screen',
        parameters=[{
            # Search the whole map in 3D.
            'local_range': 100.0,
            # Log "Search timed out" every 0.2 seconds while searching.
            'freq_min': 5.0,
            'max_retry_num': -1,
            'temporary_escape': False,
            'tolerance_range': 0.0,
        }],
    )
    test_node = Node(
        package='planner_cspace',
        executable='ros2_test_cancel_while_planning',
        name='test_cancel_while_planning',
        output='screen',
    )

    return LaunchDescription([
        planner_3d,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestCancelWhilePlanningTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestCancelWhilePlanningOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
