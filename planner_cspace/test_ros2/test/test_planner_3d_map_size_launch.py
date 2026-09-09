"""
launch_testing description for the planner_3d map size test.

ROS 2 port of the ROS 1 planner_3d_map_size_rostest.test: a bare planner_3d
node with default parameters, fed by the gtest binary which publishes both the
costmap and the ill-sized costmap updates itself. No map_server and no TF are
needed because the test never expects a plan to be produced.
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
    )
    test_node = Node(
        package='planner_cspace',
        executable='ros2_test_planner_3d_map_size',
        name='test_planner_3d_map_size',
        output='screen',
    )

    return LaunchDescription([
        planner_3d,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestPlanner3dMapSizeTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestPlanner3dMapSizeOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
