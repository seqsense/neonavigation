"""
launch_testing description for the planner_2dof_serial_joints test.

ROS 2 port of the ROS 1 planner_2dof_serial_joints_rostest.test. The ROS 1
node read the per-group settings from the nested private namespace
`~/group0/<key>`; ROS 2 parameters are flat, so the very same settings are
given here as dot separated `group0.<key>` names (launch_ros passes keys
containing dots through unchanged, they do not have to be nested dicts).
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
    planner = Node(
        package='planner_cspace',
        executable='planner_2dof_serial_joints',
        name='planner_2dof_serial_joints',
        output='screen',
        parameters=[{
            'num_groups': 1,
            'replan_interval': 1.0,

            'group0.link0_name': 'front',
            'group0.link1_name': 'rear',
            'group0.resolution': 128,
            'group0.weight_cost': 4.0,
            'group0.expand': 0.15,

            'group0.link0_joint_radius': 0.5,
            'group0.link1_joint_radius': 0.5,
            'group0.link0_end_radius': 0.5,
            'group0.link1_end_radius': 0.5,
            'group0.link0_x': 1.0,
            'group0.link1_x': -1.0,
            'group0.link0_y': 0.0,
            'group0.link1_y': 0.0,
            'group0.link0_th': 0.0,
            'group0.link1_th': 0.0,
            'group0.link0_gain_th': -1.0,
            'group0.link1_gain_th': 1.0,
            'group0.link0_length': 1.0,
            'group0.link1_length': 1.0,
        }],
    )
    test_node = Node(
        package='planner_cspace',
        executable='ros2_test_planner_2dof_serial_joints',
        name='test_planner_2dof_serial_joints',
        output='screen',
    )

    return LaunchDescription([
        planner,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestPlanner2dofSerialJointsTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestPlanner2dofSerialJointsOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
