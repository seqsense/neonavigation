"""
launch_testing description for the planner_3d dynamic parameter change test.

ROS 2 port of the ROS 1 dynamic_parameter_change_rostest.test. The ROS 1 test
reconfigured planner_3d through dynamic_reconfigure; on ROS 2 the very same
values are plain node parameters, set through rclcpp::AsyncParametersClient,
so the launch description only has to start the nodes.
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

FOOTPRINT = [0.01, -0.01, 0.01, 0.01, -0.01, 0.01, -0.01, -0.01]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    global_map_yaml = os.path.join(
        get_package_share_directory('planner_cspace'), 'test', 'data', 'global_map.yaml')

    costmap_3d = Node(
        package='costmap_cspace',
        executable='costmap_3d',
        name='costmap_3d',
        output='screen',
        parameters=[{
            'footprint': FOOTPRINT,
            'ang_resolution': 16,
            'linear_expand': 0.0,
            'linear_spread': 0.3,
        }],
    )
    planner_3d = Node(
        package='planner_cspace',
        executable='planner_3d',
        name='planner_3d',
        output='screen',
    )
    map_server_global = Node(
        package='planner_cspace',
        executable='ros2_test_map_publisher',
        name='map_server_global',
        output='screen',
        parameters=[{'yaml_filename': global_map_yaml, 'topic_name': 'map'}],
    )
    stf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf1',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
    )
    test_node = Node(
        package='planner_cspace',
        executable='ros2_test_dynamic_parameter_change',
        name='test_dynamic_parameter_change',
        output='screen',
    )

    return LaunchDescription([
        costmap_3d,
        planner_3d,
        map_server_global,
        stf1,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestDynamicParameterChangeTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=300.0)


@launch_testing.post_shutdown_test()
class TestDynamicParameterChangeOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
