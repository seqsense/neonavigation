"""
launch_testing description for the planner_3d debug output test.

ROS 2 port of the ROS 1 debug_outputs_rostest.test.

Differences to the ROS 1 launch file:

* map_server is replaced by the ros2_test_map_publisher helper (see
  test_costmap_watchdog_launch.py for the rationale).
* The layer chain of costmap_3d is given in the flattened ROS 2 form.
* The ROS 1 file held the goal with a `rostopic pub -l` process reading
  test/data/goal_debug_outputs.yaml; the goal is published by the gtest binary
  instead, so no extra process is needed.
* The ROS 1 file also set `remember_hit_miss`, which is not a planner_3d
  parameter (the real name is `remember_miss_prob`) and was silently dropped
  by ROS 1 as well, so it is not carried over.
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

FOOTPRINT = [0.2, -0.1, 0.2, 0.1, -0.2, 0.1, -0.2, -0.1]


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
            'linear_expand': 0.1,
            'linear_spread': 0.0,
            'static_layers': ['unknown'],
            'static_layer.unknown.type': 'Costmap3dLayerUnknownHandle',
            'static_layer.unknown.unknown_cost': 100,
            'layers': ['overlay'],
            'layer.overlay.type': 'Costmap3dLayerFootprint',
            'layer.overlay.overlay_mode': 'max',
        }],
    )
    planner_3d = Node(
        package='planner_cspace',
        executable='planner_3d',
        name='planner_3d',
        output='screen',
        parameters=[{
            'remember_updates': True,
            'remember_hit_prob': 0.99,
            'hist_ignore_range': 0.0,
            'hist_ignore_range_max': 1.0,
            'pos_jump': 100.0,
            'yaw_jump': 100.0,
        }],
    )
    map_server_global = Node(
        package='planner_cspace',
        executable='ros2_test_map_publisher',
        name='map_server_global',
        output='screen',
        parameters=[{'yaml_filename': global_map_yaml, 'topic_name': 'map'}],
    )
    largemap_to_map = Node(
        package='costmap_cspace',
        executable='largemap_to_map',
        name='largemap_to_map',
        output='screen',
        remappings=[('map_local', 'overlay')],
        parameters=[{
            'width': 15,
            'hz': 5.0,
        }],
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
    stf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf2',
        arguments=[
            '--x', '2.5', '--y', '0.5', '--z', '0',
            '--yaw', '3.14', '--pitch', '0', '--roll', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_link',
        ],
    )
    test_node = Node(
        package='planner_cspace',
        executable='ros2_test_debug_outputs',
        name='test_debug_outputs',
        output='screen',
    )

    return LaunchDescription([
        costmap_3d,
        planner_3d,
        map_server_global,
        largemap_to_map,
        stf1,
        stf2,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestDebugOutputsTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=180.0)


@launch_testing.post_shutdown_test()
class TestDebugOutputsOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
