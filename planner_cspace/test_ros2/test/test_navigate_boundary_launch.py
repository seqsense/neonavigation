"""
launch_testing description for the planner_3d start position boundary test.

ROS 2 port of the ROS 1 navigation_boundary_rostest.test. planner_3d_debug is
the interface node compiled with -DDEBUG so that the grid map assert()s stay
enabled; the two map_server instances are replaced by the
ros2_test_map_publisher helper (see test_costmap_watchdog_launch.py).
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
    data_dir = os.path.join(
        get_package_share_directory('planner_cspace'), 'test', 'data')

    costmap_3d = Node(
        package='costmap_cspace',
        executable='costmap_3d',
        name='costmap_3d',
        output='screen',
        parameters=[{
            'footprint': FOOTPRINT,
            'ang_resolution': 16,
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
        executable='planner_3d_debug',
        name='planner_3d',
        output='screen',
        parameters=[{
            'freq': 10.0,
            'local_range': 0.2,
            'antialias_start': True,
            'enable_crowd_mode': True,
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
    map_server_global = Node(
        package='planner_cspace',
        executable='ros2_test_map_publisher',
        name='map_server_global',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(data_dir, 'global_map.yaml'),
            'topic_name': 'map',
        }],
    )
    map_server_local = Node(
        package='planner_cspace',
        executable='ros2_test_map_publisher',
        name='map_server_local',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(data_dir, 'local_map.yaml'),
            'topic_name': 'map_with_local_objects',
        }],
    )
    largemap_to_map = Node(
        package='costmap_cspace',
        executable='largemap_to_map',
        name='largemap_to_map',
        output='screen',
        remappings=[
            ('map', 'map_with_local_objects'),
            ('map_local', 'overlay'),
        ],
        parameters=[{
            'width': 6,
            'hz': 5.0,
        }],
    )
    test_node = Node(
        package='planner_cspace',
        executable='ros2_test_navigate_boundary',
        name='test_navigate_boundary',
        output='screen',
    )

    return LaunchDescription([
        costmap_3d,
        planner_3d,
        stf1,
        map_server_global,
        map_server_local,
        largemap_to_map,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestNavigateBoundaryTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=240.0)


@launch_testing.post_shutdown_test()
class TestNavigateBoundaryOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
