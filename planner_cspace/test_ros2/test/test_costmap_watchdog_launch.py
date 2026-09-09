"""
launch_testing description for the planner_3d costmap watchdog test.

ROS 2 port of the ROS 1 costmap_watchdog_rostest.test.

Two things differ from the ROS 1 launch file:

* map_server is replaced by the ros2_test_map_publisher helper of this package.
  nav2_map_server's map_server is a lifecycle node which never activates by
  itself, so every test description would have to drive its transitions and
  race the nodes waiting for the map. The helper reuses nav2_map_server's
  map_io library, so the same YAML/PGM files are served, latched
  (transient_local) exactly like the ROS 1 node did.
* costmap_3d takes the footprint as a flat [x0, y0, x1, y1, ...] array; with no
  "layers" parameter it stays in the single-layer backward compatible mode the
  ROS 1 launch file used.
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


def global_map_yaml():
    return os.path.join(
        get_package_share_directory('planner_cspace'), 'test', 'data', 'global_map.yaml')


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    costmap_3d = Node(
        package='costmap_cspace',
        executable='costmap_3d',
        name='costmap_3d',
        output='screen',
        parameters=[{
            'footprint': FOOTPRINT,
            'ang_resolution': 16,
            'linear_expand': 0.1,
            'linear_spread': 0.1,
        }],
    )
    planner_3d = Node(
        package='planner_cspace',
        executable='planner_3d',
        name='planner_3d',
        output='screen',
        parameters=[{
            'max_vel': 0.1,
            'max_ang_vel': 0.3,
            'costmap_watchdog': 0.2,
            'freq': 5.0,
            'sw_wait': 0.0,
        }],
    )
    map_server_global = Node(
        package='planner_cspace',
        executable='ros2_test_map_publisher',
        name='map_server_global',
        output='screen',
        parameters=[{'yaml_filename': global_map_yaml(), 'topic_name': 'map'}],
    )
    dummy_robot = Node(
        package='planner_cspace',
        executable='dummy_robot',
        name='dummy_robot',
        output='screen',
        parameters=[{
            'initial_x': 2.5,
            'initial_y': 0.45,
            'initial_yaw': 3.14,
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
    test_node = Node(
        package='planner_cspace',
        executable='ros2_test_costmap_watchdog',
        name='test_costmap_watchdog',
        output='screen',
    )

    return LaunchDescription([
        costmap_3d,
        planner_3d,
        map_server_global,
        dummy_robot,
        stf1,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestCostmapWatchdogTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestCostmapWatchdogOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
