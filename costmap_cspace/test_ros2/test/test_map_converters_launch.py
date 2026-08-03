"""
launch_testing description for the map converter integration test.

The ROS 1 package had no rostest for laserscan_to_map, pointcloud2_to_map and
largemap_to_map, so this test covers the main pub/sub path of their ROS 2
ports. The three nodes are started side by side with their input/output topics
remapped to per-node names, together with a static map -> base_link transform
placing the robot at the origin. The ros2_test_map_converters gtest binary owns
its rclcpp context (calls rclcpp::init in main), so it is started as a plain
node and its exit code is checked after shutdown.
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
    laserscan_to_map = Node(
        package='costmap_cspace',
        executable='laserscan_to_map',
        name='laserscan_to_map',
        output='screen',
        parameters=[{
            'width': 8,
            'resolution': 1.0,
            'hz': 100.0,
            'z_min': -1.0,
            'z_max': 1.0,
            'accum_duration': 1.0,
            'global_frame': 'map',
            'robot_frame': 'base_link',
        }],
        remappings=[
            ('scan', '/test_scan'),
            ('map_local', '/laserscan_map_local'),
        ],
    )
    pointcloud2_to_map = Node(
        package='costmap_cspace',
        executable='pointcloud2_to_map',
        name='pointcloud2_to_map',
        output='screen',
        parameters=[{
            'width': 8,
            'resolution': 1.0,
            'hz': 100.0,
            'z_min': -1.0,
            'z_max': 1.0,
            'accum_duration': 1.0,
            'global_frame': 'map',
            'robot_frame': 'base_link',
        }],
        remappings=[
            ('cloud', '/test_cloud'),
            ('map_local', '/pointcloud_map_local'),
        ],
    )
    largemap_to_map = Node(
        package='costmap_cspace',
        executable='largemap_to_map',
        name='largemap_to_map',
        output='screen',
        parameters=[{
            'width': 4,
            'hz': 10.0,
            'robot_frame': 'base_link',
        }],
        remappings=[
            ('map', '/large_map'),
            ('map_local', '/largemap_map_local'),
        ],
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='test_static_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'base_link',
        ],
    )

    test_node = Node(
        package='costmap_cspace',
        executable='ros2_test_map_converters',
        name='test_map_converters',
        output='screen',
    )

    return LaunchDescription([
        laserscan_to_map,
        pointcloud2_to_map,
        largemap_to_map,
        static_tf,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestMapConvertersTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestMapConvertersOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
