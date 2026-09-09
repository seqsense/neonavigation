"""
launch_testing description for the planner_3d navigation integration test.

ROS 2 port of the ROS 1 navigation_rostest.test. That file was registered five
times by add_rostest with different arguments; here the same matrix is covered
by five add_launch_test() calls passing the same four launch arguments
(antialias_start / fast_map_update / with_tolerance / enable_crowd_mode).

map_server is replaced by the ros2_test_map_publisher helper (see
test_costmap_watchdog_launch.py for the rationale) and the costmap_3d layer
chain is given in the flattened ROS 2 form.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest

FOOTPRINT = [0.2, -0.1, 0.2, 0.1, -0.2, 0.1, -0.2, -0.1]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    antialias_start = ParameterValue(
        LaunchConfiguration('antialias_start', default='false'), value_type=bool)
    fast_map_update = ParameterValue(
        LaunchConfiguration('fast_map_update', default='false'), value_type=bool)
    with_tolerance = ParameterValue(
        LaunchConfiguration('with_tolerance', default='false'), value_type=bool)
    enable_crowd_mode = ParameterValue(
        LaunchConfiguration('enable_crowd_mode', default='false'), value_type=bool)

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
            'linear_expand': 0.1,
            'linear_spread': 0.1,
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
            'max_vel': 0.1,
            'max_ang_vel': 0.3,
            'goal_tolerance_lin': 0.025,
            'antialias_start': antialias_start,
            'fast_map_update': fast_map_update,
            'enable_crowd_mode': enable_crowd_mode,
            'temporary_escape': enable_crowd_mode,
            'retain_last_error_status': False,
        }],
    )
    spur = Node(
        package='trajectory_tracker',
        executable='trajectory_tracker',
        name='spur',
        output='screen',
        parameters=[{
            'max_vel': 0.1,
            'max_acc': 0.4,
            'max_angvel': 0.3,
            'max_angacc': 1.0,

            'curv_forward': 0.1,
            'look_forward': 0.0,
            'k_dist': 5.5,
            'k_ang': 3.0,
            'k_avel': 4.0,

            'path_step': 1,

            'limit_vel_by_avel': True,

            'hz': 30.0,
            'dist_lim': 0.5,

            'rotate_ang': 0.2,

            'goal_tolerance_dist': 0.025,
            'goal_tolerance_ang': 0.025,
            'stop_tolerance_dist': 0.02,
            'stop_tolerance_ang': 0.02,
            'no_position_control_dist': 0.01,
        }],
    )
    map_server_global = Node(
        package='planner_cspace',
        executable='ros2_test_map_publisher',
        name='map_server_global',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(data_dir, 'global_map.yaml'),
            'topic_name': 'map_global',
        }],
    )
    map_server_local = Node(
        package='planner_cspace',
        executable='ros2_test_map_publisher',
        name='map_server_local',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(data_dir, 'local_map.yaml'),
            'topic_name': 'map_local',
        }],
    )
    patrol = Node(
        package='planner_cspace',
        executable='patrol',
        name='patrol',
        output='screen',
        parameters=[{
            'with_tolerance': with_tolerance,
            'tolerance_lin': 0.1,
            'tolerance_ang': 0.1,
        }],
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
        executable='ros2_test_navigate',
        name='test_navigate',
        output='screen',
        parameters=[{'enable_crowd_mode': enable_crowd_mode}],
    )

    return LaunchDescription([
        costmap_3d,
        planner_3d,
        spur,
        map_server_global,
        map_server_local,
        patrol,
        dummy_robot,
        stf1,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestNavigateTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=600.0)


@launch_testing.post_shutdown_test()
class TestNavigateOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
