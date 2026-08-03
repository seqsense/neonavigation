"""
Shared launch actions for the action-based planner_3d tests.

ROS 2 counterpart of test/test/actionlib_common_rostest.test, which the ROS 1
abort / preempt / tolerant_action rostests pulled in with <include>. rostest
XML fragments have no launch_testing equivalent, so the node set is factored
out into this module and imported by the three launch descriptions.

Importing it requires the test's own directory on sys.path; every launch test
using it does

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

because launch_testing loads the description file by path, without adding its
directory to sys.path.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

FOOTPRINT = [0.2, -0.1, 0.2, 0.1, -0.2, 0.1, -0.2, -0.1]


def global_map_yaml():
    return os.path.join(
        get_package_share_directory('planner_cspace'), 'test', 'data', 'global_map.yaml')


def common_nodes():
    """Return the node set shared by the abort/preempt/tolerant_action tests."""
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
            'max_retry_num': 5,
            'tolerance_range': 0.0,
            'temporary_escape': False,
            'goal_tolerance_lin': 0.05,
        }],
    )
    # `spur` is the ROS 1 node name of the trajectory_tracker instance.
    spur = Node(
        package='trajectory_tracker',
        executable='trajectory_tracker',
        name='spur',
        output='screen',
        parameters=[{
            'max_vel': 0.3,
            'max_acc': 0.3,
            'max_angvel': 0.4,
            'max_angacc': 0.4,

            'curv_forward': 0.1,
            'look_forward': 0.0,
            'k_dist': 4.5,
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
        parameters=[{'yaml_filename': global_map_yaml(), 'topic_name': 'map'}],
    )
    patrol = Node(
        package='planner_cspace',
        executable='patrol',
        name='patrol',
        output='screen',
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
    return [costmap_3d, planner_3d, spur, map_server_global, patrol, dummy_robot, stf1]
