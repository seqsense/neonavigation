"""
launch_testing description for the tf_projection integration test.

Mirrors the ROS 1 tf_projection_rostest.test: it starts the tf_projection nodes
(old- and new-parameter forms, with/without posture projection) together with
two static_transform_publisher processes providing map->base_link and
map->base_link_tilt, then runs the ros2_test_tf_projection_node gtest binary,
which looks up the projected frames. The gtest binary owns its rclcpp context
(calls rclcpp::init in main), so it is started as a plain node and its exit code
is checked after shutdown.
"""

import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


def tf_projection_node(name, params):
    return Node(
        package='track_odometry',
        executable='tf_projection_node_exec',
        name=name,
        output='screen',
        parameters=[params],
    )


def static_tf(name, x, y, z, qx, qy, qz, qw, frame_id, child_frame_id):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--qx', qx, '--qy', qy, '--qz', qz, '--qw', qw,
            '--frame-id', frame_id, '--child-frame-id', child_frame_id,
        ],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    nodes = [
        # Deprecated parameter names (base_link_frame / projection_frame /
        # target_frame / frame) still accepted by the node.
        tf_projection_node('tf_projection', {
            'base_link_frame': 'base_link',
            'projection_frame': 'map',
            'target_frame': 'map',
            'frame': 'base_link_projected',
            'project_posture': False,
        }),
        tf_projection_node('tf_projection_new_param', {
            'source_frame': 'base_link',
            'projection_surface_frame': 'map',
            'parent_frame': 'map',
            'projected_frame': 'base_link_projected2',
            'project_posture': False,
        }),
        tf_projection_node('tf_projection_project_posture', {
            'source_frame': 'base_link_tilt',
            'projection_surface_frame': 'map',
            'parent_frame': 'map',
            'projected_frame': 'base_link_tilt_projected',
            'project_posture': True,
            'align_all_posture_to_source': False,
        }),
        tf_projection_node('tf_projection_project_posture_align_all_posture', {
            'source_frame': 'base_link_tilt',
            'projection_surface_frame': 'map',
            'parent_frame': 'map',
            'projected_frame': 'base_link_tilt_projected_with_aligned',
            'project_posture': True,
            'align_all_posture_to_source': True,
        }),
        static_tf(
            'test_static_pub', '1', '2', '3', '0', '0', '0.7071', '0.7071',
            'map', 'base_link'),
        static_tf(
            'test_static_pub2', '1', '2', '3', '0.1830', '0.1830', '0.6830', '0.6830',
            'map', 'base_link_tilt'),
    ]

    test_node = Node(
        package='track_odometry',
        executable='ros2_test_tf_projection_node',
        name='test_tf_projection_node',
        output='screen',
    )

    return LaunchDescription(
        nodes + [test_node, launch_testing.actions.ReadyToTest()]), {'test_node': test_node}


class TestTfProjectionTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestTfProjectionOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
