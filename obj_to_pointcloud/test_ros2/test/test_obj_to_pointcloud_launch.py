"""
launch_testing description for the obj_to_pointcloud integration test.

Mirrors the ROS 1 obj_to_pointcloud_rostest.test: it starts the
obj_to_pointcloud node with the sample OBJ (scaled by 0.001) and runs the
ros2_test_obj_to_pointcloud gtest binary, which subscribes to the latched
"mapcloud" topic and validates the generated point cloud. The gtest binary owns
its rclcpp context (calls rclcpp::init in main), so it is started as a plain
node and its exit code is checked after shutdown.
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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    obj_path = os.path.join(
        get_package_share_directory('obj_to_pointcloud'), 'test', 'data', 'sample.obj')

    node_under_test = Node(
        package='obj_to_pointcloud',
        executable='obj_to_pointcloud',
        name='obj_to_pointcloud',
        output='screen',
        parameters=[{
            'objs': obj_path,
            'scale': 0.001,
        }],
    )

    test_node = Node(
        package='obj_to_pointcloud',
        executable='ros2_test_obj_to_pointcloud',
        name='test_obj_to_pointcloud',
        output='screen',
    )

    return LaunchDescription(
        [node_under_test, test_node, launch_testing.actions.ReadyToTest()]), \
        {'test_node': test_node}


class TestObjToPointcloudTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=60.0)


@launch_testing.post_shutdown_test()
class TestObjToPointcloudOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
