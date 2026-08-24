"""
launch_testing description for the joystick_interrupt integration test.

Mirrors the ROS 1 joystick_interrupt_rostest.test: it starts the
joystick_interrupt node in the default and omni (cmd_vel -> cmd_vel_omni)
configurations and the joystick_mux node, then runs the
ros2_test_joystick_interrupt gtest binary against them. The gtest binary owns
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
    joystick_interrupt = Node(
        package='joystick_interrupt',
        executable='joystick_interrupt',
        name='joystick_interrupt',
        output='screen',
        parameters=[{
            'interrupt_button': 0,
            'high_speed_button': 1,
            'linear_axis': 0,
            'angular_axis': 1,
            'linear_axis2': 2,
            'angular_axis2': 3,
            'linear_vel': 1.0,
            'angular_vel': 1.0,
            'linear_high_speed_ratio': 2.0,
            'angular_high_speed_ratio': 2.0,
        }],
    )
    joystick_interrupt_omni = Node(
        package='joystick_interrupt',
        executable='joystick_interrupt',
        name='joystick_interrupt_omni',
        output='screen',
        parameters=[{
            'interrupt_button': 0,
            'high_speed_button': 1,
            'linear_axis': 0,
            'angular_axis': 1,
            'linear_axis2': 2,
            'angular_axis2': 3,
            'linear_vel': 1.0,
            'angular_vel': 1.0,
            'linear_high_speed_ratio': 2.0,
            'angular_high_speed_ratio': 2.0,
            'linear_y_vel': 0.5,
            'linear_y_axis': 4,
            'linear_y_axis2': 5,
        }],
        remappings=[('cmd_vel', 'cmd_vel_omni')],
    )
    joystick_mux = Node(
        package='joystick_interrupt',
        executable='joystick_mux',
        name='joystick_mux',
        output='screen',
        parameters=[{
            'interrupt_button': 0,
        }],
    )

    test_node = Node(
        package='joystick_interrupt',
        executable='ros2_test_joystick_interrupt',
        name='test_joystick_interrupt',
        output='screen',
    )

    return LaunchDescription([
        joystick_interrupt,
        joystick_interrupt_omni,
        joystick_mux,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestJoystickInterruptTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestJoystickInterruptOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
