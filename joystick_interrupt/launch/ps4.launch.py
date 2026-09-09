# Copyright (c) 2015-2018, the neonavigation authors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
ROS 2 port of ps4.launch (PS4 controller -> joystick_interrupt).

Starts the joystick driver (``joy`` package) and the ``joystick_interrupt``
node. Following the hybrid kit convention the nodes are loaded into a
multi-threaded component container so that the joy -> joystick_interrupt path
can use intra-process (zero-copy) transport. Set ``use_composition:=false`` to
run them as standalone processes instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_composition = LaunchConfiguration('use_composition')
    use_intra = LaunchConfiguration('use_intra_process_comms')
    device_id = LaunchConfiguration('device_id')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    cmd_vel_in = LaunchConfiguration('cmd_vel_in')
    linear_vel = LaunchConfiguration('linear_vel')
    angular_vel = LaunchConfiguration('angular_vel')

    declare_args = [
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            description='Use component container (true) or standalone nodes (false)',
        ),
        DeclareLaunchArgument(
            'use_intra_process_comms',
            default_value='true',
            description='Enable intra-process communication (zero-copy when true)',
        ),
        DeclareLaunchArgument(
            'device_id',
            default_value='1',
            description=(
                'Joystick device index for the ROS 2 joy driver. '
                'ROS 1 ps4.launch used dev=/dev/input/js1, i.e. device index 1.'
            ),
        ),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='cmd_vel',
            description='Output cmd_vel topic (remap target of joystick_interrupt cmd_vel)',
        ),
        DeclareLaunchArgument(
            'cmd_vel_in',
            default_value='cmd_vel_in',
            description='Input cmd_vel topic (remap target of joystick_interrupt cmd_vel_input)',
        ),
        DeclareLaunchArgument(
            'linear_vel',
            default_value='0.5',
            description='Manual linear velocity while the interrupt button is held [m/s]',
        ),
        DeclareLaunchArgument(
            'angular_vel',
            default_value='0.8',
            description='Manual angular velocity while the interrupt button is held [rad/s]',
        ),
    ]

    # In ROS 1 both joy_node and joystick_interrupt remapped /joy to this topic.
    joy_topic = '/joystick_interrupt/joy'

    # joy driver parameters (ROS 1 <param name="dev"> -> ROS 2 device_id).
    joy_parameters = [{
        'device_id': ParameterValue(device_id, value_type=int),
    }]
    joy_remappings = [('joy', joy_topic)]

    # joystick_interrupt parameters, carried over verbatim from ps4.launch.
    # linear_vel / angular_vel come from launch arguments; force the double type
    # so the string substitution is not interpreted as a string parameter.
    interrupt_parameters = [{
        'interrupt_button': 6,
        'high_speed_button': -1,
        'linear_axis': 1,
        'angular_axis': 0,
        'linear_axis2': 10,
        'angular_axis2': 9,
        'linear_vel': ParameterValue(linear_vel, value_type=float),
        'angular_vel': ParameterValue(angular_vel, value_type=float),
        'linear_high_speed_ratio': 1.3,
        'angular_high_speed_ratio': 1.1,
    }]
    interrupt_remappings = [
        ('joy', joy_topic),
        ('cmd_vel', cmd_vel_out),
        ('cmd_vel_input', cmd_vel_in),
    ]

    # --- Composition mode (default): zero-copy joy -> joystick_interrupt ---
    load_composable = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            ComposableNodeContainer(
                name='joystick_interrupt_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='joy',
                        plugin='joy::Joy',
                        name='joystick',
                        parameters=joy_parameters,
                        remappings=joy_remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra}],
                    ),
                    ComposableNode(
                        package='joystick_interrupt',
                        plugin='joystick_interrupt::JoystickInterrupt',
                        name='joystick_interrupt',
                        parameters=interrupt_parameters,
                        remappings=interrupt_remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra}],
                    ),
                ],
                output='screen',
            ),
        ],
    )

    # --- Standalone mode (fallback): separate processes ---
    load_standalone = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joystick',
                parameters=joy_parameters,
                remappings=joy_remappings,
                output='screen',
            ),
            Node(
                package='joystick_interrupt',
                executable='joystick_interrupt_node',
                name='joystick_interrupt',
                parameters=interrupt_parameters,
                remappings=interrupt_remappings,
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        *declare_args,
        load_composable,
        load_standalone,
    ])
