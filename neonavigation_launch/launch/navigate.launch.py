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
ROS 2 port of navigate.launch.

Starts the neonavigation stack (costmap_3d, planner_3d, trajectory_tracker,
patrol and optionally safety_limiter / map_server / a simulated robot).

Following the hybrid kit convention every neonavigation node is loaded into a
multi-threaded component container so that the costmap -> planner -> tracker
chain can use intra-process (zero-copy) transport. Set ``use_composition:=false``
to run them as standalone processes instead.

Differences from the ROS 1 launch file are documented in
``neonavigation_launch/README.md``. The most important ones are:

* the ``neonavigation_compatible`` parameter is gone; the ROS 2 nodes always use
  the plain (new) topic names,
* the goal topic of ``planner_3d`` is ``goal_pose`` instead of
  ``move_base_simple/goal``,
* ``config/navigate.yaml`` is replaced by ``config/navigate_ros2.yaml``, which
  uses the per-node ``ros__parameters`` layout and the flattened costmap_3d
  layer/footprint representation.

The node list is assembled inside an :class:`OpaqueFunction` because several
ROS 1 ``if=`` / ``unless=`` attributes select individual *parameters* (not whole
nodes), which cannot be expressed with launch conditions alone.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import yaml

# Footprint of the simulated robot, matching the <rosparam param="footprint">
# blocks of navigate.launch. ROS 2 parameters cannot hold nested arrays, so the
# ROS 1 [[x, y], ...] value becomes a flat [x0, y0, x1, y1, ...] double array.
SIMULATED_FOOTPRINT = '[0.35, -0.22, 0.35, 0.22, -0.35, 0.22, -0.35, -0.22]'

DEFAULT_CONTAINER_NAME = 'neonavigation_container'


def declare_arguments() -> list:
    """Return the DeclareLaunchArgument actions shared with demo.launch.py."""
    default_params = os.path.join(
        get_package_share_directory('neonavigation_launch'), 'config', 'navigate_ros2.yaml')

    return [
        # --- arguments of the ROS 1 navigate.launch ---
        DeclareLaunchArgument(
            'map_file', default_value='',
            description='Map YAML passed to nav2_map_server (required if use_map_server)'),
        DeclareLaunchArgument('use_map_server', default_value='true'),
        DeclareLaunchArgument('use_safety_limiter', default_value='false'),
        DeclareLaunchArgument('use_path_with_velocity', default_value='true'),
        DeclareLaunchArgument('enable_crowd_mode', default_value='false'),
        DeclareLaunchArgument('simulate', default_value='false'),
        DeclareLaunchArgument('sim_robot_x', default_value='0.0'),
        DeclareLaunchArgument('sim_robot_y', default_value='0.0'),
        DeclareLaunchArgument('sim_robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('cmd_vel_output', default_value='/cmd_vel'),
        DeclareLaunchArgument('vel', default_value='0.5'),
        DeclareLaunchArgument('acc', default_value='0.3'),
        DeclareLaunchArgument('ang_vel', default_value='0.6'),
        DeclareLaunchArgument('ang_acc', default_value='1.0'),
        DeclareLaunchArgument('look_forward', default_value='0.1'),
        DeclareLaunchArgument('curv_forward', default_value='0.2'),
        DeclareLaunchArgument('slow_and_precise', default_value='true'),
        DeclareLaunchArgument('output_info', default_value='screen'),
        # --- ROS 2 additions ---
        DeclareLaunchArgument(
            'footprint', default_value=SIMULATED_FOOTPRINT,
            description=(
                'Flat [x0, y0, x1, y1, ...] footprint applied to costmap_3d and '
                'safety_limiter when simulate:=true (as in the ROS 1 launch file)')),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='ROS 2 parameter file (ROS 2 form of config/navigate.yaml)'),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='Use component container (true) or standalone nodes (false)'),
        DeclareLaunchArgument(
            'use_intra_process_comms', default_value='true',
            description='Enable intra-process communication (zero-copy when true)'),
        DeclareLaunchArgument(
            'container_name', default_value=DEFAULT_CONTAINER_NAME,
            description='Name of the component container created in composition mode'),
    ]


def as_bool(value: str) -> bool:
    """Interpret a launch argument string as a boolean, like IfCondition does."""
    return value.lower() in ('true', '1', 'yes', 'on', 'enable')


def build_nodes(context) -> list:
    """Return the list of (package, plugin, executable, name, params, remaps) tuples."""
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    map_file = cfg('map_file')
    use_map_server = as_bool(cfg('use_map_server'))
    use_safety_limiter = as_bool(cfg('use_safety_limiter'))
    use_path_with_velocity = as_bool(cfg('use_path_with_velocity'))
    enable_crowd_mode = as_bool(cfg('enable_crowd_mode'))
    simulate = as_bool(cfg('simulate'))
    cmd_vel_output = cfg('cmd_vel_output')
    slow_and_precise = as_bool(cfg('slow_and_precise'))
    params_file = cfg('params_file')
    footprint = [float(v) for v in yaml.safe_load(cfg('footprint'))]

    if use_map_server and not map_file:
        raise RuntimeError(
            'navigate.launch.py: map_file must be given when use_map_server is true')

    nodes = []

    # costmap_3d: the footprint is only overridden in simulation, exactly as the
    # conditional <rosparam param="footprint"> of the ROS 1 launch file.
    costmap_params = [params_file]
    if simulate:
        costmap_params.append({'footprint': footprint})
    nodes.append({
        'package': 'costmap_cspace',
        'plugin': 'costmap_cspace::Costmap3DOFNode',
        'executable': 'costmap_3d',
        'name': 'costmap_3d',
        'parameters': costmap_params,
        'remappings': [],
    })

    if use_map_server:
        # nav2_map_server::MapServer is a managed (lifecycle) node. There is no
        # nav2_lifecycle_manager in this workspace, so let the node bring itself
        # up (autostart_node) and disable the bond it would open to a manager.
        nodes.append({
            'package': 'nav2_map_server',
            'plugin': 'nav2_map_server::MapServer',
            'executable': 'map_server',
            'name': 'map_server',
            'parameters': [{
                'yaml_filename': map_file,
                'autostart_node': True,
                'bond_heartbeat_period': 0.0,
            }],
            'remappings': [],
        })

    nodes.append({
        'package': 'planner_cspace',
        'plugin': 'planner_cspace::planner_3d::Planner3dNode',
        'executable': 'planner_3d',
        'name': 'planner_3d',
        'parameters': [params_file, {
            'use_path_with_velocity': use_path_with_velocity,
            'enable_crowd_mode': enable_crowd_mode,
        }],
        'remappings': [],
    })

    nodes.append({
        'package': 'trajectory_tracker',
        'plugin': 'trajectory_tracker::TrackerNode',
        'executable': 'trajectory_tracker',
        'name': 'spur',
        'parameters': [params_file, {
            'max_vel': float(cfg('vel')),
            'max_acc': float(cfg('acc')),
            'max_angvel': float(cfg('ang_vel')),
            'max_angacc': float(cfg('ang_acc')),
            'curv_forward': float(cfg('curv_forward')),
            'look_forward': float(cfg('look_forward')),
            'rotate_ang': 0.2 if slow_and_precise else 0.4,
        }],
        'remappings': [
            ('cmd_vel', 'cmd_vel_raw' if use_safety_limiter else cmd_vel_output),
        ],
    })

    if use_safety_limiter:
        safety_params = [{
            'allow_empty_cloud': True,
            'watchdog_interval': 0.0,
        }]
        if simulate:
            safety_params.append({'footprint': footprint})
        nodes.append({
            'package': 'safety_limiter',
            'plugin': 'safety_limiter::SafetyLimiterNode',
            'executable': 'safety_limiter',
            'name': 'safety_limiter',
            'parameters': safety_params,
            'remappings': [
                ('cmd_vel_in', 'cmd_vel_raw'),
                ('cmd_vel', cmd_vel_output),
            ],
        })

    nodes.append({
        'package': 'planner_cspace',
        'plugin': 'planner_cspace::PatrolActionNode',
        'executable': 'patrol',
        'name': 'patrol',
        'parameters': [],
        'remappings': [],
    })

    if simulate:
        nodes.append({
            'package': 'planner_cspace',
            'plugin': 'planner_cspace::DummyRobotNode',
            'executable': 'dummy_robot',
            'name': 'dummy_robot',
            'parameters': [],
            'remappings': [('cmd_vel', cmd_vel_output)],
        })

    return nodes


def to_composable(spec, use_intra) -> ComposableNode:
    """Build a ComposableNode description out of a node spec dictionary."""
    return ComposableNode(
        package=spec['package'],
        plugin=spec['plugin'],
        name=spec['name'],
        parameters=spec['parameters'],
        remappings=spec['remappings'],
        extra_arguments=[{'use_intra_process_comms': use_intra}],
    )


def to_standalone(spec, output) -> Node:
    """Build a standalone Node action out of a node spec dictionary."""
    return Node(
        package=spec['package'],
        executable=spec['executable'],
        name=spec['name'],
        parameters=spec['parameters'],
        remappings=spec['remappings'],
        output=output,
    )


def static_transform_node(context) -> Node:
    """Return the map -> odom static transform publisher used in simulation."""
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf1',
        arguments=[
            '--x', cfg('sim_robot_x'),
            '--y', cfg('sim_robot_y'),
            '--z', '0',
            '--yaw', cfg('sim_robot_yaw'),
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output=cfg('output_info'),
    )


def launch_setup(context, *args, **kwargs) -> list:
    """Assemble the actions once every launch argument can be evaluated."""
    del args, kwargs  # unused
    use_composition = as_bool(LaunchConfiguration('use_composition').perform(context))
    use_intra = as_bool(LaunchConfiguration('use_intra_process_comms').perform(context))
    output = LaunchConfiguration('output_info').perform(context)
    container_name = LaunchConfiguration('container_name').perform(context)
    simulate = as_bool(LaunchConfiguration('simulate').perform(context))

    specs = build_nodes(context)

    if use_composition:
        actions = [
            ComposableNodeContainer(
                name=container_name,
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    to_composable(spec, use_intra) for spec in specs
                ],
                output=output,
            ),
        ]
    else:
        actions = [to_standalone(spec, output) for spec in specs]

    if simulate:
        # tf2_ros::StaticTransformBroadcasterNode only accepts a quaternion when
        # used as a component, while the ROS 1 launch file gives a yaw angle.
        # Keep the standalone executable, which takes --yaw directly; /tf_static
        # is not on any zero-copy critical path.
        actions.append(static_transform_node(context))

    return actions


def generate_launch_description() -> LaunchDescription:
    """Entry point of the launch file."""
    return LaunchDescription([
        *declare_arguments(),
        OpaqueFunction(function=launch_setup),
    ])
