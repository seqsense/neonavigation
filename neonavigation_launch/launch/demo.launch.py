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
ROS 2 port of demo.launch.

Runs the neonavigation demo on the bundled ``map/demo_map.yaml``: it includes
navigate.launch.py in simulation mode and adds the local map server, the
largemap_to_map overlay generator and RViz 2.

The extra nodes are loaded into the very same component container that
navigate.launch.py creates (``container_name``), so the map -> costmap chain
keeps using intra-process transport. With ``use_composition:=false`` everything
runs as standalone processes.

RViz 2 uses ``config/visualization_ros2.rviz``; pass ``use_rviz:=false`` on
headless machines.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

DEFAULT_CONTAINER_NAME = 'neonavigation_container'


def as_bool(value: str) -> bool:
    """Interpret a launch argument string as a boolean, like IfCondition does."""
    return value.lower() in ('true', '1', 'yes', 'on', 'enable')


def build_nodes(context) -> list:
    """Return the node specs that demo.launch adds on top of navigate.launch."""
    share = get_package_share_directory('neonavigation_launch')

    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    return [
        {
            # ROS 1: map_server publishing "map" remapped to
            # "map_with_local_objects". nav2's map_server is a lifecycle node,
            # so it is told to bring itself up (see navigate.launch.py).
            'package': 'nav2_map_server',
            'plugin': 'nav2_map_server::MapServer',
            'executable': 'map_server',
            'name': 'map_server_local',
            'parameters': [{
                'yaml_filename': os.path.join(share, 'map', 'demo_map_local.yaml'),
                'autostart_node': True,
                'bond_heartbeat_period': 0.0,
            }],
            'remappings': [('map', 'map_with_local_objects')],
        },
        {
            'package': 'costmap_cspace',
            'plugin': 'costmap_cspace::LargeMapToMapNode',
            'executable': 'largemap_to_map',
            'name': 'largemap_to_map',
            'parameters': [{
                'width': 60,
                'hz': 1.5,
                'round_local_map': True,
                'simulate_occlusion': True,
                'simulate_surrounded': as_bool(cfg('simulate_surrounded')),
            }],
            'remappings': [
                ('map', 'map_with_local_objects'),
                ('map_local', 'overlay1'),
            ],
        },
    ]


def launch_setup(context, *args, **kwargs) -> list:
    """Assemble the actions once every launch argument can be evaluated."""
    del args, kwargs  # unused
    share = get_package_share_directory('neonavigation_launch')

    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    use_composition = as_bool(cfg('use_composition'))
    use_intra = as_bool(cfg('use_intra_process_comms'))
    container_name = cfg('container_name')

    navigate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'navigate.launch.py')),
        launch_arguments={
            'simulate': 'true',
            'enable_crowd_mode': cfg('enable_crowd_escape'),
            'map_file': os.path.join(share, 'map', 'demo_map.yaml'),
            'use_path_with_velocity': cfg('use_path_with_velocity'),
            'vel': cfg('vel'),
            'use_composition': cfg('use_composition'),
            'use_intra_process_comms': cfg('use_intra_process_comms'),
            'container_name': container_name,
        }.items(),
    )

    specs = build_nodes(context)
    if use_composition:
        extra = [
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        package=spec['package'],
                        plugin=spec['plugin'],
                        name=spec['name'],
                        parameters=spec['parameters'],
                        remappings=spec['remappings'],
                        extra_arguments=[{'use_intra_process_comms': use_intra}],
                    )
                    for spec in specs
                ],
            ),
        ]
    else:
        extra = [
            Node(
                package=spec['package'],
                executable=spec['executable'],
                name=spec['name'],
                parameters=spec['parameters'],
                remappings=spec['remappings'],
                output='screen',
            )
            for spec in specs
        ]

    actions = [navigate, *extra]

    if as_bool(cfg('use_rviz')):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', cfg('rviz_config')],
            respawn=True,
            output='screen',
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    """Entry point of the launch file."""
    default_rviz = os.path.join(
        get_package_share_directory('neonavigation_launch'),
        'config', 'visualization_ros2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_path_with_velocity', default_value='true'),
        DeclareLaunchArgument('enable_crowd_escape', default_value='false'),
        DeclareLaunchArgument('simulate_surrounded', default_value='false'),
        DeclareLaunchArgument('vel', default_value='0.5'),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='Use component container (true) or standalone nodes (false)'),
        DeclareLaunchArgument(
            'use_intra_process_comms', default_value='true',
            description='Enable intra-process communication (zero-copy when true)'),
        DeclareLaunchArgument(
            'container_name', default_value=DEFAULT_CONTAINER_NAME,
            description='Name of the component container shared with navigate.launch.py'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz 2 (set to false on headless machines)'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz 2 configuration file'),
        OpaqueFunction(function=launch_setup),
    ])
