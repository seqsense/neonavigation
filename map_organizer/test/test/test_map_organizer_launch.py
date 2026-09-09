"""
launch_testing description for the map_organizer integration test.

Mirrors the ROS 1 map_organizer_rostest.test:
  * tie_maps       -- loads test/data/{0,1}.yaml and publishes /maps, /map0, /map1
  * save_maps      -- re-serialises /maps to <tmp_prefix>{0,1}.{pgm,yaml}
  * tie_maps (ns=saved, respawn) -- reloads the saved files as /saved/maps
  * select_map     -- republishes the selected floor on /map

The ros2_test_map_organizer gtest binary owns its rclcpp context (calls
rclcpp::init in main), so it is started as a plain node and its exit code is
checked after shutdown.
"""

import glob
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


TMP_PREFIX = '/tmp/tmp-ros2-map-organizer-988dbe-'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    data_dir = os.path.join(
        get_package_share_directory('map_organizer'), 'test', 'data')

    # Remove any stale files from previous runs so tie_maps (ns=saved) only
    # succeeds once save_maps has written fresh output.
    for f in glob.glob(TMP_PREFIX + '*'):
        os.remove(f)

    map_files = '{d}/0.yaml,{d}/1.yaml'.format(d=data_dir)

    tie_maps = Node(
        package='map_organizer',
        executable='tie_maps',
        name='tie_maps',
        output='screen',
        parameters=[{'map_files': map_files, 'frame_id': 'map_ground'}],
    )
    save_maps = Node(
        package='map_organizer',
        executable='save_maps',
        name='save_maps',
        output='screen',
        arguments=['-f', TMP_PREFIX],
    )
    # respawn while the saved files do not yet exist; the ROS 2 tie_maps exits
    # when the YAML cannot be loaded, so it is restarted until save_maps has run.
    tie_maps_saved = Node(
        package='map_organizer',
        executable='tie_maps',
        name='tie_maps2',
        namespace='saved',
        output='screen',
        parameters=[{
            'map_files': '{p}0.yaml,{p}1.yaml'.format(p=TMP_PREFIX),
            'frame_id': 'map_ground',
        }],
        respawn=True,
        respawn_delay=1.0,
    )
    select_map = Node(
        package='map_organizer',
        executable='select_map',
        name='select_map',
        output='screen',
    )
    test_node = Node(
        package='map_organizer',
        executable='ros2_test_map_organizer',
        name='test_map_organizer',
        output='screen',
    )

    return LaunchDescription([
        tie_maps,
        save_maps,
        tie_maps_saved,
        select_map,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestMapOrganizerTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestMapOrganizerOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
