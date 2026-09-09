"""
launch_testing description for the costmap_3d integration test.

The ROS 1 package had no rostest for costmap_3d, so this test exists to pin
down the ROS 2 parameter representation of the layer chain, which had to be
flattened because ROS 2 parameters cannot hold the nested XmlRpc structure the
ROS 1 node parsed:

  ROS 1                              ROS 2
  ---------------------------------  ------------------------------------
  footprint:                         footprint: [0.4, -0.4, 0.4, 0.4,
    [[0.4, -0.4], [0.4, 0.4],                   -0.4, 0.4, -0.4, -0.4]
     [-0.4, 0.4], [-0.4, -0.4]]
  layers:                            layers: ["overlay"]
  - name: overlay                    layer.overlay.type: Costmap3dLayerFootprint
    type: Costmap3dLayerFootprint    layer.overlay.overlay_mode: max
    overlay_mode: max

Two costmap_3d nodes are started: one with an explicit static_layers/layers
chain and one without "layers" at all, which selects the backward compatible
single-layer mode. The ros2_test_costmap_3d gtest binary owns its rclcpp
context (calls rclcpp::init in main), so it is started as a plain node and its
exit code is checked after shutdown.
"""

import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest

# Square footprint smaller than half a cell of the 1.0 m resolution test map,
# so that the C-space of the test map is the test map itself.
FOOTPRINT = [0.4, -0.4, 0.4, 0.4, -0.4, 0.4, -0.4, -0.4]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    costmap_3d_layered = Node(
        package='costmap_cspace',
        executable='costmap_3d',
        name='costmap_3d',
        namespace='layered',
        output='screen',
        parameters=[{
            'ang_resolution': 4,
            'footprint': FOOTPRINT,
            'linear_expand': 0.0,
            'linear_spread': 0.0,
            # Layer in front of the static output: its overlays re-publish the
            # whole costmap.
            'static_layers': ['static_overlay'],
            'static_layer.static_overlay.type': 'Costmap3dLayerPlain',
            'static_layer.static_overlay.overlay_mode': 'max',
            # Layer behind the static output: its overlays emit costmap_update.
            'layers': ['overlay'],
            'layer.overlay.type': 'Costmap3dLayerFootprint',
            'layer.overlay.overlay_mode': 'max',
            'layer.overlay.footprint': FOOTPRINT,
        }],
    )
    # No "layers" parameter: single-layer backward compatibility mode, which
    # subscribes "map_overlay" and honours "overlay_mode".
    costmap_3d_compat = Node(
        package='costmap_cspace',
        executable='costmap_3d',
        name='costmap_3d',
        namespace='compat',
        output='screen',
        parameters=[{
            'ang_resolution': 4,
            'footprint': FOOTPRINT,
            'linear_expand': 0.0,
            'linear_spread': 0.0,
            'overlay_mode': 'max',
        }],
    )

    test_node = Node(
        package='costmap_cspace',
        executable='ros2_test_costmap_3d',
        name='test_costmap_3d',
        output='screen',
    )

    return LaunchDescription([
        costmap_3d_layered,
        costmap_3d_compat,
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}


class TestCostmap3dTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestCostmap3dOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
