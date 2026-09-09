"""
launch_testing description for the planner_3d tolerant_action test.

ROS 2 port of the ROS 1 tolerant_action_rostest.test, which pulled in the shared node
set with <include file="actionlib_common_rostest.test" />. The launch_testing
equivalent of that fragment is the actionlib_common module next to this file.
"""

import os
import sys
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest

# launch_testing loads this file by path without putting its directory on
# sys.path, so the shared module has to be made importable explicitly.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from actionlib_common import common_nodes  # noqa: E402,I100


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    test_node = Node(
        package='planner_cspace',
        executable='ros2_test_tolerant_action',
        name='test_tolerant_action',
        output='screen',
    )

    return LaunchDescription(
        common_nodes() + [test_node, launch_testing.actions.ReadyToTest()],
    ), {'test_node': test_node}


class TestTerminates(unittest.TestCase):

    def test_gtest_runs_to_completion(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=240.0)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
