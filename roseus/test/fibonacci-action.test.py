#!/usr/bin/env python3
"""
ROS 2 version of test-actionlib.test (ROS 1)
cf. roseus/test/test-actionlib.test
"""
import os
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    test_dir = os.path.dirname(os.path.abspath(__file__))

    # Server (fibonacci-server-ros2.l) starts first
    server = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(test_dir, 'fibonacci-server-ros2.l')],
        output='screen',
        name='server',
    )

    # Client test (test-actionlib-ros2.l) - ROS 2 action test
    test_proc = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(test_dir, 'test-actionlib-ros2.l')],
        output='screen',
        name='test_proc',
    )

    return launch.LaunchDescription([
        server,
        launch.actions.TimerAction(period=5.0, actions=[test_proc]),
        launch_testing.actions.ReadyToTest(),
    ]), {'test_proc': test_proc}


class TestFibonacciAction(unittest.TestCase):
    def test_wait_for_shutdown(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(process=test_proc, timeout=115)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_proc)
