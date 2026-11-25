#!/usr/bin/env python3
"""
Launch test for ROS 2-style action API tests.
Uses fibonacci-server-ros2-async.l which supports cooperative cancellation.
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

    # Async server (supports cooperative cancellation)
    server = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(test_dir, 'fibonacci-server-ros2-async.l')],
        output='screen',
        name='server',
    )

    # Test client
    test_proc = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(test_dir, 'test-action-ros2-api.l')],
        output='screen',
        name='test_proc',
    )

    return launch.LaunchDescription([
        server,
        launch.actions.TimerAction(period=5.0, actions=[test_proc]),
        launch_testing.actions.ReadyToTest(),
    ]), {'test_proc': test_proc}


class TestFibonacciActionRos2Api(unittest.TestCase):
    def test_wait_for_shutdown(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(process=test_proc, timeout=120)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_proc)
