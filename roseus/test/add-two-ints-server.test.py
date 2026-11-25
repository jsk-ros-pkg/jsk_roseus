#!/usr/bin/env python3
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

    # Server test (the actual test process) starts first
    test_proc = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(test_dir, 'test-add-two-ints-server-ros2.l')],
        output='screen',
        name='test_proc',
    )

    # Client starts after delay to allow server to advertise first
    client = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(test_dir, 'add-two-ints-client-ros2.l')],
        output='screen',
        name='client',
    )

    return launch.LaunchDescription([
        test_proc,
        launch.actions.TimerAction(period=5.0, actions=[client]),
        launch_testing.actions.ReadyToTest(),
    ]), {'test_proc': test_proc}


class TestAddTwoIntsServer(unittest.TestCase):
    def test_wait_for_shutdown(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(process=test_proc, timeout=115)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_proc)
