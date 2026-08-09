#!/usr/bin/env python3
"""
Regression test: CDR serialization of nested messages containing EMPTY
dynamic arrays.

A roseus action client sends FollowJointTrajectory goals whose trajectory
point has a non-empty ``positions`` array but empty ``velocities`` /
``accelerations`` / ``effort`` arrays, and a roseus action server echoes the
decoded values back. Before the rosidl_generator_eus alignment fix the goal
request failed to serialize (Fast CDR exception) and never reached the server.

cf. roseus/test/follow-joint-trajectory.test.py
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

    server = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(
            test_dir, 'test-cdr-nested-empty-array-server.l')],
        output='screen',
        name='server',
    )

    test_proc = launch.actions.ExecuteProcess(
        cmd=['roseus', os.path.join(
            test_dir, 'test-cdr-nested-empty-array-client.l')],
        output='screen',
        name='test_proc',
    )

    return launch.LaunchDescription([
        server,
        launch.actions.TimerAction(period=5.0, actions=[test_proc]),
        launch_testing.actions.ReadyToTest(),
    ]), {'test_proc': test_proc}


class TestCdrNestedEmptyArray(unittest.TestCase):
    def test_wait_for_shutdown(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(process=test_proc, timeout=115)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_proc)
