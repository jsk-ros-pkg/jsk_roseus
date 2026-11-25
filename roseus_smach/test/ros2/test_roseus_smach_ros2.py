# Copyright (c) 2026, JSK Lab, University of Tokyo.  BSD License.
"""Tests of roseus_smach on ROS 2."""

import os
import subprocess
import sys
import time

import pytest
import rclpy

TEST_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def roseus(script):
    # stdin is kept open, do-until-key stops when stdin is closed
    return subprocess.Popen(['roseus', os.path.join(TEST_DIR, script)],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, universal_newlines=True)


def wait(p, executor=None, timeout=300):
    end = time.time() + timeout
    while time.time() < end and p.poll() is None:
        if executor:
            executor.spin_once(timeout_sec=0.1)
        else:
            time.sleep(0.1)
    if p.poll() is None:
        p.kill()
    out = p.communicate()[0]
    print(out)
    return p.returncode, out


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = rclpy.create_node('test_roseus_smach_ros2')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def fibonacci_server(node, name):
    from example_interfaces.action import Fibonacci
    from rclpy.action import ActionServer

    def execute(goal_handle):
        seq = [0, 1]
        for i in range(1, goal_handle.request.order):
            seq.append(seq[i] + seq[i - 1])
        goal_handle.succeed()
        return Fibonacci.Result(sequence=seq)
    return ActionServer(node, Fibonacci, name, execute)


def test_samples(node):
    from rclpy.executors import MultiThreadedExecutor
    server = fibonacci_server(node, 'fibonacci')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        code, out = wait(roseus('test-samples.l'), executor)
    finally:
        executor.remove_node(node)
        server.destroy()
    assert code == 0, out


def test_roseus_smach_utils():
    code, out = wait(roseus('test-roseus-smach-utils.l'))
    assert code == 0, out


def test_parallel_state_machine():
    code, out = wait(roseus('test-parallel-state-machine.l'))
    assert code == 0, out


def test_async_join_state_machine_actionlib():
    servers = [roseus('multiply10-action-server.l'), roseus('sub5-action-server.l')]
    try:
        code, out = wait(roseus('test-async-join-state-machine-actionlib.l'))
    finally:
        for s in servers:
            s.kill()
            s.communicate()
    assert code == 0, out


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v', '-s']))
