# Copyright (c) 2026, JSK Lab, University of Tokyo.  BSD License.
"""Tests of roseus on ROS 2, against rclpy."""

import os
import subprocess
import sys
import time

import pytest

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))


def roseus(script, **kwargs):
    return subprocess.Popen(['roseus', os.path.join(HERE, script)],
                            stdin=subprocess.DEVNULL,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True, **kwargs)


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = rclpy.create_node('test_roseus_ros2')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def sample_messages():
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Empty, Header, String, Int8MultiArray
    from roseus.msg import FixedArray, VariableArray, TestName
    from example_interfaces.msg import WString
    from visualization_msgs.msg import Marker

    msgs = [String(data='hello'), String(data=''), Empty(),
            Header(stamp=Time(sec=1, nanosec=4000000000 - 2 ** 31), frame_id='base'),
            WString(data='abc')]
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = 1.5
    p.pose.orientation.w = -0.25
    msgs.append(p)
    msgs.append(JointState(name=['a', 'bc', 'def'], position=[1.0, 2.0],
                           velocity=[], effort=[3.0]))
    msgs.append(Int8MultiArray(data=[-128, -1, 0, 127]))
    f = FixedArray()
    f.float32_data = [1.5, -2.5, 3.0]
    # euslisp floats lose the 2 lowest bits of the mantissa, use exact values
    f.float64_data = [2.0 ** 100, -2.0 ** -100, 0.5]
    f.int16_data = [-32768, 0, 32767]
    f.int32_data = [-2 ** 31, 0, 2 ** 31 - 1]
    f.int64_data = [-2 ** 63, 0, 2 ** 63 - 1]
    f.int8_data = [-128, 0, 127]
    f.uint16_data = [0, 1, 65535]
    f.uint32_data = [0, 1, 2 ** 32 - 1]
    f.uint64_data = [0, 1, 2 ** 64 - 1]
    f.uint8_data = list(range(17))
    f.bool_data = [True, False]
    f.time_data[1].sec = 10
    f.time_data[1].nanosec = 20
    f.duration_data[0].sec = -3
    f.string_data[0].data = 'x'
    msgs.append(f)
    v = VariableArray()
    v.float32_data = [1.5]
    v.float64_data = []
    v.int16_data = [1, 2, 3]
    v.int64_data = [5]
    v.uint8_data = [0, 255, 16]
    v.bool_data = [False, True, True]
    v.time_data = [Time(sec=1, nanosec=2)]
    v.string_data = [String(data='p'), String(data='qr')]
    msgs.append(v)
    t = TestName()
    t.name.data = 'nested'
    t.name.header.frame_id = 'f'
    msgs.append(t)
    m = Marker()
    m.ns = 'ns'
    m.points = []
    m.text = 'marker'
    m.mesh_resource = 'package://a/b'
    msgs.append(m)
    return msgs


def type_name(msg):
    mod = type(msg).__module__.split('.')
    return '%s/%s/%s' % (mod[0], mod[1], type(msg).__name__)


def test_cdr_roundtrip():
    """Deserializing and serializing on roseus gives the same message."""
    msgs = sample_messages()
    lines = ''.join('%s %s\n' % (type_name(m), serialize_message(m)[4:].hex())
                    for m in msgs)
    p = subprocess.run(['roseus', os.path.join(HERE, 'cdr-roundtrip.l')],
                   input=lines, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                   universal_newlines=True, timeout=120)
    results = [line.split(' ')[1:] for line in p.stdout.splitlines()
               if line.startswith('CDR ')]
    assert len(results) == len(msgs), p.stdout
    for msg, (t, h) in zip(msgs, results):
        assert t == type_name(msg)
        back = deserialize_message(b'\x00\x01\x00\x00' + bytes.fromhex(h), get_message(t))
        assert back == msg, (t, p.stdout)


def spin_until(node, cond, timeout):
    end = time.time() + timeout
    while time.time() < end and not cond():
        rclpy.spin_once(node, timeout_sec=0.1)
    return cond()


def test_talker(node):
    from std_msgs.msg import String
    received = []
    node.create_subscription(String, 'chatter', lambda m: received.append(m.data), 10)
    p = roseus('talker.l')
    try:
        assert spin_until(node, lambda: len(received) >= 3, 60), received
        assert received[0].startswith('hello world')
    finally:
        p.kill()
        p.wait()


def test_listener(node):
    from std_msgs.msg import String
    echo = []
    pub = node.create_publisher(String, 'chatter', 10)
    node.create_subscription(String, 'chatter_echo', lambda m: echo.append(m.data), 10)
    p = roseus('listener.l')
    try:
        end = time.time() + 60
        i = 0
        while time.time() < end and p.poll() is None and len(echo) < 5:
            pub.publish(String(data='msg %d' % i))
            i += 1
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(echo) >= 1
        assert echo[0].startswith('msg ')
    finally:
        p.kill()
        out = p.communicate()[0]
    print(out)


def test_service_server(node):
    from example_interfaces.srv import AddTwoInts
    p = roseus('add-two-ints-server.l')
    try:
        cli = node.create_client(AddTwoInts, 'add_two_ints')
        assert cli.wait_for_service(timeout_sec=60)
        future = cli.call_async(AddTwoInts.Request(a=-5, b=2 ** 40))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        assert future.result().sum == 2 ** 40 - 5
    finally:
        p.kill()
        p.wait()


def test_service_client(node):
    from example_interfaces.srv import AddTwoInts
    srv = node.create_service(
        AddTwoInts, 'add_two_ints',
        lambda req, res: (setattr(res, 'sum', req.a + req.b), res)[1])
    p = roseus('add-two-ints-client.l')
    try:
        end = time.time() + 90
        while time.time() < end and p.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        out = p.communicate()[0]
        assert p.returncode == 0, out
    finally:
        node.destroy_service(srv)
        if p.poll() is None:
            p.kill()



def test_action_client(node):
    """roseus simple-action-client <-> rclpy ActionServer."""
    from rclpy.action import ActionServer, CancelResponse
    from example_interfaces.action import Fibonacci

    def execute(goal_handle):
        seq = [0, 1]
        fb = Fibonacci.Feedback()
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result(sequence=seq)
            seq.append(seq[i] + seq[i - 1])
            fb.sequence = seq
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)
        goal_handle.succeed()
        return Fibonacci.Result(sequence=seq)

    from rclpy.executors import MultiThreadedExecutor
    server = ActionServer(node, Fibonacci, 'fibonacci', execute,
                          cancel_callback=lambda g: CancelResponse.ACCEPT)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    p = roseus('fibonacci-client.l')
    try:
        end = time.time() + 90
        while time.time() < end and p.poll() is None:
            executor.spin_once(timeout_sec=0.1)
        out = p.communicate()[0]
        assert p.returncode == 0, out
    finally:
        server.destroy()
        executor.remove_node(node)
        if p.poll() is None:
            p.kill()


def test_action_server(node):
    """roseus simple-action-server <-> rclpy ActionClient."""
    from rclpy.action import ActionClient
    from example_interfaces.action import Fibonacci
    p = roseus('fibonacci-server.l')
    client = ActionClient(node, Fibonacci, 'fibonacci')
    try:
        assert client.wait_for_server(timeout_sec=60)
        feedbacks = []
        future = client.send_goal_async(Fibonacci.Goal(order=6),
                                        feedback_callback=lambda f: feedbacks.append(f))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        handle = future.result()
        assert handle.accepted
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=30)
        assert list(result.result().result.sequence) == [0, 1, 1, 2, 3, 5, 8]
        assert result.result().status == 4  # SUCCEEDED
        assert len(feedbacks) > 0
        # cancel
        future = client.send_goal_async(Fibonacci.Goal(order=100))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        handle = future.result()
        spin_until(node, lambda: False, 0.5)
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=30)
        assert len(cancel.result().goals_canceling) == 1
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=30)
        assert result.result().status == 5  # CANCELED
    finally:
        client.destroy()
        p.kill()
        out = p.communicate()[0]
    print(out)



def test_tf(node):
    """roseus transform-broadcaster / transform-listener <-> rclpy tf2_ros."""
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf, node)
    static = tf2_ros.StaticTransformBroadcaster(node)
    t = TransformStamped()
    t.header.frame_id = 'world'
    t.child_frame_id = 'rclpy_frame'
    t.transform.translation.x = 1.0
    t.transform.translation.y = -2.0
    t.transform.translation.z = 0.5
    t.transform.rotation.w = 1.0
    static.sendTransform(t)
    p = roseus('tf.l')
    try:
        def ok():
            return buf.can_transform('eus_base', 'eus_child', rclpy.time.Time())
        assert spin_until(node, ok, 60)
        tr = buf.lookup_transform('eus_base', 'eus_child', rclpy.time.Time()).transform
        assert abs(tr.translation.x - 0.1) < 1e-6 and abs(tr.translation.z - 0.3) < 1e-6
        assert abs(abs(tr.rotation.z) - 0.70710678) < 1e-4  # yaw pi/2
        spin_until(node, lambda: p.poll() is not None, 60)
    finally:
        if p.poll() is None:
            p.kill()
        out = p.communicate()[0]
        del listener
    assert 'LOOKUP #f(1000.0 -2000.0 500.0)' in out, out



def test_node_param(node):
    """ros::get-node-param / ros::set-node-param with an rclpy node."""
    from rclpy.executors import SingleThreadedExecutor
    target = rclpy.create_node('param_target')
    target.declare_parameter('gain', 1.0)
    target.declare_parameter('name', 'py')
    executor = SingleThreadedExecutor()
    executor.add_node(target)
    p = roseus('node-param.l')
    try:
        end = time.time() + 60
        while time.time() < end and p.poll() is None:
            executor.spin_once(timeout_sec=0.1)
        out = p.communicate()[0]
    finally:
        if p.poll() is None:
            p.kill()
        executor.remove_node(target)
        target.destroy_node()
    assert 'GET 1.0' in out, out
    assert 'SET t' in out, out
    assert 'SETDR t' in out, out
    assert 'GET2 2.5 eus' in out, out
    assert 'OWN 10 ((baz . s)) nil' in out, out


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v', '-s']))
