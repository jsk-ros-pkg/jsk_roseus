# Copyright (c) 2026, JSK Lab, University of Tokyo.  BSD License.
"""Test of roseus_tutorials on ROS 2: markers made by roseus-utils.l."""

import os
import subprocess
import sys
import time

import pytest
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))


def test_publish_marker():
    from visualization_msgs.msg import Marker, MarkerArray
    rclpy.init()
    node = rclpy.create_node('test_publish_marker_ros2')
    markers, arrays = [], []
    node.create_subscription(Marker, '/cylinder_marker', markers.append, 10)
    node.create_subscription(MarkerArray, '/all_marker_array', arrays.append, 10)
    p = subprocess.Popen(['roseus', os.path.join(HERE, 'publish-marker-nogui.l')],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, universal_newlines=True, errors='replace')
    try:
        end = time.time() + 60
        while time.time() < end and not (markers and arrays) and p.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        p.kill()
        out = p.communicate()[0]
        node.destroy_node()
        rclpy.shutdown()
    assert markers and arrays, out
    m = markers[0]
    assert m.type == Marker.CYLINDER and m.header.frame_id == 'test_id'
    assert abs(m.scale.x - 0.2) < 1e-6 and abs(m.scale.z - 0.2) < 1e-6
    types = sorted(x.type for x in arrays[0].markers)
    assert Marker.CUBE in types and Marker.SPHERE in types and Marker.TEXT_VIEW_FACING in types
    assert [x.text for x in arrays[0].markers if x.type == Marker.TEXT_VIEW_FACING] == ['roseus']



def interfaces_found():
    try:
        import image_view2.msg  # noqa: F401
        import opencv_apps.msg  # noqa: F401
        return True
    except ImportError:
        return False


def const(c):
    """byte constants of messages are bytes in rclpy"""
    return c[0] if isinstance(c, bytes) else c


def run_client(script, topic, msg, check, timeout=60):
    """Publish msg on topic, and wait for an image_view2 marker which satisfies check."""
    from image_view2.msg import ImageMarker2
    rclpy.init()
    node = rclpy.create_node('test_' + script.replace('-', '_').replace('.', '_'))
    markers = []
    node.create_subscription(ImageMarker2, '/image_marker', markers.append, 10)
    pub = node.create_publisher(type(msg), topic, 10)
    p = subprocess.Popen(['roseus', os.path.join(os.path.dirname(HERE), '..', 'src', script)],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, universal_newlines=True, errors='replace')
    try:
        end = time.time() + timeout
        while time.time() < end and not any(check(m) for m in markers) and p.poll() is None:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        p.kill()
        out = p.communicate()[0]
        node.destroy_node()
        rclpy.shutdown()
    assert any(check(m) for m in markers), out
    return markers


@pytest.mark.skipif(not interfaces_found(), reason='image_view2 or opencv_apps is not found')
def test_camshiftdemo():
    from image_view2.msg import ImageMarker2
    from opencv_apps.msg import RotatedRectStamped
    msg = RotatedRectStamped()
    msg.rect.center.x = 100.0
    msg.rect.center.y = 50.0
    msg.rect.size.width = 40.0
    msg.rect.size.height = 20.0
    msg.rect.angle = 90.0
    markers = run_client('camshiftdemo.l', '/camshift/track_box', msg,
                         lambda m: m.type == const(ImageMarker2.LINE_LIST))
    circle = [m for m in markers if m.type == const(ImageMarker2.CIRCLE)][0]
    assert circle.position.x == 100.0 and circle.position.y == 50.0
    lines = [m for m in markers if m.type == const(ImageMarker2.LINE_LIST)][0]
    assert len(lines.points) == 4
    # width/2 = 20 along the angle (90 deg)
    assert abs(lines.points[0].x - 100.0) < 1e-3 and abs(lines.points[0].y - 70.0) < 1e-3


@pytest.mark.skipif(not interfaces_found(), reason='image_view2 or opencv_apps is not found')
def test_face_detector_mono():
    from image_view2.msg import ImageMarker2
    from opencv_apps.msg import Face, FaceArrayStamped
    msg = FaceArrayStamped()
    f = Face()
    f.face.x, f.face.y, f.face.width, f.face.height = 100.0, 80.0, 40.0, 60.0
    msg.faces = [f]
    markers = run_client('face-detector-mono.l', '/face_detection/faces', msg,
                         lambda m: m.type == const(ImageMarker2.POLYGON))
    poly = [m for m in markers if m.type == const(ImageMarker2.POLYGON)][0]
    assert [(p.x, p.y) for p in poly.points] == [(80.0, 50.0), (80.0, 110.0), (120.0, 110.0), (120.0, 50.0)]
    assert poly.lifetime_sec == 5

if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v', '-s']))
