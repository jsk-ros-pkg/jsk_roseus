# Copyright (c) 2026, JSK Lab, University of Tokyo.  BSD License.
"""Test of roseus_mongo with message_store_node of mongodb_store (ROS 2).

mongodb_server of mongodb_store (ROS 2) waits for "mongod startup complete"
in the log of mongod >= 4.4, so this test runs mongod by itself, and
provides /mongodb_server (its parameters and /datacentre/wait_ready) for
message_store_node, so that it works with any mongod.
"""

import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading

import pytest

TEST_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def free_port():
    s = socket.socket()
    s.bind(('localhost', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.mark.parametrize('script', ['test-json-encode.l', 'test-json-decode.l'])
def test_json(script):
    p = subprocess.run(['roseus', os.path.join(TEST_DIR, script)],
                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       universal_newlines=True, errors='replace', timeout=120)
    print(p.stdout)
    assert p.returncode == 0, p.stdout


def mongodb_store_found():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('mongodb_store')
        return True
    except Exception:
        return False


@pytest.mark.skipif(shutil.which('mongod') is None, reason='mongod is not installed')
@pytest.mark.skipif(not mongodb_store_found(), reason='mongodb_store (ROS 2) is not found')
def test_mongo_client():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_srvs.srv import Empty
    db_path = tempfile.mkdtemp(prefix='roseus_mongo_')
    port = free_port()
    mongod = subprocess.Popen(['mongod', '--dbpath', db_path, '--port', str(port),
                               '--bind_ip', '127.0.0.1'],
                              stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    rclpy.init()
    server = rclpy.create_node('mongodb_server', parameter_overrides=[
        Parameter('mongodb_host', value='localhost'),
        Parameter('mongodb_port', value=port)])
    server.declare_parameter('mongodb_host', 'localhost')
    server.declare_parameter('mongodb_port', port)
    server.create_service(Empty, '/datacentre/wait_ready', lambda req, res: res)
    server.create_service(Empty, '/datacentre/shutdown', lambda req, res: res)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    store = subprocess.Popen(['ros2', 'run', 'mongodb_store', 'message_store_node'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             universal_newlines=True, start_new_session=True)
    try:
        p = subprocess.run(
            ['roseus', os.path.join(TEST_DIR, 'test-mongo-client.l'),
             '_robot/database:=test_database', '_robot/name:=test_collection'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            universal_newlines=True, errors='replace', timeout=240)
        print(p.stdout)
        assert p.returncode == 0, p.stdout
    finally:
        os.killpg(store.pid, signal.SIGINT)
        try:
            print(store.communicate(timeout=20)[0])
        except subprocess.TimeoutExpired:
            os.killpg(store.pid, signal.SIGKILL)
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()
        mongod.terminate()
        mongod.wait()
        shutil.rmtree(db_path, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v', '-s']))
