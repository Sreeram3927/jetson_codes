"""
camera_target_bridge_node

Websocket CLIENT connecting to the camera+AI container. Runs its own asyncio
event loop in a background thread (rclpy's spin() is not asyncio-based, so
the two loops are kept separate and only touch each other through
thread-safe publish calls).

Expects JSON messages matching the schema designed earlier:
  {"timestamp": ..., "frame_id": "...", "targets": [
      {"id": int, "class": str, "confidence": float,
       "position": {"x":.., "y":.., "z":..}}   # camera frame, meters
  ]}

Publishes: /camera/targets_raw (delta_msgs/TargetArray)

TODO: confirm this schema against what the camera container actually sends —
this was designed, not extracted from your existing code (no camera-side
source was provided). Adjust _parse_message() if the real format differs.
"""

import asyncio
import json
import threading
import time

import rclpy
from rclpy.node import Node
import socket

from delta_msgs.msg import TargetArray, Target


class CameraTargetBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_target_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 9999)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        self.pub = self.create_publisher(TargetArray, '/camera/targets_raw', 10)

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((host, port))
        self._server_sock.listen(1)
        self.get_logger().info(f'camera_target_bridge listening on {host}:{port}')

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def _accept_loop(self):
        while rclpy.ok():
            self.get_logger().info('Waiting for vision process to connect...')
            try:
                conn, addr = self._server_sock.accept()
            except OSError:
                break  # socket closed during shutdown
            self.get_logger().info(f'Vision process connected from {addr}')
            self._handle_connection(conn)
            self.get_logger().warn('Vision process disconnected; waiting for reconnect')

    def _handle_connection(self, conn):
        buf = b''
        with conn:
            while rclpy.ok():
                try:
                    chunk = conn.recv(4096)
                except OSError:
                    break
                if not chunk:
                    break  # peer closed the connection
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    if line.strip():
                        self._handle_message(line)

    def _handle_message(self, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON from camera container")
            return

        out = TargetArray()
        timestamp = float(data.get('timestamp', time.time()))

        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)

        out.header.stamp.sec = sec
        out.header.stamp.nanosec = nanosec

        for t in data.get('targets', []):
            target = Target()
            target.id = int(t.get('id', -1))
            target.class_name = t.get('class', '')
            target.confidence = float(t.get('confidence', 0.0))
            pos = t.get('position', {})
            target.position.x = float(pos.get('x', 0.0))
            target.position.y = float(pos.get('y', 0.0))
            target.position.z = float(pos.get('z', 0.0))
            out.targets.append(target)

        self.pub.publish(out)

    def destroy_node(self):
        try:
            self._server_sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTargetBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
