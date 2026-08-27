import socket
import threading
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DetectionRelay(Node):
    """
    Listens on a local TCP socket for JSON detection frames sent by the
    vision container (a separate Python 3.8 process, outside the ROS2
    graph) and republishes them as a ROS2 topic.

    Published as std_msgs/String containing the JSON payload rather than
    a typed message, to avoid needing a custom .msg + rosidl build step
    for now. Frontend consumers via rosbridge get the same JSON either way.
    Swap to a typed vision_msgs/Detection2DArray later if useful.
    """

    def __init__(self):
        super().__init__('detection_relay')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 9999)
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        self.publisher_ = self.create_publisher(String, '/vision/detections', 10)

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((host, port))
        self._server_sock.listen(1)
        self.get_logger().info(f'detection_relay listening on {host}:{port}')

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
                        self._republish(line)

    def _republish(self, line: bytes):
        try:
            data = json.loads(line.decode('utf-8'))
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Dropping malformed detection frame: {e}')
            return
        msg = String()
        msg.data = json.dumps(data)
        self.publisher_.publish(msg)

    def destroy_node(self):
        try:
            self._server_sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
