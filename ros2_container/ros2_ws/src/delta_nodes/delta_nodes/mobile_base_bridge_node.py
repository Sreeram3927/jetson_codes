"""
mobile_base_bridge_node

Serial bridge to the Arduino Uno mobile base, speaking the binary protocol
defined in the firmware's protocol.h/.cpp (see arduino_protocol.py's
Arduino section for the Python-side mirror of that format).

Subscribes:
  /base/cmd_vel        (geometry_msgs/Twist)
      linear.x / angular.z are treated as normalized -1.0..1.0, NOT real
      m/s or rad/s — there's no odometry yet to calibrate physical units
      against. max_linear_input / max_angular_input (ROS2 params) are the
      input magnitude that maps to +-100%; once you've measured real top
      speed, set these to real m/s and rad/s and this becomes unit-correct
      without any code changes.
  /frontend/arduino_cmd (std_msgs/String, JSON)
      Structured commands relayed from the frontend via frontend_bridge_node,
      e.g. {"command":"SET_LASER","power":50}. See _on_base_cmd() for the
      full set.

Publishes:
  /base/log            (std_msgs/String)   JSON log lines, as before
  /base/motion_state   (std_msgs/UInt8)    0=STOPPED 1=MOVING
      Derived directly from the Arduino's actual-velocity telemetry now,
      not a settle-timer heuristic — the firmware tells us the ramped
      actual speed, so "moving" is just "is that nonzero", no guessing.
  /base/status         (delta_msgs/BaseStatus)
      Richer status for the frontend relay: moving, estop_active,
      laser_pct, linear_pct, angular_pct — published on every telemetry
      frame (~10 Hz, matching the firmware's telemetry rate).

On startup, sends GET_VERSION and retries every 3s until the Arduino
replies, logging a warning if the reported protocol major version doesn't
match ARDUINO_PROTOCOL_VERSION in arduino_protocol.py.
"""

import json

import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8

from delta_msgs.msg import BaseStatus

from .protocol import arduino_protocol

MOTION_STOPPED = 0
MOTION_MOVING = 1

VERSION_RETRY_PERIOD_SEC = 3.0


class MobileBaseBridgeNode(Node):
    def __init__(self):
        super().__init__('mobile_base_bridge')

        self.declare_parameter('serial_port', '/dev/arduino_uno')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('poll_period_sec', 0.01)
        # Twist input magnitude that maps to +-100% of configured max speed.
        # Defaults assume normalized -1.0..1.0 input. Set to real m/s / rad/s
        # once you've measured actual top speed and want physical units.
        self.declare_parameter('max_linear_input', 1.0)
        self.declare_parameter('max_angular_input', 1.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        poll_period = self.get_parameter('poll_period_sec').value
        self._max_linear_input = float(self.get_parameter('max_linear_input').value)
        self._max_angular_input = float(self.get_parameter('max_angular_input').value)

        self.get_logger().info(f"Opening Arduino port {port} at {baud} baud...")
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        self._link_parser = arduino_protocol.ArduinoLinkParser()

        self._last_motion_state = MOTION_STOPPED
        self._last_estop_active = False
        self._version_confirmed = False

        self.log_pub = self.create_publisher(String, '/base/log', 10)
        self.motion_state_pub = self.create_publisher(UInt8, '/base/motion_state', 10)
        self.status_pub = self.create_publisher(BaseStatus, '/base/status', 10)

        self.cmd_vel_sub = self.create_subscription(Twist, '/base/cmd_vel', self._on_cmd_vel, 10)
        self.base_cmd_sub = self.create_subscription(String, '/frontend/base_cmd', self._on_base_cmd, 10)

        self.create_timer(poll_period, self._poll_serial)
        self._version_retry_timer = self.create_timer(VERSION_RETRY_PERIOD_SEC, self._request_version)
        self._request_version()  # fire immediately rather than waiting for the first timer tick

    # ------------------------------------------------------------------
    # Outbound (ROS2 -> Arduino)
    # ------------------------------------------------------------------
    def _write_frame(self, frame: bytes):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial port not open, dropping outbound frame")
            return
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f"Arduino write error: {e}")

    def _request_version(self):
        if self._version_confirmed:
            self._version_retry_timer.cancel()
            return
        self._write_frame(arduino_protocol.pack_arduino_get_version())

    def _on_cmd_vel(self, msg: Twist):
        linear_pct = 100.0 * msg.linear.x / self._max_linear_input if self._max_linear_input else 0.0
        angular_pct = 100.0 * msg.angular.z / self._max_angular_input if self._max_angular_input else 0.0
        self._write_frame(arduino_protocol.pack_arduino_drive(linear_pct, angular_pct))

    def _on_base_cmd(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON on /frontend/arduino_cmd: {msg.data!r}")
            return

        command = data.get('command')

        if command == 'SET_LASER':
            self._write_frame(arduino_protocol.pack_arduino_set_laser(data.get('power', 0)))
        elif command == 'SET_MAX_SPEED':
            self._write_frame(arduino_protocol.pack_arduino_set_max_speed(data.get('value', 50)))
        elif command == 'SET_RAMP_RATE':
            self._write_frame(arduino_protocol.pack_arduino_set_ramp_rate(
                data.get('accel', 4), data.get('decel', 6)))
        elif command == 'SOFT_STOP':
            self._write_frame(arduino_protocol.pack_arduino_soft_stop())
        elif command == 'GET_VERSION':
            self._write_frame(arduino_protocol.pack_arduino_get_version())
        else:
            self.get_logger().warn(f"Unknown arduino command: {command!r}")

    # ------------------------------------------------------------------
    # Inbound (Arduino -> ROS2)
    # ------------------------------------------------------------------
    def _poll_serial(self):
        if self.ser is None or self.ser.in_waiting <= 0:
            return
        try:
            raw = self.ser.read(self.ser.in_waiting)
        except serial.SerialException as e:
            self.get_logger().error(f"Arduino read error: {e}")
            return

        for event in self._link_parser.feed(raw):
            etype = event.get('type')
            if etype == 'log':
                out = String()
                out.data = json.dumps(event)
                self.log_pub.publish(out)
            elif etype == 'telemetry':
                self._handle_telemetry(event)
            elif etype == 'version':
                self._handle_version(event)

    def _handle_telemetry(self, event: dict):
        moving = (event['linear_actual'] != 0 or event['angular_actual'] != 0)
        motion_state = MOTION_MOVING if moving else MOTION_STOPPED
        if motion_state != self._last_motion_state:
            self._last_motion_state = motion_state
            out = UInt8()
            out.data = motion_state
            self.motion_state_pub.publish(out)

        if event['estop_sensed'] != self._last_estop_active:
            self._last_estop_active = event['estop_sensed']
            level = "warn" if event['estop_sensed'] else "info"
            getattr(self.get_logger(), level)(
                f"Base E-stop {'ENGAGED' if event['estop_sensed'] else 'CLEARED'}")

        status = BaseStatus()
        status.moving = moving
        status.estop_active = event['estop_sensed']
        status.laser_pct = event['laser_pct']
        status.linear_pct = event['linear_actual']
        status.angular_pct = event['angular_actual']
        self.status_pub.publish(status)

    def _handle_version(self, event: dict):
        major, minor = event['major'], event['minor']
        expected_major, expected_minor = arduino_protocol.ARDUINO_PROTOCOL_VERSION
        if major != expected_major:
            self.get_logger().error(
                f"Arduino protocol version mismatch: firmware reports {major}.{minor}, "
                f"this node expects major version {expected_major}. Commands may be misinterpreted.")
        elif minor != expected_minor:
            self.get_logger().warn(
                f"Arduino protocol minor version differs: firmware {major}.{minor}, "
                f"node expects {expected_major}.{expected_minor} (compatible, but consider syncing).")
        else:
            self.get_logger().info(f"Arduino firmware confirmed: protocol v{major}.{minor}")

        self._version_confirmed = True
        self._version_retry_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = MobileBaseBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()