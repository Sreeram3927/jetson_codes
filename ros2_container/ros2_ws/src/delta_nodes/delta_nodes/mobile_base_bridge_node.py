"""
mobile_base_bridge_node

Serial bridge to the Arduino (mobile base). Ported from ArduinoCommunicator:
  - Line-based log reading is ported as-is (simple, already known-working).
  - Outbound command FORMAT is a placeholder — your original code just took
    whatever string was passed to send_command() and appended '\\n'; the
    actual text format (what a velocity command string looks like) lives in
    whatever code was calling ArduinoCommunicator.send_command(), which
    wasn't included. See _format_cmd_vel() below — confirm/replace it.

Subscribes:  /base/cmd_vel    (std_msgs/String)
Publishes:   /base/log        (std_msgs/String)
             /base/motion_state (std_msgs/UInt8)  0=STOPPED 1=MOVING
                 -- derived here from commanded velocity + settle timer,
                    per the motion-state gating design (no encoder feedback
                    assumed; swap the source if your Arduino reports actual
                    velocity back).
"""

import time
import json
import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8

from . import protocol_manager

MOTION_SETTLE_SEC = 0.4  # TODO: tune to your base's actual stopping inertia
VELOCITY_EPSILON = 0.01  # below this magnitude counts as "stopped"


class MobileBaseBridgeNode(Node):
    def __init__(self):
        super().__init__('mobile_base_bridge')

        self.declare_parameter('serial_port', '/dev/arduino_uno')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('poll_period_sec', 0.01)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        poll_period = self.get_parameter('poll_period_sec').value

        self.get_logger().info(f"Opening Arduino port {port} at {baud} baud...")
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        self._rx_buffer = ""
        self._last_moving_time = 0.0
        self._last_state = UInt8.MOVING if hasattr(UInt8, 'MOVING') else 1
        self._current_state = 0  # STOPPED

        self.log_pub = self.create_publisher(String, '/base/log', 10)
        self.motion_state_pub = self.create_publisher(UInt8, '/base/motion_state', 10)

        self.cmd_vel_sub = self.create_subscription(String, '/base/cmd_vel', self._on_cmd_vel, 10)

        self.create_timer(poll_period, self._poll_serial)
        # self.create_timer(0.1, self._update_motion_state)  # settle-timer check independent of cmd rate

    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: String):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial port not open, dropping cmd_vel")
            return

        command = msg.strip() + '\n'
        self.ser.write(command.encode('utf-8'))

        # moving = (abs(msg.linear.x) > VELOCITY_EPSILON or abs(msg.angular.z) > VELOCITY_EPSILON)
        # if moving:
        #     self._last_moving_time = time.time()

    # def _update_motion_state(self):
    #     moving = (time.time() - self._last_moving_time) < MOTION_SETTLE_SEC
    #     new_state = 1 if moving else 0
    #     if new_state != self._current_state:
    #         self._current_state = new_state
    #         out = UInt8()
    #         out.data = new_state
    #         self.motion_state_pub.publish(out)

    def _poll_serial(self):
        if self.ser is None or self.ser.in_waiting <= 0:
            return
        try:
            new_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
            self._rx_buffer += new_data
            while '\n' in self._rx_buffer:
                line, self._rx_buffer = self._rx_buffer.split('\n', 1)
                parsed = protocol_manager.parse_arduino_log(line)
                if parsed:
                    out = String()
                    out.data = json.dumps(parsed)
                    self.log_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f"Arduino read error: {e}")


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
