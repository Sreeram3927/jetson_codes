"""
manipulator_bridge_node

Serial bridge to the ESP32 (delta manipulator). Ported from the existing
ESPCommunicator asyncio implementation:
  - Telemetry parsing (sync marker + struct format) is ported EXACTLY as-is —
    this is known-working and unchanged.
  - Outbound command packing (ProtocolManager.pack_coordinate_command /
    pack_joint_command) is NOT ported — that file (protocol.py) wasn't
    provided. See protocol_manager.py in this package: it has the same
    function signatures as a drop-in placeholder, but raises
    NotImplementedError until the real packing logic is pasted in.

Subscribes:  /manipulator/cmd      (delta_msgs/ManipulatorCommand)
Publishes:   /manipulator/telemetry (delta_msgs/ManipulatorTelemetry)
             /manipulator/log       (std_msgs/String)   -- non-telemetry serial text lines
"""

import struct

import rclpy
from rclpy.node import Node
import serial

from delta_msgs.msg import ManipulatorCommand, ManipulatorTelemetry
from std_msgs.msg import String

from . import protocol_manager


class ManipulatorBridgeNode(Node):
    def __init__(self):
        super().__init__('manipulator_bridge')

        # ROS2 params replace the old config.py — override via launch file.
        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('poll_period_sec', 0.01)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        poll_period = self.get_parameter('poll_period_sec').value

        # --- ported exactly from ESPCommunicator ---
        self.sync_marker = b'\xaa\x55\xa5\x01'
        self.packet_size = 19
        self._rx_buffer = bytearray()
        self._text_buffer = ""
        # --- end ported constants ---

        self.get_logger().info(f"Opening ESP port {port} at {baud} baud...")
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        self.telemetry_pub = self.create_publisher(ManipulatorTelemetry, '/manipulator/telemetry', 10)
        self.log_pub = self.create_publisher(String, '/manipulator/log', 10)

        self.cmd_sub = self.create_subscription(
            ManipulatorCommand, '/manipulator/cmd', self._on_cmd, 10)

        self.create_timer(poll_period, self._poll_serial)

    # ------------------------------------------------------------------
    # Outbound: ManipulatorCommand -> serial packet
    # ------------------------------------------------------------------
    def _on_cmd(self, msg: ManipulatorCommand):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial port not open, dropping command")
            return

        try:
            if msg.mode == ManipulatorCommand.MODE_COORDINATE:
                packet = protocol_manager.pack_coordinate_command(
                    msg.coordinate_target.x, msg.coordinate_target.y, msg.coordinate_target.z
                )
            else:
                # TODO: this mapping from ManipulatorCommand.mode -> the original
                # string-based `command` field (e.g. "CMD_JOG", "CMD_STOP") needs
                # your actual command vocabulary from protocol.py / your frontend
                # JS — placeholder mapping below, confirm against the real one.
                mode_to_cmd_str = {
                    ManipulatorCommand.MODE_JOINT: "CMD_MOVE_JOINT",
                    ManipulatorCommand.MODE_JOG: "CMD_JOG",
                    ManipulatorCommand.MODE_ESTOP: "CMD_STOP",
                    ManipulatorCommand.MODE_LASER: "CMD_LASER",
                }
                cmd_str = mode_to_cmd_str.get(msg.mode, "CMD_STOP")
                # TODO: motorId ('T' for all axes in the original snippet) and
                # valA/valB/valC mapping depends on cmd_str — placeholder below
                # just forwards jog velocities / joint targets positionally.
                if msg.mode == ManipulatorCommand.MODE_JOG:
                    a, b, c = msg.jog_velocity
                elif msg.mode == ManipulatorCommand.MODE_JOINT:
                    a, b, c = msg.joint_target
                else:
                    a, b, c = 0.0, 0.0, 0.0
                packet = protocol_manager.pack_joint_command(cmd_str, 'T', a, b, c)

            self.ser.write(packet)
        except NotImplementedError:
            self.get_logger().error(
                "protocol_manager packing not implemented yet — paste your real "
                "protocol.py logic into protocol_manager.py"
            )

    # ------------------------------------------------------------------
    # Inbound: serial bytes -> telemetry / log
    # Ported directly from ESPCommunicator.process_serial_loop, restructured
    # from an asyncio while-loop into a ROS2 timer callback. Parsing logic
    # itself (sync marker search, struct unpack, checksum) is unchanged.
    # ------------------------------------------------------------------
    def _poll_serial(self):
        if self.ser is None or self.ser.in_waiting <= 0:
            return

        self._rx_buffer.extend(self.ser.read(self.ser.in_waiting))

        while True:
            sync_idx = self._rx_buffer.find(self.sync_marker)
            if sync_idx != -1:
                if sync_idx > 0:
                    self._text_buffer += self._rx_buffer[:sync_idx].decode('ascii', errors='ignore')
                    del self._rx_buffer[:sync_idx]
                    sync_idx = 0

                if len(self._rx_buffer) >= self.packet_size:
                    packet_bytes = self._rx_buffer[:self.packet_size]
                    if packet_bytes[-1] == 0x0D:
                        struct_data = packet_bytes[2:18]
                        unpacked = struct.unpack('<BBfffH', struct_data)
                        calc_chk = int(unpacked[2] + unpacked[3] + unpacked[4]) & 0xFFFF

                        msg = ManipulatorTelemetry()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.angle_a = round(unpacked[2], 2)
                        msg.angle_b = round(unpacked[3], 2)
                        msg.angle_c = round(unpacked[4], 2)
                        msg.checksum_valid = (calc_chk == unpacked[5])
                        self.telemetry_pub.publish(msg)
                    else:
                        self._text_buffer += chr(self._rx_buffer[0])
                        del self._rx_buffer[:1]
                        continue

                    del self._rx_buffer[:self.packet_size]
                else:
                    break
            else:
                if len(self._rx_buffer) > 3:
                    self._text_buffer += self._rx_buffer[:-3].decode('ascii', errors='ignore')
                    del self._rx_buffer[:-3]
                break

        while '\n' in self._text_buffer:
            line, self._text_buffer = self._text_buffer.split('\n', 1)
            parsed = protocol_manager.parse_esp_log(line)
            if parsed:
                out = String()
                out.data = str(parsed)
                self.log_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
