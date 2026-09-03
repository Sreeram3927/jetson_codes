"""
frontend_bridge_node

Websocket SERVER for the frontend — direct successor to the websocket half
of your existing ESPCommunicator (ws_handler/broadcast_ws). Only responsible
for frontend<->ROS2 translation; manipulator_bridge_node owns the ESP32
serial connection, mobile_base_bridge_node owns the Arduino serial connection.

Wire schema below was reverse-engineered from your actual frontend code
(bridge.ts) rather than invented — see inline notes for the spots that
still need your confirmation.

Inbound (frontend -> ROS2), matches bridge.ts's wsXxxCommand() functions:
  {"target":"esp","command":"CMD_MOVE_ABSOLUTE","motorId":"T","valA":..,"valB":..,"valC":..}
  {"target":"esp","command":"CMD_JOG_RELATIVE","motorId":"T"|"A"|"B"|"C","valA":..,"valB":..,"valC":..}
    -> motorId "T": jog all three axes at once, [valA, valB, valC]
    -> motorId "A"/"B"/"C": jog only that axis, using valA/valB/valC
       respectively as the delta, others forced to 0
  {"target":"esp","command":"CMD_MOVE_COORDINATE","x":..,"y":..,"z":..}
  {"target":"esp","command":"CMD_STOP"}
  {"target":"arduino","command":"SET_LASER","power":50}
  {"target":"arduino","command":"SET_MAX_SPEED","value":60}
  {"target":"arduino","command":"SET_RAMP_RATE","accel":4,"decel":6}
  {"target":"arduino","command":"SOFT_STOP"}
    -- CHANGED from the old raw-string format (e.g. "L50"): the Arduino no
       longer runs the old single-letter text protocol, so 'command' here
       is a structured dict forwarded as-is (minus "target") to
       mobile_base_bridge_node, which dispatches by 'command' name. Update
       wsLaserCommand() and friends in bridge.ts to send this shape instead
       of raw strings.
  {"target":"base","command":"DRIVE","linear":-1.0..1.0,"angular":-1.0..1.0}
    -- NEW: publishes a Twist to /base/cmd_vel (linear.x, angular.z), which
       mobile_base_bridge_node scales into the Arduino's DRIVE command. This
       is the ONLY way to drive the base from the frontend — the old
       single-letter 'f'/'b'/'l'/'r' commands sent via target="arduino" are
       gone, since the Arduino's DRIVE command now takes a continuous
       linear/angular pair, not a direction letter. There's no separate stop
       message: releasing a drive button should send DRIVE with
       linear=0, angular=0, which ramps the base down using its configured
       decel rate (same effect as SOFT_STOP). IMPORTANT: because the
       firmware's comms watchdog soft-stops after 300ms of silence, whatever
       calls this while a button is held must resend it at an interval
       shorter than that (e.g. every 100-150ms), not just once on press.
  {"type":"set_autonomy","enabled":true|false}   # kept from original design;
                                                   # not seen in bridge.ts —
                                                   # confirm this still exists
                                                   # on the frontend side.

Outbound (ROS2 -> frontend), matches bridge.ts's parseBridgeMessages():
  {"type":"telemetry","timestamp":...,"j1":..,"j2":..,"j3":..}
  {"type":"target_locations","targets":[{"x":..,"y":..,"conf":..}, ...]}
    (relayed from /vision/detections, published by detection_relay —
     camera-frame x/y right now, no transform applied; see note below)
  {"type":"base_status","moving":bool,"estop_active":bool,"laser_pct":..,
   "linear_pct":..,"angular_pct":..}
    -- NEW: relayed from /base/status (delta_msgs/BaseStatus), published by
       mobile_base_bridge_node from the Arduino's telemetry. bridge.ts's
       parseBridgeMessages() needs a case added for "base_status" — this
       is the E-stop/laser/motion indicator for the operator UI.
"""

import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
import websockets

from geometry_msgs.msg import Twist
from delta_msgs.msg import ManipulatorCommand, ManipulatorTelemetry, TargetArray, BaseStatus
from std_msgs.msg import Bool, String, UInt8

TELEMETRY_RATE_HZ = 10


class FrontendBridgeNode(Node):
    def __init__(self):
        super().__init__('frontend_bridge')

        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)

        self._latest_telemetry = ManipulatorTelemetry()
        self._latest_motion_state = 0
        self._latest_autonomy_enabled = False
        self._latest_base_status = BaseStatus()

        self.manipulator_cmd_pub = self.create_publisher(ManipulatorCommand, '/frontend/manipulator_cmd', 10)
        self.base_cmd_pub = self.create_publisher(String, '/frontend/base_cmd', 10)
        self.autonomy_pub = self.create_publisher(Bool, '/system/autonomy_enabled', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/base/cmd_vel', 10)

        self.create_subscription(ManipulatorTelemetry, '/manipulator/telemetry', self._on_telemetry, 10)
        self.create_subscription(UInt8, '/base/motion_state', self._on_motion_state, 10)
        self.create_subscription(Bool, '/system/autonomy_enabled', self._on_autonomy_state, 10)
        self.create_subscription(TargetArray, '/manipulator/targets', self._on_target_detections, 10)
        self.create_subscription(BaseStatus, '/base/status', self._on_base_status, 10)

        self.create_subscription(String, '/manipulator/log', self._on_log, 10)
        self.create_subscription(String, '/base/log', self._on_log, 10)

        self._clients = set()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        self.create_timer(1.0 / TELEMETRY_RATE_HZ, self._broadcast_telemetry)
        self.create_timer(1.0 / TELEMETRY_RATE_HZ, self._broadcast_base_status)

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self):
        host = self.get_parameter('ws_host').value
        port = self.get_parameter('ws_port').value
        async with websockets.serve(self._ws_handler, host, port):
            self.get_logger().info(f"Frontend websocket server listening on {host}:{port}")
            await asyncio.Future()  # run forever

    async def _ws_handler(self, websocket, path):
        self._clients.add(websocket)
        try:
            async for message in websocket:
                self._handle_message(message)
        except websockets.exceptions.ConnectionClosedError:
            pass
        finally:
            self._clients.discard(websocket)

    # ------------------------------------------------------------------
    # Inbound
    # ------------------------------------------------------------------
    def _handle_message(self, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON from frontend")
            return

        # Kept from the original design — not present in bridge.ts as
        # shown, so this path is unverified. Remove if the frontend
        # doesn't actually have an autonomy toggle.
        if data.get('type') == 'set_autonomy':
            out = Bool()
            out.data = bool(data.get('enabled', False))
            self.autonomy_pub.publish(out)
            return

        target = data.get('target')
        command = data.get('command')

        if target == 'esp':
            self._handle_esp_command(command, data)
        elif target == 'arduino':
            self._handle_arduino_command(data)
        elif target == 'base':
            self._handle_base_command(command, data)
        else:
            self.get_logger().warn(f"Unrecognized message from frontend: {data!r}")

    def _handle_esp_command(self, command, data):
        cmd = ManipulatorCommand()

        if command == 'CMD_MOVE_ABSOLUTE':
            cmd.mode = ManipulatorCommand.CMD_MOVE_ABSOLUTE
            cmd.motor_id = data.get('motorId', 'T')
            cmd.joint_target = [
                float(data.get('valA', 0.0)),
                float(data.get('valB', 0.0)),
                float(data.get('valC', 0.0)),
            ]

        elif command == 'CMD_JOG_RELATIVE':
            cmd.mode = ManipulatorCommand.CMD_JOG_RELATIVE
            cmd.motor_id = data.get('motorId', 'T')
            cmd.joint_target = [
                float(data.get('valA', 0.0)),
                float(data.get('valB', 0.0)),
                float(data.get('valC', 0.0)),
            ]

        elif command == 'CMD_MOVE_COORDINATE':
            cmd.mode = ManipulatorCommand.CMD_MOVE_COORDINATE
            cmd.coordinate_target.x = float(data.get('x', 0.0))
            cmd.coordinate_target.y = float(data.get('y', 0.0))
            cmd.coordinate_target.z = float(data.get('z', 0.0))

        elif command == 'CMD_STOP':
            cmd.mode = ManipulatorCommand.CMD_STOP

        else:
            self.get_logger().warn(f"Unknown esp command: {command}")
            return

        self.manipulator_cmd_pub.publish(cmd)

    def _handle_arduino_command(self, data: dict):
        forward = {k: v for k, v in data.items() if k != 'target'}
        if 'command' not in forward:
            self.get_logger().warn(f"Arduino message missing 'command': {data!r}")
            return
        out = String()
        out.data = json.dumps(forward)
        self.base_cmd_pub.publish(out)

    def _handle_base_command(self, command, data):
        if command != 'DRIVE':
            self.get_logger().warn(f"Unknown base command: {command!r}")
            return

        twist = Twist()
        try:
            twist.linear.x = float(data.get('linear', 0.0))
            twist.angular.z = float(data.get('angular', 0.0))
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid DRIVE payload from frontend: {data!r}")
            return

        self.cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Outbound
    # ------------------------------------------------------------------
    def _on_telemetry(self, msg: ManipulatorTelemetry):
        self._latest_telemetry = msg

    def _on_motion_state(self, msg: UInt8):
        self._latest_motion_state = msg.data

    def _on_autonomy_state(self, msg: Bool):
        self._latest_autonomy_enabled = msg.data

    def _on_base_status(self, msg: BaseStatus):
        self._latest_base_status = msg

    def _broadcast_telemetry(self):
        if not self._clients:
            return

        t = self._latest_telemetry
        payload = json.dumps({
            "type": "telemetry",
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "j1": t.angle_a,
            "j2": t.angle_b,
            "j3": t.angle_c,
        })

        asyncio.run_coroutine_threadsafe(self._send_to_all(payload), self._loop)

    def _broadcast_base_status(self):
        if not self._clients:
            return

        s = self._latest_base_status
        payload = json.dumps({
            "type": "base_status",
            "moving": bool(s.moving),
            "estop_active": bool(s.estop_active),
            "laser_pct": int(s.laser_pct),
            "linear_pct": int(s.linear_pct),
            "angular_pct": int(s.angular_pct),
        })

        asyncio.run_coroutine_threadsafe(self._send_to_all(payload), self._loop)

    def _on_target_detections(self, msg: TargetArray):

        targets = []
        for t in msg.targets:
            targets.append({
                "id": int(t.id),
                "class_name": t.class_name,
                "x": float(t.position.x),
                "y": float(t.position.y),
                "z": float(t.position.z),
                "confidence": float(t.confidence),
            })

        payload = json.dumps({
            "type": "target_locations",
            "targets": targets,
        })

        asyncio.run_coroutine_threadsafe(self._send_to_all(payload), self._loop)

    def _on_log(self, data: String):
        asyncio.run_coroutine_threadsafe(self._send_to_all(data.data), self._loop)
    
    async def _send_to_all(self, payload: str):
        if not self._clients:
            return
        results = await asyncio.gather(
            *(c.send(payload) for c in list(self._clients)),
            return_exceptions=True
        )
        for r in results:
            if isinstance(r, Exception):
                self.get_logger().warn(f"Broadcast warning: {r}")


def main(args=None):
    rclpy.init(args=args)
    node = FrontendBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()