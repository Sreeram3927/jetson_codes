#!/usr/bin/env python3
"""
autonomy_sequencer_node.py  (service client + event-driven arrival)

Changes from the telemetry-polling version:
    - No more /manipulator/telemetry subscription, no distance/settle math.
    - When IDLE and ready to move again, this node calls the
      GetConfirmedTargets service (async, non-blocking) instead of reading
      from a cached topic subscription -- it only asks for data the moment
      it actually needs it.
    - "Arrived" is now an event: it subscribes to /manipulator/move_result,
      published by your ESP32 serial bridge the moment it parses the
      "reached position" log line coming back over UART. Each outbound
      move carries a command_id; the bridge is expected to echo that id
      back in MoveResult so a late/duplicate log line from a previous move
      can never be mistaken for the current one's completion.

State machine is unchanged in shape: IDLE -> MOVING -> AT_TARGET -> IDLE.
"""

import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool

# Must match the publisher's QoS in frontend_bridge_node -- autonomy_enabled
# is latched (TRANSIENT_LOCAL) so a node starting up after the toggle was
# already set still gets the current value immediately instead of assuming
# False and silently doing nothing.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

from delta_msgs.msg import ManipulatorCommand, MoveResult
from delta_msgs.srv import GetConfirmedTargets


class State(Enum):
    IDLE = auto()
    WAITING_FOR_TARGETS = auto()
    MOVING = auto()
    AT_TARGET = auto()


class AutonomySequencerNode(Node):
    def __init__(self):
        super().__init__('autonomy_sequencer_node')

        self.declare_parameter('laser_dwell_s', 0.5)
        self.laser_dwell_s = self.get_parameter('laser_dwell_s').value

        self._autonomy_enabled = False
        self._base_moving = False
        self._visited = []
        self._state = State.IDLE
        self._current_target = None
        self._at_target_since = None
        # command_id is owned end-to-end by the bridge (it assigns the id at
        # dispatch time and stamps it onto MoveResult) -- we don't generate
        # or match on it here. This node only ever has one command
        # outstanding at a time, and the bridge only ever has one command
        # outstanding *system-wide* (it gates manual vs autonomy commands
        # against each other), so any MoveResult that arrives while we're
        # in MOVING is unambiguously ours.

        self.create_subscription(Bool, '/system/autonomy_enabled', self._on_autonomy_enabled, LATCHED_QOS)
        self.create_subscription(Bool, '/base/motion_state', self._on_motion_state, 10)
        self.create_subscription(MoveResult, '/manipulator/move_result', self._on_move_result, 10)

        self._cmd_pub = self.create_publisher(ManipulatorCommand, '/autonomy/manipulator_cmd', 10)
        self._targets_client = self.create_client(GetConfirmedTargets, '/manipulator/get_confirmed_targets')

        self.create_timer(0.1, self._tick)

        self.get_logger().info('autonomy_sequencer_node up (service client, event-driven arrival)')

    # ------------------------------------------------------------------
    def _on_autonomy_enabled(self, msg: Bool):
        self._autonomy_enabled = msg.data
        if not self._autonomy_enabled and self._state in (State.MOVING, State.AT_TARGET):
            self._abort_to_idle('autonomy disabled mid-motion')

    def _on_motion_state(self, msg: Bool):
        was_moving = self._base_moving
        self._base_moving = msg.data
        if not was_moving and self._base_moving and self._state in (State.MOVING, State.AT_TARGET):
            self._abort_to_idle('base started moving mid-motion')
        if was_moving and not self._base_moving:
            self._visited.clear()

    def _on_move_result(self, msg: MoveResult):
        if self._state != State.MOVING:
            return  # not waiting on anything -- ignore (e.g. result for a manual command)
        if not msg.success:
            self.get_logger().warn(f'move {msg.command_id} reported failure: {msg.message}')
            self._abort_to_idle('ESP32 reported move failure')
            return
        self._state = State.AT_TARGET
        self._at_target_since = time.monotonic()
        self._fire_laser()

    # ------------------------------------------------------------------
    def _abort_to_idle(self, reason: str):
        self.get_logger().info(f'aborting current move: {reason}')
        self._publish_stop()
        self._state = State.IDLE
        self._current_target = None

    def _tick(self):
        if self._state == State.IDLE:
            if self._autonomy_enabled and not self._base_moving:
                self._request_targets()

        elif self._state == State.AT_TARGET:
            if time.monotonic() - self._at_target_since >= self.laser_dwell_s:
                self._visited.append(self._current_target)
                self._current_target = None
                self._state = State.IDLE

        # MOVING and WAITING_FOR_TARGETS just wait on callbacks -- nothing to poll

    def _request_targets(self):
        if not self._targets_client.service_is_ready():
            return
        self._state = State.WAITING_FOR_TARGETS
        future = self._targets_client.call_async(GetConfirmedTargets.Request())
        future.add_done_callback(self._on_targets_response)

    def _on_targets_response(self, future):
        if self._state != State.WAITING_FOR_TARGETS:
            return  # got aborted while the request was in flight
        try:
            targets = [(t.x, t.y, t.z) for t in future.result().targets]
        except Exception as exc:
            self.get_logger().warn(f'get_confirmed_targets call failed: {exc}')
            self._state = State.IDLE
            return

        remaining = [t for t in targets if not any(self._dist(t, v) < 0.02 for v in self._visited)]
        if not remaining:
            self._state = State.IDLE
            return

        nxt = min(remaining, key=lambda t: self._dist(t, self._current_target or t))
        self._current_target = nxt
        self._publish_move(nxt)
        self._state = State.MOVING
        self.get_logger().info(f'moving to target {nxt}')

    @staticmethod
    def _dist(a, b):
        return sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5

    # ------------------------------------------------------------------
    def _publish_move(self, xyz):
        cmd = ManipulatorCommand()
        cmd.command = 'CMD_MOVE_COORDINATE'
        # no command_id set -- the bridge assigns the real one at dispatch
        # time and stamps it into the MoveResult it publishes back
        cmd.x, cmd.y, cmd.z = xyz
        self._cmd_pub.publish(cmd)

    def _publish_stop(self):
        cmd = ManipulatorCommand()
        cmd.command = 'CMD_STOP'
        self._cmd_pub.publish(cmd)

    def _fire_laser(self):
        self.get_logger().info(f'at target {self._current_target} -- fire laser here')


def main():
    rclpy.init()
    node = AutonomySequencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()