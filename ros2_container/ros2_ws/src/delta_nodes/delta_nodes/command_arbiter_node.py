"""
command_arbiter_node

The single place that decides what actually gets sent to the manipulator:
autonomous targeting vs. manual/frontend commands.

Subscribes:
  /manipulator/target_selected        (geometry_msgs/PointStamped)  -- autonomous
  /frontend/manual_manipulator_cmd    (delta_msgs/ManipulatorCommand) -- manual, from frontend_bridge
  /system/autonomy_enabled            (std_msgs/Bool)
  /base/motion_state                  (std_msgs/UInt8)  0=STOPPED 1=MOVING

Publishes:
  /manipulator/cmd                    (delta_msgs/ManipulatorCommand) -- to manipulator_bridge

Gating rule: an autonomous target is only forwarded as a COORDINATE command
if autonomy_enabled AND base is STOPPED. Manual commands always pass through
immediately regardless of gate state (operator override always wins).

TODO: no explicit precedence/lockout if both manual and autonomous commands
arrive close together — currently whichever callback fires last wins. Add
a short manual-override lockout window if that turns out to cause fighting
between the two sources in practice.
"""

import rclpy
from rclpy.node import Node

from delta_msgs.msg import ManipulatorCommand
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, UInt8

DEFAULT_AUTONOMOUS_FEED_RATE = 0.05  # m/s -- TODO: tune


class CommandArbiterNode(Node):
    def __init__(self):
        super().__init__('command_arbiter')

        self.autonomy_enabled = False
        # self.base_motion_state = 1  # default to MOVING (safe: gate closed) until we hear otherwise

        self.cmd_pub = self.create_publisher(ManipulatorCommand, '/manipulator/cmd', 10)

        self.create_subscription(PointStamped, '/manipulator/target_selected', self._on_target, 10)
        self.create_subscription(ManipulatorCommand, '/frontend/manual_manipulator_cmd', self._on_manual, 10)
        self.create_subscription(Bool, '/system/autonomy_enabled', self._on_autonomy_enabled, 10)
        # self.create_subscription(UInt8, '/base/motion_state', self._on_motion_state, 10)

    def _on_autonomy_enabled(self, msg: Bool):
        self.autonomy_enabled = msg.data

    # def _on_motion_state(self, msg: UInt8):
    #     self.base_motion_state = msg.data

    def _on_manual(self, msg: ManipulatorCommand):
        # Manual always passes straight through.
        self.cmd_pub.publish(msg)

    def _on_target(self, msg: PointStamped):
        if not self.autonomy_enabled:
            return
        if self.base_motion_state != 0:  # not STOPPED
            return

        cmd = ManipulatorCommand()
        cmd.mode = ManipulatorCommand.CMD_MOVE_COORDINATE
        cmd.coordinate_target = msg.point
        # cmd.feed_rate = DEFAULT_AUTONOMOUS_FEED_RATE
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CommandArbiterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
