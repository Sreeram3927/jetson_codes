
#!/usr/bin/env python3
"""
target_tracker_node.py  (service-based version)

Same deduplication logic as before (nearest-neighbor association + EMA
smoothing + confirm-after-N-hits + timeout pruning + clear-on-base-move),
but confirmed targets are no longer pushed out on a topic every tick.
Instead this node runs a GetConfirmedTargets service. Consumers (the
sequencer) call it exactly when they're ready to pick a next target --
no continuous chatter, no stale-cache bugs, and no need for the consumer
to buffer/track the latest message itself.

Interface (place in your delta_msgs/srv/GetConfirmedTargets.srv):
    ---
    delta_msgs/Target[] targets

Add to CMakeLists.txt:  rosidl_generate_interfaces(... "srv/GetConfirmedTargets.srv" ...)
"""

import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from delta_msgs.msg import Target, TargetArray
from delta_msgs.srv import GetConfirmedTargets


@dataclass
class Track:
    id: int
    x: float
    y: float
    z: float
    hits: int = 1
    last_seen: float = field(default_factory=time.monotonic)


class TargetTrackerNode(Node):
    def __init__(self):
        super().__init__('target_tracker_node')

        self.declare_parameter('assoc_distance', 0.03)
        self.declare_parameter('confirm_hits', 5)
        self.declare_parameter('track_timeout', 1.0)
        self.declare_parameter('ema_alpha', 0.3)

        self.assoc_distance = self.get_parameter('assoc_distance').value
        self.confirm_hits = self.get_parameter('confirm_hits').value
        self.track_timeout = self.get_parameter('track_timeout').value
        self.ema_alpha = self.get_parameter('ema_alpha').value

        self._tracks: dict[int, Track] = {}
        self._next_id = 0
        self._base_moving = False

        self.create_subscription(TargetArray, '/manipulator/targets', self._on_targets, 10)
        self.create_subscription(Bool, '/base/motion_state', self._on_motion_state, 10)
        self.create_timer(0.2, self._prune_stale_tracks)

        self.create_service(
            GetConfirmedTargets, '/manipulator/get_confirmed_targets', self._handle_get_confirmed)

        self.get_logger().info('target_tracker_node up (service-backed)')

    # ------------------------------------------------------------------
    def _on_motion_state(self, msg: Bool):
        was_moving = self._base_moving
        self._base_moving = msg.data
        if not was_moving and self._base_moving and self._tracks:
            self.get_logger().info(f'base moving -- clearing {len(self._tracks)} track(s)')
            self._tracks.clear()

    def _on_targets(self, msg: TargetArray):
        if self._base_moving:
            return

        now = time.monotonic()
        for det in msg.targets:
            dx, dy, dz = det.position.x, det.position.y, -380
            best_id, best_dist = None, self.assoc_distance
            for tid, tr in self._tracks.items():
                d = ((tr.x - dx) ** 2 + (tr.y - dy) ** 2 + (tr.z - dz) ** 2) ** 0.5
                if d < best_dist:
                    best_id, best_dist = tid, d

            if best_id is not None:
                tr = self._tracks[best_id]
                a = self.ema_alpha
                tr.x = (1 - a) * tr.x + a * dx
                tr.y = (1 - a) * tr.y + a * dy
                tr.z = (1 - a) * tr.z + a * dz
                tr.hits += 1
                tr.last_seen = now
            else:
                self._tracks[self._next_id] = Track(id=self._next_id, x=dx, y=dy, z=dz, last_seen=now)
                self._next_id += 1

    def _prune_stale_tracks(self):
        now = time.monotonic()
        stale = [tid for tid, tr in self._tracks.items() if now - tr.last_seen > self.track_timeout]
        for tid in stale:
            del self._tracks[tid]

    # ------------------------------------------------------------------
    def _handle_get_confirmed(self, request, response):
        confirmed = [tr for tr in self._tracks.values() if tr.hits >= self.confirm_hits]
        response.targets = [self._make_target_msg(tr) for tr in confirmed]
        return response

    @staticmethod
    def _make_target_msg(tr: Track) -> Target:
        t = Target()
        t.position.x, t.position.y, t.position.z = tr.x, tr.y, tr.z
        return t


def main():
    rclpy.init()
    node = TargetTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()