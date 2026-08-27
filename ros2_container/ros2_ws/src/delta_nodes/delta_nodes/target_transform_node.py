"""
target_transform_node

Subscribes:  /camera/targets_raw          (delta_msgs/TargetArray, camera frame, meters)
Publishes:   /camera/targets_transformed  (delta_msgs/TargetArray, manipulator frame, MILLIMETERS
                                            -- see unit note in frame_transformation.py)

Uses your existing calibrated FrameTransformation.getTransformedCoordinates()
unchanged — this node is just the ROS2 plumbing around it.
"""

import rclpy
from rclpy.node import Node

from delta_msgs.msg import TargetArray, Target

from .frame_transformation import FrameTransformation


class TargetTransformNode(Node):
    def __init__(self):
        super().__init__('target_transform')

        self.transformer = FrameTransformation()

        self.sub = self.create_subscription(
            TargetArray, '/camera/targets', self._on_targets, 10)
        self.pub = self.create_publisher(
            TargetArray, '/manipulator/targets', 10)

    def _on_targets(self, msg: TargetArray):
        out = TargetArray()
        out.header = msg.header

        for t in msg.targets:
            camera_point = [t.position.x, t.position.y, t.position.z]
            x, y, z = self.transformer.getTransformedCoordinates(camera_point)

            transformed = Target()
            transformed.id = t.id
            transformed.class_name = t.class_name
            transformed.confidence = t.confidence
            transformed.position.x = x
            transformed.position.y = y
            transformed.position.z = z
            out.targets.append(transformed)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
