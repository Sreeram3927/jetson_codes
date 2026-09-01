"""
frame_transformation.py

Ported directly from your existing transformations.py (FrameTransformation
class) — calibration matrices, offsets, and axis-swap logic are unchanged.
Kept as its own module (not folded into the ROS2 node) so it stays a plain,
testable unit exactly like the original.

NOTE ON UNITS: this class outputs manipulator-frame coordinates in
millimeters (matches your original code — x1000 conversion, Z hardcoded to
620.0mm in the homography path). That's carried through as-is into
target_transform_node.py's published Target.position, even though
geometry_msgs/Point is conventionally meters elsewhere in ROS — flagging this
explicitly since it's a likely source of confusion downstream (e.g. if you
later feed this into tf2 for visualization, tf2 assumes meters).
"""

import numpy as np
import cv2


class FrameTransformation:
    def __init__(self):
        self.T_cam_to_rob = np.array([
            [-0.05207205, -0.81761635, 0.57340388, -0.26935267],
            [0.99864204, -0.04355581, 0.02858263, -0.03880375],
            [0.00160545, 0.57411358, 0.8187741, -0.62764725],
            [0., 0., 0., 1.]
        ])
        self.H_cam_to_rob = np.array([
            [0.001985, -0.000204, -0.611338],
            [0.000074, 0.002222, -0.736594],
            [0.000351, 0.002219, 1.]
        ])

    def homography_to_robotFrame(self, u_pixel, v_pixel):
        weed_pixel = np.array([[[float(u_pixel), float(v_pixel)]]], dtype=np.float32)
        robot_target = cv2.perspectiveTransform(weed_pixel, self.H_cam_to_rob)
        target_x_m = robot_target[0][0][0]
        target_y_m = robot_target[0][0][1]
        x_rob = round(target_x_m * 1000.0, 2)
        y_rob = round(target_y_m * 1000.0, 2)
        z_rob = 620.0
        return x_rob, y_rob, z_rob

    def offset_to_manipulatorFrame(self, x, y, z):
        x, y = y, x
        y -= 38
        y *= -1
        return float(x), float(y), float(z)

    def transform_to_robotFrame(self, camera_point):
        point = np.array([camera_point[0], camera_point[1], camera_point[2], 1.0])
        robot = self.T_cam_to_rob @ point
        return (robot[0] * 1000, robot[1] * 1000, robot[2] * 1000)

    def getTransformedCoordinates(self, camera_point):
        x_rob, y_rob, z_rob = self.transform_to_robotFrame(camera_point)
        x, y, z = self.offset_to_manipulatorFrame(x_rob, y_rob, z_rob)
        return x, y, z

    def performHomography(self, u_pixel, v_pixel):
        x_rob, y_rob, z_rob = self.homography_to_robotFrame(u_pixel, v_pixel)
        return self.offset_to_manipulatorFrame(x_rob, y_rob, z_rob)
