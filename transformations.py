import numpy as np
import cv2

class FrameTransformation:

	def __init__(self):
		# self.T_cam_to_rob = np.array([
		#     [ 0.02398, -0.99616, -0.08425,  0.13545],
		#     [ 0.79066, -0.03268,  0.61138,  0.08134],
		#     [-0.61179, -0.08128,  0.78684,  0.67031],
		#     [ 0.     ,  0.     ,  0.     ,  1.     ]
		# ])
		self.T_cam_to_rob = np.array([
			[ 0.001985, -0.000204, -0.611338],
			[ 0.000074,  0.002222, -0.736594],
			[ 0.000351,  0.002219,  1.      ]
		])

	def transform_to_robotFrame(self, u_pixel, v_pixel):
		"""
		Converts 2D Camera Pixels directly to physical Delta Robot mm using Homography.
		"""
		# 1. Format the pixel for OpenCV (Must be shape: 1, 1, 2)
		weed_pixel = np.array([[[float(u_pixel), float(v_pixel)]]], dtype=np.float32)
		
		# 2. Apply the 3x3 Homography Matrix (Replaces np.matmul)
		robot_target = cv2.perspectiveTransform(weed_pixel, self.T_cam_to_rob)
		
		# 3. Extract the output. 
		# CRITICAL: Because your calibration used meters (e.g., 0.051), 
		# this output is currently in METERS.
		target_x_m = robot_target[0][0][0]
		target_y_m = robot_target[0][0][1]
		
		# 4. Convert meters to millimeters for the ESP32 (multiply by 1000)
		x_rob = round(target_x_m * 1000.0, 2)
		y_rob = round(target_y_m * 1000.0, 2)
		
		# 5. Z is hardcoded to your flat table height in mm
		z_rob = 620.0 
		
		return x_rob, y_rob, z_rob
    
	def transform_to_manipulatorFrame(self, x, y, z):
		"""
		Transforms coordinates from the Robot Frame to the Manipulator Frame.
		- The manipulator origin is at (0, 0, -45) in the Robot Frame.
		- The Z-axis orientation remains the same.
		- The X and Y axes are swapped.
		"""
		return float(x), float(y), float(z)
		# 1. Swap the X and Y axes
		# x_m = y
		# y_m = x
		
		# # 2. Shift the Z axis
		# # Subtracting the origin position: z - (-45) = z + 45
		# z_m = z + 45.0
		
		# # 3. Return as floats for consistency
		# return float(x_m), float(y_m), float(z_m)

	def getTransformedCoordinates(self, u_pixel, v_pixel):
		x_rob, y_rob, z_rob = self.transform_to_robotFrame(u_pixel, v_pixel)
		x_man, y_man, z_man = self.transform_to_manipulatorFrame(x_rob, y_rob, z_rob)

		return x_man, y_man, z_man