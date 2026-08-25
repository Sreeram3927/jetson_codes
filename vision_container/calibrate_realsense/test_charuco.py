import pyrealsense2 as rs
import numpy as np
import cv2
import time

# --- 1. Setup the ChArUco Board (OpenCV 4.7+ syntax) ---
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Parameters from your PDF: 9x7 squares, 15mm square, 11mm marker
squares_x = 9
squares_y = 7
square_length_m = 0.020
marker_length_m = 0.015

# Define the board
board = cv2.aruco.CharucoBoard(
    (squares_x, squares_y), 
    square_length_m, 
    marker_length_m, 
    dictionary
)

# Initialize the detector
detector_params = cv2.aruco.DetectorParameters()
charuco_detector = cv2.aruco.CharucoDetector(board)

ROBOT_POINTS = np.array([
    [-0.085,  0.093, 0.620],  # 0: Bottom-Left
    [-0.077, -0.085, 0.620],  # 1: Top-Left
    [ 0.062, -0.081, 0.620],  # 2: Top-Right
    [ 0.051,  0.094, 0.620]   # 3: Bottom-Right
], dtype=np.float32)

# --- 2. Initialize RealSense Camera ---
pipeline = rs.pipeline()
config = rs.config()

# Enable color stream (640x480 is standard, bump to 1280x720 if you need more range)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

print("Starting RealSense camera...")
profile = pipeline.start(config)

intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                          [0, intrinsics.fy, intrinsics.ppy],
                          [0, 0, 1]])
dist_coeffs = np.zeros((4, 1))
# Give the camera a second to auto-expose
time.sleep(2.0)
print("Camera running. Searching for ChArUco board...")
print("NOTE: Running in headless Docker mode. I will save an image when detected.")

try:
    frames_processed = 0
    while True:
        # Wait for a coherent color frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert image to numpy array for OpenCV
        color_image = np.asanyarray(color_frame.get_data())

        # --- 3. Detect the Board ---
        # detectBoard returns: charuco_corners, charuco_ids, marker_corners, marker_ids
        charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(color_image)

        # Check if any individual ArUco markers were found
        if marker_ids is not None and len(marker_ids) > 0:
            print(f"\nSUCCESS: Detected {len(marker_ids)} individual ArUco markers!")
            # Draw green boxes around the individual markers
            cv2.aruco.drawDetectedMarkers(color_image, marker_corners, marker_ids)
            
            # Check if we also found the high-precision ChArUco checkerboard intersections
            if charuco_ids is not None and len(charuco_ids) > 0:
                print(f"SUCCESS: Detected {len(charuco_ids)} ChArUco sub-pixel corners!")
                # Draw blue squares at the checkerboard intersections
                cv2.aruco.drawDetectedCornersCharuco(color_image, charuco_corners, charuco_ids, (255, 0, 0))
                obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
            success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

            if success:
                R, _ = cv2.Rodrigues(rvec) 
                T_camera_to_board = np.eye(4) 
                T_camera_to_board[:3, :3] = R 
                T_camera_to_board[:3, 3] = tvec.flatten() 

                # --- 5. Generate Camera Points for the 4 Corners ---
                board_w = squares_x * square_length_m
                board_h = squares_y * square_length_m
                
                local_corners = np.array([
                    [0.0,     0.0,     0.0, 1.0], 
                    [board_w, 0.0,     0.0, 1.0], 
                    [board_w, board_h, 0.0, 1.0], 
                    [0.0,     board_h, 0.0, 1.0]  
                ])

                CAMERA_POINTS = []
                for corner in local_corners:
                    cam_pt = np.dot(T_camera_to_board, corner)
                    CAMERA_POINTS.append(cam_pt[:3])
                
                CAMERA_POINTS = np.array(CAMERA_POINTS, dtype=np.float32)

                # --- 6. Calculate Final Matrix ---
                success_affine, affine_matrix, _ = cv2.estimateAffine3D(CAMERA_POINTS, ROBOT_POINTS)
                
                if success_affine:
                    T_camera_to_robot = np.vstack((affine_matrix, [0, 0, 0, 1]))
                    print("\n=== MASTER CALIBRATION SUCCESSFUL ===")
                    print("Your Final 4x4 T_camera_to_robot Matrix:")
                    np.set_printoptions(suppress=True, precision=5)
                    print(repr(T_camera_to_robot))
                    break # Break the loop once calibration is successful
                else:
                    print("Affine3D calculation failed. Check ROBOT_POINTS arrays.")
                    break

            # --- 4. Save Image and Exit ---
            output_path = "/workspace/detection_test.jpg"
            cv2.imwrite(output_path, color_image)
            print(f"Saved visualization with drawn markers to: {output_path}")
            print("Exiting test script.")
            break 

        # Terminal feedback so you know the script hasn't frozen
        frames_processed += 1
        if frames_processed % 30 == 0:  # Print roughly every 1 second
            print("Still searching... Please hold the board in front of the camera.")

finally:
    # Stop streaming safely
    pipeline.stop()