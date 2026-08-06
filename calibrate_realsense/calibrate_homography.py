import pyrealsense2 as rs
import numpy as np
import cv2
import time

# ==========================================
# 1. YOUR ROBOT MEASUREMENTS (The 4 Outer Corners)
# ==========================================
# These are the corners of the 7x9 grid (140mm x 180mm) you measured earlier.
ROBOT_CORNERS = np.array([
    [-0.085,  0.093],  # Bottom-Left
    [ 0.051,  0.094],  # Bottom-Right
    [ 0.062, -0.081],  # Top-Right
    [-0.077, -0.085]   # Top-Left
], dtype=np.float32)

# Mathematical definition of the board based on your 20mm squares
LOCAL_CORNERS = np.array([
    [0.0,   0.0],   # Bottom-Left
    [0.140, 0.0],   # Bottom-Right
    [0.140, 0.180], # Top-Right
    [0.0,   0.180]  # Top-Left
], dtype=np.float32)

# Create a bridge between the mathematical board and your physical robot
T_local_to_robot = cv2.getPerspectiveTransform(LOCAL_CORNERS, ROBOT_CORNERS)

# ==========================================
# 2. THE MARKER MAP (col, row)
# ==========================================
# This maps every ID to its [column, row] position on the board.
MARKER_MAP = {
    31:[0,1], 30:[0,3], 29:[0,5], 28:[0,7],
    27:[1,0], 26:[1,2], 25:[1,4], 24:[1,6], 23:[1,8],
    22:[2,1], 21:[2,3], 20:[2,5], 19:[2,7],
    18:[3,0], 17:[3,2], 16:[3,4], 15:[3,6], 14:[3,8],
    13:[4,1], 12:[4,3], 11:[4,5], 10:[4,7],
    9:[5,0],  8:[5,2],  7:[5,4],  6:[5,6],  5:[5,8],
    4:[6,1],  3:[6,3],  2:[6,5],  1:[6,7]
}

# --- 3. Initialize RealSense Camera ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

print("Starting RealSense camera...")
pipeline.start(config)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

time.sleep(2.0)
print("Looking for ANY markers to calculate Homography...")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue
        
        color_image = np.asanyarray(color_frame.get_data())

        # Detect markers
        marker_corners, marker_ids, _ = detector.detectMarkers(color_image)

        if marker_ids is not None and len(marker_ids) >= 4:
            found_ids = marker_ids.flatten()
            
            camera_pixels = []
            robot_targets = []
            
            for i, marker_id in enumerate(found_ids):
                if marker_id in MARKER_MAP:
                    # 1. Get the center pixel of the marker on the camera
                    corners = marker_corners[i][0]
                    center_u = np.mean(corners[:, 0])
                    center_v = np.mean(corners[:, 1])
                    camera_pixels.append([center_u, center_v])
                    
                    # 2. Calculate exactly where this marker is in the robot's physical space
                    col, row = MARKER_MAP[marker_id]
                    local_x = (col * 0.020) + 0.010  # 20mm squares, center is at +10mm
                    local_y = (row * 0.020) + 0.010
                    
                    local_pt = np.array([[[local_x, local_y]]], dtype=np.float32)
                    robot_pt = cv2.perspectiveTransform(local_pt, T_local_to_robot)
                    
                    robot_targets.append([robot_pt[0][0][0], robot_pt[0][0][1]])
                    
                    cv2.circle(color_image, (int(center_u), int(center_v)), 4, (0, 255, 0), -1)

            # We need at least 4 known markers to calculate the Master Matrix
            if len(camera_pixels) >= 4:
                src_pts = np.array(camera_pixels, dtype=np.float32)
                dst_pts = np.array(robot_targets, dtype=np.float32)
                
                # --- 4. CALCULATE THE MASTER MATRIX ---
                # This finds the single 3x3 matrix that maps pixels -> robot motors
                Homography_Matrix, _ = cv2.findHomography(src_pts, dst_pts)
                
                print(f"\nSUCCESS: Used {len(camera_pixels)} markers to build the bridge!")
                print("\n=== YOUR MASTER 3x3 HOMOGRAPHY MATRIX ===")
                np.set_printoptions(suppress=True, precision=6)
                print(repr(Homography_Matrix))
                
                cv2.imwrite("/workspace/homography_success.jpg", color_image)
                print("\nSaved visualization to /workspace/homography_success.jpg")
                break
                
        time.sleep(0.1)

finally:
    pipeline.stop()