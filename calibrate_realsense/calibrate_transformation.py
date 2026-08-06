import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import time

# ---------------- BOARD SETTINGS ----------------

SQUARE_SIZE = 0.0197      # metres
MARKER_SIZE = 0.0144      # CHANGE if your printed marker size differs

# Marker Corners in Robot Frame
ROBOT_CORNERS = np.array([
    [-0.070,  0.096],
    [ 0.106,  0.106],
    [ 0.118, -0.029],
    [-0.059, -0.040]
], dtype=np.float32)

LOCAL_CORNERS = np.array([
    [0.0,   0.0],
    [0.140, 0.0],
    [0.140, 0.180],
    [0.0,   0.180]
], dtype=np.float32)

T_LOCAL_TO_ROBOT = cv2.getPerspectiveTransform(LOCAL_CORNERS, ROBOT_CORNERS)

MARKER_MAP = {
    31:[0,1],30:[0,3],29:[0,5],28:[0,7],
    27:[1,0],26:[1,2],25:[1,4],24:[1,6],23:[1,8],
    22:[2,1],21:[2,3],20:[2,5],19:[2,7],
    18:[3,0],17:[3,2],16:[3,4],15:[3,6],14:[3,8],
    13:[4,1],12:[4,3],11:[4,5],10:[4,7],
    9:[5,0],8:[5,2],7:[5,4],6:[5,6],5:[5,8],
    4:[6,1],3:[6,3],2:[6,5],1:[6,7]
}

def marker_local_corners(col,row):
    cx = col*SQUARE_SIZE + SQUARE_SIZE/2
    cy = row*SQUARE_SIZE + SQUARE_SIZE/2
    h = MARKER_SIZE/2
    return np.array([
        [cx-h, cy-h],
        [cx+h, cy-h],
        [cx+h, cy+h],
        [cx-h, cy+h]
    ],dtype=np.float32)

def rigid_transform(A,B):
    ca=A.mean(0); cb=B.mean(0)
    AA=A-ca; BB=B-cb
    H=AA.T@BB
    U,S,Vt=np.linalg.svd(H)
    R=Vt.T@U.T
    if np.linalg.det(R)<0:
        Vt[-1]*=-1
        R=Vt.T@U.T
    t=cb-R@ca
    T=np.eye(4)
    T[:3,:3]=R
    T[:3,3]=t
    return T

pipe=rs.pipeline()
cfg=rs.config()
cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
align=rs.align(rs.stream.color)
pipe.start(cfg)
print("Waiting 5 seconds for camera stabilization...")
time.sleep(5)

aruco=cv2.aruco
det=aruco.ArucoDetector(
    aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
    aruco.DetectorParameters()
)

cam_pts=[]
rob_pts=[]

try:
    NUM_FRAMES = 40

    for frame_no in range(NUM_FRAMES):

        frames = align.process(pipe.wait_for_frames())

        color = frames.get_color_frame()
        depth = frames.get_depth_frame()

        if not color or not depth:
            continue

        img = np.asanyarray(color.get_data())

        intr = depth.profile.as_video_stream_profile().intrinsics

        corners, ids, _ = det.detectMarkers(img)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            cv2.imwrite("detected_markers.png", img)

        if ids is None:
            continue

        ids = ids.flatten()

        print(f"Frame {frame_no+1}/{NUM_FRAMES}: {len(ids)} markers")

        for mc,mid in zip(corners,ids):
            if mid not in MARKER_MAP:
                continue
            col,row=MARKER_MAP[mid]
            local=marker_local_corners(col,row)

            for pix,lp in zip(mc[0],local):
                u,v=pix
                depth_2=depth.get_distance(int(u),int(v))
                if depth_2<=0:
                    continue

                cp=np.array(
                    rs.rs2_deproject_pixel_to_point(
                        intr,[float(u),float(v)],depth_2
                    ),dtype=float)

                rp2=cv2.perspectiveTransform(
                    np.array([[[lp[0],lp[1]]]],dtype=np.float32),
                    T_LOCAL_TO_ROBOT
                )[0,0]

                rp=np.array([rp2[0],rp2[1],0.0],dtype=float)

                cam_pts.append(cp)
                rob_pts.append(rp)

finally:
    pipe.stop()
    print(f"\nCollected {len(cam_pts)} correspondences.")

cam_pts=np.asarray(cam_pts)
rob_pts=np.asarray(rob_pts)

print(f"Using {len(cam_pts)} correspondences")

if len(cam_pts)<12:
    raise RuntimeError("Not enough correspondences.")

T=rigid_transform(cam_pts,rob_pts)

pred=(T[:3,:3]@cam_pts.T).T+T[:3,3]
err=np.linalg.norm(pred-rob_pts,axis=1)

print("\nRotation:")
print(T[:3,:3])
print("\nTranslation:")
print(T[:3,3])

print("\n========== 4x4 CAMERA -> ROBOT ==========")
print(T)

print("\nRMS Error (m):",np.sqrt(np.mean(err**2)))
print("Max Error (m):",err.max())

# np.save("camera_to_robot.npy",T)
# print("Saved camera_to_robot.npy")
