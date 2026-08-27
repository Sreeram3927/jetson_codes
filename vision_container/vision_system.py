import pyrealsense2 as rs
import numpy as np
import cv2
import subprocess
from ultralytics import YOLO
from vision_config import VisionConfig
from detection_client import DetectionSender


class VisionSystem:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.gst_process = None
        self.running = False
        # Connects to the detection_relay ROS2 node's socket server.
        # Host/port default to localhost:9999, overridable via
        # RELAY_HOST / RELAY_PORT env vars (see docker-compose.yml).
        self.sender = DetectionSender()

    def setup(self):
        # 1. HEAVY LIFTING FIRST: Load TensorRT engine (Blocks for ~8 seconds)
        print("Loading TensorRT Engine...")
        self.model = YOLO(VisionConfig.YOLO_ENGINE_PATH, task='detect')

        # WARMUP: Force the GPU to load the engine by running a blank frame
        print("Warming up YOLO to force GPU memory allocation (This will take ~8-10 seconds)...")
        dummy_frame = np.zeros((VisionConfig.CAMERA_HEIGHT, VisionConfig.CAMERA_WIDTH, 3), dtype=np.uint8)
        self.model.predict(source=dummy_frame, verbose=False)
        print("Warmup complete. GPU is ready.")

        # 2. START HARDWARE: Boot up RealSense
        print("Starting RealSense camera...")
        config = rs.config()
        config.enable_stream(rs.stream.depth, VisionConfig.CAMERA_WIDTH, VisionConfig.CAMERA_HEIGHT, rs.format.z16, VisionConfig.CAMERA_FPS)
        config.enable_stream(rs.stream.color, VisionConfig.CAMERA_WIDTH, VisionConfig.CAMERA_HEIGHT, rs.format.bgr8, VisionConfig.CAMERA_FPS)
        self.pipeline.start(config)

        # 3. START STREAMING LAST: Launch GStreamer right before the loop starts
        print("Initializing Hardware GStreamer Subprocess...")
        gst_cmd = [
            'gst-launch-1.0', '-e',
            'fdsrc', 'fd=0', '!',
            'rawvideoparse', 'use-sink-caps=false',
            f'format=bgr', f'width={VisionConfig.CAMERA_WIDTH}', f'height={VisionConfig.CAMERA_HEIGHT}', f'framerate={VisionConfig.CAMERA_FPS}/1', '!',
            'videoconvert', '!', 'video/x-raw,format=BGRx', '!',
            'nvvidconv', '!', 'video/x-raw(memory:NVMM),format=NV12', '!',
            'nvv4l2h264enc', 'maxperf-enable=1', 'insert-sps-pps=true', f'idrinterval={VisionConfig.CAMERA_FPS}', 'bitrate=2000000', '!',
            'h264parse', '!',
            'rtspclientsink', f'location={VisionConfig.RTSP_URL}', 'protocols=tcp'
        ]

        try:
            self.gst_process = subprocess.Popen(gst_cmd, stdin=subprocess.PIPE)
        except FileNotFoundError:
            raise Exception("GStreamer not installed. Run apt-get install -y gstreamer1.0-tools inside the container.")

    def run(self):
        self.running = True
        print(f"Hardware streaming live to {VisionConfig.RTSP_URL}")

        try:
            while self.running:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

                results = self.model.predict(source=color_image, conf=VisionConfig.CONFIDENCE_THRESHOLD, verbose=False)

                current_frame_targets = []

                for result in results:
                    names = result.names
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf)
                        class_id = int(box.cls[0])
                        class_name = names[class_id]

                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        z_dist = depth_frame.get_distance(cx, cy)

                        if 0.01 < z_dist < 3.0:
                            camera_point = rs.rs2_deproject_pixel_to_point(
                                depth_intrin,
                                [cx, cy],
                                z_dist,
                            )

                            # Raw camera-frame point
                            current_frame_targets.append({
                                "id": class_id,
                                "class": class_name,
                                "position" : {
                                    "x": round(camera_point[0], 4),
                                    "y": round(camera_point[1], 4),
                                    "z": round(camera_point[2], 4),
                                },
                                "confidence": round(conf, 2)
                            })

                            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.circle(color_image, (cx, cy), 4, (0, 0, 255), -1)

                            label = f"Target #{len(current_frame_targets)}"
                            cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Send this frame's targets to the ROS2 relay node instead
                # of an in-process callback. Non-blocking: drops the frame
                # silently if the relay isn't currently connected.
                if current_frame_targets:
                    self.sender.send(current_frame_targets)

                if self.gst_process and self.gst_process.poll() is None:
                    try:
                        self.gst_process.stdin.write(color_image.tobytes())
                        self.gst_process.stdin.flush()
                    except BrokenPipeError:
                        print("WARNING: GStreamer pipe broke during write. Stream lost.")
                        self.gst_process.stdin.close()
                        self.gst_process.wait()
                        self.gst_process = None
                else:
                    if self.gst_process:
                        self.gst_process = None
                        print("WARNING: GStreamer subprocess died. Video streaming disabled, but targeting continues.")

        except Exception as e:
            print(f"Vision loop error: {e}")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        self.pipeline.stop()
        self.sender.close()
        if self.gst_process:
            self.gst_process.stdin.close()
            self.gst_process.wait()
        print("Camera and hardware stream closed cleanly.")
