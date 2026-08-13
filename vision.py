import pyrealsense2 as rs
import numpy as np
import cv2
import subprocess
from ultralytics import YOLO
from config import Config
from transformations import FrameTransformation

class VisionSystem:
    def __init__(self, target_detected_callback=None, detections_callback=None):
        self.target_detected_callback = target_detected_callback
        self.detections_callback = detections_callback
        self.transformer = FrameTransformation()
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.gst_process = None
        self.running = False

    def setup(self):
        # 1. HEAVY LIFTING FIRST: Load TensorRT engine (Blocks for ~8 seconds)
        print("Loading TensorRT Engine...")
        self.model = YOLO(Config.YOLO_ENGINE_PATH, task='detect')

        # WARMUP: Force the GPU to load the engine by running a blank frame
        print("Warming up YOLO to force GPU memory allocation (This will take ~8-10 seconds)...")
        dummy_frame = np.zeros((Config.CAMERA_HEIGHT, Config.CAMERA_WIDTH, 3), dtype=np.uint8)
        self.model.predict(source=dummy_frame, verbose=False)
        print("Warmup complete. GPU is ready.")

        # 2. START HARDWARE: Boot up RealSense
        print("Starting RealSense camera...")
        config = rs.config()
        config.enable_stream(rs.stream.depth, Config.CAMERA_WIDTH, Config.CAMERA_HEIGHT, rs.format.z16, Config.CAMERA_FPS)
        config.enable_stream(rs.stream.color, Config.CAMERA_WIDTH, Config.CAMERA_HEIGHT, rs.format.bgr8, Config.CAMERA_FPS)
        self.pipeline.start(config)

        # 3. START STREAMING LAST: Launch GStreamer right before the loop starts
        print("Initializing Hardware GStreamer Subprocess...")
        gst_cmd = [
            'gst-launch-1.0', '-e',
            'fdsrc', 'fd=0', '!',
            'rawvideoparse', 'use-sink-caps=false', 
            f'format=bgr', f'width={Config.CAMERA_WIDTH}', f'height={Config.CAMERA_HEIGHT}', f'framerate={Config.CAMERA_FPS}/1', '!',
            'videoconvert', '!', 'video/x-raw,format=BGRx', '!',
            'nvvidconv', '!', 'video/x-raw(memory:NVMM),format=NV12', '!',
            'nvv4l2h264enc', 'maxperf-enable=1', 'insert-sps-pps=true', f'idrinterval={Config.CAMERA_FPS}', 'bitrate=2000000', '!',
            'h264parse', '!',
            'rtspclientsink', f'location={Config.RTSP_URL}', 'protocols=tcp'
        ]
        
        try:
            self.gst_process = subprocess.Popen(gst_cmd, stdin=subprocess.PIPE)
        except FileNotFoundError:
            raise Exception("GStreamer not installed. Run apt-get install -y gstreamer1.0-tools inside the container.")

    def run(self):
        self.running = True
        print(f"Hardware streaming live to {Config.RTSP_URL}")
        
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

                results = self.model.predict(source=color_image, conf=Config.CONFIDENCE_THRESHOLD, verbose=False)

                # 2. Initialize a list to hold all unique targets in this frame
                current_frame_targets = []

                for result in results:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf)
                        
                        # Raw pixels
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        z_dist = depth_frame.get_distance(cx, cy)

                        if 0.01 < z_dist < 3.0:
                            # 3. Transform pixels to manipulator coordinates immediately
                            camera_point = rs.rs2_deproject_pixel_to_point(
                                depth_intrin,
                                [cx, cy],
                                z_dist,
                            )
                            x_man, y_man, z_man = self.transformer.getTransformedCoordinates(camera_point)
                            
                            # Add the TRANSFORMED coordinates to our frame list
                            current_frame_targets.append({
                                "x": round(x_man, 2), 
                                "y": round(y_man, 2), 
                                "z": round(z_man, 2), 
                                "conf": round(conf, 2)
                            })

                            # Send the physical manipulator coordinates to the ESP32!
                            # if self.target_detected_callback:
                                # self.target_detected_callback(x_man, y_man, z_man)

                            # Draw visuals (OpenCV still needs raw pixels to draw on the image)
                            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.circle(color_image, (cx, cy), 4, (0, 0, 255), -1)
                            
                            # Update label to show physical coordinates instead of pixels
                            # label = f"Conf {conf:.2f} | U:{cx:.1f} V:{cy:.1f}"
                            label = f"Target #{len(current_frame_targets)}"
                            cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 4. Trigger the callback to send the full transformed array to the frontend
                if self.detections_callback and current_frame_targets:
                    self.detections_callback(current_frame_targets)

                # Write directly to the GStreamer command line process
                # if self.gst_process and self.gst_process.stdin:
                #     self.gst_process.stdin.write(color_image.tobytes())
                # Check if the subprocess is still running (poll() returns None if alive)
                if self.gst_process and self.gst_process.poll() is None:
                    try:
                        self.gst_process.stdin.write(color_image.tobytes())
                        self.gst_process.stdin.flush() # Force the buffer through immediately
                    except BrokenPipeError:
                        print("WARNING: GStreamer pipe broke during write. Stream lost.")
                        # Clean up the dead process
                        self.gst_process.stdin.close()
                        self.gst_process.wait()
                        self.gst_process = None 
                        # Optional: Add logic here to restart the subprocess if desired
                else:
                    # Failsafe: if the process died silently, clean it up
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
        if self.gst_process:
            self.gst_process.stdin.close()
            self.gst_process.wait()
        print("Camera and hardware stream closed cleanly.")