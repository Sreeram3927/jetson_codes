class VisionConfig:
    
    # Camera & AI Configuration
    CAMERA_FPS = 30
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    RTSP_URL = "rtsp://localhost:8554/realsense"
    
    # Model Configuration
    YOLO_ENGINE_PATH = "ai_models/boxes_ai/v5s/boxes_ai.engine"
    # YOLO_ENGINE_PATH = "ai_models/ee_block/best.engine"
    CONFIDENCE_THRESHOLD = 0.5