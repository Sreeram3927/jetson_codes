# ============================================================
# Vision container: RealSense capture + YOLO/TensorRT inference + RTSP push.
# Talks to the ROS2 container only via a plain TCP socket (detection_client.py).
# ============================================================
FROM ultralytics/ultralytics:latest-jetson-jetpack4

RUN apt-get update && \
    apt-get install -y \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-rtsp libgstrtspserver-1.0-0 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

COPY . /workspace

CMD ["python3", "main.py"]
