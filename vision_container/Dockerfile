# ============================================================
# Vision container: RealSense capture + YOLO/TensorRT inference + RTSP push.
# Talks to the ROS2 container only via a plain TCP socket (detection_client.py).
# ============================================================
FROM ultralytics/ultralytics@sha256:3217e85d3760be4962efaeaae73117d569aed33bfded306981319db24800ce71

RUN apt-get update && \
    apt-get install -y \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-rtsp libgstrtspserver-1.0-0 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

#---------------------------------------------
# Include the below line, if you dont want to
# bind the volumes, sutiable for prod builds
#---------------------------------------------

# COPY . /workspace

CMD ["python3", "main.py"]
# CMD ["sleep", "infinity"]
