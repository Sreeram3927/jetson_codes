# ============================================================
# Vision container: RealSense capture + YOLO/TensorRT inference + RTSP push.
# Talks to the ROS2 container only via a plain TCP socket (detection_client.py).
# ============================================================
FROM ultralytics/ultralytics@sha256:3217e85d3760be4962efaeaae73117d569aed33bfded306981319db24800ce71

# Create non-root user matching host (UID 1000)
ARG USERNAME=jetson
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        sudo \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-rtsp libgstrtspserver-1.0-0 \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

#---------------------------------------------
# Include the below line, if you dont want to
# bind the volumes, sutiable for prod builds
#---------------------------------------------

# COPY . /workspace

# Make sure the user owns the workspace
RUN chown -R $USERNAME:$USERNAME /workspace

USER $USERNAME

CMD ["python3", "main.py"]
# CMD ["sleep", "infinity"]
