# ============================================================
# ROS2 container: rosbridge (websocket for the frontend), detection_relay
# (receives vision detections over TCP, republishes as a ROS2 topic),
# and (once ported) the ESP32/Arduino serial bridge nodes.
# Python 3.6 / rclpy — does NOT run ultralytics; see vision_container.
# ============================================================
FROM dustynv/ros:foxy-pytorch-l4t-r32.7.1

# Fix expired ROS GPG key + install system dependencies
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        avrdude \
        python3-pip \
        python3-colcon-common-extensions \
        git \
    && rm -rf /var/lib/apt/lists/*

# Build rosbridge_suite from source (Foxy-compatible tag)
RUN mkdir -p /ros2_lib/src && \
    cd /ros2_lib/src && \
    git clone -b 1.3.1 https://github.com/RobotWebTools/rosbridge_suite.git && \
    cd /ros2_lib && \
    . /opt/ros/foxy/install/setup.bash && \
    colcon build --symlink-install

# ROS2 environments
RUN echo "/opt/ros/foxy/install/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_lib/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws

#---------------------------------------------
# Include the below line, if you dont want to
# bind the volumes, sutiable for prod builds
#---------------------------------------------

# Copy and build your own ROS2 workspace
# COPY ros2_ws /ros2_ws

# RUN . /opt/ros/foxy/install/setup.bash && \
#     . /ros2_lib/install/setup.bash && \
#     cd /ros2_ws && \
#     colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# CMD ["ros2", "launch", "detection_relay", "bringup.launch.py"]