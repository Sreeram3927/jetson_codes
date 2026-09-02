# ============================================================
# ROS2 container: rosbridge (websocket for the frontend), detection_relay
# (receives vision detections over TCP, republishes as a ROS2 topic),
# and (once ported) the ESP32/Arduino serial bridge nodes.
# Python 3.6 / rclpy — does NOT run ultralytics; see vision_container.
# ============================================================
FROM dustynv/ros:foxy-pytorch-l4t-r32.7.1

# Create a non-root user that matches host UID/GID (usually 1000 on Jetson)
ARG USERNAME=jetson
ARG USER_UID=1000
ARG USER_GID=1000

# Fix expired ROS GPG key + install system dependencies
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        sudo \
        avrdude \
        python3-pip \
        python3-colcon-common-extensions \
        git \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Build rosbridge_suite from source (Foxy-compatible tag)
RUN mkdir -p /ros2_lib/src && \
    cd /ros2_lib/src && \
    git clone -b 1.3.1 https://github.com/RobotWebTools/rosbridge_suite.git && \
    cd /ros2_lib && \
    . /opt/ros/foxy/install/setup.bash && \
    colcon build --symlink-install

# 1. For non-interactive shells (bridge exec, docker exec -c, scripts...)
RUN echo 'source /opt/ros/foxy/install/setup.bash' > /ros_env.sh && \
    echo 'source /ros2_lib/install/setup.bash' >> /ros_env.sh && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /ros_env.sh

ENV BASH_ENV=/ros_env.sh

# 2. For interactive shells (bridge shell, docker exec -it ...)
RUN echo 'source /opt/ros/foxy/install/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /ros2_lib/install/setup.bash' >> /etc/bash.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /etc/bash.bashrc

# Give the user ownership of the places we need
RUN chown -R $USERNAME:$USERNAME /ros2_lib /home/$USERNAME

WORKDIR /ros2_ws

COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

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

# Switch to non-root user
USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]
# CMD ["ros2", "launch", "detection_relay", "bringup.launch.py"]