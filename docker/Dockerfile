FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------------------------
# System tools
# ------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git \
    nano \
    curl \
    wget \
    tree \
    build-essential \
    cmake \
    gdb \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# Python / ROS development tools
# ------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-empy \
    python3-flake8 \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------
# ROS packages
# ------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-sensor-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2controlcli \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-sdformat-urdf \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true

# Create developer user
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Automatically source ROS
RUN echo "source /opt/ros/jazzy/setup.bash" >>  /home/ubuntu/.bashrc

# Default user
USER ubuntu

#inside container, create a workspace for ROS2
WORKDIR /home/ubuntu/mobile_3d_printer
