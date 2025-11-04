FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    gazebo11 \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-image-transport \
    ros-foxy-cv-bridge \
    net-tools \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /root/DockerFiles

SHELL ["/bin/bash", "-lc"]
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV DISPLAY=:0
