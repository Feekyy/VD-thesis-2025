FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
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
    nano \
    tree \
    python3-pip \
    python3-opencv \
    libasio-dev && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    transforms3d \
    scikit-fuzzy \
    opencv-python \
    numpy

RUN rosdep init || true && rosdep update

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

WORKDIR /root/DockerFiles

SHELL ["/bin/bash", "-lc"]
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV DISPLAY=:0
