FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install -y \
    ros-foxy-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    git \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

ENV QT_X11_NO_MITSHM=1
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SHELL ["/bin/bash","-lc"]
WORKDIR /root
