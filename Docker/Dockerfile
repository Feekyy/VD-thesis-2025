FROM osrf/ros:foxy-desktop

#Marks every image built from this file, so 'remove' in start.bash can find the old versions too
LABEL project=tello-thesis

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

#Pinned to the versions the project was developed and tested with (Python 3.8)
RUN pip3 install \
    transforms3d==0.4.2 \
    opencv-python==4.12.0.88

RUN rosdep init || true && rosdep update

#The workspace is mounted from the host at runtime (see start.bash), so it is built inside the running container
#Gazebo's own setup is needed for its shader library, without it the drone camera cannot be created
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && \
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "[ -f /root/DockerFiles/drone_ws/install/setup.bash ] && source /root/DockerFiles/drone_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root/DockerFiles

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#CPU (software) rendering, so the simulation runs on any hardware without a GPU
ENV LIBGL_ALWAYS_SOFTWARE=1
