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

#Fuzzy rule interpolation engine of the University of Miskolc (FRIBE) and the parser it needs (exprail)
#Both are pinned to the tested versions, they have no installer, so they are used through PYTHONPATH
RUN git clone -q https://github.com/piller-imre/exprail-python.git /opt/exprail-python && \
    git -C /opt/exprail-python checkout -q 1b73a0993d399ef44ada232647845a253b821ecc && \
    git clone -q https://github.com/piller-imre/fribe-python.git /opt/fribe-python && \
    git -C /opt/fribe-python checkout -q 4cd0095df0f44254fd24e6f5cbefad0e2f717bbd
ENV PYTHONPATH=/opt/exprail-python:/opt/fribe-python

#The workspace is mounted from the host at runtime (see start.bash), so it is built inside the running container
#Gazebo's own setup is needed for its shader library, without it the drone camera cannot be created
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && \
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "[ -f /root/DockerFiles/drone_ws/install/setup.bash ] && source /root/DockerFiles/drone_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root/DockerFiles

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#CPU (software) rendering, so the simulation runs on any hardware without a GPU
ENV LIBGL_ALWAYS_SOFTWARE=1
