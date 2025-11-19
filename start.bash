#!/bin/bash

DOCKER_IMAGE="foxy-docker:latest"
CONTAINER_NAME="foxy-docker"

echo "-----------------------------------"
echo " Tello Drone Launcher Script"
echo "-----------------------------------"
echo "Commands:"
echo "  build	- Start building the docker"
echo "  start   - Foxy Docker starter"
echo "  start2  - Secondary Docker starter"
echo "  gazebo  - Gazebo simulation starter"
echo "  camera1 - Open the camera feedback in rqt"
echo "  camera2 - Opening the visualation for the detector node"
echo "  drone   - Spawning the drone in the simulation"
echo "  detector- Cirdle detector node starter"
echo "  hand1   - Hand movement starter in the simulation"
echo "  save    - Save to the harddrive"
echo "  exit    - Escape the script"
echo "-----------------------------------"

read -p "Write here: " CMD

case "$CMD" in
	"build")
		echo "Building the docker!"
		cd Docker
		sudo docker build -t $CONTAINER_NAME .
	;;

	"start")
    	echo "Starting docker..."
    	docker stop $CONTAINER_NAME 2>/dev/null
    	docker rm $CONTAINER_NAME 2>/dev/null
    	xhost +local:docker
		sudo docker run -it --net=host --gpus all --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$HOME/DockerFiles:/root/DockerFiles" --name $CONTAINER_NAME $DOCKER_IMAGE
	;;

	"start2")
    	echo "Starting another docker shell..."
    	sudo docker exec -it $CONTAINER_NAME bash
	;;

	"gazebo")
		clear
    	echo "Starting the Gazebo with a preedited testing ground!"
		source /opt/ros/foxy/setup.bash
    	ros2 launch gazebo_ros gazebo.launch.py world:=~/DockerFiles/drone_ws/sim/worlds/donuttest.world
	;;

	"drone")
		clear
		echo "Spawning drone!"
		source /opt/ros/foxy/setup.bash
		ros2 run gazebo_ros spawn_entity.py -file ~/DockerFiles/drone_ws/src/tello_ros/tello_description/urdf/tello.urdf -entity tello_drone
	;;

	"camera1")
		clear
		echo "Opening camera feed"
		source /opt/ros/foxy/setup.bash
		rqt
	;;

	"camera2")
		clear
		echo "Opening visualation for the circle dtection"
		source /opt/ros/foxy/setup.bash
		cd ~/DockerFiles/drone_ws
		source install/setup.bash
		ros2 run tello_camera tello_camera --ros-args -p image_topic:=/topic_ns/image_raw -p visualize:=true -p debug:=true
	;;

	"detector")
		clear
		echo "Stating the circle detector node with less information output"
		source /opt/ros/foxy/setup.bash
		cd ~/DockerFiles/drone_ws
		source install/setup.bash
		ros2 run tello_camera tello_camera --ros-args -p image_topic:=/topic_ns/image_raw

	;;

	"hand1")
		clear
		source /opt/ros/foxy/setup.bash
		echo "Starting hand movemnt in the simulation"
		cd ~/DockerFiles/drone_ws
		source install/setup.bash
		ros2 run tello_hand_move tello_hand_move --ros-args -p model_name:=tello_drone
	;;

	"exit")
    	exit 0
    ;;

    *)
    	echo "Wrong command! Try again!"
    ;;
esac
