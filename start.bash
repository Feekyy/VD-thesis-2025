#!/bin/bash

DOCKER_IMAGE="foxy-docker:latest"
CONTAINER_NAME="foxy-docker"

echo "-----------------------------------"
echo " Tello Drone Launcher Script"
echo "-----------------------------------"
echo "Commands:"
echo "  build     - Start building the docker"
echo "  start     - Foxy Docker starter"
echo "  start2    - Secondary Docker starter"
echo "  gazebo    - Gazebo simulation starter"
echo "  camera1   - Open the camera feedback in rqt"
echo "  camera2   - Opening the visualation for the detector node"
echo "  drone     - Spawning the drone in the simulation"
echo "  detector  - Cirdle detector node starter"
echo "  hand1     - Hand movement starter in the simulation"
echo "  hand2     - Hand mvement starter"
echo "  automatic - Automatic search starter"
echo "  exit      - Escape the script"
echo "-----------------------------------"

read -p "Write here: " CMD

case "$CMD" in
	"build")
		clear
		echo "Building the docker!"
		cd Docker
		sudo docker build -t $CONTAINER_NAME .
	;;

	"start")
		clear
    	echo "Starting docker..."
    	docker stop $CONTAINER_NAME 2>/dev/null
    	docker rm $CONTAINER_NAME 2>/dev/null
    	xhost +local:docker
		sudo docker run -it --net=host --gpus all --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$HOME/DockerFiles:/root/DockerFiles" --name $CONTAINER_NAME $DOCKER_IMAGE
	;;

	"start2")
		clear
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
        ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'tello_drone'}" 2>/dev/null
		ros2 run gazebo_ros spawn_entity.py -file ~/DockerFiles/drone_ws/src/tello_ros/tello_description/urdf/tello.urdf -entity tello_drone
	;;

	"camera1")
		clear
		echo "Opening camera feed"
		ros2 run rqt_image_view rqt_image_view /topic_ns/image_raw
	;;

	"camera2")
		clear
		echo "Stating the circle detector node with visualation"
		source drone_ws/install/setup.bash
		ros2 run tello_camera tello_camera_contour --ros-args -p image_topic:=/topic_ns/image_raw -p visualize:=true -p debug:=true
	;;

	"detector")
		clear
		echo "Stating the circle detector node"
		source drone_ws/install/setup.bash
		ros2 run tello_camera tello_camera --ros-args -p image_topic:=/topic_ns/image_raw

	;;

	"hand1")
		clear
		echo "Starting hand movement for the simulation"
		source drone_ws/install/setup.bash
		ros2 run tello_hand_move tello_hand_move --ros-args -p model_name:=tello_drone
	;;

	"hand2")
		clear
		echo "Starting hand movement for the drone"
		source drone_ws/install/setup.bash
		ros2 run tello_hand_move tello_hand_move --ros-args -p simulation:=false -p model_name:=tello_drone -p cmd_vel_topic:=/cmd_vel
	;;

	"automatic")
		clear
		echo "Starting hand movemnt in the simulation"
		source drone_ws/install/setup.bash
		ros2 run tello_mission tello_mission
	;;

	"exit")
    	exit 0
    ;;

    *)
		clear
    	echo "Wrong command! Try again!"
		bash start.bash
    ;;
esac