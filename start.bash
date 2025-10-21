#!/bin/bash

DOCKER_IMAGE="ros-foxy-tello:latest"
CONTAINER_NAME="ros-foxy-tello"

echo "-----------------------------------"\n
echo " Tello Drone Launcher Script"
echo "-----------------------------------"
echo "Commands:"
echo "  start   - Foxy Docker starter"
echo "  start2  - Secondary Docker starter"
echo "  gazebo  - Gazebo simulation starter"
echo "  save    - Save to the harddrive"
echo "  exit    - Escape the script"
echo "-----------------------------------"

read -p "Write here: " CMD

case "$CMD" in

	"start")
    	echo "Starting docker..."
	docker stop $CONTAINER_NAME
	docker rm $CONTAINER_NAME
	xhost +local:root
	docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/tello_ros_ws:/root/tello_ros_ws -v /dev/dri:/dev/dri --device /dev/dri --name $CONTAINER_NAME $DOCKER_IMAGE bash
	;;

	"start2")
	echo "Starting another docker..."
	docker exec -it $CONTAINER_NAME $DOCKER_IMAGE bash
	;;

	"gazebo")
	read -p "Give me the world name: " CMD
	CMD += ".world"
	echo "Starting the gazeboo simulation..."
	gazebo /root/tello_ros_ws/sim/worlds/$CMD --verbose
	;;

	*)
    	echo "Wrong command! Exiting!"
    	exit 1
esac
