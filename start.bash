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
    	read -p "Give me the world name: " WORLD
    	WORLD="${WORLD}.world"
    	echo "Starting the Gazebo simulation with world: $WORLD"
    	sudo docker exec -it $CONTAINER_NAME bash -c "gazebo /root/tello_ros_ws/sim/worlds/${WORLD} --verbose"
	;;

	"exit")
    	exit 0
    ;;

    	*)
    	echo "Wrong command! Try again!"
		exit 0
    ;;
esac
