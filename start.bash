#!/bin/bash

DOCKER_IMAGE="foxy-docker:latest"
CONTAINER_NAME="foxy-docker"
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$ROOT_DIR/drone_ws"
WORLD="$WS_DIR/sim/worlds/donuttest.world"
DRONE_URDF="$WS_DIR/src/tello_ros/tello_description/urdf/tello.urdf"

#Commands that use ROS 2 only work inside the container
require_docker() {
	if [ ! -f /.dockerenv ]; then
		echo "This command must be run inside the container (use 'start' or 'start2' first)!"
		exit 1
	fi
	source /usr/share/gazebo/setup.sh
	source /opt/ros/foxy/setup.bash
}

#Sources the built workspace, stops if it has not been built yet
source_ws() {
	if [ ! -f "$WS_DIR/install/setup.bash" ]; then
		echo "The workspace is not built yet, run 'compile' first!"
		exit 1
	fi
	source "$WS_DIR/install/setup.bash"
}

#True when the Gazebo simulation (with its ROS services) is already running
gazebo_running() {
	ros2 service list 2>/dev/null | grep -q "/spawn_entity"
}

#Puts the drone (back) to its starting position: the old one is deleted, a new one is spawned
spawn_drone() {
	ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'tello_drone'}" > /dev/null 2>&1
	ros2 run gazebo_ros spawn_entity.py -file "$DRONE_URDF" -entity tello_drone
}

#Stops the background processes started by 'demo'
stop_demo() {
	[ -n "$DETECTOR_PID" ] && kill $DETECTOR_PID 2>/dev/null
	[ "$STARTED_GAZEBO" = "1" ] && kill $GAZEBO_PID 2>/dev/null
}

show_menu() {
	echo "-----------------------------------"
	echo " Tello Drone Launcher Script"
	echo "-----------------------------------"
	echo "On the host:"
	echo "  build     - Build the docker image"
	echo "  start     - Start the docker container"
	echo "  start2    - Open another shell in the running container"
	echo "  remove    - Delete the container and the docker image of the project"
	echo "Inside the container:"
	echo "  compile   - Build the ROS 2 workspace (drone_ws)"
	echo "  demo      - Everything at once: Gazebo, drone, detector and the automatic mission"
	echo "  gazebo    - Start the Gazebo simulation"
	echo "  drone     - Put the drone (back) to its starting position"
	echo "  camera1   - Open the camera feed in rqt"
	echo "  camera2   - Start the circle detector node with visualization"
	echo "  detector  - Start the circle detector node"
	echo "  hough     - Start the Hough circle detector node with visualization"
	echo "  hand1     - Keyboard control in the simulation"
	echo "  hand2     - Keyboard control of the real drone (tello_driver has to run)"
	echo "  automatic - Start the automatic mission in the simulation"
	echo "  exit      - Exit the script"
	echo "-----------------------------------"
}

#The command can also be given as an argument, e.g. "bash start.bash demo"
CMD="$1"
if [ -z "$CMD" ]; then
	show_menu
	read -p "Write here: " CMD
fi

case "$CMD" in
	"build")
		clear
		echo "Building the docker image!"
		docker build -t $DOCKER_IMAGE "$ROOT_DIR/Docker"
	;;

	"start")
		clear
		echo "Starting docker..."
		docker stop $CONTAINER_NAME 2>/dev/null
		docker rm $CONTAINER_NAME 2>/dev/null

		#Access to the screen: with the login cookie if there is one, otherwise every local container is allowed (xhost)
		XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
		if [ -f "$XAUTH_FILE" ]; then
			X_ACCESS=(--env="XAUTHORITY=/tmp/.xauth" --volume="$XAUTH_FILE:/tmp/.xauth:ro")
		else
			xhost +local:docker
			X_ACCESS=()
		fi

		#HOST_UID/HOST_GID: 'compile' gives the built files back to this user
		docker run -it --net=host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
			"${X_ACCESS[@]}" --env="HOST_UID=$(id -u)" --env="HOST_GID=$(id -g)" \
			--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$ROOT_DIR:/root/DockerFiles" \
			--name $CONTAINER_NAME $DOCKER_IMAGE
	;;

	"start2")
		clear
		echo "Starting another docker shell..."
		docker exec -it $CONTAINER_NAME bash
	;;

	"remove")
		clear
		echo "This deletes the '$CONTAINER_NAME' container and every '$DOCKER_IMAGE' image version."
		echo "Your files in $ROOT_DIR are not touched."
		read -p "Continue? (y/n): " ANSWER
		if [ "$ANSWER" != "y" ]; then
			echo "Cancelled."
			exit 0
		fi
		docker stop $CONTAINER_NAME 2>/dev/null
		docker rm $CONTAINER_NAME 2>/dev/null
		docker rmi $DOCKER_IMAGE 2>/dev/null
		#Older, already replaced versions of the image (left behind by rebuilds)
		docker image prune -f --filter "label=project=tello-thesis"
		echo "Docker image removed."
		echo ""
		echo "The build cache can also take several GB, but it is shared by every docker project on this machine."
		read -p "Clear the whole docker build cache too? (y/n): " ANSWER
		if [ "$ANSWER" = "y" ]; then
			docker builder prune -f
		fi
	;;

	"compile")
		clear
		require_docker
		echo "Building the ROS 2 workspace!"
		cd "$WS_DIR" && colcon build
		STATUS=$?

		#The container works as root, the built files are given back to the user of the host
		if [ -n "$HOST_UID" ]; then
			chown -R "$HOST_UID:$HOST_GID" "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"
		fi
		exit $STATUS
	;;

	"demo")
		clear
		require_docker
		source_ws
		echo "Starting the whole simulation: Gazebo, drone, detector and the automatic mission (stop: Ctrl+C)"

		#Everything started here is stopped when the demo ends, an already running Gazebo is left alone
		trap stop_demo EXIT
		trap "exit 130" INT TERM
		STARTED_GAZEBO=0
		if ! gazebo_running; then
			ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD" > /tmp/gazebo.log 2>&1 &
			GAZEBO_PID=$!
			STARTED_GAZEBO=1
			for i in $(seq 1 60); do
				gazebo_running && break
				sleep 2
			done
			if ! gazebo_running; then
				echo "Gazebo did not start, see /tmp/gazebo.log"
				exit 1
			fi
		fi

		spawn_drone
		ros2 run tello_camera tello_camera_contour --ros-args -p image_topic:=/topic_ns/image_raw -p visualize:=true > /tmp/detector.log 2>&1 &
		DETECTOR_PID=$!
		sleep 3
		ros2 run tello_mission tello_mission
	;;

	"gazebo")
		clear
		require_docker
		echo "Starting Gazebo with the test world (three donuts)!"
		ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD"
	;;

	"drone")
		clear
		require_docker
		echo "Spawning drone!"
		spawn_drone
	;;

	"camera1")
		clear
		require_docker
		echo "Opening camera feed"
		ros2 run rqt_image_view rqt_image_view /topic_ns/image_raw
	;;

	"camera2")
		clear
		require_docker
		source_ws
		echo "Starting the circle detector node with visualization"
		ros2 run tello_camera tello_camera_contour --ros-args -p image_topic:=/topic_ns/image_raw -p visualize:=true
	;;

	"detector")
		clear
		require_docker
		source_ws
		echo "Starting the circle detector node"
		ros2 run tello_camera tello_camera_contour --ros-args -p image_topic:=/topic_ns/image_raw -p visualize:=false
	;;

	"hough")
		clear
		require_docker
		source_ws
		echo "Starting the Hough circle detector node"
		ros2 run tello_camera tello_camera_hough --ros-args -p image_topic:=/topic_ns/image_raw -p visualize:=true
	;;

	"hand1")
		clear
		require_docker
		source_ws
		echo "Starting keyboard control for the simulation"
		ros2 run tello_hand_move tello_hand_move
	;;

	"hand2")
		clear
		require_docker
		source_ws
		echo "Starting keyboard control for the drone (tello_driver has to run and the drone has to be in the air)"
		ros2 run tello_hand_move tello_hand_move --ros-args -p simulation:=false
	;;

	"automatic")
		clear
		require_docker
		source_ws
		echo "Starting the automatic mission in the simulation"
		ros2 run tello_mission tello_mission
	;;

	"exit")
		exit 0
	;;

	*)
		clear
		echo "Wrong command: '$CMD'"
		#Typed into the menu: try again, given as an argument: stop
		if [ -n "$1" ]; then
			exit 1
		fi
		exec bash "$0"
	;;
esac
