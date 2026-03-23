#! /bin/bash

xhost +local:docker > /dev/null

if sudo docker ps -a | grep -q TFG_go2; then  
	sudo docker exec -it TFG_go2 /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/src/gazebo_simulation/install/setup.bash && source /ros2_ws/src/go2_ros2_sdk/install/setup.bash && /bin/bash"
else
	sudo docker run -it --rm --name TFG_go2 --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd):/ros2_ws" $(for dev in /dev/video*; do echo "--device=$dev"; done) --device=/dev/bus/usb/001/003 --privileged go2_pateo bash -c "source /ros2_ws/src/gazebo_simulation/install/setup.bash && source /ros2_ws/src/go2_ros2_sdk/install/setup.bash && bash"
fi
