#! /bin/bash

xhost +local:docker >/dev/null

if sudo docker ps -a | grep -q TFG; then
  sudo docker exec -it TFG /bin/bash
else
  sudo docker run -it --rm \
    --name TFG \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="./devDir/:/ros2_ws/devDir" \
    $(for dev in /dev/video*; do echo "--device=$dev"; done) \
    --device=/dev/bus/usb/001/003 \
    --privileged \
    tfg-img
fi
