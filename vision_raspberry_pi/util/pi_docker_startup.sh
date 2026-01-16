#!/bin/bash

# This script gets activated by the pi_docker_startup.service which is configured in /etc/systemd/system/pi_docker_startup.service

sleep 30
# Start Docker-Container
docker start -a eyecar

#docker run -it --rm --privileged --network host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw -v /run/udev:/run/udev vision_raspberry_pi
