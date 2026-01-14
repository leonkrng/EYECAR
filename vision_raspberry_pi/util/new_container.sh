#!/bin/bash

docker stop eyecar
docker container rm eyecar

cd /home/pi/Desktop/vision_raspberry_pi
docker build . -t vision_raspberry_pi -f docker/Dockerfile

docker run -it --name eyecar --privileged --network host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw -v /run/udev:/run/udev vision_raspberry_pi bash
