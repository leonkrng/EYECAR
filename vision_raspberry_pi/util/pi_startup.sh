#!/bin/bash

# Initialize camera to lower camera
pinctrl set 17 op dl
pinctrl set 4 op dl
i2cset -y 1 0x70 0x00 0x01

# Start Docker-Container
docker run -it --rm --privileged --network host --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw -v /run/udev:/run/udev vision_raspberry_pi

# Start UPD-Video-Stream
rpicam-vid -t 0 --codec mjpeg --inline --framerate 10 --width 832 --height 600 --listen -o udp://0.0.0.0:8000


