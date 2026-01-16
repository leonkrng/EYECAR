#!/bin/bash
# This script gets activated by the pi_stream_startup.service which is configured in /etc/systemd/system/pi_stream_startup.service

# Initialize camera to lower camera
pinctrl set 17 op dl
pinctrl set 4 op dl
i2cset -y 1 0x70 0x00 0x01


# Start UPD-Video-Stream
rpicam-vid -t 0 --codec mjpeg --inline --framerate 10 --width 832 --height 600 --listen -o udp://0.0.0.0:8000 
