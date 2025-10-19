#!/bin/bash

rsync -av --progress ~/Projects_Python/EYECAR/Vision_RaspberryPi pi@eye-car-pi:~/Desktop/

ssh pi@eye-car-pi "
  pkill main.py
  python ~/Documents/Vision_RaspberryPi/main.py &
"
