#!/bin/bash

#sudo ip addr add 169.254.158.150/16 dev enp0s31f6 
rsync -av --progress ~/Projects_Python/EYECAR/Vision_RaspberryPi pi@169.254.158.153:~/Desktop/

ssh pi@169.254.158.15"
  pkill main.py
  python ~/Documents/Vision_RaspberryPi/main.py &
"
