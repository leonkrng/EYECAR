#!/bin/bash
#set -e

source /opt/ros/$ROS_DISTRO/setup.bash
#source /root/ros2_ws/install/local_setup.bash

ln -s /dev/ttyAMA0 /dev/ldlidar
ls -la /dev/ldlidar

#if [ -e /dev/ttyAMA9 ]; then
#    ln -sf /dev/ttyAMA0 /dev/ldlidar
#else
#    echo "Warnung: /dev/ttyAMA0 nicht vorhanden!"
#fi

cd ~/ros2_ws/

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

source ~/ros2_ws/install/local_setup.bash
exec ros2 launch ldlidar_node ldlidar_bringup.launch.py &

sleep 5

ros2 launch ldlidar_node ldlidar_bringup.launch.py &

ros2 lifecycle set /ldlidar_node configure
ros2 lifecycle set /ldlidar_node activate

ROS2_PID=$!

if [ "$1" == "bash" ]; then
	echo "Start interactive shell..."
	exec bash
else 
	echo "Start Python-Application..."
	python3 /app/release/main.py
fi

wait $ROS2_PID
