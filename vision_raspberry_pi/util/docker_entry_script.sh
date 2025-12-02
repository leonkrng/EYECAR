#!/bin/bash
#set -e

source /opt/ros/$ROS_DISTRO/setup.bash

cd ~/ros2_ws/

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

source ~/ros2_ws/install/local_setup.bash

echo "Starting front-node."
rm ~/ros2_ws/src/ldrobot-lidar-ros2/ldlidar_node/params/ldlidar.yaml
cp /app/release/ldlidar_front.yaml ~/ros2_ws/src/ldrobot-lidar-ros2/ldlidar_node/params/ldlidar.yaml
ros2 launch ldlidar_node ldlidar_bringup.launch.py &
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link ldlidar_front &

sleep 5

ros2 lifecycle set /ldlidar_node configure
ros2 lifecycle set /ldlidar_node activate

#sleep 5 

#echo "Starting rear-node."
#rm ~/ros2_ws/src/ldrobot-lidar-ros2/ldlidar_node/params/ldlidar.yaml
#cp /app/release/ldlidar_rear.yaml ~/ros2_ws/src/ldrobot-lidar-ros2/ldlidar_node/params/ldlidar.yaml
#ros2 launch ldlidar_node ldlidar_bringup.launch.py node_name:=ldlidar_rear params:=/app/release/ldlidar_front.yaml &
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link ldlidar_rear &

#sleep 5

#ros2 lifecycle set /ldlidar_rear configure
#ros2 lifecycle set /ldlidar_rear activate

ROS2_PID=$!

if [ "$1" == "bash" ]; then
	echo "Start interactive shell..."
  cd /app/
	exec bash
else 
	echo "Start Python-Application..."
	python3 /app/release/main.py
fi

wait $ROS2_PID
