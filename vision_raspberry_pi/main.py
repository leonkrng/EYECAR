from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import cv2.aruco as aruco
import threading
from gpiozero import CPUTemperature
import serial
import numpy as np
import time
import cv2
import aruco_navigation
import communication
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from communication.serial_connection_node import SerialConnectionNode
from lidar_sensor.lidar_node import LidarNode
from camera.camera_node import CameraNode
import overlay
from std_msgs.msg import String
from main_node import MainNode
from aruco_navigation.aruco_node import ArucoNode
from camera.camera_node import CameraNode
from overlay.overlay_node import OverlayNode

#camera_resolution = [832, 600]  # Camera resolution [x, y]
#navigation_list = [1, 2, 3, 4]
#max_marker_size = 0.4  # maximum marker size in relation to camera x-resoulution (0-1)

# Camera stream pipeline
#gst_pipeline = ("udpsrc port=8000 ! jpegdec ! videoconvert ! appsink drop=true sync=false max-buffers=1")
#cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Initialize executor 
rclpy.init()
executor = MultiThreadedExecutor()

# Initialize camera-node
camera_node = CameraNode()
executor.add_node(camera_node)

# Initialize lidar-node
lidar_front_node = LidarNode()
executor.add_node(lidar_front_node)

# Initialize serial-connection-node
serial_node = SerialConnectionNode()
executor.add_node(serial_node)

# Initialize aruco-node
aruco_node = ArucoNode()
executor.add_node(aruco_node)

#Initialize overlay-node
overlay_node = OverlayNode() 
executor.add_node(overlay_node)

while 1:

    executor.spin_once(timeout_sec=0.001)

    # If the `q` key was pressed, break from the loop
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        cv2.destroyAllWindows()
        break

# Cleanup
executor.shutdown()
serial_node.destroy_node()
lidar_front_node.destroy_node()
camera_node.destroy_node()
aruco_node.destroy_node()
overlay_node.destroy_node()
rclpy.shutdown()
