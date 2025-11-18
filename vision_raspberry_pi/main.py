from __future__ import print_function
from threading import Thread
from imutils.video import WebcamVideoStream
from picamera2 import Picamera2
from imutils.video import FPS
import argparse
import imutils
import cv2.aruco as aruco
from gpiozero import CPUTemperature
import serial
import numpy as np
import time
import cv2

import aruco_navigation
import communication
import lidar_sensor
import overlay


camera_resolution = [832, 600]  # Camera resolution [x, y]
navigation_list = [1, 2, 3, 4]
max_marker_size = 0.4  # maximum marker size in relation to camera x-resoulution (0-1)
prev_aruco_navigation_active = 0  # for flank detection
actual_ID = 0
error_car = 0

print("[INFO] sampling THREADED frames from webcam...")

#video_stream = WebcamVideoStream(src=0).start()
video_stream = Picamera2()
video_stream.start()


fps = FPS().start()

serial_connection = communication.serial_connection.SerialConnection()
serial_connection.start()

while 1:
    
    #frame = video_stream.read()
    frame = video_stream.capture_array()

    if frame is None:
        print("[ERROR]: No frame found.")
        continue

    frame = cv2.resize(frame, (832, 600))

    # Read ArUco-marker
    command = aruco_navigation.read_marker.read_marker(frame,
                                           prev_aruco_navigation_active,
                                           navigation_list,
                                           max_marker_size,
                                           camera_resolution, 
                                           actual_ID)

    serial_connection.text_to_send = "1/7/" + str(command) + "/8\r\n"

    if serial_connection.telemetry_data[10] == "1" and prev_aruco_navigation_active == 0:
        actual_ID = 0
        print("Navigation wurde neu gestartet. Bitte beim ersten Marker anfangen")

    if serial_connection.telemetry_data[10] == "1":
        prev_aruco_navigation_active = 1
    else:
        prev_aruco_navigation_active = 0

    # Draw overlay
    overlay.draw_overlay.draw_overlay(frame, serial_connection.telemetry_data, navigation_list, actual_ID, prev_aruco_navigation_active)

    # If the `q` key was pressed, break from the loop
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):

        cv2.destroyAllWindows()
        break

# Cleanup
cv2.destroyAllWindows()
video_stream.stop()
serial_connection.stop()
