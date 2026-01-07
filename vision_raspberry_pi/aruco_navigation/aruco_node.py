import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from std_msgs.msg import Bool 
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from aruco_navigation.read_marker import read_marker
from aruco_navigation.movement_enum import MovementEnum

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node', allow_undeclared_parameters=True)

        self.frame_raw_subscriber = self.create_subscription(
            Image,
            '/camera/frame_raw',
            self.image_raw_callback,
            10)

        self.collision_subscriber = self.create_subscription(
                Bool,
                '/ldlidar_node/collision',
                self.collision_callback,
                10)

        self.command_publisher = self.create_publisher(String,
                                                      '/serial/write',
                                                      10)

        self.frame_processed_publisher = self.create_publisher(Image,
                                                              '/camera/frame_processed',
                                                               10)

        self.collision_detected = False
        self.bridge = CvBridge()

        
    def image_raw_callback(self, msg:Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        (corners, id) = self.read_marker(frame)
        (center_x, center_y, diag_TL_BR, diag_TR_BL) = self.calc_marker_pos(corners, id)



        ## old code 
        
        command, processed_frame = read_marker(frame)

        msg = String()

        if self.collision_detected:
            msg.data = str(MovementEnum.STOP.value)
        else:
            msg.data = str(str(command.value))
            
        self.command_publisher.publish(msg)

        msg_frame = self.bridge.cv2_to_imgmsg(processed_frame, encoding="bgr8")
        self.frame_processed_publisher.publish(msg_frame)


    def collision_callback(self, msg:Bool):

        self.collision_detected = msg.data


    # Reads the marker on the frame and returns the corners and the id.
    def read_marker(self, frame_to_read):

        grayscale_frame = cv2.cvtColor(frame_to_read, cv2.COLOR_BGR2GRAY)

        key = getattr(aruco, f"DICT_5X5_50")

        aruco_dict = aruco.getPredefinedDictionary(key)
        aruco_param = aruco.DetectorParameters()
        aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_param)

        (corners, ids, rejected) = aruco_detector.detectMarkers(grayscale_frame)

        return (corners[0], ids[0])

    # Calculates the centerpoint and the diagonales of the marker.
    def calc_marker_pos(self, corners, id):

        (top_left, top_right, bottom_right, bottom_left) = corners

        # Convert coordiantes to integer
        top_right = (int(top_right[0]), int(top_right[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
        bottom_righ = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

        # ToDo publish coordinates and id to the overlay-node an draw on the frame

        # Calculate centerpoint
        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        center_y = int((top_left[1] + bottom_right[1]) / 2.0)

        # Calculate diagonals
        diag_TL_BR = pow(pow(bottom_right[0] - top_left[0], 2) + pow(bottom_right[1] - top_left[1], 2), 0.5)
        diag_TR_BL = pow(pow(top_right[0] - bottom_left[0], 2) + pow(top_right[1] - bottom_left[1], 2), 0.5)

        return (center_x, center_y, diag_TL_BR, diag_TR_BL)



