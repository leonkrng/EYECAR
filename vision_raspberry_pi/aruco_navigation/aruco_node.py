import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from std_msgs.msg import Bool 
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from aruco_navigation.movement_enum import MovementEnum
from aruco_navigation.marker_model import MarkerModel
from overlay.draw_marker_border import draw_marker_border

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
        self.current_marker = None

        
    def image_raw_callback(self, msg:Image):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        (corners, marker_id) = self.read_marker(frame)

        command = MovementEnum.STOP

        if marker_id is not None and corners.any():
            draw_marker_border(frame, corners, marker_id)
        
            self.current_marker = MarkerModel(corners, marker_id)

            command, aligned = self.align_to_marker(frame)

            if aligned:
                pass
                # EYECAR is algined to marker
                # TODO: Add main-movement calculation here


        if self.collision_detected or command is None:
            command = MovementEnum.STOP
            

        # Publishing command and processed frame
        msg = String()
        msg.data = str(command.value)

        self.command_publisher.publish(msg)

        msg_frame = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
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

        if ids is None or len(corners) == 0:
            return None, None

        return corners[0][0], ids[0]


    # Aligns the EYECAR to the marker. 
    def align_to_marker(self, frame):

        aligned = False

        height, width = frame.shape[:2]

        # The span between 40% and 60% off the frame is considered to be the middle
        border_left = int(width * 0.4)
        border_right = int(width * 0.6)

        # A <10% difference between the lengt of the sides is considered straight
        side_diff = self.current_marker.side_TL_BL / self.current_marker.side_TR_BR

        # If the marker the diagonales are 40% of the widht the marker is considered close enough
        max_size = 0.4 * width 

        if self.current_marker.center_x < border_left:
            # Marker too far to the left
            return MovementEnum.LEFT, aligned

        if self.current_marker.center_x > border_right:
            # Marker too far to the right
            return MovementEnum.RIGHT, aligned

        if side_diff > 1.1 :
            # Marker is seen from the left side
            return MovementEnum.TURN_LEFT, aligned

        if side_diff < 0.9:
            # Marker is seen from the right side
            return MovementEnum.TURN_RIGHT, aligned

        if self.current_marker.center_x > border_left and self.current_marker.center_x < border_right:
            # Marker is in the middle but too far away
            return MovementEnum.FORWARD, aligned 

        if self.current_marker.diag_TL_BR > max_size and self.current_marker.diag_TR_BL > max_size:
            # Marker is close enough and aligend
            aligned = True
            return MovementEnum.STOP, aligned
        
        # Fallback 
        return MovementEnum.STOP, aligned

