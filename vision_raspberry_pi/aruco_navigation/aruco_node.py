import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from std_msgs.msg import Bool 
from cv_bridge import CvBridge
import cv2
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
        
        command, processed_frame = read_marker(frame)

        msg = String()

        if self.collision_detected:
            msg.data = str(MovementEnum.STOP)
        else:
            msg.data = str(str(command.value))
            
        self.command_publisher.publish(msg)

        msg_frame = self.bridge.cv2_to_imgmsg(processed_frame, encoding="bgr8")
        self.frame_processed_publisher.publish(msg_frame)


    def collision_callback(self, msg:Bool):

        self.collision_detected = msg.data
        



