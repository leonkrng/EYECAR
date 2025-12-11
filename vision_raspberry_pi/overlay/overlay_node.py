import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from overlay.draw_overlay import draw_overlay

class OverlayNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.frame_processed_subscriber = self.create_subscription( Image,
                                                                   'camera/frame_processed',
                                                                   self.processed_frame_callback,
                                                                   10)
        self.bridge = CvBridge()

    def processed_frame_callback(self, msg:Image):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        draw_overlay(frame)
