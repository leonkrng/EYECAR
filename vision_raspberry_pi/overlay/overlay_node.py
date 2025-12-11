import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool
import cv2
from vision_raspberry_pi.overlay.draw_overlay import draw_overlay

class OverlayNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.frame_processed_subscriber = self.create_subscription( Image,
                                                                   'camera/frame_processed',
                                                                   self.processed_frame_callback,
                                                                   10)

    def processed_frame_callback(self, msg:Image):
        draw_overlay(msg.data)
