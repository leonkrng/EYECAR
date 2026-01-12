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

        self.last_frame = None
        cv2.namedWindow("EYECAR", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("EYECAR", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self.frame_timer = self.create_timer(0.03, self.show_window_callback)

    def processed_frame_callback(self, msg:Image):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_frame = draw_overlay(frame)

    def show_window_callback(self):
        if self.last_frame is not None:
                frame = draw_overlay(self.last_frame)
                cv2.imshow("EYECAR", frame)
                cv2.waitKey(1)
