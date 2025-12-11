import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from std_msgs.msg import Bool 
import cv2
from aruco_navigation.read_marker import read_marker
from vision_raspberry_pi.aruco_navigation.movement_enum import MovementEnum

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.frame_raw_subscriber = self.create_subscription(
            Image,
            '/camera/frame_raw',
            self.image_raw_callback,
            10)

        self.collision_subscriber = self.create_subscription(
                String,
                '/ldlidar_node/collision',
                self.collision_callback,
                10)

        self.command_publisher = self.createpublisher(String,
                                                      '/serial/write',
                                                      10)

        self.frame_processed_publisher = self.create_publisher(Image,
                                                              '/camera/frame_processed',
                                                               10)

        self.collision_detected = False

        
    def image_raw_callback(self, msg:Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        command, processed_frame = read_marker(frame)

        if self.collision_detected:
            self.command_publisher.publish(MovementEnum.STOP)
        else:
            self.command_publisher.publish(command)
            
        self.frame_processed_publisher.publish(processed_frame)


    def collision_callback(self, msg:Bool):

        self.collision_detected = msg.Data
        



