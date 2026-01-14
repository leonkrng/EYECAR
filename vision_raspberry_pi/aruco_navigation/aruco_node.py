from warnings import warn
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from std_msgs.msg import Bool 
from std_msgs.msg import Float32 
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from aruco_navigation.movement_enum import MovementEnum
from aruco_navigation.marker_model import MarkerModel
from overlay.draw_marker_border import draw_marker_border
from aruco_navigation.read_marker import read_marker
from aruco_navigation.align_to_marker import align_to_marker 
from aruco_navigation.movement_base import MovementBase
from aruco_navigation.movement_pick_and_place import MovementPickAndPlace
import time

from vision_raspberry_pi.aruco_navigation import marker_model

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node', allow_undeclared_parameters=True)

        self.frame_raw_subscriber = self.create_subscription(
            Image,
            '/camera/frame_raw',
            self.image_raw_callback,
            10)

        self.front_collision_subscriber = self.create_subscription(
                Bool,
                '/ldlidar_node/collision_front',
                self.front_collision_callback,
                10)
        self.back_collision_subscriber = self.create_subscription(
                Bool,
                '/ldlidar_node/collision_back',
                self.back_collision_callback,
                10)
        self.right_collision_subscriber = self.create_subscription(
                Bool,
                '/ldlidar_node/collision_right',
                self.right_collision_callback,
                10)
        self.left_collision_subscriber = self.create_subscription(
                Bool,
                '/ldlidar_node/collision_left',
                self.left_collision_callback,
                10)

        self.workstation_distance_subscriber = self.create_subscription(
                Float32,
                '/ldlidar_node/workstation_distance',
                self.workstation_distance_callback,
                10)

        self.serial_read_subscriber = self.create_subscription(
                String,
                '/serial/read',
                self.serial_read_callback,
                10)

        self.command_publisher = self.create_publisher(String,
                                                      '/serial/write',
                                                      10)

        self.frame_processed_publisher = self.create_publisher(Image,
                                                              '/camera/frame_processed',
                                                               10)

        self.front_collision_detected = False
        self.back_collision_detected = False
        self.right_collision_detected = False
        self.left_collision_detected = False

        self.bridge = CvBridge()
        self.current_marker = None
        self.last_marker = None 
        self.aligned_to_marker = False
        self.movement_routine = MovementPickAndPlace()

        self.auto_mode = False
        self.command = None


        
    def image_raw_callback(self, msg:Image):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        (corners, marker_id) = read_marker(frame)


        if marker_id is not None and corners.any():
            # Marker detected
            draw_marker_border(frame, corners, marker_id)
        
            self.current_marker = MarkerModel(corners, marker_id)

            if self.last_marker is None:
                # First marker
                self.last_marker = self.current_marker


            if (self.current_marker.marker_id != self.last_marker.marker_id) and (self.movement_routine.current_sub_step == self.movement_routine.SUB_ROUTINE_DONE) or (self.auto_mode == False):
                # New Marker: reset step and alignment
                self.movement_routine.current_sub_step = 0
                self.aligned_to_marker = False


            if self.aligned_to_marker:
                pass
                # Alignment is completed and the workstation-routine should run
                self.command = self.movement_routine.movement_routine(self.current_marker.marker_id)

            else:
                pass
                # Alignment is not completed and the alignment-routine should run
                self.command, self.aligned_to_marker = align_to_marker(frame, self.current_marker)

                collision_detected = self.collision_in_direction(self.command)

                if collision_detected:
                    self.command = MovementEnum.STOP

            # Set current marker ad last marker for the next cycle
            self.last_marker = self.current_marker
        else:
            if self.aligned_to_marker == False:
                # Marker was lost during the alginment
                self.command = MovementEnum.STOP

        self. publish_data(self.command, frame)


    # Publish command and processed frame
    def publish_data(self, command, frame):

        if command is not None:
            msg_command = String()
            msg_command.data = str(command.value)

            self.command_publisher.publish(msg)

        if frame is not None:
            msg_frame = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.frame_processed_publisher.publish(msg_frame)




    def front_collision_callback(self, msg:Bool):

        self.front_collision_detected = msg.data

    def back_collision_callback(self, msg:Bool):

        self.back_collision_detected = msg.data

    def right_collision_callback(self, msg:Bool):

        self.back_collision_detected = msg.data

    def left_collision_callback(self, msg:Bool):

        self.left_collision_detected = msg.data

    def workstation_distance_callback(self, msg:Float32):
        self.movement_routine.workstation_distance = msg.data

    # Checks if there is a collision in the movement direction.
    def collision_in_direction(self, command: "MovementEnum") -> bool:
        block_left  = self.left_collision_detected or self.back_collision_detected
        block_right = self.right_collision_detected or self.back_collision_detected
        block_front = (
            self.front_collision_detected
            or self.left_collision_detected
            or self.right_collision_detected
        )
        any_collision = (
            self.front_collision_detected
            or self.back_collision_detected
            or self.right_collision_detected
            or self.left_collision_detected
        )

        conditions = {
            MovementEnum.FORWARD: block_front,

            MovementEnum.LEFT: block_left,
            MovementEnum.RIGHT: block_right,

            MovementEnum.FORWARD_LEFT: (block_front or block_left),
            MovementEnum.FORWARD_RIGHT: (block_front or block_right),

            MovementEnum.BACKWARD: any_collision,
            MovementEnum.BACKWARD_LEFT: any_collision,
            MovementEnum.BACKWARD_RIGHT: any_collision,
            MovementEnum.TURN_LEFT: any_collision,
            MovementEnum.TURN_RIGHT: any_collision,
            MovementEnum.FORWARD_TURN_LEFT: any_collision,
            MovementEnum.FORWARD_TURN_RIGHT: any_collision,
            MovementEnum.BACKWARD_TURN_LEFT: any_collision,
            MovementEnum.BACKWARD_TURN_RIGHT: any_collision,
        }

        return conditions.get(command, False)

    def serial_read_callback(self, msg:String):
        if msg.data == 1:
            self.auto_mode = True
        else:
            self.auto_mode = False


