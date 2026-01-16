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

        self.aligned_publisher = self.create_publisher(Bool,
                                                       '/ldlidar_node/aligned',
                                                       10)

        self.publish_aligned_timer = self.create_timer(1, self.publish_aligned_callback)


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
        command = MovementEnum.STOP

        (corners, marker_id) = read_marker(frame)

        if marker_id is not None and corners.any():

            self.current_marker = MarkerModel(corners, marker_id)

            if self.last_marker is None:
                # First Marker
                self.last_marker = self.current_marker


        match self.aligned_to_marker:
            case False:
                command = self.align_routine(frame)
            case True:
                command = self.auto_routine(frame)

        self.publish_data(command, frame)

        if self.auto_mode == False:
            self.aligned_to_marker = False
            self.movement_routine.current_sub_step = 0

    
    def align_routine(self, frame):
        command = MovementEnum.STOP

        self.publish_data(command, None)

        if self.current_marker is None or self.last_marker is None:
            return command

        if self.current_marker.marker_id != self.last_marker.marker_id:
            self.movement_routine.current_sub_step = 0

        command, self.aligned_to_marker = align_to_marker(frame, self.current_marker)

        #collision_detected = self.get_collision_detected()
        collision_detected = self.collision_in_direction(command)

        if collision_detected:
            command = MovementEnum.STOP


        return command

    def auto_routine(self, frame):
        command = MovementEnum.STOP

        if self.current_marker is None or self.last_marker is None:
            return command

        command = self.movement_routine.movement_routine(self.current_marker.marker_id)

        if self.movement_routine.current_sub_step == self.movement_routine.SUB_ROUTINE_DONE:
            # Routine done: reset
            print("Routine Done")
            self.aligned_to_marker = False

            if self.last_marker.marker_id != self.current_marker.marker_id:
                self.movement_routine.current_sub_step = 0

        self.last_marker = self.current_marker

        return command
    

    def get_collision_detected(self):

        if self.front_collision_detected or self.back_collision_detected or self.left_collision_detected or self.right_collision_detected:
            return True
        else:
            return False

    
    # Publish command and processed frame
    def publish_data(self, command, frame):

           if command is not None:
               msg_command = String()
               msg_command.data = str(command.value)

               self.command_publisher.publish(msg_command)

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
        if msg.data == "1":
            self.auto_mode = True
        else:
            self.auto_mode = False

    def publish_aligned_callback(self):
        msg = Bool()
        msg.data = self.aligned_to_marker
        self.aligned_publisher.publish(msg)


