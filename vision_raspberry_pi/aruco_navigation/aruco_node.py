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

        self.current_sub_step = 0
        self.SUB_ROUTINE_DONE = 999
        self.sub_step_start_time = None
        self.workstation_distance = 0

        # Constans for the duration that the movement needs
        self.LOWER_GRIPPER_DURATION = 20_000
        self.LIFT_GRIPPER_DURATION = 20_000
        self.CLOSE_GRIPPER_DURATION = 10_000
        self.OPEN_GRIPPER_DURATION = 10_000

        # Distance between the lidar and the workstation for gripping the object (in m)
        self.GRIP_DISTANCE = 0.1

        
    def image_raw_callback(self, msg:Image):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        (corners, marker_id) = read_marker(frame)

        command = MovementEnum.STOP

        if marker_id is not None and corners.any():
            draw_marker_border(frame, corners, marker_id)
        
            self.current_marker = MarkerModel(corners, marker_id)

            if self.last_marker is None:
                # First marker
                self.last_marker = self.current_marker

            if self.current_marker.marker_id != self.last_marker.marker_id and self.current_sub_step == self.SUB_ROUTINE_DONE:
                # New Marker
                self.current_sub_step = 0


            command, self.aligned_to_marker = align_to_marker(frame, self.current_marker)

            if self.aligned_to_marker and self.current_sub_step != self.SUB_ROUTINE_DONE:
                # Gets the command from the marker-routines. Ignores collisions.
                command = self.movement_routine(marker_id)


        collision_detected = self.collision_in_direction(command)

        if collision_detected or command is None:
            command = MovementEnum.STOP
            

        # Publishing command and processed frame
        msg = String()
        msg.data = str(command.value)

        self.command_publisher.publish(msg)

        msg_frame = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.frame_processed_publisher.publish(msg_frame)

        self.last_marker = self.current_marker


    def front_collision_callback(self, msg:Bool):

        self.front_collision_detected = msg.data

    def back_collision_callback(self, msg:Bool):

        self.back_collision_detected = msg.data

    def right_collision_callback(self, msg:Bool):

        self.back_collision_detected = msg.data

    def left_collision_callback(self, msg:Bool):

        self.left_collision_detected = msg.data

    def workstation_distance_callback(self, msg:Float32):
        self.workstation_distance = msg.data

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

    def movement_routine(self, marker_id):

        match marker_id:
            case 1:
                return self.marker_1_movement()
            case 2:
                return self.marker_2_movement()
            case 3:
                return self.marker_3_movement()
            case 4:
                return self.marker_4_movement()

    def marker_1_movement(self):
        
        match self.current_sub_step:
            case 0:
                # Gripper down
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if (time.monotonic_ns() - self.sub_step_start_time) > self.LOWER_GRIPPER_DURATION:
                    self.current_sub_step = 1
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_DOWN

            case 1:
                # Gripper open
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.OPEN_GRIPPER_DURATION:
                    self.current_sub_step = 2
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_OPEN
        
            case 2: 
                # Get close to the workstation 
                if self.workstation_distance <= self.GRIP_DISTANCE:
                    self.current_sub_step = 3
                    return MovementEnum.STOP
                else:
                    return MovementEnum.FORWARD
            case 3:
                # Close Gripper 
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.CLOSE_GRIPPER_DURATION:
                    self.current_sub_step = 4
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_CLOSE

            case 4:
                # Gripper Up
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.OPEN_GRIPPER_DURATION:
                    self.current_sub_step = 5
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_UP

            case 5:
                # Backup from workstation
                if self.workstation_distance > (self.GRIP_DISTANCE + 0.5):
                    self.current_sub_step = 6
                    return MovementEnum.STOP
                else:
                    return MovementEnum.BACKWARD

            case 6:
                # Rotate 180°
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.Rotate_180_DURATION:
                    self.current_sub_step = self.SUB_ROUTINE_DONE 
                    return MovementEnum.STOP
                else:
                    return MovementEnum.TURN_RIGHT

    def marker_2_movement(self):

        match self.current_sub_step:
            case 0:
                # Get close to workstation
                if self.workstation_distance <= self.GRIP_DISTANCE:
                    self.current_sub_step = 1
                    return MovementEnum.STOP
                else:
                    return MovementEnum.FORWARD

            case 1:
                # Lower Gripper
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.LOWER_GRIPPER_DURATION:
                    self.current_sub_step = 2
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_DOWN

            case 2:
                # Open Gripper
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.OPEN_GRIPPER_DURATION:
                    self.current_sub_step = 3
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_OPEN

            case 3:
                # Backup from workstation
                if self.workstation_distance > (self.GRIP_DISTANCE + 0.5):
                    self.current_sub_step = 4 
                    return MovementEnum.STOP
                else:
                    return MovementEnum.BACKWARD

            case 4:
                # Rotate 180°
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.Rotate_180_DURATION:
                    self.current_sub_step = self.SUB_ROUTINE_DONE 
                    return MovementEnum.STOP
                else:
                    return MovementEnum.TURN_RIGHT

    def marker_3_movement(self):
        return MovementEnum.STOP

    def marker_4_movement(self):
        return MovementEnum.STOP
