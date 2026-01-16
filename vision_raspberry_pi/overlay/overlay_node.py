import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class OverlayNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.frame_processed_subscriber = self.create_subscription( Image,
                                                                   '/camera/frame_processed',
                                                                   self.processed_frame_callback,
                                                                   10)
        self.serial_read_subscriber = self.create_subscription(String,
                                                               '/serial/read',
                                                               self.serial_read_callback,
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
        self.aligned_subscriber = self.create_subscription(
                Bool,
                '/ldlidar_node/aligned',
                self.aligned_callback,
                10)


        self.bridge = CvBridge()

        self.current_frame = cv2.imread("/app/release/overlay/no_signal.jpg")
        
        self.drive_mode = "Manuell Mode"
        self.front_collision_detected = False
        self.back_collision_detected = False
        self.left_collision_detected = False
        self.right_collision_detected = False
        self.aligned_to_marker = False

        cv2.namedWindow("EYECAR", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("EYECAR", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow("EYECAR", 832, 600)

        self.frame_timer = self.create_timer(0.03, self.show_window_callback)

    def processed_frame_callback(self, msg:Image):

        new_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.draw_crosshairs(new_frame)
        self.draw_box(new_frame)
        self.draw_drive_mode(new_frame)
        self.draw_collisions(new_frame)

        self.current_frame = new_frame

    def show_window_callback(self):
        if self.current_frame is not None:
                cv2.imshow("EYECAR", self.current_frame)
                cv2.waitKey(1)

    def draw_crosshairs(self, frame):
        cv2.line(frame, (396,312), (436, 312), (255,255,255), 1)
        cv2.line(frame, (416,292), (416, 332), (255,255,255), 1)

    def draw_box(self, frame):
        cv2.rectangle(frame, (15,20), (275,190), (50,50,50), -1)

    def draw_drive_mode(self, frame):
         cv2.putText(img=frame, 
                     text=self.drive_mode,
                     org=(20, 40),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7, color=(255, 255, 255),
                     thickness=1)

    def draw_collisions(self, frame):
         cv2.putText(img=frame,
                     text="Front: ",
                     org=(20, 60),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)
         cv2.putText(img=frame,
                     text=str(self.front_collision_detected),
                     org=(100, 60),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)

         cv2.putText(img=frame,
                     text="Left: ",
                     org=(20, 80),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)
         cv2.putText(img=frame,
                     text=str(self.left_collision_detected),
                     org=(100, 80),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)

         cv2.putText(img=frame,
                     text="Right: ",
                     org=(20, 100),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)
         cv2.putText(img=frame,
                     text=str(self.right_collision_detected),
                     org=(100, 100),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)

         cv2.putText(img=frame,
                     text="Back: ",
                     org=(20, 120),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)
         cv2.putText(img=frame,
                     text=str(self.back_collision_detected),
                     org=(100, 120),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)

         cv2.putText(img=frame,
                     text="Aligned: ",
                     org=(20, 140),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)
         cv2.putText(img=frame,
                     text=str(self.aligned_to_marker),
                     org=(100, 140),
                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                     fontScale=0.7,
                     color=(255, 255, 255),
                     thickness=1)

    def serial_read_callback(self, msg:String):
        if msg.data == "1":
            self.drive_mode = "Autonomous Mode"
        else:
            self.drive_mode = "Manuell Mode"


    def front_collision_callback(self, msg:Bool):

        self.front_collision_detected = msg.data

    def back_collision_callback(self, msg:Bool):

        self.back_collision_detected = msg.data

    def right_collision_callback(self, msg:Bool):

        self.right_collision_detected = msg.data

    def left_collision_callback(self, msg:Bool):

        self.left_collision_detected = msg.data

    def aligned_callback(self, msg:Bool):
        self.aligned_to_marker = msg.data

