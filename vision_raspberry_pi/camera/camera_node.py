import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.frame_raw_publisher = self.create_publisher(Image,
                                                         'camera/frame_raw',
                                                         10)

        self.bridge = CvBridge()

        gst_pipeline = (
            "udpsrc port=8000 ! jpegdec ! videoconvert ! "
            "appsink emit-signals=false drop=true sync=false max-buffers=1")
        
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.fallback_image_path = os.path.join(base_dir, "no_signal.jpg")

        try:
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception as e:
            self.get_logger().error(f"Could not start videocapture because of error: {e}")

        self.timer = self.create_timer(0.02, self.publish_frame_raw)

    def publish_frame_raw(self):
        try:

            if self.cap.isOpened():

                ret, frame = self.cap.read()

                if not ret:
                    self.get_logger().error("Failed to read frame.")
                    return
            else:
                self.get_logger().error("Camera stream is not opened.")
                return

            #cv2.imshow("Debug Window", frame)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.frame_raw_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Could not publish frame because of error: {e}")
