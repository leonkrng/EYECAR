import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MainNode(Node):
    def __init__(self):
        super().__init__("main_node")

        # Serial publisher
        self.serial_write_pub = self.create_publisher(
            String,
                "/serial/write",
                10
        )

        self.serial_read_sub = self.create_subscription(
            String,
            "/serial/read",
            self.read_callback,
            10
        )

        self.telemetry_data = ["-1"] * 22
        self.prev_aruco_navigation_active = 0
        self.actual_ID = 0

        self.get_logger().info("Main-Node started.")

    def read_callback(self, msg: String):
        self.telemetry_data = msg.data.split("/")

        if len(self.telemetry_data) < 11:
            return

        if self.telemetry_data[10] == "1" and self.prev_aruco_navigation_active == 0:
            self.actual_ID = 0
            self.get_logger().info("Navigation restarted. Please start at first marker.")

        if self.telemetry_data[10] == "1":
            self.prev_aruco_navigation_active = 1
        else:
            self.prev_aruco_navigation_active = 0
