import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        self.lidar_scan_subscriber = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.lidar_scan_callback,
            10)

        self.collision_publisher = self.create_publisher(Bool,
                                                         '/ldlidar_node/collision',
                                                         10)


    def lidar_scan_callback(self, scan: LaserScan):

        valid_ranges = [
            r for r in scan.ranges
            if not math.isinf(r) and not math.isnan(r) and r > 0.0
        ]

        if not valid_ranges:
            return

        min_distance = min(valid_ranges)

        collision = False

        if min_distance <= 0.3:
            collision = True
            self.get_logger().info(
                f'Collision detected: {min_distance:.2f} m'
            )

        collision_msg = Bool()
        collision_msg.data = collision 
        self.collision_publisher.publish(collision_msg)


            
