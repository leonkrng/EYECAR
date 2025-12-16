import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import bisect

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


    def lidar_scan_callback(self, msg: LaserScan):

        max_distance = min(msg.ranges)

        if max_distance <= 0.3:
            self.get_logger().info(f'Collision detectet: {max_distance} m')
            msg = Bool()
            msg.data = True
            self.collision_publisher.publish(msg)



        
