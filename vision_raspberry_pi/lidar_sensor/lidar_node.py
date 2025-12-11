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

        # Split the message in one part for each EYECAR side
        side_ranges = [msg.ranges[i:i +120] for i in range(0, len(msg.ranges),120)]

        for side in side_ranges:

            max_distance = max(side)

            if max_distance <= 0.3:
                self.get_logger().info(f'Collision detectet: {max_distance} m')
                self.collision_publisher.publish(True)



        
