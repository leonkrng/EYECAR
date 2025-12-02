import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_sub')
        self.subscription = self.create_subscription(
            LaserScan,
            "/ldlidar_node/scan",
            self.lidar_callback,
            10)
        self.threshold = 0.5


    def lidar_callback(self, msg: LaserScan):

        max_distance = max(msg.ranges)


        if max_distance <= self.threshold:
            self.get_logger().info(f'Max-Distance: {max_distance} <= {self.threshold}')
        else:
            self.get_logger().info(f'Max-Distance: {max_distance} > {self.threshold}')
