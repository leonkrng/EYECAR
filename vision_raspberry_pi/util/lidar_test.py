
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan   # Nachrichtentyp für LiDAR-Daten

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        # Subscriber auf das Topic /scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10)

    def listener_callback(self, msg: LaserScan):
        # Beispiel: kleinste Distanz und erste Intensität ausgeben
        min_distance = min(msg.ranges)
        max_distance = max(msg.ranges)
        first_intensity = msg.intensities[0] if msg.intensities else None
        self.get_logger().info(
            f'Min-Distanz: {min_distance:.2f} m | Max-Distanz: {max_distance}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)   # hält den Node aktiv und ruft listener_callback auf
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
