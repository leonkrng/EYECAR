import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from itertools import chain
import math

class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        # Lidar Zone-Ranges in RAD. Some zones overlap
        self.FRONT_1_START = 0.0 # 0° 
        self.FRONT_1_END = 3.142 # 180°
        self.FRONT_2_START = 6.109 # 350°
        self.FRONT_2_END = 6.283 # 360°

        self.BACK_START = 3.316 # 190°
        self.BACK_END = 6.109 # 350°

        self.LEFT_START = 2.967 # 170°
        self.LEFT_END = 3.491 # 200°

        self.RIGHT_1_START = 0 # 0°
        self.RIGHT_1_END = 0.3491 # 20°
        self.RIGHT_2_START = 6.109 # 350°
        self.RIGHT_2_END = 6.283 # 360°

        # Collision Thresholds in meter
        self.FRONT_COLL = 0.30
        self.BACK_COLL = 0.45
        self.SIDE_COLL = 0.35

        self.lidar_scan_subscriber = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.lidar_scan_callback,
            10)

        self.collision_publisher_front = self.create_publisher(Bool,
                                                         '/ldlidar_node/collision_front',
                                                         10)
        self.collision_publisher_back = self.create_publisher(Bool,
                                                         '/ldlidar_node/collision_back',
                                                         10)
        self.collision_publisher_left = self.create_publisher(Bool,
                                                         '/ldlidar_node/collision_left',
                                                         10)
        self.collision_publisher_right = self.create_publisher(Bool,
                                                         '/ldlidar_node/collision_right',
                                                         10)


    def lidar_scan_callback(self, scan: LaserScan):

        # Get the valid ranges of each EYECAR-side
        front_ranges_1 = self.get_valid_ranges(scan.ranges, self.FRONT_1_START, self.FRONT_1_END, scan.angle_min, scan.angle_increment)
        front_ranges_2 = self.get_valid_ranges(scan.ranges, self.FRONT_1_START, self.FRONT_2_END, scan.angle_min, scan.angle_increment)
        front_ranges = list(chain(front_ranges_1, front_ranges_2))

        back_ranges = self.get_valid_ranges(scan.ranges, self.BACK_START, self.BACK_END, scan.angle_min, scan.angle_increment)

        left_ranges = self.get_valid_ranges(scan.ranges, self.LEFT_START, self.LEFT_END, scan.angle_min, scan.angle_increment)

        right_ranges_1 = self.get_valid_ranges(scan.ranges, self.RIGHT_1_START, self.RIGHT_1_END, scan.angle_min, scan.angle_increment)
        right_ranges_2 = self.get_valid_ranges(scan.ranges, self.RIGHT_2_START, self.RIGHT_2_END, scan.angle_min, scan.angle_increment)
        right_ranges = list(chain(right_ranges_1, right_ranges_2))

        coll_msg = Bool()
        coll_msg.data = False

        # Check collision for each EYECAR-side
        if min(front_ranges) <= self.FRONT_COLL:
            coll_msg.data = True
            self.collision_publisher_front.publish(coll_msg)
        else:
            coll_msg.data = False
            self.collision_publisher_front.publish(coll_msg)

        if min(back_ranges) <= self.BACK_COLL:
            coll_msg.data = True
            self.collision_publisher_back.publish(coll_msg)
        else:
            coll_msg.data = False
            self.collision_publisher_back.publish(coll_msg)

        if min(left_ranges) <= self.SIDE_COLL:
            coll_msg.data = True
            self.collision_publisher_left.publish(coll_msg)
        else:
            coll_msg.data = False
            self.collision_publisher_left.publish(coll_msg)

        if min(right_ranges) <= self.SIDE_COLL:
            coll_msg.data = True
            self.collision_publisher_right.publish(coll_msg)
        else:
            coll_msg.data = False
            self.collision_publisher_right.publish(coll_msg)


    def get_valid_ranges(self, scan_ranges, start, end, angle_min, angle_increment):
        i_begin = math.ceil((start - angle_min) / angle_increment)
        i_end   = math.floor((end   - angle_min) / angle_increment)

        # Bounds absichern
        i_begin = max(0, i_begin)
        i_end   = min(len(scan_ranges) - 1, i_end)

        if i_end < i_begin:
            return [], (i_begin, i_end)

        ranges_slice = scan_ranges[i_begin:i_end + 1]

        valid_ranges = [
            float(r) for r in ranges_slice
            if math.isfinite(r) and r >= 0.0
        ]

        return valid_ranges
