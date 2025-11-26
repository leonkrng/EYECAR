import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Header

import serial

import math

import time

 

class LD06LaserScanPublisher(Node):

    def __init__(self):

        super().__init__('ld06_laser_scan_publisher')

        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)

        self.timer = self.create_timer(0.1, self.publish_scan)

        self.ser = serial.Serial('/dev/ttyAMA10', baudrate=230400, timeout=1)

 

    def parse_ld06_packet(self, packet):

        if len(packet) != 47 or packet[0] != 0x54 or packet[1] != 0x2C:

            return []

        angle = packet[2] + packet[3] * 256

        angle = angle / 100.0

        distances = []

        for i in range(12):

            offset = 4 + i * 3

            distance = packet[offset] + (packet[offset + 1] << 8)

            quality = packet[offset + 2]

            if quality > 0 and distance > 0:

                point_angle = angle + i * 1.0

                distances.append((point_angle % 360, distance / 1000.0))

        return distances

 

    def publish_scan(self):

        scan_msg = LaserScan()

        scan_msg.header = Header()

        scan_msg.header.stamp = self.get_clock().now().to_msg()

        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = 0.0

        scan_msg.angle_max = 2 * math.pi

        scan_msg.angle_increment = math.radians(1.0)

        scan_msg.time_increment = 0.0

        scan_msg.scan_time = 0.1

        scan_msg.range_min = 0.05

        scan_msg.range_max = 12.0

 

        ranges = [float('inf')] * 360

        while True:

            packet = self.ser.read(47)

            points = self.parse_ld06_packet(packet)

            if points:

                for angle_deg, distance in points:

                    index = int(angle_deg) % 360

                    ranges[index] = distance

                break

 

        scan_msg.ranges = ranges

        self.publisher_.publish(scan_msg)

 

def main(args=None):

    rclpy.init(args=args)

    node = LD06LaserScanPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

 

if __name__ == '__main__':

    main()
