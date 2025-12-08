import serial 
import socket
import rclpy
from sclpy.node import Node
from std_msgs.msg import String

class SerialConnectionNode(Node):
    def __init__(self):
        super().__init__('serial_connection')

        if socket.gethostname() != "eye-car-pi":
            self.ser = serial.serial_for_url('loop://', baudrate = 115200, timeout = 1)
        else:
            self.ser = serial.Serial(
                'dev/ttyAMA0',
                baudrate = 115200,
                bytesize = 8,
                timeout = 0.05,
                xonxoff = False,
                rtscts = False,
                dsrdtr = False,
            )

        self.read_pub = self.create_publisher(String, '/serial/read', 10)

        self.write_sub = self.create_subscription(
            String,
            '/serial/write',
            self.write_callback,
            10
        )

        # Read serial input every 10 ms
        self.read_timer = self.create_timer(0.01, self.read_serial)

    def write_callback(self, msg:String):
        try:
            msg_array = msg.data.split("/")
            self.ser.write(msg_array)
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()

            if len(line) > 0:
                # Publish
                message = String()
                .data = line
                self.telemetry_pub.publish(message)
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
