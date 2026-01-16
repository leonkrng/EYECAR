import serial 
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SerialConnectionNode(Node):
    def __init__(self):
        super().__init__('serial_connection')

        self.ser = serial.Serial(
            '/dev/ttyAMA0',
            baudrate = 115200,
            bytesize = 8,
            timeout = 0.05,
            xonxoff = False,
            rtscts = False,
            dsrdtr = False,
        )

        self.serial_read_publisher = self.create_publisher(String,
                                              '/serial/read',
                                              10)

        self.serial_write_subscriber = self.create_subscription(
            String,
            '/serial/write',
            self.serial_write_callback,
            10
        )

        # Read serial input every 100 ms
        self.read_timer = self.create_timer(0.1, self.read_serial)


    def serial_write_callback(self, msg: String):
        try:
            data = msg.data.encode("utf-8") + b"\n"
            self.ser.write(data)
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


    def read_serial(self):
        try:
            raw = self.ser.readline()
            line = raw.decode("utf-8", errors="ignore").strip()

            if line:
                msg = String()
                msg.data = line
                self.serial_read_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
