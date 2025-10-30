
from rplidar import RPLidar

lidar = RPLidar('/dev/serial0', baudrate=115200)

try:
    info = lidar.get_info()
    print("LiDAR Info:", info)
except Exception as e:
    print("Fehler:", e)
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
