import rclpy
from rclpy.node import Node
import sqlite3
import threading

from sensor_msgs.msg import Imu, MagneticField, NavSatFix

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        self.db_conn = sqlite3.connect('sensor_logs.db', check_same_thread=False)
        self.cursor = self.db_conn.cursor()
        self.setup_database()

        self.sub_imu1 = self.create_subscription(Imu, '/imu1', 
            lambda msg: self.imu_callback(msg, 'IMU1'), 10)
        
        self.sub_imu2 = self.create_subscription(Imu, '/imu2', 
            lambda msg: self.imu_callback(msg, 'IMU2'), 10)
        
        self.sub_mag = self.create_subscription(MagneticField, '/imu2/mag', 
            lambda msg: self.mag_callback(msg, 'MAG'), 10)
        
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', 
            lambda msg: self.gps_callback(msg, 'GPS'), 10)

        self.get_logger().info("Node is active. Monitoring IMU1, IMU2 Mag, and GPS...")

    def setup_database(self):
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS imu_logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,
            sensor TEXT,
            ax REAL, ay REAL, az REAL,
            gx REAL, gy REAL, gz REAL)''')
        
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS mag_logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,
            bx REAL, by REAL, bz REAL)''')
        self.db_conn.commit()

    def imu_callback(self, msg, sensor_name):
        accel_threshold = 0.1
        gyro_threshold = 0.05

        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        if any(abs(v) > accel_threshold for v in [ax, ay, az]) or \
           any(abs(v) > gyro_threshold for v in [gx, gy, gz]):
            
            print(f"--- {sensor_name} Motion Detected ---")
            print(f"Accel: [{ax:.2f}, {ay:.2f}, {az:.2f}] | Gyro: [{gx:.2f}, {gy:.2f}, {gz:.2f}]")
            
            timestamp = self.get_clock().now().to_msg().sec + (self.get_clock().now().to_msg().nanosec / 1e9)
            self.cursor.execute('''INSERT INTO imu_logs (timestamp, sensor, ax, ay, az, gx, gy, gz) 
                                   VALUES (?, ?, ?, ?, ?, ?, ?, ?)''', 
                                (timestamp, sensor_name, ax, ay, az, gx, gy, gz))
            self.db_conn.commit()

    def mag_callback(self, msg, sensor_name):
        b = msg.magnetic_field
        print(f"--- {sensor_name} Magnetic Field ---")
        print(f"Bx: {b.x:.2f} | By: {b.y:.2f} | Bz: {b.z:.2f}")

    def gps_callback(self, msg, sensor_name):
        print(f"--- {sensor_name} GPS Fix ---")
        print(f"Lat: {msg.latitude:.6f} | Lon: {msg.longitude:.6f} | Alt: {msg.altitude:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down node...")
    finally:
        node.db_conn.close()
        node.destroy_node()
        rclpy.shutdown()
        print("Cleanup complete.")

if __name__ == '__main__':
    main()