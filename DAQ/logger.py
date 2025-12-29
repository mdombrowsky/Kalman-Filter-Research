import rclpy
from rclpy.node import Node
import sqlite3
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Imu, MagneticField, NavSatFix

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        self.db_lock = threading.Lock()
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.db_conn = sqlite3.connect('sensor_logs.db', check_same_thread=False)
        self.cursor = self.db_conn.cursor()
        self.setup_database()

        self.sub_imu1 = self.create_subscription(
            Imu, '/imu1', lambda msg: self.imu_callback(msg, 'IMU1'), 10,
            callback_group=self.callback_group)
        
        self.sub_imu2 = self.create_subscription(
            Imu, '/imu2', lambda msg: self.imu_callback(msg, 'IMU2'), 10,
            callback_group=self.callback_group)
        
        self.sub_mag = self.create_subscription(
            MagneticField, '/imu2/mag', lambda msg: self.mag_callback(msg, 'MAG'), 10,
            callback_group=self.callback_group)
        
        self.sub_gps = self.create_subscription(
            NavSatFix, '/gps/fix', lambda msg: self.gps_callback(msg, 'GPS'), 10,
            callback_group=self.callback_group)

        self.create_timer(1.0, self.timer_commit_callback, callback_group=self.callback_group)

        self.get_logger().info("Multi-threaded Node active.")

    def setup_database(self):
        with self.db_lock:
            self.cursor.execute('''CREATE TABLE IF NOT EXISTS imu_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL, sensor TEXT,
                ax REAL, ay REAL, az REAL,
                gx REAL, gy REAL, gz REAL)''')
            
            self.cursor.execute('''CREATE TABLE IF NOT EXISTS mag_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL, bx REAL, by REAL, bz REAL)''')
            
            self.cursor.execute('''CREATE TABLE IF NOT EXISTS gps_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL, lat REAL, lon REAL, alt REAL)''')
            self.db_conn.commit()

    def timer_commit_callback(self):
        with self.db_lock:
            self.db_conn.commit()
            # self.get_logger().debug("Database changes committed.")

    def get_timestamp(self):
        t = self.get_clock().now().to_msg()
        return t.sec + (t.nanosec / 1e9)

    def imu_callback(self, msg, sensor_name):
        accel_threshold = 0.1
        gyro_threshold = 0.05

        t = msg.header.stamp
        timestamp = t.sec + (t.nanosec / 1e9)
        
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        if any(abs(v) > 0.1 for v in [ax, ay, az]) or any(abs(v) > 0.05 for v in [gx, gy, gz]):
            #timestamp = self.get_timestamp()
            with self.db_lock:
                self.cursor.execute('''INSERT INTO imu_logs (timestamp, sensor, ax, ay, az, gx, gy, gz) 
                                       VALUES (?, ?, ?, ?, ?, ?, ?, ?)''', 
                                    (timestamp, sensor_name, ax, ay, az, gx, gy, gz))
            
            print(f"--- {sensor_name} IMU Motion ---")
            print(f"Accel: [{ax:.2f}, {ay:.2f}, {az:.2f}] | Gyro: [{gx:.2f}, {gy:.2f}, {gz:.2f}]")   

    def mag_callback(self, msg, sensor_name):
        
        t = msg.header.stamp
        timestamp = t.sec + (t.nanosec / 1e9)
        #timestamp = self.get_timestamp()
        
        b = msg.magnetic_field
        with self.db_lock:
            self.cursor.execute('''INSERT INTO mag_logs (timestamp, bx, by, bz) VALUES (?, ?, ?, ?)''', 
                                (timestamp, b.x, b.y, b.z))

        print(f"--- {sensor_name} Magnetic Field ---")
        print(f"Bx: {b.x:.2f} | By: {b.y:.2f} | Bz: {b.z:.2f}")

    def gps_callback(self, msg, sensor_name):

        t = msg.header.stamp
        timestamp = t.sec + (t.nanosec / 1e9)
        #timestamp = self.get_timestamp()
        
        with self.db_lock:
            self.cursor.execute('''INSERT INTO gps_logs (timestamp, lat, lon, alt) VALUES (?, ?, ?, ?)''', 
                                (timestamp, msg.latitude, msg.longitude, msg.altitude))

        print(f"--- {sensor_name} GPS Fix ---")
        print(f"Lat: {msg.latitude:.6f} | Lon: {msg.longitude:.6f} | Alt: {msg.altitude:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down node...")
        pass
    finally:
        node.db_conn.close()
        node.destroy_node()
        rclpy.shutdown()
        print("Cleanup complete.")

if __name__ == '__main__':
    main()