import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np
from rclpy.qos import qos_profile_system_default
import websockets
import asyncio
from std_msgs.msg import String

import csv
import time

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu', qos_profile_system_default)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('IMU Publisher has been started')

    async def timer_callback(self):
        # imu_msg = Imu()
        # imu_msg.header.stamp = self.get_clock().now().to_msg()
        # imu_msg.header.frame_id = 'base_link'

        # print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        # print("Magnetometer (microteslas): {}".format(sensor.magnetic))
        # print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        # print("Euler angle: {}".format(sensor.euler))
        # print("Quaternion: {}".format(sensor.quaternion))
        # print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
        # print("Gravity (m/s^2): {}".format(sensor.gravity))

        # Example orientation (quaternion)

        # with open('/home/kb/Anveshika/imu_readings.csv', 'r') as csvfile:
        # # Create a CSV reader
        #     csv_reader = csv.reader(csvfile)
            
        #     # Read the header row (optional)
        #     header = next(csv_reader)
        #     print("Header:", header)
            
        #     # Move the file pointer to the beginning
        #     csvfile.seek(0)
            
        #     # Skip the header row
        #     next(csv_reader)
            
        #     # Read the latest row of sensor data
        #     latest_values = next(csv_reader)
            
        #     # Print the sensor values
        #     print("Temperature (C):", latest_values[0])
        #     print("Accelerometer (m/s^2):", latest_values[1])
        #     print("Magnetometer (microteslas):", latest_values[2])
        #     print("Gyroscope (rad/sec):", latest_values[3])
        #     print("Euler angle:", latest_values[4])
        #     print("Quaternion:", latest_values[5])
        #     print("Linear acceleration (m/s^2):", latest_values[6])
        #     print("Gravity (m/s^2):", latest_values[7])
        #     print()

        #     # Wait for 1 second before reading again
        #     time.sleep(1)

        uri = "ws://localhost:8765"  # WebSocket server URI
        async with websockets.connect(uri) as websocket:
            while True:
                try:
                    # Receive data from WebSocket
                    data = await websocket.recv()
                    self.get_logger().info(f"Received data: {data}")

                    # Convert data to a ROS message
                    msg = String()
                    msg.data = data

                    # Publish data to the ROS 2 topic
                    self.publisher_.publish(msg)

                except websockets.ConnectionClosed as e:
                    self.get_logger().error(f"WebSocket connection closed: {e}")
                    break
                except Exception as e:
                    self.get_logger().error(f"Error receiving data: {e}")

                # Optional: Sleep or handle other tasks
                await asyncio.sleep(0.1)

    def start_listening(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self.timer_callback())


        # imu_msg.orientation = Quaternion(x = latest_values[5][0],
        #                                  y = latest_values[5][1],
        #                                  z = latest_values[5][2],
        #                                  w = latest_values[5][3]
        #                                  )
        # imu_msg.orientation_covariance = [0.0] * 9

        # # Example angular velocity (rad/s)
        # imu_msg.angular_velocity.x = latest_values[3][0]
        # imu_msg.angular_velocity.y = latest_values[3][1]
        # imu_msg.angular_velocity.z = latest_values[3][2]
        # imu_msg.angular_velocity_covariance = [0.0] * 9

        # # Example linear acceleration (m/s^2)
        # imu_msg.linear_acceleration.x = latest_values[1][0]
        # imu_msg.linear_acceleration.y = latest_values[1][1]
        # imu_msg.linear_acceleration.z = latest_values[1][2]
        # imu_msg.linear_acceleration_covariance = [0.0] * 9

        # self.publisher_.publish(imu_msg)
        # self.get_logger().info('Publishing IMU data')

def main(args=None):
    try:
        rclpy.init(args=args)
        imu_publisher = IMUNode()
        imu_publisher.start_listening()
        rclpy.spin(imu_publisher)
    finally:    
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
