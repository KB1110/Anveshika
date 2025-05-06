# import rclpy
# from rclpy.node import Node
# from rclpy.qos import qos_profile_system_default
# import csv

# from std_msgs.msg import Float32
# from std_msgs.msg import String

# from simple_pid import PID
# import time

# class ArmNode(Node):

#     def __init__(self):
#         super().__init__('driving_node')

#         # self.base_subscriber = self.create_subscription(Float32, '/base_angle', self.base_callback, qos_profile_system_default)
#         # self.shoulder_subscriber = self.create_subscription(Float32, '/shoulder_angle', self.shoulder_callback, qos_profile_system_default)
#         # self.elbow_subscriber = self.create_subscription(Float32, '/elbow_angle', self.elbow_callback, qos_profile_system_default)
#         # self.wrist1_subscriber = self.create_subscription(Float32, '/wrist1_angle', self.wrist1_callback, qos_profile_system_default)
#         # self.wrist2_subscriber = self.create_subscription(Float32, '/wrist2_angle', self.wrist2_callback, qos_profile_system_default)
#         # self.gripper_subscriber = self.create_subscription(Float32, '/gripper_angle', self.gripper_callback, qos_profile_system_default)

#         self.error_subscriber = self.create_subscription(Float32, '/error', self.error_callback, qos_profile_system_default)
#         self.error_x_subscriber = self.create_subscription(Float32, '/error_x', self.error_x_callback, qos_profile_system_default)
#         self.error_y_subscriber = self.create_subscription(Float32, '/error_y', self.error_y_callback, qos_profile_system_default)

#         self.center_x_subscriber = self.create_subscription(Float32, '/center_x', self.center_x_callback, qos_profile_system_default)
#         self.center_y_subscriber = self.create_subscription(Float32, '/center_y', self.center_y_callback, qos_profile_system_default)

#         self.mode_publisher = self.create_publisher(String, '/mode_in', qos_profile_system_default)
#         self.mode_subscriber = self.create_subscription(String, '/mode_out', self.mode_callback, qos_profile_system_default)

#         self.mode = "4"

#         self.error = 0.0
#         self.error_x = 0.0
#         self.error_y = 0.0

#         self.center_x = 0.0
#         self.center_y = 0.0

#         self.default_pos = [90, 120, 180, 90, 120, 50]
#         self.search_pos = [90, 90, 150, 90, 120, 50]
#         self.pick_up_pos = [90, 50, 180, 90, 120, 50]

#         self.kp = 0.1
#         self.kd = 0.05
#         self.ki = 0.0

#         self.base_pickup_angle = self.pick_up_pos[0]
#         self.shoulder_pickup_angle = self.pick_up_pos[1]
#         self.elbow_pickup_angle = self.pick_up_pos[2]


#         self.pid = PID(self.kp, self.kd, self.ki, setpoint=0)

#         self.base_angle = self.default_pos[0]
#         self.shoulder_angle = self.default_pos[1]
#         self.elbow_angle = self.default_pos[2]
#         self.wrist1_angle = self.default_pos[3]
#         self.wrist2_angle = self.default_pos[4]
#         self.gripper_angle = self.default_pos[5]

#         self.timer = self.create_timer(0.1, self.timer_callback)

#         # Open CSV file for appending and create a CSV writer object
#         self.csv_file = open('/home/pi/Anveshika/arm_data.csv', mode='a', newline='')
#         self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=["base",
#                                                                     "shoulder",
#                                                                     "elbow",
#                                                                     "wrist1",
#                                                                     "wrist2",
#                                                                     "gripper"])

#         # Write header only if file is new
#         if self.csv_file.tell() == 0:
#             self.csv_writer.writeheader()
    
#     # def base_callback(self, msg):
#     #     self.base_angle = msg.data
    
#     # def shoulder_callback(self, msg):
#     #     self.shoulder_angle = msg.data

#     # def elbow_callback(self, msg):
#     #     self.elbow_angle = msg.data

#     # def wrist1_callback(self, msg):
#     #     self.wrist1_angle = msg.data

#     # def wrist2_callback(self, msg):
#     #     self.wrist2_angle = msg.data

#     # def gripper_callback(self, msg):
#     #     self.gripper_angle = msg.data

#     def mode_callback(self, msg):
#         self.mode = msg.data

#     def error_callback(self, msg):
#         self.error = msg.data

#     def error_x_callback(self, msg):
#         self.error_x = msg.data

#     def error_y_callback(self, msg):
#         self.error_y = msg.data

#     def center_x_callback(self, msg):
#         self.center_x = msg.data

#     def center_y_callback(self, msg):
#         self.center_y = msg.data

#     def timer_callback(self):
        
#         # Rover will turn to face the waypoint coordinate or navigate, or stop
#         if self.mode == "0" or self.mode == "1" or self.mode == "-1":

#             data = {"base": self.default_pos[0],
#                     "shoulder": self.default_pos[1],
#                     "elbow": self.default_pos[2],
#                     "wrist1": self.default_pos[3],
#                     "wrist2": self.default_pos[4],
#                     "gripper": self.default_pos[5]}

#         # Searches for sample or container or travels to them
#         elif self.mode == "2" or self.mode == "3" or self.mode == "5" or self.mode == "6":

#             data = {"base": self.search_pos[0],
#                     "shoulder": self.search_pos[1],
#                     "elbow": self.search_pos[2],
#                     "wrist1": self.search_pos[3],
#                     "wrist2": self.search_pos[4],
#                     "gripper": self.search_pos[5]}
        
#         elif self.mode == "4" or self.mode == "7":
                
#             # data = {"base": self.base_angle,
#             #     "shoulder": self.shoulder_angle,
#             #     "elbow": self.elbow_angle,
#             #     "wrist1": self.wrist1_angle,
#             #     "wrist2": self.wrist2_angle,
#             #     "gripper": self.gripper_angle}

#             # print(self.center_x, self.error_x, self.center_y, self.error_y)

#             if self.center_x == 0.0:
            
#                 # print(self.error_x)

#                 control_x = self.pid(self.error_x)

#                 control_x *= -1

#                 control_x = max(-40, min(40, control_x))

#                 # print(control)

#                 self.base_pickup_angle = self.pick_up_pos[0] + control_x

#                 data = {"base":  + self.base_pickup_angle,
#                         "shoulder": self.pick_up_pos[1],
#                         "elbow": self.pick_up_pos[2],
#                         "wrist1": self.pick_up_pos[3],
#                         "wrist2": self.pick_up_pos[4],
#                         "gripper": self.pick_up_pos[5]}
            
#             if self.center_x == 1.0: #and self.center_y == 0.0:
#                 # print(self.error_y)

#                 control_y = self.pid(self.error_y)

#                 control_y *= -2

#                 control_y = max(-40, min(40, control_y))
#                 self.shoulder_pickup_angle = max(0, min(180, self.pick_up_pos[1] + control_y))
#                 self.elbow_pickup_angle = max(0, min(180, self.pick_up_pos[2] + (2 * control_y)))

#                 # self.shoulder_pickup_angle = 15
#                 # self.elbow_pickup_angle = 110

#                 data = {"base": self.base_pickup_angle,
#                         "shoulder": self.shoulder_pickup_angle,
#                         "elbow": self.elbow_pickup_angle,
#                         "wrist1": self.pick_up_pos[3],
#                         "wrist2": self.pick_up_pos[4] + 30,
#                         "gripper": self.pick_up_pos[5]}
                
#                 print(self.shoulder_pickup_angle, self.elbow_pickup_angle)
                
#             if self.center_x == 1.0 and self.center_y == 1.0:
#                 self.pickup_sample()
        
#         try:
#             # Write data to the CSV file
#             self.csv_writer.writerow(data)
            
#             # Flush the file to ensure data is written immediately
#             self.csv_file.flush()
            
#             # self.get_logger().info(f"Data written to CSV: {data}")
#         except IOError as e:
#             self.get_logger().error(f"Error writing to CSV: {e}")
    
#     def destroy_node(self):
#         # Close the CSV file when the node is destroyed
#         if self.csv_file:
#             self.csv_file.close()
#         super().destroy_node()

#     def pickup_sample(self):
#         time.sleep(5)
#         data = {"base": self.base_pickup_angle,
#                     "shoulder": self.shoulder_pickup_angle,
#                     "elbow": self.elbow_pickup_angle,
#                     "wrist1": self.pick_up_pos[3],
#                     "wrist2": self.pick_up_pos[4],
#                     "gripper": 130}
        
#         try:
#             # Write data to the CSV file
#             self.csv_writer.writerow(data)
            
#             # Flush the file to ensure data is written immediately
#             self.csv_file.flush()
            
#             # self.get_logger().info(f"Data written to CSV: {data}")
#         except IOError as e:
#             self.get_logger().error(f"Error writing to CSV: {e}")


#         time.sleep(5)
#         data = {"base": self.base_pickup_angle,
#                 # "shoulder": max(0, self.shoulder_pickup_angle - 10),
#                 "shoulder": self.shoulder_pickup_angle,
#                 "elbow": self.elbow_pickup_angle - 60,
#                 "wrist1": self.pick_up_pos[3],
#                 "wrist2": self.pick_up_pos[4],
#                 "gripper": 130}
        
#         try:
#             # Write data to the CSV file
#             self.csv_writer.writerow(data)
            
#             # Flush the file to ensure data is written immediately
#             self.csv_file.flush()
            
#             # self.get_logger().info(f"Data written to CSV: {data}")
#         except IOError as e:
#             self.get_logger().error(f"Error writing to CSV: {e}")

#         time.sleep(5)
        
#         data = {"base": self.base_pickup_angle,
#                 # "shoulder": max(0, self.shoulder_pickup_angle - 10),
#                 "shoulder": 0,
#                 "elbow": self.elbow_pickup_angle - 60,
#                 "wrist1": self.pick_up_pos[3],
#                 "wrist2": self.pick_up_pos[4],
#                 "gripper": 130}
        
#         try:
#             # Write data to the CSV file
#             self.csv_writer.writerow(data)
            
#             # Flush the file to ensure data is written immediately
#             self.csv_file.flush()
            
#             # self.get_logger().info(f"Data written to CSV: {data}")
#         except IOError as e:
#             self.get_logger().error(f"Error writing to CSV: {e}")
        
#         time.sleep(5)

#         data = {"base": self.base_pickup_angle,
#                 # "shoulder": max(0, self.shoulder_pickup_angle - 10),
#                 "shoulder": 0,
#                 "elbow": self.elbow_pickup_angle - 60,
#                 "wrist1": self.pick_up_pos[3],
#                 "wrist2": self.pick_up_pos[4],
#                 "gripper": 50}
        
#         try:
#             # Write data to the CSV file
#             self.csv_writer.writerow(data)
            
#             # Flush the file to ensure data is written immediately
#             self.csv_file.flush()
            
#             # self.get_logger().info(f"Data written to CSV: {data}")
#         except IOError as e:
#             self.get_logger().error(f"Error writing to CSV: {e}")
        
#         time.sleep(5)

#         data = {"base": self.default_pos[0],
#                 "shoulder": self.default_pos[1],
#                 "elbow": self.default_pos[2],
#                 "wrist1": self.default_pos[3],
#                 "wrist2": self.default_pos[4],
#                 "gripper": self.default_pos[5]}
        
#         try:
#             # Write data to the CSV file
#             self.csv_writer.writerow(data)
            
#             # Flush the file to ensure data is written immediately
#             self.csv_file.flush()
            
#             # self.get_logger().info(f"Data written to CSV: {data}")
#         except IOError as e:
#             self.get_logger().error(f"Error writing to CSV: {e}")

#         self.mode = "-1"
        

    
# def main(args=None):
#     rclpy.init(args=args)

#     # Initialize and run the driving node
#     arm_node = ArmNode()

#     try:
#         rclpy.spin(arm_node)  # Keep the node running
#     finally:
#         arm_node.destroy_node()  # Ensure node and resources are properly destroyed
#         rclpy.shutdown()  # Shutdown rclpy

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import csv

from std_msgs.msg import Float32
from std_msgs.msg import String

from simple_pid import PID
import time

class ArmNode(Node):

    def __init__(self):
        super().__init__('driving_node')

        # self.base_subscriber = self.create_subscription(Float32, '/base_angle', self.base_callback, qos_profile_system_default)
        # self.shoulder_subscriber = self.create_subscription(Float32, '/shoulder_angle', self.shoulder_callback, qos_profile_system_default)
        # self.elbow_subscriber = self.create_subscription(Float32, '/elbow_angle', self.elbow_callback, qos_profile_system_default)
        # self.wrist1_subscriber = self.create_subscription(Float32, '/wrist1_angle', self.wrist1_callback, qos_profile_system_default)
        # self.wrist2_subscriber = self.create_subscription(Float32, '/wrist2_angle', self.wrist2_callback, qos_profile_system_default)
        # self.gripper_subscriber = self.create_subscription(Float32, '/gripper_angle', self.gripper_callback, qos_profile_system_default)

        self.error_subscriber = self.create_subscription(Float32, '/error', self.error_callback, qos_profile_system_default)
        self.error_x_subscriber = self.create_subscription(Float32, '/error_x', self.error_x_callback, qos_profile_system_default)
        self.error_y_subscriber = self.create_subscription(Float32, '/error_y', self.error_y_callback, qos_profile_system_default)

        self.center_x_subscriber = self.create_subscription(Float32, '/center_x', self.center_x_callback, qos_profile_system_default)
        self.center_y_subscriber = self.create_subscription(Float32, '/center_y', self.center_y_callback, qos_profile_system_default)

        self.mode_publisher = self.create_publisher(String, '/mode_in', qos_profile_system_default)
        self.mode_subscriber = self.create_subscription(String, '/mode_out', self.mode_callback, qos_profile_system_default)

        self.mode = "4"

        self.error = 0.0
        self.error_x = 0.0
        self.error_y = 0.0

        self.center_x = 0.0
        self.center_y = 0.0

        self.default_pos = [90, 120, 180, 90, 120, 50]
        self.search_pos = [90, 90, 150, 90, 120, 50]
        self.pick_up_pos = [90, 50, 180, 90, 120, 50]

        self.kp = 0.1
        self.kd = 0.05
        self.ki = 0.0

        self.base_pickup_angle = self.pick_up_pos[0]
        self.shoulder_pickup_angle = self.pick_up_pos[1]
        self.elbow_pickup_angle = self.pick_up_pos[2]


        self.pid = PID(self.kp, self.kd, self.ki, setpoint=0)

        self.base_angle = self.default_pos[0]
        self.shoulder_angle = self.default_pos[1]
        self.elbow_angle = self.default_pos[2]
        self.wrist1_angle = self.default_pos[3]
        self.wrist2_angle = self.default_pos[4]
        self.gripper_angle = self.default_pos[5]

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Open CSV file for appending and create a CSV writer object
        self.csv_file = open('/home/pi/Anveshika/arm_data.csv', mode='a', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=["base",
                                                                    "shoulder",
                                                                    "elbow",
                                                                    "wrist1",
                                                                    "wrist2",
                                                                    "gripper"])

        # Write header only if file is new
        if self.csv_file.tell() == 0:
            self.csv_writer.writeheader()
    
    # def base_callback(self, msg):
    #     self.base_angle = msg.data
    
    # def shoulder_callback(self, msg):
    #     self.shoulder_angle = msg.data

    # def elbow_callback(self, msg):
    #     self.elbow_angle = msg.data

    # def wrist1_callback(self, msg):
    #     self.wrist1_angle = msg.data

    # def wrist2_callback(self, msg):
    #     self.wrist2_angle = msg.data

    # def gripper_callback(self, msg):
    #     self.gripper_angle = msg.data

    def mode_callback(self, msg):
        self.mode = msg.data

    def error_callback(self, msg):
        self.error = msg.data

    def error_x_callback(self, msg):
        self.error_x = msg.data

    def error_y_callback(self, msg):
        self.error_y = msg.data

    def center_x_callback(self, msg):
        self.center_x = msg.data

    def center_y_callback(self, msg):
        self.center_y = msg.data

    def timer_callback(self):
        
        # Rover will turn to face the waypoint coordinate or navigate, or stop
        if self.mode == "0" or self.mode == "1" or self.mode == "-1":

            data = {"base": self.default_pos[0],
                    "shoulder": self.default_pos[1],
                    "elbow": self.default_pos[2],
                    "wrist1": self.default_pos[3],
                    "wrist2": self.default_pos[4],
                    "gripper": self.default_pos[5]}

        # Searches for sample or container or travels to them
        elif self.mode == "2" or self.mode == "3" or self.mode == "5" or self.mode == "6":

            data = {"base": self.search_pos[0],
                    "shoulder": self.search_pos[1],
                    "elbow": self.search_pos[2],
                    "wrist1": self.search_pos[3],
                    "wrist2": self.search_pos[4],
                    "gripper": self.search_pos[5]}
        
        elif self.mode == "4" or self.mode == "7":
                
            # data = {"base": self.base_angle,
            #     "shoulder": self.shoulder_angle,
            #     "elbow": self.elbow_angle,
            #     "wrist1": self.wrist1_angle,
            #     "wrist2": self.wrist2_angle,
            #     "gripper": self.gripper_angle}

            # print(self.center_x, self.error_x, self.center_y, self.error_y)

            if self.center_x == 0.0:
            
                # print(self.error_x)

                control_x = self.pid(self.error_x)

                control_x *= -1

                control_x = max(-40, min(40, control_x))

                # print(control)

                self.base_pickup_angle = self.pick_up_pos[0] + control_x

                data = {"base":  + self.base_pickup_angle,
                        "shoulder": self.pick_up_pos[1],
                        "elbow": self.pick_up_pos[2],
                        "wrist1": self.pick_up_pos[3],
                        "wrist2": self.pick_up_pos[4],
                        "gripper": self.pick_up_pos[5]}
            
            if self.center_x == 1.0: #and self.center_y == 0.0:
                # print(self.error_y)

                control_y = self.pid(self.error_y)

                control_y *= -1

                control_y = max(-40, min(40, control_y))
                self.shoulder_pickup_angle = max(0, min(180, self.pick_up_pos[1] + control_y))
                self.elbow_pickup_angle = max(0, min(180, self.pick_up_pos[2] + (2 * control_y)))

                # self.shoulder_pickup_angle = 15
                # self.elbow_pickup_angle = 110

                data = {"base": self.base_pickup_angle,
                        "shoulder": self.shoulder_pickup_angle,
                        "elbow": self.elbow_pickup_angle,
                        "wrist1": self.pick_up_pos[3],
                        "wrist2": self.pick_up_pos[4] + 30,
                        "gripper": self.pick_up_pos[5]}
                
                print(self.shoulder_pickup_angle, self.elbow_pickup_angle)
                
            if self.center_x == 1.0 and self.center_y == 1.0:
                # self.pickup_sample()
                pass

        try:
            # Write data to the CSV file
            self.csv_writer.writerow(data)
            
            # Flush the file to ensure data is written immediately
            self.csv_file.flush()
            
            # self.get_logger().info(f"Data written to CSV: {data}")
        except IOError as e:
            self.get_logger().error(f"Error writing to CSV: {e}")
    
    def destroy_node(self):
        # Close the CSV file when the node is destroyed
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

    def pickup_sample(self):
        time.sleep(5)
        data = {"base": self.base_pickup_angle,
                    "shoulder": self.shoulder_pickup_angle,
                    "elbow": self.elbow_pickup_angle,
                    "wrist1": self.pick_up_pos[3],
                    "wrist2": self.pick_up_pos[4],
                    "gripper": 130}
        
        try:
            # Write data to the CSV file
            self.csv_writer.writerow(data)
            
            # Flush the file to ensure data is written immediately
            self.csv_file.flush()
            
            # self.get_logger().info(f"Data written to CSV: {data}")
        except IOError as e:
            self.get_logger().error(f"Error writing to CSV: {e}")


        time.sleep(5)
        data = {"base": self.base_pickup_angle,
                # "shoulder": max(0, self.shoulder_pickup_angle - 10),
                "shoulder": self.shoulder_pickup_angle,
                "elbow": self.elbow_pickup_angle - 60,
                "wrist1": self.pick_up_pos[3],
                "wrist2": self.pick_up_pos[4],
                "gripper": 130}
        
        try:
            # Write data to the CSV file
            self.csv_writer.writerow(data)
            
            # Flush the file to ensure data is written immediately
            self.csv_file.flush()
            
            # self.get_logger().info(f"Data written to CSV: {data}")
        except IOError as e:
            self.get_logger().error(f"Error writing to CSV: {e}")

        time.sleep(5)
        
        data = {"base": self.base_pickup_angle,
                # "shoulder": max(0, self.shoulder_pickup_angle - 10),
                "shoulder": 0,
                "elbow": self.elbow_pickup_angle - 60,
                "wrist1": self.pick_up_pos[3],
                "wrist2": self.pick_up_pos[4],
                "gripper": 130}
        
        try:
            # Write data to the CSV file
            self.csv_writer.writerow(data)
            
            # Flush the file to ensure data is written immediately
            self.csv_file.flush()
            
            # self.get_logger().info(f"Data written to CSV: {data}")
        except IOError as e:
            self.get_logger().error(f"Error writing to CSV: {e}")
        
        time.sleep(5)

        data = {"base": self.base_pickup_angle,
                # "shoulder": max(0, self.shoulder_pickup_angle - 10),
                "shoulder": 0,
                "elbow": self.elbow_pickup_angle - 60,
                "wrist1": self.pick_up_pos[3],
                "wrist2": self.pick_up_pos[4],
                "gripper": 50}
        
        try:
            # Write data to the CSV file
            self.csv_writer.writerow(data)
            
            # Flush the file to ensure data is written immediately
            self.csv_file.flush()
            
            # self.get_logger().info(f"Data written to CSV: {data}")
        except IOError as e:
            self.get_logger().error(f"Error writing to CSV: {e}")
        
        time.sleep(5)

        data = {"base": self.default_pos[0],
                "shoulder": self.default_pos[1],
                "elbow": self.default_pos[2],
                "wrist1": self.default_pos[3],
                "wrist2": self.default_pos[4],
                "gripper": self.default_pos[5]}
        
        try:
            # Write data to the CSV file
            self.csv_writer.writerow(data)
            
            # Flush the file to ensure data is written immediately
            self.csv_file.flush()
            
            # self.get_logger().info(f"Data written to CSV: {data}")
        except IOError as e:
            self.get_logger().error(f"Error writing to CSV: {e}")

        self.mode = "-1"
        

    
def main(args=None):
    rclpy.init(args=args)

    # Initialize and run the driving node
    arm_node = ArmNode()

    try:
        rclpy.spin(arm_node)  # Keep the node running
    finally:
        arm_node.destroy_node()  # Ensure node and resources are properly destroyed
        rclpy.shutdown()  # Shutdown rclpy

if __name__ == '__main__':
    main()