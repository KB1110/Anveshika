import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import csv

from std_msgs.msg import Float32
from std_msgs.msg import String

from simple_pid import PID

class DrivingNode(Node):

    def __init__(self):
        super().__init__('driving_node')

        self.dist_subscriber = self.create_subscription(Float32, '/dist', self.dist_callback, qos_profile_system_default)
        self.steer_subscriber = self.create_subscription(Float32, '/steer', self.steer_callback, qos_profile_system_default)

        self.mode_publisher = self.create_publisher(String, '/mode_in', qos_profile_system_default)
        self.mode_subscriber = self.create_subscription(String, '/mode_out', self.mode_callback, qos_profile_system_default)

        self.mode = "1"

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.dist = 0
        self.steer_ang = 90

        self.kp = 3
        self.kd = 0.0
        self.ki = 0.00

        self.pid = PID(self.kp, self.kd, self.ki, setpoint=0)

        # Open CSV file for appending and create a CSV writer object
        self.csv_file = open('/home/pi/Anveshika/driving_data.csv', mode='a', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=["fl_steer",
                                                                    "fr_steer",
                                                                    "rl_steer",
                                                                    "rr_steer",
                                                                    "fl_drive",
                                                                    "fr_drive",
                                                                    "rl_drive",
                                                                    "rr_drive"])

        # Write header only if file is new
        if self.csv_file.tell() == 0:
            self.csv_writer.writeheader()
    
    def dist_callback(self, msg):
        self.dist = msg.data
    
    def steer_callback(self, msg):
        self.steer_ang = msg.data

    def mode_callback(self, msg):
        self.mode = msg.data

    def timer_callback(self):

        print(self.mode)
        
        # Rover will turn to face the waypoint coordinate or Search for sample or container
        if self.mode == "0" or self.mode == "2" or self.mode == "5":
            if self.dist != 0 and self.steer_ang != 0:
                control = self.pid(self.dist)

                control *= -40

                control = max(-90, min(90, control))

                # steer_diff = 90 - self.steer_ang
                # steer_diff = max(-45, min(45, steer_diff))

                self.steer_ang = max(-45, min(45, self.steer_ang))

                print(control, self.steer_ang)

                data = {"fl_steer": 90 + self.steer_ang,
                        "fr_steer": 90 - self.steer_ang,
                        "rl_steer": 90 - self.steer_ang,
                        "rr_steer": 90 + self.steer_ang,
                        "fl_drive": 90 + control,
                        "fr_drive": 90 - control,
                        "rl_drive": 90 + control,
                        "rr_drive": 90 - control,
                }
            else:
                data = {"fl_steer": 90,
                        "fr_steer": 90,
                        "rl_steer": 90,
                        "rr_steer": 90,
                        "fl_drive": 90,
                        "fr_drive": 90,
                        "rl_drive": 90,
                        "rr_drive": 90,
                }

        # Autonomous navigation to coordinate
        elif self.mode == "1":

            if self.dist != 0 and self.steer_ang != 0:
                control = self.pid(self.dist)

                control *= -40

                control = max(-90, min(90, control))

                # steer_diff = 90 - self.steer_ang
                # steer_diff = max(-45, min(45, steer_diff))

                self.steer_ang = max(-45, min(45, self.steer_ang))

                print(control, self.steer_ang)

                data = {"fl_steer": 90 + self.steer_ang,
                        "fr_steer": 90 + self.steer_ang,
                        "rl_steer": 90 - self.steer_ang,
                        "rr_steer": 90 - self.steer_ang,
                        "fl_drive": 90 + control,
                        "fr_drive": 90 + control,
                        "rl_drive": 90 + control,
                        "rr_drive": 90 + control,
                }
            else:
                data = {"fl_steer": 90,
                        "fr_steer": 90,
                        "rl_steer": 90,
                        "rr_steer": 90,
                        "fl_drive": 90,
                        "fr_drive": 90,
                        "rl_drive": 90,
                        "rr_drive": 90,
                }
        
        elif self.mode == "3" or self.mode == "6":
            if self.dist != 0 and self.steer_ang != 0:

                control = 20
                
                data = {"fl_steer": 90 + self.steer_ang,
                        "fr_steer": 90 + self.steer_ang,
                        "rl_steer": 90 + self.steer_ang,
                        "rr_steer": 90 + self.steer_ang,
                        "fl_drive": 90 + control,
                        "fr_drive": 90 + control,
                        "rl_drive": 90 + control,
                        "rr_drive": 90 + control,
                }
            else:
                data = {"fl_steer": 90,
                        "fr_steer": 90,
                        "rl_steer": 90,
                        "rr_steer": 90,
                        "fl_drive": 90,
                        "fr_drive": 90,
                        "rl_drive": 90,
                        "rr_drive": 90,
                }

        # STOP AT EXIT or Sample Pick or Sample Drop
        elif self.mode == "-1" or self.mode == "4" or self.mode == "7":
            data = {"fl_steer": 90,
                    "fr_steer": 90,
                    "rl_steer": 90,
                    "rr_steer": 90,
                    "fl_drive": 90,
                    "fr_drive": 90,
                    "rl_drive": 90,
                    "rr_drive": 90,
                }
        
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
    
def main(args=None):
    rclpy.init(args=args)

    # Initialize and run the driving node
    driving_node = DrivingNode()

    try:
        rclpy.spin(driving_node)  # Keep the node running
    finally:
        driving_node.destroy_node()  # Ensure node and resources are properly destroyed
        rclpy.shutdown()  # Shutdown rclpy

if __name__ == '__main__':
    main()