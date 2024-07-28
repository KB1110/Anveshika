import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Float32

from adafruit_servokit import ServoKit
from simple_pid import PID

kit = ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(500, 2500)
kit.servo[1].set_pulse_width_range(500, 2500)
kit.servo[2].set_pulse_width_range(500, 2500)
kit.servo[3].set_pulse_width_range(500, 2500)

kit.servo[4].set_pulse_width_range(1000, 2000)
kit.servo[5].set_pulse_width_range(1000, 2000)
kit.servo[6].set_pulse_width_range(1000, 2000)
kit.servo[7].set_pulse_width_range(1000, 2000)

kit.servo[0].angle = 90
kit.servo[1].angle = 90
kit.servo[2].angle = 90
kit.servo[3].angle = 90

kit.servo[4].angle = 90
kit.servo[5].angle = 90
kit.servo[6].angle = 90
kit.servo[7].angle = 90

class DrivingNode(Node):

    def __init__(self):
        super().__init__('driving_node')

        self.dist_subscriber = self.create_subscription(Float32, '/dist', self.dist_callback, qos_profile_system_default)
        self.steer_subscriber = self.create_subscription(Float32, '/steer', self.steer_callback, qos_profile_system_default)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.dist = 0
        self.steer_ang = 90

        self.pid = PID(1, 0.1, 0.05, setpoint=0)

        self.kp = 1
        self.kd = 0.1
        self.ki = 0.05
    
    def dist_callback(self, msg):
        self.dist = msg.data
    
    def steer_callback(self, msg):
        self.steer_ang = msg.data

    def timer_callback(self):
        if self.dist != 0 and self.steer_ang != 90:
            control = self.pid(self.dist)

            control = max(-90, min(90, control))

            kit.servo[4].angle = 90 + control
            kit.servo[5].angle = 90 + control
            kit.servo[6].angle = 90 + control
            kit.servo[7].angle = 90 + control

            steer_diff = 90 - self.steer_ang

            kit.servo[0].angle = 90 + steer_diff
            kit.servo[1].angle = 90 + steer_diff
            kit.servo[2].angle = 90 - steer_diff
            kit.servo[3].angle = 90 - steer_diff

        else:
            kit.servo[0].angle = 90
            kit.servo[1].angle = 90
            kit.servo[2].angle = 90
            kit.servo[3].angle = 90

            kit.servo[4].angle = 90
            kit.servo[5].angle = 90
            kit.servo[6].angle = 90
            kit.servo[7].angle = 90
    



def main(args=None):
    rclpy.init(args=args)

    driving_node = DrivingNode()

    rclpy.spin(driving_node)

    driving_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()