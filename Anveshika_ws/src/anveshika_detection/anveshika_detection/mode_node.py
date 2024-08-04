import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import String

'''
MODE 0: Rover will turn to face the waypoint coordinate - Done
MODE 1: Autonomous navigation to coordinate - Done
MODE 2: Search for Sample
MODE 3: Travel to Sample
MODE 4: Pick up Sample
MODE 5: Search for Sample Container
MODE 6: Travel to Container
MODE 7: Drop sample in container
MODE -1: STOP AT EXIT - Done

Order of operation ideally

0 - 1 - 2 - 3 - 4 - 0 - 1 - 5 - 6 - 7
0 - 1 - 2 - 3 - 4 - 0 - 1 - 5 - 6 - 7
0 - 1 - 2 - 3 - 4 - 0 - 1 - 5 - 6 - 7
0 - 1 - -1 
'''

class ModeNode(Node):

    def __init__(self):
        super().__init__('mode_node')
        self.mode_publisher = self.create_publisher(String, '/mode_out', qos_profile_system_default)
        self.mode_subscriber = self.create_subscription(String, '/mode_in', self.mode_callback, qos_profile_system_default)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mode = "4"

    def timer_callback(self):
        msg = String()
        msg.data = self.mode
        self.mode_publisher.publish(msg)
    
    def mode_callback(self, msg):
        self.mode = msg.data

def main(args=None):
    rclpy.init(args=args)

    mode_node = ModeNode()

    rclpy.spin(mode_node)

    mode_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()