import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np

import slam_lidar.lidar_to_grid_map as lg
import matplotlib.pyplot as plt

class MappingNode(Node):

    def __init__(self):
        super().__init__('mapping_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, qos_profile_sensor_data)
        # self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

    def listener_callback(self, msg):
        
        angleIncrement = msg.angle_increment

        # print(msg.ranges)
        ang = []
        
        dist = []

        # self.get_logger().info(f"{msg.angle_increment}")
        currAngle = 0
        for i in range(len(msg.ranges)):
            # print(f"Angle {i}: {msg.ranges[i]}")
            ang.append(currAngle)
            dist.append(msg.ranges[i])
            currAngle += angleIncrement

        xy_resolution = 0.02  # x-y grid resolution
        ox = np.sin(ang) * dist
        oy = np.cos(ang) * dist
        try:
            plt.clf()
            occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = lg.generate_ray_casting_grid_map(ox, oy, xy_resolution, True)
            xy_res = np.array(occupancy_map).shape
            plt.figure(1, figsize=(10, 4))
            # plt.subplot(122)
            plt.imshow(occupancy_map, cmap="PiYG_r")
            plt.pause(0.01)
            
        except Exception as e:
            pass
        # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
        # plt.clim(-0.4, 1.4)
        # plt.gca().set_xticks(np.arange(-.5, xy_res[1], 1), minor=True)
        # plt.gca().set_yticks(np.arange(-.5, xy_res[0], 1), minor=True)
        # plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        # plt.colorbar()
        # plt.subplot(121)
        # plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
        # plt.axis("equal")
        # plt.plot(0.0, 0.0, "ob")
        # plt.gca().set_aspect("equal", "box")
        # bottom, top = plt.ylim()  # return the current y-lim
        # plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        # plt.grid(True)

def main(args=None):
    rclpy.init(args=args)

    mapping_node = MappingNode()

    rclpy.spin(mapping_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mapping_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()