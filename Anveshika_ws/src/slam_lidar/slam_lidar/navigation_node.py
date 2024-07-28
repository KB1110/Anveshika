import rclpy
import tf_transformations
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32

import heapq

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from slam_lidar.grid_mapping import OccupancyGridMap
from slam_lidar.a_star import a_star
from slam_lidar.a_star import inflate_obstacles
from slam_lidar.path_tracking import *

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile_system_default)
        self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.tf_callback, qos_profile_system_default)

        self.distance_publisher = self.create_publisher(Float32, '/dist', qos_profile_system_default)
        self.steer_publisher = self.create_publisher(Float32, '/steer', qos_profile_system_default)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.map = None
        self.tf = None

        self.position = [0, 0, 0] #x, y, yaw
        self.base_link = [0, 0, 0]

        self.waypoint = [2.0, -0.5]

        # self.waypoint = [1.0, 0.0]

        self.fig, self.ax = plt.subplots()
        self.map_image = None
        self.robot_marker, = self.ax.plot([], [], 'go', markersize=8, label='Container')  # Robot position marker
        self.waypoint_marker, = self.ax.plot([], [], 'rx', markersize=8, label='Waypoint')  # Waypoint marker
        self.base_marker, = self.ax.plot([], [], 'ro', markersize=8, label='Base')

        self.path, = self.ax.plot([], [], 'r-', markersize=2, label='Path')

        self.robot_arrow = None
        self.base_arrow = None

        self.ax.set_title("Occupancy Grid Map with Robot Position")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")

        self.prev_map_length = None

        self.path_line = None

        self.steer_angle = 90
        self.wheel_speed = 0

        self.robot_radius = 0.3

    def map_callback(self, msg):
        self.map = msg

    def tf_callback(self, msg):
        self.tf = msg

    def timer_callback(self):
        if self.map and self.tf:

            quaternion = (self.tf.transforms[1].transform.rotation.x, 
                    self.tf.transforms[1].transform.rotation.y,
                    self.tf.transforms[1].transform.rotation.z,
                    self.tf.transforms[1].transform.rotation.w)
        
            euler_angles = tf_transformations.euler_from_quaternion(quaternion)

            self.sx = self.position[0] = self.tf.transforms[1].transform.translation.x
            self.sy = self.position[1] = self.tf.transforms[1].transform.translation.y
            self.position[2] = euler_angles[2]

            quat_base = (self.tf.transforms[0].transform.rotation.x, 
                    self.tf.transforms[0].transform.rotation.y,
                    self.tf.transforms[0].transform.rotation.z,
                    self.tf.transforms[0].transform.rotation.w)
            
            euler_angles_base = tf_transformations.euler_from_quaternion(quat_base)

            self.base_link[0] = self.tf.transforms[0].transform.translation.x
            self.base_link[1] = self.tf.transforms[0].transform.translation.y
            self.base_link[2] = euler_angles_base[2]

            # print(self.position)
            self.update_plot(None)
    
    def update_plot(self, frame):

        if self.map:

            map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))

            map_resolution = self.map.info.resolution

            # Transform the occupancy grid data for visualization
            # Convert from [0, 100] to [0, 1], with -1 as unknown
            map_data = np.where(map_data == -1, 0.5, map_data / 100.0)

            # #OccupancyGridMap
            # og_map = OccupancyGridMap(map_data, map_resolution)

            inflated_map = inflate_obstacles(map_data, self.robot_radius, self.map.info.resolution)
            og_map_inflated = OccupancyGridMap(inflated_map, self.map.info.resolution)

            # print(og_map.data.shape, map_data.shape)

            # Display the map
            if self.map_image is None or len(self.map.data) != self.prev_map_length:
                self.map_image = self.ax.imshow(
                    map_data, cmap='gray', origin='lower')
                
                # self.map_image = self.ax.imshow(
                #     og_map.get_data, cmap='gray', origin='lower')
                
            else:
                self.map_image.set_data(map_data)

            self.ax.set_xlim(0, self.map.info.width)
            self.ax.set_ylim(0, self.map.info.height)

            x_ticks = self.ax.get_xticks()
            y_ticks = self.ax.get_yticks()
            
            self.ax.set_xticklabels([f"{tick * map_resolution:.2f}" for tick in x_ticks])
            self.ax.set_yticklabels([f"{tick * map_resolution:.2f}" for tick in y_ticks])

            map_origin_x = self.map.info.origin.position.x
            map_origin_y = self.map.info.origin.position.y

            # Convert world coordinates to map indices
            robot_x = (self.position[0] - map_origin_x) / map_resolution
            robot_y = (self.position[1] - map_origin_y) / map_resolution

            # Update robot marker position
            self.robot_marker.set_data([robot_x], [robot_y])

            if self.robot_arrow:
                self.robot_arrow.remove()

            # Draw an arrow for robot orientation
            robot_angle = self.position[2]  # Orientation in radians
            arrow_length = -0.5 / map_resolution  # Length of the arrow in map units
            arrow_dx = arrow_length * np.cos(robot_angle)
            arrow_dy = arrow_length * np.sin(robot_angle)

            self.robot_arrow = patches.FancyArrow(
                robot_x, robot_y, arrow_dx, arrow_dy, width=0.1, color="blue"
            )
            self.ax.add_patch(self.robot_arrow)

            # Convert world coordinates to map indices
            base_x = (self.base_link[0] - map_origin_x) / map_resolution
            base_y = (self.base_link[1] - map_origin_y) / map_resolution

            # Update robot marker position
            self.base_marker.set_data([base_x], [base_y])

            if self.base_arrow:
                self.base_arrow.remove()

            # Draw an arrow for robot orientation
            base_angle = self.base_link[2]  # Orientation in radians
            arrow_length = -0.5 / map_resolution  # Length of the arrow in map units
            base_arrow_dx = arrow_length * np.cos(base_angle)
            base_arrow_dy = arrow_length * np.sin(base_angle)

            self.base_arrow = patches.FancyArrow(
                base_x, base_y, base_arrow_dx, base_arrow_dy, width=0.1, color="blue"
            )
            self.ax.add_patch(self.base_arrow)

            # Convert waypoint to map indices and plot it
            waypoint_x = -1 * (self.waypoint[0] * np.cos(base_angle) - self.waypoint[1] * np.sin(base_angle)) + self.base_link[0]
            waypoint_y = (self.waypoint[0] * np.sin(base_angle) - self.waypoint[1] * np.cos(base_angle)) + self.base_link[1]

            # Convert the waypoint position from world coordinates to map indices
            waypoint_base_x = (waypoint_x - map_origin_x) / map_resolution
            waypoint_base_y = (waypoint_y - map_origin_y) / map_resolution

            # print(f"Start: ({robot_x}, {robot_y}), Goal: ({waypoint_base_x}, {waypoint_base_y})")

            self.waypoint_marker.set_data([waypoint_base_x], [waypoint_base_y])

            if self.path_line:
                self.path_line.remove()  # Remove existing line

            self.path_line = plt.Line2D(
                [robot_x, waypoint_base_x], [robot_y, waypoint_base_y], color='cyan', linestyle='--', linewidth=1
            )
            self.ax.add_line(self.path_line)

            start_m = (robot_x * map_resolution, robot_y * map_resolution)
            goal_m = (waypoint_base_x * map_resolution, waypoint_base_y * map_resolution)

            # start_m = (robot_y * map_resolution, robot_x * map_resolution)
            # goal_m = (waypoint_base_y * map_resolution, waypoint_base_x * map_resolution)

            # path = a_star(start_m, goal_m, og_map, movement='8N', occupancy_cost_factor=3)

            # print(path)
            
            try:
                path = a_star(start_m, goal_m, og_map_inflated, movement='8N', occupancy_cost_factor=4)

                # path = a_star(start_m, goal_m, og_map, movement='8N', occupancy_cost_factor=4)

                # print(f"The path is {path}")

                # print(len(path))

                path_map_x = [points[0] for points in path[1]]
                path_map_y = [points[1] for points in path[1]]

                self.path.set_data([path_map_x], [path_map_y])

                path_x = [points[0] for points in path[0]]
                path_y = [points[1] for points in path[0]]

                traj = Trajectory(path_x, path_y)

                PI_yaw = PI()

                distance = getDistance(start_m, goal_m)

                if distance > 1:
                    target_point = traj.getTargetPoint(start_m)

                    yaw_err = math.atan2(target_point[1] - start_m[1], target_point[0] - start_m[1]) - (robot_angle + math.pi/2)
                    delta = PI_yaw.control(yaw_err)

                    # move the vehicle
                    self.steer_angle, _ = inverse_kinematics_zero_sideslip(0.16, delta)

                    dist_msg = Float32()
                    dist_msg.data = distance
            # og_map = OccupancyGridMap(map_data, map_resolution)

                    steer_msg = Float32()
                    steer_msg.data = self.steer_angle

                    # print(self.steer_angle)
                
                else:
                    self.steer_angle = 90
                    self.distance = 0
                
                self.distance_publisher.publish(dist_msg)
                self.steer_publisher.publish(steer_msg)
                
            except Exception as e:
                print(e)

            # try:
            #     path, path_idx = a_star(start_m, goal_m, og_map, movement='8N', occupancy_cost_factor=3)
            #     print(f"Path found: {path}")
            # except Exception as e:
            #     print(f"Error finding path: {e}")
            #     path, path_idx = None, None

            # Redraw the plot
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            plt.show(block=False)

            self.prev_map_length = len(self.map.data)
    

def main(args=None):
    rclpy.init(args=args)

    navigation_node = NavigationNode()

    rclpy.spin(navigation_node)

    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()