import math
import matplotlib.pyplot as plt
import numpy as np

L = 0.5

dt = 0.01

def getDistance(p1, p2):
    """
    Calculate distance
    :param p1: list, point1
    :param p2: list, point2
    :return: float, distance
    """
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

class Trajectory:
    def __init__(self, traj_x, traj_y):
        """
        Define a trajectory class
        :param traj_x: list, list of x position
        :param traj_y: list, list of y position
        """
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0

    def getPoint(self, idx):
        return [self.traj_x[idx], self.traj_y[idx]]

    def getTargetPoint(self, pos):
        """
        Get the next look ahead point
        :param pos: list, vehicle position
        :return: list, target point
        """
        target_idx = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist = getDistance(pos, target_point)

        while curr_dist < L and target_idx < len(self.traj_x) - 1:
            target_idx += 1
            target_point = self.getPoint(target_idx)
            curr_dist = getDistance(pos, target_point)

        self.last_idx = target_idx
        return self.getPoint(target_idx)


class PI:
    def __init__(self, kp=1.0, ki=0.0):
        """
        Define a PID controller class
        :param kp: float, kp coeff
        :param ki: float, ki coeff
        :param kd: float, kd coeff
        """
        self.kp = kp
        self.ki = ki
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.last_error = 0.0

    def control(self, error):
        """
        PID main function, given an input, this function will output a control unit
        :param error: float, error term
        :return: float, output control
        """
        self.Pterm = self.kp * error
        # self.Iterm += error * dt

        self.last_error = error
        output = self.Pterm #+ self.ki * self.Iterm
        return output

def inverse_kinematics_zero_sideslip(fwd_vel, ang_vel):
    wheel_rad = 0.05
    dist_to_f_wheels = 0.25
    dist_to_r_wheels = 0.25
    speed_constant = 5

    # print(ang_vel)

    steer_ang = math.atan(ang_vel * (dist_to_f_wheels + dist_to_r_wheels) / (fwd_vel))

    wheel_speed = fwd_vel / (wheel_rad * math.cos(steer_ang) * speed_constant)

    steer_ang = math.degrees(steer_ang)

    steer_ang += 90
    
    return steer_ang, wheel_speed

def main():

    # target velocity
    target_vel = 10

    # target course
    traj_x = np.arange(0, 100, 0.5)
    traj_y = [math.sin(x / 10.0) * x / 2.0 for x in traj_x]
    traj = Trajectory(traj_x, traj_y)
    goal = traj.getPoint(len(traj_x) - 1)

    # create PI controller
    PI_acc = PI()
    PI_yaw = PI()

    # real trajectory
    traj_ego_x = []
    traj_ego_y = []

    plt.figure(figsize=(12, 8))
    while getDistance([ego.x, ego.y], goal) > 1:
        target_point = traj.getTargetPoint([ego.x, ego.y])

        # use PID to control the vehicle
        vel_err = target_vel - ego.vel
        acc = PI_acc.control(vel_err)

        yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw
        delta = PI_yaw.control(yaw_err)

        # move the vehicle
        ego.update(acc, delta)

        # store the trajectory
        traj_ego_x.append(ego.x)
        traj_ego_y.append(ego.y)

        # plots
        plt.cla()
        plt.plot(traj_x, traj_y, "-r", linewidth=5, label="course")
        plt.plot(traj_ego_x, traj_ego_y, "-b", linewidth=2, label="trajectory")
        plt.plot(target_point[0], target_point[1], "og", ms=5, label="target point")
        plotVehicle(ego.x, ego.y, ego.yaw, delta)
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.legend()
        plt.grid(True)
        plt.pause(0.1)


if __name__ == "__main__":
    main()