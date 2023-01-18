import numpy as np
from math import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation




class Robot:
    def __init__(self, h):
        self.x = 0 #Meters
        self.y = 10 #Meters
        self.theta = pi/4 #Rad
        self.phi = 0  #Rad
        self.v = 20 #Meters/s
        self.L = 2.46 # Meters
        self.omega = 0
        self.omega_s = 0
        # Derivatives are indicated with an extra "_"

        self.theta_ = 0
        self.phi_ = 0
        # Reference points

        self.x_ref = 0
        self.y_ref = 10
        self.theta_ref = 0
        self.ref_point_counter = 0
        self.dist_from_ref_point = 0l
        # Control variables

        self.b_e = 0
        self.b_e_ = 0
        self.b_e_int = 0

        Ku = 1
        iter = 218 #iterations for a perdiod
        Tu = iter*h #period

        self.K_x = 0
        self.K_y = 0
        self.K_theta = 0

        self.K_x_ = 0
        self.K_y_ = 0
        self.K_theta_ = 0

        self.K_x_int = 0
        self.K_y_int = 0
        self.K_theta_int = 0

    def kinematics(self, h):
        self.x = self.x + h * cos(self.theta) * self.v
        self.y = self.y + h * sin(self.theta) * self.v
        self.theta = self.theta + (h * tan(self.phi) * self.v) / self.L
        self.phi = self.phi + self.phi_ * h
        if abs(self.phi) > pi/8:
            self.phi = sign(self.phi)* pi/8

    def find_ref_point(self, ref_point_traj):
        distance_list = np.empty(ref_point_traj.shape[0])
        for i, point in enumerate(ref_point_traj):
            distance_list[i] = dist((self.x, self.y), point)
        min_point = np.argmin(distance_list)
        self.x_ref = ref_point_traj[min_point][0]
        self.y_ref = ref_point_traj[min_point][1]

        """if distance_list[min_point] < 2 :
            min_point += 3
            min_point = min(min_point, len(distance_list)-1)
            self.x_ref = ref_point_traj[min_point][0]
            self.y_ref = ref_point_traj[min_point][1]"""

        """derivs = np.array([self.x_, self.y_, self.theta_, self.phi_])
        A = np.array([cos(self.theta),      0],
                     [sin(self.theta),      0],
                     [tan(self.phi/self.L), 0],
                     [0,                    1])
        phys_param = np.array([self.v, self.omega_s])

        derivs = A.dot(phys_param)

        np.array([self.x, self.y, self.theta, self.phi]) ="""
        return

    def control_param(self, h):
        w_e = np.array(([self.x_ref - self.x, self.y_ref-self.y, self.theta_ref-self.theta]))
        A = np.array(([cos(self.theta),  sin(self.theta_),  0],
                       [-sin(self.theta), cos(self.theta),   0],
                       [0,                       0,          1]))
        past_be = self.b_e
        self.b_e = np.dot(A,w_e)
        self.b_e_ = (self.b_e - past_be)/h
        self.b_e_int += self.b_e

    def set_new_param(self):
        self.v = self.K_x * self.b_e[0] + self.K_y * self.b_e[1]
        self.phi_ = self.K_theta * self.b_e[2] + self.K_y_int * self.b_e_int[1] + self.K_x_int * self.b_e_int[0] + self.K_theta_ * self.b_e_[2]


def sign(x):
    if x > 0:
        return 1
    elif x < 0 :
        return -1
    else:
        return 0


"""#Main loop:
def main_loop(robot):
    robot.kinematics()
    robot.control_param()
    robot.set_new_param()
    return robot.x, robot.y

"""

"""while True:
    robot.kinematics()
    robot.control_param()
    robot.set_new_param()
    point = [robot.x, robot.y]
    points.append(point)
    render.animate(i,h,t)
    i += 1"""
