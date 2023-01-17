import numpy as np
from math import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation




class Robot:
    def __init__(self, h):
        self.x = 0 #Meters
        self.y = 3 #Meters
        self.theta = 0 #Rad
        self.phi = 0  #Rad
        self.v = 5 #Meters/s
        self.L = 2.46 # Meters
        self.omega = 0
        self.omega_s = 0
        # Derivatives are indicated with an extra "_"

        self.theta_ = 0
        self.phi_ = 0
        # Reference points

        self.x_ref = 40
        self.y_ref = 40
        self.theta_ref = 0
        # Control variables

        self.b_e = 0
        self.b_e_ = 0


        self.K_v = 3
        self.K_s = 100
        self.K_l = 20

        self.K_v_ = 1
        self.K_s_ = 1
        self.K_l_ = 1

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

    def set_new_param(self):
        self.v = self.K_v * self.b_e[0]
        self.phi_ = self.K_s * self.b_e[2] + self.K_l*self.b_e[1] # - self.K_l_ * self.b_e_[1]

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
