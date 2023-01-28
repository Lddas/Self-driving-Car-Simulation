import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import robot
import cv2
import path_planning
import numpy as np
from math import cos,sin


fig, ax = plt.subplots(1, 1)
fig.set_size_inches(5,5)

h = 0.01      #seconds/iteration
t = 30     #seconds
points = []
road = path_planning.path_planning(17, 14)
ref_traj = np.array(road.final_trajectory)
np.save("roadby4", ref_traj)
#ref_traj = np.load("roadby4new.npy", allow_pickle = True)
robot = robot.Robot(h,ref_traj[0][0],ref_traj[0][1])
"""ref_traj_1 = np.stack((np.arange(0, 50, 1), np.ones((50)) * 10), axis = -1)
ref_traj_2 = np.stack((np.ones((10)) * 50, np.arange(10, 20, 1)), axis = -1)
ref_traj_3 = np.stack((np.arange(50, 70, 1), np.ones((20)) * 20), axis = -1)
ref_traj = np.concatenate((ref_traj_1, ref_traj_2,ref_traj_3))
print(t)"""
img = plt.imread("googlemapp.png")
shape = img.shape
img = cv2.resize(img, (int(shape[1]/4), int(shape[0]/4)))
shape = img.shape

def main_loop(robot):
    for i in range(int(t/h)):
        robot.kinematics(h)
        robot.find_initial_ref_point(ref_traj)
        robot.control_param(h)
        robot.set_new_param(ref_traj)
        points.append((robot.x,robot.y,robot.theta, robot.x_ref,robot.y_ref))


def animate(i):
    ax.clear()
    ax.imshow(img)
    point = points[i]
    ax.plot(point[0], point[1], color='green',label='original', marker='o')
    #ax.plot(ref_traj[:,0],ref_traj[:,1], color='blue',label='original', marker='o')
    #ax.plot(point[3], point[4], color='red',label='original', marker='o')
    ax.plot(ref_traj[-1,0], ref_traj[-1,1], color='red',label='original', marker='o')
    ax.arrow(point[0]- 2 * robot.L * cos(point[2]), point[1] - 2 * robot.L * sin(point[2]),
              4*robot.L * cos(point[2]), 4*robot.L * sin(point[2]))

    ax.set_xlim([0, shape[1]])
    ax.set_ylim([0, shape[0]])

main_loop(robot)
anim = FuncAnimation(fig, animate,frames=int(t/h), interval=int(h * 1000), repeat=False)
plt.show()
