import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import main
import code_leo
from matplotlib.animation import PillowWriter
import numpy as np
from math import *


fig, ax = plt.subplots(1, 1)
fig.set_size_inches(5,5)

h = 0.01      #seconds/iteration
t = 15     #seconds
points = []
robot = main.Robot(h)
route = code_leo.path_planning(2,5)
ref_traj = np.array(route.twoD_traj)
"""ref_traj_1 = np.stack((np.arange(0, 50, 1), np.ones((50)) * 10), axis = -1)
ref_traj_2 = np.stack((np.ones((10)) * 50, np.arange(10, 20, 1)), axis = -1)
ref_traj_3 = np.stack((np.arange(50, 70, 1), np.ones((20)) * 20), axis = -1)
ref_traj = np.concatenate((ref_traj_1, ref_traj_2,ref_traj_3))
print(t)
"""
def main_loop(robot):
    for i in range(int(t/h)):
        robot.kinematics(h)
        robot.find_initial_ref_point(ref_traj)
        robot.control_param(h)
        robot.set_new_param()
        points.append((robot.x,robot.y,robot.theta, robot.x_ref,robot.y_ref))

def animate(i):
    ax.clear()
    point = points[i]
    ax.plot(point[0], point[1], color='green',label='original', marker='o')
    ax.plot(ref_traj[:,0],ref_traj[:,1], color='blue',label='original', marker='o')
    ax.plot(point[3], point[4], color='red',label='original', marker='o')
    ax.arrow(point[0]- 0.5 * robot.L * cos(point[2]), point[1] - 0.5 * robot.L * sin(point[2]),
              robot.L * cos(point[2]), robot.L * sin(point[2]))

    ax.set_xlim([0, 70])
    ax.set_ylim([0, 100])

main_loop(robot)
anim = FuncAnimation(fig, animate,frames=int(t/h), interval=int(h * 1000), repeat=False)
# Save the animation as an animated GIF
#anim.save("simple_animation.gif", dpi = 300, writer=PillowWriter(fps=int(1/h)))
plt.show()