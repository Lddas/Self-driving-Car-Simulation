import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import main
from matplotlib.animation import PillowWriter
import numpy as np
from math import *


fig, ax = plt.subplots(1, 1)
fig.set_size_inches(5,5)

h = 0.01      #seconds/iteration
t = 15     #seconds
points = []
robot = main.Robot(h)
ref_traj = np.stack((np.arange(0, 40, 0.05), np.ones((800)) * 10 ), axis = -1)

def main_loop(robot):
    for i in range(int(t/h)):
        robot.kinematics(h)
        robot.find_ref_point(ref_traj)
        robot.control_param(h)
        robot.set_new_param()
        points.append((robot.x,robot.y,robot.theta))

def animate(i):
    ax.clear()
    point = points[i]
    ax.plot(point[0], point[1], color='green',label='original', marker='o')
    ax.plot(ref_traj, color='blue',label='original', marker='o')
    ax.plot(robot.x_ref, robot.y_ref, color='red',label='original', marker='o')
    ax.arrow(point[0]- 0.5 * robot.L * cos(point[2]), point[1] - 0.5 * robot.L * sin(point[2]),
              robot.L * cos(point[2]), robot.L * sin(point[2]))

    ax.set_xlim([0, 50])
    ax.set_ylim([0, 50])

main_loop(robot)
anim = FuncAnimation(fig, animate,frames=int(t/h), interval=int(h * 1000), repeat=False)
# Save the animation as an animated GIF
#anim.save("simple_animation.gif", dpi = 300, writer=PillowWriter(fps=int(1/h)))
plt.show()