import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

fig, ax = plt.subplots(1, 1)
fig.set_size_inches(5,5)

h = 0.1
v = 30
Ws = 0.1
L = 2.46
t = 5
points = []


class car_simulation:
    def _init_(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.phi = 0

    def coord_computation(self):
        self.x = self.x + h * math.cos(self.theta) * v
        self.y = self.y + h * math.sin(self.theta) * v
        self.theta = self.theta + (h * math.tan(self.phi) * v)/L
        if self.phi > math.pi/4:
            self.phi = math.pi/4
        else:
            self.phi = self.phi + h * Ws
        return self.x, self.y, self.theta, self.phi


def animate(i):
    #ax.clear()
    # Plot that point using the x and y coordinates
    point = points[i]
    ax.plot(point[0], point[1], color='green',
            label='original', marker='o')
    # Set the x and y axis to display a fixed range
    ax.set_xlim([-int(t/h), int(t/h)])
    ax.set_ylim([-int(t/h), int(t/h)])


def main():
    print(t/h)
    car = car_simulation()

    for i in range(int(t/h)):
        x, y, theta, phi = car.coord_computation()
        print("x, y , theta, phi", x, y, theta, phi)
        point = [x, y]
        points.append(point)

    ani = FuncAnimation(fig, animate, frames=int(t/h),
                            interval=500, repeat=False)

    plt.show()


main()