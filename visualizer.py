import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import math
import config


class Visualizer:
    def __init__(self, world, goal):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        size = config.GRID_SIZE * config.RESOLUTION
        self.ax.imshow(world.static, cmap="gray_r", origin="lower", extent=[0, size, 0, size])
        self.ax.plot(*goal, "y*", markersize=15, markeredgecolor="k")

        self.path_line, = self.ax.plot([], [], "b-", lw=2)
        self.trail_line, = self.ax.plot([], [], "g:", lw=1)
        self.car = Polygon(np.zeros((3, 2)), closed=True, color="green")
        self.ax.add_patch(self.car)
        self.circles = []
        for o in world.obstacles:
            c = Circle((o.x, o.y), o.radius, color="red")
            self.ax.add_patch(c)
            self.circles.append(c)
        self.tx, self.ty = [], []

    def update(self, sim):
        v = sim.vehicle
        self.tx.append(v.x)
        self.ty.append(v.y)
        c, s = math.cos(v.theta), math.sin(v.theta)
        self.car.set_xy([(v.x + 0.6 * c, v.y + 0.6 * s),
                         (v.x - 0.4 * c - 0.3 * s, v.y - 0.4 * s + 0.3 * c),
                         (v.x - 0.4 * c + 0.3 * s, v.y - 0.4 * s - 0.3 * c)])
        px, py = zip(*sim.controller.path)
        self.path_line.set_data(px, py)
        self.trail_line.set_data(self.tx, self.ty)
        for circle, o in zip(self.circles, sim.world.obstacles):
            circle.center = (o.x, o.y)
        self.ax.set_title(f"t = {sim.t:.1f}s   status: {sim.status}")
        plt.pause(0.001)
