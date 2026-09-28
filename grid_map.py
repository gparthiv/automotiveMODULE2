import numpy as np
from scipy.ndimage import binary_dilation
import config


class DynamicObstacle:
    """A circle that moves back and forth inside a box."""

    def __init__(self, x, y, vx, vy, bounds):
        self.x, self.y = x, y
        self.vx, self.vy = vx, vy
        self.xmin, self.xmax, self.ymin, self.ymax = bounds
        self.radius = config.OBSTACLE_RADIUS

    def step(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        if self.x < self.xmin or self.x > self.xmax:
            self.vx = -self.vx
            self.x = min(max(self.x, self.xmin), self.xmax)
        if self.y < self.ymin or self.y > self.ymax:
            self.vy = -self.vy
            self.y = min(max(self.y, self.ymin), self.ymax)


class GridMap:
    """True = blocked cell, False = free cell. Indexed as grid[iy, ix]."""

    def __init__(self):
        n = config.GRID_SIZE
        g = np.zeros((n, n), dtype=bool)
        g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = True   # outer walls
        g[0:26, 14] = True                                # wall A (gap at the top)
        g[14:n, 26] = True                                # wall B (gap at the bottom)
        self.static = g
        # inflate walls by 1 cell so the car doesn't scrape them
        self.inflated = binary_dilation(g, iterations=1)

        self.obstacles = [
            DynamicObstacle(9.0, 5.0, 0.0, 1.0, (9.0, 9.0, 3.0, 17.0)),
            DynamicObstacle(2.0, 16.0, 1.2, 0.0, (1.5, 6.0, 16.0, 16.0)),
            DynamicObstacle(16.0, 10.0, 0.0, 0.8, (16.0, 16.0, 8.0, 18.0)),
        ]

    def step(self, dt):
        for o in self.obstacles:
            o.step(dt)

    def grid_with_obstacles(self):
        """Copy of the inflated map with moving obstacles painted in (for replanning)."""
        g = self.inflated.copy()
        res = config.RESOLUTION
        for o in self.obstacles:
            r = config.SAFETY_RADIUS
            for iy in range(int((o.y - r) / res), int((o.y + r) / res) + 2):
                for ix in range(int((o.x - r) / res), int((o.x + r) / res) + 2):
                    if 0 <= ix < g.shape[1] and 0 <= iy < g.shape[0]:
                        cx, cy = (ix + 0.5) * res, (iy + 0.5) * res
                        if (cx - o.x) ** 2 + (cy - o.y) ** 2 <= r ** 2:
                            g[iy, ix] = True
        return g
