import math
import config
from localization import wrap


class PurePursuit:
    """Chase a point on the path that's LOOKAHEAD meters ahead of the car."""

    def __init__(self):
        self.path = []
        self.idx = 0

    def set_path(self, path):
        self.path = path
        self.idx = 0

    def path_ahead(self):
        """Path points within CHECK_DISTANCE of where we are on the path."""
        pts, dist = [self.path[self.idx]], 0.0
        for a, b in zip(self.path[self.idx:-1], self.path[self.idx + 1:]):
            dist += math.hypot(b[0] - a[0], b[1] - a[1])
            if dist > config.CHECK_DISTANCE:
                break
            pts.append(b)
        return pts

    def command(self, pose):
        x, y, theta = pose
        # move idx forward to the closest point on the path
        while self.idx < len(self.path) - 1 and \
                math.hypot(self.path[self.idx + 1][0] - x, self.path[self.idx + 1][1] - y) < \
                math.hypot(self.path[self.idx][0] - x, self.path[self.idx][1] - y):
            self.idx += 1

        # pick the lookahead target
        target = self.path[-1]
        for p in self.path[self.idx:]:
            if math.hypot(p[0] - x, p[1] - y) >= config.LOOKAHEAD:
                target = p
                break

        alpha = wrap(math.atan2(target[1] - y, target[0] - x) - theta)
        dist_goal = math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)

        if abs(alpha) > 1.0:
            # target is way off to the side: turn on the spot first
            v = 0.0
            w = max(-config.MAX_OMEGA, min(config.MAX_OMEGA, 2.0 * alpha))
        else:
            v = config.MAX_SPEED * max(0.3, math.cos(alpha))
            v = min(v, max(0.3, dist_goal))                 # slow down near the goal
            w = 2.0 * v * math.sin(alpha) / config.LOOKAHEAD  # pure pursuit curvature
            w = max(-config.MAX_OMEGA, min(config.MAX_OMEGA, w))
        return v, w


class SafetyMonitor:
    """Looks at the path ahead and says 'ok', 'replan' or 'stop' (obstacle too close)."""

    def check(self, pose, path_ahead, obstacles):
        x, y, theta = pose
        status = "ok"
        for o in obstacles:
            # where is the obstacle now, and 1 second from now?
            spots = [(o.x, o.y), (o.x + o.vx, o.y + o.vy)]
            too_close = math.hypot(o.x - x, o.y - y) < config.OBSTACLE_RADIUS + config.VEHICLE_RADIUS + 0.4
            if too_close:
                return "stop"
            for (ox, oy) in spots:
                for (px, py) in path_ahead:
                    if math.hypot(px - ox, py - oy) < config.THREAT_RADIUS:
                        status = "replan"
        return status
