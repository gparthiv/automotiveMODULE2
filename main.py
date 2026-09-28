import math
import config
from grid_map import GridMap
from localization import Vehicle, Localizer
from a_star_planner import plan_path
from controller import PurePursuit, SafetyMonitor


class Simulation:
    def __init__(self):
        self.world = GridMap()
        self.vehicle = Vehicle(config.START[0], config.START[1], 0.0)
        self.localizer = Localizer(config.START[0], config.START[1], 0.0)
        self.controller = PurePursuit()
        self.monitor = SafetyMonitor()
        self.goal = config.GOAL
        self.t = 0.0
        self.steps = 0
        self.v, self.w = 0.0, 0.0
        self.status = "ok"
        self.last_replan = -100.0
        self.collisions = 0
        self.set_goal(self.goal)

    def set_goal(self, goal):
        self.goal = goal
        pose = self.localizer.pose()
        path = plan_path(self.world.grid_with_obstacles(), pose[:2], goal, config.RESOLUTION)
        if path is None:
            path = plan_path(self.world.inflated, pose[:2], goal, config.RESOLUTION)
        if path is None:
            raise RuntimeError("No path to goal")
        self.controller.set_path(path)
        print(f"[plan] path with {len(path)} waypoints to {goal}")

    def step(self):
        dt = config.DT
        # 1. world moves, car moves using last command
        self.world.step(dt)
        self.vehicle.move(self.v, self.w, dt)
        pose = self.localizer.update(self.v, self.w, dt, self.vehicle)

        # 2. safety check (replan if something blocks the path)
        self.status = self.monitor.check(pose, self.controller.path_ahead(), self.world.obstacles)
        if self.status != "ok" and self.t - self.last_replan > config.REPLAN_INTERVAL:
            self.last_replan = self.t
            new_path = plan_path(self.world.grid_with_obstacles(), pose[:2], self.goal, config.RESOLUTION)
            if new_path:
                self.controller.set_path(new_path)
                print(f"[t={self.t:5.1f}] obstacle on path -> replanned")
                if self.status == "replan":
                    self.status = self.monitor.check(pose, self.controller.path_ahead(), self.world.obstacles)
            else:
                print(f"[t={self.t:5.1f}] no free path -> waiting")

        # 3. control
        if self.status == "ok":
            self.v, self.w = self.controller.command(pose)
        elif self.status == "stop":
            # obstacle too close: drive straight away from it (reverse if it's in front, forward if behind)
            near = min(self.world.obstacles, key=lambda o: math.hypot(o.x - pose[0], o.y - pose[1]))
            in_front = math.cos(pose[2]) * (near.x - pose[0]) + math.sin(pose[2]) * (near.y - pose[1]) > 0
            self.v, self.w = (-1.5 if in_front else 1.5), 0.0
        else:
            self.v, self.w = 0.0, 0.0        # obstacle near the path: brake and yield

        # 4. bookkeeping
        for o in self.world.obstacles:
            if math.hypot(o.x - self.vehicle.x, o.y - self.vehicle.y) < o.radius + config.VEHICLE_RADIUS:
                self.collisions += 1
        self.t += dt
        self.steps += 1
        if self.steps % 20 == 0:
            err = math.hypot(pose[0] - self.vehicle.x, pose[1] - self.vehicle.y)
            print(f"[t={self.t:5.1f}] est=({pose[0]:5.2f},{pose[1]:5.2f}) "
                  f"v={self.v:4.2f} loc_err={err:4.2f}m status={self.status}")

    def reached_goal(self):
        return math.hypot(self.goal[0] - self.vehicle.x, self.goal[1] - self.vehicle.y) < config.GOAL_TOLERANCE

    def done(self):
        return self.reached_goal() or self.t > config.MAX_TIME


def main():
    import matplotlib.pyplot as plt
    from visualizer import Visualizer

    sim = Simulation()
    viz = Visualizer(sim.world, sim.goal)
    while not sim.done():
        sim.step()
        if sim.steps % 2 == 0:
            viz.update(sim)

    if sim.reached_goal():
        print(f"GOAL REACHED in {sim.t:.1f}s, collisions: {sim.collisions}")
    else:
        print(f"TIMEOUT after {sim.t:.1f}s, collisions: {sim.collisions}")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
