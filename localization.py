import math
import numpy as np
import config

rng = np.random.default_rng(config.SEED)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Vehicle:
    """The 'real' car (ground truth). It moves perfectly; only the sensors are noisy."""

    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta

    def move(self, v, w, dt):
        self.theta = wrap(self.theta + w * dt)
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt


class Localizer:
    """
    Simple sensor fusion:
      1. Predict: dead reckoning with noisy odometry (drifts over time)
      2. Correct: nudge the estimate toward a noisy GPS + compass reading
    It's a complementary filter, a simplified cousin of the EKF.
    """

    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta

    def update(self, v, w, dt, true_vehicle):
        # 1. predict with noisy odometry
        v_meas = v + rng.normal(0, config.ODOM_V_NOISE)
        w_meas = w + rng.normal(0, config.ODOM_W_NOISE) + config.ODOM_W_BIAS
        self.theta = wrap(self.theta + w_meas * dt)
        self.x += v_meas * math.cos(self.theta) * dt
        self.y += v_meas * math.sin(self.theta) * dt

        # 2. correct with noisy GPS + compass
        gps_x = true_vehicle.x + rng.normal(0, config.GPS_NOISE)
        gps_y = true_vehicle.y + rng.normal(0, config.GPS_NOISE)
        compass = true_vehicle.theta + rng.normal(0, config.COMPASS_NOISE)
        self.x += config.GPS_GAIN * (gps_x - self.x)
        self.y += config.GPS_GAIN * (gps_y - self.y)
        self.theta = wrap(self.theta + config.HEADING_GAIN * wrap(compass - self.theta))
        return self.pose()

    def pose(self):
        return (self.x, self.y, self.theta)
