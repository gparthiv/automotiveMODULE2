# All the tunable numbers live here.

# Map
RESOLUTION = 0.5        # meters per grid cell
GRID_SIZE = 40          # 40 x 40 cells = 20m x 20m

# Mission
START = (2.0, 2.0)
GOAL = (18.0, 18.0)
GOAL_TOLERANCE = 0.5
MAX_TIME = 90.0         # seconds before giving up

# Time step (0.05 s = 20 Hz)
DT = 0.05

# Vehicle
MAX_SPEED = 2.0         # m/s
MAX_OMEGA = 2.0         # rad/s
LOOKAHEAD = 1.5         # pure pursuit lookahead distance (m)
VEHICLE_RADIUS = 0.3
OBSTACLE_RADIUS = 0.4

# Safety
SAFETY_RADIUS = 1.3     # dynamic obstacles are inflated by this much when replanning
THREAT_RADIUS = 1.0     # path point closer than this to an obstacle = threat
CHECK_DISTANCE = 5.0    # only look this far ahead along the path (m)
REPLAN_INTERVAL = 1.0   # min seconds between replans

# Sensor noise (for localization)
ODOM_V_NOISE = 0.05
ODOM_W_NOISE = 0.03
ODOM_W_BIAS = 0.02      # small constant turn error, makes pure dead reckoning drift
GPS_NOISE = 0.3
COMPASS_NOISE = 0.05
GPS_GAIN = 0.1          # how much we trust GPS each step (0..1)
HEADING_GAIN = 0.1

SEED = 1
