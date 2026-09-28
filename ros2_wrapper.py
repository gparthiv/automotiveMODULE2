"""
Optional ROS 2 wrapper. Run with:  python3 ros2_wrapper.py
Publishes:  /map (OccupancyGrid), /estimated_pose (PoseStamped), /cmd_vel (Twist)
Subscribes: /goal_pose (PoseStamped) -> sets a new goal and replans
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
import config
from main import Simulation


class PipelineNode(Node):
    def __init__(self):
        super().__init__("av_pipeline")
        self.sim = Simulation()
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.pose_pub = self.create_publisher(PoseStamped, "/estimated_pose", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_timer(config.DT, self.tick)     # 20 Hz
        self.create_timer(1.0, self.publish_map)

    def on_goal(self, msg):
        self.sim.set_goal((msg.pose.position.x, msg.pose.position.y))

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = config.RESOLUTION
        msg.info.width = msg.info.height = config.GRID_SIZE
        msg.info.origin.orientation.w = 1.0
        msg.data = [100 if c else 0 for c in self.sim.world.static.flatten()]
        self.map_pub.publish(msg)

    def tick(self):
        if not self.sim.done():
            self.sim.step()
        x, y, th = self.sim.localizer.pose()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(th / 2)
        pose.pose.orientation.w = math.cos(th / 2)
        self.pose_pub.publish(pose)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self.sim.v, self.sim.w
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(PipelineNode())


if __name__ == "__main__":
    main()
