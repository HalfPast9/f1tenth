#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import csv
import math
import numpy as np


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter('waypoints_file', '/sim_ws/src/pure_pkg/pure_pkg/waypoints.csv')
        self.declare_parameter('lookahead_distance', 1.20)   # metres — tune this
        self.declare_parameter('speed', 2.0)                # m/s — tune this
        self.declare_parameter('wheelbase', 0.33)           # F1Tenth wheelbase
        self.declare_parameter('max_steer', 0.4189)         # ~24 deg in radians

        self.L   = self.get_parameter('lookahead_distance').value
        self.spd = self.get_parameter('speed').value
        self.wb  = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steer').value

        wf = self.get_parameter('waypoints_file').value
        self.waypoints = self._load_waypoints(wf)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {wf}')

        self.current_idx = 0

        self.sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

    def _load_waypoints(self, filepath):
        pts = []
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pts.append((float(row['x']), float(row['y'])))
        return np.array(pts)

    def _get_yaw(self, msg):
        q = msg.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _find_lookahead(self, x, y):
        n = len(self.waypoints)

        # advance current_idx to stay close to the car (never go backwards)
        for _ in range(n):
            idx = self.current_idx % n
            dist = math.hypot(self.waypoints[idx][0] - x,
                              self.waypoints[idx][1] - y)
            if dist < self.L:
                self.current_idx += 1
            else:
                break

        # from current_idx, find first waypoint >= L away
        for i in range(n):
            idx = (self.current_idx + i) % n
            dist = math.hypot(self.waypoints[idx][0] - x,
                              self.waypoints[idx][1] - y)
            if dist >= self.L:
                return self.waypoints[idx]

        # fallback — return current waypoint
        return self.waypoints[self.current_idx % n]

    def odom_callback(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = self._get_yaw(msg)

        lx, ly = self._find_lookahead(x, y)

        # angle from car heading to lookahead point
        alpha = math.atan2(ly - y, lx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # wrap to [-pi, pi]

        # pure pursuit formula: delta = atan(2 * wb * sin(alpha) / L)
        dist  = math.hypot(lx - x, ly - y)
        delta = math.atan2(2.0 * self.wb * math.sin(alpha), dist)
        delta = max(-self.max_steer, min(self.max_steer, delta))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = delta
        drive_msg.drive.speed = self.spd
        self.pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()