#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        self.declare_parameter('output_file', 'waypoints.csv')
        self.declare_parameter('min_distance', 0.1)  # metres between waypoints

        self.output_file = self.get_parameter('output_file').value
        self.min_distance = self.get_parameter('min_distance').value

        self.waypoints = []
        self.last_x = None
        self.last_y = None

        self.sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(f'Recording to: {self.output_file}')
        self.get_logger().info(f'Min spacing: {self.min_distance}m')
        self.get_logger().info('Drive the car one full lap. Ctrl+C to save and exit.')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # quaternion → yaw without tf_transformations dependency
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        if self.last_x is None:
            self._record(x, y, yaw)
            return

        dist = math.sqrt((x - self.last_x) ** 2 + (y - self.last_y) ** 2)
        if dist >= self.min_distance:
            self._record(x, y, yaw)

    def _record(self, x, y, yaw):
        self.waypoints.append((x, y, yaw))
        self.last_x = x
        self.last_y = y
        self.get_logger().info(
            f'[{len(self.waypoints)}] ({x:.3f}, {y:.3f}, {yaw:.3f})'
        )

    def save(self):
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw'])
            writer.writerows(self.waypoints)
        self.get_logger().info(
            f'Saved {len(self.waypoints)} waypoints to {self.output_file}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()