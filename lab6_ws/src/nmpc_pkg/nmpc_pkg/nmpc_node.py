#!/usr/bin/env python3
import math
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from nmpc_pkg.params import VehicleParams, SolverParams
from nmpc_pkg.reference_generator import ReferenceGenerator
from nmpc_pkg.nmpc_solver import NMPCSolver
from nmpc_pkg.diagnostics import DiagnosticLogger

_shutdown = False


def _signal_handler(sig, frame):
    global _shutdown
    _shutdown = True


class NMPCController(Node):
    def __init__(self):
        super().__init__('nmpc_controller')

        self.declare_parameter('waypoints_file',
                               '/sim_ws/src/nmpc_pkg/nmpc_pkg/waypoints.csv')
        self.declare_parameter('control_rate', 30.0)

        vp = VehicleParams()
        self.sp = sp = SolverParams()

        wf = self.get_parameter('waypoints_file').value
        self.ref_gen = ReferenceGenerator(wf, d_s=sp.d_s, P=sp.P)
        self.get_logger().info(
            f'Loaded centerline: {self.ref_gen.K} points from {wf}')

        self.solver = NMPCSolver(vp, sp)
        self.get_logger().info(
            f'NMPC solver ready (7-state dynamic): N={sp.N}, T_s={sp.T_s}')

        # Full 7-state: [px, py, delta, vel, yaw, yaw_rate, slip_angle]
        self.current_state = None
        self.u_prev = np.array([0.0, 0.0])  # [sv, accl]
        self.last_good_u = np.array([0.0, 0.0])
        self.last_steer_cmd = 0.0

        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_callback)

        self.diag = DiagnosticLogger()
        self.get_logger().info('NMPC controller node started (diagnostics ON)')

    def odom_callback(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vel = math.sqrt(vx**2 + vy**2)
        yaw_rate = msg.twist.twist.angular.z

        if vel > 0.1:
            slip_angle = math.atan2(vy, vx)
        else:
            slip_angle = 0.0

        # state: [px, py, delta, vel, yaw, yaw_rate, slip_angle]
        # delta (steering angle) is not directly observable from odom;
        # estimate from last command
        self.current_state = np.array([
            px, py, self.last_steer_cmd, vel, yaw, yaw_rate, slip_angle
        ])

    def control_callback(self):
        if _shutdown or self.current_state is None:
            return

        reference, max_speeds, ref_headings = self.ref_gen.get_reference(
            self.current_state[0], self.current_state[1])

        try:
            u_opt, X_sol, info = self.solver.solve(
                self.current_state, reference,
                max_speeds, ref_headings, self.u_prev)
        except Exception as e:
            self.get_logger().error(f'Solver failed: {e}')
            self._publish_safe()
            return

        if info['status'] != 'Solve_Succeeded':
            self.get_logger().warn(
                f'NMPC {info["time_ms"]:.0f}ms | {info["status"]} — holding safe',
                throttle_duration_sec=0.5)
            self.diag.log(self.current_state, self.last_good_u, self.u_prev,
                          reference, max_speeds, X_sol, info)
            self._publish_safe()
            self.solver.reset_warm_start()
            return

        speed_cmd = float(u_opt[0])
        steer_cmd = float(u_opt[1])

        self.diag.log(self.current_state, u_opt, self.u_prev,
                      reference, max_speeds, X_sol, info)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = max(speed_cmd, 0.0)
        drive_msg.drive.steering_angle = steer_cmd
        self.drive_pub.publish(drive_msg)

        self.u_prev = info['u_internal']
        self.last_good_u = u_opt.copy()
        self.last_steer_cmd = steer_cmd

        self.get_logger().info(
            f'NMPC {info["time_ms"]:.0f}ms | '
            f'spd={speed_cmd:.2f} str={steer_cmd:.3f} | '
            f'{info["status"]}',
            throttle_duration_sec=1.0)

    def _publish_safe(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = max(float(self.last_good_u[0]) * 0.5, 0.0)
        drive_msg.drive.steering_angle = float(self.last_good_u[1])
        self.drive_pub.publish(drive_msg)


def main(args=None):
    signal.signal(signal.SIGINT, _signal_handler)
    rclpy.init(args=args)

    node = NMPCController()

    while not _shutdown:
        rclpy.spin_once(node, timeout_sec=0.05)

    stop_msg = AckermannDriveStamped()
    stop_msg.drive.speed = 0.0
    stop_msg.drive.steering_angle = 0.0
    node.drive_pub.publish(stop_msg)

    path = node.diag.save()
    print(f'Diagnostics saved to {path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
