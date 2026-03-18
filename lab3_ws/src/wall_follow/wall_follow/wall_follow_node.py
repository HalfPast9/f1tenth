import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.subscription = self.create_subscription(LaserScan,'/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        self.kp = 3.0
        self.kd = 0.75
        self.ki = 0.0  # leave at 0 until P+D is working
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need

        self.perp_angle = np.radians(90)
        self.chosen_angle = np.radians(65)
        self.theta = self.perp_angle - self.chosen_angle
        self.lookahead_dist = 1
        self.prev_time = 0.0
        self.angle = 0.0

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        
        index = (angle - self.angle_min) / self.angle_increment
        
        if range_data[int(index)] == float('inf') or np.isnan(range_data[int(index)]):
            return -1.0
        else:
            return range_data[int(index)]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement

        range_at_90 = self.get_range(range_data, self.perp_angle) # b
        range_at_45 = self.get_range(range_data, self.chosen_angle) # a

        range_at_90 = min(range_at_90, 3.0)
        range_at_45 = min(range_at_45, 3.0)

        if range_at_90 < 0 or range_at_45 < 0:
            return 0.0


        ego_pose = np.arctan((range_at_45 * np.cos(self.theta) - range_at_90) / (range_at_45 * np.sin(self.theta))) #alpha
        Dt = range_at_90 * np.cos(ego_pose)
        Dt_future = Dt + self.lookahead_dist * np.sin(ego_pose)

        future_error = Dt_future - dist


        return future_error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # TODO: Use kp, ki & kd to implement a PID controller
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.prev_time if self.prev_time > 0.0 else 0.1
        self.prev_time = now

        self.integral += error * dt
        P = self.kp * error
        I = self.ki * self.integral 
        D = self.kd * (error - self.prev_error) / dt 
        self.angle = np.clip(P + I + D, -0.418, 0.418)
        self.prev_error = error 

        # TODO: fill in drive message and publish   
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = self.angle
        drive_msg.drive.speed = velocity
        self.publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.desired_distance = 1.0 # TODO: tune this value

        error = self.get_error(msg.ranges, self.desired_distance) # TODO: replace with error calculated by get_error()
        
        if abs(self.angle) < np.radians(10):
            velocity = 3.0
        elif abs(self.angle) < np.radians(20):
            velocity = 1.5
        else:
            velocity = 0.75
        
        self.pid_control(error, velocity) # TODO: actuate the car with PID
        


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()