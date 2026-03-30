import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive
        self.subscription = self.create_subscription(LaserScan,'/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.filtered_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        self.disparity_pub = self.create_publisher(LaserScan, '/disparity_scan', 10)

        self.disparity_threshold = 0.2
        self.max_speed = 2.5
        self.min_speed = 0.5
        self.max_range = 20.0
        self.car_width = 0.8
        self.max_dist = 3.0
        self.min_dist = 0.8

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)
        #TODO: replace NaNs and infs with sensible values, clip to max range
        for i in range(len(proc_ranges)):
            if proc_ranges[i] == float('inf') or np.isnan(proc_ranges[i]):
                proc_ranges[i] = self.max_range
            elif proc_ranges[i] > self.max_range:
                proc_ranges[i] = self.max_range
        return proc_ranges


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        print(f"angle_min: {data.angle_min}, angle_max: {data.angle_max}, total samples: {len(data.ranges)}")
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        angles = data.angle_min + np.arange(len(proc_ranges)) * data.angle_increment

        diffs = np.abs(np.diff(proc_ranges))
        disparity_indices = np.where(diffs > self.disparity_threshold)[0]
        for i in disparity_indices:
            closer_dist = min(proc_ranges[i], proc_ranges[i+1])
            n_samples = int(np.ceil((self.car_width / 2) / (closer_dist * data.angle_increment)))
            if proc_ranges[i+1] > proc_ranges[i]:
                # right side is farther, extend rightward
                end = min(i + 1 + n_samples, len(proc_ranges))
                for j in range(i+1, end):
                    proc_ranges[j] = min(proc_ranges[j], closer_dist)
            else:
                # left side is farther, extend leftward
                start = max(i - n_samples + 1, 0)
                for j in range(start, i+1):
                    proc_ranges[j] = min(proc_ranges[j], closer_dist)

        forward_mask = (angles >= -np.pi/2) & (angles <= np.pi/2)
        masked_ranges = np.where(forward_mask, proc_ranges, -1)
        top_n = 10
        top_indices = np.argpartition(masked_ranges, -top_n)[-top_n:]
        best_angle = np.mean(angles[top_indices])

        if best_angle > 0:  # turning left, check left rear
            rear_mask = angles > np.pi/2
        else:  # turning right, check right rear
            rear_mask = angles < -np.pi/2

        rear_ranges = proc_ranges[rear_mask]
        if len(rear_ranges) > 0 and np.min(rear_ranges) < 0.3:
            best_angle = 0.0  # go straight

        fwd_idx = int(np.argmin(np.abs(angles)))
        fwd_dist = float(proc_ranges[fwd_idx])

        if fwd_dist >= self.max_dist:
            speed = self.max_speed
        elif fwd_dist <= self.min_dist:
            speed = self.min_speed
        else:
            t = (fwd_dist - self.min_dist) / (self.max_dist - self.min_dist)
            speed = self.min_speed + t * (self.max_speed - self.min_speed)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = best_angle
        self.publisher.publish(drive_msg)   

        # filtered scan - full processed ranges
        filtered_msg = LaserScan()
        filtered_msg.header = data.header
        filtered_msg.angle_min = data.angle_min
        filtered_msg.angle_max = data.angle_max
        filtered_msg.angle_increment = data.angle_increment
        filtered_msg.range_min = data.range_min
        filtered_msg.range_max = data.range_max
        filtered_msg.ranges = proc_ranges.tolist()
        self.filtered_pub.publish(filtered_msg)

        # disparity scan - only changed points
        disparity_ranges = np.where(proc_ranges < np.array(ranges), proc_ranges, self.max_range)
        disparity_msg = LaserScan()
        disparity_msg.header = data.header
        disparity_msg.angle_min = data.angle_min
        disparity_msg.angle_max = data.angle_max
        disparity_msg.angle_increment = data.angle_increment
        disparity_msg.range_min = data.range_min
        disparity_msg.range_max = data.range_max
        disparity_msg.ranges = disparity_ranges.tolist()
        self.disparity_pub.publish(disparity_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()