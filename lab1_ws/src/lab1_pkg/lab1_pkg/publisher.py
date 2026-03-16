import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.declare_parameter("V", 1.0)
        self.declare_parameter("D", 2.0)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        msg = AckermannDriveStamped()
        msg.drive.speed = self.get_parameter("V").get_parameter_value().double_value
        msg.drive.steering_angle = self.get_parameter("D").get_parameter_value().double_value

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.drive.speed) + " ," + str(msg.drive.steering_angle))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()