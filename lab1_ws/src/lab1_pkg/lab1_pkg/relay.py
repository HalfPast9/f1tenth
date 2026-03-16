import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(AckermannDriveStamped,'drive', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)
        self.i = 0

    def listener_callback(self, msg):
        msg_out = AckermannDriveStamped()
        msg_out.drive.speed = (msg.drive.speed * 3)
        msg_out.drive.steering_angle = (msg.drive.steering_angle * 3)
        self.publisher_.publish(msg_out)
        self.get_logger().info('Publishing: "%s"' % str(msg_out.drive.speed) + " ," + str(msg_out.drive.steering_angle))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()