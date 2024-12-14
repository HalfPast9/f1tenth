import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class MinimalSubscriber(Node):

   def __init__(self):
      super().__init__('minimal_subscriber')
      self.subscription = self.create_subscription(AckermannDriveStamped, 'drive', self.listener_callback, 10)
      self.subscription  # prevent unused variable warning

      self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

   def listener_callback(self, msg):
      relay_msg = AckermannDriveStamped()
      relay_msg.drive.speed = msg.drive.speed * 3
      relay_msg.drive.steering_angle = msg.drive.steering_angle * 3

      self.publisher_.publish(relay_msg)

      self.get_logger().info('Relaying: v=%s, d=%s' % (relay_msg.drive.speed, relay_msg.drive.steering_angle))
   
def main(args=None):
   rclpy.init(args=args)
   minimal_subscriber = MinimalSubscriber()
   rclpy.spin(minimal_subscriber)
   minimal_subscriber.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
