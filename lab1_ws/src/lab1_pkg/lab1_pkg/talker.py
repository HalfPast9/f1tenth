import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class MinimalPublisher(Node):

   def __init__(self):
      super().__init__('minimal_publisher')
      self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
      self.declare_parameter('v', 1.0)
      self.declare_parameter('d', 1.0)
      timer_period = 0  # seconds
      self.timer = self.create_timer(timer_period, self.timer_callback)

   def timer_callback(self):
      msg = AckermannDriveStamped()
      msg.drive.speed = self.get_parameter('v').get_parameter_value().double_value
      msg.drive.steering_angle = self.get_parameter('d').get_parameter_value().double_value

      self.publisher_.publish(msg)

      self.get_logger().info('Publishing: v=%s, d=%s' % (msg.drive.speed, msg.drive.steering_angle))
      
      
def main(args=None):
   rclpy.init(args=args)
   minimal_publisher = MinimalPublisher()
   rclpy.spin(minimal_publisher)
   minimal_publisher.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()