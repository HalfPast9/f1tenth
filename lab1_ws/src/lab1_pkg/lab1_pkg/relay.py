import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

   def __init__(self):
      super().__init__('minimal_subscriber')
      self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
      self.subscription  # prevent unused variable warning

   def listener_callback(self, msg_v, msg_d):

      v = int(msg_v.data.split()[2])
      d = int(msg_d.data.split()[2])
      self.get_logger().info('I heard: v=%s, d=%s' % (v, d))
   
def main(args=None):
   rclpy.init(args=args)
   minimal_subscriber = MinimalSubscriber()
   rclpy.spin(minimal_subscriber)
   minimal_subscriber.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
