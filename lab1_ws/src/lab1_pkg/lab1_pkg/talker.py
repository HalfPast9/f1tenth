import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

   def __init__(self):
      super().__init__('minimal_publisher')
      self.publisher_ = self.create_publisher(String, 'topic', 10)
      timer_period = 0.5  # seconds
      self.timer = self.create_timer(timer_period, self.timer_callback)
      self.i = 0

   def timer_callback(self):
      v = 1
      d = 1

      msg_v = String()
      msg_d = String()
      msg_v.data = f'Value v: {v}'  # Set the string data
      msg_d.data = f'Value d: {d}'  # Set the string data
      self.publisher_.publish(msg_d)
      self.publisher_.publish(msg_v)
      self.get_logger().info(f'Publishing: "{msg_v.data}", "{msg_d.data}"')
      
def main(args=None):
   rclpy.init(args=args)
   minimal_publisher = MinimalPublisher()
   rclpy.spin(minimal_publisher)
   minimal_publisher.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()