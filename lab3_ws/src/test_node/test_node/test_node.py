import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('mount works!')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()