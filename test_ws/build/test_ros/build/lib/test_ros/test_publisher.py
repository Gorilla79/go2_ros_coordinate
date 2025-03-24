import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(String, 'test_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Test Publisher Node started.')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from test node!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
