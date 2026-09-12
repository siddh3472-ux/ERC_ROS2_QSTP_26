import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            Float32,
            '/random_number',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data:.2f}. Multiplied value: {msg.data * 2:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
