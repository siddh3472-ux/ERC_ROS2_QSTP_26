import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        self.is_active = False

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.toggle_service = self.create_service(
            SetBool,
            '/toggle_robot',
            self.toggle_robot_callback
        )

        self.get_logger().info('Obstacle Avoider ready. Robot is OFF.')

    def toggle_robot_callback(self, request, response):
        self.is_active = request.data

        response.success = True

        if self.is_active:
            response.message = 'Robot turned ON'
            self.get_logger().info('Robot turned ON')
        else:
            response.message = 'Robot turned OFF'
            self.get_logger().info('Robot turned OFF')
            self.stop_robot()

        return response

    def scan_callback(self, msg):

        if not self.is_active:
            self.stop_robot()
            return

        front_ranges = msg.ranges[0:30] + msg.ranges[330:360]

        valid_ranges = [
            r for r in front_ranges
            if r > 0.0
        ]

        if not valid_ranges:
            return

        front_distance = min(valid_ranges)

        twist = Twist()

        if front_distance < 1.0:

            left_distance = msg.ranges[90]
            right_distance = msg.ranges[270]

            if left_distance > right_distance:
                twist.angular.z = 0.8
            else:
                twist.angular.z = -0.8

            twist.linear.x = 0.0

        else:

            twist.linear.x = 0.15
            twist.angular.z = 0.3

        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoider()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
