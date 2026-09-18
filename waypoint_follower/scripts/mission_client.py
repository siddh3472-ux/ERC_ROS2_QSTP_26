#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from waypoint_follower.action import Mission


class MissionClient(Node):

    def __init__(self):

        super().__init__('mission_client')

        self.client = ActionClient(
            self,
            Mission,
            'follow_mission'
        )

        self.goal_handle = None

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Waypoint: '
            f'{feedback.current_waypoint_index} | '
            f'{feedback.status} | '
            f'Distance: '
            f'{feedback.distance_to_target:.2f} m'
        )

    def send_goal(self, filename):

        self.get_logger().info(
            'Waiting for mission server...'
        )

        if not self.client.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                'Mission server not available'
            )

            return False

        goal = Mission.Goal()

        goal.mission_file = filename

        self.get_logger().info(
            f'Sending mission: {filename}'
        )

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.goal_handle = future.result()

        if self.goal_handle is None:

            self.get_logger().error(
                'Goal was not accepted'
            )

            return False

        if not self.goal_handle.accepted:

            self.get_logger().error(
                'Goal rejected'
            )

            return False

        self.get_logger().info(
            'Goal accepted'
        )

        result_future = (
            self.goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result().result

        self.get_logger().info(
            f'Mission finished | '
            f'Success: {result.success} | '
            f'Distance: '
            f'{result.total_distance:.2f} m | '
            f'Waypoints completed: '
            f'{result.waypoints_completed}'
        )

        return True

    def cancel(self):

        if self.goal_handle is None:
            return

        if not self.goal_handle.accepted:
            return

        self.get_logger().info(
            'Cancelling mission...'
        )

        future = (
            self.goal_handle.cancel_goal_async()
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )


def main(args=None):

    rclpy.init(args=args)

    node = MissionClient()

    if len(sys.argv) > 1:

        filename = sys.argv[1]

    else:

        filename = 'mission_square.yaml'

    try:

        node.send_goal(filename)

    except KeyboardInterrupt:

        node.cancel()

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
