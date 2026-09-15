#!/usr/bin/env python3

import sys

import rclpy
from rclpy.action import ActionClient

from waypoint_follower.action import Mission


class MissionClient:

    def __init__(self):
        self.node = rclpy.create_node('mission_client')

        self.action_client = ActionClient(
            self.node,
            Mission,
            'follow_mission'
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.node.get_logger().info(
            f'Waypoint: {feedback.current_waypoint_index} | '
            f'{feedback.status} | '
            f'Distance: {feedback.distance_to_target:.2f} m'
        )

    def send_goal(self, mission_file):

        self.node.get_logger().info(
            'Waiting for mission server...'
        )

        self.action_client.wait_for_server()

        goal_msg = Mission.Goal()
        goal_msg.mission_file = mission_file

        self.node.get_logger().info(
            f'Sending mission: {mission_file}'
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(
            self.node,
            send_goal_future
        )

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error(
                'Mission goal was rejected.'
            )
            return

        self.node.get_logger().info(
            'Mission goal accepted.'
        )

        result_future = goal_handle.get_result_async()

        try:
            rclpy.spin_until_future_complete(
                self.node,
                result_future
            )

            result = result_future.result().result

            self.node.get_logger().info(
                f'Success: {result.success}'
            )
            self.node.get_logger().info(
                f'Total distance: '
                f'{result.total_distance:.2f} m'
            )
            self.node.get_logger().info(
                f'Waypoints completed: '
                f'{result.waypoints_completed}'
            )

        except KeyboardInterrupt:

            self.node.get_logger().info(
                'Ctrl+C detected. Cancelling mission...'
            )

            cancel_future = goal_handle.cancel_goal_async()

            rclpy.spin_until_future_complete(
                self.node,
                cancel_future
            )

            self.node.get_logger().info(
                'Cancellation request sent.'
            )


def main(args=None):

    rclpy.init(args=args)

    mission_file = sys.argv[1] if len(sys.argv) > 1 else "mission_square.yaml"

    client = MissionClient()

    try:
        client.send_goal(mission_file)
    except KeyboardInterrupt:
        pass

    client.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
