#!/usr/bin/env python3

import math
import os
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

from waypoint_follower.action import Mission


class MissionServer(Node):

    def __init__(self):
        super().__init__("mission_server")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.action_server = ActionServer(
            self,
            Mission,
            "follow_mission",
            self.execute_callback
        )

        self.get_logger().info("Mission Server Ready")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(sin_yaw, cos_yaw)
        self.odom_received = True

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def angle_error(self, target_x, target_y):
        target_angle = math.atan2(
            target_y - self.y,
            target_x - self.x
        )

        error = target_angle - self.yaw

        return math.atan2(math.sin(error), math.cos(error))

    def execute_callback(self, goal_handle):

        self.get_logger().info("Mission Started")

        # Wait until we have a real odometry reading.
        while rclpy.ok() and not self.odom_received:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Mission.Result()
                result.success = False
                result.total_distance = 0.0
                result.waypoints_completed = 0
                return result

            threading.Event().wait(0.05)

        mission_file = goal_handle.request.mission_file

        package_path = get_package_share_directory(
            "waypoint_follower"
        )

        mission_path = os.path.join(
            package_path,
            "missions",
            mission_file
        )

        try:
            with open(mission_path, "r") as f:
                mission = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(
                f"Could not load mission: {e}"
            )

            goal_handle.abort()

            result = Mission.Result()
            result.success = False
            result.total_distance = 0.0
            result.waypoints_completed = 0

            return result

        waypoints = mission.get("waypoints", [])

        base = mission.get(
            "base",
            {"x": 0.0, "y": 0.0}
        )

        targets = list(waypoints)

        if mission.get("return_to_base", False):
            targets.append(base)

        completed = 0
        total_distance = 0.0

        last_x = self.x
        last_y = self.y

        for i, target in enumerate(targets):

            target_x = float(target["x"])
            target_y = float(target["y"])

            while rclpy.ok():

                if goal_handle.is_cancel_requested:

                    self.stop_robot()
                    goal_handle.canceled()

                    result = Mission.Result()
                    result.success = False
                    result.total_distance = float(total_distance)
                    result.waypoints_completed = completed

                    return result

                dx = target_x - self.x
                dy = target_y - self.y

                distance = math.sqrt(
                    dx * dx + dy * dy
                )

                moved = math.sqrt(
                    (self.x - last_x) ** 2 +
                    (self.y - last_y) ** 2
                )

                total_distance += moved

                last_x = self.x
                last_y = self.y

                if distance < 0.15:

                    self.stop_robot()
                    break

                error = self.angle_error(
                    target_x,
                    target_y
                )

                cmd = Twist()

                cmd.linear.x = min(
                    0.4,
                    0.7 * distance
                )

                cmd.angular.z = 1.5 * error

                if abs(error) > 0.6:
                    cmd.linear.x = 0.0

                self.cmd_pub.publish(cmd)

                feedback = Mission.Feedback()

                if i < len(waypoints):
                    feedback.current_waypoint_index = i + 1
                    feedback.status = (
                        f"En route to waypoint "
                        f"{i + 1}/{len(waypoints)}"
                    )
                else:
                    feedback.current_waypoint_index = len(waypoints)
                    feedback.status = "Returning to base"

                feedback.distance_to_target = float(distance)

                goal_handle.publish_feedback(feedback)

                threading.Event().wait(0.05)

            if i < len(waypoints):

                completed += 1

                feedback = Mission.Feedback()
                feedback.current_waypoint_index = i + 1
                feedback.status = (
                    f"Waypoint {i + 1} completed"
                )
                feedback.distance_to_target = 0.0

                goal_handle.publish_feedback(feedback)

        self.stop_robot()

        goal_handle.succeed()

        result = Mission.Result()
        result.success = True
        result.total_distance = float(total_distance)
        result.waypoints_completed = completed

        self.get_logger().info(
            f"Mission Complete: {total_distance:.2f} m"
        )

        return result


def main(args=None):

    rclpy.init(args=args)

    node = MissionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
