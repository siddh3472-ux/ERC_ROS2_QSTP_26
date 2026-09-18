#!/usr/bin/env python3

import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory

from waypoint_follower.action import Mission


def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def clamp(value, low, high):

    return max(low, min(high, value))


class WaypointFollower(Node):

    def __init__(self):

        super().__init__('mission_server')

        # Current robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.pose_received = False

        self.callback_group = ReentrantCallbackGroup()

        # Velocity publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        # Action server
        self.action_server = ActionServer(
            self,
            Mission,
            'follow_mission',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            'Mission Server Ready'
        )

    # ---------------------------------------------------------
    # ODOMETRY
    # ---------------------------------------------------------

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

        self.pose_received = True

    # ---------------------------------------------------------
    # ACTION GOAL
    # ---------------------------------------------------------

    def goal_callback(self, goal_request):

        self.get_logger().info(
            'Mission goal received'
        )

        return GoalResponse.ACCEPT

    # ---------------------------------------------------------
    # CANCEL
    # ---------------------------------------------------------

    def cancel_callback(self, goal_handle):

        self.stop_robot()

        self.get_logger().info(
            'Mission cancellation requested'
        )

        return CancelResponse.ACCEPT

    # ---------------------------------------------------------
    # STOP
    # ---------------------------------------------------------

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ---------------------------------------------------------
    # WAIT FOR ODOM
    # ---------------------------------------------------------

    def wait_for_odom(self, goal_handle):

        start = time.monotonic()

        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                return False

            if self.pose_received:
                return True

            if time.monotonic() - start > 10.0:
                return False

            time.sleep(0.05)

        return False

    # ---------------------------------------------------------
    # MOVE TO ONE WAYPOINT
    # ---------------------------------------------------------

    def move_to_waypoint(
        self,
        goal_handle,
        target_x,
        target_y,
        index,
        total,
        feedback
    ):

        # =====================================================
        # PHASE 1: TURN TOWARDS WAYPOINT
        # =====================================================

        turn_start = time.monotonic()

        while rclpy.ok():

            if goal_handle.is_cancel_requested:

                self.stop_robot()
                return False, 0.0

            dx = target_x - self.x
            dy = target_y - self.y

            distance = math.hypot(
                dx,
                dy
            )

            if distance <= 0.20:

                self.stop_robot()
                return True, distance

            desired_yaw = math.atan2(
                dy,
                dx
            )

            yaw_error = normalize_angle(
                desired_yaw - self.yaw
            )

            # Already facing target
            if abs(yaw_error) <= 0.08:
                break

            cmd = Twist()

            cmd.linear.x = 0.0

            # Very slow, bounded turning
            cmd.angular.z = clamp(
                0.70 * yaw_error,
                -0.12,
                0.12
            )

            self.cmd_pub.publish(cmd)

            feedback.current_waypoint_index = index
            feedback.status = (
                f'Turning to waypoint '
                f'{index}/{total}'
            )
            feedback.distance_to_target = float(
                distance
            )

            goal_handle.publish_feedback(
                feedback
            )

            time.sleep(0.05)

            # Safety timeout
            if time.monotonic() - turn_start > 15.0:

                self.stop_robot()

                break

        self.stop_robot()
        time.sleep(0.15)

        # =====================================================
        # PHASE 2: DRIVE TOWARDS WAYPOINT
        # =====================================================

        while rclpy.ok():

            if goal_handle.is_cancel_requested:

                self.stop_robot()
                return False, 0.0

            dx = target_x - self.x
            dy = target_y - self.y

            distance = math.hypot(
                dx,
                dy
            )

            # Waypoint reached
            if distance <= 0.20:

                self.stop_robot()

                return True, distance

            desired_yaw = math.atan2(
                dy,
                dx
            )

            yaw_error = normalize_angle(
                desired_yaw - self.yaw
            )

            cmd = Twist()

            # If heading error becomes large,
            # stop and turn again instead of spinning
            # while driving.
            if abs(yaw_error) > 0.30:

                cmd.linear.x = 0.0

                cmd.angular.z = clamp(
                    0.60 * yaw_error,
                    -0.10,
                    0.10
                )

            else:

                # Proportional forward speed
                cmd.linear.x = clamp(
                    0.25 * distance,
                    0.04,
                    0.12
                )

                # Small heading correction
                cmd.angular.z = clamp(
                    0.25 * yaw_error,
                    -0.04,
                    0.04
                )

            self.cmd_pub.publish(cmd)

            feedback.current_waypoint_index = index
            feedback.status = (
                f'En route to waypoint '
                f'{index}/{total}'
            )
            feedback.distance_to_target = float(
                distance
            )

            goal_handle.publish_feedback(
                feedback
            )

            time.sleep(0.05)

        self.stop_robot()

        return False, 0.0

    # ---------------------------------------------------------
    # LOAD MISSION
    # ---------------------------------------------------------

    def load_mission(self, filename):

        package_dir = get_package_share_directory(
            'waypoint_follower'
        )

        mission_path = os.path.join(
            package_dir,
            'missions',
            filename
        )

        with open(
            mission_path,
            'r'
        ) as file:

            return yaml.safe_load(file)

    # ---------------------------------------------------------
    # ACTION EXECUTION
    # ---------------------------------------------------------

    def execute_callback(self, goal_handle):

        result = Mission.Result()
        feedback = Mission.Feedback()

        self.get_logger().info(
            'Executing mission'
        )

        # -----------------------------------------------------
        # WAIT FOR ODOM
        # -----------------------------------------------------

        if not self.wait_for_odom(goal_handle):

            self.stop_robot()

            result.success = False
            result.total_distance = 0.0
            result.waypoints_completed = 0

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()

            return result

        # -----------------------------------------------------
        # LOAD YAML
        # -----------------------------------------------------

        try:

            mission = self.load_mission(
                goal_handle.request.mission_file
            )

        except Exception as error:

            self.get_logger().error(
                f'Failed to load mission: {error}'
            )

            self.stop_robot()

            result.success = False
            result.total_distance = 0.0
            result.waypoints_completed = 0

            goal_handle.abort()

            return result

        base = mission.get(
            'base',
            {'x': 0.0, 'y': 0.0}
        )

        waypoints = mission.get(
            'waypoints',
            []
        )

        return_to_base = mission.get(
            'return_to_base',
            False
        )

        points = []

        for waypoint in waypoints:

            points.append(
                (
                    float(waypoint['x']),
                    float(waypoint['y'])
                )
            )

        if return_to_base:

            points.append(
                (
                    float(base['x']),
                    float(base['y'])
                )
            )

        total = len(points)

        completed = 0
        total_distance = 0.0

        # -----------------------------------------------------
        # FOLLOW WAYPOINTS
        # -----------------------------------------------------

        for i, point in enumerate(points):

            target_x = point[0]
            target_y = point[1]

            index = i + 1

            self.get_logger().info(
                f'Going to waypoint '
                f'{index}/{total}: '
                f'({target_x}, {target_y})'
            )

            start_x = self.x
            start_y = self.y

            success, remaining = (
                self.move_to_waypoint(
                    goal_handle,
                    target_x,
                    target_y,
                    index,
                    total,
                    feedback
                )
            )

            if not success:

                self.stop_robot()

                result.success = False
                result.total_distance = float(
                    total_distance
                )
                result.waypoints_completed = int(
                    completed
                )

                if goal_handle.is_cancel_requested:

                    goal_handle.canceled()

                else:

                    goal_handle.abort()

                return result

            end_x = self.x
            end_y = self.y

            segment_distance = math.hypot(
                end_x - start_x,
                end_y - start_y
            )

            total_distance += segment_distance

            completed += 1

            self.stop_robot()

            time.sleep(0.25)

        # -----------------------------------------------------
        # SUCCESS
        # -----------------------------------------------------

        self.stop_robot()

        result.success = True
        result.total_distance = float(
            total_distance
        )
        result.waypoints_completed = int(
            completed
        )

        goal_handle.succeed()

        self.get_logger().info(
            'Mission completed successfully'
        )

        return result


def main(args=None):

    rclpy.init(args=args)

    node = WaypointFollower()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
