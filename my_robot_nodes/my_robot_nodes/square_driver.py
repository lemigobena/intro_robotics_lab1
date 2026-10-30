#!/usr/bin/env python3
"""Action client that drives the robot around a square."""

import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

        # Start the sequence in a background worker thread
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def run(self):
        side_length = float(self.get_parameter("side_length").value)
        quarter_turn = math.pi / 2.0

        self.get_logger().info("Waiting for /drive_distance action server...")
        self.drive_client.wait_for_server()
        self.get_logger().info("Waiting for /rotate action server...")
        self.rotate_client.wait_for_server()

        self.get_logger().info(f"Starting square trajectory with side length {side_length:.2f} m")

        for side in range(1, 5):
            self.get_logger().info(f"--- Side {side} of 4: Driving {side_length:.2f} m ---")
            drive_success = self.drive(side_length)
            if not drive_success:
                self.get_logger().error(f"Driving failed on side {side}, stopping sequence.")
                return

            self.get_logger().info(f"--- Side {side} of 4: Turning quarter turn left (90 deg) ---")
            rotate_success = self.rotate(quarter_turn)
            if not rotate_success:
                self.get_logger().error(f"Rotation failed on side {side}, stopping sequence.")
                return

        self.get_logger().info("Square driving sequence completed successfully!")

    def drive(self, distance):
        goal = DriveDistance.Goal()
        goal.distance = distance

        event = threading.Event()
        outcome = {"success": False}

        def on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Drive goal was rejected by server")
                event.set()
                return

            result_future = goal_handle.get_result_async()

            def on_result(res_future):
                status = res_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    outcome["success"] = True
                event.set()

            result_future.add_done_callback(on_result)

        send_future = self.drive_client.send_goal_async(goal)
        send_future.add_done_callback(on_goal_response)
        event.wait()
        return outcome["success"]

    def rotate(self, angle):
        goal = Rotate.Goal()
        goal.angle = angle

        event = threading.Event()
        outcome = {"success": False}

        def on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Rotate goal was rejected by server")
                event.set()
                return

            result_future = goal_handle.get_result_async()

            def on_result(res_future):
                status = res_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    outcome["success"] = True
                event.set()

            result_future.add_done_callback(on_result)

        send_future = self.rotate_client.send_goal_async(goal)
        send_future.add_done_callback(on_goal_response)
        event.wait()
        return outcome["success"]


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
