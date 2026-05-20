#!/usr/bin/env python3

import time

import rclpy

from rclpy.node import Node

from addverb_cobot_msgs.srv import Gripper as GripperSrv


class GripperClient(Node):

    def __init__(self):

        super().__init__("demo_gripper")

        self.client = self.create_client(
            GripperSrv,
            "/gripper_controller/command"
        )

        self.get_logger().info(
            "Waiting for gripper service..."
        )

        while not self.client.wait_for_service(timeout_sec=1.0):

            self.get_logger().info(
                "Gripper service not available, waiting again..."
            )

        self.get_logger().info(
            "Connected to gripper service. Starting loop..."
        )

        self.run_loop()

    def run_loop(self):

        open_gripper = False

        force = 50.0

        while rclpy.ok():

            request = GripperSrv.Request()

            request.position = 1.0 if open_gripper else 0.0

            request.grasp_force = force

            action = "Opening" if open_gripper else "Closing"

            self.get_logger().info(
                f"{action} gripper "
                f"(pos={request.position:.2f}, "
                f"force={request.grasp_force:.2f})..."
            )

            # Send async request
            future = self.client.call_async(request)

            # Wait for response
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:

                response = future.result()

                if response.success:

                    self.get_logger().info(
                        f"Gripper {action} successful: "
                        f"{response.message}"
                    )

                else:

                    self.get_logger().warn(
                        f"Gripper {action} failed: "
                        f"{response.message}"
                    )

            else:

                self.get_logger().error(
                    "Failed to call gripper service"
                )

            # Toggle gripper state
            open_gripper = not open_gripper

            # Wait 3 seconds
            time.sleep(3.0)


def main(args=None):

    rclpy.init(args=args)

    node = GripperClient()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()