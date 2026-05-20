#!/usr/bin/env python3

import time

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class VelocityPublisher(Node):

    def __init__(self):

        super().__init__("demo_velocity")

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/velocity_controller/commands",
            10
        )

        self.update_rate = 30.0

    #
    # Publish velocity for duration
    #
    def run_velocity(self, velocity, duration):

        self.get_logger().info(
            f"Velocity = {velocity:.2f} "
            f"for {duration:.2f} sec..."
        )

        start_time = time.time()

        while (
            (time.time() - start_time) < duration
            and rclpy.ok()
        ):

            msg = Float64MultiArray()

            msg.data = [
                velocity,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ]

            self.publisher.publish(msg)

            rclpy.spin_once(
                self,
                timeout_sec=0.0
            )

            time.sleep(
                1.0 / self.update_rate
            )

    #
    # Demo sequence
    #
    def change_velocity(self):

        self.get_logger().info(
            "Starting demo..."
        )

        self.run_velocity(0.01, 11.0)

        self.run_velocity(0.0, 0.1)

        self.run_velocity(-0.03, 5.0)

        self.run_velocity(0.0, 0.5)

        self.run_velocity(0.06, 9.0)

        self.run_velocity(0.0, 0.8)

        self.run_velocity(-0.1, 5.0)

        self.run_velocity(0.0, 0.8)

        self.get_logger().info(
            "Demo Complete!"
        )


def main(args=None):

    rclpy.init(args=args)

    node = VelocityPublisher()

    node.change_velocity()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()