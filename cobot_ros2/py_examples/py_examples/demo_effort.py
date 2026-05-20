#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class EffortPublisher(Node):

    def __init__(self):

        super().__init__("demo_effort")

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/effort_controller/commands",
            10
        )

        self.timer = self.create_timer(
            0.1,   # 100 ms
            self.timer_callback
        )

    def timer_callback(self):

        effort = Float64MultiArray()

        effort.data = [
            0.0,
            -22.0,
            22.0,
            0.0,
            0.0,
            0.0
        ]

        self.get_logger().info(
            "Publishing joint efforts"
        )

        self.publisher.publish(effort)


def main(args=None):

    rclpy.init(args=args)

    node = EffortPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()