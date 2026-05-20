#!/usr/bin/env python3


import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CartesianJoggingPublisher(Node):

    def __init__(self):

        super().__init__("demo_cartesian_jogging")

        self.update_rate = 30.0
        self.duration_sec = 2.0

        # Create publisher for cartesian jogging
        self.publisher = self.create_publisher(
            Twist,
            "/cartesian_jogging_controller/cartesian_jogging/command",
            10
        )

    def publish_motion(self, msg, duration):

        start_time = time.time()

        while (
            (time.time() - start_time) < duration
            and rclpy.ok()
        ):

            self.publisher.publish(msg)

            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

            time.sleep(
                1.0 / self.update_rate
            )

    def perform_demo(self):

        msg = Twist()

        self.get_logger().info("Starting linear motions...")

        # +X
        self.get_logger().info(
            f"Linear +X for {self.duration_sec:.2f} sec..."
        )

        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publish_motion(msg, self.duration_sec)

        # -X
        self.get_logger().info(
            f"Linear -X for {self.duration_sec:.2f} sec..."
        )

        msg.linear.x = -1.0

        self.publish_motion(msg, self.duration_sec)

        # +Y
        self.get_logger().info(
            f"Linear +Y for {self.duration_sec:.2f} sec..."
        )

        msg.linear.x = 0.0
        msg.linear.y = 1.0

        self.publish_motion(msg, self.duration_sec)

        # -Y
        self.get_logger().info(
            f"Linear -Y for {self.duration_sec:.2f} sec..."
        )

        msg.linear.y = -1.0

        self.publish_motion(msg, self.duration_sec)

        # +Z
        self.get_logger().info(
            f"Linear +Z for {self.duration_sec:.2f} sec..."
        )

        msg.linear.y = 0.0
        msg.linear.z = 1.0

        self.publish_motion(msg, self.duration_sec)

        # -Z
        self.get_logger().info(
            f"Linear -Z for {self.duration_sec:.2f} sec..."
        )

        msg.linear.z = -1.0

        self.publish_motion(msg, self.duration_sec)

        self.get_logger().info("Starting angular motions...")

        # +Angular X
        self.get_logger().info(
            f"Angular +X for {self.duration_sec:.2f} sec..."
        )

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 1.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publish_motion(msg, self.duration_sec)

        # -Angular X
        self.get_logger().info(
            f"Angular -X for {self.duration_sec:.2f} sec..."
        )

        msg.angular.x = -1.0

        self.publish_motion(msg, self.duration_sec)

        # +Angular Y
        self.get_logger().info(
            f"Angular +Y for {self.duration_sec:.2f} sec..."
        )

        msg.angular.x = 0.0
        msg.angular.y = 1.0

        self.publish_motion(msg, self.duration_sec)

        # -Angular Y
        self.get_logger().info(
            f"Angular -Y for {self.duration_sec:.2f} sec..."
        )

        msg.angular.y = -1.0

        self.publish_motion(msg, self.duration_sec)

        # +Angular Z
        self.get_logger().info(
            f"Angular +Z for {self.duration_sec:.2f} sec..."
        )

        msg.angular.y = 0.0
        msg.angular.z = 1.0

        self.publish_motion(msg, self.duration_sec)

        # -Angular Z
        self.get_logger().info(
            f"Angular -Z for {self.duration_sec:.2f} sec..."
        )

        msg.angular.z = -1.0

        self.publish_motion(msg, self.duration_sec)

        self.get_logger().info("Motion sequence complete.")

        # Stop motion
        stop_msg = Twist()

        self.publisher.publish(stop_msg)


def main(args=None):

    rclpy.init(args=args)

    node = CartesianJoggingPublisher()

    node.perform_demo()

    node.destroy_node()



if __name__ == "__main__":
    main()