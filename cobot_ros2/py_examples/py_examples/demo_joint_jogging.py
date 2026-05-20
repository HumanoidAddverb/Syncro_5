#!/usr/bin/env python3

"""
@file demo_joint_jogging.py
@author Siddhi Jain
@brief Demo on how to use JointJoggingController
@version 0.1
@date 2025-07-07
"""

import rclpy

from rclpy.node import Node

from addverb_cobot_msgs.msg import JointJoggingVelocity


class JointJoggingPublisher(Node):

    def __init__(self):

        super().__init__("demo_joint_jogging")

        #
        # Create publisher for joint jogging commands
        #
        self.publisher = self.create_publisher(
            JointJoggingVelocity,
            "/joint_jogging_controller/joint_jogging/command",
            10
        )

        #
        # Declare parameters
        #
        self.declare_parameter("num_joints", 6)

        self.declare_parameter("speed_factor", 1.0)

        self.num_joints = (
            self.get_parameter("num_joints")
            .get_parameter_value()
            .integer_value
        )

        #
        # Initialize jogging sequence variables
        #
        self.current_joint = 0

        self.going_forward = True

        #
        # Timer to publish jogging commands
        #
        self.timer = self.create_timer(
            0.5,   # 500 ms
            self.timer_callback
        )

        #
        # Timer to switch active joint
        #
        self.joint_timer = self.create_timer(
            3.0,   # 3000 ms
            self.switch_joint
        )

        self.get_logger().info(
            "Starting with joint 0, "
            "will switch every 3 seconds"
        )

    #
    # Switch to next joint
    #
    def switch_joint(self):

        if self.going_forward:

            self.current_joint += 1

            if self.current_joint > self.num_joints - 1:

                self.current_joint = self.num_joints - 1

                self.going_forward = False

        else:

            self.current_joint -= 1

            if self.current_joint < 0:

                self.current_joint = 0

                self.joint_timer.cancel()

                self.get_logger().info(
                    "Demo Complete!"
                )

                rclpy.shutdown()

                return

        self.get_logger().info(
            f"Switching to joint {self.current_joint}"
        )

    #
    # Publish jogging command
    #
    def timer_callback(self):

        msg = JointJoggingVelocity()

        #
        # Initialize all joints to zero velocity
        #
        msg.jvel_scaling_factor = [
            0.0
        ] * self.num_joints

        #
        # Get speed factor parameter
        #
        speed_factor = (
            self.get_parameter("speed_factor")
            .get_parameter_value()
            .double_value
        )

        #
        # Reverse direction if moving backward
        #
        if not self.going_forward:

            speed_factor = -speed_factor

        #
        # Set velocity for current joint
        #
        msg.jvel_scaling_factor[
            self.current_joint
        ] = speed_factor

        #
        # Publish message
        #
        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = JointJoggingPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()