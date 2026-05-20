#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class CartesianImpedanceTestClient(Node):

    def __init__(self):
        super().__init__("cartesian_impedance_test_client")

        self.sent = False

        self.get_logger().info(
            "Cartesian Impedance Test Client Node Initialized"
        )

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/cartesian_impedance_controller/follow_joint_trajectory"
        )

        self.set_controller_parameters()

        self.timer = self.create_timer(
            0.5,
            self.send_goal
        )

    def set_controller_parameters(self):

        stiffness_values = [1500.0, 1000.0, 2000.0, 250.0, 250.0, 250.0]
        damping_values = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
        mass_matrix_values = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
        ft_force_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target_force_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.declare_parameter(
            "cartesian_impedance_controller.stiffness",
            stiffness_values
        )

        self.declare_parameter(
            "cartesian_impedance_controller.damping",
            damping_values
        )

        self.declare_parameter(
            "cartesian_impedance_controller.mass_matrix",
            mass_matrix_values
        )

        self.declare_parameter(
            "cartesian_impedance_controller.ft_force",
            ft_force_values
        )

        self.declare_parameter(
            "cartesian_impedance_controller.target_force",
            target_force_values
        )

        self.get_logger().info(
            "Controller parameters declared (local to this node)."
        )

    def send_goal(self):

        self.timer.cancel()

        if self.sent:
            return

        self.get_logger().info("Waiting for action server...")

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            rclpy.shutdown()
            return

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6"
        ]

        pt = JointTrajectoryPoint()

        pt.positions = [0.4, 0.0, 0.0, 0.0, 0.0, 0.1]

        pt.time_from_start = Duration(
            seconds=5.0
        ).to_msg()

        goal_msg.trajectory.points.append(pt)

        self.get_logger().info("Sending goal...")

        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

        self.sent = True

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result().result

        self.get_logger().info("Result received")

        self.get_logger().info(str(result))

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        if feedback:

            positions = feedback.actual.positions

            pos_string = ", ".join(
                [str(p) for p in positions]
            )

            self.get_logger().info(
                f"Actual positions: {pos_string}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = CartesianImpedanceTestClient()

    rclpy.spin(node)

    node.destroy_node()



if __name__ == "__main__":
    main()