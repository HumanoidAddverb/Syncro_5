#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


MULTI_POINT_TARGET = True


class DemoClient(Node):

    def __init__(self):

        super().__init__("demo_ptp_client")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/ptp_joint_controller/follow_joint_trajectory"
        )

        self.once = False

        self.timer = self.create_timer(
            0.5,
            self.send_goal
        )

    def send_goal(self):

        self.timer.cancel()

        if self.once:
            return

        self.get_logger().info(
            "Waiting for action server..."
        )

        if not self.client.wait_for_server(timeout_sec=5.0):

            self.get_logger().error(
                "Action server not available"
            )

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

        #
        # Multi-point trajectory
        #
        if MULTI_POINT_TARGET:

            trajectory_points = [
                ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 4.0),
                ([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], 8.0),
                ([0.5, 0.0, 0.3, 0.0, 0.0, 0.0], 12.0),
                ([-0.5, 0.0, 0.3, 0.0, 0.0, 0.0], 18.0),
                ([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0], 22.0),
                ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 25.0),
            ]

            for positions, time_sec in trajectory_points:

                pt = JointTrajectoryPoint()

                pt.positions = positions

                pt.time_from_start = Duration(
                    seconds=time_sec
                ).to_msg()

                goal_msg.trajectory.points.append(pt)

        #
        # Single-point trajectory
        #
        else:

            pt = JointTrajectoryPoint()

            pt.positions = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ]

            pt.time_from_start = Duration(
                seconds=5.0
            ).to_msg()

            goal_msg.trajectory.points.append(pt)

        self.get_logger().info(
            "Sending trajectory goal..."
        )

        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

        self.once = True

        print("sent goal once")

    #
    # Goal response callback
    #
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                "Goal rejected"
            )

            rclpy.shutdown()

            return

        self.get_logger().info(
            "Goal accepted"
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    #
    # Result callback
    #
    def result_callback(self, future):

        _result = future.result().result

        self.get_logger().info(
            "Result received"
        )

        rclpy.shutdown()

    #
    # Feedback callback
    #
    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        if feedback:

            if feedback.actual.positions:

                pos_string = ", ".join(
                    [str(p) for p in feedback.actual.positions]
                )

                self.get_logger().info(
                    f"Feedback: Actual positions: "
                    f"[{pos_string}]"
                )


def main(args=None):

    rclpy.init(args=args)

    node = DemoClient()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()