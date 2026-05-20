#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from addverb_cobot_msgs.action import FollowCartesianTrajectory
from addverb_cobot_msgs.msg import CartesianTrajectoryPoint


class PTPTCPTestClient(Node):

    def __init__(self):

        super().__init__("demo_ptp_tcp_client")

        self.client = ActionClient(
            self,
            FollowCartesianTrajectory,
            "/ptp_tcp_controller/follow_cartesian_trajectory"
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

        goal_msg = FollowCartesianTrajectory.Goal()

        #
        # Create trajectory points
        #
        points = []

        #
        # Point 1
        #
        pt = CartesianTrajectoryPoint()

        pt.point.position.x = 0.00095306
        pt.point.position.y = 0.468059
        pt.point.position.z = 0.5385

        pt.point.orientation.x = 0.0
        pt.point.orientation.y = 0.0
        pt.point.orientation.z = 0.0

        pt.time_from_start = 1.5

        points.append(pt)

        #
        # Point 2
        #
        pt = CartesianTrajectoryPoint()

        pt.point.position.x = 0.00095306
        pt.point.position.y = 0.468059
        pt.point.position.z = 0.4385

        pt.point.orientation.x = 0.0
        pt.point.orientation.y = 0.0
        pt.point.orientation.z = 0.0

        pt.time_from_start = 15.05

        points.append(pt)

        #
        # Point 3
        #
        pt = CartesianTrajectoryPoint()

        pt.point.position.x = -0.399047
        pt.point.position.y = 0.468059
        pt.point.position.z = 0.4385

        pt.point.orientation.x = 0.0
        pt.point.orientation.y = 0.0
        pt.point.orientation.z = 0.0

        pt.time_from_start = 30.05

        points.append(pt)

        #
        # Point 4
        #
        pt = CartesianTrajectoryPoint()

        pt.point.position.x = 0.00095306
        pt.point.position.y = 0.468059
        pt.point.position.z = 0.4385

        pt.point.orientation.x = 0.0
        pt.point.orientation.y = 0.0
        pt.point.orientation.z = 0.0

        pt.time_from_start = 45.05

        points.append(pt)

        #
        # Point 5
        #
        pt = CartesianTrajectoryPoint()

        pt.point.position.x = -0.399047
        pt.point.position.y = 0.468059
        pt.point.position.z = 0.4385

        pt.point.orientation.x = 0.0
        pt.point.orientation.y = 0.0
        pt.point.orientation.z = 0.0

        pt.time_from_start = 60.05

        points.append(pt)

        #
        # Assign points to trajectory
        #
        goal_msg.trajectory.points = points

        self.get_logger().info(
            "Sending Cartesian trajectory goal..."
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

            self.get_logger().info(
                "Feedback received"
            )


def main(args=None):

    rclpy.init(args=args)

    node = PTPTCPTestClient()

    rclpy.spin(node)

    node.destroy_node()



if __name__ == "__main__":

    main()