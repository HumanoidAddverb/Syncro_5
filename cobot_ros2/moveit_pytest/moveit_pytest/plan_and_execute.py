"""
moveit_test.py
Author: Harsh Wadibhasme (harsh.wadibhasme@addverb.com)
Brief: plans a trajectory and executes it from to a randomly generaged position using moveit.
Version: 0.1
Date: 2025-09-19

Copyright (c) 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory

import numpy as np
import math
import time
import sys
import select

#  CONTROLLER : ptp_joint_controller

class MoveItPlanExecuteClient(Node): 
    def __init__(self):
        super().__init__('moveit_plan_execute_client')

        # Planning service client
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /plan_kinematic_path service...')
        self.get_logger().info('/plan_kinematic_path service available!')

        # Execution action client
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        while not self.exec_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for /execute_trajectory action server...')
        self.get_logger().info('/execute_trajectory action server available!')

    def plan_to_joint_positions(self, joint_positions, group_name='heal'):
        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = group_name

        joint_constraints = []
        for joint_name, position in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints

        motion_plan_request.goal_constraints.append(goal_constraints)
        request.motion_plan_request = motion_plan_request

        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.motion_plan_response.error_code.val == 1:
                self.get_logger().info('Planning successful!')
                return response.motion_plan_response.trajectory
            else:
                self.get_logger().error(f'Planning failed with error code: {response.motion_plan_response.error_code.val}')
        else:
            self.get_logger().error('Service call failed')
        return None

    def execute_trajectory(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_goal_future = self.exec_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            return False

        self.get_logger().info('Goal accepted, executing trajectory...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        if result.status == 4:
            self.get_logger().error('Execution aborted.')
            return False
        elif result.status == 5:
            self.get_logger().error('Execution rejected.')
            return False
        else:
            self.get_logger().info('Execution succeeded!')
            return True


def main(args=None):

    joint_limits={0:(-math.pi/4,math.pi/4),
                1:(0,math.pi/10),
                2:(-math.pi/6,math.pi/10),
                3:(-math.pi/3,math.pi/6),
                4:(-math.pi/3,math.pi/3),
                5:(-math.pi/4,math.pi/4)}
    
    rclpy.init(args=args)
    node = MoveItPlanExecuteClient()

    while True:

        random_jpos = np.random.uniform(-math.pi/2, math.pi/2, size=6) # generates randon points in radians

        target_positions = {
            'joint1': np.clip(random_jpos[0],joint_limits[0][0],joint_limits[0][1]),
            'joint2': np.clip(random_jpos[1],joint_limits[1][0],joint_limits[1][1]),
            'joint3': np.clip(random_jpos[2],joint_limits[2][0],joint_limits[2][1]),
            'joint4': np.clip(random_jpos[3],joint_limits[3][0],joint_limits[3][1]),
            'joint5': np.clip(random_jpos[4],joint_limits[4][0],joint_limits[4][1]),
            'joint6': np.clip(random_jpos[5],joint_limits[5][0],joint_limits[5][1]),
        }

        trajectory = node.plan_to_joint_positions(target_positions, group_name='heal')

        if trajectory:
            node.get_logger().info(f'Received trajectory with {len(trajectory.joint_trajectory.points)} points')

            success = node.execute_trajectory(trajectory)

            if success:
                node.get_logger().info('Trajectory execution completed successfully.')
            else:
                node.get_logger().error('Trajectory execution failed.')
        else:
            node.get_logger().error('No trajectory received; aborting execution.')

        node.get_logger().info('press "q" to exit')
        if select.select([sys.stdin], [], [], 0)[0]:
            inp = sys.stdin.readline().strip()
            if inp.lower() == 'q':
                print("Detected 'q', exiting")
                break

        time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
