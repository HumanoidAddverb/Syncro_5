"""
moveit_test.py
Author: Harsh Wadibhasme (harsh.wadibhasme@addverb.com)
Brief: plans a trajectory from the current position to the position given using moveit.
Version: 0.1
Date: 2025-09-19

Copyright (c) 2025
"""

import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

class MoveItPlanClient(Node):
    def __init__(self):
        super().__init__('moveit_plan_client')
        self.client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /plan_kinematic_path service...')
        self.get_logger().info('/plan_kinematic_path service available!')

    def plan_to_joint_positions(self, joint_positions, group_name='heal'):
        """
        joint_positions: dict {joint_name: position_in_radians}
        group_name: name of MoveIt planning group
        """
        request = GetMotionPlan.Request()
        
        # Create MotionPlanRequest
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = group_name
        
        # Build goal constraints from joint_positions
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
        
        # Call the service
        future = self.client.call_async(request)
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

def main(args=None):
    rclpy.init(args=args)
    node = MoveItPlanClient()
    
    # Example target joint positions (radians)
    target_positions = {
        'joint1': 0.2,
        'joint2': -0.1,
        'joint3': 0.1,
        'joint4': 0.0,
        'joint5': 0.01,
        'joint6': -0.1,
    }
    
    trajectory = node.plan_to_joint_positions(target_positions, group_name='heal')
    
    if trajectory:
        node.get_logger().info(f'Received trajectory with {len(trajectory.joint_trajectory.points)} points')
    else:
        node.get_logger().error('No trajectory received.')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
