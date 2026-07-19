#!/usr/bin/env python3

"""
homing.py
Author: Harsh Wadibhasme (harsh.wadibhasme@addverb.com)
Brief: Homing script that automatically switches to ptp_joint_controller,
       sends the robot to home position (all zeros), then restores the
       previously active controller.
Version: 0.3
Date: 2025-09-06

Copyright (c) 2025
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController


# Controllers that are managed by this system (excludes joint_state_broadcaster etc.)
MANAGED_CONTROLLERS = {
    'velocity_controller',
    'effort_controller',
    'gravity_comp_effort_controller',
    'free_drive_controller',
    'recorder_controller',
    'ptp_joint_controller',
    'ptp_tcp_controller',
    'joint_jogging_controller',
    'cartesian_jogging_controller',
    'joint_impedance_controller',
    'cartesian_impedance_controller',
}

HOMING_CONTROLLER = 'ptp_joint_controller'


class HomingClient(Node):

    def __init__(self):
        super().__init__('homing_client')

        # Use a separate callback group for service clients so they
        # don't deadlock with the timer callback.
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # Service clients for controller management
        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=self.service_cb_group
        )
        self.switch_controller_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller',
            callback_group=self.service_cb_group
        )

        # Action client for ptp_joint_controller
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/ptp_joint_controller/follow_joint_trajectory'
        )

        self.previously_active_controllers = []
        self.once = False
        self.multi_point_target = True  # Change this to False for single point

        # Start the homing sequence on a separate thread to avoid
        # the spin_until_future_complete deadlock.
        self.homing_thread = threading.Thread(
            target=self._run_homing_sequence, daemon=True
        )
        self.homing_thread.start()
        self.get_logger().info("HomingClient node initialized.")

    # ------------------------------------------------------------------ #
    #  Controller management helpers
    # ------------------------------------------------------------------ #

    def get_active_controllers(self):
        """Query controller_manager for currently active controllers.
        Returns a list of controller names that are in 'active' state
        and belong to MANAGED_CONTROLLERS."""
        if not self.list_controllers_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'ListControllers service not available.'
            )
            return []

        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('Failed to call ListControllers service.')
            return []

        active = []
        for ctrl in future.result().controller:
            if ctrl.state == 'active' and ctrl.name in MANAGED_CONTROLLERS:
                active.append(ctrl.name)
                self.get_logger().info(
                    f"  Found active managed controller: {ctrl.name}"
                )

        return active

    def switch_controllers(self, activate, deactivate):
        """Call the SwitchController service.
        activate  : list of controller names to activate
        deactivate: list of controller names to deactivate
        Returns True on success, False on failure."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'SwitchController service not available.'
            )
            return False

        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.BEST_EFFORT  # 1
        request.start_asap = False
        request.activate_asap = False
        request.timeout = Duration(sec=0, nanosec=0)

        future = self.switch_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('Failed to call SwitchController service.')
            return False

        if future.result().ok:
            self.get_logger().info(
                f"Controller switch OK — activated: {activate}, "
                f"deactivated: {deactivate}"
            )
        else:
            self.get_logger().error(
                f"Controller switch FAILED — activate: {activate}, "
                f"deactivate: {deactivate}"
            )

        return future.result().ok

    # ------------------------------------------------------------------ #
    #  Homing sequence  (runs on a background thread)
    # ------------------------------------------------------------------ #

    def _run_homing_sequence(self):
        """Entry point for the homing thread. Waits briefly for the
        executor to start spinning, then runs the homing steps."""
        # Give the executor a moment to start spinning the node
        import time
        time.sleep(1.0)
        self.start_homing_sequence()

    def start_homing_sequence(self):
        """Entry point: discover active controllers, switch to
        ptp_joint_controller, then send the homing goal."""
        if self.once:
            return
        self.once = True

        # --- Step 1: Read currently active controllers ---
        self.get_logger().info("Step 1: Reading currently active controllers...")
        self.previously_active_controllers = self.get_active_controllers()

        if not self.previously_active_controllers:
            self.get_logger().warn(
                "No active managed controllers found. "
                "Will activate ptp_joint_controller directly."
            )

        # --- Step 2: Switch to ptp_joint_controller ---
        need_switch = True
        deactivate_list = [
            c for c in self.previously_active_controllers
            if c != HOMING_CONTROLLER
        ]
        activate_list = (
            [HOMING_CONTROLLER]
            if HOMING_CONTROLLER not in self.previously_active_controllers
            else []
        )

        if not deactivate_list and not activate_list:
            self.get_logger().info(
                "ptp_joint_controller is already the only active controller."
            )
            need_switch = False

        if need_switch:
            self.get_logger().info(
                f"Step 2: Switching controllers — "
                f"activating: {activate_list}, deactivating: {deactivate_list}"
            )
            success = self.switch_controllers(activate_list, deactivate_list)
            if not success:
                self.get_logger().error(
                    "Failed to switch to ptp_joint_controller. Aborting."
                )
                rclpy.shutdown()
                return

        # --- Step 3: Send homing goal ---
        self.get_logger().info("Step 3: Sending homing goal (all joints → 0.0)...")
        self.send_homing_goal()

    def send_homing_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            self.restore_controllers()
            rclpy.shutdown()
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        if self.multi_point_target:
            points = [
                ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 10)
            ]
            for pos, sec in points:
                pt = JointTrajectoryPoint()
                pt.positions = pos
                pt.time_from_start = Duration(sec=sec, nanosec=0)
                goal_msg.trajectory.points.append(pt)
        else:
            pt = JointTrajectoryPoint()
            pt.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pt.time_from_start = Duration(sec=5, nanosec=0)
            goal_msg.trajectory.points.append(pt)

        self.get_logger().info("Sending goal...")
        self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    # ------------------------------------------------------------------ #
    #  Action callbacks
    # ------------------------------------------------------------------ #

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.restore_controllers()
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Homing complete — result received.")

        # --- Step 4: Restore previously active controllers ---
        self.restore_controllers()
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.joint_names and feedback.actual.positions:
            pos_str = ', '.join(f"{p:.3f}" for p in feedback.actual.positions)
            self.get_logger().info(f"Feedback - Actual positions: [{pos_str}]")

    # ------------------------------------------------------------------ #
    #  Restore previous controllers
    # ------------------------------------------------------------------ #

    def restore_controllers(self):
        """Reactivate the controllers that were active before homing."""
        restore_list = [
            c for c in self.previously_active_controllers
            if c != HOMING_CONTROLLER
        ]

        if not restore_list:
            self.get_logger().info(
                "No previous controllers to restore "
                "(ptp_joint_controller remains active)."
            )
            return

        deactivate_homing = (
            [HOMING_CONTROLLER]
            if HOMING_CONTROLLER not in self.previously_active_controllers
            else []
        )

        self.get_logger().info(
            f"Step 4: Restoring controllers — "
            f"activating: {restore_list}, deactivating: {deactivate_homing}"
        )
        success = self.switch_controllers(restore_list, deactivate_homing)
        if success:
            self.get_logger().info("Previous controllers restored successfully.")
        else:
            self.get_logger().error(
                "Failed to restore previous controllers. "
                "Manual intervention may be required."
            )


def main(args=None):
    rclpy.init(args=args)
    node = HomingClient()

    # Use a MultiThreadedExecutor so that the service calls from the
    # background thread can be processed concurrently with action
    # callbacks and other subscriptions.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
