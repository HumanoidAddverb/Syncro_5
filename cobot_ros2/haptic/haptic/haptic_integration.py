#!/usr/bin/env python3
"""
@file haptic_integration.py
@brief example script of how haptic device can be used to control the cobot
@description this code uses cartesian jogging controller to controll the robot the
             commands taken from the haptic device and ft sensor (ft sensor for force control) 
@version 0.1
@date 2025-11-4
"""

import time
import math
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
from std_msgs.msg import Float64MultiArray
from addverb_cobot_msgs.msg import CartesianPoint
from haptic_pkg.msg import ReadData, WriteData # from haptic ROS2
from addverb_cobot_msgs.srv import Gripper
from haptic.haptic_config import cart, rot , MAX_FORCE_LIMIT, MAX_TORQUE_LIMIT
from haptic.haptic_defs import rot_mat

scale_fz=1.0
scale_fxy=1.0
scale_j=1.0
force_val=1.0
def get_scale():
    global scale_j, scale_fz, scale_fxy, force_val
    while True:
        val = input("enter four values : ").split()
        scale_j = float(val[0])
        scale_fz = float(val[1])
        scale_fxy = float(val[2])
        force_val = float(val[3])

threading.Thread(target=get_scale, daemon=True).start()


class DemoRPYJog(Node):
    def __init__(self):
        super().__init__('demo_rpy_twist_dynamic_duration')

        self.publisher_ = self.create_publisher(
            Twist,
            '/cartesian_jogging_controller/cartesian_jogging/command',
            10
        )

        self.subscription_ = self.create_subscription(
            ReadData,
            '/hap/motion_data',
            self.hapdata_callback,
            10
        )

        self.subscription = self.create_subscription(
            CartesianPoint,
            '/ee_pos_data',
            self.ee_pos_callback,
            10)

        self.hap_force_pub = self.create_publisher(
            WriteData,
            '/hap/force_data',
            10
        )

        self.ft_data_sub = self.create_subscription(
            Wrench,
            '/ft_data',
            self.get_ft_callback,
            10
        )

        # Gripper client setup
        self.gripper_client = self.create_client(Gripper, '/gripper_controller/command')
        while not self.gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /**gripper_controller/command service...")

        self.get_logger().info("Connected to gripper service!")

        self.update_rate = 50.0
        self.latch_count = 200
        self.latest_hap_data = None
        self.latest_ft_value = None  # for /ft_data
        self.rotated_ft= None

        self.linear_dir = np.zeros(3)
        self.angular_dir = np.zeros(3)

        self.gripper_open = True
        self.gripper_force = 50.0
        self.start_time_b = 0
        self.time_thresh_hold = 1
        self.jog_val=1*scale_j
        self.jog_val_f=1
        self.jogxy= 1
        self.jogz =1
        self.calib_count=0
        self.fx_buffer= []
        self.fy_buffer= []
        self.fz_buffer= []

        self.weight=1.4*10
        self.com_distance=0.07
        self.grav= np.array([0,0,-1])
        self.ori = np.zeros(3)

        self.ft_offsets = np.zeros(6)
        self.raw_ft_value= np.zeros(6)

        self.cobot_force_rot = rot_mat(0, math.pi, 0)  # Rotation matrix for force transformation
        self.hap_force_rot=rot_mat(0, 0, -math.pi/4)

        self.get_logger().info("Node initialized successfully!")

    def ee_pos_callback(self, msg):
        self.ori[0] = msg.orientation.x
        self.ori[1] = msg.orientation.y
        self.ori[2] = msg.orientation.z

    def get_ft_offsets(self):
        self.ft_offsets=self.raw_ft_value
        self.get_logger().info(f"ft value set ")


    # Subscriber Callbacks
    def hapdata_callback(self, msg: ReadData):
        """Receive and store latest haptic input data."""
        self.latest_hap_data = msg

    def get_ft_callback(self, msg: Wrench):
        """Receive and store latest force-torque data (Wrench type)."""
        # Wrench message has:
        # force.x,y,z  and torque.x,y,z

        self.latest_ft_value = [
            msg.force.x,
            msg.force.y,
            msg.force.z,
            msg.torque.x,
            msg.torque.y,
            msg.torque.z
        ]

        buffer_size= 1 # increasing this will make the force control responce slower
        if self.calib_count < buffer_size :
            self.fx_buffer.append(msg.force.x)
            self.fy_buffer.append(msg.force.y)
            self.fz_buffer.append(msg.force.z)
            self.calib_count+=1
        else:
            self.fx_buffer.pop(0)
            self.fy_buffer.pop(0)
            self.fz_buffer.pop(0)
            self.fx_buffer.append(msg.force.x)
            self.fy_buffer.append(msg.force.y)
            self.fz_buffer.append(msg.force.z)
            self.latest_ft_value[0]=sum(self.fx_buffer)/buffer_size
            self.latest_ft_value[1]=sum(self.fy_buffer)/buffer_size
            self.latest_ft_value[2]=sum(self.fz_buffer)/buffer_size

        ft_data_np = np.array(self.latest_ft_value, dtype=float)

        # Split into force and torque parts
        force = ft_data_np[0:3]
        torque = ft_data_np[3:6]

        # Rotate both vectors using rotation matrix
        rotated_force = self.cobot_force_rot @ force
        rotated_torque = self.cobot_force_rot @ torque

        # Recombine
        self.raw_ft_value=np.hstack((rotated_force, rotated_torque))
        self.get_logger().info(f"x {self.raw_ft_value[0]}, y {self.raw_ft_value[1]}, z {self.raw_ft_value[2]}, xx {self.raw_ft_value[3]}, yy {self.raw_ft_value[4]}, zz {self.raw_ft_value[5]}")

        #subtracting offsets
        rotated_data_offoset = self.raw_ft_value-self.ft_offsets 

        # self.get_logger().info(f"x {rotated_data_ofoset[0]}, y {rotated_data_ofoset[1]}, z {rotated_data_ofoset[2]}, xx {rotated_data_ofoset[3]}, yy {rotated_data_ofoset[4]}, zz {rotated_data_ofoset[5]}")
        # self.get_logger().info(f"fx {self.ft_offsets[0]}, fy {self.ft_offsets[1]}, fz {self.ft_offsets[2]}, fxx {self.ft_offsets[3]}, fyy {self.ft_offsets[4]}, fzz {self.ft_offsets[5]}")
        self.rotated_ft=rotated_data_offoset

    # Gripper
    def handle_gripper(self):
        request = Gripper.Request()
        request.position = 1.0 if not self.gripper_open else 0.0
        request.grasp_force = self.gripper_force
        action = "Opening" if not self.gripper_open else "Closing"
        self.get_logger().info(f"{action} gripper...")

        future = self.gripper_client.call_async(request)

        def callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info(f"Gripper {action} successful: {response.message}")
                else:
                    self.get_logger().warn(f"Gripper {action} failed: {response.message}")
            except Exception as e:
                self.get_logger().error(f"Gripper service call failed: {e}")

        future.add_done_callback(callback)
        self.gripper_open = not self.gripper_open
        self.start_time_b = time.time()

    # Direction Computation
    def get_dir(self, current_time):
        """Compute motion direction from haptic data."""
        self.jog_val=1*scale_j
        self.jogz=1*scale_fz
        self.jogxy=1*scale_fxy
        if self.latest_hap_data is None:
            return

        pos = self.latest_hap_data.platform_position
        touch = self.latest_hap_data.enable_button
        grab = self.latest_hap_data.grab_button

        self.linear_dir.fill(0)
        self.angular_dir.fill(0)

        if touch == 1:
            if grab == 1:
                for i in range(2):
                    if pos[i] > cart[i][0]:
                        self.linear_dir[i] = self.jog_val
                    elif pos[i] < cart[i][1]:
                        self.linear_dir[i] = -self.jog_val 
                if pos[2] < cart[2][0]:
                        self.linear_dir[2] = self.jog_val
                elif pos[2] > cart[2][1]:
                        self.linear_dir[2] = -self.jog_val 

            else:
                if pos[3] > rot[0][0]:
                    self.angular_dir[1] = self.jog_val
                elif pos[3] < rot[0][1]:
                    self.angular_dir[1] = -self.jog_val

                if pos[1] > rot[1][0]:
                    self.angular_dir[0] = -self.jog_val
                elif pos[1] < rot[1][1]:
                    self.angular_dir[0] = self.jog_val

                if pos[5] < rot[2][1]:
                        self.angular_dir[2] = self.jog_val
                elif pos[5] > rot[2][0]:
                    self.angular_dir[2] = -self.jog_val

                self.get_ft_offsets()

        elif current_time - self.start_time_b > self.time_thresh_hold:
            if grab == 0:
                self.handle_gripper()

        force_deadband=(-1.9,  1.9)
        torque_deadband=(-1.5, 1.5)
        if grab ==1 and self.latest_ft_value:
            if self.rotated_ft[0] > force_deadband[1] :
                self.linear_dir[0] = self.jogxy
            elif self.rotated_ft[0] < force_deadband[0]:
                self.linear_dir[0] = -self.jogxy

            if self.rotated_ft[1] > force_deadband[1] :
                self.linear_dir[1] = self.jogxy
            elif self.rotated_ft[1] <  force_deadband[0] :
                self.linear_dir[1] = -self.jogxy

            if self.rotated_ft[2] > force_deadband[1] :
                self.linear_dir[2] = self.jogz
            elif self.rotated_ft[2] < force_deadband[0] :
                self.linear_dir[2] = -self.jogz
            

    # Motion Control
    def move(self, linear, angular, duration=0.1):
        """Send Twist command."""
        msg = Twist()
        start_time = self.get_clock().now()
        while (
            (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration
            and rclpy.ok()
        ):
            msg.linear.x, msg.linear.y, msg.linear.z = linear
            msg.angular.x, msg.angular.y, msg.angular.z = angular
            self.publisher_.publish(msg)
            time.sleep(1.0 / self.update_rate)

    def do_control(self):
        """Perform control action."""
        self.move(self.linear_dir, self.angular_dir, duration=0.1)

    # --------------------------------------------------------------
    # Main Loop
    # --------------------------------------------------------------
    def run_forever(self):
        self.get_logger().info("Starting continuous control loop...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

            # Handle haptic motion
            if self.latest_hap_data:
                current_time = time.time()
                self.get_dir(current_time)
                self.do_control()

            # Handle FT data and publish rotated result
            if self.latest_ft_value is not None:
                ft_data_np = np.array(self.rotated_ft, dtype=float)

                # Split into force and torque parts
                force = ft_data_np[0:3]*force_val
                torque = ft_data_np[3:6]
                force[2] =force[2]
                # Rotate both vectors using rotation matrix
                rotated_force = np.clip(self.hap_force_rot @ force, -MAX_FORCE_LIMIT, MAX_FORCE_LIMIT)
                rotated_torque = np.clip(self.hap_force_rot @ torque, -MAX_TORQUE_LIMIT, MAX_TORQUE_LIMIT)

                # Recombine
                rotated_data_ofoset = np.hstack((rotated_force, rotated_torque))

                # Publish as WriteData
                msg = WriteData()
                msg.force_data = rotated_data_ofoset
                self.hap_force_pub.publish(msg)

            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = DemoRPYJog()
    try:
        node.run_forever()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
