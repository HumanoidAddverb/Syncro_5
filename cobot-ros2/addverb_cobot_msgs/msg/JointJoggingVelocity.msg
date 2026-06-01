# JointJoggingVelocity.msg
#
# This message is used to control the jogging (motion by prefixed velocity) of robot joints.
#
# The array 'joint_velocity_scaling_factor' should have a length equal to the number of joints of the robot.
# Each value in the array represents the velocity scaling factor for the corresponding joint.
# - By default, all values should be set to 0 (no movement).
# - To jog a specific joint, set its corresponding value to a positive or negative number.
#   For example, to move only joint 1 at normal speed: [1, 0, 0, 0, 0, 0]
#   To move joint 1 faster: [3, 0, 0, 0, 0, 0]
#   To move joint 2 in the opposite direction: [0, -1, 0, 0, 0, 0]
# The magnitude of each value acts as a multiplying factor for the joint's jogging speed.
# Only the joints with non-zero values will move.

float64[] jvel_scaling_factor