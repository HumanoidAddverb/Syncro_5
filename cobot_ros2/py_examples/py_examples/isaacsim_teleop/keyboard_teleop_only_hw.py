import time
import sys
import tty
import termios
import threading
import signal
import atexit
import rclpy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from addverb_cobot_msgs.srv import Gripper
import math
from std_msgs.msg import Float32

START_TIME = time.monotonic()
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Velocity magnitude for direct control
VELOCITY_MAGNITUDE = 0.05

STDIN_FD = sys.stdin.fileno()
ORIGINAL_TERMIOS = termios.tcgetattr(STDIN_FD)


def restore_terminal():
    try:
        termios.tcsetattr(STDIN_FD, termios.TCSADRAIN, ORIGINAL_TERMIOS)
    except Exception:
        pass


atexit.register(restore_terminal)


def sigint_handler(signum, frame):
    restore_terminal()
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, sigint_handler)


# Shared state
last_pressed_key = None
key_lock = threading.Lock()

shared_joint_velocities = [0.0] * len(JOINT_NAMES)
velocity_lock = threading.Lock()

current_robot_jpos = {}
current_robot_jvel = {}
robot_state_lock = threading.Lock()


def getch():
    tty.setcbreak(STDIN_FD)
    ch = sys.stdin.read(1)
    return ch


def joint_state_callback(msg):
    local_pos = {}
    local_vel = {}

    for i, name in enumerate(msg.name):
        if name not in JOINT_NAMES:
            continue
        if i < len(msg.position):
            local_pos[name] = float(msg.position[i])
        if i < len(msg.velocity):
            local_vel[name] = float(msg.velocity[i])

    with robot_state_lock:
        global current_robot_jpos, current_robot_jvel
        current_robot_jpos = local_pos
        current_robot_jvel = local_vel


def keyboard_thread():
    global last_pressed_key
    while True:
        ch = getch()
        with key_lock:
            last_pressed_key = ch
        if ch == "\x03":
            break


from std_msgs.msg import Float32


def call_gripper_service(
    node,
    client,
    gripper_pub,
    position,
    force=100.0,
):

    req = Gripper.Request()

    req.position = float(position)
    req.grasp_force = float(force)

    future = client.call_async(req)

    def done_callback(fut):

        try:

            result = fut.result()

            if result is None:
                print("[Gripper] Service returned no result")
                return

            if result.success:

                print(f"[Gripper] SUCCESS | " f"position={position}, " f"force={force}")

                msg = Float32()

                # reversing because we want to log actions and general idea is closed = 1, open = 0
                if position == 1.0:
                    msg.data = 0.0
                elif position == 0.0:
                    msg.data = 1.0

                gripper_pub.publish(msg)
            else:
                print(f"[Gripper] FAILED | " f"message={result.message}")

        except Exception as e:

            print(f"[Gripper] Service call exception: {e}")

    future.add_done_callback(done_callback)


def publisher_timer_callback(publisher):
    with velocity_lock:
        velocities = shared_joint_velocities[:]

    msg = Float64MultiArray()
    msg.data = velocities
    publisher.publish(msg)


def teleop_loop(node, gripper_client, gripper_pub):

    global last_pressed_key
    global shared_joint_velocities

    print("Teleop ready. Use keyboard to command velocities.")

    # Desired velocities from keyboard
    target_velocities = [0.0] * len(JOINT_NAMES)

    # Actual published velocities (ramped)
    current_velocities = [0.0] * len(JOINT_NAMES)

    # Ramp parameters
    MAX_VEL = VELOCITY_MAGNITUDE
    MAX_ACCEL = 0.4  # rad/s^2

    CONTROL_DT = 0.02  # 50 Hz

    while True:

        # ---------------------------------------------------
        # Read latest key
        # ---------------------------------------------------
        ch = None

        with key_lock:
            if last_pressed_key:
                ch = last_pressed_key
                last_pressed_key = None

        if ch == "\x03":
            break

        # ---------------------------------------------------
        # Default target = zero velocity
        # ---------------------------------------------------
        target_velocities = [0.0] * len(JOINT_NAMES)

        # ---------------------------------------------------
        # Keyboard mapping
        # ---------------------------------------------------
        if ch:

            # Positive direction
            if ch in "qwerty":
                idx = "qwerty".index(ch)

                if idx < len(JOINT_NAMES):
                    target_velocities[idx] = MAX_VEL

            # Negative direction
            elif ch in "asdfgh":
                idx = "asdfgh".index(ch)

                if idx < len(JOINT_NAMES):
                    target_velocities[idx] = -MAX_VEL

            # Gripper open
            elif ch == "u":
                call_gripper_service(node, gripper_client, gripper_pub, position=0.0)

            # Gripper close
            elif ch == "j":
                call_gripper_service(node, gripper_client, gripper_pub, position=1.0)

        # ---------------------------------------------------
        # Ramp current velocity toward target velocity
        # ---------------------------------------------------
        max_step = MAX_ACCEL * CONTROL_DT

        for i in range(len(JOINT_NAMES)):

            error = target_velocities[i] - current_velocities[i]

            if abs(error) <= max_step:
                current_velocities[i] = target_velocities[i]

            else:
                current_velocities[i] += max_step * math.copysign(1.0, error)

        # ---------------------------------------------------
        # Publish ramped velocities
        # ---------------------------------------------------
        with velocity_lock:
            shared_joint_velocities = current_velocities[:]

        time.sleep(CONTROL_DT)


def main():
    rclpy.init()

    node = rclpy.create_node("key_teleop_velocity")

    publisher = node.create_publisher(
        Float64MultiArray, "/velocity_controller/commands", 10
    )

    gripper_pub = node.create_publisher(
        Float32,
        "/gripper_action_command",
        10,
    )

    node.create_subscription(JointState, "/joint_states", joint_state_callback, 10)

    gripper_client = node.create_client(Gripper, "/gripper_controller/command")

    while not gripper_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for gripper service...")

    node.create_timer(1.0 / 50.0, lambda: publisher_timer_callback(publisher))

    threading.Thread(target=keyboard_thread, daemon=True).start()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print("--- KEYBOARD CONTROL ACTIVE ---")
    print("q w e r t y → +0.01 rad/s")
    print("a s d f g h → -0.01 rad/s")
    print("u → open gripper | j → close gripper")
    print("Ctrl+C to exit")
    print("-------------------------------")

    try:
        teleop_loop(node, gripper_client, gripper_pub)
    except KeyboardInterrupt:
        pass
    finally:
        restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
