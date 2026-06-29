
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import launch_ros.descriptions

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("addverb_cobot_description"),
                    "urdf",
                    "syncro_ros2_control.urdf.xacro",
                ]
            ),
        ]
    ) 

    robot_description = {"robot_description": launch_ros.descriptions.ParameterValue(robot_description_content,
                         value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("addverb_cobot_control"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
        remappings=[
                ("~/robot_description", "robot_description"),], 
        on_exit= Shutdown()
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller","--param-file", robot_controllers, "--inactive"],)
    
    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller","--param-file", robot_controllers, "--inactive"],)
    
    gravity_comp_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gravity_comp_effort_controller","--param-file", robot_controllers, "--inactive"],)
    
    free_drive_controller_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["free_drive_controller", "--param-file", robot_controllers, "--inactive" ],
    )

    recorder_controller_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["recorder_controller", "--param-file", robot_controllers, "--inactive"],
    )

    ptp_joint_controller_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["ptp_joint_controller", "--param-file", robot_controllers, "--inactive"],
    )

    ptp_tcp_controller_spawner = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["ptp_tcp_controller", "--param-file", robot_controllers , "--inactive"],
    )

    joint_jogging_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_jogging_controller", "--param-file", robot_controllers, "--inactive" ],
    )

    cartesian_jogging_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_jogging_controller", "--param-file", robot_controllers , "--inactive"],
    )

    joint_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_impedance_controller", "--param-file", robot_controllers  , "--inactive" ],
    )

    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_impedance_controller", "--param-file", robot_controllers ,"--inactive"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--param-file", robot_controllers],
    )

    gripper_bridge = Node(
        package="addverb_cobot_controllers",
        executable="gripper_action_bridge",
        output="screen"
    )

    nodes = [
    control_node,
    robot_state_pub_node,
    joint_state_broadcaster_spawner,
    velocity_controller_spawner,
    ptp_tcp_controller_spawner,
    ptp_joint_controller_spawner,
    recorder_controller_spawner,
    joint_jogging_controller_spawner,
    cartesian_jogging_controller_spawner,
    joint_impedance_controller_spawner,
    cartesian_impedance_controller_spawner,
    effort_controller_spawner,
    free_drive_controller_spawner,
    gripper_controller_spawner,
    gravity_comp_effort_controller_spawner,
    gripper_bridge,
]
    return LaunchDescription(nodes)



