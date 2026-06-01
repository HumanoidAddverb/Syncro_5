import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import launch_ros.descriptions

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

def generate_merged_yaml(prefixes):
    """Helper function to read the template and generate a single merged YAML file."""
    pkg_share = get_package_share_directory("addverb_cobot_control")
    yaml_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    with open(yaml_path, 'r') as f:
        yaml_template = f.read()

    combined_config = {"controller_manager": {"ros__parameters": {}}}

    for prefix in prefixes:
        arm_yaml_str = yaml_template.replace("$(var prefix)", prefix)
        arm_dict = yaml.safe_load(arm_yaml_str)

        if "controller_manager" in arm_dict and "ros__parameters" in arm_dict["controller_manager"]:
            combined_config["controller_manager"]["ros__parameters"].update(
                arm_dict["controller_manager"]["ros__parameters"]
            )

        for key, value in arm_dict.items():
            if key != "controller_manager":
                combined_config[key] = value

    output_file_path = "/tmp/generated_ros2_controllers.yaml"
    with open(output_file_path, 'w') as f:
        yaml.dump(combined_config, f, default_flow_style=False)
        
    return output_file_path

def launch_setup(context: LaunchContext, *args, **kwargs):
    # 1. Parse prefixes
    prefix_str = LaunchConfiguration("prefix").perform(context)
    if not prefix_str.strip():
        prefixes = [""]
    else:
        prefixes = [p.strip() for p in prefix_str.split(',') if p.strip()]

    # 2. Generate the YAML using the single helper function
    generated_yaml_path = generate_merged_yaml(prefixes)

    # 3. Setup Robot Description 
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("addverb_cobot_description"),
                    "urdf/dual_arm_syncro",
                    "dual_arm.xacro",
                ]
            ),
        ]
    ) 
    robot_description = {"robot_description": launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)}

    # List to hold all launch nodes
    nodes_to_start = []

    # 4. Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ParameterFile(generated_yaml_path, allow_substs=True)],
        output="screen",
        remappings=[("~/robot_description", "robot_description")], 
        on_exit=Shutdown()
    )
    nodes_to_start.append(control_node)

    # 5. Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    nodes_to_start.append(robot_state_pub_node)

    # 6. Joint State Broadcaster (Only one needed globally)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    nodes_to_start.append(joint_state_broadcaster_spawner)

    # 7. Loop to generate spawners for EVERY prefix
    inactive_controllers = [
        "velocity_controller", "effort_controller", "gravity_comp_effort_controller",
        "free_drive_controller", "recorder_controller", "ptp_joint_controller",
        "ptp_tcp_controller", "joint_jogging_controller", "cartesian_jogging_controller",
        "joint_impedance_controller", "cartesian_impedance_controller"
    ]
    
    active_controllers = [
        "gripper_controller"
    ]

    for prefix in prefixes:
        # Spawn inactive controllers
        for controller_name in inactive_controllers:
            full_name = f"{prefix}{controller_name}"
            spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=[full_name, "--param-file", generated_yaml_path, "--inactive"],
            )
            nodes_to_start.append(spawner)
            
        # Spawn active controllers
        for controller_name in active_controllers:
            full_name = f"{prefix}{controller_name}"
            spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=[full_name, "--param-file", generated_yaml_path],
            )
            nodes_to_start.append(spawner)

        # Spawn the gripper action server to be used with moveit
        gripper_node = Node(
        package='addverb_cobot_controllers',       
        executable='gripper_action_bridge', 
        name=f'{prefix}gripper_action_server', 
        output='screen',
        parameters=[
            {'arm_prefix': prefix}        # This passes the parameter to C++
        ]
        )
        nodes_to_start.append(gripper_node)

    return nodes_to_start


def generate_launch_description():
    declare_prefix = DeclareLaunchArgument(
        "prefix",
        default_value="arm_1_,arm_2_"
    )

    return LaunchDescription([
        declare_prefix,
        OpaqueFunction(function=launch_setup)
    ])