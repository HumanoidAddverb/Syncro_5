from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    gui_arg = DeclareLaunchArgument(
        name= 'gui',
        default_value= 'true'
    )

    robot_arg = DeclareLaunchArgument(
        name= 'robot_name',
        default_value= 'heal'
    )

    urdf = PathJoinSubstitution(
        [
            FindPackageShare("addverb_cobot_description"),
            "urdf",
            'syncro.urdf.xacro',
        ]
    )
    
    # robot state publisher for joint state publisher gui
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name = 'robot_state_pubslisher_gui',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf]),
                                                                    value_type=str)}],
        condition= conditions.IfCondition(LaunchConfiguration('gui')),
    )
    
    # joint state publisher gui node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition= conditions.IfCondition(LaunchConfiguration('gui')),
    )

    # default rviz configuration
    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('addverb_cobot_description'), 'rviz', 'default.rviz']),
    )

    #launch rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    return LaunchDescription([
        gui_arg,
        robot_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rvizconfig_arg,
        rviz_node,
    ])

