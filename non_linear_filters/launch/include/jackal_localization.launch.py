from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Configs
    efk_config = os.path.join(
        get_package_share_directory('non_linear_filters'),
        'config',
        'ekf_config.yaml'
    )

    # Odometry estimation
    odometry_group_action = GroupAction([
        Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_gps',
        output='screen',
        parameters=[efk_config]
    )
    ])

    # Global Localization (NavSat)
    global_navsat = GroupAction([
        # GNSS conversion
        Node(
            package='pecore_launch', 
            executable='navsat_odom', 
            name='navsat_odom',
            output='screen'
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(odometry_group_action)
    ld.add_action(global_navsat)
    return ld
