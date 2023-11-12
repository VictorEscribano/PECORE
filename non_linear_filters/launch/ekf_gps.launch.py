from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Rviz config file
    rviz_config_file = PathJoinSubstitution([FindPackageShare("pecore_launch"), "rviz", "practicum2.rviz"]) 
    world_file = PathJoinSubstitution([FindPackageShare('pecore_launch'), 'worlds', 'boxes.world'])  
    launch_perception = LaunchConfiguration('launch_perception', default=True)
    launch_navigation = LaunchConfiguration('launch_navigation', default=True)

    declared_arguments = []

    declared_arguments.append(LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above
        ])
    )

    # Jackal Robot
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('non_linear_filters'), 
                             'launch/include/jackal_sim.launch.py')
            ),
            launch_arguments=[('rviz_config_file', rviz_config_file),
                              ('world_file', world_file),
                              ('launch_perception', launch_perception),
                              ('launch_navigation', launch_navigation)]
        )
    )

    declared_arguments.append(
        Node(
            package='tf_transformations',
            executable='generate_map_frame',
            name='generate_map_frame',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[]
        )
    )

    declared_arguments.append(
        Node(
            package='non_linear_filters',
            executable='odom_ground_truth',
            name='odom_ground_truth',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[]
        )
    )

    return LaunchDescription(declared_arguments)