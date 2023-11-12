from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import (OnExecutionComplete)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path, PurePath

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Environment variables
    description_set_lidar3D = SetEnvironmentVariable(name='JACKAL_LASER_3D', value='1')
    description_set_lidar_tower = SetEnvironmentVariable(name='JACKAL_LASER_3D_TOWER', value='1') 
    description_set_flea3 = SetEnvironmentVariable(name='JACKAL_FLEA3', value='1')

    gazebo_set_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(get_package_share_directory('jackal_description')).
                                                    parent.resolve())+':',
                                                str(Path(get_package_share_directory('velodyne_description')).
                                                    parent.resolve())+':',
                                                str(Path(get_package_share_directory('pointgrey_camera_description')).
                                                    parent.resolve())+':',
                                                str(PurePath(Path(get_package_share_directory('pecore_launch'),'models')))
                                                    ])

    # Launch args
    is_sim = LaunchConfiguration('is_sim', default=True)
    world_file = LaunchConfiguration('world_file', default=PathJoinSubstitution([FindPackageShare('pecore_launch'), 'worlds', 'boxes.world']))
    use_gui = LaunchConfiguration('use_gui', default=True)
    rviz_config_file= LaunchConfiguration('rviz_config_file', default=PathJoinSubstitution([FindPackageShare("pecore_launch"), "rviz", "tutorial3.rviz"]))
    launch_rviz = LaunchConfiguration('launch_rviz', default=True)
    launch_perception = LaunchConfiguration('launch_perception', default=False)
    launch_navigation = LaunchConfiguration('launch_navigation', default=False)
    
    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'control.yaml']
    )

    # Get URDF via xacro
    robot_description_command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            ),
            ' ',
            'is_sim:=true',
            ' ',
            'gazebo_controllers:=',
            config_jackal_velocity_controller,
        ]

    jackal_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('jackal_description'),
                     'launch',
                     'description.launch.py']
                )
            ),
            launch_arguments=[('robot_description_command', robot_description_command)]
        )

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--verbose',
             world_file],
        output='screen',
    )

    # Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(use_gui)
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_jackal',
        arguments=['-entity',
                   'jackal',
                   '-topic',
                   'robot_description'],
        output='screen',
    )

    # Launch Localization
    jackal_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('non_linear_filters'), 'launch/include', 'ukf_localization.launch.py']
            ))
        )
    
    # Launch Control
    jackal_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pecore_launch'), 'launch/include', 'jackal_control.launch.py']
            )),
            launch_arguments=[('robot_description_command', robot_description_command),
                              ('is_sim', is_sim)]
        )

    # Launch Perception
    jackal_perception = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pecore_launch'), 'launch/include', 'jackal_perception.launch.py']
            )),
            condition=IfCondition(launch_perception)
        )

    # Launch Navigation
    jackal_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('pecore_launch'), 'launch/include', 'jackal_navigation.launch.py']
            )),
            launch_arguments=[('robot_description_command', robot_description_command),
                              ('is_sim', is_sim)],
            condition=IfCondition(launch_navigation)
        )

    # Launch RVIZ
    rviz_node = Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="log",
                    arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "rviz2:=warn"],
                    condition=IfCondition(launch_rviz)
                )
    
    rviz_launch_event = RegisterEventHandler(
        OnExecutionComplete(
            target_action=gazebo_spawn_robot,
            on_completion=[
                LogInfo(msg='Spawn finished'),
                rviz_node,
                ]
            )
        )

    # Arguments
    ld =  LaunchDescription()

    ld.add_action(rviz_launch_event)

    ld.add_action(gazebo_set_resource_path)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    ld.add_action(description_set_lidar3D)
    ld.add_action(description_set_lidar_tower)
    ld.add_action(description_set_flea3)

    ld.add_action(jackal_description)

    ld.add_action(gazebo_spawn_robot)

    ld.add_action(jackal_localization)
    ld.add_action(jackal_control)
    ld.add_action(jackal_perception)
    ld.add_action(jackal_navigation)

    return ld
