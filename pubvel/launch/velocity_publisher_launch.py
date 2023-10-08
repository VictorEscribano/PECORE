from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubvel',
            executable='pid_pubvel',
            name='velocity_publisher',
            output='screen',
            parameters=[
                {'Kp': 18.0},
                {'Ki': 0.1},
                {'Kd': 2.0},
                {'min_vel': -3.0},                
                {'max_vel': 0.5},  # Aquí faltaba una coma
                {'desired_position': [4.0, 1.0, 0.0]}
            ]
        ),
    ])

