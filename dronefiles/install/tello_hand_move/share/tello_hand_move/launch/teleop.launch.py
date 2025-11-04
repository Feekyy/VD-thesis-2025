from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_hand_move',
            executable='teleop_wasd_altitude',
            name='teleop_wasd_altitude',
            output='screen',
            parameters=[{
                'model_name': 'tello',
                'reference_frame': 'world',
                'cmd_vel_topic': '/cmd_vel',
                'lin_speed': 0.8,
                'yaw_speed': 1.2,
                'dz_step': 0.1,
            }]
        )
    ])
