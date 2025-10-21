from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- FONTOS: Plugin path hozzáadása ---
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH',
                               value='/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu'),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH',
                               value='/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu'),

        # --- Gazebo szerver indítása ---
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '/root/tello_ros_ws/sim/worlds/cube_cam.world'],
            output='screen'
        ),

        # --- Gazebo kliens (GUI) opcionálisan ---
        # ExecuteProcess(
        #     cmd=['gzclient'],
        #     output='screen'
        # ),

        # --- Például a Tello vezérlés node ---
        Node(
            package='tello_ros',
            executable='teleop_wasd_altitude',
            name='teleop_tello',
            output='screen'
        )
    ])
