from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('turbopi_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'turbopi.urdf')

    return LaunchDescription([
        # Start Gazebo Sim Harmonic with an empty world, -r for real time headless physics
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the robot entity from URDF
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', urdf_path, '-name', 'turbopi'],
            output='screen'
        ),

        # Bridge ROS <-> Gazebo Twist for driving
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/turbopi/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        )

    ])
