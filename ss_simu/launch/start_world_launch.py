import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    simu_pkg = get_package_share_directory('ss_simu')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'--render-engine ogre -r -v 4 {os.path.join(simu_pkg, "worlds", "env1.world")}',    #-r start simulation, otherwise it will be paused
        }.items(),
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simu_pkg, 'launch', 'spawn_robot_launch.py')),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(simu_pkg, 'config', 'ros_gz_bridge.yaml'),
        }],
        output='screen'
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ["-d", os.path.join(simu_pkg, "config", "config.rviz")]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot_launch,
        bridge,
        # rviz_node,
    ])