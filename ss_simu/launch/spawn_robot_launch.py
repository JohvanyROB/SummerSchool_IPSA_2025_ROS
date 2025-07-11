import os, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('ss_desc'), "xacro", "x500", "x500.xacro")

    robot_conf = {"name": "drone1", "base_color": "Grey", "x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0}

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{
            "world": "my_env",
            "topic": f"/{robot_conf['name']}/robot_description",
            'name': robot_conf['name'],
            'x': robot_conf['x'],
            'y': robot_conf['y'],
            'z': robot_conf['z'],
            'yaw': robot_conf['yaw']}],
        output='screen',
    )

    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        namespace = robot_conf["name"],
        parameters = [
            {"robot_description": xacro.process_file(xacro_file, mappings={"robot_name": robot_conf["name"], "base_color": robot_conf["base_color"]}).toxml()},
        ]
    )

    return LaunchDescription([
        spawn,
        robot_state_publisher
    ])
