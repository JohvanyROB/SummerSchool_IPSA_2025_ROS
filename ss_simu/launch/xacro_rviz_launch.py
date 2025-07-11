import os, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    simu_pkg = get_package_share_directory('ss_simu')

    xacro_file = os.path.join(get_package_share_directory('ss_desc'), "xacro", "x500", "x500.xacro")

    robot_conf = {"name": "drone1", "base_color": "Grey"}

    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        namespace = robot_conf["name"],
        parameters = [
            {"robot_description": xacro.process_file(xacro_file, mappings={"robot_name": robot_conf["name"], "base_color": robot_conf["base_color"]}).toxml()},
        ]
    )

    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
        namespace = robot_conf["name"],
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ["-d", os.path.join(simu_pkg, "config", "config_drone.rviz")]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
