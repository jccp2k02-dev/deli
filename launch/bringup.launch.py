import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanumbot_bringup')
    config_path = os.path.join(pkg_share, 'config', 'mecanum_drive_controller.yaml')

    # Controller Manager (ros2_control node)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config_path],
        output="screen"
    )

    # Spawner สำหรับ joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner สำหรับ mecanum_drive_controller
    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
    ])