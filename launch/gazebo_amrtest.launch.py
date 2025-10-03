import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'robot_deli_urdf_v2'
    xacro_file_name = 'Robot_Deli_URDF_V2.xacro'
    world_file_name = 'teenoi.world'

    pkg_share = get_package_share_directory(pkg_name)
    xacro_path = os.path.join(pkg_share, 'urdf', xacro_file_name)
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # --- สร้าง robot_description จาก xacro ---
    robot_description_config = xacro.process_file(xacro_path).toxml()
    robot_description = {'robot_description': robot_description_config, 'use_sim_time': True}

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # --- Spawn Entity ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'amr_witharm'],
        output='screen'
    )

    # --- Static TFs (new-style arguments) ---
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x','0','--y','0','--z','0',
            '--qx','0','--qy','0','--qz','0','--qw','1',
            '--frame-id','map','--child-frame-id','odom'
        ]
    )

    static_base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x','0','--y','0','--z','0',
            '--qx','0','--qy','0','--qz','0','--qw','1',
            '--frame-id','base_link','--child-frame-id','base_footprint'
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        static_map_to_odom,
        static_base_to_footprint,
        spawn_entity,
    ])