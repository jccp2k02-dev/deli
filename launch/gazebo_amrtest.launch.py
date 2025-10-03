import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Path ของ Package และไฟล์ ---
    pkg_name = 'robot_deli_urdf_v2'
    xacro_file_name = 'Robot_Deli_URDF_V2.xacro'
    world_file_name = 'teenoi.world'

    pkg_share = get_package_share_directory(pkg_name)
    xacro_path = os.path.join(pkg_share, 'urdf', xacro_file_name)
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # --- สร้าง robot_description จาก xacro ---
    robot_description_config = xacro.process_file(xacro_path).toxml()
    robot_description = {
        'robot_description': robot_description_config,
        'use_sim_time': True
    }

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': '/home/pooh_ubuntu/ros2_ws/src/robot_deli_urdf_v2/worlds/teenoi.world'}.items()
    )

    # --- Robot State Publisher ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[('/robot_description', 'robot_description')]
    )

    # --- Spawn Entity (ใส่หุ่นเข้า Gazebo) ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'amr_witharm'],
        output='screen'
    )

        # Publish TFs from URDF (base_footprint -> base_link, wheels, etc.)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # TEMP: static map->odom for debugging (remove when using AMCL/SLAM)
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )


    static_transform_publisher_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'map', '--rate', '10.0']
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'base_footprint', '--rate', '10.0']
    )
    print('base_link->base_footprint')
    
    
    # You can add the rest of your nodes here.
    
    # --- Launch Description ---
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        robot_state_publisher_node,
        static_transform_publisher_map_node,
        static_transform_publisher_node,
        static_map_to_odom,
        spawn_entity,
    ])