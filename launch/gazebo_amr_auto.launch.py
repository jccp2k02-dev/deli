import os
import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare('robot_deli_urdf_v2').find('robot_deli_urdf_v2')
    world_path = '/home/jccp/delirobot/src/robot_deli_urdf_v2/worlds/teenoi.world'
    urdf_path  = "/home/jccp/delirobot/src/robot_deli_urdf_v2/urdf/Robot_Deli_URDF_V2.urdf"

    # Read URDF
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read().strip()
    if robot_description.startswith("<?xml"):
        robot_description = "\n".join(robot_description.split("\n")[1:])

    # Joint states (for fixed joints it still helps robot_state_publisher)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
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

    # Gazebo (official launcher)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world':'/home/jccp/delirobot/src/robot_deli_urdf_v2/worlds/teenoi.world'}.items()
    )

    # Spawn the robot from the /robot_description topic
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot_deli_urdf_v2', '-topic', 'robot_description'],
        output='screen'
    )

    return launch.LaunchDescription([
        gazebo_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        static_map_to_odom,     # <— now included
        spawn_entity_node,
    ])
