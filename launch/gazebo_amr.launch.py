# import os
# import launch
# import launch_ros
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription, RegisterEventHandler
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from ament_index_python.packages import get_package_share_directory



# def generate_launch_description():
#     world_path = '/home/tonnam/test_ws/src/rebot_robot/worlds/obstacles.world'  # Change this to your actual world file path
#     # Get package share directory
#     pkg_share = FindPackageShare('rebot_robot').find('rebot_robot')
    
#     # Path to URDF file
#     urdf_path = "/home/tonnam/test_ws/src/rebot_robot/models/AMR_Rebot/urdf/AMR.urdf"#os.path.join(pkg_share, 'urdf', 'RebotURDF.urdf')

#     # Read and sanitize URDF file content
#     with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
#         robot_description = urdf_file.read()

#     # Remove XML declaration if present
#     if robot_description.startswith("<?xml"):
#         robot_description = "\n".join(robot_description.split("\n")[1:])

#     # Joint state publisher node
#     joint_state_publisher_node = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher'
#     )

#     # Robot state publisher node
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description , 'use_sim_time' : True}]
#     )

#     # Spawn entity in Gazebo
#     spawn_entity_node = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-entity', 'rebot_robot', '-topic', 'robot_description'],
#         output='screen'
#     )
    
#     # Static transform publisher
#     static_transform_publisher_node = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40']
#     )


#     # # Launch Gazebo with ROS plugins
#     # gazebo_process = launch.actions.ExecuteProcess(
#     #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
#     #     output='screen'
#     # )
    
#     gazebo_process = launch.actions.ExecuteProcess(
#         cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
#         output='screen'
#     )

#     # gazebo_params_file = os.path.join(get_package_share_directory('rebot_robot'), 'config', 'gazebo_params.yaml')
#     # gazebo = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource(
#     #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
#     #     ), launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
#     # )
    
    
#     return launch.LaunchDescription([
#         gazebo_process,
#         joint_state_publisher_node,
#         robot_state_publisher_node,
#         static_transform_publisher_node,
#         spawn_entity_node,
#     ])



import os
import launch
import launch_ros
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define paths
    pkg_share = FindPackageShare('robotrobot_deli_urdf_v2').find('robot_deli_urdf_v2')
    # world_path = os.path.join(pkg_share, 'worlds', 'obstacles.world')  # Ensure correct path
    # 
    world_path = '/home/pooh_ubuntu/ros2_ws/src/robot_deli_urdf_v2/worlds/teenoi.world'
    urdf_path = '/home/pooh_ubuntu/ros2_ws/src/amr_simulate/models/urdf/MiniAMR.urdf'

    # Read and sanitize URDFS
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read().strip()

    # Remove XML declaration if present
    if robot_description.startswith("<?xml"):
        robot_description = "\n".join(robot_description.split("\n")[1:])

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Gazebo launch file (official way)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )
    
        # Static transform publisher
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'base_footprint', '--rate', '10.0']
    )
    print('base_link->base_footprint')
    
    static_transform_publisher_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'map', '--rate', '10.0']
    )


    print("map->base_link")

    # Delay the spawn entity to ensure Gazebo is ready
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'amr_simulate', '-topic', 'robot_description'],
        output='screen'
    )

    return launch.LaunchDescription([
        # LogInfo(condition=launch.conditions.IfCondition(true), msg="Launching Gazebo with world..."),
        gazebo_launch,  # Start Gazebo
        joint_state_publisher_node,
        robot_state_publisher_node,
        static_transform_publisher_node,
        static_transform_publisher_map_node,
        spawn_entity_node,  # Spawn robot in Gazebo
    ])

