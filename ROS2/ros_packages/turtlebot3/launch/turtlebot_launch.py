# #!/usr/bin/python3

# Author: Simon Wilmots
# Intelligent Systems for Robotics

import os
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
    package_dir = get_package_share_directory('turtlebot3')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'simonwilmots_turtlebot3.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    turtlebot_driver = WebotsController(
        robot_name='SimonWilmots_TurtleBot3',
        parameters=[
            {'robot_description': robot_description_path,
             #'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # --- Nav2 navigation ----
    navigation_nodes = []
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    nav2_map = os.path.join(package_dir, 'resource', 'simons_new_map.yaml')
    nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')
    
    turtlebot_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')),
        launch_arguments=[
            ('map', nav2_map),
            ('params_file', nav2_params),
            ('use_sim_time', use_sim_time),
        ],
        condition=launch.conditions.IfCondition(use_nav))
    navigation_nodes.append(turtlebot_navigation)

    # --- Cartographer SLAM ----
    turtlebot_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
        ],
        condition=launch.conditions.IfCondition(use_slam))
    navigation_nodes.append(turtlebot_slam)

    # Wait for webots before launching turtebot driver
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=navigation_nodes + ros_control_spawners
    )

    # Custom shutdown handler to shutdown all nodes if webots exits
    shutdown_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[
                launch.actions.EmitEvent(event=launch.events.Shutdown())
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot_world.wbt',
            description='Custom turtlebot playgound by Simon Wilmots :)'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Set webots startup mode to realtime'
        ),

        # webots + webots/ros supervisor
        webots,
        webots._supervisor,

        # publishers of the turtlebots' states
        robot_state_publisher,
        footprint_publisher,

        # All other nodes dependent on Webots nodes
        turtlebot_driver,
        waiting_nodes,

        shutdown_handler
    ])