import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('turtlebot3')

    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_description_path = os.path.join(package_dir, 'resource', 'simonwilmots_turtlebot3.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    rviz2_config = os.path.join(package_dir, 'rviz2', 'rviz2.rviz')

    turtlebot_driver = WebotsController(
        robot_name='SimonWilmots_TurtleBot3',
        parameters=[
            {'robot_description': robot_description_path,
             'set_robot_state_publisher': True},
            ros2_control_params,
        ]
    )
    
    rviz2 = Node(
        package='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        executable='rviz2',
        #remappings=[('/tf', 'tf'), ('tf_static', 'tf_static'),],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # map_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    # )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

     # ------ Navigation --------
    navigation_nodes = []
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    nav2_map = os.path.join(package_dir, 'resource', 'nav2_map.yaml')
    nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')
    if 'turtlebot3_navigation2' in get_packages_with_prefixes():
        print("Using nav: ", use_nav)
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

    # ------ SLAM --------
    if 'turtlebot3_cartographer' in get_packages_with_prefixes():
        print("Using slam: ", use_slam)
        turtlebot_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_slam))
        navigation_nodes.append(turtlebot_slam)

    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=navigation_nodes #+ ros_control_spawners
    )

    shutdown_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=turtlebot_driver,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription([
        turtlebot_driver,
        waiting_nodes,
        rviz2,
        footprint_publisher,
        #map_publisher,
        robot_state_publisher,
        shutdown_handler
    ])