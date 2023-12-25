import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('turtlebot3')
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

    # nav2_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
    #     launch_arguments=[
    #         ('map', nav2_map),
    #         ('params_file', nav2_params),
    #     ],
    # )
    # todo https://github.com/turtlebot/turtlebot4_desktop/tree/humble
        
    #description_launch = PathJoinSubstitution(
    #    [package_dir, 'launch', 'robot_description.launch.py']
    #)
    
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

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    shutdown_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=turtlebot_driver,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription([
        turtlebot_driver,
        # nav2_node
        rviz2,
        footprint_publisher,
        robot_state_publisher,
        shutdown_handler
    ])