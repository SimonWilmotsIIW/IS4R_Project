import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('turtlebot3')
    robot_description_path = os.path.join(package_dir, 'resource', 'simonwilmots_turtlebot3.urdf')

    turtlebot_driver = WebotsController(
        robot_name='SimonWilmots_TurtleBot3',
        parameters=[
            {'robot_description': robot_description_path},
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

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
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
        shutdown_handler
    ])