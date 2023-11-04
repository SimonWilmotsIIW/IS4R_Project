import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    #robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    #use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    obstacle_avoider = Node(
        package='my_package',
        executable='obstacle_avoider',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # handler that shuts all nodes down when robot_driver exits
    shutdown_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            # target_action=webots,
            target_action=my_robot_driver,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription([
        # webots,
        # webots._supervisor,
        my_robot_driver,
        obstacle_avoider,
        rviz,
        shutdown_handler
    ])