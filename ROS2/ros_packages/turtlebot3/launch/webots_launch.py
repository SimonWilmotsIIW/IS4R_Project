import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('turtlebot3')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'turtlebot_world.wbt'),
        ros2_supervisor=True
    )


    return LaunchDescription([
        webots,
        webots._supervisor,
    ])