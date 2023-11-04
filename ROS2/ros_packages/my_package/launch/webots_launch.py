import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt'),
        #mode=mode,
        ros2_supervisor=True,
        respawn=True
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
    ])