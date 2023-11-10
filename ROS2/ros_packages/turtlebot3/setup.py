from setuptools import setup

package_name = 'turtlebot3'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/turtlebot_launch.py', 'launch/webots_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/turtlebot_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/simonwilmots_turtlebot3.urdf', 'resource/ros2control.yaml']))
data_files.append(('share/' + package_name + '/rviz2', ['rviz2/rviz2.rviz']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simon',
    maintainer_email='simon.wilmots@student.uhasselt.be',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_driver = turtlebot3.turtlebot_driver:main',
            #'obstacle_avoider = my_package.obstacle_avoider:main'
],
    },
)